//! USB full-speed device implementation

use core::cell::RefCell;
use critical_section::{self, Mutex};
use embedded_hal::blocking::delay::DelayUs;
use usb_device::{
    bus::{PollResult, UsbBus as UsbBusTrait},
    endpoint::{EndpointAddress, EndpointType},
    Result, UsbDirection, UsbError,
};

use crate::{delay::McycleDelay, rcu::Clocks};

use super::PortSpeed;

/// Maximum number of IN/OUT endpoints.
pub const MAX_ENDPOINTS: usize = 4;

const WORD_LEN: usize = 4;

const USBFS_RAM_BASE_PTR: *const u8 = gd32vf103_pac::USBFS_GLOBAL::PTR as *const u8;
const USBFS_RAM_BASE_MUT_PTR: *mut u8 = gd32vf103_pac::USBFS_GLOBAL::PTR as *mut u8;

// Offset of the USBFS FIFO in SRAM
const USB_FIFO_OFFSET: usize = 0x1000;
// RX FIFO depth (in bytes, spec reset value 0x200 is in 32-bit words)
const RX_FIFO_LEN: usize = 0x80;

// TX FIFO depth (in bytes, spec reset value 0x200 is in 32-bit words)
// FIXME: should these be configurable to allow for more use-cases?
const TX0_FIFO_OFFSET: usize = USB_FIFO_OFFSET + RX_FIFO_LEN;
const TX0_FIFO_LEN: usize = 0x80;

const TX1_FIFO_OFFSET: usize = TX0_FIFO_OFFSET + TX0_FIFO_LEN;
const TX1_FIFO_LEN: usize = 0x40;

const TX2_FIFO_OFFSET: usize = TX1_FIFO_OFFSET + TX1_FIFO_LEN;
const TX2_FIFO_LEN: usize = 0x0;

const TX3_FIFO_OFFSET: usize = TX2_FIFO_OFFSET + TX2_FIFO_LEN;
const TX3_FIFO_LEN: usize = 0x0;

const TX_FIFO_OFFSETS: [usize; MAX_ENDPOINTS] = [
    TX0_FIFO_OFFSET,
    TX1_FIFO_OFFSET,
    TX2_FIFO_OFFSET,
    TX3_FIFO_OFFSET,
];

const TX_FIFO_LENGTHS: [usize; MAX_ENDPOINTS] =
    [TX0_FIFO_LEN, TX1_FIFO_LEN, TX2_FIFO_LEN, TX3_FIFO_LEN];

macro_rules! configure_endpoint {
    ($epctl:ident, $eplen:ident, $epintf:ident) => {
        $epctl.modify(|r, w| {
            if r.epen().bit_is_set() {
                w
                    // disable the endpoint
                    .epd()
                    .set_bit()
                    // set NAK
                    .snak()
                    .set_bit()
            } else {
                // clear all bitfields
                unsafe { w.bits(0) }
            }
        });

        // set IN endpoint transfer length to 0
        $eplen.write(|w| unsafe { w.bits(0) });

        // clear all pending IN endpoint interrupts
        $epintf.write(|w| unsafe { w.bits(0xff) });
    };
}

// Special-case OUT endpoint 0, its EPD register is fixed to 0
macro_rules! configure_out0_endpoint {
    ($epctl:ident, $eplen:ident, $epintf:ident) => {
        $epctl.modify(|r, w| {
            if r.epen().bit_is_set() {
                w.snak().set_bit()
            } else {
                // clear all bitfields
                unsafe { w.bits(0) }
            }
        });

        // set IN endpoint transfer length to 0
        $eplen.write(|w| unsafe { w.bits(0) });

        // clear all pending IN endpoint interrupts
        $epintf.write(|w| unsafe { w.bits(0xff) });
    };
}

macro_rules! setup_control_in_endpoint_xfer {
    ($eplen:ident, $epctl:ident, $epfeinten:ident, $index:tt, $len:tt) => {
        $eplen.modify(|_, w| w.pcnt().variant(1).tlen().variant($len as u8));

        if $len > 0 {
            // enable the TX FIFO empty interrupt for this endpoint
            $epfeinten.modify(|_, w| w.ieptxfeie().variant($index as u8));
        }

        $epctl.modify(|_, w| w.cnak().set_bit().epen().set_bit());
    };
}

macro_rules! setup_in_endpoint_xfer {
    ($eplen:ident, $epctl:ident, $dstat:ident, $epfeinten:ident, $ep:tt, $index:tt, $len:tt) => {
        $eplen.modify(|_, w| w.pcnt().variant(1).tlen().variant($len as u32));

        if $ep.ep_type() == EndpointType::Isochronous {
            // set the multi-packet count per frame to one
            $eplen.modify(|_, w| w.mcpf().variant(1));

            let fnrsof = $dstat.read().fnrsof();

            if ((fnrsof.bits() >> 8) & 0x1) != 0 {
                $epctl.modify(|_, w| w.sd1pid_soddfrm().set_bit());
            } else {
                $epctl.modify(|_, w| w.sd0pid_sevenfrm().set_bit());
            }
        } else {
            if $len > 0 {
                // enable the TX FIFO empty interrupt for this endpoint
                $epfeinten.modify(|_, w| w.ieptxfeie().variant($index as u8));
            }
        }

        $epctl.modify(|_, w| w.cnak().set_bit().epen().set_bit());
    };
}

macro_rules! setup_control_out_endpoint_xfer {
    ($eplen:ident, $epctl:ident, $len:tt) => {
        $eplen.modify(|_, w| w.pcnt().set_bit().tlen().variant($len as u8));

        $epctl.modify(|_, w| w.cnak().set_bit().epen().set_bit());
    };
}

macro_rules! setup_out_endpoint_xfer {
    ($eplen:ident, $epctl:ident, $dstat:ident, $ep:tt, $len:tt) => {
        $eplen.modify(|_, w| w.pcnt().variant(1).tlen().variant($len as u32));

        if $ep.ep_type() == EndpointType::Isochronous {
            let fnrsof = $dstat.read().fnrsof();

            if ((fnrsof.bits() >> 8) & 0x1) != 0 {
                $epctl.modify(|_, w| w.sd1pid_soddfrm().set_bit());
            } else {
                $epctl.modify(|_, w| w.sd0pid_sevenfrm().set_bit());
            }
        }

        $epctl.modify(|_, w| w.cnak().set_bit().epen().set_bit());
    };
}

macro_rules! set_in_stalled {
    ($epctl:ident, $stall:tt) => {
        $epctl.modify(|r, w| {
            if r.epen().bit_is_set() {
                w.epd().set_bit();
            }

            if $stall {
                w.stall().set_bit()
            } else {
                w.stall().clear_bit()
            }
        });
    };
}

macro_rules! set_out_stalled {
    ($epctl:ident, $stall:tt) => {
        $epctl.modify(|_, w| {
            if $stall {
                w.stall().set_bit()
            } else {
                w.stall().clear_bit()
            }
        });
    };
}

/// Sets the End of Periodic Frame (EOPF) to trigger at a threshold percentage of frame time.
#[repr(u8)]
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum EOPFSetting {
    /// 00: 80% of frame time
    Pct80 = 0b00,
    /// 01: 85% of frame time
    Pct85 = 0b01,
    /// 10: 90% of frame time
    Pct90 = 0b10,
    /// 11: 95% of frame time
    Pct95 = 0b11,
}

/// Selector for which TX FIFO is flushed in the GRSTCTL register.
#[repr(u8)]
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum TXFlushNum {
    Zero = 0b0_0000,
    One = 0b0_0001,
    Two = 0b0_0010,
    Three = 0b0_0011,
    All = 0b1_0000,
}

/// Maximum packet length settings.
#[repr(u8)]
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum MPL {
    SixtyFour = 0b00,
    ThirtyTwo = 0b01,
    Sixteen = 0b10,
    Eight = 0b11,
}

/// Global register received packet status settings.
#[repr(u8)]
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum ReceivedPacketStatus {
    GlobalOutNak = 0b0001,
    OutPacketReceived = 0b0010,
    OutTransferCompleted = 0b0011,
    SetupTransactionCompleted = 0b0100,
    SetupPacketReceived = 0b0110,
    Reserved = 0xff,
}

/// Represents an USB endpoint.
#[repr(C)]
#[derive(Debug)]
pub struct Endpoint {
    address: EndpointAddress,
    ep_type: EndpointType,
    max_packet_size: u16,
    interval: u8,
}

impl Endpoint {
    /// Creates a new [Endpoint].
    pub const fn new(
        address: EndpointAddress,
        ep_type: EndpointType,
        max_packet_size: u16,
        interval: u8,
    ) -> Self {
        Self {
            address,
            ep_type,
            max_packet_size,
            interval,
        }
    }

    unsafe fn as_raw_parts(&self) -> (*const u8, usize) {
        if self.address.is_out() {
            // All OUT endpoints share the same RX FIFO space.
            (
                USBFS_RAM_BASE_PTR.offset(USB_FIFO_OFFSET as isize),
                RX_FIFO_LEN,
            )
        } else {
            // In the reference SDK from Gigadevice:
            //
            // TX0 and TX1 FIFO have a set size
            // TX2 and TX3 FIFOs are zero-length
            let index = core::cmp::min(self.address.index(), MAX_ENDPOINTS - 1);
            (
                USBFS_RAM_BASE_PTR.offset(TX_FIFO_OFFSETS[index] as isize),
                TX_FIFO_LENGTHS[index],
            )
        }
    }

    unsafe fn as_raw_word_parts(&self) -> (*const u32, usize) {
        if self.address.is_out() {
            // All OUT endpoints share the same RX FIFO space.
            (
                USBFS_RAM_BASE_PTR.offset(USB_FIFO_OFFSET as isize) as *const u32,
                RX_FIFO_LEN / 4,
            )
        } else {
            // In the reference SDK from Gigadevice:
            //
            // TX0 and TX1 FIFO have a set size
            // TX2 and TX3 FIFOs are zero-length
            let index = core::cmp::min(self.address.index(), MAX_ENDPOINTS - 1);
            (
                USBFS_RAM_BASE_PTR.offset(TX_FIFO_OFFSETS[index] as isize) as *const u32,
                TX_FIFO_LENGTHS[index] / 4,
            )
        }
    }

    unsafe fn as_raw_parts_mut(&self) -> (*mut u8, usize) {
        if self.address.is_out() {
            // All OUT endpoints share the same RX FIFO space.
            (
                USBFS_RAM_BASE_MUT_PTR.offset(USB_FIFO_OFFSET as isize),
                RX_FIFO_LEN,
            )
        } else {
            // In the reference SDK from Gigadevice:
            //
            // TX0 and TX1 FIFO have a set size
            // TX2 and TX3 FIFOs are zero-length
            let index = core::cmp::min(self.address.index(), MAX_ENDPOINTS - 1);
            (
                USBFS_RAM_BASE_MUT_PTR.offset(TX_FIFO_OFFSETS[index] as isize),
                TX_FIFO_LENGTHS[index],
            )
        }
    }

    unsafe fn as_raw_word_parts_mut(&self) -> (*mut u32, usize) {
        if self.address.is_out() {
            // All OUT endpoints share the same RX FIFO space.
            (
                USBFS_RAM_BASE_MUT_PTR.offset(USB_FIFO_OFFSET as isize) as *mut u32,
                RX_FIFO_LEN / 4,
            )
        } else {
            // In the reference SDK from Gigadevice:
            //
            // TX0 and TX1 FIFO have a set size
            // TX2 and TX3 FIFOs are zero-length
            let index = core::cmp::min(self.address.index(), MAX_ENDPOINTS - 1);
            (
                USBFS_RAM_BASE_MUT_PTR.offset(TX_FIFO_OFFSETS[index] as isize) as *mut u32,
                TX_FIFO_LENGTHS[index] / 4,
            )
        }
    }

    /// Gets the RX/TX FIFO as byte buffer.
    pub fn buf(&self) -> &'static [u8] {
        unsafe {
            let (base, len) = self.as_raw_parts();
            core::slice::from_raw_parts(base, len)
        }
    }

    /// Gets the RX/TX FIFO as word buffer.
    pub fn word_buf(&self) -> &'static [u32] {
        unsafe {
            let (base, len) = self.as_raw_word_parts();
            core::slice::from_raw_parts(base, len)
        }
    }

    /// Gets the RX/TX FIFO as a mutable byte buffer.
    pub fn buf_mut(&self) -> &'static mut [u8] {
        unsafe {
            let (base, len) = self.as_raw_parts_mut();
            core::slice::from_raw_parts_mut(base, len)
        }
    }

    /// Gets the RX/TX FIFO as a mutable word buffer.
    pub fn word_buf_mut(&self) -> &'static mut [u32] {
        unsafe {
            let (base, len) = self.as_raw_word_parts_mut();
            core::slice::from_raw_parts_mut(base, len)
        }
    }

    /// Gets the [EndpointAddress].
    pub fn address(&self) -> EndpointAddress {
        self.address
    }

    /// Gets the [EndpointType].
    pub fn ep_type(&self) -> EndpointType {
        self.ep_type
    }

    /// Gets the max packet size.
    pub fn max_packet_size(&self) -> u16 {
        self.max_packet_size
    }

    /// Gets the interval.
    pub fn interval(&self) -> u8 {
        self.interval
    }
}

/// Inner implementation for the [`UsbBus`] struct.
///
/// Separated from the main implementation for interior mutability.
pub struct UsbBusInner {
    in_endpoints: [Option<Endpoint>; MAX_ENDPOINTS],
    out_endpoints: [Option<Endpoint>; MAX_ENDPOINTS],
    usbfs_global: gd32vf103_pac::USBFS_GLOBAL,
    usbfs_device: gd32vf103_pac::USBFS_DEVICE,
    usbfs_pwrclk: gd32vf103_pac::USBFS_PWRCLK,
    delay: McycleDelay,
}

impl UsbBusInner {
    /// Creates a new [UsbBusInner].
    pub fn new(
        usbfs_global: gd32vf103_pac::USBFS_GLOBAL,
        usbfs_device: gd32vf103_pac::USBFS_DEVICE,
        usbfs_pwrclk: gd32vf103_pac::USBFS_PWRCLK,
        clocks: &Clocks,
    ) -> Self {
        Self {
            in_endpoints: [None, None, None, None],
            out_endpoints: [None, None, None, None],
            usbfs_global,
            usbfs_device,
            usbfs_pwrclk,
            delay: McycleDelay::new(&clocks),
        }
    }

    /// Allocates an endpoint and specified endpoint parameters. This method is called by the device
    /// and class implementations to allocate endpoints, and can only be called before
    /// [`enable`](UsbBus::enable) is called.
    ///
    /// # Arguments
    ///
    /// * `ep_dir` - The endpoint direction.
    /// * `ep_addr` - A static endpoint address to allocate. If Some, the implementation should
    ///   attempt to return an endpoint with the specified address. If None, the implementation
    ///   should return the next available one.
    /// * `max_packet_size` - Maximum packet size in bytes.
    /// * `interval` - Polling interval parameter for interrupt endpoints.
    ///
    /// # Errors
    ///
    /// * [`EndpointOverflow`](usb_device::UsbError::EndpointOverflow) - Available total number of
    ///   endpoints, endpoints of the specified type, or endpoind packet memory has been exhausted.
    ///   This is generally caused when a user tries to add too many classes to a composite device.
    /// * [`InvalidEndpoint`](usb_device::UsbError::InvalidEndpoint) - A specific `ep_addr` was specified
    ///   but the endpoint in question has already been allocated.
    pub fn alloc_ep(
        &mut self,
        ep_dir: UsbDirection,
        ep_addr: Option<EndpointAddress>,
        ep_type: EndpointType,
        max_packet_size: u16,
        interval: u8,
    ) -> Result<EndpointAddress> {
        if let Some(addr) = ep_addr {
            match ep_dir {
                UsbDirection::In => {
                    let index = addr.index() as usize;
                    if self.in_endpoints[index].is_some() {
                        return Err(UsbError::InvalidEndpoint);
                    }
                    self.in_endpoints[index] =
                        Some(Endpoint::new(addr, ep_type, max_packet_size, interval));
                }
                UsbDirection::Out => {
                    let index = addr.index() as usize;
                    if self.out_endpoints[index].is_some() {
                        return Err(UsbError::InvalidEndpoint);
                    }
                    self.out_endpoints[index] =
                        Some(Endpoint::new(addr, ep_type, max_packet_size, interval));
                }
            }

            Ok(addr)
        } else {
            match ep_dir {
                UsbDirection::In => {
                    if let Some(index) = self.in_endpoints.iter().position(|e| e.is_none()) {
                        let addr = EndpointAddress::from_parts(index, ep_dir);
                        self.in_endpoints[index] =
                            Some(Endpoint::new(addr, ep_type, max_packet_size, interval));
                        Ok(addr)
                    } else {
                        Err(UsbError::EndpointOverflow)
                    }
                }
                UsbDirection::Out => {
                    if let Some(index) = self.out_endpoints.iter().position(|e| e.is_none()) {
                        let addr = EndpointAddress::from_parts(index, ep_dir);
                        self.out_endpoints[index] =
                            Some(Endpoint::new(addr, ep_type, max_packet_size, interval));
                        Ok(addr)
                    } else {
                        Err(UsbError::EndpointOverflow)
                    }
                }
            }
        }
    }

    /// Enables and initializes the USBFS device.
    ///
    /// Initialization sequence taken from the [GD32VF103_Firmware_Library](https://github.com/GigaDevice-Semiconductor/GD32VF103_Firmware_Library).
    pub fn enable(&mut self) {
        // force to peripheral mode
        self.usbfs_global
            .gusbcs
            .modify(|_, w| w.fdm().clear_bit().fhm().clear_bit());

        // force device mode
        self.usbfs_global.gusbcs.modify(|_, w| w.fdm().set_bit());

        // restart the PHY clock (maybe don't need to)
        self.usbfs_pwrclk
            .pwrclkctl
            .write(|w| w.suclk().clear_bit().shclk().clear_bit());

        // set device speed
        self.usbfs_device.dcfg.modify(|_, w| {
            w
                // Set EOPF to 80% of frame time
                .eopft()
                .variant(EOPFSetting::Pct80 as u8)
                // Set device speed to Full Speed
                .ds()
                .variant(PortSpeed::FullSpeedDevice as u8)
        });

        self.setup_fifos();

        self.fifo_flush();

        // clear all pending device interrupts
        self.usbfs_device.diepinten.write(|w| {
            w.tfen()
                .clear_bit()
                .epdisen()
                .clear_bit()
                .citoen()
                .clear_bit()
                .eptxfuden()
                .clear_bit()
                .iepneen()
                .clear_bit()
        });

        self.usbfs_device.doepinten.write(|w| {
            w.btbstpen()
                .clear_bit()
                .epdisen()
                .clear_bit()
                .eprxfovren()
                .clear_bit()
                .stpfen()
                .clear_bit()
                .tfen()
                .clear_bit()
        });

        self.configure_endpoints();

        self.enable_interrupts();
    }

    /// Performs setup for the RX/TX FIFOs.
    pub fn setup_fifos(&self) {
        // set the RX FIFO length
        self.usbfs_global
            .grflen
            .modify(|_, w| w.rxfd().variant(RX_FIFO_LEN as u16));

        // set the start address as offset from the base
        let mut ram_addr = RX_FIFO_LEN as u16;

        // set the TX addresses and lengths
        for (i, txlen) in TX_FIFO_LENGTHS.iter().map(|&t| t as u16).enumerate() {
            if i == 0 {
                self.usbfs_global
                    .diep0tflen()
                    .write(|w| w.iep0txfd().variant(txlen).iep0txrsar().variant(ram_addr));
            } else if i == 1 {
                self.usbfs_global
                    .diep1tflen
                    .write(|w| w.ieptxfd().variant(txlen).ieptxrsar().variant(ram_addr));
            } else if i == 2 {
                self.usbfs_global
                    .diep2tflen
                    .modify(|_, w| w.ieptxfd().variant(txlen).ieptxrsar().variant(ram_addr));
            } else {
                self.usbfs_global
                    .diep3tflen
                    .modify(|_, w| w.ieptxfd().variant(txlen).ieptxrsar().variant(ram_addr));
            }

            ram_addr += txlen;
        }
    }

    /// Configures the IN/OUT endpoints.
    pub fn configure_endpoints(&self) {
        let usb_device = &self.usbfs_device;

        let (diep0ctl, diep0len, diep0intf) = (
            &usb_device.diep0ctl,
            &usb_device.diep0len,
            &usb_device.diep0intf,
        );

        let (diep1ctl, diep1len, diep1intf) = (
            &usb_device.diep1ctl,
            &usb_device.diep1len,
            &usb_device.diep1intf,
        );

        let (diep2ctl, diep2len, diep2intf) = (
            &usb_device.diep2ctl,
            &usb_device.diep2len,
            &usb_device.diep2intf,
        );

        let (diep3ctl, diep3len, diep3intf) = (
            &usb_device.diep3ctl,
            &usb_device.diep3len,
            &usb_device.diep3intf,
        );

        configure_endpoint!(diep0ctl, diep0len, diep0intf);
        configure_endpoint!(diep1ctl, diep1len, diep1intf);
        configure_endpoint!(diep2ctl, diep2len, diep2intf);
        configure_endpoint!(diep3ctl, diep3len, diep3intf);

        let (doep0ctl, doep0len, doep0intf) = (
            &usb_device.doep0ctl,
            &usb_device.doep0len,
            &usb_device.doep0intf,
        );

        let (doep1ctl, doep1len, doep1intf) = (
            &usb_device.doep1ctl,
            &usb_device.doep1len,
            &usb_device.doep1intf,
        );

        let (doep2ctl, doep2len, doep2intf) = (
            &usb_device.doep2ctl,
            &usb_device.doep2len,
            &usb_device.doep2intf,
        );

        let (doep3ctl, doep3len, doep3intf) = (
            &usb_device.doep3ctl,
            &usb_device.doep3len,
            &usb_device.doep3intf,
        );

        configure_out0_endpoint!(doep0ctl, doep0len, doep0intf);
        configure_endpoint!(doep1ctl, doep1len, doep1intf);
        configure_endpoint!(doep2ctl, doep2len, doep2intf);
        configure_endpoint!(doep3ctl, doep3len, doep3intf);
    }

    /// Enables the USB device mode interrupts.
    pub fn enable_interrupts(&self) {
        // clear any pending USB OTG interrupts
        self.usbfs_global
            .gotgintf
            .write(|w| unsafe { w.bits(0xffff_ffff) });

        // clear any pending interrupts
        self.usbfs_global
            .gintf
            .write(|w| unsafe { w.bits(0xbfff_ffff) });

        // enable the USB wakeup and suspend interrupts
        self.usbfs_global
            .ginten
            .write(|w| w.wkupie().set_bit().spie().set_bit());

        // enable device_mode-related interrupts
        self.usbfs_global.ginten.modify(|_, w| {
            w.rxfneie()
                .set_bit()
                .rstie()
                .set_bit()
                .enumfie()
                .set_bit()
                .iepie()
                .set_bit()
                .oepie()
                .set_bit()
                .sofie()
                .set_bit()
                .mfie()
                .set_bit()
        });

        if cfg!(feature = "vbus_sensing") {
            self.usbfs_global
                .ginten
                .modify(|_, w| w.sesie().set_bit().otgie().set_bit());
        }

        self.usbfs_global.gahbcs.modify(|_, w| w.ginten().set_bit());
    }

    /// Flushes all the data FIFOs.
    pub fn fifo_flush(&mut self) {
        self.tx_fifo_flush();
        self.tx_fifo_flush();
    }

    /// Flushes all the TX FIFOs.
    pub fn tx_fifo_flush(&mut self) {
        self.tx_fifo_flush_by_index(TXFlushNum::All)
    }

    /// Flushes the specified TX FIFO.
    pub fn tx_fifo_flush_by_index(&mut self, index: TXFlushNum) {
        self.usbfs_global
            .grstctl
            .modify(|_, w| w.txfnum().variant(index as u8).txff().set_bit());

        // wait for an additional 3 PHY clock cycles
        self.delay.delay_us(3u8);
    }

    /// Flushes the RX FIFO.
    pub fn rx_fifo_flush(&mut self) {
        self.usbfs_global.grstctl.modify(|_, w| w.rxff().set_bit());

        // wait for an additional 3 PHY clock cycles
        self.delay.delay_us(3u8);
    }

    /// Configures the USB device to be disconnected.
    pub fn disconnect(&self) {
        self.usbfs_device.dctl.modify(|_, w| w.sd().set_bit());
    }

    /// Configures the USB device to be connected.
    pub fn connect(&self) {
        self.usbfs_device.dctl.modify(|_, w| w.sd().clear_bit());
    }

    /// Causes the Host to enumerate the device again by disconnecting, flushing the FIFOs,
    /// and reconnecting after a short delay.
    pub fn reset(&mut self) {
        self.fifo_flush();
        self.disconnect();

        self.delay.delay_us(50u8);

        self.connect();
    }

    /// Delay the device for `delay` microseconds.
    pub fn delay_us(&mut self, delay: u8) {
        self.delay.delay_us(delay);
    }

    /// Sets the USB device address.
    pub fn set_device_address(&self, addr: u8) {
        self.usbfs_device.dcfg.modify(|_, w| w.dar().variant(addr));
    }

    /// Write data to an IN endpoint data FIFO.
    ///
    /// # Errors
    ///
    /// * [`InvalidEndpoint`](usb_device::UsbError::InvalidEndpoint) - The `ep_addr` does not point to a
    ///   valid endpoint that was previously allocated with
    ///   [`UsbBus::alloc_ep`](usb_device::bus::UsbBus::alloc_ep).
    /// * [`WouldBlock`](usb_device::UsbError::WouldBlock) - A previously written packet is still pending
    ///   to be sent.
    /// * [`BufferOverflow`](usb_device::UsbError::BufferOverflow) - The packet is too long to fit in the
    ///   transmission buffer. This is generally an error in the class implementation, because the
    ///   class shouldn't provide more data than the `max_packet_size` it specified when allocating
    ///   the endpoint.
    pub fn write(&self, ep_addr: EndpointAddress, buf: &[u8]) -> Result<usize> {
        use core::convert::TryInto;

        let index = ep_addr.index();
        if index >= MAX_ENDPOINTS || ep_addr.is_out() || self.in_endpoints[index].is_none() {
            return Err(UsbError::InvalidEndpoint);
        }

        let ep = self.in_endpoints[index].as_ref().unwrap();

        let len = buf.len();

        if len > ep.max_packet_size() as usize {
            return Err(UsbError::BufferOverflow);
        }

        self.setup_in_xfer(ep, len)?;

        // Transfer the data in words (4-bytes).
        let fifo = ep.word_buf_mut();
        for (i, word) in buf.chunks_exact(WORD_LEN).enumerate() {
            // FIXME: does machine endianess matter here?
            fifo[i] = u32::from_le_bytes(word.try_into().unwrap());
        }

        Ok(len)
    }

    fn setup_in_xfer(&self, ep: &Endpoint, len: usize) -> Result<()> {
        let ep_addr = ep.address();
        let index = ep_addr.index();

        match index {
            0 => {
                let eplen = &self.usbfs_device.diep0len;
                let epctl = &self.usbfs_device.diep0ctl;
                let epfeinten = &self.usbfs_device.diepfeinten;

                setup_control_in_endpoint_xfer!(eplen, epctl, epfeinten, len, index);

                Ok(())
            }
            1 => {
                let eplen = &self.usbfs_device.diep1len;
                let epctl = &self.usbfs_device.diep1ctl;
                let epfeinten = &self.usbfs_device.diepfeinten;
                let dstat = &self.usbfs_device.dstat;

                setup_in_endpoint_xfer!(eplen, epctl, dstat, epfeinten, ep, len, index);

                Ok(())
            }
            2 => {
                let eplen = &self.usbfs_device.diep2len;
                let epctl = &self.usbfs_device.diep2ctl;
                let epfeinten = &self.usbfs_device.diepfeinten;
                let dstat = &self.usbfs_device.dstat;

                setup_in_endpoint_xfer!(eplen, epctl, dstat, epfeinten, ep, len, index);

                Ok(())
            }
            3 => {
                let eplen = &self.usbfs_device.diep3len;
                let epctl = &self.usbfs_device.diep3ctl;
                let epfeinten = &self.usbfs_device.diepfeinten;
                let dstat = &self.usbfs_device.dstat;

                setup_in_endpoint_xfer!(eplen, epctl, dstat, epfeinten, ep, len, index);

                Ok(())
            }
            _ => Err(UsbError::EndpointOverflow),
        }
    }

    /// Reads a single packet of data from the RX data FIFO.
    ///
    /// All endpoints share the same RX data FIFO.
    ///
    /// # Errors
    ///
    /// * [`InvalidEndpoint`](usb_device::UsbError::InvalidEndpoint) - The `ep_addr` does not point to a
    ///   valid endpoint that was previously allocated with
    ///   [`UsbBus::alloc_ep`](usb_device::bus::UsbBus::alloc_ep).
    /// * [`WouldBlock`](usb_device::UsbError::WouldBlock) - There is no packet to be read. Note that
    ///   this is different from a received zero-length packet, which is valid in USB. A zero-length
    ///   packet will return `Ok(0)`.
    /// * [`BufferOverflow`](usb_device::UsbError::BufferOverflow) - The received packet is too long to
    ///   fit in `buf`. This is generally an error in the class implementation, because the class
    ///   should use a buffer that is large enough for the `max_packet_size` it specified when
    ///   allocating the endpoint.
    pub fn read(&self, ep_addr: EndpointAddress, buf: &mut [u8]) -> Result<usize> {
        let len = buf.len();
        let index = ep_addr.index();

        if ep_addr.index() >= MAX_ENDPOINTS
            || ep_addr.is_in()
            || self.out_endpoints[index].is_none()
        {
            return Err(UsbError::InvalidEndpoint);
        }

        let ep = self.out_endpoints[index].as_ref().unwrap();

        if len < ep.max_packet_size() as usize {
            return Err(UsbError::BufferOverflow);
        }

        self.setup_out_xfer(ep, len)?;

        let mut read = ep.max_packet_size() as usize;

        // Transfer data in words (4-bytes).
        let fifo = ep.word_buf();
        for (fifo_word, buf_word) in fifo.iter().zip(buf.chunks_exact_mut(WORD_LEN)) {
            // FIXME: does machine endianess matter here?
            buf_word.copy_from_slice(fifo_word.to_le_bytes().as_ref());
            read = read.saturating_sub(WORD_LEN);

            if read == 0 {
                break;
            }
        }

        Ok(len)
    }

    fn setup_out_xfer(&self, ep: &Endpoint, len: usize) -> Result<()> {
        match ep.address().index() {
            0 => {
                let eplen = &self.usbfs_device.doep0len;
                let epctl = &self.usbfs_device.doep0ctl;

                setup_control_out_endpoint_xfer!(eplen, epctl, len);

                Ok(())
            }
            1 => {
                let eplen = &self.usbfs_device.doep1len;
                let epctl = &self.usbfs_device.doep1ctl;
                let dstat = &self.usbfs_device.dstat;

                setup_out_endpoint_xfer!(eplen, epctl, dstat, ep, len);

                Ok(())
            }
            2 => {
                let eplen = &self.usbfs_device.doep2len;
                let epctl = &self.usbfs_device.doep2ctl;
                let dstat = &self.usbfs_device.dstat;

                setup_out_endpoint_xfer!(eplen, epctl, dstat, ep, len);

                Ok(())
            }
            3 => {
                let eplen = &self.usbfs_device.doep3len;
                let epctl = &self.usbfs_device.doep3ctl;
                let dstat = &self.usbfs_device.dstat;

                setup_out_endpoint_xfer!(eplen, epctl, dstat, ep, len);

                Ok(())
            }
            _ => Err(UsbError::EndpointOverflow),
        }
    }

    /// Sets or clears the STALL condition for an endpoint.
    pub fn set_stalled(&self, ep_addr: EndpointAddress, stalled: bool) {
        let index = ep_addr.index();

        if ep_addr.is_in() {
            match index {
                0 => {
                    let epctl = &self.usbfs_device.diep0ctl;
                    set_in_stalled!(epctl, stalled);
                }
                1 => {
                    let epctl = &self.usbfs_device.diep1ctl;
                    set_in_stalled!(epctl, stalled);
                }
                2 => {
                    let epctl = &self.usbfs_device.diep2ctl;
                    set_in_stalled!(epctl, stalled);
                }
                3 => {
                    let epctl = &self.usbfs_device.diep3ctl;
                    set_in_stalled!(epctl, stalled);
                }
                _ => (),
            }
        } else {
            match index {
                0 => {
                    let epctl = &self.usbfs_device.doep0ctl;
                    set_out_stalled!(epctl, stalled);
                }
                1 => {
                    let epctl = &self.usbfs_device.doep1ctl;
                    set_out_stalled!(epctl, stalled);
                }
                2 => {
                    let epctl = &self.usbfs_device.doep2ctl;
                    set_out_stalled!(epctl, stalled);
                }
                3 => {
                    let epctl = &self.usbfs_device.doep3ctl;
                    set_out_stalled!(epctl, stalled);
                }
                _ => (),
            }
        }
    }

    /// Gets whether the STALL condition is set for an endpoint.
    pub fn is_stalled(&self, ep_addr: EndpointAddress) -> bool {
        let is_in = ep_addr.is_in();

        match ep_addr.index() {
            0 => {
                if is_in {
                    self.usbfs_device.diep0ctl.read().stall().bit_is_set()
                } else {
                    self.usbfs_device.doep0ctl.read().stall().bit_is_set()
                }
            }
            1 => {
                if is_in {
                    self.usbfs_device.diep1ctl.read().stall().bit_is_set()
                } else {
                    self.usbfs_device.doep1ctl.read().stall().bit_is_set()
                }
            }
            2 => {
                if is_in {
                    self.usbfs_device.diep2ctl.read().stall().bit_is_set()
                } else {
                    self.usbfs_device.doep2ctl.read().stall().bit_is_set()
                }
            }
            3 => {
                if is_in {
                    self.usbfs_device.diep3ctl.read().stall().bit_is_set()
                } else {
                    self.usbfs_device.doep3ctl.read().stall().bit_is_set()
                }
            }
            _ => false,
        }
    }

    /// Causes the USB peripheral to enter USB suspend mode, lowering power consumption and
    /// preparing to detect a USB wakeup event.
    pub fn suspend(&self) {
        // switch off USB clocks
        self.usbfs_pwrclk
            .pwrclkctl
            .write(|w| w.suclk().set_bit().shclk().set_bit());

        // enter DEEP_SLEEP mode with LDO in low power mode
        // FIXME: implement after PMU is implemented
    }

    /// Resumes from suspend mode.
    pub fn resume(&self) {
        self.usbfs_pwrclk
            .pwrclkctl
            .write(|w| w.suclk().set_bit().shclk().set_bit());

        self.usbfs_device.dctl.modify(|_, w| w.rwkup().set_bit());
    }

    /// Gets information about events and incoming data.
    pub fn poll(&self) -> PollResult {
        let gintf = self.usbfs_global.gintf.read();
        let grstatp = self.usbfs_global.grstatp_device().read();
        let rpckst = grstatp.rpckst().bits();

        if rpckst == ReceivedPacketStatus::OutPacketReceived as u8 {
            PollResult::Data {
                ep_out: grstatp.epnum().bits() as u16,
                ep_in_complete: 0,
                ep_setup: 0,
            }
        } else if rpckst == ReceivedPacketStatus::SetupPacketReceived as u8 {
            PollResult::Data {
                ep_out: 0,
                ep_in_complete: 0,
                ep_setup: grstatp.epnum().bits() as u16,
            }
        } else if gintf.wkupif().bit_is_set() {
            PollResult::Resume
        } else if gintf.rst().bit_is_set() {
            PollResult::Reset
        } else if gintf.sp().bit_is_set() {
            PollResult::Suspend
        } else {
            PollResult::None
        }
    }
}

/// [UsbBus](usb_device::bus::UsbBus) implementation for [gd32vf103xx](crate) devices.
pub struct UsbBus {
    inner: Mutex<RefCell<UsbBusInner>>,
}

impl UsbBus {
    /// Creates a new [UsbBus].
    pub fn new(
        usbfs_global: gd32vf103_pac::USBFS_GLOBAL,
        usbfs_device: gd32vf103_pac::USBFS_DEVICE,
        usbfs_pwrclk: gd32vf103_pac::USBFS_PWRCLK,
        clocks: &Clocks,
    ) -> Self {
        Self {
            inner: Mutex::new(RefCell::new(UsbBusInner::new(
                usbfs_global,
                usbfs_device,
                usbfs_pwrclk,
                clocks,
            ))),
        }
    }

    /// Delay the device for `delay` microseconds.
    pub fn delay_us(&self, delay: u8) {
        critical_section::with(|cs| {
            self.inner.borrow(cs).borrow_mut().delay_us(delay);
        });
    }
}

impl UsbBusTrait for UsbBus {
    fn alloc_ep(
        &mut self,
        ep_dir: UsbDirection,
        ep_addr: Option<EndpointAddress>,
        ep_type: EndpointType,
        max_packet_size: u16,
        interval: u8,
    ) -> Result<EndpointAddress> {
        critical_section::with(|cs| {
            let mut inner = self.inner.borrow(cs).borrow_mut();

            inner.alloc_ep(ep_dir, ep_addr, ep_type, max_packet_size, interval)
        })
    }

    fn enable(&mut self) {
        critical_section::with(|cs| {
            let mut inner = self.inner.borrow(cs).borrow_mut();

            inner.enable();
        })
    }

    fn reset(&self) {
        critical_section::with(|cs| {
            self.inner.borrow(cs).borrow_mut().reset();
        })
    }

    fn set_device_address(&self, addr: u8) {
        critical_section::with(|cs| {
            self.inner.borrow(cs).borrow().set_device_address(addr);
        })
    }

    fn write(&self, ep_addr: EndpointAddress, buf: &[u8]) -> Result<usize> {
        critical_section::with(|cs| self.inner.borrow(cs).borrow().write(ep_addr, buf))
    }

    fn read(&self, ep_addr: EndpointAddress, buf: &mut [u8]) -> Result<usize> {
        critical_section::with(|cs| self.inner.borrow(cs).borrow().read(ep_addr, buf))
    }

    fn set_stalled(&self, ep_addr: EndpointAddress, stalled: bool) {
        critical_section::with(|cs| {
            self.inner.borrow(cs).borrow().set_stalled(ep_addr, stalled);
        })
    }

    fn is_stalled(&self, ep_addr: EndpointAddress) -> bool {
        critical_section::with(|cs| self.inner.borrow(cs).borrow().is_stalled(ep_addr))
    }

    fn suspend(&self) {
        critical_section::with(|cs| {
            self.inner.borrow(cs).borrow().suspend();
        })
    }

    fn resume(&self) {
        critical_section::with(|cs| {
            self.inner.borrow(cs).borrow().resume();
        })
    }

    fn poll(&self) -> PollResult {
        critical_section::with(|cs| self.inner.borrow(cs).borrow().poll())
    }
}
