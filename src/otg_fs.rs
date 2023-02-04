//! USB OTG full-speed peripheral
//!
//! Requires the usb_fs feature.

use crate::{
    gpio::{
        gpioa::{PA11, PA12},
        Floating, Input,
    },
    pac,
    time::Hertz,
};
pub use synopsys_usb_otg::UsbBus;
use synopsys_usb_otg::UsbPeripheral;

pub type UsbBusType = UsbBus<USB>;

#[allow(dead_code)]
pub struct USB {
    pub usb_global: pac::USBFS_GLOBAL,
    pub usb_device: pac::USBFS_DEVICE,
    pub usb_pwrclk: pac::USBFS_PWRCLK,
    pub pin_dm: PA11<Input<Floating>>,
    pub pin_dp: PA12<Input<Floating>>,
    pub hclk: Hertz,
}

unsafe impl Sync for USB {}

unsafe impl UsbPeripheral for USB {
    const REGISTERS: *const () = pac::USBFS_GLOBAL::ptr() as *const ();

    const HIGH_SPEED: bool = false;
    const FIFO_DEPTH_WORDS: usize = 320;
    const ENDPOINT_COUNT: usize = 4;

    fn enable() {
        let rcu = unsafe { &*pac::RCU::ptr() };

        riscv::interrupt::free(|| {
            // Enable USB peripheral
            rcu.ahben.modify(|_, w| w.usbfsen().set_bit());

            // Reset USB peripheral
            rcu.ahbrst.modify(|_, w| w.usbfsrst().set_bit());
            rcu.ahbrst.modify(|_, w| w.usbfsrst().clear_bit());
        });
    }

    fn ahb_frequency_hz(&self) -> u32 {
        self.hclk.0
    }
}
