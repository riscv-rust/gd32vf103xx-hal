//! # CAN
//!
//! ## Example usage:
//!  ```rust
//!    let dp = pac::Peripherals::take().unwrap();
//!
//!    let mut rcu = dp.RCU.configure()
//!        .ext_hf_clock(8.mhz())
//!        .sysclk(108.mhz())
//!        .freeze();
//!
//!    let mut afio = dp.AFIO.constrain(&mut rcu);
//!
//!    let gpioa = dp.GPIOA.split(&mut rcu);
//!    let gpiob = dp.GPIOB.split(&mut rcu);
//!
//!    let pa12 = gpioa.pa12.into_alternate_push_pull();
//!    let pa11 = gpioa.pa11.into_floating_input();
//!
//!    let pb13 = gpiob.pb13.into_alternate_push_pull();
//!    let pb12 = gpiob.pb12.into_floating_input();
//!
//!    let mut config = can::Config::default();
//!    config.loopback_communication = true;
//!
//!    let mut can0 = Can::<CAN0, can::NoRemap>::new(dp.CAN0, (pa12, pa11), config, &mut afio, &mut rcu);
//!    can0.set_filter(0, Filter {
//!        mode: FilterMode::Mask,
//!        entry: FilterEntry::Entry32([0, 0]),
//!        fifo_index: FIFO::FIFO0,
//!    });
//!
//!    let mut config = can::Config::default();
//!    config.loopback_communication = true;
//!
//!    let mut can1 = Can::<CAN1, can::NoRemap>::new(dp.CAN1, (pb13, pb12), config, &mut afio, &mut rcu);
//!    can0.set_filter(15, Filter {
//!        mode: FilterMode::Mask,
//!        entry: FilterEntry::Entry32([0, 0]),
//!        fifo_index: FIFO::FIFO1,
//!    });
//!
//!    let frame = Frame {
//!        id: Id::Extended(ExtendedId::new(114514).unwrap()),
//!        ft: false,
//!        dlen: 8,
//!        data: [11,22,33,44,55,66,77,88],
//!    };
//!
//!    nb::block!(can0.transmit(&frame));
//!
//!    let a = nb::block!(can0.receive()).unwrap();
//!
//!    nb::block!(can1.transmit(&frame));
//!
//!    let a = nb::block!(can1.receive()).unwrap();
//!}
//!  ```

use core::marker::PhantomData;

use nb;

use gd32vf103_pac::{CAN0, CAN1};

use crate::afio::{Afio, Remap};
use crate::gpio::gpioa::*;
use crate::gpio::gpiob::*;
use crate::gpio::gpiod::*;
use crate::gpio::{Alternate, Floating, Input, PushPull};
use crate::rcu::{Enable, Rcu, Reset};

use embedded_can::{ExtendedId, Id, StandardId};

#[derive(Debug)]
pub enum Error {}

// Time quantum
pub struct TQ(u8);

//TODO baudrate
pub struct Config {
    pub prescaler: u16,
    pub auto_bus_off_recovery: bool,
    pub auto_retransmission: bool,
    pub auto_wake_up: bool,
    pub receive_fifo_overwrite: bool,
    pub time_triggered_communication: bool,
    pub trans_fifo_order: bool,
    pub silent_communication: bool,
    pub loopback_communication: bool,
    pub resync_jump_width: TQ,
    pub time_segment_1: TQ,
    pub time_segment_2: TQ,
}

impl Default for Config {
    fn default() -> Config {
        Config {
            // 125KBps
            prescaler: 48,
            auto_bus_off_recovery: false,
            auto_retransmission: true,
            auto_wake_up: false,
            receive_fifo_overwrite: true,
            time_triggered_communication: false,
            trans_fifo_order: false,
            silent_communication: false,
            loopback_communication: false,
            resync_jump_width: TQ(0),
            time_segment_1: TQ(4),
            time_segment_2: TQ(2),
        }
    }
}

#[derive(Debug)]
pub struct Frame {
    pub id: Id,
    pub ft: bool,
    pub dlen: u8,
    pub data: [u8; 8],
}

pub enum MailBox {
    MailBox0,
    MailBox1,
    MailBox2,
}

pub enum FIFO {
    FIFO0,
    FIFO1,
}

pub enum FilterMode {
    List,
    Mask,
}

pub enum FilterEntry {
    Entry32([u32; 2]),
    Entry16([u16; 4]),
}

pub struct Filter {
    pub entry: FilterEntry,
    pub mode: FilterMode,
    pub fifo_index: FIFO,
}

pub trait Pins<CAN, REMAP> {
    fn remap(&self) -> REMAP;
}
macro_rules! can_pin {
    ($can:ty, $($remap:ident: (
        tx:$tx_pin:ident, rx:$rx_pin:ident, $t:ty, $val:expr
    ),)+ ) => {
    $(
        impl Pins<$can, $remap> for ($tx_pin<Alternate<PushPull>>, $rx_pin<Input<Floating>>) {
            fn remap(&self) -> $remap { $remap {} }
        }

        impl From<$remap> for $t {
            fn from(_: $remap) -> Self {
                $val
            }
        }
     )+
    }
}

pub struct NoRemap;
pub struct Remap1;
pub struct Remap2;

can_pin! {
    CAN0,
    NoRemap: (tx: PA12, rx: PA11, u8, 0),
    Remap1: (tx: PB9, rx: PB8, u8, 1),
    Remap2: (tx: PD1, rx: PD0, u8, 2),
}

can_pin! {
    CAN1,
    NoRemap: (tx: PB13, rx: PB12, bool, false),
    Remap1: (tx: PB6, rx: PB5, bool, true),
}

//TODO ECLIC
pub struct Can<CAN, REMAP> {
    can: CAN,
    _remap: PhantomData<REMAP>,
}

macro_rules! can {
    ($CAN:ident) => {
        impl<REMAP: Into<u8> + Into<bool>> Can<$CAN, REMAP>
        {
            pub fn new(
                can: $CAN,
                pins: impl Pins<$CAN, REMAP>,
                config: Config,
                afio: &mut Afio,
                rcu: &mut Rcu
                ) -> Self {
                $CAN::remap(afio, pins.remap().into());

                $CAN::enable(rcu);
                $CAN::reset(rcu);

                // Disable sleep working mode
                can.ctl.modify(|_, w| w.slpwmod().clear_bit());

                // Enable initial working mode
                can.ctl.modify(|_, w| w.iwmod().set_bit());
                loop {
                    if can.stat.read().iws().bit_is_set() {
                        break;
                    }
                }

                can.bt.modify(|_, w| unsafe {
                    w
                        .scmod().bit(config.silent_communication)
                        .lcmod().bit(config.loopback_communication)
                        .sjw().bits(config.resync_jump_width.0)
                        .bs1().bits(config.time_segment_1.0)
                        .bs2().bits(config.time_segment_2.0)
                        .baudpsc().bits(config.prescaler - 1)
                });

                can.ctl.modify(|_, w|
                               w
                               .ttc().bit(config.time_triggered_communication)
                               .abor().bit(config.auto_bus_off_recovery)
                               .awu().bit(config.auto_wake_up)
                               .ard().bit(!config.auto_retransmission)
                               .rfod().bit(!config.receive_fifo_overwrite)
                               .tfo().bit(config.trans_fifo_order)
                );

                // Disable initial working mode
                can.ctl.modify(|_, w| w.iwmod().clear_bit());
                loop {
                    if can.stat.read().iws().bit_is_clear() {
                        break;
                    }
                }

                Can {
                    can: can,
                    _remap: PhantomData::<REMAP>,
                }
            }

            pub fn transmit(self: &mut Self, frame: &Frame) -> nb::Result<(), Error> {
                self.transmit_mailbox(MailBox::MailBox0, &frame)
                    .or(self.transmit_mailbox(MailBox::MailBox1, &frame))
                    .or(self.transmit_mailbox(MailBox::MailBox2, &frame))
            }

            pub fn transmit_mailbox(self: &mut Self, mailbox: MailBox, frame: &Frame) -> nb::Result<(), Error> {
                macro_rules! transmit {
                    ($tme:ident, $tmi:ident, $tmp:ident, $tmdata0:ident, $tmdata1:ident) => {{

                        // Transmit mailbox empty
                        if !self.can.tstat.read().$tme().bits() {
                            return Err(nb::Error::WouldBlock);
                        }

                        // Disable Transmit
                        self.can.$tmi.modify(|_, w| w.ten().clear_bit());

                        match frame.id {
                            Id::Standard(id) => {
                                self.can.$tmi.modify(|_, w| unsafe {
                                    w
                                        .sfid_efid().bits(id.as_raw())
                                        .ff().clear_bit()
                                        .ft().bit(frame.ft)
                                });
                            },
                            Id::Extended(id) => {
                                self.can.$tmi.modify(|_, w| unsafe {
                                    w
                                        .sfid_efid().bits((id.as_raw() >> 18) as u16)
                                        .efid().bits(id.as_raw())
                                        .ff().set_bit()
                                        .ft().bit(frame.ft)
                                });
                            }
                        }

                        self.can.$tmp.modify(|_, w| unsafe { w.dlenc().bits(frame.dlen) });
                        self.can.$tmdata0.write(|w| unsafe {
                            w
                                .db0().bits(frame.data[0])
                                .db1().bits(frame.data[1])
                                .db2().bits(frame.data[2])
                                .db3().bits(frame.data[3])
                        });
                        self.can.$tmdata1.write(|w| unsafe {
                            w
                                .db4().bits(frame.data[4])
                                .db5().bits(frame.data[5])
                                .db6().bits(frame.data[6])
                                .db7().bits(frame.data[7])
                        });

                        // Enable transmit
                        self.can.$tmi.modify(|_, w| w.ten().set_bit());

                        Ok(())
                    }}
                }

                match mailbox {
                    MailBox::MailBox0 => transmit!(tme0, tmi0, tmp0, tmdata00, tmdata10),
                    MailBox::MailBox1 => transmit!(tme1, tmi1, tmp1, tmdata01, tmdata11),
                    MailBox::MailBox2 => transmit!(tme2, tmi2, tmp2, tmdata02, tmdata12),
                }
            }

            pub fn receive(self: &mut Self) -> nb::Result<Frame, Error> {
                self.receive_fifo(FIFO::FIFO0)
                    .or(self.receive_fifo(FIFO::FIFO1))
            }

            pub fn receive_fifo(self: &mut Self, fifo: FIFO) -> nb::Result<Frame, Error> {
                macro_rules! receive {
                    ($rfifo:ident, $rfl:ident, $rfd:ident, $rfifomi:ident, $rfifomp:ident, $rfifomdata0:ident, $rfifomdata1:ident) => {{
                        // Receive FIFO length
                        if self.can.$rfifo.read().$rfl().bits() == 0 {
                            return Err(nb::Error::WouldBlock);
                        }

                        let id =
                            if self.can.$rfifomi.read().ff().bits() {
                                let sfid_efid = self.can.$rfifomi.read().sfid_efid().bits() as u32;
                                let efid = self.can.$rfifomi.read().efid().bits() as u32;

                                Id::Extended(ExtendedId::new(sfid_efid << 18 | efid).unwrap())
                            }
                            else {
                                let sfid = self.can.$rfifomi.read().sfid_efid().bits() as u16;

                                Id::Standard(StandardId::new(sfid).unwrap())
                            };

                        let frame = Frame {
                            id: id,
                            ft: self.can.$rfifomi.read().ft().bits(),
                            dlen: self.can.$rfifomp.read().dlenc().bits(),
                            data: [
                                self.can.$rfifomdata0.read().db0().bits(),
                                self.can.$rfifomdata0.read().db1().bits(),
                                self.can.$rfifomdata0.read().db2().bits(),
                                self.can.$rfifomdata0.read().db3().bits(),
                                self.can.$rfifomdata1.read().db4().bits(),
                                self.can.$rfifomdata1.read().db5().bits(),
                                self.can.$rfifomdata1.read().db6().bits(),
                                self.can.$rfifomdata1.read().db7().bits(),
                            ],
                        };

                        // Receive FIFO dequeue
                        self.can.$rfifo.modify(|_, w| w.$rfd().set_bit());

                        Ok(frame)
                    }}
                }

                match fifo {
                    FIFO::FIFO0 => receive!(rfifo0, rfl0, rfd0, rfifomi0, rfifomp0, tmdata00, tmdata10),
                    FIFO::FIFO1 => receive!(rfifo1, rfl1, rfd1, rfifomi1, rfifomp1, tmdata01, tmdata11),
                }
            }
        }
    }
}

impl<REMAP> Can<CAN0, REMAP> {
    pub fn enable_filter(self: &mut Self, index: u8, enable: bool) {
        let index_bit = 1 << index;

        // Disable filter lock
        self.can.fctl.modify(|_, w| w.fld().set_bit());

        // filter working
        if enable {
            self.can
                .fw
                .modify(|r, w| unsafe { w.bits(r.bits() | index_bit) });
        } else {
            self.can
                .fw
                .modify(|r, w| unsafe { w.bits(r.bits() & !index_bit) });
        }

        // Enable filter lock
        self.can.fctl.modify(|_, w| w.fld().clear_bit());
    }

    pub fn set_filter(self: &mut Self, index: u8, filter: Filter) {
        let index_bit = 1 << index;

        // Disable filter lock
        self.can.fctl.modify(|_, w| w.fld().set_bit());

        // Disable filter working
        self.can
            .fw
            .modify(|r, w| unsafe { w.bits(r.bits() & !index_bit) });

        let (data0, data1) = match filter.entry {
            FilterEntry::Entry32([a, b]) => {
                self.can
                    .fscfg
                    .modify(|r, w| unsafe { w.bits(r.bits() | index_bit) });

                (a, b)
            }
            FilterEntry::Entry16([a, b, c, d]) => {
                self.can
                    .fscfg
                    .modify(|r, w| unsafe { w.bits(r.bits() & !index_bit) });
                let x = ((a as u32) << 16) | b as u32;
                let y = ((c as u32) << 16) | d as u32;

                (x, y)
            }
        };

        macro_rules! write_fndata {
            ($fndata0:ident, $fndata1:ident) => {{
                self.can.$fndata0.write(|w| unsafe { w.bits(data0) });
                self.can.$fndata1.write(|w| unsafe { w.bits(data1) });
            }};
        }

        match index {
            0 => write_fndata!(f0data0, f0data1),
            1 => write_fndata!(f1data0, f1data1),
            2 => write_fndata!(f2data0, f2data1),
            3 => write_fndata!(f3data0, f3data1),
            4 => write_fndata!(f4data0, f4data1),
            5 => write_fndata!(f5data0, f5data1),
            6 => write_fndata!(f6data0, f6data1),
            7 => write_fndata!(f7data0, f7data1),
            8 => write_fndata!(f8data0, f8data1),
            9 => write_fndata!(f9data0, f9data1),
            10 => write_fndata!(f10data0, f10data1),
            11 => write_fndata!(f11data0, f11data1),
            12 => write_fndata!(f12data0, f12data1),
            13 => write_fndata!(f13data0, f13data1),
            14 => write_fndata!(f14data0, f14data1),
            15 => write_fndata!(f15data0, f15data1),
            16 => write_fndata!(f16data0, f16data1),
            17 => write_fndata!(f17data0, f17data1),
            18 => write_fndata!(f18data0, f18data1),
            19 => write_fndata!(f19data0, f19data1),
            20 => write_fndata!(f20data0, f20data1),
            21 => write_fndata!(f21data0, f21data1),
            22 => write_fndata!(f22data0, f22data1),
            23 => write_fndata!(f23data0, f23data1),
            24 => write_fndata!(f24data0, f24data1),
            25 => write_fndata!(f25data0, f25data1),
            26 => write_fndata!(f26data0, f26data1),
            27 => write_fndata!(f27data0, f27data1),
            _ => panic!(),
        };

        match filter.mode {
            FilterMode::Mask => self
                .can
                .fmcfg
                .modify(|r, w| unsafe { w.bits(r.bits() & !index_bit) }),
            FilterMode::List => self
                .can
                .fmcfg
                .modify(|r, w| unsafe { w.bits(r.bits() | index_bit) }),
        }

        match filter.fifo_index {
            FIFO::FIFO0 => self
                .can
                .fafifo
                .modify(|r, w| unsafe { w.bits(r.bits() & !index_bit) }),
            FIFO::FIFO1 => self
                .can
                .fafifo
                .modify(|r, w| unsafe { w.bits(r.bits() | index_bit) }),
        }

        // Enable filter working
        self.can
            .fw
            .modify(|r, w| unsafe { w.bits(r.bits() | index_bit) });

        // Enable filter lock
        self.can.fctl.modify(|_, w| w.fld().clear_bit());
    }
}

can! {CAN0}
can! {CAN1}
