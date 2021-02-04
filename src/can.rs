//! # Controller area network controller

use nb;

pub use crate::hal_can::{Id, Frame as FrameTrait,Can as CanTrait};
use crate::pac::{can0, CAN0, CAN1};
use crate::gpio::gpioa::{PA11, PA12};
use crate::gpio::gpiob::{PB5, PB6, PB8, PB9, PB12, PB13};
use crate::gpio::gpiod::{PD0, PD1};
use crate::gpio::{Alternate, Floating, Input, PushPull};
use crate::rcu::{Rcu, Enable, Reset, BaseFrequency};
use crate::time::Hertz;
use crate::afio::{Afio, Remap};
use core::{convert::TryInto, ops::Deref};

pub struct Frame{
}

impl FrameTrait for Frame {
    /// Creates a new frame.
    /// Returns an error when the data slice is too long.
    fn new(id: impl Into<Id>, data: &[u8]) -> Result<Self, ()>{
        unimplemented!()
    }

    /// Creates a new remote frame (RTR bit set).
    /// Returns an error when the data length code (DLC) is not valid.
    fn new_remote(id: impl Into<Id>, dlc: usize) -> Result<Self, ()>{
        unimplemented!()
    }

    /// Returns true if this frame is a extended frame.
    fn is_extended(&self) -> bool{
        unimplemented!()
    }


    /// Returns true if this frame is a remote frame.
    fn is_remote_frame(&self) -> bool{
        unimplemented!()
    }

    /// Returns the frame identifier.
    fn id(&self) -> Id{
        unimplemented!()
    }

    /// Returns the data length code (DLC) which is in the range 0..8.
    ///
    /// For data frames the DLC value always matches the length of the data.
    /// Remote frames do not carry any data, yet the DLC can be greater than 0.
    fn dlc(&self) -> usize{
        unimplemented!()
    }

    /// Returns the frame data (0..8 bytes in length).
    fn data(&self) -> &[u8]{
        unimplemented!()
    }
}

/// Can error
#[derive(Debug)]
pub enum Error {
    /// Overrun occurred
    Overrun,
    /// Mode fault occurred
    ModeFault,
    /// CRC error
    Crc,
    #[doc(hidden)]
    _Extensible,
}

pub enum CanMode{
    Normal = 0b00,
    Silent = 0b10,
    Loopback = 0b01,
    SilentLoopback = 0b11
}

#[derive(Clone)]
pub struct TimeQuantaConfiguration{
    pub sync: u8,
    pub propagandation: u8,
    pub before_sample: u8,
    pub after_sample: u8
}

impl TimeQuantaConfiguration{
    fn sum(&self) -> u32{
        (self.sync + self.propagandation + self.before_sample + self.after_sample) as u32
    }
    fn bs1(&self) -> u8 {
        self.propagandation + self.before_sample
    }
    fn bs2(&self) -> u8 {
        self.after_sample
    }
}

#[doc(hidden)]
pub trait CanX: Deref<Target = can0::RegisterBlock> {}
impl CanX for CAN0 {}
impl CanX for CAN1 {}

pub trait Pins<CAN> {
    type Variant;
    const REMAP: Self::Variant;
}

impl Pins<CAN0>
    for (
        PA11<Input<Floating>>,
        PA12<Alternate<PushPull>>,
    )
{
    type Variant = u8;
    const REMAP: u8 = 0b00;
}
impl Pins<CAN0>
    for (
        PB8<Input<Floating>>,
        PB9<Alternate<PushPull>>,
    )
{
    type Variant = u8;
    const REMAP: u8 = 0b10;
}
impl Pins<CAN0>
    for (
        PD0<Input<Floating>>,
        PD1<Alternate<PushPull>>,
    )
{
    type Variant = u8;
    const REMAP: u8 = 0b11;
}

impl Pins<CAN1>
    for (
        PB12<Input<Floating>>,
        PB13<Alternate<PushPull>>,
    )
{
    type Variant = bool;
    const REMAP: bool = false;
}
impl Pins<CAN1>
    for (
        PB5<Input<Floating>>,
        PB6<Alternate<PushPull>>,
    )
{
    type Variant = bool;
    const REMAP: bool = true;
}

pub struct Can<CAN, PINS> {
    can: CAN,
    pins: PINS,
}

impl<PINS: Pins<CAN0, Variant=u8>> Can<CAN0, PINS> {
    pub fn can0(
        can: CAN0,
        pins: PINS,
        afio: &mut Afio,
        tqc : TimeQuantaConfiguration,
        mode: CanMode,
        freq: impl Into<Hertz>,
        rcu: &mut Rcu
    ) -> Self
    {
        CAN0::remap(afio, PINS::REMAP);
        Can::new(can, pins, tqc, mode,freq, rcu)
    }
}

impl<PINS: Pins<CAN1, Variant=bool>> Can<CAN1, PINS> {
    pub fn can1(
        can: CAN1,
        pins: PINS,
        afio: &mut Afio,
        tqc : TimeQuantaConfiguration,
        mode: CanMode,
        freq: impl Into<Hertz>,
        rcu: &mut Rcu
    ) -> Self
    {
        CAN1::remap(afio, PINS::REMAP);
        Can::new(can, pins, tqc, mode, freq, rcu)
    }
}


impl<CAN, PINS> Can<CAN, PINS> where CAN: CanX
{
    const MAX_PSC : u16 = 0b111111111;
    fn new(
        can: CAN,
        pins: PINS,
        tqc : TimeQuantaConfiguration,
        mode: CanMode,
        freq: impl Into<Hertz>,
        rcu: &mut Rcu
    ) -> Self where CAN: Enable + Reset + BaseFrequency {

        let baudpsc:u16= (
            CAN::base_frequency(rcu).0 / (freq.into().0  * tqc.sum() )
        ).try_into().unwrap_or(Self::MAX_PSC);
        let baudpsc:u16 = if baudpsc > Self::MAX_PSC {
            Self::MAX_PSC
        } else if baudpsc == 0 {
            unreachable!();
        } else{
            baudpsc
        };

        CAN::enable(rcu);
        CAN::reset(rcu);
        can.bt.write(|w| unsafe{w
            .baudpsc().bits(baudpsc)
            .scmod().bit(match mode {
                CanMode::Silent | CanMode::SilentLoopback => {true},
                _ => {false},
            })
            .lcmod().bit(match mode {
                CanMode::Loopback | CanMode::SilentLoopback => {true},
                _ => {false},
            })
            .sjw().bits(tqc.sync)
            .bs1().bits(tqc.bs1())
            .bs2().bits(tqc.bs2())
        });
        Can{can, pins}
    }
}


/// A CAN interface that is able to transmit and receive frames.
impl<CAN, PINS> CanTrait for Can<CAN, PINS> {
    /// Associated frame type.
    type Frame = Frame;

    /// Associated error type.
    type Error = Error;

    /// Puts a frame in the transmit buffer to be sent on the bus.
    ///
    /// If the transmit buffer is full, this function will try to replace a pending
    /// lower priority frame and return the frame that was replaced.
    /// Returns `Err(WouldBlock)` if the transmit buffer is full and no frame can be
    /// replaced.
    ///
    /// # Notes for implementers
    ///
    /// * Frames of equal identifier shall be transmited in FIFO fashion when more
    ///   than one transmit buffer is available.
    /// * When replacing pending frames make sure the frame is not in the process of
    ///   being send to the bus.
    fn try_transmit(&mut self, frame: &Self::Frame) 
        -> nb::Result<Option<Self::Frame>, Self::Error>{
            unimplemented!()
    }

    /// Returns a received frame if available.
    fn try_receive(&mut self) -> nb::Result<Self::Frame, Self::Error>{
        unimplemented!()
    }
}