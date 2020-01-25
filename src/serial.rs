//! # Serial Communication (USART)
//! This module contains the functions to utilize the USART (Universal
//! synchronous asynchronous receiver transmitter)
//!
//!
//! ## Example usage:
//!  ```rust
//! // prelude: create handles to the peripherals and registers
//! let p = crate::pac::Peripherals::take().unwrap();
//! let clocks = Clocks;
//! let mut gpioa = p.GPIOA.split();
//!
//! // USART0 on Pins A9 and A10
//! let pin_tx = gpioa.pa9.into_alternate_push_pull();
//! let pin_rx = gpioa.pa10.into_floating_input();
//! // Create an interface struct for USART1 with 9600 Baud
//! let serial = Serial::usart0(
//!     p.USART0,
//!     (pin_tx, pin_rx),
//!     Config::default().baudrate(9_600.bps()),
//!     clocks,
//! );
//!
//! // separate into tx and rx channels
//! let (mut tx, mut rx) = serial.split();
//!
//! // Write 'R' to the USART
//! block!(tx.write(b'R')).ok();
//! // Receive a byte from the USART and store it in "received"
//! let received = block!(rx.read()).unwrap();
//!  ```

use core::marker::PhantomData;
use core::ptr;

use nb;
use crate::pac::{USART0, USART1, USART2};
use core::convert::Infallible;
//use void::Void;
use embedded_hal::serial::Write;

//use crate::afio::MAPR;
//use crate::dma::{dma1, CircBuffer, Static, Transfer, R, W, RxDma, TxDma};
use crate::gpio::gpioa::{PA10, PA2, PA3, PA9};
use crate::gpio::gpiob::{PB10, PB11};
use crate::gpio::{Alternate, Floating, Input, PushPull};
use crate::rcu::{Rcu, Enable, Reset, BaseFrequency};
use crate::time::{U32Ext, Bps};

/// Interrupt event
pub enum Event {
    /// New data has been received
    Rxne,
    /// New data can be sent
    Txe,
}

/// Serial error
#[derive(Debug)]
pub enum Error {
    /// Framing error
    Framing,
    /// Noise error
    Noise,
    /// RX buffer overrun
    Overrun,
    /// Parity check error
    Parity,
    #[doc(hidden)]
    _Extensible,
}

pub trait Pins<USART> {
    const REMAP: u8;
}

impl Pins<USART0> for (PA9<Alternate<PushPull>>, PA10<Input<Floating>>) {
    const REMAP: u8 = 0;
}

//impl Pins<USART0> for (PB6<Alternate<PushPull>>, PB7<Input<Floating>>) {
//    const REMAP: u8 = 1;
//}

impl Pins<USART1> for (PA2<Alternate<PushPull>>, PA3<Input<Floating>>) {
    const REMAP: u8 = 0;
}

// impl Pins<USART1> for (PD5<Alternate<PushPull>>, PD6<Input<Floating>>) {
//     const REMAP: u8 = 0;
// }

impl Pins<USART2> for (PB10<Alternate<PushPull>>, PB11<Input<Floating>>) {
    const REMAP: u8 = 0;
}

// impl Pins<USART2> for (PC10<Alternate<PushPull>>, PC11<Input<Floating>>) {
//     const REMAP: u8 = 1;
// }

// impl Pins<USART2> for (PD8<Alternate<PushPull>>, PD9<Input<Floating>>) {
//     const REMAP: u8 = 0b11;
// }

pub enum Parity {
    ParityNone,
    ParityEven,
    ParityOdd,
}

pub enum StopBits {
    #[doc = "1 stop bit"]
    STOP1,
    #[doc = "0.5 stop bits"]
    STOP0P5,
    #[doc = "2 stop bits"]
    STOP2,
    #[doc = "1.5 stop bits"]
    STOP1P5,
}

pub struct Config {
    pub baudrate: Bps,
    pub parity: Parity,
    pub stopbits: StopBits,
}

impl Config {
    pub fn baudrate(mut self, baudrate: Bps) -> Self {
        self.baudrate = baudrate;
        self
    }

    pub fn parity_none(mut self) -> Self {
        self.parity = Parity::ParityNone;
        self
    }

    pub fn parity_even(mut self) -> Self {
        self.parity = Parity::ParityEven;
        self
    }

    pub fn parity_odd(mut self) -> Self {
        self.parity = Parity::ParityOdd;
        self
    }

    pub fn stopbits(mut self, stopbits: StopBits) -> Self {
        self.stopbits = stopbits;
        self
    }
}

impl Default for Config {
    fn default() -> Config {
        let baudrate = 115_200_u32.bps();
        Config {
            baudrate,
            parity: Parity::ParityNone,
            stopbits: StopBits::STOP1,
        }
    }
}

/// Serial abstraction
pub struct Serial<USART, PINS> {
    usart: USART,
    pins: PINS,
}

/// Serial receiver
pub struct Rx<USART> {
    _usart: PhantomData<USART>,
}

/// Serial transmitter
pub struct Tx<USART> {
    _usart: PhantomData<USART>,
}

macro_rules! hal {
    ($(
        $(#[$meta:meta])*
        $USARTX:ident: (
            $usartX:ident,
            $usartX_remap:ident,
            $bit:ident,
            $closure:expr,
        ),
    )+) => {
        $(
            $(#[$meta])*
            /// The behaviour of the functions is equal for all three USARTs.
            /// Except that they are using the corresponding USART hardware and pins.
            impl<PINS> Serial<$USARTX, PINS> {

                /// Configures the serial interface and creates the interface
                /// struct.
                ///
                /// `Bps` is the baud rate of the interface.
                ///
                /// `Clocks` passes information about the current frequencies of
                /// the clocks.  The existence of the struct ensures that the
                /// clock settings are fixed.
                ///
                /// The `serial` struct takes ownership over the `USARTX` device
                /// registers and the specified `PINS`
                ///
                /// `MAPR` and `APBX` are register handles which are passed for
                /// configuration. (`MAPR` is used to map the USART to the
                /// corresponding pins. `APBX` is used to reset the USART.)
                pub fn $usartX(
                    usart: $USARTX,
                    pins: PINS,
                    //mapr: &mut MAPR,
                    config: Config,
                    rcu: &mut Rcu
                ) -> Self
                where
                    PINS: Pins<$USARTX>,
                {
                    // enable and reset $USARTX
                    $USARTX::enable(rcu);
                    $USARTX::reset(rcu);

//                    #[allow(unused_unsafe)]
//                    mapr.modify_mapr(|_, w| unsafe{
//                            w.$usartX_remap().$bit(($closure)(PINS::REMAP))
//                        });

                    // enable DMA transfers
                    usart.ctl2.write(|w| w.dent().set_bit().denr().set_bit());

                    // Configure baud rate
                    let brr = $USARTX::base_frequency(rcu).0 / config.baudrate.0;
                    assert!(brr >= 16, "impossible baud rate");
                    usart.baud.write(|w| unsafe { w.bits(brr) });

                    // Configure parity and word length
                    // Unlike most uart devices, the "word length" of this usart device refers to
                    // the size of the data plus the parity bit. I.e. "word length"=8, parity=even
                    // results in 7 bits of data. Therefore, in order to get 8 bits and one parity
                    // bit, we need to set the "word" length to 9 when using parity bits.
                    let (word_length, parity_control_enable, parity) = match config.parity {
                        Parity::ParityNone => (false, false, false),
                        Parity::ParityEven => (true, true, false),
                        Parity::ParityOdd => (true, true, true),
                    };
                    usart.ctl0.modify(|_r, w| {
                        w
                            .wl().bit(word_length)
                            .pm().bit(parity)
                            .pcen().bit(parity_control_enable)
                    });

                    // Configure stop bits
                    let stop_bits = match config.stopbits {
                        StopBits::STOP1 => 0b00,
                        StopBits::STOP0P5 => 0b01,
                        StopBits::STOP2 => 0b10,
                        StopBits::STOP1P5 => 0b11,
                    };
                    usart.ctl1.modify(|_r, w| unsafe {
                        w.stb().bits(stop_bits)
                    });

                    // UE: enable USART
                    // RE: enable receiver
                    // TE: enable transceiver
                    usart
                        .ctl0
                        .modify(|_r, w| w.uen().set_bit().ren().set_bit().ten().set_bit());

                    Serial { usart, pins }
                }

                /// Starts listening to the USART by enabling the _Received data
                /// ready to be read (RXNE)_ interrupt and _Transmit data
                /// register empty (TXE)_ interrupt
                pub fn listen(&mut self, event: Event) {
                    match event {
                        Event::Rxne => self.usart.ctl0.modify(|_, w| w.rbneie().set_bit()),
                        Event::Txe => self.usart.ctl0.modify(|_, w| w.tbeie().set_bit()),
                    }
                }

                /// Stops listening to the USART by disabling the _Received data
                /// ready to be read (RXNE)_ interrupt and _Transmit data
                /// register empty (TXE)_ interrupt
                pub fn unlisten(&mut self, event: Event) {
                    match event {
                        Event::Rxne => self.usart.ctl0.modify(|_, w| w.rbneie().clear_bit()),
                        Event::Txe => self.usart.ctl0.modify(|_, w| w.tbeie().clear_bit()),
                    }
                }

                /// Returns ownership of the borrowed register handles
                pub fn release(self) -> ($USARTX, PINS) {
                    (self.usart, self.pins)
                }

                /// Separates the serial struct into separate channel objects for sending (Tx) and
                /// receiving (Rx)
                pub fn split(self) -> (Tx<$USARTX>, Rx<$USARTX>) {
                    (
                        Tx {
                            _usart: PhantomData,
                        },
                        Rx {
                            _usart: PhantomData,
                        },
                    )
                }
            }

            impl Tx<$USARTX> {
                pub fn listen(&mut self) {
                    unsafe { (*$USARTX::ptr()).ctl0.modify(|_, w| w.tbeie().set_bit()) };
                }

                pub fn unlisten(&mut self) {
                    unsafe { (*$USARTX::ptr()).ctl0.modify(|_, w| w.tbeie().clear_bit()) };
                }
            }

            impl Rx<$USARTX> {
                pub fn listen(&mut self) {
                    unsafe { (*$USARTX::ptr()).ctl0.modify(|_, w| w.rbneie().set_bit()) };
                }

                pub fn unlisten(&mut self) {
                    unsafe { (*$USARTX::ptr()).ctl0.modify(|_, w| w.rbneie().clear_bit()) };
                }
            }

            impl crate::hal::serial::Read<u8> for Rx<$USARTX> {
                type Error = Error;

                fn read(&mut self) -> nb::Result<u8, Error> {
                    // NOTE(unsafe) atomic read with no side effects
                    let sr = unsafe { (*$USARTX::ptr()).stat.read() };

                    // Check for any errors
                    let err = if sr.perr().bit_is_set() {
                        Some(Error::Parity)
                    } else if sr.ferr().bit_is_set() {
                        Some(Error::Framing)
                    } else if sr.nerr().bit_is_set() {
                        Some(Error::Noise)
                    } else if sr.orerr().bit_is_set() {
                        Some(Error::Overrun)
                    } else {
                        None
                    };

                    if let Some(err) = err {
                        // Some error occured. In order to clear that error flag, you have to
                        // do a read from the sr register followed by a read from the dr
                        // register
                        // NOTE(read_volatile) see `write_volatile` below
                        unsafe {
                            ptr::read_volatile(&(*$USARTX::ptr()).stat as *const _ as *const _);
                            ptr::read_volatile(&(*$USARTX::ptr()).data as *const _ as *const _);
                        }
                        Err(nb::Error::Other(err))
                    } else {
                        // Check if a byte is available
                        if sr.rbne().bit_is_set() {
                            // Read the received byte
                            // NOTE(read_volatile) see `write_volatile` below
                            Ok(unsafe {
                                ptr::read_volatile(&(*$USARTX::ptr()).data as *const _ as *const _)
                            })
                        } else {
                            Err(nb::Error::WouldBlock)
                        }
                    }
                }
            }

            impl crate::hal::serial::Write<u8> for Tx<$USARTX> {
                type Error = Infallible;

                fn flush(&mut self) -> nb::Result<(), Self::Error> {
                    // NOTE(unsafe) atomic read with no side effects
                    let sr = unsafe { (*$USARTX::ptr()).stat.read() };

                    if sr.tc().bit_is_set() {
                        Ok(())
                    } else {
                        Err(nb::Error::WouldBlock)
                    }
                }

                fn write(&mut self, byte: u8) -> nb::Result<(), Self::Error> {
                    // NOTE(unsafe) atomic read with no side effects
                    let sr = unsafe { (*$USARTX::ptr()).stat.read() };

                    if sr.tbe().bit_is_set() {
                        // NOTE(unsafe) atomic write to stateless register
                        // NOTE(write_volatile) 8-bit write that's not possible through the svd2rust API
                        unsafe {
                            ptr::write_volatile(&(*$USARTX::ptr()).data as *const _ as *mut _, byte)
                        }
                        Ok(())
                    } else {
                        Err(nb::Error::WouldBlock)
                    }
                }
            }
        )+
    }
}

impl<USART> core::fmt::Write for Tx<USART>
where
    Tx<USART>: embedded_hal::serial::Write<u8>,
{
    fn write_str(&mut self, s: &str) -> core::fmt::Result {
        s.as_bytes()
            .iter()
            .try_for_each(|c| nb::block!(self.write(*c)))
            .map_err(|_| core::fmt::Error)
    }
}

hal! {
    /// # USART0 functions
    USART0: (
        usart0,
        usart0_remap,
        bit,
        |remap| remap == 1,
    ),
    /// # USART1 functions
    USART1: (
        usart1,
        usart1_remap,
        bit,
        |remap| remap == 1,
    ),
    /// # USART2 functions
    USART2: (
        usart2,
        usart2_remap,
        bits,
        |remap| remap,
    ),
}
