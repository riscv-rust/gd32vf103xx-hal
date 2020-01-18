use crate::timer::Timer;
use crate::time::U32Ext;

use cast::u32;
use embedded_hal::blocking::delay::{DelayMs, DelayUs};
use embedded_hal::timer::CountDown;
use gd32vf103_pac::{TIMER0, TIMER1, TIMER2, TIMER3, TIMER4, TIMER5, TIMER6};

/// TIMER as a delay provider
pub struct Delay<TIMER> where Timer<TIMER>: CountDown {
    timer: Timer<TIMER>,
}

macro_rules! delay {
    ($($TIMER:ident,)+) => {
        $(
            impl Delay<$TIMER> {
                /// Configures the timer as a delay provider
                pub fn new(timer: Timer<$TIMER>) -> Self {

                    Delay { timer, }
                }

                /// Releases the timer resource
                pub fn free(self) -> Timer<$TIMER> {
                    self.timer
                }
            }

            impl DelayMs<u32> for Delay<$TIMER> {
                fn delay_ms(&mut self, ms: u32) {
                    self.delay_us(ms * 1_000);
                }
            }

            impl DelayMs<u16> for Delay<$TIMER> {
                fn delay_ms(&mut self, ms: u16) {
                    self.delay_ms(u32(ms));
                }
            }

            impl DelayMs<u8> for Delay<$TIMER> {
                fn delay_ms(&mut self, ms: u8) {
                    self.delay_ms(u32(ms));
                }
            }

            impl DelayUs<u32> for Delay<$TIMER> {
                fn delay_us(&mut self, us: u32) {
                    let freq = 1_000_000 / us;
                    self.timer.start(freq.hz());
                    while let Err(_) = self.timer.wait() { }
                    self.timer.tim.ctl0.modify(|_, w| w.cen().clear_bit());
                }
            }

            impl DelayUs<u16> for Delay<$TIMER> {
                fn delay_us(&mut self, us: u16) {
                    self.delay_us(u32(us))
                }
            }

            impl DelayUs<u8> for Delay<$TIMER> {
                fn delay_us(&mut self, us: u8) {
                    self.delay_us(u32(us))
                }
            }
        )+
    }
}

delay! {
    TIMER0,
    TIMER1,
    TIMER2,
    TIMER3,
    TIMER4,
    TIMER5,
    TIMER6,
}
