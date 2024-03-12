//! Delays

use crate::time::U32Ext;
use crate::timer::Timer;

use crate::hal::delay::DelayNs;
use crate::hal_02::blocking::delay::{DelayMs, DelayUs};
use crate::hal_02::timer::CountDown;
use crate::rcu::Clocks;
use gd32vf103_pac::{TIMER0, TIMER1, TIMER2, TIMER3, TIMER4, TIMER5, TIMER6};

/// Machine mode cycle counter (`mcycle`) as a delay provider
#[derive(Copy, Clone)]
pub struct McycleDelay {
    core_frequency: u32,
}

impl McycleDelay {
    /// Constructs the delay provider
    pub fn new(clocks: &Clocks) -> Self {
        Self {
            core_frequency: clocks.sysclk().0,
        }
    }
}

impl DelayNs for McycleDelay {
    fn delay_ns(&mut self, ns: u32) {
        let t0 = riscv::register::mcycle::read64();
        let clocks = (ns as u64 * (self.core_frequency as u64)) / 1_000_000_000;
        while riscv::register::mcycle::read64().wrapping_sub(t0) <= clocks {}
    }
}

impl DelayUs<u64> for McycleDelay {
    fn delay_us(&mut self, us: u64) {
        self.delay_ns((us * 1_000) as u32);
    }
}

impl DelayUs<u32> for McycleDelay {
    #[inline(always)]
    fn delay_us(&mut self, us: u32) {
        DelayNs::delay_us(self, us);
    }
}

// Implemented for constructions like `delay.delay_us(50_000);`
impl DelayUs<i32> for McycleDelay {
    #[inline(always)]
    fn delay_us(&mut self, us: i32) {
        assert!(us >= 0);
        DelayNs::delay_us(self, us as u32);
    }
}

impl DelayUs<u16> for McycleDelay {
    #[inline(always)]
    fn delay_us(&mut self, us: u16) {
        DelayNs::delay_us(self, us as u32);
    }
}

impl DelayUs<u8> for McycleDelay {
    #[inline(always)]
    fn delay_us(&mut self, us: u8) {
        DelayNs::delay_us(self, us as u32);
    }
}

impl DelayMs<u32> for McycleDelay {
    fn delay_ms(&mut self, ms: u32) {
        DelayNs::delay_us(self, ms * 1000);
    }
}

// Implemented for constructions like `delay.delay_ms(50_000);`
impl DelayMs<i32> for McycleDelay {
    #[inline(always)]
    fn delay_ms(&mut self, ms: i32) {
        assert!(ms >= 0);
        DelayNs::delay_ms(self, ms as u32);
    }
}

impl DelayMs<u16> for McycleDelay {
    #[inline(always)]
    fn delay_ms(&mut self, ms: u16) {
        DelayNs::delay_ms(self, ms as u32);
    }
}

impl DelayMs<u8> for McycleDelay {
    #[inline(always)]
    fn delay_ms(&mut self, ms: u8) {
        DelayNs::delay_ms(self, ms as u32);
    }
}

/// TIMER as a delay provider
pub struct Delay<TIMER>
where
    Timer<TIMER>: CountDown,
{
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

            impl DelayNs for Delay<$TIMER> {
                fn delay_ns(&mut self, ns: u32) {
                    let freq = 1_000_000_000 / ns;
                    self.timer.start(freq.hz());
                    while let Err(_) = self.timer.wait() { }
                    self.timer.tim.ctl0.modify(|_, w| w.cen().clear_bit());
                }
            }

            impl DelayMs<u32> for Delay<$TIMER> {
                fn delay_ms(&mut self, ms: u32) {
                    DelayNs::delay_ms(self, ms * 1_000);
                }
            }

            impl DelayMs<u16> for Delay<$TIMER> {
                fn delay_ms(&mut self, ms: u16) {
                    DelayNs::delay_ms(self, ms as u32);
                }
            }

            impl DelayMs<u8> for Delay<$TIMER> {
                fn delay_ms(&mut self, ms: u8) {
                    DelayNs::delay_ms(self, ms as u32);
                }
            }

            impl DelayUs<u32> for Delay<$TIMER> {
                fn delay_us(&mut self, us: u32) {
                    DelayNs::delay_us(self, us);
                }
            }

            impl DelayUs<u16> for Delay<$TIMER> {
                fn delay_us(&mut self, us: u16) {
                    DelayNs::delay_us(self, us as u32);
                }
            }

            impl DelayUs<u8> for Delay<$TIMER> {
                fn delay_us(&mut self, us: u8) {
                    DelayNs::delay_us(self, us as u32);
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
