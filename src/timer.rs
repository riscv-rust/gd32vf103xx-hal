use crate::time::Hertz;
use crate::rcu::{Clocks, APB1, APB2};

use gd32vf103_pac::{TIMER0, TIMER1, TIMER2, TIMER3, TIMER4, TIMER5, TIMER6};
use embedded_hal::timer::{CountDown, Periodic};
use void::Void;

/// Hardware timer
pub struct Timer<TIM> {
    pub(crate) tim: TIM,
    pub(crate) clk: Clocks,
    pub(crate) timeout: Hertz,
}

/// Interrupt events
pub enum Event {
    /// Timer timed out / count down ended
    TimeOut,
}

macro_rules! hal {
    ($($TIM:ident: ($tim:ident, $timXen:ident, $timXrst:ident, $APB:ident),)+) => {
        $(
            impl Timer<$TIM> {
                pub fn $tim<T>(timer: $TIM, timeout: T, clocks: Clocks, apb: &mut $APB) -> Self
                    where T: Into<Hertz> {
                    apb.en().modify(|_r, w| w.$timXen().set_bit());
                    apb.rstr().modify(|_r, w| w.$timXrst().set_bit());
                    apb.rstr().modify(|_r, w| w.$timXrst().clear_bit());

                    let mut t = Timer {
                        clk: clocks,
                        tim: timer,
                        timeout: Hertz(0),
                    };
                    t.start(timeout);

                    t
                }

                /// Releases the TIMER peripheral
                pub fn free(self) -> $TIM {
                    self.tim.ctl0.modify(|_, w| w.cen().clear_bit());
                    self.tim
                }
            }

            impl Periodic for Timer<$TIM> {}

            impl CountDown for Timer<$TIM> {
                type Time = Hertz;

                fn start<T>(&mut self, timeout: T)
                    where
                        T: Into<Hertz>,
                {
                    self.timeout = timeout.into();

                    self.tim.ctl0.modify(|_, w| w.cen().clear_bit());
                    self.tim.cnt.reset();

                    let timer_clock = self.clk.sysclk().0;
                    let ticks = timer_clock / self.timeout.0;
                    let psc = ((ticks - 1) / (1 << 16)) as u16;
                    let car = (ticks / ((psc + 1) as u32)) as u16;
                    self.tim.psc.write(|w| unsafe { w.bits(psc) } );
                    self.tim.car.write(|w| unsafe { w.bits(car) } );
                    self.tim.ctl0.write(|w| { w
                        .updis().clear_bit()
                        .cen().set_bit()
                    });
                }

                fn wait(&mut self) -> nb::Result<(), Void> {
                    if self.tim.intf.read().upif().bit_is_clear() {
                        Err(nb::Error::WouldBlock)
                    } else {
                        self.tim.intf.modify(|_r, w| w.upif().clear_bit());
                        Ok(())
                    }
                }
            }
        )+
    }
}

hal! {
    TIMER0: (timer0, timer0en, timer0rst, APB2),
    TIMER1: (timer1, timer1en, timer1rst, APB1),
    TIMER2: (timer2, timer2en, timer2rst, APB1),
    TIMER3: (timer3, timer3en, timer3rst, APB1),
    TIMER4: (timer4, timer4en, timer4rst, APB1),
    TIMER5: (timer5, timer5en, timer5rst, APB1),
    TIMER6: (timer6, timer6en, timer6rst, APB1),
}