//! # API for the Analog to Digital converter

use embedded_hal::adc::{Channel, OneShot};
use gd32vf103_pac::{ADC0, ADC1};

use crate::gpio::Analog;
use crate::gpio::{gpioa, gpiob, gpioc};
use crate::rcu::{Rcu, Clocks, Enable, Reset};

use crate::delay::McycleDelay;
use embedded_hal::blocking::delay::DelayUs;


#[derive(Clone, Copy, Debug, PartialEq)]
#[allow(non_camel_case_types)]
/// ADC sampling time
///
/// Options for the sampling time, each is T + 0.5 ADC clock cycles.
pub enum SampleTime {
    /// 1.5 cycles sampling time
    T_1,
    /// 7.5 cycles sampling time
    T_7,
    /// 13.5 cycles sampling time
    T_13,
    /// 28.5 cycles sampling time
    T_28,
    /// 41.5 cycles sampling time
    T_41,
    /// 55.5 cycles sampling time
    T_55,
    /// 71.5 cycles sampling time
    T_71,
    /// 239.5 cycles sampling time
    T_239,
}

impl Default for SampleTime {
    /// Get the default sample time (currently 28.5 cycles)
    fn default() -> Self {
        SampleTime::T_28
    }
}

impl From<SampleTime> for u8 {
    fn from(val: SampleTime) -> Self {
        use SampleTime::*;
        match val {
            T_1 => 0,
            T_7 => 1,
            T_13 => 2,
            T_28 => 3,
            T_41 => 4,
            T_55 => 5,
            T_71 => 6,
            T_239 => 7,
        }
    }
}

#[derive(Clone, Copy, Debug, PartialEq)]
/// ADC data register alignment
pub enum Align {
    /// Right alignment of output data
    Right,
    /// Left alignment of output data
    Left,
}

impl Default for Align {
    /// Default: right alignment
    fn default() -> Self {
        Align::Right
    }
}

impl From<Align> for bool {
    fn from(val: Align) -> Self {
        match val {
            Align::Right => false,
            Align::Left => true,
        }
    }
}

macro_rules! adc_pins {
    ($ADC:ident, $($pin:ty => $chan:expr),+ $(,)*) => {
        $(
            impl Channel<$ADC> for $pin {
                type ID = u8;

                fn channel() -> u8 { $chan }
            }
        )+
    };
}

adc_pins!(ADC0,
    gpioa::PA0<Analog> => 0_u8,
    gpioa::PA1<Analog> => 1_u8,
    gpioa::PA2<Analog> => 2_u8,
    gpioa::PA3<Analog> => 3_u8,
    gpioa::PA4<Analog> => 4_u8,
    gpioa::PA5<Analog> => 5_u8,
    gpioa::PA6<Analog> => 6_u8,
    gpioa::PA7<Analog> => 7_u8,
    gpiob::PB0<Analog> => 8_u8,
    gpiob::PB1<Analog> => 9_u8,
    gpioc::PC0<Analog> => 10_u8,
    gpioc::PC1<Analog> => 11_u8,
    gpioc::PC2<Analog> => 12_u8,
    gpioc::PC3<Analog> => 13_u8,
    gpioc::PC4<Analog> => 14_u8,
    gpioc::PC5<Analog> => 15_u8,
);

adc_pins!(ADC1,
    gpioa::PA0<Analog> => 0_u8,
    gpioa::PA1<Analog> => 1_u8,
    gpioa::PA2<Analog> => 2_u8,
    gpioa::PA3<Analog> => 3_u8,
    gpioa::PA4<Analog> => 4_u8,
    gpioa::PA5<Analog> => 5_u8,
    gpioa::PA6<Analog> => 6_u8,
    gpioa::PA7<Analog> => 7_u8,
    gpiob::PB0<Analog> => 8_u8,
    gpiob::PB1<Analog> => 9_u8,
    gpioc::PC0<Analog> => 10_u8,
    gpioc::PC1<Analog> => 11_u8,
    gpioc::PC2<Analog> => 12_u8,
    gpioc::PC3<Analog> => 13_u8,
    gpioc::PC4<Analog> => 14_u8,
    gpioc::PC5<Analog> => 15_u8,
);


pub enum ConversionMode {
    SingleMode,
    ContinuousMode,
    ScanMode,
    DiscontinuousMode,
}

/// ADC configuration
pub struct Adc<ADC> {
    adc: ADC,
    sample_time: SampleTime,
    align: Align,
    clocks: Clocks,
}



macro_rules! adc_hal {
    ($(
        $ADC:ident: ($adc:ident),
    )+) => {
        $(

            impl Adc<$ADC> {
                /// Init a new Adc
                ///
                /// Sets all configurable parameters to one-shot defaults,
                /// performs a boot-time calibration.
                pub fn new(rcu: &mut Rcu, adc: $ADC) -> Self {
                    let mut s = Self {
                        adc: adc,
                        sample_time: SampleTime::default(),
                        align: Align::default(),
                        clocks: rcu.clocks,
                    };

                    $ADC::enable(rcu);
                    $ADC::reset(rcu);

                    s.power_up();
                    s.calibrate();
                    s
                }


                /// Set ADC sampling time
                ///
                /// Options can be found in [SampleTime](crate::adc::SampleTime).
                pub fn set_sample_time(&mut self, t_samp: SampleTime) {
                    self.sample_time = t_samp;
                }

                /// Set the Adc result alignment
                ///
                /// Options can be found in [Align](crate::adc::Align).
                pub fn set_align(&mut self, align: Align) {
                    self.align = align;
                }

                /// Returns the largest possible sample value for the current settings
                pub fn max_sample(&self) -> u16 {
                    match self.align {
                        Align::Left => u16::max_value(),
                        Align::Right => (1 << 12) - 1,
                    }
                }

                fn power_up(&mut self) {
                    /* Up and wait ADC is stabilized. */
                    self.adc.ctl1.modify(|_, w| w.adcon().set_bit());

                    // The reference manual says that a stabilization time is
                    // needed after power_up, this time can be found in the
                    // datasheets, it is 14 adc clock cycles.
                    let clocks = self.clocks;
                    let mut delay = McycleDelay::new(&clocks);

                    delay.delay_us(14_000_000 / clocks.adc().0);
                }

                fn power_down(&mut self) {
                    self.adc.ctl1.modify(|_, w| w.adcon().clear_bit());
                }

                fn calibrate(&mut self) {
                    self.adc.ctl1.modify(|_, w| { w.rstclb().set_bit() });
                    while self.adc.ctl1.read().rstclb().bit_is_set() {}

                    self.adc.ctl1.modify(|_, w| { w.clb().set_bit() });
                    while self.adc.ctl1.read().clb().bit_is_set() {}
                }

                fn setup_oneshot(&mut self) {
                    /* Configure in Free Sync mode */
                    self.adc.ctl0.write(|w| unsafe { w.syncm().bits(0) });

                    self.adc.ctl1.write(|w| unsafe { w
                        /* Configure software trigger */
                        .etsrc().bits(0b111)
                        /* Enable extern trigger */
                        .eterc().set_bit()
                        /* MSB Alignement */
                        .dal().set_bit()
                    });
                }

                pub fn set_channel_sample_time(&mut self, chan: u8, sample_time: SampleTime) {
                    let sample_time = sample_time.into();
                    match chan {
                        0 => self.adc.sampt1.modify(|_, w| unsafe { w.spt0().bits(sample_time) }),
                        1 => self.adc.sampt1.modify(|_, w| unsafe { w.spt1().bits(sample_time) }),
                        2 => self.adc.sampt1.modify(|_, w| unsafe { w.spt2().bits(sample_time) }),
                        3 => self.adc.sampt1.modify(|_, w| unsafe { w.spt3().bits(sample_time) }),
                        4 => self.adc.sampt1.modify(|_, w| unsafe { w.spt4().bits(sample_time) }),
                        5 => self.adc.sampt1.modify(|_, w| unsafe { w.spt5().bits(sample_time) }),
                        6 => self.adc.sampt1.modify(|_, w| unsafe { w.spt6().bits(sample_time) }),
                        7 => self.adc.sampt1.modify(|_, w| unsafe { w.spt7().bits(sample_time) }),
                        8 => self.adc.sampt1.modify(|_, w| unsafe { w.spt8().bits(sample_time) }),
                        9 => self.adc.sampt1.modify(|_, w| unsafe { w.spt9().bits(sample_time) }),

                        10 => self.adc.sampt0.modify(|_, w| unsafe { w.spt10().bits(sample_time) }),
                        11 => self.adc.sampt0.modify(|_, w| unsafe { w.spt11().bits(sample_time) }),
                        12 => self.adc.sampt0.modify(|_, w| unsafe { w.spt12().bits(sample_time) }),
                        13 => self.adc.sampt0.modify(|_, w| unsafe { w.spt13().bits(sample_time) }),
                        14 => self.adc.sampt0.modify(|_, w| unsafe { w.spt14().bits(sample_time) }),
                        15 => self.adc.sampt0.modify(|_, w| unsafe { w.spt15().bits(sample_time) }),
                        16 => self.adc.sampt0.modify(|_, w| unsafe { w.spt16().bits(sample_time) }),
                        17 => self.adc.sampt0.modify(|_, w| unsafe { w.spt17().bits(sample_time) }),
                        _ => unreachable!(),
                    }
                }

                pub fn set_conversion_mode(&mut self, mode: ConversionMode) {}

                pub fn set_regular_sequence (&mut self, channels: &[u8]) {
                    let len = channels.len();
                    let bits = channels.iter().take(6).enumerate().fold(0u32, |s, (i, c)|
                        s | ((*c as u32) << (i * 5))
                    );

                    self.adc.rsq2.write(|w| unsafe { w
                        .bits( bits )
                    });

                    if len > 6 {
                        let bits = channels.iter().skip(6).take(6).enumerate().fold(0u32, |s, (i, c)|
                            s | ((*c as u32) << (i * 5))
                        );
                        self.adc.rsq0.write(|w| unsafe { w
                            .bits( bits )
                        });
                    }else if len > 12 {
                        let bits = channels.iter().skip(12).take(4).enumerate().fold(0u32, |s, (i, c)|
                            s | ((*c as u32) << (i * 5))
                        );
                        self.adc.rsq0.write(|w| unsafe { w
                            .bits( bits )
                        });
                    }

                    self.adc.rsq0.modify(|_, w| unsafe { w.rl().bits((len-1) as u8) });
                }

                fn set_continuous_mode(&mut self, continuous: bool) {}

                fn set_scan_mode(&mut self) {}

                fn set_discontinuous_mode(&mut self, channels_count: Option<u8>) {}

                fn convert(&mut self, chan: u8) -> u16 {
                    self.adc.ctl1.modify(|_, w| w
                        /* MSB Alignement */
                        .dal().set_bit()
                    );

                    self.adc.ctl1.modify(|_, w| w
                        /* trigger a new sample */
                        .swrcst().set_bit()
                    );

                    while self.adc.stat.read().eoc().bit_is_clear() {}

                    let rdata = self.adc.rdata.read().rdata().bits();
                    self.adc.stat.modify(|_, w| w.eoc().clear_bit() );

                    rdata
                }
            }

            impl<WORD, PIN> OneShot<$ADC, WORD, PIN> for Adc<$ADC>
            where
                WORD: From<u16>,
                PIN: Channel<$ADC, ID = u8>,
                {
                    type Error = ();

                    fn read(&mut self, _pin: &mut PIN) -> nb::Result<WORD, Self::Error> {
                        let res = self.convert(PIN::channel());
                        Ok(res.into())
                    }
                }
        )+
    }
}

adc_hal! {
    ADC0: (adc0),
}
