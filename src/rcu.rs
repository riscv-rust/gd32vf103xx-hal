use crate::pac::{rcu, RCU, PMU};
use riscv::interrupt;
use crate::backup_domain::BackupDomain;
use crate::time::Hertz;
use core::cmp;


/// Extension trait that constrains the `RCU` peripheral
pub trait RcuExt {
    /// Constrains the `RCU` peripheral so it plays nicely with the other abstractions
    fn constrain(self) -> Rcu;
}

impl RcuExt for RCU {
    fn constrain(self) -> Rcu {
        Rcu {
            ahb: AHB::new(),
            apb1: APB1::new(),
            apb2: APB2::new(),
            cctl: CCTL::new(),
            bkp: BKP { _0: () },
        }
    }
}

/// Constrained RCU peripheral
pub struct Rcu {
    /// AMBA High-performance Bus (AHB) registers
    pub ahb: AHB,
    /// Advanced Peripheral Bus 1 (APB1) registers
    pub apb1: APB1,
    /// Advanced Peripheral Bus 2 (APB2) registers
    pub apb2: APB2,
    pub cctl: CCTL,
    pub bkp: BKP,
}

/// AMBA High-performance Bus (AHB) registers
pub struct AHB {
    _0: (),
}

impl AHB {
    fn new() -> Self {
        Self { _0: () }
    }

    // TODO remove `allow`
    #[allow(dead_code)]
    pub(crate) fn en(&mut self) -> &rcu::AHBEN {
        // NOTE(unsafe) this proxy grants exclusive access to this register
        unsafe { &(*RCU::ptr()).ahben }
    }
}

/// Advanced Peripheral Bus 1 (APB1) registers
pub struct APB1 {
    _0: (),
}


impl APB1 {
    fn new() -> Self {
        Self { _0: () }
    }

    pub(crate) fn en(&mut self) -> &rcu::APB1EN {
        // NOTE(unsafe) this proxy grants exclusive access to this register
        unsafe { &(*RCU::ptr()).apb1en }
    }

    pub(crate) fn rstr(&mut self) -> &rcu::APB1RST {
        // NOTE(unsafe) this proxy grants exclusive access to this register
        unsafe { &(*RCU::ptr()).apb1rst }
    }
}


impl APB1 {
    /// Set power interface clock (PWREN) bit in RCU_APB1EN
    pub fn set_pwren(&mut self) {
        self.en().modify(|_r, w| {
            w.pmuen().set_bit()
        })
    }
}

/// Advanced Peripheral Bus 2 (APB2) registers
pub struct APB2 {
    _0: (),
}

impl APB2 {
    fn new() -> Self {
        Self { _0: () }
    }

    pub(crate) fn en(&mut self) -> &rcu::APB2EN {
        // NOTE(unsafe) this proxy grants exclusive access to this register
        unsafe { &(*RCU::ptr()).apb2en }
    }

    pub(crate) fn rstr(&mut self) -> &rcu::APB2RST {
        // NOTE(unsafe) this proxy grants exclusive access to this register
        unsafe { &(*RCU::ptr()).apb2rst }
    }
}

pub struct CCTL {
    hxtal: Option<u32>,
    sysclk: Option<u32>,
}

impl CCTL {
    fn new() -> Self {
        Self {
            hxtal: None,
            sysclk: None,
        }
    }

    /// Uses an external oscillator instead of IRC8M (internal RC oscillator) as the high-speed
    /// clock source. Will result in a hang if an external oscillator is not connected or it fails
    /// to start.
    pub fn ext_hf_clock(mut self, freq: impl Into<Hertz>) -> Self {
        let freq = freq.into().0;
        assert!(4_000_000 <= freq && freq <= 32_000_000);

        self.hxtal = Some(freq);
        self
    }

    /// Sets the desired frequency for the SYSCLK clock
    pub fn sysclk(mut self, freq: impl Into<Hertz>) -> Self {
        let freq = freq.into().0;
        assert!(freq <= 108_000_000);

        self.sysclk = Some(freq);
        self
    }

    /// Freezes clock configuration, making it effective
    pub fn freeze(self) -> Clocks {
        const IRC8M: u32 = 8_000_000;

        let target_sysclk = self.sysclk.unwrap_or(IRC8M);

        let (scs_bits, use_pll) = match (self.hxtal, target_sysclk) {
            (Some(freq), sysclk) if freq == sysclk => (0b01, false),
            (None, sysclk) if IRC8M == sysclk => (0b00, false),
            _ => (0b10, true),
        };

        let pllsel_bit;
        let predv0_bits;
        let pllmf_bits;
        if use_pll {
            let pllmf;

            if let Some(hxtal_freq) = self.hxtal {
                // Use external clock + divider
                pllsel_bit = true;

                let calculate_pll = |source: u32, target: u32| -> Option<(u8, u8)> {
                    const PLL_IN_MIN: u32 = 600_000;
                    let div_max = cmp::min(16, source / PLL_IN_MIN);

                    for d in 1..=div_max {
                        let pllsource = source / d;
                        let pllm = target / pllsource;
                        if pllm < 2 || pllm == 15 || pllm > 32{
                            continue;
                        }
                        let actual_freq = pllsource * pllm;
                        if actual_freq == target {
                            return Some((d as u8, pllm as u8));
                        }
                    }
                    None
                };

                let (d, m) = calculate_pll(hxtal_freq, target_sysclk).expect("invalid sysclk value");
                predv0_bits = d - 1;
                pllmf = m;
            } else {
                // IRC8M/2 is used as an input clock
                pllsel_bit = false;

                let pllsource = IRC8M / 2;
                let m = target_sysclk / pllsource;
                let m = cmp::max(2, cmp::min(m, 32));
                assert_ne!(m, 15, "invalid sysclk value");
                let actual_sysclk = pllsource * m;
                assert_eq!(target_sysclk, actual_sysclk, "invalid sysclk value");

                predv0_bits = 0;
                pllmf = m as u8;
            }

            pllmf_bits = match pllmf {
                2..=14 => pllmf - 2,
                16..=32 => pllmf - 1,
                _ => unreachable!("invalid pll multiplier"),
            };
        } else {
            pllsel_bit = false;
            predv0_bits = 0;
            pllmf_bits = 0;
        }

        // Switch to the internal clock
        let rcu = unsafe { &*crate::pac::RCU::ptr() };
        rcu.ctl.modify(|_, w| w.irc8men().set_bit()); // Enable IRC8M oscillator
        while rcu.ctl.read().irc8mstb().bit_is_clear() {} // Wait for oscillator to stabilize
        rcu.cfg0.modify(|_, w| unsafe { w.scs().bits(0b00) }); // Switch to the internal oscillator
        rcu.ctl.modify(|_, w| w.pllen().clear_bit()); // Disable PLL

        // Set bus prescalers
        rcu.cfg0.modify(|_, w| unsafe { w.ahbpsc().bits(0b0000) }); // CK_SYS
        rcu.cfg0.modify(|_, w| unsafe { w.apb1psc().bits(0b100) }); // CK_AHB / 2
        rcu.cfg0.modify(|_, w| unsafe { w.apb2psc().bits(0b000) }); // CK_AHB

        if self.hxtal.is_some() {
            // Enable external oscillator
            rcu.ctl.modify(|_, w| w.hxtalen().set_bit());
            // Wait for oscillator to stabilize
            while rcu.ctl.read().hxtalstb().bit_is_clear() {}

            // Select HXTAL as prescaler input source clock
            rcu.cfg1.modify(|_, w| w.predv0sel().clear_bit());
            // Configure the prescaler
            rcu.cfg1.modify(|_, w| unsafe { w.predv0().bits(predv0_bits) });
        }

        if use_pll {
            // Configure PLL input selector
            rcu.cfg0.modify(|_, w| w.pllsel().bit(pllsel_bit));
            // Configure PLL multiplier
            rcu.cfg0.modify(|_, w| unsafe { w
                .pllmf_4().bit(pllmf_bits & 0x10 != 0)
                .pllmf_3_0().bits(pllmf_bits & 0xf)
            });
            // Enable PLL
            rcu.ctl.modify(|_, w| w.pllen().set_bit());
            // Wait for PLL to stabilize
            while rcu.ctl.read().pllstb().bit_is_clear() {}
        } else {
            // Disable PLL
            rcu.ctl.modify(|_, w| w.pllen().clear_bit());
        }

        // Switch to the configured clock source
        rcu.cfg0.modify(|_, w| unsafe { w.scs().bits(scs_bits) });

        let usbclk_valid;
        if use_pll {
            let pllclk = target_sysclk;
            let (valid, pr) = match pllclk {
                48_000_000 => (true, 0b01), // pllclk / 1
                72_000_000 => (true, 0b00), // pllclk / 1.5
                96_000_000 => (true, 0b11), // pllclk / 2
                _ => (false, 0),
            };
            usbclk_valid = valid;

            // Configure USB prescaler
            rcu.cfg0.modify(|_, w| unsafe { w.usbfspsc().bits(pr) });
        } else {
            usbclk_valid = false;
        }

        Clocks {
            sysclk: Hertz(target_sysclk),
            usbclk_valid
        }
    }
}

#[derive(Copy, Clone)]
pub struct Clocks {
    sysclk: Hertz,
    usbclk_valid: bool,
}

impl Clocks {
    /// Returns the system (core) frequency
    pub fn sysclk(&self) -> Hertz {
        self.sysclk
    }

    /// Returns the frequency of the AHB
    pub fn hclk(&self) -> Hertz {
        self.sysclk
    }

    /// Returns the frequency of the APB1
    pub fn pclk1(&self) -> Hertz {
        Hertz(self.sysclk.0 / 2)
    }

    /// Returns the frequency of the APB2
    pub fn pclk2(&self) -> Hertz {
        self.sysclk
    }

    /// Returns whether the USBCLK clock frequency is valid for the USB peripheral
    pub fn usbclk_valid(&self) -> bool {
        self.usbclk_valid
    }
}

pub struct BKP {
    _0: ()
}

impl BKP {
    /// Enables write access to the registers in the backup domain
    pub fn constrain(self, bkp: crate::pac::BKP, apb1: &mut APB1, pmu: &mut PMU) -> BackupDomain {
        // Enable the backup interface by setting PWREN and BKPEN
        apb1.en().modify(|_r, w| {
            w
                .bkpien().set_bit()
                .pmuen().set_bit()
        });

        // Enable access to the backup registers
        pmu.ctl.modify(|_r, w| {
            w
                .bkpwen().set_bit()
        });

        BackupDomain {
            _regs: bkp,
        }
    }
}

/// Bus associated to peripheral
pub trait RcuBus {
    /// Bus type;
    type Bus;
}

/// Enable/disable peripheral
pub(crate) trait Enable: RcuBus {
    fn enable();
    fn disable();
}

/// Reset peripheral
pub(crate) trait Reset: RcuBus {
    fn reset();
}

macro_rules! bus {
    ($($PER:ident => ($apbX:ty, $peren:ident, $perrst:ident),)+) => {
        $(
            impl RcuBus for crate::pac::$PER {
                type Bus = $apbX;
            }
            impl Enable for crate::pac::$PER {
                #[inline(always)]
                fn enable() {
                    let mut apb = <$apbX>::new();
                    interrupt::free(|_| {
                        apb.en().modify(|_, w| w.$peren().set_bit());
                    });
                }

                #[inline(always)]
                fn disable() {
                    let mut apb = <$apbX>::new();
                    interrupt::free(|_| {
                        apb.en().modify(|_, w| w.$peren().clear_bit());
                    });
                }
            }
            impl Reset for crate::pac::$PER {
                #[inline(always)]
                fn reset() {
                    let mut apb = <$apbX>::new();
                    interrupt::free(|_| {
                        apb.rstr().modify(|_, w| w.$perrst().set_bit());
                        apb.rstr().modify(|_, w| w.$perrst().clear_bit());
                    });
                }
            }
        )+
    }
}

bus! {
    TIMER0 => (APB2, timer0en, timer0rst),
    ADC1 => (APB2, adc1en, adc1rst),
    CAN0 => (APB1, can0en, can0rst),
    ADC0 => (APB2, adc0en, adc0rst),
    AFIO => (APB2, afen, afrst),
    GPIOA => (APB2, paen, parst),
    GPIOB => (APB2, pben, pbrst),
    GPIOC => (APB2, pcen, pcrst),
    GPIOD => (APB2, pden, pdrst),
    GPIOE => (APB2, peen, perst),
    I2C0 => (APB1, i2c0en, i2c0rst),
    I2C1 => (APB1, i2c1en, i2c1rst),
    SPI0 => (APB2, spi0en, spi0rst),
    SPI1 => (APB1, spi1en, spi1rst),
    SPI2 => (APB1, spi2en, spi2rst),
    TIMER1 => (APB1, timer1en, timer1rst),
    TIMER2 => (APB1, timer2en, timer2rst),
    TIMER3 => (APB1, timer3en, timer3rst),
    TIMER4 => (APB1, timer4en, timer4rst),
    TIMER5 => (APB1, timer5en, timer5rst),
    TIMER6 => (APB1, timer6en, timer6rst),
    USART0 => (APB2, usart0en, usart0rst),
    USART1 => (APB1, usart1en, usart1rst),
    USART2 => (APB1, usart2en, usart2rst),
    UART3 => (APB1, uart3en, uart3rst),
    WWDGT => (APB1, wwdgten, wwdgtrst),
}
