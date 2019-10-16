use crate::pac::{rcu, RCU, PMU};

use crate::backup_domain::BackupDomain;


/// Extension trait that constrains the `RCU` peripheral
pub trait RcuExt {
    /// Constrains the `RCU` peripheral so it plays nicely with the other abstractions
    fn constrain(self) -> Rcu;
}

impl RcuExt for RCU {
    fn constrain(self) -> Rcu {
        Rcu {
            ahb: AHB { _0: () },
            apb1: APB1 { _0: () },
            apb2: APB2 { _0: () },
            //cfgr: CFGR {
                //hse: None,
                //hclk: None,
                //pclk1: None,
                //pclk2: None,
                //sysclk: None,
                //adcclk: None,
            //},
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
    //pub cfgr: CFGR,
    pub bkp: BKP,
}

/// AMBA High-performance Bus (AHB) registers
pub struct AHB {
    _0: (),
}

impl AHB {
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
    pub(crate) fn en(&mut self) -> &rcu::APB2EN {
        // NOTE(unsafe) this proxy grants exclusive access to this register
        unsafe { &(*RCU::ptr()).apb2en }
    }

    pub(crate) fn rstr(&mut self) -> &rcu::APB2RST {
        // NOTE(unsafe) this proxy grants exclusive access to this register
        unsafe { &(*RCU::ptr()).apb2rst }
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
    fn enable(apb: &mut Self::Bus);
    fn disable(apb: &mut Self::Bus);
}

/// Reset peripheral
pub(crate) trait Reset: RcuBus {
    fn reset(apb: &mut Self::Bus);
}

macro_rules! bus {
    ($($PER:ident => ($apbX:ty, $peren:ident, $perrst:ident),)+) => {
        $(
            impl RcuBus for crate::pac::$PER {
                type Bus = $apbX;
            }
            impl Enable for crate::pac::$PER {
                #[inline(always)]
                fn enable(apb: &mut Self::Bus) {
                    apb.en().modify(|_, w| w.$peren().set_bit());
                }
                #[inline(always)]
                fn disable(apb: &mut Self::Bus) {
                    apb.en().modify(|_, w| w.$peren().clear_bit());
                }
            }
            impl Reset for crate::pac::$PER {
                #[inline(always)]
                fn reset(apb: &mut Self::Bus) {
                    apb.rstr().modify(|_, w| w.$perrst().set_bit());
                    apb.rstr().modify(|_, w| w.$perrst().clear_bit());
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
    TIMER1 => (APB1, timer1en, timer1rst),
    TIMER2 => (APB1, timer2en, timer2rst),
    TIMER3 => (APB1, timer3en, timer3rst),
    USART0 => (APB2, usart0en, usart0rst),
    USART1 => (APB1, usart1en, usart1rst),
    UART3 => (APB1, uart3en, uart3rst),
    WWDGT => (APB1, wwdgten, wwdgtrst),
}
