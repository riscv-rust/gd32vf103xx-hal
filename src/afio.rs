use crate::pac::AFIO;
use crate::rcu::{Rcu, Enable, Reset};

pub trait AfioExt {
    fn constrain(self, rcu: &mut Rcu) -> Afio;
}

impl AfioExt for AFIO {
    fn constrain(self, rcu: &mut Rcu) -> Afio {
        AFIO::enable(rcu);
        AFIO::reset(rcu);

        Afio { afio: self }
    }
}

pub struct Afio {
    afio: AFIO
}

pub trait Remap {
    type Variant;

    fn remap(afio: &mut Afio, variant: Self::Variant);
}

macro_rules! remap_set {
    ($pcf0:ident, $field:ident, bool, $value:ident) => {
        $pcf0.write(|w| w.$field().bit($value));
    };
    ($pcf0:ident, $field:ident, $type:ty, $value:ident) => {
        $pcf0.write(|w| unsafe {
            w.$field().bits(u8::from($value))
        });
    }
}

macro_rules! remap {
    ($($PER:ident => ($field:ident, $variant:tt),)+) => {
        $(
            impl Remap for crate::pac::$PER {
                type Variant = $variant;

                #[inline(always)]
                fn remap(afio: &mut Afio, variant: $variant) {
                    let pcf0 = &afio.afio.pcf0;
                    remap_set!(pcf0, $field, $variant, variant);
                }
            }
        )+
    }
}

remap! {
    SPI0 => (spi0_remap, bool),
    SPI2 => (spi2_remap, bool),
    USART0 => (usart0_remap, bool),
    USART1 => (usart1_remap, bool),
    USART2 => (usart2_remap, u8),
}
