use crate::pac::AFIO;

pub trait Remap {
    type Variant;

    fn remap(variant: Self::Variant);
}

macro_rules! remap_set {
    ($field:ident, bool, $value:ident) => {
        let pcf0 = &(*AFIO::ptr()).pcf0;
        pcf0.write(|w| w.$field().bit($value));
    };
    ($field:ident, $type:ty, $value:ident) => {
        let pcf0 = &(*AFIO::ptr()).pcf0;
        pcf0.write(|w| w.$field().bits(u8::from($value)));
    }
}

macro_rules! remap {
    ($($PER:ident => ($field:ident, $variant:tt),)+) => {
        $(
            impl Remap for crate::pac::$PER {
                type Variant = $variant;

                #[inline(always)]
                fn remap(variant: $variant) {
                    unsafe {
                        remap_set!($field, $variant, variant);
                    }
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
