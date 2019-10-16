use crate::rcu::APB2;
use core::marker::PhantomData;

/// Extension trait to split a GPIO peripheral in independent pins and registers
pub trait GpioExt {
    /// The to split the GPIO into
    type Parts;

    /// Splits the GPIO block into independent pins and registers
    fn split(self, apb2: &mut APB2) -> Self::Parts;
}

/// Marker trait for active states.
pub trait Active {}

/// Input mode (type state)
pub struct Input<MODE> {
    _mode: PhantomData<MODE>,
}
impl<MODE> Active for Input<MODE> {}

/// Used by the debugger
pub struct Debugger;
/// Floating Input
pub struct Floating;
/// Pulled down Input
pub struct PullDown;
/// Pulled up Input
pub struct PullUp;

/// Output mode (type state)
pub struct Output<MODE> {
    _mode: PhantomData<MODE>,
}
impl<MODE> Active for Output<MODE> {}

/// Totem Pole aka Push-Pull
pub struct PushPull;
/// Open drain output
pub struct OpenDrain;

/// Analog mode
pub struct Analog;

/// Alternate function
pub struct Alternate<MODE> {
    _mode: PhantomData<MODE>,
}
impl<MODE> Active for Alternate<MODE> {}

pub enum State {
    High,
    Low,
}

macro_rules! gpio {
    ($GPIOX:ident, $gpiox:ident, $gpioy:ident, $PXx:ident, [
        $($PXi:ident: ($pxi:ident, $i:expr, $MODE:ty, $CTL:ident),)+
    ]) => {
        /// GPIO
        pub mod $gpiox {
            use void::Void;
            use core::marker::PhantomData;
            use crate::hal::digital::v2::{OutputPin, InputPin, StatefulOutputPin, toggleable};
            use crate::pac::{$gpioy, $GPIOX};
            use crate::rcu::{APB2, Enable, Reset};
            use super::{
                Alternate, Floating, GpioExt, Input,
                OpenDrain,
                Output,
                PullDown,
                PullUp,
                PushPull,
                Analog,
                State,
                Active,
                Debugger,
                Pxx
            };

            /// GPIO parts
            pub struct Parts {
                /// Opaque CTL0 register
                pub ctl0: CTL0,
                /// Opaque CTL1 register
                pub ctl1: CTL1,
                $(
                    /// Pin
                    pub $pxi: $PXi<$MODE>,
                )+
            }

            impl GpioExt for $GPIOX {
                type Parts = Parts;

                fn split(self, apb: &mut APB2) -> Parts {
                    $GPIOX::enable(apb);
                    $GPIOX::reset(apb);

                    Parts {
                        ctl0: CTL0 { _0: () },
                        ctl1: CTL1 { _0: () },
                        $(
                            $pxi: $PXi { _mode: PhantomData },
                        )+
                    }
                }
            }

            /// Opaque CTL0 register
            pub struct CTL0 {
                _0: (),
            }

            impl CTL0 {
                #[allow(dead_code)]
                pub(crate) fn ctl(&mut self) -> &$gpioy::CTL0 {
                    unsafe { &(*$GPIOX::ptr()).ctl0 }
                }
            }

            /// Opaque CTL1 register
            pub struct CTL1 {
                _0: (),
            }

            impl CTL1 {
                pub(crate) fn ctl(&mut self) -> &$gpioy::CTL1 {
                    unsafe { &(*$GPIOX::ptr()).ctl1 }
                }
            }

            /// Partially erased pin. Only used in the Pxx enum
            pub struct Generic<MODE> {
                i: u8,
                _mode: PhantomData<MODE>,
            }

            impl<MODE> Generic<MODE> {
                pub fn downgrade(self) -> Pxx<MODE> {
                    Pxx::$PXx(self)
                }
            }

            impl<MODE> OutputPin for Generic<Output<MODE>> {
                type Error = Void;
                fn set_high(&mut self) -> Result<(), Self::Error> {
                    // NOTE(unsafe) atomic write to a stateless register
                    Ok(unsafe { (*$GPIOX::ptr()).bop.write(|w| w.bits(1 << self.i)) })
                }

                fn set_low(&mut self) -> Result<(), Self::Error> {
                    // NOTE(unsafe) atomic write to a stateless register
                    Ok(unsafe { (*$GPIOX::ptr()).bop.write(|w| w.bits(1 << (16 + self.i))) })
                }
            }

            impl<MODE> InputPin for Generic<Input<MODE>> {
                type Error = Void;
                fn is_high(&self) -> Result<bool, Self::Error> {
                    self.is_low().map(|b| !b)
                }

                fn is_low(&self) -> Result<bool, Self::Error> {
                    // NOTE(unsafe) atomic read with no side effects
                    Ok(unsafe { (*$GPIOX::ptr()).istat.read().bits() & (1 << self.i) == 0 })
                }
            }


            impl<MODE> StatefulOutputPin for Generic<Output<MODE>> {
                fn is_set_high(&self) -> Result<bool, Self::Error> {
                    self.is_set_low().map(|b| !b)
                }

                fn is_set_low(&self) -> Result<bool, Self::Error> {
                    // NOTE(unsafe) atomic read with no side effects
                    Ok(unsafe { (*$GPIOX::ptr()).octl.read().bits() & (1 << self.i) == 0 })
                }
            }

            impl<MODE> toggleable::Default for Generic<Output<MODE>> {}

            impl InputPin for Generic<Output<OpenDrain>> {
                type Error = Void;
                fn is_high(&self) -> Result<bool, Self::Error> {
                    self.is_low().map(|b| !b)
                }

                fn is_low(&self) -> Result<bool, Self::Error> {
                    // NOTE(unsafe) atomic read with no side effects
                    Ok(unsafe { (*$GPIOX::ptr()).istat.read().bits() & (1 << self.i) == 0 })
                }
            }

            pub type $PXx<MODE> = Pxx<MODE>;



            $(
                /// Pin
                pub struct $PXi<MODE> {
                    _mode: PhantomData<MODE>,
                }

                impl $PXi<Debugger> {
                    /// Put the pin in an active state. The caller
                    /// must enforce that the pin is really in this
                    /// state in the hardware.
                    #[allow(dead_code)]
                    pub(crate) unsafe fn activate(self) -> $PXi<Input<Floating>> {
                        $PXi { _mode: PhantomData }
                    }
                }

                impl<MODE> $PXi<MODE> where MODE: Active {
                    /// Configures the pin to operate as an alternate function push-pull output
                    /// pin.
                    pub fn into_alternate_push_pull(
                        self,
                        ctl: &mut $CTL,
                    ) -> $PXi<Alternate<PushPull>> {
                        const OFFSET: u32 = (4 * $i) % 32;
                        // Alternate function output push pull
                        const CNF: u32 = 0b10;
                        // Output mode, max speed 50 MHz
                        const MODE: u32 = 0b11;
                        const BITS: u32 = (CNF << 2) | MODE;

                        // input mode
                        ctl
                            .ctl()
                            .modify(|r, w| unsafe {
                                w.bits((r.bits() & !(0b1111 << OFFSET)) | (BITS << OFFSET))
                            });

                        $PXi { _mode: PhantomData }
                    }

                    /// Configures the pin to operate as an alternate function open-drain output
                    /// pin.
                    pub fn into_alternate_open_drain(
                        self,
                        ctl: &mut $CTL,
                    ) -> $PXi<Alternate<OpenDrain>> {
                        const OFFSET: u32 = (4 * $i) % 32;
                        // Alternate function output open drain
                        const CNF: u32 = 0b11;
                        // Output mode, max speed 50 MHz
                        const MODE: u32 = 0b11;
                        const BITS: u32 = (CNF << 2) | MODE;

                        // input mode
                        ctl
                            .ctl()
                            .modify(|r, w| unsafe {
                                w.bits((r.bits() & !(0b1111 << OFFSET)) | (BITS << OFFSET))
                            });

                        $PXi { _mode: PhantomData }
                    }

                    /// Configures the pin to operate as a floating input pin
                    pub fn into_floating_input(
                        self,
                        ctl: &mut $CTL
                    ) -> $PXi<Input<Floating>> {
                        const OFFSET: u32 = (4 * $i) % 32;
                        // Floating input
                        const CNF: u32 = 0b01;
                        // Input mode
                        const MODE: u32 = 0b00;
                        const BITS: u32 = (CNF << 2) | MODE;

                        // input mode
                        ctl
                            .ctl()
                            .modify(|r, w| unsafe {
                                w.bits((r.bits() & !(0b1111 << OFFSET)) | (BITS << OFFSET))
                            });

                        $PXi { _mode: PhantomData }
                    }

                    /// Configures the pin to operate as a pulled down input pin
                    pub fn into_pull_down_input(
                        self,
                        ctl: &mut $CTL,
                    ) -> $PXi<Input<PullDown>> {
                        const OFFSET: u32 = (4 * $i) % 32;
                        // Pull up/down input
                        const CNF: u32 = 0b10;
                        // Input mode
                        const MODE: u32 = 0b00;
                        const BITS: u32 = (CNF << 2) | MODE;

                        //pull down:
                        // NOTE(unsafe) atomic write to a stateless register
                        unsafe { (*$GPIOX::ptr()).bop.write(|w| w.bits(1 << (16 + $i))) }

                        // input mode
                        ctl
                            .ctl()
                            .modify(|r, w| unsafe {
                                w.bits((r.bits() & !(0b1111 << OFFSET)) | (BITS << OFFSET))
                            });

                        $PXi { _mode: PhantomData }
                    }

                    /// Configures the pin to operate as a pulled up input pin
                    pub fn into_pull_up_input(
                        self,
                        ctl: &mut $CTL,
                    ) -> $PXi<Input<PullUp>> {
                        const OFFSET: u32 = (4 * $i) % 32;
                        // Pull up/down input
                        const CNF: u32 = 0b10;
                        // Input mode
                        const MODE: u32 = 0b00;
                        const BITS: u32 = (CNF << 2) | MODE;

                        //pull up:
                        // NOTE(unsafe) atomic write to a stateless register
                        unsafe { (*$GPIOX::ptr()).bop.write(|w| w.bits(1 << $i)) }

                        // input mode
                        ctl
                            .ctl()
                            .modify(|r, w| unsafe {
                                w.bits((r.bits() & !(0b1111 << OFFSET)) | (BITS << OFFSET))
                            });

                        $PXi { _mode: PhantomData }
                    }

                    /// Configures the pin to operate as an open-drain output pin.
                    /// Initial state will be low.
                    pub fn into_open_drain_output(
                        self,
                        ctl: &mut $CTL,
                    ) -> $PXi<Output<OpenDrain>> {
                        self.into_open_drain_output_with_state(ctl, State::Low)
                    }

                    /// Configures the pin to operate as an open-drain output pin.
                    /// `initial_state` specifies whether the pin should be initially high or low.
                    pub fn into_open_drain_output_with_state(
                        self,
                        ctl: &mut $CTL,
                        initial_state: State,
                    ) -> $PXi<Output<OpenDrain>> {
                        const OFFSET: u32 = (4 * $i) % 32;
                        // General purpose output open-drain
                        const CNF: u32 = 0b01;
                        // Open-Drain Output mode, max speed 50 MHz
                        const MODE: u32 = 0b11;
                        const BITS: u32 = (CNF << 2) | MODE;

                        let mut res = $PXi { _mode: PhantomData };

                        match initial_state {
                            State::High => res.set_high(),
                            State::Low  => res.set_low(),
                        }.unwrap();

                        ctl
                            .ctl()
                            .modify(|r, w| unsafe {
                                w.bits((r.bits() & !(0b1111 << OFFSET)) | (BITS << OFFSET))
                            });

                        res
                    }

                    /// Configures the pin to operate as an push-pull output pin.
                    /// Initial state will be low.
                    pub fn into_push_pull_output(
                        self,
                        ctl: &mut $CTL,
                    ) -> $PXi<Output<PushPull>> {
                        self.into_push_pull_output_with_state(ctl, State::Low)
                    }

                    /// Configures the pin to operate as an push-pull output pin.
                    /// `initial_state` specifies whether the pin should be initially high or low.
                    pub fn into_push_pull_output_with_state(
                        self,
                        ctl: &mut $CTL,
                        initial_state: State,
                    ) -> $PXi<Output<PushPull>> {
                        const OFFSET: u32 = (4 * $i) % 32;
                        // General purpose output push-pull
                        const CNF: u32 = 0b00;
                        // Output mode, max speed 50 MHz
                        const MODE: u32 = 0b11;
                        const BITS: u32 = (CNF << 2) | MODE;

                        let mut res = $PXi { _mode: PhantomData };

                        match initial_state {
                            State::High => res.set_high(),
                            State::Low  => res.set_low(),
                        }.unwrap();

                        ctl
                            .ctl()
                            .modify(|r, w| unsafe {
                                w.bits((r.bits() & !(0b1111 << OFFSET)) | (BITS << OFFSET))
                            });

                        res
                    }


                    /// Configures the pin to operate as an analog input pin
                    pub fn into_analog(self, ctl: &mut $CTL) -> $PXi<Analog> {
                        const OFFSET: u32 = (4 * $i) % 32;
                        // Analog input
                        const CNF: u32 = 0b00;
                        // Input mode
                        const MODE: u32 = 0b00;
                        const BITS: u32 = (CNF << 2) | MODE;

                        // analog mode
                        ctl
                            .ctl()
                            .modify(|r, w| unsafe {
                                w.bits((r.bits() & !(0b1111 << OFFSET)) | (BITS << OFFSET))
                            });

                        $PXi { _mode: PhantomData }
                    }
                }

                impl<MODE> $PXi<MODE> where MODE: Active {
                    /// Erases the pin number from the type
                    fn into_generic(self) -> Generic<MODE> {
                        Generic {
                            i: $i,
                            _mode: self._mode,
                        }
                    }

                    /// Erases the pin number and port from the type
                    ///
                    /// This is useful when you want to collect the pins into an array where you
                    /// need all the elements to have the same type
                    pub fn downgrade(self) -> Pxx<MODE> {
                        self.into_generic().downgrade()
                    }
                }

                impl<MODE> OutputPin for $PXi<Output<MODE>> {
                    type Error = Void;
                    fn set_high(&mut self) -> Result<(), Self::Error> {
                        // NOTE(unsafe) atomic write to a stateless register
                        Ok(unsafe { (*$GPIOX::ptr()).bop.write(|w| w.bits(1 << $i)) })
                    }

                    fn set_low(&mut self) -> Result<(), Self::Error> {
                        // NOTE(unsafe) atomic write to a stateless register
                        Ok(unsafe { (*$GPIOX::ptr()).bop.write(|w| w.bits(1 << (16 + $i))) })
                    }
                }

                impl<MODE> StatefulOutputPin for $PXi<Output<MODE>> {
                    fn is_set_high(&self) -> Result<bool, Self::Error> {
                        self.is_set_low().map(|b| !b)
                    }

                    fn is_set_low(&self) -> Result<bool, Self::Error> {
                        // NOTE(unsafe) atomic read with no side effects
                        Ok(unsafe { (*$GPIOX::ptr()).octl.read().bits() & (1 << $i) == 0 })
                    }
                }

                impl<MODE> toggleable::Default for $PXi<Output<MODE>> {}

                impl<MODE> InputPin for $PXi<Input<MODE>> {
                    type Error = Void;
                    fn is_high(&self) -> Result<bool, Self::Error> {
                        self.is_low().map(|b| !b)
                    }

                    fn is_low(&self) -> Result<bool, Self::Error> {
                        // NOTE(unsafe) atomic read with no side effects
                        Ok(unsafe { (*$GPIOX::ptr()).istat.read().bits() & (1 << $i) == 0 })
                    }
                }

                impl InputPin for $PXi<Output<OpenDrain>> {
                    type Error = Void;
                    fn is_high(&self) -> Result<bool, Self::Error> {
                        self.is_low().map(|b| !b)
                    }

                    fn is_low(&self) -> Result<bool, Self::Error> {
                        // NOTE(unsafe) atomic read with no side effects
                        Ok(unsafe { (*$GPIOX::ptr()).istat.read().bits() & (1 << $i) == 0 })
                    }
                }
            )+
        }
    }
}

macro_rules! impl_pxx {
    ($(($port:ident :: $pin:ident)),*) => {
        use void::Void;
        use embedded_hal::digital::v2::{InputPin, StatefulOutputPin, OutputPin};

        pub enum Pxx<MODE> {
            $(
                $pin($port::Generic<MODE>)
            ),*
        }

        impl<MODE> OutputPin for Pxx<Output<MODE>> {
            type Error = Void;
            fn set_high(&mut self) -> Result<(), Void> {
                match self {
                    $(Pxx::$pin(pin) => pin.set_high()),*
                }
            }

            fn set_low(&mut self) -> Result<(), Void> {
                match self {
                    $(Pxx::$pin(pin) => pin.set_low()),*
                }
            }
        }

        impl<MODE> StatefulOutputPin for Pxx<Output<MODE>> {
            fn is_set_high(&self) -> Result<bool, Self::Error> {
                match self {
                    $(Pxx::$pin(pin) => pin.is_set_high()),*
                }
            }

            fn is_set_low(&self) -> Result<bool, Self::Error> {
                match self {
                    $(Pxx::$pin(pin) => pin.is_set_low()),*
                }
            }
        }

        impl<MODE> InputPin for Pxx<Input<MODE>> {
            type Error = Void;
            fn is_high(&self) -> Result<bool, Void> {
                match self {
                    $(Pxx::$pin(pin) => pin.is_high()),*
                }
            }

            fn is_low(&self) -> Result<bool, Void> {
                match self {
                    $(Pxx::$pin(pin) => pin.is_low()),*
                }
            }
        }
    }
}

impl_pxx! {
    (gpioa::PAx),
    (gpiob::PBx),
    (gpioc::PCx),
    (gpiod::PDx),
    (gpioe::PEx)
}

gpio!(GPIOA, gpioa, gpioa, PAx, [
    PA0: (pa0, 0, Input<Floating>, CTL0),
    PA1: (pa1, 1, Input<Floating>, CTL0),
    PA2: (pa2, 2, Input<Floating>, CTL0),
    PA3: (pa3, 3, Input<Floating>, CTL0),
    PA4: (pa4, 4, Input<Floating>, CTL0),
    PA5: (pa5, 5, Input<Floating>, CTL0),
    PA6: (pa6, 6, Input<Floating>, CTL0),
    PA7: (pa7, 7, Input<Floating>, CTL0),
    PA8: (pa8, 8, Input<Floating>, CTL1),
    PA9: (pa9, 9, Input<Floating>, CTL1),
    PA10: (pa10, 10, Input<Floating>, CTL1),
    PA11: (pa11, 11, Input<Floating>, CTL1),
    PA12: (pa12, 12, Input<Floating>, CTL1),
    PA13: (pa13, 13, Debugger, CTL1),
    PA14: (pa14, 14, Debugger, CTL1),
    PA15: (pa15, 15, Debugger, CTL1),
]);

gpio!(GPIOB, gpiob, gpioa, PBx, [
    PB0: (pb0, 0, Input<Floating>, CTL0),
    PB1: (pb1, 1, Input<Floating>, CTL0),
    PB2: (pb2, 2, Input<Floating>, CTL0),
    PB3: (pb3, 3, Debugger, CTL0),
    PB4: (pb4, 4, Debugger, CTL0),
    PB5: (pb5, 5, Input<Floating>, CTL0),
    PB6: (pb6, 6, Input<Floating>, CTL0),
    PB7: (pb7, 7, Input<Floating>, CTL0),
    PB8: (pb8, 8, Input<Floating>, CTL1),
    PB9: (pb9, 9, Input<Floating>, CTL1),
    PB10: (pb10, 10, Input<Floating>, CTL1),
    PB11: (pb11, 11, Input<Floating>, CTL1),
    PB12: (pb12, 12, Input<Floating>, CTL1),
    PB13: (pb13, 13, Input<Floating>, CTL1),
    PB14: (pb14, 14, Input<Floating>, CTL1),
    PB15: (pb15, 15, Input<Floating>, CTL1),
]);

gpio!(GPIOC, gpioc, gpioa, PCx, [
    PC0: (pc0, 0, Input<Floating>, CTL0),
    PC1: (pc1, 1, Input<Floating>, CTL0),
    PC2: (pc2, 2, Input<Floating>, CTL0),
    PC3: (pc3, 3, Input<Floating>, CTL0),
    PC4: (pc4, 4, Input<Floating>, CTL0),
    PC5: (pc5, 5, Input<Floating>, CTL0),
    PC6: (pc6, 6, Input<Floating>, CTL0),
    PC7: (pc7, 7, Input<Floating>, CTL0),
    PC8: (pc8, 8, Input<Floating>, CTL1),
    PC9: (pc9, 9, Input<Floating>, CTL1),
    PC10: (pc10, 10, Input<Floating>, CTL1),
    PC11: (pc11, 11, Input<Floating>, CTL1),
    PC12: (pc12, 12, Input<Floating>, CTL1),
    PC13: (pc13, 13, Input<Floating>, CTL1),
    PC14: (pc14, 14, Input<Floating>, CTL1),
    PC15: (pc15, 15, Input<Floating>, CTL1),
]);

gpio!(GPIOD, gpiod, gpioa, PDx, [
    PD0: (pd0, 0, Input<Floating>, CTL0),
    PD1: (pd1, 1, Input<Floating>, CTL0),
    PD2: (pd2, 2, Input<Floating>, CTL0),
    PD3: (pd3, 3, Input<Floating>, CTL0),
    PD4: (pd4, 4, Input<Floating>, CTL0),
    PD5: (pd5, 5, Input<Floating>, CTL0),
    PD6: (pd6, 6, Input<Floating>, CTL0),
    PD7: (pd7, 7, Input<Floating>, CTL0),
    PD8: (pd8, 8, Input<Floating>, CTL1),
    PD9: (pd9, 9, Input<Floating>, CTL1),
    PD10: (pd10, 10, Input<Floating>, CTL1),
    PD11: (pd11, 11, Input<Floating>, CTL1),
    PD12: (pd12, 12, Input<Floating>, CTL1),
    PD13: (pd13, 13, Input<Floating>, CTL1),
    PD14: (pd14, 14, Input<Floating>, CTL1),
    PD15: (pd15, 15, Input<Floating>, CTL1),
]);

gpio!(GPIOE, gpioe, gpioa, PEx, [
    PE0: (pe0, 0, Input<Floating>, CTL0),
    PE1: (pe1, 1, Input<Floating>, CTL0),
    PE2: (pe2, 2, Input<Floating>, CTL0),
    PE3: (pe3, 3, Input<Floating>, CTL0),
    PE4: (pe4, 4, Input<Floating>, CTL0),
    PE5: (pe5, 5, Input<Floating>, CTL0),
    PE6: (pe6, 6, Input<Floating>, CTL0),
    PE7: (pe7, 7, Input<Floating>, CTL0),
    PE8: (pe8, 8, Input<Floating>, CTL1),
    PE9: (pe9, 9, Input<Floating>, CTL1),
    PE10: (pe10, 10, Input<Floating>, CTL1),
    PE11: (pe11, 11, Input<Floating>, CTL1),
    PE12: (pe12, 12, Input<Floating>, CTL1),
    PE13: (pe13, 13, Input<Floating>, CTL1),
    PE14: (pe14, 14, Input<Floating>, CTL1),
    PE15: (pe15, 15, Input<Floating>, CTL1),
]);
