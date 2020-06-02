use crate::pac::ECLIC;
use riscv::interrupt::Nr;

const EFFECTIVE_BITS: u8 = 4;

#[repr(u8)]
#[derive(Debug)]
pub enum LevelPriorityBits {
    Level0Priority4 = 0,
    Level1Priority3 = 1,
    Level2Priority2 = 2,
    Level3Priority1 = 3,
    Level4Priority0 = 4,
}

#[repr(u8)]
#[derive(Debug)]
pub enum TriggerType {
    Level = 0,
    RisingEdge = 1,
    FallingEdge = 3,
}

pub trait EclicExt {
    fn setup();

    /// Set interrupt threshold level
    fn set_threshold_level(level: u8);

    fn set_level_priority_bits(lp: LevelPriorityBits);

    /// Enable `interrupt`
    unsafe fn unmask<I: Nr>(interrupt: I);

    /// Disables `interrupt`
    fn mask<I: Nr>(interrupt: I);

    /// Checks if `interrupt` is enabled
    fn is_enabled<I: Nr>(interrupt: I) -> bool;

    /// Forces `interrupt` into pending state
    fn pend<I: Nr>(interrupt: I);

    /// Clears `interrupt`'s pending state
    fn unpend<I: Nr>(interrupt: I);

    /// Checks if `interrupt` is pending
    fn is_pending<I: Nr>(interrupt: I) -> bool;

    /// Set `interrupt` trigger type
    fn set_trigger_type<I: Nr>(interrupt: I, tt: TriggerType);

    /// Get `interrupt` trigger type
    fn get_trigger_type<I: Nr>(interrupt: I) -> TriggerType;
}

impl EclicExt for ECLIC {
    #[inline]
    fn setup() {
        let eclic = unsafe { &*Self::ptr() };

        eclic.cliccfg.write(|w| unsafe { w.bits(0) });
        eclic.mth.write(|w| unsafe { w.bits(0) });

        for nr in 0..eclic.clicinfo.read().num_interrupt().bits() as usize {
            eclic.clicints[nr].clicintip.write(|w| unsafe { w.bits(0) });
            eclic.clicints[nr].clicintie.write(|w| unsafe { w.bits(0) });
            eclic.clicints[nr]
                .clicintattr
                .write(|w| unsafe { w.bits(0) });
            eclic.clicints[nr]
                .clicintctl
                .write(|w| unsafe { w.bits(0) });
        }
    }

    #[inline]
    fn set_threshold_level(level: u8) {
        unsafe { (*Self::ptr()).mth.write(|w| w.mth().bits(level)) }
    }

    #[inline]
    fn set_level_priority_bits(lp: LevelPriorityBits) {
        unsafe { (*Self::ptr()).cliccfg.write(|w| w.nlbits().bits(lp as u8)) }
    }

    #[inline]
    unsafe fn unmask<I: Nr>(interrupt: I) {
        let nr = usize::from(interrupt.nr());

        (*Self::ptr()).clicints[nr]
            .clicintie
            .write(|w| w.ie().set_bit())
    }

    #[inline]
    fn mask<I: Nr>(interrupt: I) {
        let nr = usize::from(interrupt.nr());

        unsafe {
            (*Self::ptr()).clicints[nr]
                .clicintie
                .write(|w| w.ie().clear_bit())
        }
    }

    #[inline]
    fn is_enabled<I: Nr>(interrupt: I) -> bool {
        let nr = usize::from(interrupt.nr());

        unsafe {
            (*Self::ptr()).clicints[nr]
                .clicintie
                .read()
                .ie()
                .bit_is_set()
        }
    }

    #[inline]
    fn pend<I: Nr>(interrupt: I) {
        let nr = usize::from(interrupt.nr());

        unsafe {
            (*Self::ptr()).clicints[nr]
                .clicintip
                .write(|w| w.ip().set_bit())
        }
    }

    #[inline]
    fn unpend<I: Nr>(interrupt: I) {
        let nr = usize::from(interrupt.nr());

        unsafe {
            (*Self::ptr()).clicints[nr]
                .clicintip
                .write(|w| w.ip().clear_bit())
        }
    }

    #[inline]
    fn is_pending<I: Nr>(interrupt: I) -> bool {
        let nr = usize::from(interrupt.nr());

        unsafe {
            (*Self::ptr()).clicints[nr]
                .clicintip
                .read()
                .ip()
                .bit_is_set()
        }
    }

    #[inline]
    fn set_trigger_type<I: Nr>(interrupt: I, tt: TriggerType) {
        let nr = usize::from(interrupt.nr());

        unsafe {
            (*Self::ptr()).clicints[nr]
                .clicintattr
                .write(|w| w.trig().bits(tt as u8).shv().clear_bit())
        }
    }

    #[inline]
    fn get_trigger_type<I: Nr>(interrupt: I) -> TriggerType {
        let nr = usize::from(interrupt.nr());

        match unsafe { (*Self::ptr()).clicints[nr].clicintattr.read().trig().bits() } {
            0 => TriggerType::Level,
            1 => TriggerType::RisingEdge,
            3 => TriggerType::FallingEdge,
            _ => panic!("invalid trigger type"),
        }
    }
}
