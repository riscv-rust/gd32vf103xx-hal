use crate::pac::ECLIC;
use riscv::interrupt::Nr;

const EFFECTIVE_LEVEL_PRIORITY_BITS: u8 = 4;

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
    /// Reset all ECLIC registers to 0
    fn reset();

    /// Set interrupts threshold level
    fn set_threshold_level(level: u8);

    /// Get interrupts threshold level
    fn get_threshold_level() -> u8;

    fn set_level_priority_bits(lp: LevelPriorityBits);

    fn get_level_priority_bits() -> Option<LevelPriorityBits>;

    /// Get number of bits designated for interrupt level
    fn get_level_bits() -> u8;

    /// Get number of bits designated for interrupt priority
    fn get_priority_bits() -> u8;

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
    fn get_trigger_type<I: Nr>(interrupt: I) -> Option<TriggerType>;

    // Set `interrupt` level
    fn set_level<I: Nr>(interrupt: I, level: u8);

    // Get `interrupt` level
    fn get_level<I: Nr>(interrupt: I) -> u8;

    // Set `interrupt` priority
    fn set_priority<I: Nr>(interrupt: I, priority: u8);

    // Get `interrupt` interrupt
    fn get_priority<I: Nr>(interrupt: I) -> u8;
}

impl EclicExt for ECLIC {
    fn reset() {
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
    fn get_threshold_level() -> u8 {
        unsafe { (*Self::ptr()).mth.read().mth().bits() }
    }

    #[inline]
    fn set_level_priority_bits(lp: LevelPriorityBits) {
        unsafe { (*Self::ptr()).cliccfg.write(|w| w.nlbits().bits(lp as u8)) }
    }

    #[inline]
    fn get_level_priority_bits() -> Option<LevelPriorityBits> {
        match unsafe { (*Self::ptr()).cliccfg.read().nlbits().bits() } {
            0 => Some(LevelPriorityBits::Level0Priority4),
            1 => Some(LevelPriorityBits::Level1Priority3),
            2 => Some(LevelPriorityBits::Level2Priority2),
            3 => Some(LevelPriorityBits::Level3Priority1),
            4 => Some(LevelPriorityBits::Level4Priority0),
            _ => None,
        }
    }

    #[inline]
    fn get_level_bits() -> u8 {
        let bits = unsafe { (*Self::ptr()).cliccfg.read().nlbits().bits() };

        core::cmp::min(bits, EFFECTIVE_LEVEL_PRIORITY_BITS)
    }

    #[inline]
    fn get_priority_bits() -> u8 {
        EFFECTIVE_LEVEL_PRIORITY_BITS - Self::get_level_bits()
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
    fn get_trigger_type<I: Nr>(interrupt: I) -> Option<TriggerType> {
        let nr = usize::from(interrupt.nr());

        match unsafe { (*Self::ptr()).clicints[nr].clicintattr.read().trig().bits() } {
            0 => Some(TriggerType::Level),
            1 => Some(TriggerType::RisingEdge),
            3 => Some(TriggerType::FallingEdge),
            _ => None,
        }
    }

    #[inline]
    fn set_level<I: Nr>(interrupt: I, level: u8) {
        let level_bits = Self::get_level_bits();

        assert!(level <= (1 << level) - 1);

        let nr = usize::from(interrupt.nr());

        let mut intctl = unsafe {
            (*Self::ptr()).clicints[nr]
                .clicintctl
                .read()
                .level_priority()
                .bits()
        };

        intctl <<= level_bits;
        intctl >>= level_bits;

        let level = level << (8 - level_bits);

        unsafe {
            (*Self::ptr()).clicints[nr]
                .clicintctl
                .write(|w| w.level_priority().bits(intctl | level))
        }
    }

    #[inline]
    fn get_level<I: Nr>(interrupt: I) -> u8 {
        let nr = usize::from(interrupt.nr());

        let intctl = unsafe {
            (*Self::ptr()).clicints[nr]
                .clicintctl
                .read()
                .level_priority()
                .bits()
        };

        intctl >> (8 - Self::get_level_bits())
    }

    #[inline]
    fn set_priority<I: Nr>(interrupt: I, priority: u8) {
        let level_bits = Self::get_level_bits();

        assert!(priority <= (1 << (EFFECTIVE_LEVEL_PRIORITY_BITS - level_bits)) - 1);

        let nr = usize::from(interrupt.nr());

        let mut intctl = unsafe {
            (*Self::ptr()).clicints[nr]
                .clicintctl
                .read()
                .level_priority()
                .bits()
        };

        intctl >>= 8 - level_bits;
        intctl <<= 8 - level_bits;

        let priority = priority << (8 - EFFECTIVE_LEVEL_PRIORITY_BITS);

        unsafe {
            (*Self::ptr()).clicints[nr]
                .clicintctl
                .write(|w| w.level_priority().bits(intctl | priority))
        }
    }

    #[inline]
    fn get_priority<I: Nr>(interrupt: I) -> u8 {
        let nr = usize::from(interrupt.nr());

        let intctl = unsafe {
            (*Self::ptr()).clicints[nr]
                .clicintctl
                .read()
                .level_priority()
                .bits()
        };

        let level_bits = Self::get_level_bits();

        (intctl << level_bits) >> (level_bits + (8 - EFFECTIVE_LEVEL_PRIORITY_BITS))
    }
}
