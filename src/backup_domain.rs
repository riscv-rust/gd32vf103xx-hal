/*!
  Registers that are not reset as long as Vbat or Vdd has power.

  The registers retain their values during wakes from standby mode or system resets. They also
  retain their value when Vdd is switched off as long as V_BAT is powered.
  The backup domain also contains tamper protection and writes to it must be enabled in order
  to use the real time clock (RTC).
  Write access to the backup domain is enabled using the `BKP::configure(rcu, &mut pmu)`
  function.
*/

use crate::pac::{rcu, BKP, PMU};
use crate::rcu::{Rcu, Enable};

/// Extension trait that sets up the `BKP` peripheral
pub trait BkpExt {
    /// Configure the `BKP` peripheral
    fn configure(self, rcu: &mut Rcu, pmu: &mut PMU) -> BackupDomain;
}

impl BkpExt for BKP {
    fn configure(self, rcu: &mut Rcu, pmu: &mut PMU) -> BackupDomain {
        // Enable the backup interface
        BKP::enable(rcu);
        PMU::enable(rcu);

        // Enable access to the backup registers
        pmu.ctl.modify(|_r, w| w.bkpwen().set_bit());
        BackupDomain {
            _regs: self
        }
    }
}

/**
  The existence of this struct indicates that writing to the the backup
  domain has been enabled. It is acquired by calling `configure` on `BKP`
*/
pub struct BackupDomain {
    pub(crate) _regs: BKP,
}

/// This marks that the LXTAL clock source has been configured and has stabilized.
#[derive(Clone, Copy)]
pub struct Lxtal(
    /// Non-public field to stop user from instantiating this type
    (),
);

impl Lxtal {
    /// Enable and don't wait for stabilization.
    fn just_enable(rcu: &rcu::RegisterBlock) {
        // Enable LXTAL
        rcu.bdctl
        .modify(|_, w| w.lxtalen().set_bit().lxtalbps().clear_bit());
    }
    fn is_stable(rcu: &rcu::RegisterBlock) -> bool {
        rcu.bdctl.read().lxtalstb().bit()
    }
    /// Enable the clock and block until stabilized.
    pub fn enable_block(rcu: &Rcu) -> Self {
        //let rcu = unsafe { &*RCU::ptr() };
        let rcu = &rcu.regs;
        Self::just_enable(rcu);
        // Wait for stable LXTAL
        while !Self::is_stable(rcu) {}
        Self(())
    }
}
