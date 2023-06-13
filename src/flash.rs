//! Flash memory

use crate::pac::{fmc, FMC};

/// Flash statiing address
pub const FLASH_START: u32 = 0x0800_0000;
/// Flash end address
pub const FLASH_END: u32 = 0x0801_FFFF;

const _RDPRT_KEY: u16 = 0x00A5;
const KEY1: u32 = 0x45670123;
const KEY2: u32 = 0xCDEF89AB;

const SZ_1K: u32 = 1024;

/// Flash operation result
pub type Result<T> = core::result::Result<T, Error>;

/// Flash error
#[derive(Debug, Clone, Copy, PartialEq, Eq, Ord, PartialOrd)]
#[non_exhaustive]
pub enum Error {
    /// Address is out of flash memory area
    AddressLargerThanFlash,
    /// Address is not aligned to multiple of two
    AddressMisaligned,
    /// Length is not a multiple of two
    LengthNotMultiple2,
    /// End address is out of flash memory area
    LengthTooLong,
    /// Flash erasing failed
    EraseError,
    /// Flash programming failed
    ///
    /// When program to the flash while it is not 0xFFFF.
    ProgrammingError,
    /// Flash writing failed
    ///
    /// When erase/program on protected pages.
    WriteError,
    /// Flash data verification failed
    ///
    /// When flash content is not equals to expected.
    VerifyError,
    /// Flash unlocking failed
    UnlockError,
    /// Flash locking failed
    LockError,
}

/// Flash size specifier
#[derive(Debug, Clone, Copy, PartialEq, Eq, Ord, PartialOrd)]
#[repr(u8)]
pub enum FlashSize {
    Sz16K = 16,
    Sz32K = 32,
    Sz64K = 64,
    Sz128K = 128,
}

impl FlashSize {
    pub const fn bytes(self) -> u32 {
        SZ_1K * self as u32
    }
}

pub struct FlashWriter<'a> {
    fmc: &'a mut Parts,
    flash_sz: FlashSize,
    verify: bool,
}

impl<'a> FlashWriter<'a> {
    fn busy_wait(&mut self) {
        // Wait for any ongoing operations
        while self.fmc.stat.stat().read().busy().bit_is_set() {}
    }

    fn is_locked(&mut self) -> bool {
        self.fmc.ctl.ctl().read().lk().bit_is_set()
    }

    fn unlock(&mut self) -> Result<()> {
        if !self.is_locked() {
            return Ok(());
        }

        self.busy_wait();

        // NOTE(unsafe) write Keys to the key register. This is safe because the
        // only side effect of these writes is to unlock the fmc control
        // register, which is the intent of this function. Do not rearrange the
        // order of these writes or the control register will be permanently
        // locked out until reset.
        self.fmc.key.key().write(|w| unsafe { w.key().bits(KEY1) });
        self.fmc.key.key().write(|w| unsafe { w.key().bits(KEY2) });

        // Verify success
        if self.is_locked() {
            Err(Error::UnlockError)
        } else {
            Ok(())
        }
    }

    fn lock(&mut self) -> Result<()> {
        if self.is_locked() {
            return Ok(());
        }

        self.busy_wait();

        // Set lock bit
        self.fmc.ctl.ctl().modify(|_, w| w.lk().set_bit());

        // Verify success
        if self.is_locked() {
            Ok(())
        } else {
            Err(Error::LockError)
        }
    }

    fn valid_address(&self, offset: u32) -> Result<()> {
        if FLASH_START + offset > FLASH_END {
            Err(Error::AddressLargerThanFlash)
        } else if offset & 0x1 != 0 {
            Err(Error::AddressMisaligned)
        } else {
            Ok(())
        }
    }

    fn valid_length(&self, offset: u32, length: usize) -> Result<()> {
        if offset + length as u32 > self.flash_sz.bytes() as u32 {
            Err(Error::LengthTooLong)
        } else if length & 0x1 != 0 {
            Err(Error::LengthNotMultiple2)
        } else {
            Ok(())
        }
    }

    /// Erase sector which contains `start_offset`
    pub fn page_erase(&mut self, start_offset: u32) -> Result<()> {
        self.valid_address(start_offset)?;

        // Unlock Flash
        self.unlock()?;

        // Set Page Erase
        self.fmc.ctl.ctl().modify(|_, w| w.per().set_bit());

        // Write address bits
        // NOTE(unsafe) This sets the page address in the Address Register.
        // The side-effect of this write is that the page will be erased when we
        // set the STRT bit in the CTL below. The address is validated by the
        // call to self.valid_address() above.
        self.fmc
            .addr
            .addr()
            .write(|w| unsafe { w.addr().bits(FLASH_START + start_offset) });

        // Start Operation
        self.fmc.ctl.ctl().modify(|_, w| w.start().set_bit());

        // Wait for operation to finish
        self.busy_wait();

        // Check for errors
        let stat = self.fmc.stat.stat().read();

        // Remove Page Erase Operation bit
        self.fmc.ctl.ctl().modify(|_, w| w.per().clear_bit());

        // Re-lock flash
        self.lock()?;

        if stat.wperr().bit_is_set() {
            self.fmc.stat.stat().modify(|_, w| w.wperr().set_bit());
            Err(Error::EraseError)
        } else {
            if !self.verify || self.is_page_erased(start_offset) {
                Ok(())
            } else {
                Err(Error::VerifyError)
            }
        }
    }

    fn is_page_erased(&self, start_offset: u32) -> bool {
        // By subtracting 1 from the sector size and masking with
        // start_offset, we make 'start' point to the beginning of the
        // page. We do this because the entire page should have been
        // erased, regardless of where in the page the given
        // 'start_offset' was.
        let start = start_offset & !(SZ_1K - 1);
        for idx in start..start + SZ_1K {
            let write_address = (FLASH_START + idx as u32) as *const u16;
            let verify: u16 = unsafe { core::ptr::read_volatile(write_address) };
            if verify != 0xFFFF {
                return false;
            }
        }
        true
    }

    /// Erase the Flash Sectors from `FLASH_START + start_offset` to `length`
    pub fn erase_range(&mut self, range: core::ops::Range<u32>) -> Result<()> {
        self.valid_length(range.start, range.len())?;

        // Erase every sector touched by start_offset + length
        for offset in
            range.step_by(SZ_1K as usize)
        {
            self.page_erase(offset)?;
        }

        // Report Success
        Ok(())
    }

    /// Erase the Flash Sectors from `FLASH_START + start_offset` to `length`
    pub fn erase(&mut self, start_offset: u32, length: usize) -> Result<()> {
        self.erase_range(start_offset..start_offset + length as u32)
    }

    /// Retrieve a slice of data from `FLASH_START + offset`
    pub fn read(&self, offset: u32, length: usize) -> Result<&[u8]> {
        self.valid_address(offset)?;

        if offset + length as u32 > self.flash_sz.bytes() as u32 {
            return Err(Error::LengthTooLong);
        }

        let address = (FLASH_START + offset) as *const _;

        Ok(
            // NOTE(unsafe) read with no side effects. The data returned will
            // remain valid for its lifetime because we take an immutable
            // reference to this FlashWriter, and any operation that would
            // invalidate the data returned would first require taking a mutable
            // reference to this FlashWriter.
            unsafe { core::slice::from_raw_parts(address, length) },
        )
    }

    /// Write data to `FLASH_START + offset`
    pub fn write(&mut self, offset: u32, data: &[u8]) -> Result<()> {
        self.valid_length(offset, data.len())?;

        // Unlock Flash
        self.unlock()?;

        let mut result = Ok(());

        for idx in (0..data.len()).step_by(2) {
            self.valid_address(offset + idx as u32)?;

            let write_address = (FLASH_START + offset + idx as u32) as *mut u16;

            // Set Page Programming to 1
            self.fmc.ctl.ctl().modify(|_, w| w.pg().set_bit());

            self.busy_wait();

            // Flash is written 16 bits at a time, so combine two bytes to get a
            // half-word
            let hword: u16 = (data[idx] as u16) | (data[idx + 1] as u16) << 8;

            // NOTE(unsafe) Write to FLASH area with no side effects
            unsafe { core::ptr::write_volatile(write_address, hword) };

            // Wait for write
            self.busy_wait();

            // Set Page Programming to 0
            self.fmc.ctl.ctl().modify(|_, w| w.pg().clear_bit());

            // Check for errors
            if self.fmc.stat.stat().read().pgerr().bit_is_set() {
                self.fmc.stat.stat().modify(|_, w| w.pgerr().set_bit());

                result = Err(Error::ProgrammingError);
                break;
            } else if self.fmc.stat.stat().read().wperr().bit_is_set() {
                self.fmc.stat.stat().modify(|_, w| w.wperr().set_bit());

                result = Err(Error::WriteError);
                break;
            } else if self.verify {
                // Verify written WORD
                // NOTE(unsafe) read with no side effects within FLASH area
                let verify: u16 = unsafe { core::ptr::read_volatile(write_address) };
                if verify != hword {
                    result = Err(Error::VerifyError);
                    break;
                }
            }
        }

        // Lock Flash and report success
        self.lock()?;

        result
    }

    /// Enable/disable verifying that each erase or write operation completed
    /// successfuly.
    ///
    /// When enabled, after each erase operation every address is read to make
    /// sure it contains the erase value of 0xFFFF. After each write operation,
    /// every address written is read and compared to the value that should have
    /// been written. If any address does not contain the expected value, the
    /// function will return Err.
    /// When disabled, no verification is performed, erase/write operations are
    /// assumed to have succeeded.
    pub fn change_verification(&mut self, verify: bool) {
        self.verify = verify;
    }
}

#[cfg(feature = "embedded-storage")]
impl embedded_storage::nor_flash::NorFlashError for Error {
    fn kind(&self) -> embedded_storage::nor_flash::NorFlashErrorKind {
        use embedded_storage::nor_flash::NorFlashErrorKind::*;

        match self {
            Error::AddressLargerThanFlash | Error::LengthTooLong => OutOfBounds,
            Error::AddressMisaligned | Error::LengthNotMultiple2 => NotAligned,
            Error::EraseError | Error::ProgrammingError | Error::WriteError | Error::VerifyError | Error::UnlockError | Error::LockError => Other,
        }
    }
}

#[cfg(feature = "embedded-storage")]
impl<'a> embedded_storage::nor_flash::ErrorType for FlashWriter<'a> {
    type Error = Error;
}

#[cfg(feature = "embedded-storage")]
impl<'a> embedded_storage::nor_flash::ReadNorFlash for FlashWriter<'a> {
    const READ_SIZE: usize = 1;

    fn read(&mut self, offset: u32, bytes: &mut [u8]) -> Result<()> {
        FlashWriter::read(self, offset, bytes.len()).map(|data| {
            bytes.copy_from_slice(data);
        })
    }

    fn capacity(&self) -> usize {
        self.flash_sz.bytes() as _
    }
}

#[cfg(feature = "embedded-storage")]
impl<'a> embedded_storage::nor_flash::NorFlash for FlashWriter<'a> {
    const WRITE_SIZE: usize = 2;
    const ERASE_SIZE: usize = SZ_1K as _;

    fn erase(&mut self, from: u32, to: u32) -> Result<()> {
        FlashWriter::erase_range(self, from..to)
    }

    fn write(&mut self, offset: u32, bytes: &[u8]) -> Result<()> {
        FlashWriter::write(self, offset, bytes)
    }
}

/// Extension trait to constrain the FMC peripheral
pub trait FlashExt {
    /// Constrains the FMC peripheral to play nicely with the other abstractions
    fn constrain(self) -> Parts;
}

impl FlashExt for FMC {
    fn constrain(self) -> Parts {
        Parts {
            ws: WS { _0: () },
            addr: ADDR { _0: () },
            ctl: CTL { _0: () },
            key: KEY { _0: () },
            _obstat: OBSTAT { _0: () },
            _obkey: OBKEY { _0: () },
            stat: STAT { _0: () },
            _wp: WP { _0: () },
        }
    }
}

/// Constrained FMC peripheral
pub struct Parts {
    /// Opaque wait state register
    pub ws: WS,

    /// Opaque unlock key register
    pub(crate) key: KEY,

    /// Opaque option byte unlock key register
    pub(crate) _obkey: OBKEY,

    /// Opaque status register
    pub(crate) stat: STAT,

    /// Opaque control register
    pub(crate) ctl: CTL,

    /// Opaque address register
    pub(crate) addr: ADDR,

    /// Opaque option byte status register
    pub(crate) _obstat: OBSTAT,

    /// Opaque write protection register
    pub(crate) _wp: WP,
}

impl Parts {
    pub fn writer(&mut self, flash_sz: FlashSize) -> FlashWriter {
        FlashWriter {
            fmc: self,
            flash_sz,
            verify: true,
        }
    }
}

macro_rules! reg_proxy {
    ($($(#[$($meta:meta)*])* $type:ident => $pac_type:ident: $field:ident => $pac_field:ident,)*) => {
        $(
        $(#[$($meta)*])*
            pub struct $type {
                _0: (),
            }

            #[allow(dead_code)]
            impl $type {
                pub(crate) fn $field(&mut self) -> &fmc::$pac_type {
                    // NOTE(unsafe) this proxy grants exclusive access to this register
                    unsafe { &(*FMC::ptr()).$pac_field }
                }
            }
        )*
    };
}

reg_proxy! {
    /// Opaque wait state register
    WS => WS: ws => ws,

    /// Opaque unlock key register
    KEY => KEY0: key => key0,

    /// Opaque option byte unlock key register
    OBKEY => OBKEY: obkey => obkey,

    /// Opaque status register
    STAT => STAT0: stat => stat0,

    /// Opaque control register
    CTL => CTL0: ctl => ctl0,

    /// Opaque address register
    ADDR => ADDR0: addr => addr0,

    /// Opaque option byte status register
    OBSTAT => OBSTAT: obstat => obstat,

    /// Opaque write protection register
    WP => WP: wp => wp,
}
