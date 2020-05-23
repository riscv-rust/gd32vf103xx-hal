//! HAL for the GD32VF103xx family
//!
//! This is an implementation of the [`embedded-hal`] traits for the GD32VF103xx family

//#![deny(missing_docs)]
#![no_std]

pub use gd32vf103_pac as pac;

use embedded_hal as hal;

pub mod afio;
pub mod backup_domain;
pub mod delay;
pub mod exmc;
pub mod gpio;
pub mod prelude;
pub mod pwm;
pub mod rcu;
pub mod rtc;
pub mod serial;
pub mod spi;
pub mod time;
pub mod timer;
