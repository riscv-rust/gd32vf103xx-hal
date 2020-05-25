//! HAL for the GD32VF103xx family
//!
//! This is an implementation of the [`embedded-hal`] traits for the GD32VF103xx family

//#![deny(missing_docs)]
#![no_std]

pub use gd32vf103_pac as pac;

use embedded_hal as hal;

pub mod afio;
pub mod exmc;
pub mod gpio;
pub mod i2c;
pub mod rcu;
pub mod backup_domain;
pub mod prelude;
pub mod serial;
pub mod spi;
pub mod time;
pub mod timer;
pub mod rtc;
pub mod delay;
pub mod pwm;
