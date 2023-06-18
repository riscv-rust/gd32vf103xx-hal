//! USB full-speed

pub mod device;

/// Settings for port speed.
///
/// In Host mode, either setting is acceptable.
///
/// In Device mode, port speed is fixed to full-speed.
#[repr(u8)]
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum PortSpeed {
    FullSpeedHost = 0b01,
    LowSpeedHost = 0b10,
    FullSpeedDevice = 0b11,
}
