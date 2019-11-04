use crate::time::Hertz;

pub struct Clocks;

impl Clocks {
    pub fn pclk1(&self) -> Hertz {
        Hertz(54_000_000)
    }

    pub fn pclk2(&self) -> Hertz {
        Hertz(108_000_000)
    }
}
