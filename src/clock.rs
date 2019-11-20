use crate::time::Hertz;

pub struct Clocks;

impl Clocks {
    fn base(&self) -> Hertz {
        Hertz(8_000_000)
    }
    pub fn pclk1(&self) -> Hertz {
        Hertz(self.base().0 / 2)
    }

    pub fn pclk2(&self) -> Hertz {
        self.base()
    }
}
