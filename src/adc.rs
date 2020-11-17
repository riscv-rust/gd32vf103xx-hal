//! Analog-to-digital converter
//!
use core::marker::PhantomData;

// NOTE: embedded_hal's Channel is not suitable for this rank + channel style ADC.
use crate::delay::McycleDelay;
use crate::gpio::gpioa::{PA0, PA1, PA2, PA3, PA4, PA5, PA6, PA7};
use crate::gpio::gpiob::{PB0, PB1};
use crate::gpio::gpioc::{PC0, PC1, PC2, PC3, PC4, PC5};
use crate::gpio::Analog;
use crate::hal::adc::Channel;
use crate::pac::{ADC0, ADC1};
use crate::rcu::Rcu;

macro_rules! adc_pins {
    ($ADC:ident, $($input:ty => $chan:expr),+ $(,)*) => {
        $(
            impl Channel<$ADC> for $input {
                type ID = u8;

                fn channel() -> u8 {
                    $chan
                }
            }
        )+
    };
}

/// Contains types related to ADC configuration
#[allow(non_camel_case_types)]
pub mod config {

    /// The number of cycles to sample a given channel for
    #[derive(Clone, Copy, Debug, PartialEq)]
    #[allow(non_camel_case_types)]
    #[repr(u8)]
    pub enum SampleTime {
        /// 1.5 cycles
        Point_1_5 = 0,
        /// 7.5 cycles
        Point_7_5,
        /// 13.5 cycles
        Point_13_5,
        /// 28.5 cycles
        Point_28_5,
        /// 41.5 cycles
        Point_41_5,
        /// 55.5 cycles
        Point_55_5,
        /// 71.5 cycles
        Point_71_5,
        /// 239.5 cycles
        Point_239_5,
    }

    impl Default for SampleTime {
        fn default() -> Self {
            SampleTime::Point_55_5
        }
    }

    /// Clock config for the ADC
    /// Check the datasheet for the maximum speed the ADC supports
    #[derive(Debug, Clone, Copy)]
    pub enum Clock {
        /// ADC prescaler select CK_APB2/2
        Apb2_div_2 = 0,
        /// ADC prescaler select CK_APB2/4
        Apb2_div_4 = 1,
        /// ADC prescaler select CK_APB2/6
        Apb2_div_6 = 2,
        /// ADC prescaler select CK_APB2/8
        Apb2_div_8 = 3,
        /// ADC prescaler select CK_APB2/12
        Apb2_div_12 = 5,
        /// ADC prescaler select CK_APB2/16
        Apb2_div_16 = 7,
    }

    /// Resolution to sample at
    #[derive(Debug, Clone, Copy)]
    #[repr(u8)]
    pub enum Resolution {
        /// 12-bit ADC resolution
        Twelve = 0,
        /// 10-bit ADC resolution
        Ten = 1,
        /// 8-bit ADC resolution
        Eight = 2,
        /// 6-bit ADC resolution
        Six = 3,
    }

    /// Regular group trigger source.
    #[derive(Debug, Clone, Copy)]
    pub enum RegularExternalTrigger {
        /// TIMER0 CH0 event select
        Timer0_Ch0 = 0b000,
        /// TIMER0 CH1 event select
        Timer0_Ch1 = 0b001,
        /// TIMER0 CH2 event select
        Timer0_Ch2 = 0b010,
        /// TIMER1 CH1 event select
        Timer1_Ch1 = 0b011,
        /// TIMER2 TRGO event select
        Timer2_Trgo = 0b100,
        /// TIMER3 CH3 event select
        Timer3_Ch3 = 0b101,
        /// external interrupt line 11
        Exti11 = 0b110,
        /// software trigger
        None = 0b111,
    }

    /// Inserted group trigger source.
    #[derive(Debug, Clone, Copy)]
    pub enum InsertedExternalTrigger {
        /// TIMER0 TRGO event select
        Timer2_Trgo = 0b000,
        /// TIMER0 CH3 event select
        Timer0_Ch3 = 0b001,
        /// TIMER1 TRGO event select
        Timer1_Trgo = 0b010,
        /// TIMER1 CH0 event select
        Timer1_Ch0 = 0b011,
        /// TIMER2 CH3 event select
        Timer2_Ch3 = 0b100,
        /// TIMER3 TRGO event select
        Timer3_Trgo = 0b101,
        /// external interrupt line 15
        Exti15 = 0b110,
        /// software trigger
        None = 0b111,
    }

    /// ADC data alignment
    #[derive(Debug, Clone, Copy)]
    pub enum Align {
        /// LSB alignment
        Right,
        /// MSB alignment
        Left,
    }

    impl From<Align> for bool {
        fn from(a: Align) -> bool {
            match a {
                Align::Right => false,
                Align::Left => true,
            }
        }
    }

    /// Scan enable/disable
    #[derive(Debug, Clone, Copy)]
    pub enum Scan {
        /// Scan mode disabled
        Disabled,
        /// Scan mode enabled
        Enabled,
    }
    impl From<Scan> for bool {
        fn from(s: Scan) -> bool {
            match s {
                Scan::Disabled => false,
                Scan::Enabled => true,
            }
        }
    }

    /// Continuous mode enable/disable
    #[derive(Debug, Clone, Copy)]
    pub enum Continuous {
        /// Single mode, continuous disabled
        Single,
        /// Continuous mode enabled
        Continuous,
    }
    impl From<Continuous> for bool {
        fn from(c: Continuous) -> bool {
            match c {
                Continuous::Single => false,
                Continuous::Continuous => true,
            }
        }
    }

    /// Config for regular channel group
    #[derive(Debug, Clone, Copy)]
    pub struct RegularChannelGroupConfig {
        pub(crate) external_trigger: RegularExternalTrigger,
    }

    impl RegularChannelGroupConfig {
        /// change the external_trigger field
        pub fn external_trigger(mut self, external_trigger: RegularExternalTrigger) -> Self {
            self.external_trigger = external_trigger;
            self
        }
    }

    impl Default for RegularChannelGroupConfig {
        fn default() -> Self {
            Self {
                external_trigger: RegularExternalTrigger::None,
            }
        }
    }

    /// Inserted channel management
    #[derive(Debug, Clone, Copy)]
    pub enum Insertion {
        /// Disabled
        Triggered,
        /// Inserted channel group convert automatically
        Auto,
    }
    impl From<Insertion> for bool {
        fn from(c: Insertion) -> bool {
            match c {
                Insertion::Triggered => false,
                Insertion::Auto => true,
            }
        }
    }

    /// Config for inserted channel group
    #[derive(Debug, Clone, Copy)]
    pub struct InsertedChannelGroupConfig {
        pub(crate) external_trigger: InsertedExternalTrigger,
        pub(crate) insertion: Insertion,
    }

    impl InsertedChannelGroupConfig {
        /// change the external_trigger field
        pub fn external_trigger(mut self, external_trigger: InsertedExternalTrigger) -> Self {
            self.external_trigger = external_trigger;
            self
        }
        /// change the insertion field
        pub fn insertion(mut self, insertion: Insertion) -> Self {
            self.insertion = insertion;
            self
        }
    }

    impl Default for InsertedChannelGroupConfig {
        fn default() -> Self {
            Self {
                external_trigger: InsertedExternalTrigger::None,
                insertion: Insertion::Triggered,
            }
        }
    }

    // TODO: DMA for regular channel

    /// Configuration for the adc.
    /// There are some additional parameters on the adc peripheral that can be
    /// added here when needed but this covers several basic usecases.
    #[derive(Debug, Clone, Copy)]
    pub struct AdcConfig {
        pub(crate) resolution: Resolution,
        pub(crate) align: Align,
        pub(crate) scan: Scan,
        pub(crate) continuous: Continuous,
        pub(crate) regular_channel: Option<RegularChannelGroupConfig>,
        pub(crate) inserted_channel: Option<InsertedChannelGroupConfig>,
        pub(crate) default_sample_time: SampleTime,
    }

    impl AdcConfig {
        /// change the resolution field
        pub fn resolution(mut self, resolution: Resolution) -> Self {
            self.resolution = resolution;
            self
        }
        /// change the align field
        pub fn align(mut self, align: Align) -> Self {
            self.align = align;
            self
        }
        /// change the scan field
        pub fn scan(mut self, scan: Scan) -> Self {
            self.scan = scan;
            self
        }
        /// change the continuous field
        pub fn continuous(mut self, continuous: Continuous) -> Self {
            self.continuous = continuous;
            self
        }
        /// change the external_trigger field
        pub fn enable_regular_channel(mut self, cfg: RegularChannelGroupConfig) -> Self {
            self.regular_channel = Some(cfg);
            self
        }
        /// change the external_trigger field
        pub fn enable_inserted_channel(mut self, cfg: InsertedChannelGroupConfig) -> Self {
            self.inserted_channel = Some(cfg);
            self
        }
        /// change the default_sample_time field
        pub fn default_sample_time(mut self, default_sample_time: SampleTime) -> Self {
            self.default_sample_time = default_sample_time;
            self
        }
    }

    impl Default for AdcConfig {
        fn default() -> Self {
            Self {
                resolution: Resolution::Twelve,
                align: Align::Right,
                scan: Scan::Disabled,
                continuous: Continuous::Single,
                regular_channel: None,
                inserted_channel: None,
                default_sample_time: SampleTime::Point_55_5,
            }
        }
    }
}

/// Enabled ADC (type state)
pub struct Enabled;
/// Disabled ADC (type state)
pub struct Disabled;

/// Type state trait for a enabled ADC channel
pub trait ED {}
/// Enabled ADC (type state)
impl ED for Enabled {}
/// Disabled ADC (type state)
impl ED for Disabled {}

adc_pins!(ADC0,
    PA0<Analog> => 0,
    PA1<Analog> => 1,
    PA2<Analog> => 2,
    PA3<Analog> => 3,
    PA4<Analog> => 4,
    PA5<Analog> => 5,
    PA6<Analog> => 6,
    PA7<Analog> => 7,
    PB0<Analog> => 8,
    PB1<Analog> => 9,
    PC0<Analog> => 10,
    PC1<Analog> => 11,
    PC2<Analog> => 12,
    PC3<Analog> => 13,
    PC4<Analog> => 14,
    PC5<Analog> => 15,
    Temperature<Enabled> => 16,
    Vrefint<Enabled> => 17,
);

adc_pins!(ADC1,
    PA0<Analog> => 0,
    PA1<Analog> => 1,
    PA2<Analog> => 2,
    PA3<Analog> => 3,
    PA4<Analog> => 4,
    PA5<Analog> => 5,
    PA6<Analog> => 6,
    PA7<Analog> => 7,
    PB0<Analog> => 8,
    PB1<Analog> => 9,
    PC0<Analog> => 10,
    PC1<Analog> => 11,
    PC2<Analog> => 12,
    PC3<Analog> => 13,
    PC4<Analog> => 14,
    PC5<Analog> => 15,
);

/// Analog to Digital Converter
pub struct Adc<ADC, ED> {
    rb: ADC,
    config: config::AdcConfig,
    _enabled: PhantomData<ED>,
}

// applies to all ADCs
impl<ADC> Adc<ADC, Disabled> {
    /// Set ADC clock div
    pub fn set_clock(&mut self, clock: config::Clock, rcu: &mut Rcu) {
        use self::config::Clock::*;

        match clock {
            Apb2_div_2 | Apb2_div_4 | Apb2_div_6 | Apb2_div_8 => unsafe {
                rcu.regs
                    .cfg0
                    .modify(|_, w| w.adcpsc_1_0().bits(clock as u8).adcpsc_2().clear_bit());
            },
            Apb2_div_12 | Apb2_div_16 => unsafe {
                rcu.regs.cfg0.modify(|_, w| {
                    w.adcpsc_1_0()
                        .bits(clock as u8 >> 2)
                        .adcpsc_2()
                        .bit(clock as u8 & 0x1 == 0x1)
                });
            },
        }
    }

    /// Applies all fields in AdcConfig
    pub fn apply_config(&mut self, config: config::AdcConfig) {
        self.config = config;
    }

    /// Sets the sampling resolution
    pub fn set_resolution(&mut self, resolution: config::Resolution) {
        self.config.resolution = resolution;
    }

    /// Sets the DR register alignment to left or right
    pub fn set_align(&mut self, align: config::Align) {
        self.config.align = align;
    }

    /// Enables and disables scan mode
    pub fn set_scan(&mut self, scan: config::Scan) {
        self.config.scan = scan;
    }

    /// Enables and disables continuous mode
    pub fn set_continuous(&mut self, continuous: config::Continuous) {
        self.config.continuous = continuous;
    }
}

impl Adc<ADC0, Disabled> {
    // adc_mode_config for single channel, use free mode
    fn pre_configure(&mut self, resolution: config::Resolution) {
        // resolution for ADC0
        unsafe {
            self.rb
                .ovsampctl
                .modify(|_, w| w.dres().bits(resolution as u8));
        }
    }
}

impl Adc<ADC1, Disabled> {
    fn pre_configure(&mut self, _resolution: config::Resolution) {}
}

macro_rules! adc {
    ($($adc_type:ident => ($constructor_fn_name:ident, $rcu_en_field:ident, $rcu_rst_field:ident)),+ $(,)*) => {
        $(
            impl Adc<$adc_type, Disabled> {
                /// Enables the ADC clock, resets the peripheral
                pub fn $constructor_fn_name(adc: $adc_type, rcu: &mut Rcu) -> Self {
                    let mut adc = Self::default_from_rb(adc);
                    // enable ADC clock
                    rcu.regs.apb2en.modify(|_, w| w.$rcu_en_field().set_bit());
                    // config default ADC clock
                    adc.set_clock(config::Clock::Apb2_div_16, rcu);
                    // adc_deinit
                    adc.reset(rcu);

                    unsafe {
                        // reset inserted sequence
                        adc.rb.isq.modify(|_, w| w.il().bits(0x00));
                    }
                    adc
                }
                /// Creates ADC with default settings
                fn default_from_rb(rb: $adc_type) -> Self {
                    Self {
                        rb,
                        config: config::AdcConfig::default(),
                        _enabled: PhantomData,
                    }
                }
                fn reset(&mut self, rcu: &mut Rcu) {
                    rcu.regs.apb2rst.modify(|_, w| w.$rcu_rst_field().set_bit());
                    rcu.regs.apb2rst.modify(|_, w| w.$rcu_rst_field().clear_bit());
                }
                fn configure(&mut self) {
                    let config = &self.config;
                    // ADC scan function enable
                    unsafe {
                        // data align
                        self.rb.ctl1.modify(|_, w| w.dal().bit(config.align.into()));
                        // scan mode
                        self.rb.ctl0.modify(|_, w| w.sm().bit(config.scan.into()));
                        // continuous mode
                        self.rb
                            .ctl1
                            .modify(|_, w| w.ctn().bit(config.continuous.into()));
                        // external trigger source
                        if let Some(trigger_config) = config.regular_channel {
                            self.rb
                                .ctl1
                                .modify(|_, w| w.etsrc().bits(trigger_config.external_trigger as u8));
                            self.rb.ctl1.modify(|_, w| w.eterc().set_bit());
                        }
                        if let Some(trigger_config) = config.inserted_channel {
                            self.rb
                                .ctl0
                                .modify(|_, w| w.ica().bit(trigger_config.insertion.into()));
                            self.rb
                                .ctl1
                                .modify(|_, w| w.etsic().bits(trigger_config.external_trigger as u8));
                            self.rb.ctl1.modify(|_, w| w.eteic().set_bit());
                        }
                        // TODO: discontinuous mode
                        // TODO: configuration constraints
                    }
                }
                /// ADC sampling time config
                fn set_channel_sample_time(&mut self, channel: u8, sample_time: config::SampleTime) {
                    match channel {
                        0..=9 => unsafe {
                            let mask = !(0b111 << (3 * channel));
                            self.rb.sampt1.modify(|r, w| {
                                let cleared = r.bits() & mask;
                                let masked = (sample_time as u8 as u32) << (3 * channel);
                                w.bits(cleared | masked)
                            });
                        },
                        10..=17 => unsafe {
                            let mask = !(0b111 << (3 * (channel - 10)));
                            self.rb.sampt0.modify(|r, w| {
                                let cleared = r.bits() & mask;
                                let masked = (sample_time as u8 as u32) << (3 * (channel - 10));
                                w.bits(cleared | masked)
                            });
                        },
                        _ => unreachable!("invalid channel"),
                    }
                }
                /// configure ADC regular channel
                ///
                /// 12 - Used in single mode
                pub fn configure_regular_channel<CHANNEL>(
                    &mut self,
                    rank: u8,
                    _channel: &CHANNEL,
                    sample_time: config::SampleTime,
                ) where
                    CHANNEL: Channel<ADC0, ID = u8>,
                {
                    let channel = CHANNEL::channel();
                    // ADC regular sequence config
                    match rank {
                        0..=5 => unsafe {
                            let mask = !(0b11111 << (5 * rank));
                            self.rb.rsq2.modify(|r, w| {
                                let cleared = r.bits() & mask;
                                w.bits(cleared | ((rank as u32) << (5 * rank)))
                            })
                        },
                        6..=11 => unsafe {
                            let mask = !(0b11111 << (5 * (rank - 6)));
                            self.rb.rsq1.modify(|r, w| {
                                let cleared = r.bits() & mask;
                                w.bits(cleared | ((rank as u32) << (5 * (rank - 6))))
                            })
                        },
                        12..=15 => unsafe {
                            let mask = !(0b11111 << (5 * (rank - 12)));
                            self.rb.rsq0.modify(|r, w| {
                                let cleared = r.bits() & mask;
                                w.bits(cleared | ((rank as u32) << (5 * (rank - 12))))
                            })
                        },
                        _ => panic!("invalid rank"),
                    }
                    // ADC sampling time config
                    self.set_channel_sample_time(channel, sample_time);
                }
                /// configure ADC inserted channel
                ///
                /// - 0 -> ISQ3
                /// - 1 -> ISQ2
                /// - 2 -> ISQ1
                /// - 3 -> ISQ0
                pub fn configure_inserted_channel<CHANNEL>(
                    &mut self,
                    rank: u8,
                    _channel: &CHANNEL,
                    sample_time: config::SampleTime,
                ) where
                    CHANNEL: Channel<ADC0, ID = u8>,
                {
                    // Check the sequence is long enough
                    self.rb.isq.modify(|r, w| {
                        let prev = r.il().bits();
                        if prev < rank {
                            unsafe { w.il().bits(rank) }
                        } else {
                            w
                        }
                    });
                    let channel = CHANNEL::channel();
                    unsafe {
                        // the channel number is written to these bits to select a channel
                        // as the nth conversion in the inserted channel group
                        //
                        // Inserted channels are converted starting from (4 - IL[1:0] - 1),
                        // if IL[1:0] length is less than 4.
                        match rank {
                            0 => self.rb.isq.modify(|_, w| w.isq3().bits(channel)),
                            1 => self.rb.isq.modify(|_, w| w.isq2().bits(channel)),
                            2 => self.rb.isq.modify(|_, w| w.isq1().bits(channel)),
                            3 => self.rb.isq.modify(|_, w| w.isq0().bits(channel)),
                            _ => panic!("invalid rank"),
                        }
                    }
                    // ADC sampling time config
                    self.set_channel_sample_time(channel, sample_time);
                }
                /// Enables the adc
                pub fn enable(mut self) -> Adc<$adc_type, Enabled> {
                    self.pre_configure(self.config.resolution);
                    self.configure();
                    self.rb.ctl1.modify(|_, w| w.adcon().set_bit());
                    Adc {
                        rb: self.rb,
                        config: self.config,
                        _enabled: PhantomData,
                    }
                }
            }

            impl Adc<$adc_type, Enabled> {
                /// Disable the ADC
                pub fn disable(self) -> Adc<$adc_type, Disabled> {
                    self.rb.ctl1.modify(|_, w| w.adcon().clear_bit());
                    Adc {
                        rb: self.rb,
                        config: self.config,
                        _enabled: PhantomData,
                    }
                }
                /// Enable software trigger for regular channel and inserted channel (if any)
                pub fn enable_software_trigger(&mut self) {
                    if self.config.regular_channel.is_some() {
                        self.rb.ctl1.modify(|_, w| w.swrcst().set_bit());
                    }
                    if self.config.inserted_channel.is_some() {
                        self.rb.ctl1.modify(|_, w| w.swicst().set_bit());
                    }
                }
                /// Enable interrupts for regular channel and inserted channel (if any)
                pub fn enable_interrupt(&mut self) {
                    if self.config.regular_channel.is_some() {
                        self.rb.ctl1.modify(|_, w| w.eterc().set_bit());
                    }
                    if self.config.inserted_channel.is_some() {
                        self.rb.ctl1.modify(|_, w| w.eteic().set_bit());
                    }
                }
                /// Disable interrupts for regular channel and inserted channel (if any)
                pub fn disable_interrupt(&mut self) {
                    if self.config.regular_channel.is_some() {
                        self.rb.ctl1.modify(|_, w| w.eterc().clear_bit());
                    }
                    if self.config.inserted_channel.is_some() {
                        self.rb.ctl1.modify(|_, w| w.eteic().clear_bit());
                    }
                }
                /// Wait for the conversion sequence to finished
                pub fn wait_for_conversion(&self) {
                    while self.rb.stat.read().eoc().bit_is_clear() {}
                    if self.config.inserted_channel.is_some() {
                        while self.rb.stat.read().eoic().bit_is_clear() {}
                    }
                }
                /// Resets the end-of-conversion flag, and optionally end-of-inserted-conversion flag
                pub fn clear_end_of_conversion_flag(&self) {
                    self.rb.stat.modify(|_, w| {
                        if self.config.inserted_channel.is_some() {
                            w.eoc().clear_bit().eoic().clear_bit()
                        } else {
                            w.eoc().clear_bit()
                        }
                    });
                }
                // reset the selected ADC calibration registers
                fn reset_calibrate(&mut self) {
                    self.rb.ctl1.modify(|_, w| w.rstclb().set_bit());
                    while self.rb.ctl1.read().rstclb().bit_is_set() {}
                }
                /// Calibrates the ADC in single channel mode
                pub fn calibrate(&mut self) {
                    self.reset_calibrate();
                    self.rb.ctl1.modify(|_, w| w.clb().set_bit());
                    while self.rb.ctl1.read().clb().bit_is_set() {}
                }
                /// Read data from regular channel
                pub fn read_rdata(&self) -> u16 {
                    self.rb.rdata.read().rdata().bits()
                }
                /// Read data from inserted channel 0
                pub fn read_idata0(&self) -> u16 {
                    self.rb.idata0.read().idatan().bits()
                }
                /// Read data from inserted channel 1
                pub fn read_idata1(&self) -> u16 {
                    self.rb.idata1.read().idatan().bits()
                }
                /// Read data from inserted channel 2
                pub fn read_idata2(&self) -> u16 {
                    self.rb.idata2.read().idatan().bits()
                }
                /// Read data from inserted channel 3
                pub fn read_idata3(&self) -> u16 {
                    self.rb.idata3.read().idatan().bits()
                }
            }
        )+
    };
}

adc!(
    ADC0 => (adc0, adc0en, adc0rst),
    ADC1 => (adc1, adc1en, adc1rst)
);

/// Internal temperature sensor
pub struct Temperature<ED> {
    _marker: PhantomData<ED>,
}

impl Temperature<Disabled> {
    /// Internal temperature sensor
    // TODO: avoid creating multiple instences
    pub fn new() -> Self {
        Self {
            _marker: PhantomData,
        }
    }
}

impl Temperature<Disabled> {
    /// Enable temperature sensor channel
    pub fn enable(&mut self, adc: &mut Adc<ADC0, Disabled>) -> Temperature<Enabled> {
        adc.rb.ctl1.modify(|_, w| w.tsvren().set_bit());
        Temperature {
            _marker: PhantomData,
        }
    }
}

/// Vref internal signal
// internally connected to the ADC0_CH17 input channel
pub struct Vrefint<ED> {
    _marker: PhantomData<ED>,
}

impl Vrefint<Disabled> {
    /// New Vref internal signal
    pub fn new() -> Self {
        Self {
            _marker: PhantomData,
        }
    }
}

impl Vrefint<Disabled> {
    /// Enable Vrefint sensor channel
    pub fn enable(self, adc: &mut Adc<ADC0, Disabled>) -> Vrefint<Enabled> {
        adc.rb.ctl1.modify(|_, w| w.tsvren().set_bit());
        Vrefint {
            _marker: PhantomData,
        }
    }
}

/// ADC sync mode
#[repr(u8)]
pub enum SyncMode {
    /// all the ADCs work independently
    Free = 0,
    /// ADC0 and ADC1 work in combined regular parallel + inserted parallel mode
    DualRegulalParallelInsertedParallel,
    ///  ADC0 and ADC1 work in combined regular parallel + trigger rotation mode
    DualRegulalParallelInsertedRotation,
    /// ADC0 and ADC1 work in combined inserted parallel + follow-up fast mode
    DualInsertedParallelRegulalFollowupFast,
    /// ADC0 and ADC1 work in combined inserted parallel + follow-up slow mode
    DualInsertedParallelRegulalFollowupSlow,
    /// ADC0 and ADC1 work in inserted parallel mode only
    DualInsertedParallel,
    /// ADC0 and ADC1 work in regular parallel mode only
    DualRegulalParallel,
    /// ADC0 and ADC1 work in follow-up fast mode only
    DualRegulalFollowupFast,
    /// ADC0 and ADC1 work in follow-up slow mode only
    DualRegulalFollowupSlow,
    /// ADC0 and ADC1 work in trigger rotation mode only
    DualInsertedTriggerRotation,
}

/// Init ADC0 and ADC1 at once, enabling sync mode.
pub fn adc01(
    _adc0: ADC0,
    _adc1: ADC1,
    _sync_mode: SyncMode,
    _delay: &mut McycleDelay,
) -> (Adc<ADC0, Disabled>, Adc<ADC1, Disabled>) {
    // NOTE: default is SyncMode::Free
    // adc0.rb
    //     .ctl0
    //     .modify(|_, w| w.syncm().bits(SyncMode::Free as u8));
    unimplemented!("DMA required to use sync mode")
}
