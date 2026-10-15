//! Timer/Counter
//!

#[cfg(feature = "efemb")]
pub mod efemb;
pub mod irq;

pub use crate::pac::timer::vals::{
    CcCtrlCmoa, CcCtrlIcedge, CcCtrlIcevctrl, CcCtrlMode, CcCtrlPrssel, CcLoc, Clksel, CtrlMode,
    Presc,
};
use crate::{
    cmu::Clocks,
    gpio::pin::Pin,
    peripherals, prs,
    timer::irq::{default_handler, TIMER_HANDLER},
    Sealed, SingleCycleRMW,
};
use core::{convert::Infallible, marker::PhantomData};
use cortex_m::peripheral::NVIC;
use embassy_hal_internal::Peri;
use embedded_hal::{
    delay::DelayNs,
    digital::OutputPin,
    pwm::{ErrorType, SetDutyCycle},
};

/// Timer peripheral ID
#[derive(Debug, Clone, Copy, Eq, PartialEq, Ord, PartialOrd)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[repr(u8)]
pub enum TimerId {
    /// Timer0 peripheral
    Timer0,
    /// Timer1 peripheral
    Timer1,
}

impl TimerId {
    /// Number of Timer peripherals
    pub const COUNT: usize = 2;
}

/// A timer peripheral instance usable by the HAL timer driver.
pub trait TimerInstance: Sealed + embassy_hal_internal::PeripheralType + 'static {
    /// Returns the chiptool PAC register-block handle for this timer instance.
    fn regs(&self) -> crate::pac::timer::Timer;
    /// Timer peripheral ID
    fn id(&self) -> TimerId;
    /// Peripheral Interrupt
    fn interrupt(&self) -> crate::pac::Interrupt;
}

impl Sealed for peripherals::Timer0 {}
impl TimerInstance for peripherals::Timer0 {
    fn regs(&self) -> crate::pac::timer::Timer {
        crate::pac::TIMER0
    }

    fn id(&self) -> TimerId {
        TimerId::Timer0
    }

    fn interrupt(&self) -> crate::pac::Interrupt {
        crate::pac::Interrupt::TIMER0
    }
}

impl Sealed for peripherals::Timer1 {}
impl TimerInstance for peripherals::Timer1 {
    fn regs(&self) -> crate::pac::timer::Timer {
        crate::pac::TIMER1
    }

    fn id(&self) -> TimerId {
        TimerId::Timer1
    }

    fn interrupt(&self) -> crate::pac::Interrupt {
        crate::pac::Interrupt::TIMER1
    }
}

/// Handler function for a TIMER interrupt
pub type TimerIrqHandler = fn(flags: Interrupts, regs: crate::pac::timer::Timer);

/// Timer
#[derive(Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct Timer<'d, T: TimerInstance> {
    peri: Peri<'d, T>,
    /// Timer Capture/Compare channels
    pub channels: Channels<'d, T>,
}

impl<'d, T: TimerInstance> Timer<'d, T> {
    /// New Timer driver
    ///
    /// **NOTE**: This will enable the NVIC interrupt
    pub fn new(peri: Peri<'d, T>, config: &Config) -> Self {
        let channels = Channels {
            ch0: Channel::new(&peri),
            ch1: Channel::new(&peri),
            ch2: Channel::new(&peri),
            ch3: Channel::new(&peri),
        };
        let mut instance = Self { peri, channels };
        let p = instance.peri.regs();

        NVIC::mask(instance.peri.interrupt());

        // Enable the timer peripheral clock for this instance.
        crate::pac::CMU
            .hfperclken0()
            .modify(|w| w.set_timer(instance.peri.id() as usize, true));

        instance.stop();
        instance.disable_interrupts(Interrupts::ALL);
        instance.clear_interrupts(Interrupts::ALL);

        p.ctrl().write(|w| {
            w.set_mode(config.mode);
            w.set_sync(config.synchronized);
            w.set_osmen(config.one_shot);
            w.set_clksel(config.clock);
            w.set_presc(config.presc);
        });
        p.cnt().write(|w| w.set_cnt(config.count));
        p.top().write(|w| w.set_top(config.top));

        instance
            .channels
            .ch0
            .set_config(&config.channels.configs[ChannelId::Ch0 as usize]);
        instance
            .channels
            .ch1
            .set_config(&config.channels.configs[ChannelId::Ch1 as usize]);
        instance
            .channels
            .ch2
            .set_config(&config.channels.configs[ChannelId::Ch2 as usize]);
        instance
            .channels
            .ch3
            .set_config(&config.channels.configs[ChannelId::Ch3 as usize]);

        // Handle interrupt configs
        match config.interrupts.handler {
            // FIXME: unmask in NVIC?
            Some(handler) => instance.set_irq_handler(handler),
            None => instance.clear_irq_handler(),
        }
        instance.enable_interrupts(config.interrupts.flags);
        unsafe {
            NVIC::unmask(instance.peri.interrupt());
        };

        instance
    }

    /// Start Timer
    pub fn start(&mut self) {
        self.peri.regs().cmd().write(|w| w.set_start(true));
    }

    /// Stop Timer
    pub fn stop(&mut self) {
        self.peri.regs().cmd().write(|w| w.set_stop(true));
    }

    /// Get Timer count value
    pub fn count(&mut self) -> u16 {
        self.peri.regs().cnt().read().cnt()
    }

    /// Set Timer count value
    pub fn set_count(&mut self, count: u16) {
        self.peri.regs().cnt().write(|w| w.set_cnt(count));
    }

    /// Wait for timer flag by polling the peripheral interrupt flag
    ///
    /// Timer must be running before calling this function,
    pub fn poll_for_interrupt(&mut self, flag: Interrupt) {
        self.clear_interrupt(flag);
        while !self.get_interrupt(flag) {
            cortex_m::asm::nop();
        }
        self.clear_interrupt(flag);
    }

    /// Enable single interrupt
    pub fn enable_interrupt(&mut self, flag: Interrupt) {
        self.peri.regs().ien().sc_set(1u32 << flag as u8);
    }

    /// Set single interrupt flag
    pub fn get_interrupt(&self, flag: Interrupt) -> bool {
        (self.peri.regs().if_().read().0 & (1u32 << flag as u8)) != 0
    }

    /// Set single interrupt flag
    pub fn set_interrupt(&mut self, flag: Interrupt) {
        self.peri.regs().ifs().sc_set(1u32 << flag as u8);
    }

    /// Clear single interrupt flag
    pub fn clear_interrupt(&mut self, flag: Interrupt) {
        self.peri.regs().ifc().sc_set(1u32 << flag as u8);
    }

    /// Enable multiple interrupts
    pub fn enable_interrupts(&mut self, flags: Interrupts) {
        self.peri.regs().ien().sc_set(flags.0);
    }

    /// Disable multiple interrupts
    pub fn disable_interrupts(&mut self, flags: Interrupts) {
        self.peri.regs().ien().sc_clear(flags.0);
    }

    /// Get all interrupt flags
    pub fn get_interrupts(&mut self) -> Interrupts {
        Interrupts::new_unchecked(self.peri.regs().if_().read().0)
    }

    /// Set multiple interrupt flags
    pub fn set_interrupts(&mut self, flags: Interrupts) {
        self.peri.regs().ifs().sc_set(flags.0);
    }

    /// Clear multiple interrupt flags
    pub fn clear_interrupts(&mut self, flags: Interrupts) {
        self.peri.regs().ifc().sc_set(flags.0);
    }

    /// Async await for timer interrupt
    #[cfg(feature = "efemb")]
    pub async fn interrupt(&self) -> Interrupts {
        efemb::TimerWaker::new(unsafe { self.peri.clone_unchecked() }).await
    }

    pub(crate) fn set_irq_handler(&mut self, handler: TimerIrqHandler) {
        critical_section::with(|cs| {
            TIMER_HANDLER[self.peri.id() as usize]
                .borrow(cs)
                .replace(handler)
        });
    }

    pub(crate) fn clear_irq_handler(&mut self) {
        critical_section::with(|cs| {
            TIMER_HANDLER[self.peri.id() as usize]
                .borrow(cs)
                .replace(default_handler)
        });
    }
}

/// Timer interrupt flags
#[derive(Debug, Default, Clone, Copy, Eq, PartialEq, Ord, PartialOrd)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[repr(u8)]
pub enum Interrupt {
    /// Overflow Interrupt Flag
    #[default]
    Overflow = 0,
    /// Underflow Interrupt Flag
    Underflow,
    /// Direction Change Detect Interrupt Flag.
    ///
    /// Set only in Quadrature Decoder mode
    DirChange,
    /// CC Channel 0 Interrupt Flag
    Cc0 = 4,
    /// CC Channel 1 Interrupt Flag
    Cc1,
    /// CC Channel 2 Interrupt Flag
    Cc2,
    /// CC Channel 3 Interrupt Flag
    Cc3,
    /// CC Channel 0 Input Capture Buffer Overflow Interrupt Flag.
    ///
    /// A new capture value has pushed an unread value out of TIMERn_CC0_CCVB
    Cc0InputCaptureBufOvf,
    /// CC Channel 1 Input Capture Buffer Overflow Interrupt Flag.
    ///
    /// A new capture value has pushed an unread value out of TIMERn_CC1_CCVB
    Cc1InputCaptureBufOvf,
    /// CC Channel 2 Input Capture Buffer Overflow Interrupt Flag.
    ///
    /// A new capture value has pushed an unread value out of TIMERn_CC2_CCVB
    Cc2InputCaptureBufOvf,
    /// CC Channel 3 Input Capture Buffer Overflow Interrupt Flag.
    ///
    /// A new capture value has pushed an unread value out of TIMERn_CC3_CCVB
    Cc3InputCaptureBufOvf,
}

/// Interrupt flags bitfield
#[derive(Debug, Default, Clone, Copy, PartialEq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[repr(transparent)]
pub struct Interrupts(u32);

impl Interrupts {
    /// All interrupt flags
    pub const ALL: Self = Self::new(Interrupt::Overflow)
        .with_flag(Interrupt::Underflow)
        .with_flag(Interrupt::DirChange)
        .with_flag(Interrupt::Cc0)
        .with_flag(Interrupt::Cc1)
        .with_flag(Interrupt::Cc2)
        .with_flag(Interrupt::Cc3)
        .with_flag(Interrupt::Cc0InputCaptureBufOvf)
        .with_flag(Interrupt::Cc1InputCaptureBufOvf)
        .with_flag(Interrupt::Cc2InputCaptureBufOvf)
        .with_flag(Interrupt::Cc3InputCaptureBufOvf);

    /// Create interrupts bitfield starting from given flag
    pub const fn new(flag: Interrupt) -> Self {
        Self(1u32 << flag as u8)
    }

    /// Bitfield builder with given interrupt flag
    pub const fn with_flag(self, flag: Interrupt) -> Self {
        Self(self.0 | (1u32 << flag as u8))
    }

    /// Get state of given flag
    pub const fn is_set(&self, flag: Interrupt) -> bool {
        (self.0 & (1u32 << flag as u8)) != 0
    }

    /// Create a new bitfield from unchecked `u32` raw bitfield.
    ///
    /// This is meant to be used to convert the values of the IF* registers into [`Interrupts`]
    fn new_unchecked(raw_flags: u32) -> Self {
        Self(raw_flags)
    }
}

impl From<Interrupts> for u32 {
    fn from(value: Interrupts) -> Self {
        value.0
    }
}

/// Timer channel
#[derive(Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct Channel<'d, T: TimerInstance, const CN: u8> {
    peri: Peri<'d, T>,
}

impl<'d, T: TimerInstance, const CN: u8> Channel<'d, T, CN> {
    fn new(parent_peri: &Peri<'d, T>) -> Self {
        Self {
            peri: unsafe { parent_peri.clone_unchecked() },
        }
    }

    /// Set channel config
    pub fn set_config(&mut self, config: &CcConfig) {
        self.peri.regs().cc(CN as usize).ctrl().write(|w| {
            w.set_mode(config.mode);
            w.set_icedge(config.ic_edge_select);
            w.set_icevctrl(config.ic_event_control);
        });
        match config.input_sel {
            CcInputSel::Pin(loc) => {
                self.peri.regs().cc(CN as usize).ctrl().modify(|w| {
                    w.set_insel(false);
                });
                self.peri
                    .regs()
                    .routeloc0()
                    .modify(|w| w.set_cc_loc(CN as usize, loc));
            }
            CcInputSel::Prs(prs_channel_id) => {
                self.peri.regs().cc(CN as usize).ctrl().modify(|w| {
                    w.set_insel(true);
                    w.set_prssel(CcCtrlPrssel::from_bits(prs_channel_id as u8));
                });
            }
        }
    }

    /// Convert timer channel to a PWM
    pub fn into_pwm<PIN>(self, pin: PIN) -> ChannelPwm<'d, T, CN, PIN>
    where
        PIN: OutputPin + TimerPin<CN>,
    {
        let p = self.peri.regs();

        // FIXME: PWM - Set the resolution of the counter to MAX - 1 because if the timer is going to be split into
        // channels and any of them is used as PWM, we need to allow the PWM channel to set its compare value to TOP + 1
        // in order to achieve 100% duty cycle

        p.routeloc0()
            .modify(|w| w.set_cc_loc(CN as usize, pin.loc()));
        p.cc(CN as usize).ctrl().write(|w| {
            w.set_icedge(CcCtrlIcedge::Both);
            w.set_cmoa(CcCtrlCmoa::Toggle);
            w.set_mode(CcCtrlMode::Pwm)
        });
        p.routepen().modify(|w| w.set_cc_pen(CN as usize, true));

        ChannelPwm {
            peri: self.peri,
            _pwm_pin: PhantomData,
        }
    }

    /// Convert timer to a Delay
    pub fn into_delay(self, clocks: &Clocks) -> ChannelDelay<'d, T, CN> {
        let p = self.peri.regs();
        let timer_div: u8 = p.ctrl().read().presc().to_bits();
        let timer_freq = clocks.hf_per_clk() / (timer_div + 1) as u32;

        p.cc(CN as usize)
            .ctrl()
            .write(|w| w.set_mode(CcCtrlMode::Outputcompare));

        ChannelDelay {
            peri: self.peri,
            timer_freq,
        }
    }
}

/// Capture/Compare channels container
#[derive(Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct Channels<'d, T: TimerInstance> {
    /// Capture/Compare channel 0
    pub ch0: Channel<'d, T, 0>,
    /// Capture/Compare channel 1
    pub ch1: Channel<'d, T, 1>,
    /// Capture/Compare channel 2
    pub ch2: Channel<'d, T, 2>,
    /// Capture/Compare channel 3
    pub ch3: Channel<'d, T, 3>,
}

/// Timer driver config
#[derive(Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct Config {
    /// Counting mode for the Timer (`MODE`)
    pub mode: CtrlMode,
    /// Timer is started/stopped/reloaded by start/stop/reload commands in the other timers
    pub synchronized: bool,
    /// One-shot counter mode enabled
    pub one_shot: bool,
    /// Clock source for the timer (`CLKSEL`)
    pub clock: Clksel,
    /// Prescaling factor (`PRESC`)
    pub presc: Presc,
    /// Initial counter value (`CNT`)
    pub count: u16,
    /// Top value for the counter (`TOP`)
    pub top: u16,
    /// Capture/Compare configs for channels
    pub channels: ChannelCofigs,
    /// Enable interrupts, and provide an optional Interrupt Handler function
    pub interrupts: InterruptConfig,
}

impl Default for Config {
    fn default() -> Self {
        Self {
            mode: CtrlMode::Up,
            synchronized: false,
            one_shot: false,
            clock: Clksel::Preschfperclk,
            presc: Presc::Div1,
            count: Default::default(),
            // match the register default (makes sense since `count` defaults to `0` and `mode` to `Up`)
            top: u16::MAX,
            channels: Default::default(),
            interrupts: Default::default(),
        }
    }
}

/// Timer peripheral interrupt configuration
#[derive(Debug, Default)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct InterruptConfig {
    /// Enable interrupt flags
    pub flags: Interrupts,
    /// Optional timer interrupt reques thandler function
    pub handler: Option<TimerIrqHandler>,
}

/// Timer Capture/Compare Channel ID
#[derive(Debug, Default, Clone, Copy, Eq, PartialEq, Ord, PartialOrd)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[repr(u8)]
pub enum ChannelId {
    /// Timer Capture/Compare Channel 0
    #[default]
    Ch0,
    /// Timer Capture/Compare Channel 1
    Ch1,
    /// Timer Capture/Compare Channel 2
    Ch2,
    /// Timer Capture/Compare Channel 3
    Ch3,
}

impl ChannelId {
    /// Number of Timer CC channels
    pub const COUNT: usize = 4;
}

/// Timer Compare/Capture channel config
#[derive(Debug, Clone, Copy)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct CcConfig {
    /// channel mode
    pub mode: CcCtrlMode,
    /// channel input select
    pub input_sel: CcInputSel,
    /// Input Capture Edge Select
    pub ic_edge_select: CcCtrlIcedge,
    /// Input Capture Event Control
    pub ic_event_control: CcCtrlIcevctrl,
}

impl Default for CcConfig {
    fn default() -> Self {
        Self {
            mode: CcCtrlMode::Off,
            input_sel: Default::default(),
            ic_edge_select: CcCtrlIcedge::Rising,
            ic_event_control: CcCtrlIcevctrl::Everyedge,
        }
    }
}

/// Configurations for all Capture/Compare channels of a Timer
///
/// Implements [`Default`], and provides a method for initializing the configs for one specific channel.
///
/// # Example
///
/// Initialize configs with default values except for channel `2`:
///
/// ```no_run
///
/// ChannelCofigs::default()
///     .with_cc_config(
///         ChannelId::Id2,
///         CcConfig {
///             mode: CcCtrlMode::Pwm,
///             ..Default::default()
///         }
///     )
/// ```
#[derive(Debug, Default, Clone, Copy)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct ChannelCofigs {
    configs: [CcConfig; ChannelId::COUNT],
}

impl ChannelCofigs {
    /// Set config for the Capture/Compare channel with the given `id`
    pub fn with_cc_config(mut self, id: ChannelId, config: CcConfig) -> Self {
        self.configs[id as usize] = config;
        self
    }
}

/// Timer Capture/Compare channel input selection
#[derive(Debug, Clone, Copy)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum CcInputSel {
    /// TIMERnCCx pin is selected
    Pin(CcLoc),
    /// PRS input (selected by PRSSEL) is selected
    Prs(prs::ChannelId),
}

impl Default for CcInputSel {
    fn default() -> Self {
        Self::Pin(CcLoc::Loc0)
    }
}

/// Specialize the timer channel to be used for delays
#[derive(Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct ChannelDelay<'d, T: TimerInstance, const CN: u8> {
    peri: Peri<'d, T>,
    timer_freq: u32,
}

impl<'d, T: TimerInstance, const CN: u8> DelayNs for ChannelDelay<'d, T, CN> {
    fn delay_ns(&mut self, ns: u32) {
        let microsecs = ns / 1000;

        // FIXME: converting ns to us is just a band-aid in order to avoid the delay duration being smaller than this
        //        code can handle. Worst case scenario is if the timer frequency is the same as the core frequency,
        //        in which case wanting to wait for a few nanoseconds may take longer than expected because the code
        //        below needs to calculate a Compare value which may have already elapsed by the time it's written to
        //        the compare field of the CC channel.
        //        A better accuracy may be obtained if we implement `DelayNs` for `Timer` instead of `TimerChannelDelay`
        //        since we can control when the timer starts.
        if microsecs > 0 {
            let p = self.peri.regs();
            let ticks_left = self.timer_freq as u64 * microsecs as u64 / 1_000_000_u64;
            let reload_max = p.top().read().top() as u32;
            let reference_count = p.cnt().read().cnt() as u32;

            let mut ticks_left = ticks_left as u32;
            let mut reload = ticks_left.min(reload_max);
            let mut compare = (reference_count + reload) % reload_max;

            while ticks_left > 0 {
                p.ifc().write(|w| w.set_cc(CN as usize, true));
                p.cc(CN as usize).ccv().write(|w| w.set_ccv(compare as u16));
                p.ien().modify(|w| w.set_cc(CN as usize, true));

                // calculate next loop's values _before_ waiting so that the jitter between loops is minimal
                ticks_left -= reload;
                reload = ticks_left.min(reload_max);
                compare = (reference_count + reload) % reload_max;

                while !p.if_().read().cc(CN as usize) {}
            }
        }
    }
}

/// PWM
#[derive(Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct ChannelPwm<'d, T: TimerInstance, const CN: u8, PIN>
where
    PIN: OutputPin + TimerPin<CN>,
{
    peri: Peri<'d, T>,
    _pwm_pin: PhantomData<PIN>,
}

impl<'d, T: TimerInstance, const CN: u8, PIN> SetDutyCycle for ChannelPwm<'d, T, CN, PIN>
where
    PIN: OutputPin + TimerPin<CN>,
{
    fn max_duty_cycle(&self) -> u16 {
        // A 100% duty cycle is obtained by setting the channel Capture/Compare value to `top + 1`
        self.peri.regs().top().read().top().saturating_add(1)
    }

    fn set_duty_cycle(&mut self, duty: u16) -> Result<(), Self::Error> {
        self.peri
            .regs()
            .cc(CN as usize)
            .ccvb()
            .write(|w| w.set_ccvb(duty));

        Ok(())
    }
}

impl<'d, T: TimerInstance, const CN: u8, PIN> ErrorType for ChannelPwm<'d, T, CN, PIN>
where
    PIN: OutputPin + TimerPin<CN>,
{
    type Error = Infallible;
}

/// Trait to specify the location values for TIMERn_ROUTELOC0 and TIMERn_ROUTELOC1 for pins which can be used as PWM
pub trait TimerPin<const CN: u8> {
    /// TIMERn_ROUTELOC0 and TIMERn_ROUTELOC1 values for each pin which implements this trait
    fn loc(&self) -> CcLoc;
}

/// Implement pin location trait for each of the timer channels and their sets of 32 pins
///
/// (timer_channel, loc, port, pin)
macro_rules! impl_timer_channel_loc {
    ($channel:literal, $loc:expr, $port:literal, $pin:literal) => {
        impl<ANY> TimerPin<$channel> for Pin<$port, $pin, ANY> {
            fn loc(&self) -> CcLoc {
                $loc
            }
        }
    };
}

impl_timer_channel_loc!(0, CcLoc::Loc0, 'A', 0);
impl_timer_channel_loc!(0, CcLoc::Loc1, 'A', 1);
impl_timer_channel_loc!(0, CcLoc::Loc2, 'A', 2);
impl_timer_channel_loc!(0, CcLoc::Loc3, 'A', 3);
impl_timer_channel_loc!(0, CcLoc::Loc4, 'A', 4);
impl_timer_channel_loc!(0, CcLoc::Loc5, 'A', 5);
impl_timer_channel_loc!(0, CcLoc::Loc6, 'B', 11);
impl_timer_channel_loc!(0, CcLoc::Loc7, 'B', 12);
impl_timer_channel_loc!(0, CcLoc::Loc8, 'B', 13);
impl_timer_channel_loc!(0, CcLoc::Loc9, 'B', 14);
impl_timer_channel_loc!(0, CcLoc::Loc10, 'B', 15);
impl_timer_channel_loc!(0, CcLoc::Loc11, 'C', 6);
impl_timer_channel_loc!(0, CcLoc::Loc12, 'C', 7);
impl_timer_channel_loc!(0, CcLoc::Loc13, 'C', 8);
impl_timer_channel_loc!(0, CcLoc::Loc14, 'C', 9);
impl_timer_channel_loc!(0, CcLoc::Loc15, 'C', 10);
impl_timer_channel_loc!(0, CcLoc::Loc16, 'C', 11);
impl_timer_channel_loc!(0, CcLoc::Loc17, 'D', 9);
impl_timer_channel_loc!(0, CcLoc::Loc18, 'D', 10);
impl_timer_channel_loc!(0, CcLoc::Loc19, 'D', 11);
impl_timer_channel_loc!(0, CcLoc::Loc20, 'D', 12);
impl_timer_channel_loc!(0, CcLoc::Loc21, 'D', 13);
impl_timer_channel_loc!(0, CcLoc::Loc22, 'D', 14);
impl_timer_channel_loc!(0, CcLoc::Loc23, 'D', 15);
impl_timer_channel_loc!(0, CcLoc::Loc24, 'F', 0);
impl_timer_channel_loc!(0, CcLoc::Loc25, 'F', 1);
impl_timer_channel_loc!(0, CcLoc::Loc26, 'F', 2);
impl_timer_channel_loc!(0, CcLoc::Loc27, 'F', 3);
impl_timer_channel_loc!(0, CcLoc::Loc28, 'F', 4);
impl_timer_channel_loc!(0, CcLoc::Loc29, 'F', 5);
impl_timer_channel_loc!(0, CcLoc::Loc30, 'F', 6);
impl_timer_channel_loc!(0, CcLoc::Loc31, 'F', 7);

impl_timer_channel_loc!(1, CcLoc::Loc0, 'A', 1);
impl_timer_channel_loc!(1, CcLoc::Loc1, 'A', 2);
impl_timer_channel_loc!(1, CcLoc::Loc2, 'A', 3);
impl_timer_channel_loc!(1, CcLoc::Loc3, 'A', 4);
impl_timer_channel_loc!(1, CcLoc::Loc4, 'A', 5);
impl_timer_channel_loc!(1, CcLoc::Loc5, 'B', 11);
impl_timer_channel_loc!(1, CcLoc::Loc6, 'B', 12);
impl_timer_channel_loc!(1, CcLoc::Loc7, 'B', 13);
impl_timer_channel_loc!(1, CcLoc::Loc8, 'B', 14);
impl_timer_channel_loc!(1, CcLoc::Loc9, 'B', 15);
impl_timer_channel_loc!(1, CcLoc::Loc10, 'C', 6);
impl_timer_channel_loc!(1, CcLoc::Loc11, 'C', 7);
impl_timer_channel_loc!(1, CcLoc::Loc12, 'C', 8);
impl_timer_channel_loc!(1, CcLoc::Loc13, 'C', 9);
impl_timer_channel_loc!(1, CcLoc::Loc14, 'C', 10);
impl_timer_channel_loc!(1, CcLoc::Loc15, 'C', 11);
impl_timer_channel_loc!(1, CcLoc::Loc16, 'D', 9);
impl_timer_channel_loc!(1, CcLoc::Loc17, 'D', 10);
impl_timer_channel_loc!(1, CcLoc::Loc18, 'D', 11);
impl_timer_channel_loc!(1, CcLoc::Loc19, 'D', 12);
impl_timer_channel_loc!(1, CcLoc::Loc20, 'D', 13);
impl_timer_channel_loc!(1, CcLoc::Loc21, 'D', 14);
impl_timer_channel_loc!(1, CcLoc::Loc22, 'D', 15);
impl_timer_channel_loc!(1, CcLoc::Loc23, 'F', 0);
impl_timer_channel_loc!(1, CcLoc::Loc24, 'F', 1);
impl_timer_channel_loc!(1, CcLoc::Loc25, 'F', 2);
impl_timer_channel_loc!(1, CcLoc::Loc26, 'F', 3);
impl_timer_channel_loc!(1, CcLoc::Loc27, 'F', 4);
impl_timer_channel_loc!(1, CcLoc::Loc28, 'F', 5);
impl_timer_channel_loc!(1, CcLoc::Loc29, 'F', 6);
impl_timer_channel_loc!(1, CcLoc::Loc30, 'F', 7);
impl_timer_channel_loc!(1, CcLoc::Loc31, 'A', 0);

impl_timer_channel_loc!(2, CcLoc::Loc0, 'A', 2);
impl_timer_channel_loc!(2, CcLoc::Loc1, 'A', 3);
impl_timer_channel_loc!(2, CcLoc::Loc2, 'A', 4);
impl_timer_channel_loc!(2, CcLoc::Loc3, 'A', 5);
impl_timer_channel_loc!(2, CcLoc::Loc4, 'B', 11);
impl_timer_channel_loc!(2, CcLoc::Loc5, 'B', 12);
impl_timer_channel_loc!(2, CcLoc::Loc6, 'B', 13);
impl_timer_channel_loc!(2, CcLoc::Loc7, 'B', 14);
impl_timer_channel_loc!(2, CcLoc::Loc8, 'B', 15);
impl_timer_channel_loc!(2, CcLoc::Loc9, 'C', 6);
impl_timer_channel_loc!(2, CcLoc::Loc10, 'C', 7);
impl_timer_channel_loc!(2, CcLoc::Loc11, 'C', 8);
impl_timer_channel_loc!(2, CcLoc::Loc12, 'C', 9);
impl_timer_channel_loc!(2, CcLoc::Loc13, 'C', 10);
impl_timer_channel_loc!(2, CcLoc::Loc14, 'C', 11);
impl_timer_channel_loc!(2, CcLoc::Loc15, 'D', 9);
impl_timer_channel_loc!(2, CcLoc::Loc16, 'D', 10);
impl_timer_channel_loc!(2, CcLoc::Loc17, 'D', 11);
impl_timer_channel_loc!(2, CcLoc::Loc18, 'D', 12);
impl_timer_channel_loc!(2, CcLoc::Loc19, 'D', 13);
impl_timer_channel_loc!(2, CcLoc::Loc20, 'D', 14);
impl_timer_channel_loc!(2, CcLoc::Loc21, 'D', 15);
impl_timer_channel_loc!(2, CcLoc::Loc22, 'F', 0);
impl_timer_channel_loc!(2, CcLoc::Loc23, 'F', 1);
impl_timer_channel_loc!(2, CcLoc::Loc24, 'F', 2);
impl_timer_channel_loc!(2, CcLoc::Loc25, 'F', 3);
impl_timer_channel_loc!(2, CcLoc::Loc26, 'F', 4);
impl_timer_channel_loc!(2, CcLoc::Loc27, 'F', 5);
impl_timer_channel_loc!(2, CcLoc::Loc28, 'F', 6);
impl_timer_channel_loc!(2, CcLoc::Loc29, 'F', 7);
impl_timer_channel_loc!(2, CcLoc::Loc30, 'A', 0);
impl_timer_channel_loc!(2, CcLoc::Loc31, 'A', 1);

impl_timer_channel_loc!(3, CcLoc::Loc0, 'A', 3);
impl_timer_channel_loc!(3, CcLoc::Loc1, 'A', 4);
impl_timer_channel_loc!(3, CcLoc::Loc2, 'A', 5);
impl_timer_channel_loc!(3, CcLoc::Loc3, 'B', 11);
impl_timer_channel_loc!(3, CcLoc::Loc4, 'B', 12);
impl_timer_channel_loc!(3, CcLoc::Loc5, 'B', 13);
impl_timer_channel_loc!(3, CcLoc::Loc6, 'B', 14);
impl_timer_channel_loc!(3, CcLoc::Loc7, 'B', 15);
impl_timer_channel_loc!(3, CcLoc::Loc8, 'C', 6);
impl_timer_channel_loc!(3, CcLoc::Loc9, 'C', 7);
impl_timer_channel_loc!(3, CcLoc::Loc10, 'C', 8);
impl_timer_channel_loc!(3, CcLoc::Loc11, 'C', 9);
impl_timer_channel_loc!(3, CcLoc::Loc12, 'C', 10);
impl_timer_channel_loc!(3, CcLoc::Loc13, 'C', 11);
impl_timer_channel_loc!(3, CcLoc::Loc14, 'D', 9);
impl_timer_channel_loc!(3, CcLoc::Loc15, 'D', 10);
impl_timer_channel_loc!(3, CcLoc::Loc16, 'D', 11);
impl_timer_channel_loc!(3, CcLoc::Loc17, 'D', 12);
impl_timer_channel_loc!(3, CcLoc::Loc18, 'D', 13);
impl_timer_channel_loc!(3, CcLoc::Loc19, 'D', 14);
impl_timer_channel_loc!(3, CcLoc::Loc20, 'D', 15);
impl_timer_channel_loc!(3, CcLoc::Loc21, 'F', 0);
impl_timer_channel_loc!(3, CcLoc::Loc22, 'F', 1);
impl_timer_channel_loc!(3, CcLoc::Loc23, 'F', 2);
impl_timer_channel_loc!(3, CcLoc::Loc24, 'F', 3);
impl_timer_channel_loc!(3, CcLoc::Loc25, 'F', 4);
impl_timer_channel_loc!(3, CcLoc::Loc26, 'F', 5);
impl_timer_channel_loc!(3, CcLoc::Loc27, 'F', 6);
impl_timer_channel_loc!(3, CcLoc::Loc28, 'F', 7);
impl_timer_channel_loc!(3, CcLoc::Loc29, 'A', 0);
impl_timer_channel_loc!(3, CcLoc::Loc30, 'A', 1);
impl_timer_channel_loc!(3, CcLoc::Loc31, 'A', 2);
