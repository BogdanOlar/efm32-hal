//! Real Time Counter and Calendar
//!
//! [`RtTimer`] drives the RTCC in _normal_ mode: a 32 bit up-counter (`CNT`), ticked by a 15 bit pre-counter
//! (`PRECNT`) running on `LFECLK`, with three Capture/Compare channels.
//!
//! **NOTE**: the RTCC keeps running through a system reset (it's only reset by power loss, pin, or software reset), so
//! [`RtTimer::set_config`] always stops and fully reconfigures it.

#[cfg(feature = "_efemb")]
pub mod efemb;
pub mod irq;

pub use crate::pac::rtcc::vals::{CcCtrlCmoa, CcCtrlIcEdge, CcCtrlPrsSel, CntPresc};
use crate::{
    SingleCycleRMW,
    pac::{
        CMU, RTCC,
        msc::vals::Lockkey,
        rtcc::{regs::Ifs, vals::CcCtrlMode},
    },
    peripherals,
    rtcc::irq::{RTCC_HANDLER, default_handler},
};
use cortex_m::{asm::nop, peripheral::NVIC};
use embassy_hal_internal::Peri;

/// Rtcc Capture/Compare channel IDs
#[derive(Debug, Default, Clone, Copy)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[repr(u8)]
pub enum ChannelId {
    /// Rtcc Capture/Compare channel Id 0
    #[default]
    Ch0,
    /// Rtcc Capture/Compare channel Id 1
    Ch1,
    /// Rtcc Capture/Compare channel Id 2
    Ch2,
}

impl peripherals::Rtcc {
    /// Returns the chiptool PAC register-block handle for the RTCC
    pub(crate) const fn regs(&self) -> crate::pac::rtcc::Rtcc {
        RTCC
    }

    /// Peripheral Interrupt
    pub(crate) const fn interrupt(&self) -> crate::pac::Interrupt {
        crate::pac::Interrupt::RTCC
    }
}

/// Handler function for the RTCC interrupt
pub type RtTimerIrqHandler = fn(flags: Interrupts, regs: crate::pac::rtcc::Rtcc);

/// RTCC timer (normal mode)
#[derive(Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct RtTimer<'d> {
    peri: Peri<'d, peripherals::Rtcc>,
}

impl<'d> RtTimer<'d> {
    /// Unlock code for `RTCC_LOCK` (RM 11.5.15)
    const UNLOCK_KEY: u16 = 0xAEE8;

    /// Instantiate a RtTimer driver
    ///
    /// The LFECLK must be enabled before calling this.
    pub fn new(peri: Peri<'d, peripherals::Rtcc>, config: &Config) -> Self {
        Self::new_uninit(peri).with_config(config)
    }

    /// Instantiate an unitialized RtTimer driver
    pub(crate) const fn new_uninit(peri: Peri<'d, peripherals::Rtcc>) -> Self {
        Self { peri }
    }

    /// Build the RtTimer instance with the given config
    #[must_use]
    pub fn with_config(mut self, config: &Config) -> Self {
        self.set_config(config);
        self
    }

    /// Stop the timer, and configure it
    ///
    /// The timer has to be started with [`RtTimer::start`] afterwards.
    ///
    /// **NOTE**: This will enable the NVIC interrupt
    pub fn set_config(&mut self, config: &Config) {
        let p = self.peri.regs();

        NVIC::mask(self.peri.interrupt());

        // Enable RTCC clock
        CMU.lfeclken0().modify(|w| w.set_rtcc(true));

        // Sync
        while CMU.syncbusy().read().lfeclken0() {
            nop()
        }

        // The RTCC survives a system reset, so it may have been locked by a previous run
        p.lock()
            .write(|w| w.set_lockkey(Lockkey::from_bits(Self::UNLOCK_KEY)));

        // If CC1 was used as top value, a CCV1TOP wrap may be pending, which would make the first counter tick a wrap
        // to 0 (RM 11.3.1.1). Clear it by letting the counter tick once with a plain configuration.
        let ctrl = p.ctrl().read();
        if ctrl.enable() && ctrl.ccv1top() {
            p.cc(ChannelId::Ch1 as usize)
                .ctrl()
                .write(|w| w.set_mode(CcCtrlMode::Off));
            p.ctrl().write(|w| w.set_enable(true));
            let count = p.cnt().read();
            while p.cnt().read() == count {
                nop()
            }
        }

        self.stop();
        // The RTCC survives a system reset, so calendar mode flags may have been left enabled by a previous run
        self.disable_interrupts(Interrupts::ALL);
        self.clear_interrupts(Interrupts::ALL);

        // CCV1TOP must be set before (or when) the RTCC is enabled (RM 11.3.1.1), so the RTCC stays disabled here
        p.ctrl().write(|w| {
            w.set_debugrun(config.debug_run);
            w.set_preccv0top(config.precount_ccv0_top);
            w.set_ccv1top(config.ccv1_top);
            w.set_cntpresc(config.prescaler);
            w.set_cnttick(config.counter_tick.into());
            w.set_oscfdeten(config.osc_fail_detection);
            w.set_cntmode(CntMode::Normal.into());
        });
        p.precnt().write(|w| w.set_precnt(config.precount));
        p.cnt().write_value(config.count);

        for (channel, channel_config) in [ChannelId::Ch0, ChannelId::Ch1, ChannelId::Ch2]
            .into_iter()
            .zip(&config.channels)
        {
            self.set_channel_config(channel, channel_config);
        }

        // Handle interrupt configs
        match config.interrupts.handler {
            Some(handler) => self.set_irq_handler(handler),
            None => self.clear_irq_handler(),
        }
        self.enable_interrupts(config.interrupts.flags);
        unsafe {
            NVIC::unmask(self.peri.interrupt());
        };
    }

    /// Configure a Capture/Compare channel
    pub fn set_channel_config(&mut self, channel: ChannelId, config: &ChannelConfig) {
        let cc = self.peri.regs().cc(channel as usize);

        cc.ctrl().write(|w| {
            w.set_mode(match config.mode {
                ChannelMode::Off => CcCtrlMode::Off,
                ChannelMode::Compare => CcCtrlMode::Outputcompare,
                ChannelMode::Capture => CcCtrlMode::Inputcapture,
            });
            w.set_cmoa(config.output_action);
            w.set_icedge(config.capture_edge);
            w.set_prssel(config.capture_prs);
            w.set_compbase(config.compare_base.into());
            w.set_compmask(config.compare_mask);
        });
        cc.ccv().write_value(config.value);
    }

    /// Start Timer
    pub fn start(&mut self) {
        self.peri.regs().ctrl().modify(|w| w.set_enable(true));
    }

    /// Stop Timer
    pub fn stop(&mut self) {
        self.peri.regs().ctrl().modify(|w| w.set_enable(false));
    }

    /// Check if the Timer is running
    pub fn is_running(&self) -> bool {
        self.peri.regs().ctrl().read().enable()
    }

    /// Get main counter value (`CNT`)
    pub fn count(&self) -> u32 {
        self.peri.regs().cnt().read()
    }

    /// Set main counter value (`CNT`)
    pub fn set_count(&mut self, count: u32) {
        self.peri.regs().cnt().write_value(count);
    }

    /// Get pre-counter value (`PRECNT`)
    pub fn precount(&self) -> u16 {
        self.peri.regs().precnt().read().precnt()
    }

    /// Set pre-counter value (`PRECNT`)
    ///
    /// **NOTE**: this may alter the period of the current main counter tick
    pub fn set_precount(&mut self, precount: u16) {
        self.peri.regs().precnt().write(|w| w.set_precnt(precount));
    }

    /// Get compare value (`CCV`) of the given channel
    pub fn compare(&self, channel: ChannelId) -> u32 {
        self.peri.regs().cc(channel as usize).ccv().read()
    }

    /// Set compare value (`CCV`) of the given channel
    pub fn set_compare(&mut self, channel: ChannelId, value: u32) {
        self.peri
            .regs()
            .cc(channel as usize)
            .ccv()
            .write_value(value);
    }

    /// Get the last captured value of the given channel (when configured as [`ChannelMode::Capture`])
    pub fn capture(&self, channel: ChannelId) -> u32 {
        self.compare(channel)
    }

    /// Wait for timer flag by polling the peripheral interrupt flag
    ///
    /// Timer must be running before calling this function, and the `flag` interrupt must **not** be enabled, otherwise
    /// the interrupt handler will clear the flag before it can be observed here.
    pub fn poll_for_interrupt(&mut self, flag: Interrupt) {
        self.clear_interrupt(flag);
        while !self.get_interrupt(flag) {
            nop();
        }
        self.clear_interrupt(flag);
    }

    /// Enable single interrupt
    pub fn enable_interrupt(&mut self, flag: Interrupt) {
        self.enable_interrupts(Interrupts::new(flag));
    }

    /// Disable single interrupt
    pub fn disable_interrupt(&mut self, flag: Interrupt) {
        self.disable_interrupts(Interrupts::new(flag));
    }

    /// Get single interrupt flag
    pub fn get_interrupt(&self, flag: Interrupt) -> bool {
        self.get_interrupts().is_set(flag)
    }

    /// Set single interrupt flag
    pub fn set_interrupt(&mut self, flag: Interrupt) {
        self.set_interrupts(Interrupts::new(flag));
    }

    /// Clear single interrupt flag
    pub fn clear_interrupt(&mut self, flag: Interrupt) {
        self.clear_interrupts(Interrupts::new(flag));
    }

    /// Enable multiple interrupts
    pub fn enable_interrupts(&mut self, flags: Interrupts) {
        self.peri.regs().ien().sc_set(flags.into());
    }

    /// Disable multiple interrupts
    pub fn disable_interrupts(&mut self, flags: Interrupts) {
        self.peri.regs().ien().sc_clear(flags.into());
    }

    /// Get enabled interrupts
    pub fn enabled_interrupts(&self) -> Interrupts {
        Interrupts::new_unchecked(self.peri.regs().ien().read().0)
    }

    /// Get all interrupt flags
    pub fn get_interrupts(&self) -> Interrupts {
        Interrupts::new_unchecked(self.peri.regs().if_().read().0)
    }

    /// Set multiple interrupt flags
    pub fn set_interrupts(&mut self, flags: Interrupts) {
        self.peri.regs().ifs().write_value(Ifs(flags.into()));
    }

    /// Clear multiple interrupt flags
    pub fn clear_interrupts(&mut self, flags: Interrupts) {
        self.peri.regs().ifc().write_value(Ifs(flags.into()));
    }

    /// Async await for timer interrupt
    ///
    /// Only interrupts enabled with [`RtTimer::enable_interrupts`] (or [`InterruptConfig::flags`]) will be returned.
    #[cfg(feature = "_efemb")]
    pub async fn interrupt(&self) -> Interrupts {
        efemb::interrupts().await
    }

    /// Async wait until the main counter reaches `value`, using the given Capture/Compare `channel`
    ///
    /// The channel is configured as [`ChannelMode::Compare`] against `CNT`, and its interrupt is only enabled while
    /// waiting. Since the compare match only triggers when `CNT` _becomes_ equal to `value`, a `value` which has
    /// already been reached will only match after the counter wraps around.
    #[cfg(feature = "_efemb")]
    pub async fn wait_for_count(&mut self, channel: ChannelId, value: u32) {
        let flag = match channel {
            ChannelId::Ch0 => Interrupt::Cc0,
            ChannelId::Ch1 => Interrupt::Cc1,
            ChannelId::Ch2 => Interrupt::Cc2,
        };

        self.set_channel_config(
            channel,
            &ChannelConfig {
                mode: ChannelMode::Compare,
                value,
                ..Default::default()
            },
        );
        self.clear_interrupt(flag);
        self.enable_interrupt(flag);

        while !self.interrupt().await.is_set(flag) {}

        self.disable_interrupt(flag);
    }

    pub(crate) fn set_irq_handler(&mut self, handler: RtTimerIrqHandler) {
        critical_section::with(|cs| RTCC_HANDLER.borrow(cs).replace(handler));
    }

    pub(crate) fn clear_irq_handler(&mut self) {
        critical_section::with(|cs| RTCC_HANDLER.borrow(cs).replace(default_handler));
    }
}

/// RtTimer driver config
#[derive(Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct Config {
    /// Keep the timer running while the core is halted by the debugger (`DEBUGRUN`)
    pub debug_run: bool,
    /// Pre-counter tap used to tick the main counter, when [`Config::counter_tick`] is [`CounterTick::Prescaler`]
    /// (`CNTPRESC`)
    pub prescaler: CntPresc,
    /// Source of the main counter ticks (`CNTTICK`)
    pub counter_tick: CounterTick,
    /// The pre-counter wraps around when it reaches `CC0_CCV[14:0]` (`PRECCV0TOP`)
    pub precount_ccv0_top: bool,
    /// The main counter wraps around on a channel 1 compare match (`CCV1TOP`)
    pub ccv1_top: bool,
    /// Set the [`Interrupt::OscFail`] flag if no `LFECLK` ticks are detected within one `ULFRCO` cycle (`OSCFDETEN`)
    pub osc_fail_detection: bool,
    /// Initial main counter value (`CNT`)
    pub count: u32,
    /// Initial pre-counter value (`PRECNT`)
    pub precount: u16,
    /// Capture/Compare channels configuration
    pub channels: [ChannelConfig; 3],
    /// Enable interrupts, and provide an optional Interrupt Handler function
    pub interrupts: InterruptConfig,
}

impl Default for Config {
    fn default() -> Self {
        // match the register reset values
        Self {
            debug_run: false,
            prescaler: CntPresc::Div1,
            counter_tick: CounterTick::Prescaler,
            precount_ccv0_top: false,
            ccv1_top: false,
            osc_fail_detection: false,
            count: 0,
            precount: 0,
            channels: Default::default(),
            interrupts: Default::default(),
        }
    }
}

/// Source of the main counter ticks
#[derive(Debug, Default, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum CounterTick {
    /// The main counter ticks according to [`Config::prescaler`]
    #[default]
    Prescaler,
    /// The main counter ticks when the pre-counter matches `CC0_CCV[14:0]`
    Ccv0Match,
}

impl From<CounterTick> for bool {
    fn from(value: CounterTick) -> Self {
        value == CounterTick::Ccv0Match
    }
}

/// Main counter mode (`CNTMODE`)
#[derive(Debug, Default, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum CntMode {
    /// The main counter (`CNT`) is incremented by 1 on each tick
    #[default]
    Normal,
    /// The main counter is in calendar mode (`TIME`/`DATE`)
    Calendar,
}

impl From<CntMode> for bool {
    fn from(value: CntMode) -> Self {
        value == CntMode::Calendar
    }
}

/// Capture/Compare channel configuration
#[derive(Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct ChannelConfig {
    /// Channel mode (`MODE`)
    pub mode: ChannelMode,
    /// Compare value (`CCV`)
    pub value: u32,
    /// What the compare value is compared against (`COMPBASE`)
    pub compare_base: CompareBase,
    /// Number of least significant bits excluded from the comparison (`COMPMASK`)
    pub compare_mask: u8,
    /// PRS output action on compare match (`CMOA`)
    pub output_action: CcCtrlCmoa,
    /// Input capture edge (`ICEDGE`)
    pub capture_edge: CcCtrlIcEdge,
    /// Input capture PRS channel (`PRSSEL`)
    pub capture_prs: CcCtrlPrsSel,
}

impl Default for ChannelConfig {
    fn default() -> Self {
        // match the register reset values
        Self {
            mode: ChannelMode::Off,
            value: 0,
            compare_base: CompareBase::Cnt,
            compare_mask: 0,
            output_action: CcCtrlCmoa::Pulse,
            capture_edge: CcCtrlIcEdge::Rising,
            capture_prs: CcCtrlPrsSel::Prsch0,
        }
    }
}

/// Capture/Compare channel mode
#[derive(Debug, Default, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum ChannelMode {
    /// Channel turned off
    #[default]
    Off,
    /// Output compare
    Compare,
    /// Input capture of the main counter, on a PRS edge
    Capture,
}

/// Compare base of a Capture/Compare channel
#[derive(Debug, Default, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum CompareBase {
    /// `CNT` is compared against `CCV`
    #[default]
    Cnt,
    /// `{CNT[16:0], PRECNT[14:0]}` is compared against `CCV`
    PreCnt,
}

impl From<CompareBase> for bool {
    fn from(value: CompareBase) -> Self {
        value == CompareBase::PreCnt
    }
}

/// RtTimer peripheral interrupt configuration
#[derive(Debug, Default)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct InterruptConfig {
    /// Enable interrupt flags
    pub flags: Interrupts,
    /// Optional timer interrupt request handler function
    pub handler: Option<RtTimerIrqHandler>,
}

/// RTCC Interrupt Flags
#[derive(Debug, Default, Clone, Copy, Eq, PartialEq, Ord, PartialOrd)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[repr(u8)]
pub enum Interrupt {
    /// Main counter overflow interrupt flag
    #[default]
    Overflow,
    /// Channel 0 interrupt flag
    Cc0,
    /// Channel 1 interrupt flag
    Cc1,
    /// Channel 2 interrupt flag
    Cc2,
    /// Oscillator failure interrupt flag
    OscFail,
    /// Main counter tick interrupt flag
    CntTick,
    /// Minute tick interrupt flag (calendar mode only)
    MinTick,
    /// Hour tick interrupt flag (calendar mode only)
    HourTick,
    /// Day tick interrupt flag (calendar mode only)
    DayTick,
    /// Day of week overflow interrupt flag (calendar mode only)
    DayOfWeekOverflow,
    /// Month tick interrupt flag (calendar mode only)
    MonthTick,
}

/// Interrupt flags bitfield
#[derive(Debug, Default, Clone, Copy, PartialEq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[repr(transparent)]
pub struct Interrupts(u32);

impl Interrupts {
    /// All interrupt flags, including the calendar mode ones
    pub const ALL: Self = Self::new(Interrupt::Overflow)
        .with_flag(Interrupt::Cc0)
        .with_flag(Interrupt::Cc1)
        .with_flag(Interrupt::Cc2)
        .with_flag(Interrupt::OscFail)
        .with_flag(Interrupt::CntTick)
        .with_flag(Interrupt::MinTick)
        .with_flag(Interrupt::HourTick)
        .with_flag(Interrupt::DayTick)
        .with_flag(Interrupt::DayOfWeekOverflow)
        .with_flag(Interrupt::MonthTick);

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
