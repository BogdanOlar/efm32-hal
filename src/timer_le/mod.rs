//! Low Energy Timer
//!

#[cfg(feature = "efemb")]
pub mod efemb;
pub mod irq;

pub use crate::pac::letimer::vals::Repmode;
use crate::{
    Sealed, SingleCycleRMW,
    gpio::pin::Pin,
    pac::{
        CMU,
        letimer::{
            regs::Ifs,
            vals::{Out0loc, Ufoa0},
        },
    },
    peripherals,
    timer_le::irq::{LETIMER_HANDLER, default_handler},
};
use core::marker::PhantomData;
use cortex_m::{asm::nop, peripheral::NVIC};
use embassy_hal_internal::Peri;
use embedded_hal::digital::OutputPin;

/// LeTimer peripheral ID
#[derive(Debug, Clone, Copy, Eq, PartialEq, Ord, PartialOrd)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[repr(u8)]
pub enum LeTimerId {
    /// Low Energy Timer 0
    LeTimer0,
}

impl LeTimerId {
    /// Number of LeTimer peripherals
    pub const COUNT: usize = 1;
}

/// A low-energy timer peripheral instance usable by the HAL timer driver.
pub trait LeTimerInstance:
    Sealed + embassy_hal_internal::PeripheralType + Send + Sync + 'static
{
    /// Returns the chiptool PAC register-block handle for this timer instance.
    fn regs(&self) -> crate::pac::letimer::LeTimer;
    /// Timer peripheral ID
    fn id(&self) -> LeTimerId;
    /// Peripheral Interrupt
    fn interrupt(&self) -> crate::pac::Interrupt;
}

impl Sealed for peripherals::LeTimer0 {}
impl LeTimerInstance for peripherals::LeTimer0 {
    fn regs(&self) -> crate::pac::letimer::LeTimer {
        crate::pac::LETIMER0
    }

    fn id(&self) -> LeTimerId {
        LeTimerId::LeTimer0
    }

    fn interrupt(&self) -> crate::pac::Interrupt {
        crate::pac::Interrupt::LETIMER0
    }
}

/// Handler function for a LETIMER interrupt
pub type LeTimerIrqHandler = fn(flags: Interrupts, regs: crate::pac::letimer::LeTimer);

/// Low Energy timer
/// Timer
#[derive(Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct LeTimer<'d, T: LeTimerInstance> {
    peri: Peri<'d, T>,
}

impl<'d, T: LeTimerInstance> LeTimer<'d, T> {
    /// Instantiate a LeTimer driver
    pub fn new(peri: Peri<'d, T>, config: &Config) -> Self {
        Self::new_uninit(peri).with_config(config)
    }

    /// Instantiate an unitialized LeTimer driver
    pub(crate) const fn new_uninit(peri: Peri<'d, T>) -> Self {
        Self { peri }
    }

    /// Build the LeTimer instance with the given config
    #[must_use]
    pub fn with_config(mut self, config: &Config) -> Self {
        self.set_config(config);
        self
    }

    /// Configure timer
    ///
    /// **NOTE**: This will enable the NVIC interrupt
    pub fn set_config(&mut self, config: &Config) {
        let p = self.peri.regs();

        NVIC::mask(self.peri.interrupt());

        // Enable LE Timer
        CMU.lfaclken0()
            .modify(|w| w.set_letimer(self.peri.id() as usize, true));

        // Sync
        while CMU.syncbusy().read().lfaclken0() {
            nop()
        }

        self.stop();
        // make sure the timer is stopped
        self.wait_for_sync();
        self.disable_interrupts(Interrupts::ALL);
        self.clear_interrupts(Interrupts::ALL);

        p.ctrl().write(|w| {
            w.set_repmode(config.repeat_mode);
            w.set_comp0top(config.comp0_top);
            w.set_buftop(config.buffered_top);
            w.set_debugrun(config.debug_run);
        });
        p.cnt().write(|w| w.set_cnt(config.count));
        p.comp0().write(|w| w.set_comp0(config.comp0));
        p.comp1().write(|w| w.set_comp1(config.comp1));
        p.rep0().write(|w| w.set_rep0(config.rep0));
        p.rep1().write(|w| w.set_rep1(config.rep1));

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

    /// Start Timer
    pub fn start(&mut self) {
        self.command(|w| w.set_start(true));
    }

    /// Stop Timer
    pub fn stop(&mut self) {
        self.command(|w| w.set_stop(true));
    }

    /// Clear Timer counter (`CNT`)
    pub fn clear(&mut self) {
        self.command(|w| w.set_clear(true));
    }

    /// Check if the Timer is running
    pub fn is_running(&self) -> bool {
        self.peri.regs().status().read().running()
    }

    /// Get Timer count value
    pub fn count(&self) -> u16 {
        self.peri.regs().cnt().read().cnt()
    }

    /// Set Timer count value
    pub fn set_count(&mut self, count: u16) {
        self.peri.regs().cnt().write(|w| w.set_cnt(count));
    }

    /// Get compare value (`COMPn`) of the given channel
    pub fn compare(&self, channel: ChannelId) -> u16 {
        let p = self.peri.regs();
        match channel {
            ChannelId::Ch0 => p.comp0().read().comp0(),
            ChannelId::Ch1 => p.comp1().read().comp1(),
        }
    }

    /// Set compare value (`COMPn`) of the given channel
    pub fn set_compare(&mut self, channel: ChannelId, value: u16) {
        let p = self.peri.regs();
        match channel {
            ChannelId::Ch0 => p.comp0().write(|w| w.set_comp0(value)),
            ChannelId::Ch1 => p.comp1().write(|w| w.set_comp1(value)),
        }
    }

    /// Get repeat counter value (`REPn`) of the given channel
    pub fn repeat(&self, channel: ChannelId) -> u8 {
        let p = self.peri.regs();
        match channel {
            ChannelId::Ch0 => p.rep0().read().rep0(),
            ChannelId::Ch1 => p.rep1().read().rep1(),
        }
    }

    /// Set repeat counter value (`REPn`) of the given channel
    pub fn set_repeat(&mut self, channel: ChannelId, value: u8) {
        let p = self.peri.regs();
        match channel {
            ChannelId::Ch0 => p.rep0().write(|w| w.set_rep0(value)),
            ChannelId::Ch1 => p.rep1().write(|w| w.set_rep1(value)),
        }
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
    /// Only interrupts enabled with [`LeTimer::enable_interrupts`] (or [`InterruptConfig::flags`]) will be returned.
    #[cfg(feature = "efemb")]
    pub async fn interrupt(&self) -> Interrupts {
        efemb::InterruptsFuture::new(unsafe { self.peri.clone_unchecked() }).await
    }

    pub(crate) fn set_irq_handler(&mut self, handler: LeTimerIrqHandler) {
        critical_section::with(|cs| {
            LETIMER_HANDLER[self.peri.id() as usize]
                .borrow(cs)
                .replace(handler)
        });
    }

    pub(crate) fn clear_irq_handler(&mut self) {
        critical_section::with(|cs| {
            LETIMER_HANDLER[self.peri.id() as usize]
                .borrow(cs)
                .replace(default_handler)
        });
    }

    /// Write `CMD` register,
    /// Waits for any ongoing commands to synchronize before executing the new one
    fn command(&mut self, f: impl FnOnce(&mut crate::pac::letimer::regs::Cmd)) {
        // A CMD write while a previous command is still pending isn't supported (RM 4.3.1)
        self.wait_for_sync();
        self.peri.regs().cmd().write(f);
    }

    /// Wait for `command` to be synchronized
    fn wait_for_sync(&self) {
        while self.peri.regs().syncbusy().read().cmd() {
            nop()
        }
    }

    /// Convert timer to PWM
    pub fn into_ch0_pwm<PIN>(self, pin: PIN) -> LeTimerPwm<0, PIN>
    where
        PIN: OutputPin + LeTimerPin<0>,
    {
        let le_timer = self.peri.regs();

        le_timer.rep0().write(|w| w.set_rep0(1));
        le_timer.comp0().write(|w| w.set_comp0(1000));
        le_timer.comp1().write(|w| w.set_comp1(500));
        le_timer.routepen().write(|w| w.set_out0pen(true));
        le_timer
            .routeloc0()
            .write(|w| w.set_out0loc(Out0loc::from_bits(pin.loc())));
        le_timer.ctrl().write(|w| {
            w.set_comp0top(true);
            w.set_ufoa0(Ufoa0::Pwm)
        });

        // start timer
        le_timer.cmd().write(|w| w.set_start(true));

        // Sync
        while le_timer.syncbusy().read().cmd() {
            nop()
        }

        LeTimerPwm {
            _pwm_pin: PhantomData,
        }
    }
}

/// LeTimer driver config
#[derive(Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct Config {
    /// Repeat mode (`REPMODE`)
    pub repeat_mode: Repmode,
    /// `COMP0` is used as the top value (reload value for `CNT` on underflow) instead of `0xFFFF` (`COMP0TOP`)
    pub comp0_top: bool,
    /// Load `COMP1` into `COMP0` when `REP0` reaches zero (`BUFTOP`)
    pub buffered_top: bool,
    /// Keep the timer running while the core is halted by the debugger (`DEBUGRUN`)
    pub debug_run: bool,
    /// Initial counter value (`CNT`)
    pub count: u16,
    /// Compare value 0 (`COMP0`)
    pub comp0: u16,
    /// Compare value 1 (`COMP1`)
    pub comp1: u16,
    /// Repeat counter 0 (`REP0`)
    pub rep0: u8,
    /// Repeat counter 1 (`REP1`)
    pub rep1: u8,
    /// Enable interrupts, and provide an optional Interrupt Handler function
    pub interrupts: InterruptConfig,
}

impl Default for Config {
    fn default() -> Self {
        // match the register reset values
        Self {
            repeat_mode: Repmode::Free,
            comp0_top: false,
            buffered_top: false,
            debug_run: false,
            count: 0,
            comp0: 0,
            comp1: 0,
            rep0: 0,
            rep1: 0,
            interrupts: Default::default(),
        }
    }
}

/// LeTimer peripheral interrupt configuration
#[derive(Debug, Default)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct InterruptConfig {
    /// Enable interrupt flags
    pub flags: Interrupts,
    /// Optional timer interrupt request handler function
    pub handler: Option<LeTimerIrqHandler>,
}

/// LeTimer Compare Channel ID
#[derive(Debug, Default, Clone, Copy)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[repr(u8)]
pub enum ChannelId {
    /// LeTimer Compare Channel ID 0
    #[default]
    Ch0,
    /// LeTimer Compare Channel ID 1
    Ch1,
}

/// Low Energy Timer Interrupt Flags
#[derive(Debug, Default, Clone, Copy, Eq, PartialEq, Ord, PartialOrd)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[repr(u8)]
pub enum Interrupt {
    /// Comparator 0 interrupt flag
    #[default]
    Comp0,
    /// Comparator 1 interrupt flag
    Comp1,
    /// Underflow interrupt flag
    Underflow,
    /// Repeat 0 interrupt flag
    Rep0,
    /// Repeat 1 interrupt flag
    Rep1,
}

/// Interrupt flags bitfield
#[derive(Debug, Default, Clone, Copy, PartialEq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[repr(transparent)]
pub struct Interrupts(u32);

impl Interrupts {
    /// All interrupt flags
    pub const ALL: Self = Self::new(Interrupt::Comp0)
        .with_flag(Interrupt::Comp1)
        .with_flag(Interrupt::Underflow)
        .with_flag(Interrupt::Rep0)
        .with_flag(Interrupt::Rep1);

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

/// Low Energy Timer PWM
pub struct LeTimerPwm<const CN: u8, PIN>
where
    PIN: OutputPin + LeTimerPin<CN>,
{
    _pwm_pin: PhantomData<PIN>,
}

/// Trait for each of the LE timer channels and their sets of 32 pins
pub trait LeTimerPin<const CN: u8> {
    /// Value to be written to LETIMERn_ROUTELOC0 register for the Pin implementing this trait
    fn loc(&self) -> u8;
}

/// Implement pin location trait for each of the LE timer channels and their sets of 32 pins
macro_rules! impl_le_timer_channel_loc {
    ($channel:literal, $loc:literal, $port:literal, $pin:literal) => {
        impl<ANY> LeTimerPin<$channel> for Pin<$port, $pin, ANY> {
            fn loc(&self) -> u8 {
                $loc
            }
        }
    };
}

impl_le_timer_channel_loc!(0, 0, 'A', 0);
impl_le_timer_channel_loc!(0, 1, 'A', 1);
impl_le_timer_channel_loc!(0, 2, 'A', 2);
impl_le_timer_channel_loc!(0, 3, 'A', 3);
impl_le_timer_channel_loc!(0, 4, 'A', 4);
impl_le_timer_channel_loc!(0, 5, 'A', 5);
impl_le_timer_channel_loc!(0, 6, 'B', 11);
impl_le_timer_channel_loc!(0, 7, 'B', 12);
impl_le_timer_channel_loc!(0, 8, 'B', 13);
impl_le_timer_channel_loc!(0, 9, 'B', 14);
impl_le_timer_channel_loc!(0, 10, 'B', 15);
impl_le_timer_channel_loc!(0, 11, 'C', 6);
impl_le_timer_channel_loc!(0, 12, 'C', 7);
impl_le_timer_channel_loc!(0, 13, 'C', 8);
impl_le_timer_channel_loc!(0, 14, 'C', 9);
impl_le_timer_channel_loc!(0, 15, 'C', 10);
impl_le_timer_channel_loc!(0, 16, 'C', 11);
impl_le_timer_channel_loc!(0, 17, 'D', 9);
impl_le_timer_channel_loc!(0, 18, 'D', 10);
impl_le_timer_channel_loc!(0, 19, 'D', 11);
impl_le_timer_channel_loc!(0, 20, 'D', 12);
impl_le_timer_channel_loc!(0, 21, 'D', 13);
impl_le_timer_channel_loc!(0, 22, 'D', 14);
impl_le_timer_channel_loc!(0, 23, 'D', 15);
impl_le_timer_channel_loc!(0, 24, 'F', 0);
impl_le_timer_channel_loc!(0, 25, 'F', 1);
impl_le_timer_channel_loc!(0, 26, 'F', 2);
impl_le_timer_channel_loc!(0, 27, 'F', 3);
impl_le_timer_channel_loc!(0, 28, 'F', 4);
impl_le_timer_channel_loc!(0, 29, 'F', 5);
impl_le_timer_channel_loc!(0, 30, 'F', 6);
impl_le_timer_channel_loc!(0, 31, 'F', 7);

impl_le_timer_channel_loc!(1, 0, 'A', 1);
impl_le_timer_channel_loc!(1, 1, 'A', 2);
impl_le_timer_channel_loc!(1, 2, 'A', 3);
impl_le_timer_channel_loc!(1, 3, 'A', 4);
impl_le_timer_channel_loc!(1, 4, 'A', 5);
impl_le_timer_channel_loc!(1, 5, 'B', 11);
impl_le_timer_channel_loc!(1, 6, 'B', 12);
impl_le_timer_channel_loc!(1, 7, 'B', 13);
impl_le_timer_channel_loc!(1, 8, 'B', 14);
impl_le_timer_channel_loc!(1, 9, 'B', 15);
impl_le_timer_channel_loc!(1, 10, 'C', 6);
impl_le_timer_channel_loc!(1, 11, 'C', 7);
impl_le_timer_channel_loc!(1, 12, 'C', 8);
impl_le_timer_channel_loc!(1, 13, 'C', 9);
impl_le_timer_channel_loc!(1, 14, 'C', 10);
impl_le_timer_channel_loc!(1, 15, 'C', 11);
impl_le_timer_channel_loc!(1, 16, 'D', 9);
impl_le_timer_channel_loc!(1, 17, 'D', 10);
impl_le_timer_channel_loc!(1, 18, 'D', 11);
impl_le_timer_channel_loc!(1, 19, 'D', 12);
impl_le_timer_channel_loc!(1, 20, 'D', 13);
impl_le_timer_channel_loc!(1, 21, 'D', 14);
impl_le_timer_channel_loc!(1, 22, 'D', 15);
impl_le_timer_channel_loc!(1, 23, 'F', 0);
impl_le_timer_channel_loc!(1, 24, 'F', 1);
impl_le_timer_channel_loc!(1, 25, 'F', 2);
impl_le_timer_channel_loc!(1, 26, 'F', 3);
impl_le_timer_channel_loc!(1, 27, 'F', 4);
impl_le_timer_channel_loc!(1, 28, 'F', 5);
impl_le_timer_channel_loc!(1, 29, 'F', 6);
impl_le_timer_channel_loc!(1, 30, 'F', 7);
impl_le_timer_channel_loc!(1, 31, 'A', 0);
