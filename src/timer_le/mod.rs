//! Low Energy Timer
//!

#[cfg(feature = "efemb")]
pub mod efemb;

use crate::{
    Sealed,
    gpio::pin::Pin,
    pac::{
        CMU,
        letimer::vals::{Out0loc, Ufoa0},
    },
    peripherals,
};
use core::marker::PhantomData;
use cortex_m::asm::nop;
use embassy_hal_internal::Peri;
use embedded_hal::digital::OutputPin;

/// LeTimer peripheral ID
pub enum LeTimerId {
    /// Low Energy Timer 0
    LeTimer0,
}

/// A timer peripheral instance usable by the HAL timer driver.
pub trait LeTimerInstance: Sealed + embassy_hal_internal::PeripheralType + 'static {
    /// Returns the chiptool PAC register-block handle for this timer instance.
    fn regs(&self) -> crate::pac::letimer::LeTimer;
    /// Timer peripheral ID
    fn id(&self) -> LeTimerId;
}

// Only expose the LeTimer0 HAL peripheral if it is **not** used as the embassy ticker
#[cfg(not(feature = "_timdrv-letim0"))]
impl Sealed for peripherals::LeTimer0 {}

// Only expose the LeTimer0 HAL peripheral if it is **not** used as the embassy ticker
#[cfg(not(feature = "_timdrv-letim0"))]
impl LeTimerInstance for peripherals::LeTimer0 {
    fn regs(&self) -> crate::pac::letimer::LeTimer {
        crate::pac::LETIMER0
    }

    fn id(&self) -> LeTimerId {
        LeTimerId::LeTimer0
    }
}

/// Low Energy timer
/// Timer
#[derive(Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct LeTimer<'d, T: LeTimerInstance> {
    peri: Peri<'d, T>,
}

impl<'d, T: LeTimerInstance> LeTimer<'d, T> {
    /// Instantiate a LeTimer driver
    pub fn new(peri: Peri<'d, T>) -> Self {
        let instance = Self { peri };
        // Enable LE Timer
        CMU.lfaclken0()
            .modify(|w| w.set_letimer(instance.peri.id() as usize, true));

        // Sync
        while CMU.syncbusy().read().lfaclken0() {
            nop()
        }

        instance
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
