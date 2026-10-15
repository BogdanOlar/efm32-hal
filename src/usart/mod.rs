//! Universal Synchronous Asynchronous Receiver/Transmitter
//!
//! This module provides SPI drivers for the USART peripherals

pub mod spi;

use crate::{peripherals, Sealed};
use embassy_hal_internal::PeripheralType;

/// Identifies which USART peripheral a driver instance is bound to.
///
/// `Spi` is a specialisation of the USART peripheral, so this runtime identifier lives at the
/// `usart` module level. Drivers such as [`spi::Spi`](crate::usart::spi::Spi) store a `UsartId`
/// (rather than a raw `u8`) to make the peripheral selection self-documenting and exhaustive at
/// every `match`. The [`UsartInstance`] trait maps a peripheral singleton
/// (`[`peripherals::Usart0`]` / `[`peripherals::Usart1`]`) to its `UsartId` and register block.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[repr(u8)]
pub enum UsartId {
    /// USART0.
    Usart0 = 0,
    /// USART1.
    Usart1 = 1,
}

/// A USART peripheral instance usable by the HAL USART/SPI drivers.
///
/// Sealed and implemented only for the singleton types [`peripherals::Usart0`] and
/// [`peripherals::Usart1`], obtained from [`crate::efm32_init`]. Drivers such as
/// [`spi::Spi`](crate::usart::spi::Spi) are generic over `T: UsartInstance` and store the
/// peripheral singleton (via [`PeripheralRef`](embassy_hal_internal::PeripheralRef)), so the same
/// peripheral cannot be used to build two drivers.
pub trait UsartInstance: Sealed + PeripheralType + 'static {
    /// Returns the runtime [`UsartId`] for this instance (used e.g. for DMA source selection).
    fn id(&self) -> UsartId;
    /// Returns the chiptool PAC register-block handle for this USART instance.
    fn regs(&self) -> crate::pac::usart::Usart;
}

impl Sealed for peripherals::Usart0 {}
impl UsartInstance for peripherals::Usart0 {
    fn id(&self) -> UsartId {
        UsartId::Usart0
    }
    fn regs(&self) -> crate::pac::usart::Usart {
        crate::pac::USART0
    }
}

impl Sealed for peripherals::Usart1 {}
impl UsartInstance for peripherals::Usart1 {
    fn id(&self) -> UsartId {
        UsartId::Usart1
    }
    fn regs(&self) -> crate::pac::usart::Usart {
        crate::pac::USART1
    }
}
