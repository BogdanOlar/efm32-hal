//! efm32-hal
//!
//! ## Feature flags
#![doc = document_features::document_features!()]
//!
#![no_std]
#![warn(missing_docs)]
#![warn(clippy::missing_safety_doc)]
#![cfg_attr(docsrs, feature(doc_cfg))]
// #![warn(clippy::undocumented_unsafe_blocks)]

pub use efm32_pac as pac;

pub mod acmp;
pub mod cmu;
pub mod crc;
pub mod dma;
pub mod gpio;
pub mod prs;
pub mod rtcc;
pub mod timer;
pub mod timer_le;
pub mod usart;

mod sealed {
    /// Sealed (typestate) marker trait for singleton types.
    /// Used to ensure that certain types may not be instantiated outside this crate.
    pub trait Sealed {}
}

pub(crate) use sealed::Sealed;

// Generate the HAL peripheral singletons. Corresponding drivers can use these to create driver instances.
embassy_hal_internal::peripherals!(
    Acmp0, Adc, Idac, Gpio, I2c, Usart0, Usart1, Timer0, Timer1, Gpcrc, Cryotimer, Rtcc, LeTimer0,
    LeUart0, Pcnt, Wdog0, Msc, Fpueh, Ldma, Emu, Cmu, Rmu, Prs, Crypto,
);

pub use embassy_hal_internal::{Peri, PeripheralType};

/// Initialize the HAL and return the (HAL) peripheral singletons.
///
/// This may be called only once; a second call panics.
pub fn init() -> Peripherals {
    Peripherals::take()
}

/// Convenience module which exports the most used types for each module
pub mod prelude {
    pub use crate::{
        cmu::{Cmu, HfClockPrescaler, HfClockSource, LfClockSource},
        gpio::{
            pin::mode::{
                Analog, Disabled, DisabledPu, InFilt, InFloat, InPd, InPdFilt, InPu, InPuFilt,
                OutOd, OutOdAlt, OutOdFilt, OutOdFiltAlt, OutOdPu, OutOdPuAlt, OutOdPuFilt,
                OutOdPuFiltAlt, OutOs, OutOsPd, OutPp, OutPpAlt,
            },
            port::{DataInCtrl, DriveStrength},
            Gpio, GpioError,
        },
        usart::spi::{dma::Spi, BitOrder, Config, SpiError, SpiParts},
    };
    pub use efm32_pac as pac;
    pub use embedded_hal::{
        delay::DelayNs,
        digital::{InputPin, OutputPin, PinState, StatefulOutputPin},
        pwm::SetDutyCycle,
        spi::{self, SpiBus},
    };
}

/// Peripheral single-cycle read-modify-write
///
/// The EFM32 Gecko supports bit set and bit clear access to all peripherals except those listed in
/// Table 4.1 Peripherals that Do Not Support Bit Set and Bit Clear on page 38. The bit set and bit clear functionality
/// (also called Bit Access) enables modification of bit fields (single bit or multiple bit wide) without the need to
/// perform a read-modify-write (though it is functionally equivalent). Also, the operation is contained within a single
/// bus access (for HF peripherals), unlike the Bit-banding operation described in section 4.2.2 Bit-banding which
/// consumes two bus accesses per operation. All AHB masters can utilize this feature.
///
/// See [Documentation](../../doc/efm32pg1-rm.pdf#page919)
trait SingleCycleRMW {
    const BIT_CLEAR_BASE_ADDR: usize = 0x44000000;
    const BIT_SET_BASE_ADDR: usize = 0x46000000;
    const PERIPHERALS_BASE_ADDR: usize = 0x40000000;

    /// Single cycle bit(s) set
    ///
    /// **WARNING**: don't use this for **EMU**, **RMU**, and **CRYOTIMER** peripheral registers!
    fn sc_set(&self, mask: u32);

    /// Single cycle bit(s) clear
    ///
    /// **WARNING**: don't use this for **EMU**, **RMU**, and **CRYOTIMER** peripheral registers!
    fn sc_clear(&self, mask: u32);
}

impl<T: Copy, A: crate::pac::common::Access> SingleCycleRMW for crate::pac::common::Reg<T, A> {
    fn sc_set(&self, mask: u32) {
        let addr = Self::BIT_SET_BASE_ADDR + (self.as_ptr().addr() - Self::PERIPHERALS_BASE_ADDR);
        unsafe { (addr as *mut u32).write_volatile(mask) };
    }

    fn sc_clear(&self, mask: u32) {
        let addr = Self::BIT_CLEAR_BASE_ADDR + (self.as_ptr().addr() - Self::PERIPHERALS_BASE_ADDR);
        unsafe { (addr as *mut u32).write_volatile(mask) };
    }
}
