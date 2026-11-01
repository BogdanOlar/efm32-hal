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

// Generate the HAL peripheral types
embassy_hal_internal::peripherals_definition!(
    Acmp0, Adc, Idac, Gpio, I2c, Usart0, Usart1, Timer0, Timer1, Gpcrc, Cryotimer, Rtcc, LeTimer0,
    LeUart0, Pcnt, Wdog0, Msc, Fpueh, Ldma, Emu, Cmu, Rmu, Prs, Crypto,
);

// Generate the HAL peripheral singletons. Corresponding drivers can use these to create driver instances.
embassy_hal_internal::peripherals_struct!(
    Acmp0,
    Adc,
    Idac,
    Gpio,
    I2c,
    Usart0,
    Usart1,
    Timer0,
    Timer1,
    Gpcrc,
    Cryotimer,
    Rtcc,
    // Only hand out the LeTimer0 HAL peripheral if it is **not** used for the embassy timer driver
    #[cfg(not(feature = "_timdrv-letim0"))]
    LeTimer0,
    LeUart0,
    Pcnt,
    Wdog0,
    Msc,
    Fpueh,
    Ldma,
    Emu,
    Cmu,
    Rmu,
    Prs,
    Crypto,
);

pub use embassy_hal_internal::{Peri, PeripheralType};

/// Initialize the HAL and return the (HAL) peripheral singletons.
///
/// This may be called only once; a second call panics.
pub fn init() -> Peripherals {
    // Return the HAL peripherals
    let peripherals = Peripherals::take();

    // FIXME: configure the clocks according to the caller's given config
    let cmu = crate::cmu::Cmu::new(unsafe { peripherals.Cmu.clone_unchecked() }).with_hf_clk(
        crate::cmu::HfClockSource::HfXO(40_000_000),
        crate::cmu::HfClockPrescaler::Div1,
    );

    // If embassy feature is enabled, initialize the timer driver for it

    // LeTimer0
    #[cfg(feature = "efemb-timdrv-letim0-hz-32_768")]
    let cmu = cmu.with_lfa_clk(crate::cmu::LfClockSource::LfRco);
    #[cfg(feature = "efemb-timdrv-letim0-hz-1_000")]
    let cmu = cmu.with_lfa_clk(crate::cmu::LfClockSource::UlfRco);
    #[cfg(feature = "_timdrv-letim0")]
    timer_le::efemb::time_driver::Ticker::<peripherals::LeTimer0>::init();

    // Compile-time error if no driver is selected
    #[cfg(all(feature = "efemb", not(feature = "_timdrv")))]
    compile_error!(
        "Embassy feature needs a timer driver specified with one of the `efemb-timdrv-*` features"
    );

    // Pacify clippy
    let _ = cmu.freeze();

    peripherals
}

/// Convenience module which exports the most used types for each module
pub mod prelude {
    pub use crate::{
        cmu::{Cmu, HfClockPrescaler, HfClockSource, LfClockSource},
        gpio::{
            Gpio, GpioError,
            pin::mode::{
                Analog, Disabled, DisabledPu, InFilt, InFloat, InPd, InPdFilt, InPu, InPuFilt,
                OutOd, OutOdAlt, OutOdFilt, OutOdFiltAlt, OutOdPu, OutOdPuAlt, OutOdPuFilt,
                OutOdPuFiltAlt, OutOs, OutOsPd, OutPp, OutPpAlt,
            },
            port::{DataInCtrl, DriveStrength},
        },
        usart::spi::{BitOrder, Config, SpiError, SpiParts, dma::Spi},
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

mod sealed {
    /// Sealed (typestate) marker trait for singleton types.
    /// Used to ensure that certain types may not be instantiated outside this crate.
    pub trait Sealed {}
}

pub(crate) use sealed::Sealed;
