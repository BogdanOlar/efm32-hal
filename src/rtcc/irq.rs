//! RTCC IRQ
//!

use crate::rtcc::{Interrupts, RtTimerIrqHandler};
use core::cell::RefCell;
use critical_section::Mutex;

/// Handler which does nothing
pub(super) const fn default_handler(_: Interrupts, _: crate::pac::rtcc::Rtcc) {}

/// Interrupt handler for the RTCC
pub(super) static RTCC_HANDLER: Mutex<RefCell<RtTimerIrqHandler>> =
    Mutex::new(RefCell::new(default_handler));

// If embassy is using the RTCC as time driver, then it also takes over its interrupt
#[cfg(not(feature = "_timdrv-rtcc"))]
mod handlers {
    use super::RTCC_HANDLER;
    use crate::{
        pac::{RTCC, interrupt, rtcc::regs::Ifs},
        rtcc::Interrupts,
    };

    #[interrupt]
    fn RTCC() {
        // Only handle enabled interrupts, so that flags which are polled by the user are left untouched
        let flags = Interrupts::new_unchecked(RTCC.if_().read().0 & RTCC.ien().read().0);

        // clear interrupt flags
        RTCC.ifc().write_value(Ifs(flags.into()));

        #[cfg(feature = "_efemb")]
        crate::rtcc::efemb::async_handler(flags);

        critical_section::with(|cs| RTCC_HANDLER.borrow(cs).borrow()(flags, RTCC));
    }
}
