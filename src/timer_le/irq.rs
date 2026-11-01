//! LeTimer IRQs
//!

use crate::timer_le::{Interrupts, LeTimerId, LeTimerIrqHandler};
use core::cell::RefCell;
use critical_section::Mutex;

/// Handler which does nothing
pub(super) const fn default_handler(_: Interrupts, _: crate::pac::letimer::LeTimer) {}

/// Interrupt handlers for LE timers
pub(super) static LETIMER_HANDLER: [Mutex<RefCell<LeTimerIrqHandler>>; LeTimerId::COUNT] =
    [Mutex::new(RefCell::new(default_handler))];

mod handlers {
    use super::LETIMER_HANDLER;
    #[allow(unused_imports)]
    use crate::pac::interrupt;
    use crate::{pac::letimer::regs::Ifs, timer_le::Interrupts};

    fn letimer_regs<const PN: u8>() -> crate::pac::letimer::LeTimer {
        match PN {
            0 => crate::pac::LETIMER0,
            _ => unreachable!(),
        }
    }

    #[allow(dead_code)]
    fn base_irq_handler<const PN: u8>() {
        let regs = letimer_regs::<PN>();
        // Only handle enabled interrupts, so that flags which are polled by the user are left untouched
        let flags = Interrupts::new_unchecked(regs.if_().read().0 & regs.ien().read().0);

        // clear interrupt flags
        regs.ifc().write_value(Ifs(flags.into()));

        #[cfg(feature = "efemb")]
        crate::timer_le::efemb::async_handler::<PN>(flags);

        critical_section::with(|cs| {
            LETIMER_HANDLER[PN as usize].borrow(cs).borrow()(flags, letimer_regs::<PN>())
        });
    }

    #[cfg(not(feature = "_timdrv-letim0"))]
    #[interrupt]
    fn LETIMER0() {
        base_irq_handler::<0>();
    }
}
