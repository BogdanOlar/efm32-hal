//! Acmp IRQs
//!

#[cfg(feature = "efemb")]
use crate::acmp::Interrupts;
use crate::{
    acmp::{AcmpId, AcmpIrqHandler},
    pac::{acmp::regs::Ifs, interrupt},
};
use core::cell::RefCell;
use critical_section::Mutex;

/// Handler which does nothing
pub(crate) const fn default_handler(_: Interrupts, _: crate::pac::acmp::Acmp) {}

/// Interrupt handlers for timers
pub(crate) static ACMP_HANDLER: [Mutex<RefCell<AcmpIrqHandler>>; AcmpId::COUNT] =
    [Mutex::new(RefCell::new(default_handler))];

fn acmp_regs<const PN: u8>() -> crate::pac::acmp::Acmp {
    match PN {
        0 => crate::pac::ACMP0,
        _ => unreachable!(),
    }
}

#[interrupt]
fn ACMP0() {
    base_irq_handler::<0>();
}

fn base_irq_handler<const PN: u8>() {
    let regs = acmp_regs::<PN>();
    let flags = Interrupts::new_unchecked(regs.if_().read().0);

    // clear interrupt flags
    regs.ifc().write_value(Ifs(flags.into()));

    #[cfg(feature = "efemb")]
    super::efemb::async_handler::<PN>(flags);

    critical_section::with(|cs| {
        ACMP_HANDLER[PN as usize].borrow(cs).borrow()(flags, acmp_regs::<PN>())
    });
}
