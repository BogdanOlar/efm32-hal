//! Timer IRQs
//!

#[cfg(feature = "efemb")]
use crate::timer::Interrupts;
use crate::{
    pac::{interrupt, timer::regs::Ifs},
    timer::{TimerId, TimerIrqHandler},
};
use core::cell::RefCell;
use critical_section::Mutex;

/// Handler which does nothing
pub(crate) const fn default_handler(_: Interrupts, _: crate::pac::timer::Timer) {}

/// Interrupt handlers for timers
pub(crate) static TIMER_HANDLER: [Mutex<RefCell<TimerIrqHandler>>; TimerId::COUNT] = [
    Mutex::new(RefCell::new(default_handler)),
    Mutex::new(RefCell::new(default_handler)),
];

fn timer_regs<const PN: u8>() -> crate::pac::timer::Timer {
    match PN {
        0 => crate::pac::TIMER0,
        1 => crate::pac::TIMER1,
        _ => unreachable!(),
    }
}

#[interrupt]
fn TIMER0() {
    base_irq_handler::<0>();
}

#[interrupt]
fn TIMER1() {
    base_irq_handler::<1>();
}

fn base_irq_handler<const PN: u8>() {
    let regs = timer_regs::<PN>();
    let flags = Interrupts::new_unchecked(regs.if_().read().0);

    // clear interrupt flags
    regs.ifc().write_value(Ifs(flags.into()));

    #[cfg(feature = "efemb")]
    super::efemb::async_handler::<PN>(flags);

    critical_section::with(|cs| {
        TIMER_HANDLER[PN as usize].borrow(cs).borrow()(flags, timer_regs::<PN>())
    });
}
