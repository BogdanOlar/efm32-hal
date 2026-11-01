//! Async timer

use crate::timer::{Interrupts, TimerId, TimerInstance};
use core::sync::atomic::AtomicU32;
use embassy_hal_internal::Peri;

static TIMER_WAKERS: [embassy_sync::waitqueue::AtomicWaker; TimerId::COUNT] =
    [const { embassy_sync::waitqueue::AtomicWaker::new() }; _];
static TIMER_RAW_FLAGS: [AtomicU32; TimerId::COUNT] = [const { AtomicU32::new(0) }; _];

pub(super) fn async_handler<const PN: u8>(flags: Interrupts) {
    // Accumulate flags until the future consumes them, so none are lost if several IRQs fire before it is polled
    TIMER_RAW_FLAGS[PN as usize].fetch_or(flags.into(), core::sync::atomic::Ordering::SeqCst);

    TIMER_WAKERS[PN as usize].wake();
}

pub(super) struct InterruptsFuture<'d, T: TimerInstance> {
    peri: Peri<'d, T>,
}

impl<'d, T: TimerInstance> InterruptsFuture<'d, T> {
    pub(super) fn new(peri: Peri<'d, T>) -> Self {
        Self { peri }
    }
}

impl<'d, T: TimerInstance> core::future::Future for InterruptsFuture<'d, T> {
    type Output = Interrupts;

    fn poll(
        self: core::pin::Pin<&mut Self>,
        cx: &mut core::task::Context<'_>,
    ) -> core::task::Poll<Self::Output> {
        TIMER_WAKERS[self.peri.id() as usize].register(cx.waker());

        let raw_flags =
            TIMER_RAW_FLAGS[self.peri.id() as usize].swap(0, core::sync::atomic::Ordering::SeqCst);

        if raw_flags != 0 {
            core::task::Poll::Ready(Interrupts::new_unchecked(raw_flags))
        } else {
            core::task::Poll::Pending
        }
    }
}
