//! Async timer

use crate::acmp::{AcmpId, AcmpInstance, Interrupts};
use core::sync::atomic::AtomicU32;
use defmt::warn;
use embassy_hal_internal::Peri;

static ACMP_WAKERS: [embassy_sync::waitqueue::AtomicWaker; AcmpId::COUNT] =
    [const { embassy_sync::waitqueue::AtomicWaker::new() }; _];
static ACMP_RAW_FLAGS: [AtomicU32; AcmpId::COUNT] = [const { AtomicU32::new(0) }; _];

pub(super) fn async_handler<const PN: u8>(flags: Interrupts) {
    let old = ACMP_RAW_FLAGS[PN as usize].swap(flags.into(), core::sync::atomic::Ordering::SeqCst);

    if old != 0 {
        warn!("ACMP: IRQ unhandled raw flags {}", old);
    }

    ACMP_WAKERS[PN as usize].wake();
}

pub(super) struct InterruptsFuture<'d, T: AcmpInstance> {
    peri: Peri<'d, T>,
}

impl<'d, T: AcmpInstance> InterruptsFuture<'d, T> {
    pub(super) fn new(peri: Peri<'d, T>) -> Self {
        Self { peri }
    }
}

impl<'d, T: AcmpInstance> core::future::Future for InterruptsFuture<'d, T> {
    type Output = Interrupts;

    fn poll(
        self: core::pin::Pin<&mut Self>,
        cx: &mut core::task::Context<'_>,
    ) -> core::task::Poll<Self::Output> {
        ACMP_WAKERS[self.peri.id() as usize].register(cx.waker());

        let raw_flags =
            ACMP_RAW_FLAGS[self.peri.id() as usize].swap(0, core::sync::atomic::Ordering::SeqCst);

        if raw_flags != 0 {
            core::task::Poll::Ready(Interrupts::new_unchecked(raw_flags))
        } else {
            core::task::Poll::Pending
        }
    }
}
