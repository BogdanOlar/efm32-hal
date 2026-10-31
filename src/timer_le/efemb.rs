//! Async LeTimer, and Embassy time driver implementation for LeTimer
//!

use crate::timer_le::{Interrupts, LeTimerId, LeTimerInstance};
use core::sync::atomic::{AtomicU32, Ordering};
use embassy_hal_internal::Peri;

static LETIMER_WAKERS: [embassy_sync::waitqueue::AtomicWaker; LeTimerId::COUNT] =
    [const { embassy_sync::waitqueue::AtomicWaker::new() }; _];
static LETIMER_RAW_FLAGS: [AtomicU32; LeTimerId::COUNT] = [const { AtomicU32::new(0) }; _];

pub(super) fn async_handler<const PN: u8>(flags: Interrupts) {
    // Accumulate flags until the future consumes them, so none are lost if several IRQs fire before it is polled
    LETIMER_RAW_FLAGS[PN as usize].fetch_or(flags.into(), Ordering::SeqCst);

    LETIMER_WAKERS[PN as usize].wake();
}

pub(super) struct InterruptsFuture<'d, T: LeTimerInstance> {
    peri: Peri<'d, T>,
}

impl<'d, T: LeTimerInstance> InterruptsFuture<'d, T> {
    pub(super) fn new(peri: Peri<'d, T>) -> Self {
        Self { peri }
    }
}

impl<'d, T: LeTimerInstance> core::future::Future for InterruptsFuture<'d, T> {
    type Output = Interrupts;

    fn poll(
        self: core::pin::Pin<&mut Self>,
        cx: &mut core::task::Context<'_>,
    ) -> core::task::Poll<Self::Output> {
        LETIMER_WAKERS[self.peri.id() as usize].register(cx.waker());

        let raw_flags = LETIMER_RAW_FLAGS[self.peri.id() as usize].swap(0, Ordering::SeqCst);

        if raw_flags != 0 {
            core::task::Poll::Ready(Interrupts::new_unchecked(raw_flags))
        } else {
            core::task::Poll::Pending
        }
    }
}

/// Embassy time driver, when one of the LeTimer peripherals is used as the time driver
#[cfg(feature = "_timdrv-letim")]
pub(crate) mod time_driver {
    use crate::{
        pac::interrupt,
        timer_le::{
            ChannelId, Config, Interrupt, InterruptConfig, Interrupts, LeTimerId, LeTimerRegs,
        },
    };
    use core::{
        cell::RefCell,
        sync::atomic::{AtomicBool, AtomicU32, Ordering},
    };
    use critical_section::{CriticalSection, Mutex};
    use embassy_time_driver::Driver;
    use embassy_time_queue_utils::Queue;

    /// Embassy time driver using LeTimer<PN> peripheral
    pub struct Ticker<const PN: u8> {
        /// Number of times the timer counter overflowed
        /// LeTimer is actually count-down, but we're calling this `overflow` just to avoid confusion
        ovf_count: AtomicU32,
        /// Initialization flag. If the ticker is not initialized, then [`Ticker::now()`] will just return `0`
        is_init: AtomicBool,
        /// Waker queue
        queue: Mutex<RefCell<Queue>>,
    }

    impl<const PN: u8> Ticker<PN> {
        /// Initialize the embassy time driver.
        pub fn init() {
            let timer = Self::timer();

            timer.enable_clock();
            timer.configure(&Config {
                count: u16::MAX,
                comp0: u16::MAX,
                comp1: u16::MAX,
                interrupts: InterruptConfig {
                    flags: Interrupts::new(Interrupt::Underflow).with_flag(Interrupt::Comp0),
                    // LeTimer IRQ is handled by the time driver itself
                    handler: None,
                },
                ..Default::default()
            });

            // Enable the timer interrupt
            unsafe {
                cortex_m::peripheral::NVIC::unmask(crate::pac::Interrupt::LETIMER0);
            }

            // start the timer
            timer.start();

            // Mark the timer as initialized
            EFEMB_TIME_DRIVER.is_init.store(true, Ordering::Relaxed);
        }

        /// Handle LeTimer interrupt
        pub fn on_interrupt(&self) {
            let timer = Self::timer();

            critical_section::with(|cs| {
                if timer.get_interrupt(Interrupt::Underflow) {
                    self.ovf_count.fetch_add(1, Ordering::Relaxed);
                    timer.clear_interrupt(Interrupt::Underflow);
                }

                if timer.get_interrupt(Interrupt::Comp0) {
                    timer.clear_interrupt(Interrupt::Comp0);
                }

                if timer.get_interrupt(Interrupt::Comp1) {
                    timer.clear_interrupt(Interrupt::Comp1);
                    timer.disable_interrupt(Interrupt::Comp1);
                }

                let mut queue = self.queue.borrow(cs).borrow_mut();

                loop {
                    let next = queue.next_expiration(self.checked_now(&cs));

                    if self.set_alarm(&cs, self.checked_now(&cs), next) {
                        break;
                    }
                }
            });
        }

        /// Set an interrupt to trigger at the given `next` logical time, relative to the current `now` logical time
        fn set_alarm(&self, _cs: &CriticalSection, now: u64, next: u64) -> bool {
            let timer = Self::timer();

            if now >= next {
                // alarm has already expired
                false
            } else {
                let now_count = (now & u16::MAX as u64) as u16;
                let rem_of_count = u16::MAX - now_count;
                let dif = next - now;

                if dif < rem_of_count as u64 {
                    // alarm will occur before next timer overflow
                    let cnt = next % (u16::MAX as u64 + 1);

                    timer.set_compare(ChannelId::Ch0, u16::MAX - (cnt as u16));
                    timer.enable_interrupt(Interrupt::Comp0);
                } else {
                    // we'll set the alarm on one of the next timer overflows
                    timer.disable_interrupt(Interrupt::Comp0);
                }

                true
            }
        }

        /// Get the current logical time while in a critical section.
        ///
        /// Useful because while in a critical section [`Ticker::ovf_count`] will not be incremented, so calling
        /// [`Driver::now()`] would return an incorect (smaller) time, whic breaks the 'monotonic' timer spec.
        ///
        /// This will increment the overflow counter is the underflow flag is set.
        fn checked_now(&self, _cs: &CriticalSection) -> u64 {
            let timer = Self::timer();

            let ovf = if timer.get_interrupt(Interrupt::Underflow) {
                timer.clear_interrupt(Interrupt::Underflow);

                self.ovf_count.fetch_add(1, Ordering::Relaxed) + 1
            } else {
                self.ovf_count.load(Ordering::Relaxed)
            };

            let counter = u16::MAX - timer.count();
            (ovf as u64) << u16::BITS | counter as u64
        }

        /// Get the LeTimer registers
        const fn timer() -> LeTimerRegs {
            match PN {
                0 => LeTimerRegs::new(LeTimerId::LeTimer0),
                _ => unreachable!(),
            }
        }
    }

    impl<const PN: u8> Driver for Ticker<PN> {
        fn now(&self) -> u64 {
            if self.is_init.load(Ordering::Relaxed) {
                let timer = Self::timer();
                loop {
                    let ovf_before = self.ovf_count.load(Ordering::SeqCst);
                    // An underflow that the interrupt hasn't counted yet (e.g. it is held off by a
                    // critical section) still has to be counted, or time would jump back one period
                    let uf_before = timer.get_interrupt(Interrupt::Underflow);
                    let counter = u16::MAX - timer.count();
                    let uf = timer.get_interrupt(Interrupt::Underflow);
                    let ovf = self.ovf_count.load(Ordering::SeqCst);
                    // Retry if the interrupt ran, or an underflow happened, while reading the counter
                    if ovf_before == ovf && uf_before == uf {
                        break ((ovf + uf as u32) as u64) << u16::BITS | counter as u64;
                    }
                }
            } else {
                // Ticker not initialized
                0
            }
        }

        fn schedule_wake(&self, at: u64, waker: &core::task::Waker) {
            let timer = Self::timer();

            critical_section::with(|cs| {
                let mut queue = self.queue.borrow(cs).borrow_mut();
                if queue.schedule_wake(at, waker) {
                    // Use the Comp1 interrupt flag to trigger the [`Ticker::on_interrupt()`] execution, which will handle
                    // setting the timer alarm.
                    timer.enable_interrupt(Interrupt::Comp1);
                    timer.set_interrupt(Interrupt::Comp1);
                }
            });
        }
    }

    #[cfg(feature = "_timdrv-letim0")]
    #[interrupt]
    fn LETIMER0() {
        EFEMB_TIME_DRIVER.on_interrupt();
    }

    #[cfg(feature = "_timdrv-letim0")]
    embassy_time_driver::time_driver_impl!(static EFEMB_TIME_DRIVER: Ticker<0> = Ticker {
        ovf_count:AtomicU32::new(0),
        is_init:AtomicBool::new(false),
        queue: Mutex::new(RefCell::new(Queue::new())),
    });
}
