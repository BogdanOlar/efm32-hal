//! Async RtTimer, and Embassy time driver implementation for the RTCC
//!

use crate::rtcc::Interrupts;
use core::sync::atomic::{AtomicU32, Ordering};

static RTCC_WAKER: embassy_sync::waitqueue::AtomicWaker =
    embassy_sync::waitqueue::AtomicWaker::new();
static RTCC_RAW_FLAGS: AtomicU32 = AtomicU32::new(0);

/// Async handler for the RTCC
#[allow(dead_code)]
pub(super) fn async_handler(flags: Interrupts) {
    // Accumulate flags until the future consumes them, so none are lost if several IRQs fire before it is polled
    RTCC_RAW_FLAGS.fetch_or(flags.into(), Ordering::SeqCst);

    RTCC_WAKER.wake();
}

/// Wait for (enabled) RTCC interrupts
pub(super) async fn interrupts() -> Interrupts {
    core::future::poll_fn(|cx| {
        RTCC_WAKER.register(cx.waker());

        let raw_flags = RTCC_RAW_FLAGS.swap(0, Ordering::SeqCst);

        if raw_flags != 0 {
            core::task::Poll::Ready(Interrupts::new_unchecked(raw_flags))
        } else {
            core::task::Poll::Pending
        }
    })
    .await
}

#[cfg(feature = "_timdrv-rtcc")]
pub(crate) mod time_driver {
    //! Embassy time driver, when the RTCC is used as the time driver
    //!
    //! The 32 bit RTCC counter is extended to 64 bits by counting half-periods: [`Ticker::period`] is incremented
    //! both on counter overflow (`0xFFFF_FFFF -> 0`) and when the counter reaches `0x8000_0000` (CC1 match).
    //! The parity of `period` tells which half of the counter range we _should_ be in, which lets [`Driver::now()`]
    //! detect (and correct for) a wrap whose interrupt hasn't been handled yet. See [`Ticker::calc_now()`].
    //!
    //! The only requirement is that the RTCC interrupt is never delayed by more than half a period
    //! (i.e. ~18h at 32.768 kHz).

    use crate::{
        pac::interrupt,
        peripherals,
        rtcc::{
            ChannelConfig, ChannelId, ChannelMode, Config, Interrupt, InterruptConfig, Interrupts,
            RtTimer,
        },
    };
    use core::{
        cell::{Cell, RefCell},
        sync::atomic::{AtomicBool, AtomicU32, Ordering, compiler_fence},
    };
    use critical_section::{CriticalSection, Mutex};
    use embassy_hal_internal::Peri;
    use embassy_time_driver::Driver;
    use embassy_time_queue_utils::Queue;

    /// Embassy time driver using the RTCC peripheral
    pub(crate) struct Ticker {
        /// RTCC peripheral, see [`Self::timer()`]
        peri: Peri<'static, peripherals::Rtcc>,
        /// Number of elapsed half-periods (`0x8000_0000` ticks each).
        period: AtomicU32,
        /// Timestamp of the next alarm, `u64::MAX` if no alarm is set
        alarm_at: Mutex<Cell<u64>>,
        /// Initialization flag. If the ticker is not initialized, then [`Ticker::now()`] will just return `0`
        is_init: AtomicBool,
        /// Waker queue
        queue: Mutex<RefCell<Queue>>,
    }

    impl Ticker {
        /// Counter value at which the CC1 (half-period) interrupt triggers
        const HALF_PERIOD: u32 = 0x8000_0000;

        /// Channel used for the alarm (CC1 is used for the half-period interrupt)
        const ALARM_CHANNEL: ChannelId = ChannelId::Ch0;

        /// An alarm is only armed (CC0 interrupt enabled) once it is closer than this many ticks.
        ///
        /// CC0 only matches the low 32 bits, so arming earlier than one full period ahead would cause an early match.
        /// [`Ticker::next_period()`] runs every `0x8000_0000` ticks, so a `0xC000_0000` window guarantees the alarm
        /// gets armed at least `0x4000_0000` ticks before it's due.
        const ARM_WINDOW: u64 = 0xC000_0000;

        /// Uninitialized time driver, see [`Ticker::start()`]
        const fn new(peri: Peri<'static, peripherals::Rtcc>) -> Self {
            Self {
                peri,
                period: AtomicU32::new(0),
                alarm_at: Mutex::new(Cell::new(u64::MAX)),
                is_init: AtomicBool::new(false),
                queue: Mutex::new(RefCell::new(Queue::new())),
            }
        }

        /// Initialize the embassy time driver
        pub(crate) fn init() {
            EFEMB_TIME_DRIVER.start();
        }

        /// Configure and start the timer
        fn start(&self) {
            let mut timer = self.timer();
            timer.set_config(&Config {
                // The tick rate is set by the `LFECLK` frequency (see `crate::init()`)
                prescaler: crate::rtcc::CntPresc::Div1,
                // The half-period scheme needs the counter to wrap at `0xFFFF_FFFF`
                ccv1_top: false,
                count: 0,
                channels: [
                    // Alarm channel (CCV is set by `set_alarm`)
                    ChannelConfig {
                        mode: ChannelMode::Compare,
                        ..Default::default()
                    },
                    // Half-period channel
                    ChannelConfig {
                        mode: ChannelMode::Compare,
                        value: Self::HALF_PERIOD,
                        ..Default::default()
                    },
                    Default::default(),
                ],
                interrupts: InterruptConfig {
                    // CC0 (alarm) interrupt stays disabled until an alarm is armed
                    flags: Interrupts::new(Interrupt::Overflow).with_flag(Interrupt::Cc1),
                    // RTCC IRQ is handled by the time driver itself
                    handler: None,
                },
                ..Default::default()
            });

            // start the timer
            timer.start();

            // Mark the timer as initialized
            self.is_init.store(true, Ordering::Relaxed);
        }

        /// Handle RTCC interrupt
        fn on_interrupt(&self) {
            let mut timer = self.timer();

            critical_section::with(|cs| {
                // Both just advance the half-period count, so the order in which they're handled doesn't matter
                if timer.get_interrupt(Interrupt::Overflow) {
                    timer.clear_interrupt(Interrupt::Overflow);
                    self.next_period(cs);
                }

                if timer.get_interrupt(Interrupt::Cc1) {
                    timer.clear_interrupt(Interrupt::Cc1);
                    self.next_period(cs);
                }

                // Timer interrupt was triggered by an alarm (which will be treated below)
                if timer.get_interrupt(Interrupt::Cc0) {
                    timer.clear_interrupt(Interrupt::Cc0);
                }

                // Alarm triggered
                if self.now() >= self.alarm_at.borrow(cs).get() {
                    timer.disable_interrupt(Interrupt::Cc0);
                    self.alarm_at.borrow(cs).set(u64::MAX);

                    let mut queue = self.queue.borrow(cs).borrow_mut();
                    let mut next = queue.next_expiration(self.now());
                    while !self.set_alarm(cs, next) {
                        next = queue.next_expiration(self.now());
                    }
                }
            });
        }

        /// Advance the half-period count, and arm the alarm if it's due within [`Self::ARM_WINDOW`]
        fn next_period(&self, cs: CriticalSection) {
            let mut timer = self.timer();
            let period = self.period.fetch_add(1, Ordering::Relaxed) + 1;
            let now = (period as u64) << 31;

            if self.alarm_at.borrow(cs).get() < now + Self::ARM_WINDOW {
                // Any pending CC0 flag is usually from an earlier, unrelated match, since an alarm armed here is at
                // least `0x4000_0000` ticks away (otherwise it'd have been armed on the previous half-period).
                // An alarm armed by [`Ticker::set_alarm()`] may be due already, though, and its flag gets cleared too:
                // that's fine since [`Ticker::on_interrupt()`] checks the alarm against `now` after this.
                timer.clear_interrupt(Interrupt::Cc0);
                timer.enable_interrupt(Interrupt::Cc0);
            }
        }

        /// Set an interrupt to trigger at the given `at` logical time.
        ///
        /// Returns `false` if `at` is already in the past, in which case no alarm is set.
        fn set_alarm(&self, cs: CriticalSection, at: u64) -> bool {
            let mut timer = self.timer();
            let alarm = self.alarm_at.borrow(cs);

            let now = self.now();
            if at <= now {
                // alarm has already expired
                timer.disable_interrupt(Interrupt::Cc0);
                alarm.set(u64::MAX);
                return false;
            }

            // The counter counts up, so the low 32 bits of `at` are the `CCV` value
            timer.set_compare(Self::ALARM_CHANNEL, at as u32);
            timer.clear_interrupt(Interrupt::Cc0);

            if at - now < Self::ARM_WINDOW {
                timer.enable_interrupt(Interrupt::Cc0);
            } else {
                // [`Ticker::next_period()`] will arm it once it gets within range
                timer.disable_interrupt(Interrupt::Cc0);
            }

            // The counter may have passed `at` while the alarm was being set, in which case the CC0 match was missed
            if at <= self.now() {
                timer.disable_interrupt(Interrupt::Cc0);
                alarm.set(u64::MAX);
                return false;
            }

            alarm.set(at);
            true
        }

        /// Compute the logical time from the half-period count and the counter value
        ///
        /// If the counter is in the "wrong" half for the `period` parity, the corresponding interrupt hasn't been
        /// handled yet, and XOR-ing the top bit compensates for it:
        /// * even `period`, `counter >= 0x8000_0000`: half-period interrupt pending, counter is used as-is
        /// * odd `period`, `counter < 0x8000_0000`: overflow interrupt pending, counter gets `0x8000_0000` added (i.e.
        ///   next period)
        ///
        /// Interrupts handled _late_ are fine as long as they're not delayed by more than half a period
        /// (~18h at 32.768 kHz).
        const fn calc_now(period: u32, counter: u32) -> u64 {
            ((period as u64) << 31) + ((counter ^ ((period & 1) << 31)) as u64)
        }

        /// Get a handle to the RtTimer driver
        ///
        /// The driver is a shared `static`, so it only gets `&self`, while most [`RtTimer`] methods need `&mut self`.
        /// The returned handle shares the peripheral with `self.peri`, which is fine since the handles are
        /// short-lived, and register accesses which need to be atomic are done in a critical section.
        fn timer(&self) -> RtTimer<'_> {
            RtTimer::new_uninit(unsafe { self.peri.clone_unchecked() })
        }
    }

    impl Driver for Ticker {
        fn now(&self) -> u64 {
            if !self.is_init.load(Ordering::Relaxed) {
                // Ticker not initialized
                return 0;
            }

            // `period` must be read before the counter, see [`calc_now()`]
            let period = self.period.load(Ordering::Relaxed);
            compiler_fence(Ordering::Acquire);
            let counter = self.timer().count();

            Self::calc_now(period, counter)
        }

        fn schedule_wake(&self, at: u64, waker: &core::task::Waker) {
            critical_section::with(|cs| {
                let mut queue = self.queue.borrow(cs).borrow_mut();
                if queue.schedule_wake(at, waker) {
                    let mut next = queue.next_expiration(self.now());
                    while !self.set_alarm(cs, next) {
                        next = queue.next_expiration(self.now());
                    }
                }
            });
        }
    }

    /// If embassy is using the RTCC as time driver, then it also takes over its interrupt
    #[interrupt]
    fn RTCC() {
        EFEMB_TIME_DRIVER.on_interrupt();
    }

    embassy_time_driver::time_driver_impl!(
        static EFEMB_TIME_DRIVER: Ticker = Ticker::new(unsafe { peripherals::Rtcc::steal() })
    );
}
