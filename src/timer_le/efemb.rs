//! Async LeTimer, and Embassy time driver implementation for LeTimer
//!

use crate::timer_le::{Interrupts, LeTimerId, LeTimerInstance};
use core::sync::atomic::{AtomicU32, Ordering};
use embassy_hal_internal::Peri;

static LETIMER_WAKERS: [embassy_sync::waitqueue::AtomicWaker; LeTimerId::COUNT] =
    [const { embassy_sync::waitqueue::AtomicWaker::new() }; _];
static LETIMER_RAW_FLAGS: [AtomicU32; LeTimerId::COUNT] = [const { AtomicU32::new(0) }; _];

/// Async handler for each LETIMER<PN>
#[allow(dead_code)]
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

#[cfg(feature = "_timdrv-letim")]
pub(crate) mod time_driver {
    //! Embassy time driver, when one of the LeTimer peripherals is used as the time driver
    //!
    //! The 16 bit LeTimer counter is extended to 64 bits by counting half-periods: [`Ticker::period`] is incremented
    //! both on timer underflow (tick count wraps `0xFFFF -> 0`) and when the tick count reaches `0x8000` (Comp1 match).
    //! The parity of `period` tells which half of the counter range we _should_ be in, which lets [`Driver::now()`]
    //! detect (and correct for) a wrap whose interrupt hasn't been handled yet. See [`Ticker::calc_now()`].
    //!
    //! The only requirement is that the LeTimer interrupt is never delayed by more than half a period
    //! (i.e. ~1s at 32.768 kHz, ~32s at 1 kHz).

    use crate::{
        pac::interrupt,
        timer_le::{
            ChannelId, Config, Interrupt, InterruptConfig, Interrupts, LeTimerId, LeTimerRegs,
            Repmode,
        },
    };
    use core::{
        cell::{Cell, RefCell},
        sync::atomic::{AtomicBool, AtomicU32, Ordering, compiler_fence},
    };
    use critical_section::{CriticalSection, Mutex};
    use embassy_time_driver::Driver;
    use embassy_time_queue_utils::Queue;

    /// Embassy time driver using LeTimer<PN> peripheral
    pub(crate) struct Ticker<const PN: u8> {
        /// Number of elapsed half-periods (`0x8000` ticks each).
        period: AtomicU32,
        /// Timestamp of the next alarm, `u64::MAX` if no alarm is set
        alarm_at: Mutex<Cell<u64>>,
        /// Initialization flag. If the ticker is not initialized, then [`Ticker::now()`] will just return `0`
        is_init: AtomicBool,
        /// Waker queue
        queue: Mutex<RefCell<Queue>>,
    }

    impl<const PN: u8> Ticker<PN> {
        /// Tick count (low 16 bits) at which the Comp1 (half-period) interrupt triggers
        const HALF_PERIOD: u16 = 0x8000;

        /// An alarm is only armed (Comp0 interrupt enabled) once it is closer than this many ticks.
        ///
        /// Comp0 only matches the low 16 bits, so arming earlier than one full period ahead would cause an early match.
        /// [`Ticker::next_period()`] runs every `0x8000` ticks, so a `0xC000` window guarantees the alarm gets armed at
        /// least `0x4000` ticks before it's due.
        const ARM_WINDOW: u64 = 0xC000;

        /// Uninitialized time driver, see [`Ticker::start()`]
        const fn new() -> Self {
            Self {
                period: AtomicU32::new(0),
                alarm_at: Mutex::new(Cell::new(u64::MAX)),
                is_init: AtomicBool::new(false),
                queue: Mutex::new(RefCell::new(Queue::new())),
            }
        }

        /// Configure and start the timer
        fn start(&self) {
            let timer = Self::timer();

            timer.enable_clock();
            timer.configure(&Config {
                // The half-period scheme needs the counter to wrap at `0xFFFF` and keep running
                repeat_mode: Repmode::Free,
                comp0_top: false,
                buffered_top: false,
                // Start at tick count `1`, so the first underflow only happens after a full count-down, when the tick
                // count wraps `0xFFFF -> 0` (the counter doesn't start at `0`, so it doesn't immediately wrap)
                count: u16::MAX,
                // u16::MAX - (ticks - 1)
                comp1: Self::HALF_PERIOD.wrapping_neg(),
                interrupts: InterruptConfig {
                    // Comp0 (alarm) interrupt stays disabled until an alarm is armed
                    flags: Interrupts::new(Interrupt::Underflow).with_flag(Interrupt::Comp1),
                    // LeTimer IRQ is handled by the time driver itself
                    handler: None,
                },
                ..Default::default()
            });

            // Enable the timer interrupt
            unsafe {
                cortex_m::peripheral::NVIC::unmask(timer.interrupt());
            }

            // start the timer
            timer.start();

            // Mark the timer as initialized
            self.is_init.store(true, Ordering::Relaxed);
        }

        /// Handle LeTimer interrupt
        fn on_interrupt(&self) {
            let timer = Self::timer();

            critical_section::with(|cs| {
                // Both just advance the half-period count, so the order in which they're handled doesn't matter
                if timer.get_interrupt(Interrupt::Underflow) {
                    timer.clear_interrupt(Interrupt::Underflow);
                    self.next_period(cs);
                }

                if timer.get_interrupt(Interrupt::Comp1) {
                    timer.clear_interrupt(Interrupt::Comp1);
                    self.next_period(cs);
                }

                // Timer interrupt was triggered by an alarm (which will be treated below)
                if timer.get_interrupt(Interrupt::Comp0) {
                    timer.clear_interrupt(Interrupt::Comp0);
                }

                // Alarm triggered
                if self.now() >= self.alarm_at.borrow(cs).get() {
                    timer.disable_interrupt(Interrupt::Comp0);
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
            let timer = Self::timer();
            let period = self.period.fetch_add(1, Ordering::Relaxed) + 1;
            let now = (period as u64) << 15;

            if self.alarm_at.borrow(cs).get() < now + Self::ARM_WINDOW {
                // Any pending Comp0 flag is usually from an earlier, unrelated match, since an alarm armed here is at
                // least `0x4000` ticks away (otherwise it'd have been armed on the previous half-period).
                // An alarm armed by [`Ticker::set_alarm()`] may be due already, though, and its flag gets cleared too:
                // that's fine since [`Ticker::on_interrupt()`] checks the alarm against `now` after this.
                timer.clear_interrupt(Interrupt::Comp0);
                timer.enable_interrupt(Interrupt::Comp0);
            }
        }

        /// Set an interrupt to trigger at the given `at` logical time.
        ///
        /// Returns `false` if `at` is already in the past, in which case no alarm is set.
        fn set_alarm(&self, cs: CriticalSection, at: u64) -> bool {
            let timer = Self::timer();
            let alarm = self.alarm_at.borrow(cs);

            let now = self.now();
            if at <= now {
                // alarm has already expired
                timer.disable_interrupt(Interrupt::Comp0);
                alarm.set(u64::MAX);
                return false;
            }

            // Convert the driver's tick count (low 16 bits) to a `CNT`/`COMPn` register value
            timer.set_compare(ChannelId::Ch0, (at as u16).wrapping_neg());
            timer.clear_interrupt(Interrupt::Comp0);

            if at - now < Self::ARM_WINDOW {
                timer.enable_interrupt(Interrupt::Comp0);
            } else {
                // [`Ticker::next_period()`] will arm it once it gets within range
                timer.disable_interrupt(Interrupt::Comp0);
            }

            // The counter may have passed `at` while the alarm was being set, in which case the Comp0 match was missed
            if at <= self.now() {
                timer.disable_interrupt(Interrupt::Comp0);
                alarm.set(u64::MAX);
                return false;
            }

            alarm.set(at);
            true
        }

        /// Compute the logical time from the half-period count and the tick count
        ///
        /// The `counter` is **not** the `CNT` value from the timer register, since LeTimer counts _down_.
        /// It's `CNT.wrapping_neg()`, i.e. the up-counter `u16::MAX - CNT` offset by `1`.
        ///
        /// The offset is needed because `UF` is set when `CNT` _reaches_ `0` (RM 19.3.3): it makes the tick count
        /// wrap `0xFFFF -> 0` exactly when `UF` is set. Without it, `UF` would be set one tick _before_ the wrap,
        /// and `now()` would jump ahead by almost a full period for that tick.
        ///
        /// If the counter is in the "wrong" half for the `period` parity, the corresponding interrupt hasn't been
        /// handled yet, and XOR-ing the top bit compensates for it:
        /// * even `period`, `counter >= 0x8000`: half-period interrupt pending, counter is used as-is
        /// * odd `period`, `counter < 0x8000`: underflow interrupt pending, counter gets `0x8000` added (i.e. next
        ///   period)
        ///
        /// Interrupts handled _late_ are fine as long as they're not delayed by more than half a period
        /// (~1s at 32.768 kHz, ~32s at 1 kHz).
        const fn calc_now(period: u32, counter: u16) -> u64 {
            ((period as u64) << 15) + ((counter as u32 ^ ((period & 1) << 15)) as u64)
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
            if !self.is_init.load(Ordering::Relaxed) {
                // Ticker not initialized
                return 0;
            }

            // `period` must be read before the counter, see [`calc_now()`]
            let period = self.period.load(Ordering::Relaxed);
            compiler_fence(Ordering::Acquire);
            // Convert a `CNT` register value to the driver's tick count (low 16 bits).
            //
            // LeTimer counts _down_, so ticks are counted from `CNT = 0`: the logical up-counter is `u16::MAX - CNT`,
            // offset by `1` because the underflow flag is set when `CNT` _reaches_ `0` (RM 19.3.3). This aligns the
            // underflow interrupt with the tick count wrapping `0xFFFF -> 0`
            let counter = Self::timer().count().wrapping_neg();

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

    #[cfg(feature = "_timdrv-letim0")]
    impl Ticker<0> {
        /// Initialize the embassy time driver
        pub(crate) fn init() {
            // Typed binding: if the static isn't a `Ticker<0>`, this doesn't compile
            let driver: &'static Ticker<0> = &EFEMB_TIME_DRIVER;
            driver.start();
        }
    }

    #[cfg(feature = "_timdrv-letim0")]
    #[interrupt]
    fn LETIMER0() {
        EFEMB_TIME_DRIVER.on_interrupt();
    }

    #[cfg(feature = "_timdrv-letim0")]
    embassy_time_driver::time_driver_impl!(static EFEMB_TIME_DRIVER: Ticker<0> = Ticker::new());
}
