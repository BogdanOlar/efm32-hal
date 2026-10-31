//! Embassy time driver implementation for LeTimer
//!

#[cfg(any(feature = "_timdrv-letim0"))]
pub(crate) mod time_driver {
    use crate::{
        pac::{CMU, Interrupt, interrupt},
        timer_le::Interrupts,
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
        ///
        /// WARNING: LfAClk needs to be enabled, and the HfClk source clock needs to be one of the HF sources (i.e. not LF),
        ///          otherwise you'll get a bus fault.
        ///
        ///For example, if you want the timer driver to run at 32.768 kHz, make sure to enable the
        /// `efemb-timdrv-letim0-hz-32_768` crate feature, and initialize the clock sources
        ///
        /// ```rust,norun
        /// let _clocks = p
        ///     .cmu
        ///     .split()
        ///     .with_hf_clk(HfClockSource::HfRco, HfClockPrescaler::Div1)
        ///     .with_lfa_clk(LfClockSource::LfRco);
        /// ```
        /// For a 1 kKz timer, enable the `efemb-timdrv-letim0-hz-1_000` feature, and configure the LfAClk
        /// ```rust,norun
        /// let _clocks = p
        ///     // ...
        ///     .with_lfa_clk(LfClockSource::UlfRco);
        /// ```
        pub fn init() {
            let p = Self::p();

            // Enable LE Timer
            CMU.lfaclken0().modify(|w| w.set_letimer(PN as usize, true));

            // Stop the timer if it's running
            if p.status().read().running() {
                p.cmd().write(|w| w.set_stop(true));
            }

            // Manually checked RM that all fields default to 0 at reset/POR
            p.ctrl().write_value(Default::default());
            p.cnt().write(|w| w.set_cnt(u16::MAX));
            p.comp0().write(|w| w.set_comp0(u16::MAX));
            p.comp1().write(|w| w.set_comp1(u16::MAX));
            p.rep0().write(|w| w.set_rep0(0));
            p.rep1().write(|w| w.set_rep1(0));
            p.ien().write_value(crate::pac::letimer::regs::Ien(
                Interrupts::new(crate::timer_le::Interrupt::Underflow)
                    .with_flag(crate::timer_le::Interrupt::Comp0)
                    .into(),
            ));
            p.ifc()
                .write_value(crate::pac::letimer::regs::Ifs(Interrupts::ALL.into()));

            // Enable the timer interrupt
            unsafe {
                cortex_m::peripheral::NVIC::unmask(Interrupt::LETIMER0);
            }

            // start the timer
            p.cmd().write(|w| w.set_start(true));

            // Mark the timer as initialized
            EFEMB_TIME_DRIVER.is_init.store(true, Ordering::Relaxed);
        }

        /// Handle LeTimer0 interrupt
        pub fn on_interrupt(&self) {
            let p = Self::p();

            critical_section::with(|cs| {
                if p.if_().read().uf() {
                    self.ovf_count.fetch_add(1, Ordering::Relaxed);
                    p.ifc().write(|w| w.set_uf(true));
                }

                if p.if_().read().comp0() {
                    p.ifc().write(|w| w.set_comp0(true));
                }

                if p.if_().read().comp1() {
                    p.ifc().write(|w| w.set_comp1(true));
                    p.ien().write(|w| w.set_comp1(false));
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
            let p = Self::p();

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

                    p.comp0().write(|w| w.set_comp0(u16::MAX - (cnt as u16)));
                    p.ien().write(|w| w.set_comp0(true));
                } else {
                    // we'll set the alarm on one of the next timer overflows
                    p.ien().write(|w| w.set_comp0(false));
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
            let p = Self::p();

            let ovf = if p.if_().read().uf() {
                p.ifc().write(|w| w.set_uf(true));

                self.ovf_count.fetch_add(1, Ordering::Relaxed) + 1
            } else {
                self.ovf_count.load(Ordering::Relaxed)
            };

            let counter = u16::MAX - p.cnt().read().cnt();
            (ovf as u64) << u16::BITS | counter as u64
        }

        /// Get the LeTimer pac peripheral
        const fn p() -> crate::pac::letimer::LeTimer {
            match PN {
                0 => crate::pac::LETIMER0,
                _ => unreachable!(),
            }
        }
    }

    impl<const PN: u8> Driver for Ticker<PN> {
        fn now(&self) -> u64 {
            if self.is_init.load(Ordering::Relaxed) {
                let p = Self::p();
                loop {
                    let ovf_before = self.ovf_count.load(Ordering::SeqCst);
                    let counter = u16::MAX - p.cnt().read().cnt();
                    let ovf = self.ovf_count.load(Ordering::SeqCst);
                    if ovf_before == ovf {
                        break (ovf as u64) << u16::BITS | counter as u64;
                    }
                }
            } else {
                // Ticker not initialized
                0
            }
        }

        fn schedule_wake(&self, at: u64, waker: &core::task::Waker) {
            let p = Self::p();

            critical_section::with(|cs| {
                let mut queue = self.queue.borrow(cs).borrow_mut();
                if queue.schedule_wake(at, waker) {
                    // Use the Comp1 interrupt flag to trigger the [`Ticker::on_interrupt()`] execution, which will handle
                    // setting the timer alarm.
                    p.ien().write(|w| w.set_comp1(true));
                    p.ifs().write(|w| w.set_comp1(true));
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
