//! Build with `cargo build --example timer --features="defmt"`

#![no_main]
#![no_std]

use cortex_m_rt::entry;
use efm32_hal::{
    cmu::{Cmu, LfClockSource},
    gpio::{Gpio, OutPp},
    init,
    timer::{Timer, TimerDivider},
    timer_le::LeTimerExt,
};

use embedded_hal::{delay::DelayNs, digital::StatefulOutputPin};
// pick a panicking behavior
use panic_halt as _; // you can put a breakpoint on `rust_begin_unwind` to catch panics
                     // use panic_abort as _; // requires nightly
                     // use panic_itm as _; // logs messages over ITM; requires ITM support
                     // use panic_semihosting as _; // logs messages to the host stderr; requires a debugger
use defmt::println;
use defmt_rtt as _;

#[entry]
fn main() -> ! {
    let _core_p = cortex_m::Peripherals::take().unwrap();
    let p = init();
    let clocks = Cmu::new(p.Cmu).with_lfa_clk(LfClockSource::LfRco).freeze();
    let gpio = Gpio::new(p.Gpio);

    let mut pin_delay = gpio.pd14.into_mode::<OutPp>();

    let timer = Timer::new(p.Timer0, TimerDivider::Div1024);
    let (tim0ch0, _tim0ch1, _tim0ch2, _tim0ch3) = timer.into_channels();
    let mut delayer = tim0ch0.into_delay(&clocks);
    println!("{}", &delayer);

    let pin_pwm = gpio.pd13.into_mode::<OutPp>();
    let _pwm = p.Letimer.into_timer().into_ch0_pwm(pin_pwm);

    let is_le_timer_running = efm32_hal::pac::LETIMER.status().read().running();
    println!("is_le_timer_running: {}", &is_le_timer_running);

    let mut seconds: u32 = 0;
    let mut percent = 0;
    loop {
        if seconds > 8 {
            seconds = 0;
        } else {
            seconds += 2;
        }

        println!("Delay {} seconds, pwm {} %", seconds, percent);

        // let _ = pwm.set_duty_cycle_percent(percent);
        percent = if percent < 100 { percent + 10 } else { 0 };

        let _ = pin_delay.toggle();
        delayer.delay_ms(seconds * 1_000);
    }
}
