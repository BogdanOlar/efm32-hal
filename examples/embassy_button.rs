#![no_std]
#![no_main]

use cortex_m::peripheral::NVIC;
use defmt_rtt as _;
use efm32_hal::{
    gpio::{dynamic::DynamicPin, efemb::AsyncInputPin},
    pac::Interrupt,
    prelude::*,
};
use embassy_executor::Spawner;
// required for defmt timestamps to work
use embassy_time as _;
use embedded_hal_async::digital::Wait;
use panic_halt as _;

#[embassy_executor::main]
async fn main(spawner: Spawner) {
    // Also initializes the embassy time driver, which provides the logging timestamps
    let p = efm32_hal::init();

    // Initialize GPIO
    let gpio = Gpio::new(p.Gpio);

    // ---- NVIC ----
    unsafe {
        NVIC::unmask(Interrupt::GPIO_EVEN);
        NVIC::unmask(Interrupt::GPIO_ODD);
    }

    // ---- Button 0 ----
    let led0 = gpio.pf4.into_mode::<OutPp>().into_dynamic_pin();
    let btn0 = gpio
        .pf6
        .into_mode::<InFloat>()
        .into_async_input(gpio.exti4ctrl);
    // start BTN0 task
    spawner.spawn(button_task(btn0, led0).expect("Could not spawn Task"));

    // ---- Button 1 ----
    let led1 = gpio.pf5.into_mode::<OutPpAlt>().into_dynamic_pin();
    let btn1 = gpio
        .pf7
        .into_mode::<InFilt>()
        .into_dynamic_pin()
        .try_into_async_input(gpio.exti5ctrl)
        .unwrap();
    // start BTN1 task
    spawner.spawn(button_task(btn1, led1).expect("Could not spawn Task"));

    defmt::info!("press BTN0 (PF6) or BTN1 (PF7)");
}

#[embassy_executor::task(pool_size = 2)]
async fn button_task(mut btn: AsyncInputPin, mut led: DynamicPin) {
    loop {
        // Wait for button press (active low)
        let _ = btn.wait_for_low().await;
        let _ = led.set_high();
        defmt::info!("{} pressed", &btn);

        // Wait for button release
        let _ = btn.wait_for_high().await;
        let _ = led.set_low();
        defmt::info!("{} released", &btn);
    }
}
