#![no_main]
#![no_std]

use efm32_hal::cmu::{Cmu, HfClockPrescaler, HfClockSource};
use embassy_executor::Spawner;
use embassy_time::Timer;
use {defmt_rtt as _, panic_probe as _};

const TASK_COUNT: usize = 10;

#[embassy_executor::main]
async fn main(spawner: Spawner) {
    // Also configures the LF clock, and starts the embassy time driver
    let p = efm32_hal::init();
    let _clocks = Cmu::new(p.Cmu)
        // Prescaling the HF clock to the lowest frequency possible, to stress test the scheduler algorithm
        // .with_hf_clk(HfClockSource::HfRco, HfClockPrescaler::Div32)
        // .with_hf_clk(HfClockSource::HfXO(40_000_000), HfClockPrescaler::Div1);
        .with_hf_clk(HfClockSource::HfRco, HfClockPrescaler::Div1)
        .freeze();

    // Timestamps should be non-zero, monotonically incremented
    defmt::info!("\tHello info world!");
    defmt::warn!("\tHello warn world!");
    defmt::error!("\tHello error world!");
    defmt::trace!("\tHello trace world!");

    for i in 0..TASK_COUNT {
        spawner.spawn(my_task(i).expect("Could not spawn Task"));
    }
}

#[embassy_executor::task(pool_size = TASK_COUNT)]
async fn my_task(task_number: usize) {
    let mut counter: u64 = 0;

    loop {
        Timer::after_millis(1000).await;
        defmt::info!("Task {} ping {}", task_number, counter);
        counter += 1;
    }
}
