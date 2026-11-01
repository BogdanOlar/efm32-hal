#![no_std]
#![no_main]

use defmt::{error, info};
use defmt_rtt as _;
use efm32_hal::dma::{Dma, DmaChannel};
use embassy_executor::Spawner;
// required for defmt timestamps to work
use embassy_time as _;
use panic_halt as _;

#[embassy_executor::main]
async fn main(spawner: Spawner) {
    let p = efm32_hal::init();

    // Initialize GPIO
    let dma = Dma::init(p.Ldma);

    // ---- Channel 0 ----
    spawner.spawn(transfer(dma.ch0).expect("Could not spawn Task"));

    // ---- Channel 1 ----
    spawner.spawn(transfer(dma.ch1).expect("Could not spawn Task"));

    info!("Starting transfers...");
}

#[embassy_executor::task(pool_size = 2)]
async fn transfer(mut ch: DmaChannel) {
    const BUF_UNIT_SIZE: usize = 0x400 * 10;

    // const TRANSFER_UNIT_COUNT: usize = 15;
    // const TRANSFER_UNIT_COUNT: usize = 32;
    // const TRANSFER_UNIT_COUNT: usize = 40;
    // const TRANSFER_UNIT_COUNT: usize = 70;
    // const TRANSFER_UNIT_COUNT: usize = 300;
    const TRANSFER_UNIT_COUNT: usize = 0x800 * 4 + 5;

    static SRC_U8: [u8; BUF_UNIT_SIZE] = {
        let mut seq = [0; BUF_UNIT_SIZE];
        let mut i = 0;
        // Fill the buffer with values from 1 to 255
        while i < BUF_UNIT_SIZE {
            seq[i] = 1 + (i % u8::MAX as usize) as u8;
            i += 1;
        }
        seq
    };
    let src = &SRC_U8[..TRANSFER_UNIT_COUNT];
    let mut dst_u8: [u8; BUF_UNIT_SIZE] = [0u8; _];

    // take an unaligned slice of `dst` so that the transfer is done with bytes, not half-words or words
    let dst = &mut dst_u8[1..TRANSFER_UNIT_COUNT + 1];

    // // Half-Word aligned transfer
    // let dst = &mut dst_u8[2..TRANSFER_UNIT_COUNT + 2];

    // // Word-aligned transfer
    // let dst = &mut dst_u8[0..TRANSFER_UNIT_COUNT];

    let transfer_result = ch.memory_transfer_async(src, dst).await;

    // Print results
    match transfer_result {
        Ok(()) => {
            info!("Ok: {}", ch.id());
            if dst_u8[1..TRANSFER_UNIT_COUNT + 1] != SRC_U8[..TRANSFER_UNIT_COUNT] {
                error!("dst doesn't match src");
            }
        }
        Err(e) => error!("Err: {}: {}", ch.id(), e),
    }
}
