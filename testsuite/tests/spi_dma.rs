#![no_std]
#![no_main]

use defmt::error;
use efm32_hal::{crc::Crc, usart::spi};
use embedded_hal::spi::{ErrorType, SpiBus};

#[cfg(test)]
#[embedded_test::tests]
mod tests {
    use crate::{test_buffers, test_read, test_transfer, test_write};
    use defmt::error;
    use defmt_rtt as _;
    use efm32_hal::{
        crc::{algos::CRC_32_CKSUM, Crc, CrcDriver},
        dma::descriptor::Descriptor,
        dma::Dma,
        gpio::{Gpio, InFilt, OutPp},
        peripherals::{Usart0, Usart1},
        usart::spi::dma::Spi,
        usart::spi::{Config, SpiParts},
    };
    use embedded_hal::spi::MODE_2;

    /// `Descriptor::MAX_TRANSFER_UNITS` = `0x800` = `2048` bytes
    ///
    /// The size of RAM is 32K, and since the tests may use a destination (RX) buffer of size `MAX_RAM_TRANSFERS`, then
    /// the value needs to be smaller than 32K (probably less than that, since the executable also uses some memory)
    const MAX_RAM_TRANSFERS: usize = 13;
    /// This value may need to get adjusted as we add more tests, since it occupies the vast majority of Flash
    const MAX_ROM_TRANSFERS: usize = 80;
    /// Huge ROM (Flash) array
    const SRC_BUF_SIZE: usize = Descriptor::MAX_TRANSFER_UNITS * MAX_ROM_TRANSFERS;

    /// ROM Src bytes with values in repeating interval [1..254]
    static SRC_BUF: [u8; SRC_BUF_SIZE] = {
        let mut seq = [0; SRC_BUF_SIZE];
        let mut i = 0;
        // Fill the buffer with values from 1 to 254 (`0x00` is reserved for the initial contents of the RX buffer,
        // and `0xff` for the filler value for TX transactions where TX is smaller than RX)
        while i < SRC_BUF_SIZE {
            seq[i] = 1 + (i % (u8::MAX as usize - 1)) as u8;
            i += 1;
        }
        seq
    };

    #[init]
    fn init() -> (Spi<'static, Usart0>, Spi<'static, Usart1>, Crc<u32>) {
        let p = efm32_hal::init();
        let crc = CrcDriver::new(p.Gpcrc).into_algo_32(&CRC_32_CKSUM);
        let gpio = Gpio::new(p.Gpio);
        let dma = Dma::init(p.Ldma);
        let spi0 = efm32_hal::usart::spi::dma::Spi::new(
            SpiParts::new(
                p.Usart0,
                gpio.pc8.into_mode::<OutPp>(),
                gpio.pc6.into_mode::<OutPp>(),
                gpio.pc7.into_mode::<InFilt>(),
            ),
            &Config::new(MODE_2, 1).with_loopback(true),
            dma.ch1,
            dma.ch0,
        );
        let spi1 = efm32_hal::usart::spi::dma::Spi::new(
            SpiParts::new(
                p.Usart1,
                gpio.pd9.into_mode::<OutPp>(),
                gpio.pd10.into_mode::<OutPp>(),
                gpio.pd11.into_mode::<InFilt>(),
            ),
            &Config::new(MODE_2, 1).with_loopback(true),
            dma.ch3,
            dma.ch2,
        );
        (spi0, spi1, crc)
    }

    /// A full-duplex SPI test case: `(src_len, dst_len, offset, repeat)`.
    ///
    /// Both TX (MOSI) and RX (MISO) are active. `repeat == 3` reproduces the `mul_3` tests
    /// (3 iterations with the destination buffer reset between them).
    #[derive(Clone, Copy)]
    struct FullDuplexCase {
        /// TX (source) slice length, in bytes.
        src_len: usize,
        /// RX (destination) slice length, in bytes.
        dst_len: usize,
        /// Padding before and after the destination slice, used to detect under/overflow.
        offset: usize,
        /// Number of iterations (`1` for the single tests, `3` for the `mul_3` tests).
        repeat: u8,
    }

    /// A simplex SPI test case: `(len, offset, repeat)`.
    ///
    /// Only one direction is active: RX for `read` (TX is empty) or TX for `write` (RX is empty).
    /// `repeat == 3` reproduces the `mul_3` tests (3 iterations with the destination buffer
    /// reset between them).
    #[derive(Clone, Copy)]
    struct SimplexCase {
        /// Length of the active direction's slice, in bytes (RX for `read`, TX for `write`).
        len: usize,
        /// Padding before and after the destination slice, used to detect under/overflow.
        offset: usize,
        /// Number of iterations (`1` for the single tests, `3` for the `mul_3` tests).
        repeat: u8,
    }

    const TRANSFER_CASES: &[FullDuplexCase] = &[
        FullDuplexCase {
            src_len: 0,
            dst_len: 0,
            offset: 10,
            repeat: 1,
        },
        FullDuplexCase {
            src_len: 0,
            dst_len: Descriptor::MAX_TRANSFER_UNITS,
            offset: 10,
            repeat: 1,
        },
        FullDuplexCase {
            src_len: 1,
            dst_len: 1,
            offset: 10,
            repeat: 1,
        },
        FullDuplexCase {
            src_len: Descriptor::MAX_TRANSFER_UNITS,
            dst_len: Descriptor::MAX_TRANSFER_UNITS,
            offset: 10,
            repeat: 1,
        },
        FullDuplexCase {
            src_len: Descriptor::MAX_TRANSFER_UNITS,
            dst_len: 0,
            offset: 10,
            repeat: 1,
        },
        FullDuplexCase {
            src_len: Descriptor::MAX_TRANSFER_UNITS,
            dst_len: 1,
            offset: 10,
            repeat: 1,
        },
        FullDuplexCase {
            src_len: 1,
            dst_len: Descriptor::MAX_TRANSFER_UNITS,
            offset: 10,
            repeat: 1,
        },
        FullDuplexCase {
            src_len: Descriptor::MAX_TRANSFER_UNITS * 2,
            dst_len: 1,
            offset: 10,
            repeat: 1,
        },
        FullDuplexCase {
            src_len: 1,
            dst_len: Descriptor::MAX_TRANSFER_UNITS * 2,
            offset: 10,
            repeat: 1,
        },
        FullDuplexCase {
            src_len: Descriptor::MAX_TRANSFER_UNITS * 3,
            dst_len: 1,
            offset: 10,
            repeat: 1,
        },
        FullDuplexCase {
            src_len: 1,
            dst_len: Descriptor::MAX_TRANSFER_UNITS * 3,
            offset: 10,
            repeat: 1,
        },
        FullDuplexCase {
            src_len: Descriptor::MAX_TRANSFER_UNITS * 4,
            dst_len: 1,
            offset: 10,
            repeat: 1,
        },
        FullDuplexCase {
            src_len: 1,
            dst_len: Descriptor::MAX_TRANSFER_UNITS * 4,
            offset: 10,
            repeat: 1,
        },
        FullDuplexCase {
            src_len: Descriptor::MAX_TRANSFER_UNITS * (MAX_RAM_TRANSFERS - 1),
            dst_len: 1,
            offset: 10,
            repeat: 1,
        },
        FullDuplexCase {
            src_len: 1,
            dst_len: Descriptor::MAX_TRANSFER_UNITS * (MAX_RAM_TRANSFERS - 1),
            offset: 10,
            repeat: 1,
        },
        FullDuplexCase {
            src_len: Descriptor::MAX_TRANSFER_UNITS * MAX_RAM_TRANSFERS,
            dst_len: Descriptor::MAX_TRANSFER_UNITS * MAX_RAM_TRANSFERS,
            offset: 0,
            repeat: 1,
        },
        FullDuplexCase {
            src_len: 0,
            dst_len: 0,
            offset: 10,
            repeat: 3,
        },
        FullDuplexCase {
            src_len: 0,
            dst_len: Descriptor::MAX_TRANSFER_UNITS,
            offset: 10,
            repeat: 3,
        },
        FullDuplexCase {
            src_len: 1,
            dst_len: 1,
            offset: 10,
            repeat: 3,
        },
        FullDuplexCase {
            src_len: Descriptor::MAX_TRANSFER_UNITS,
            dst_len: Descriptor::MAX_TRANSFER_UNITS,
            offset: 10,
            repeat: 3,
        },
        FullDuplexCase {
            src_len: Descriptor::MAX_TRANSFER_UNITS,
            dst_len: 0,
            offset: 10,
            repeat: 3,
        },
        FullDuplexCase {
            src_len: Descriptor::MAX_TRANSFER_UNITS,
            dst_len: 1,
            offset: 10,
            repeat: 3,
        },
        FullDuplexCase {
            src_len: 1,
            dst_len: Descriptor::MAX_TRANSFER_UNITS,
            offset: 10,
            repeat: 3,
        },
        FullDuplexCase {
            src_len: Descriptor::MAX_TRANSFER_UNITS * 2,
            dst_len: 1,
            offset: 10,
            repeat: 3,
        },
        FullDuplexCase {
            src_len: 1,
            dst_len: Descriptor::MAX_TRANSFER_UNITS * 2,
            offset: 10,
            repeat: 3,
        },
        FullDuplexCase {
            src_len: Descriptor::MAX_TRANSFER_UNITS * 3,
            dst_len: 1,
            offset: 10,
            repeat: 3,
        },
        FullDuplexCase {
            src_len: 1,
            dst_len: Descriptor::MAX_TRANSFER_UNITS * 3,
            offset: 10,
            repeat: 3,
        },
        FullDuplexCase {
            src_len: Descriptor::MAX_TRANSFER_UNITS * 4,
            dst_len: 1,
            offset: 10,
            repeat: 3,
        },
        FullDuplexCase {
            src_len: 1,
            dst_len: Descriptor::MAX_TRANSFER_UNITS * 4,
            offset: 10,
            repeat: 3,
        },
        FullDuplexCase {
            src_len: Descriptor::MAX_TRANSFER_UNITS * (MAX_RAM_TRANSFERS - 1),
            dst_len: 1,
            offset: 10,
            repeat: 3,
        },
        FullDuplexCase {
            src_len: 1,
            dst_len: Descriptor::MAX_TRANSFER_UNITS * (MAX_RAM_TRANSFERS - 1),
            offset: 10,
            repeat: 3,
        },
        FullDuplexCase {
            src_len: Descriptor::MAX_TRANSFER_UNITS * MAX_RAM_TRANSFERS,
            dst_len: Descriptor::MAX_TRANSFER_UNITS * MAX_RAM_TRANSFERS,
            offset: 0,
            repeat: 3,
        },
    ];

    const READ_CASES: &[SimplexCase] = &[
        SimplexCase {
            len: 0,
            offset: 10,
            repeat: 1,
        },
        SimplexCase {
            len: 1,
            offset: 10,
            repeat: 1,
        },
        SimplexCase {
            len: Descriptor::MAX_TRANSFER_UNITS,
            offset: 10,
            repeat: 1,
        },
        SimplexCase {
            len: Descriptor::MAX_TRANSFER_UNITS * 2,
            offset: 10,
            repeat: 1,
        },
        SimplexCase {
            len: Descriptor::MAX_TRANSFER_UNITS * 3,
            offset: 10,
            repeat: 1,
        },
        SimplexCase {
            len: Descriptor::MAX_TRANSFER_UNITS * 4,
            offset: 10,
            repeat: 1,
        },
        SimplexCase {
            len: Descriptor::MAX_TRANSFER_UNITS * (MAX_RAM_TRANSFERS - 1),
            offset: 10,
            repeat: 1,
        },
        SimplexCase {
            len: Descriptor::MAX_TRANSFER_UNITS * MAX_RAM_TRANSFERS,
            offset: 10,
            repeat: 1,
        },
        SimplexCase {
            len: 0,
            offset: 10,
            repeat: 3,
        },
        SimplexCase {
            len: 1,
            offset: 10,
            repeat: 3,
        },
        SimplexCase {
            len: Descriptor::MAX_TRANSFER_UNITS,
            offset: 10,
            repeat: 3,
        },
        SimplexCase {
            len: Descriptor::MAX_TRANSFER_UNITS * 2,
            offset: 10,
            repeat: 3,
        },
        SimplexCase {
            len: Descriptor::MAX_TRANSFER_UNITS * 3,
            offset: 10,
            repeat: 3,
        },
        SimplexCase {
            len: Descriptor::MAX_TRANSFER_UNITS * 4,
            offset: 10,
            repeat: 3,
        },
        SimplexCase {
            len: Descriptor::MAX_TRANSFER_UNITS * (MAX_RAM_TRANSFERS - 1),
            offset: 10,
            repeat: 3,
        },
        SimplexCase {
            len: Descriptor::MAX_TRANSFER_UNITS * MAX_RAM_TRANSFERS,
            offset: 10,
            repeat: 3,
        },
    ];

    const WRITE_CASES: &[SimplexCase] = &[
        SimplexCase {
            len: 0,
            offset: 10,
            repeat: 1,
        },
        SimplexCase {
            len: 1,
            offset: 10,
            repeat: 1,
        },
        SimplexCase {
            len: Descriptor::MAX_TRANSFER_UNITS,
            offset: 10,
            repeat: 1,
        },
        SimplexCase {
            len: Descriptor::MAX_TRANSFER_UNITS * 2,
            offset: 10,
            repeat: 1,
        },
        SimplexCase {
            len: Descriptor::MAX_TRANSFER_UNITS * 3,
            offset: 10,
            repeat: 1,
        },
        SimplexCase {
            len: Descriptor::MAX_TRANSFER_UNITS * 4,
            offset: 10,
            repeat: 1,
        },
        SimplexCase {
            len: Descriptor::MAX_TRANSFER_UNITS * (MAX_RAM_TRANSFERS - 1),
            offset: 10,
            repeat: 1,
        },
        SimplexCase {
            len: Descriptor::MAX_TRANSFER_UNITS * MAX_RAM_TRANSFERS,
            offset: 10,
            repeat: 1,
        },
        SimplexCase {
            len: Descriptor::MAX_TRANSFER_UNITS * MAX_ROM_TRANSFERS,
            offset: 10,
            repeat: 1,
        },
        SimplexCase {
            len: 0,
            offset: 10,
            repeat: 3,
        },
        SimplexCase {
            len: 1,
            offset: 10,
            repeat: 3,
        },
        SimplexCase {
            len: Descriptor::MAX_TRANSFER_UNITS,
            offset: 10,
            repeat: 3,
        },
        SimplexCase {
            len: Descriptor::MAX_TRANSFER_UNITS * 2,
            offset: 10,
            repeat: 3,
        },
        SimplexCase {
            len: Descriptor::MAX_TRANSFER_UNITS * 3,
            offset: 10,
            repeat: 3,
        },
        SimplexCase {
            len: Descriptor::MAX_TRANSFER_UNITS * 4,
            offset: 10,
            repeat: 3,
        },
        SimplexCase {
            len: Descriptor::MAX_TRANSFER_UNITS * (MAX_RAM_TRANSFERS - 1),
            offset: 10,
            repeat: 3,
        },
        SimplexCase {
            len: Descriptor::MAX_TRANSFER_UNITS * MAX_RAM_TRANSFERS,
            offset: 10,
            repeat: 3,
        },
        SimplexCase {
            len: Descriptor::MAX_TRANSFER_UNITS * MAX_ROM_TRANSFERS,
            offset: 10,
            repeat: 3,
        },
    ];

    /// Sync SPI DMA `transfer` over every case (single + `mul_3`).
    ///
    /// Each entry in [`TRANSFER_CASES`] is `(src_len, dst_len, offset, repeat)`; `repeat == 3`
    /// reproduces the `mul_3` tests (3 iterations with the destination buffer reset between them).
    /// In loopback mode the received bytes must match the transmitted bytes, verified by CRC in
    /// [`test_transfer`].
    ///
    /// Every case is run to completion: a failed case is logged with its index and parameters and
    /// the harness moves on to the next one. Returns `Ok(())` only if all cases passed.
    #[test]
    #[timeout(60)]
    fn transfer_u8_dma(
        (mut spi, _spi1, crc): (Spi<'static, Usart0>, Spi<'static, Usart1>, Crc<u32>),
    ) -> Result<(), ()> {
        let mut failed: usize = 0;
        for (
            i,
            &FullDuplexCase {
                src_len,
                dst_len,
                offset,
                repeat,
            },
        ) in TRANSFER_CASES.iter().enumerate()
        {
            const DST_BUF_OFFSET: usize = 10;
            const DST_BUF_SIZE: usize =
                2 * DST_BUF_OFFSET + Descriptor::MAX_TRANSFER_UNITS * MAX_RAM_TRANSFERS;
            let dst_buf_size = offset + dst_len + offset;
            for r in 0..repeat {
                let src = &SRC_BUF[..src_len];
                let mut dst_buf: [u8; DST_BUF_SIZE] = [0; DST_BUF_SIZE];
                let res = test_transfer(
                    src,
                    &mut dst_buf[..dst_buf_size],
                    dst_len,
                    offset,
                    &mut spi,
                    &crc,
                );
                if res.is_err() {
                    error!(
                        "transfer_u8_dma: case #{} (iter {}/{}) FAILED — src_len={}, dst_len={}, offset={}",
                        i,
                        r + 1,
                        repeat,
                        src_len,
                        dst_len,
                        offset
                    );
                    failed += 1;
                }
                dst_buf[..dst_buf_size].fill(0);
            }
        }
        if failed == 0 {
            Ok(())
        } else {
            error!("transfer_u8_dma: {} iteration(s) failed", failed);
            Err(())
        }
    }

    /// Sync SPI DMA `read` (RX-only) over every case (single + `mul_3`).
    ///
    /// Each entry in [`READ_CASES`] is `(len, offset, repeat)`, where `len` is the RX length
    /// (TX is empty). `repeat == 3` reproduces the `mul_3` tests. The received bytes are the TX
    /// filler (`spi::TX_FILLER_BYTE`) since TX is empty; [`test_read`] verifies the buffers.
    ///
    /// Every case is run to completion: a failed case is logged with its index and parameters and
    /// the harness moves on to the next one. Returns `Ok(())` only if all cases passed.
    #[test]
    #[timeout(60)]
    fn read_u8_dma(
        (mut spi, _spi1, crc): (Spi<'static, Usart0>, Spi<'static, Usart1>, Crc<u32>),
    ) -> Result<(), ()> {
        let mut failed: usize = 0;
        for (
            i,
            &SimplexCase {
                len,
                offset,
                repeat,
            },
        ) in READ_CASES.iter().enumerate()
        {
            const DST_BUF_OFFSET: usize = 10;
            const DST_BUF_SIZE: usize =
                2 * DST_BUF_OFFSET + Descriptor::MAX_TRANSFER_UNITS * MAX_RAM_TRANSFERS;
            let dst_buf_size = offset + len + offset;
            for r in 0..repeat {
                let mut dst_buf: [u8; DST_BUF_SIZE] = [0; DST_BUF_SIZE];
                let res = test_read(
                    &[],
                    &mut dst_buf[..dst_buf_size],
                    len,
                    offset,
                    &mut spi,
                    &crc,
                );
                if res.is_err() {
                    error!(
                        "read_u8_dma: case #{} (iter {}/{}) FAILED — len={}, offset={}",
                        i,
                        r + 1,
                        repeat,
                        len,
                        offset
                    );
                    failed += 1;
                }
                dst_buf[..dst_buf_size].fill(0);
            }
        }
        if failed == 0 {
            Ok(())
        } else {
            error!("read_u8_dma: {} iteration(s) failed", failed);
            Err(())
        }
    }

    /// Sync SPI DMA `write` (TX-only) over every case (single + `mul_3`).
    ///
    /// Each entry in [`WRITE_CASES`] is `(len, offset, repeat)`, where `len` is the TX length
    /// (RX is empty). `repeat == 3` reproduces the `mul_3` tests. The destination slice is empty,
    /// so [`test_write`] only checks the under/overflow padding of `dst_buf`.
    ///
    /// Every case is run to completion: a failed case is logged with its index and parameters and
    /// the harness moves on to the next one. Returns `Ok(())` only if all cases passed.
    #[test]
    #[timeout(60)]
    fn write_u8_dma(
        (mut spi, _spi1, crc): (Spi<'static, Usart0>, Spi<'static, Usart1>, Crc<u32>),
    ) -> Result<(), ()> {
        let mut failed: usize = 0;
        for (
            i,
            &SimplexCase {
                len,
                offset,
                repeat,
            },
        ) in WRITE_CASES.iter().enumerate()
        {
            const DST_BUF_OFFSET: usize = 10;
            const DST_BUF_SIZE: usize =
                2 * DST_BUF_OFFSET + Descriptor::MAX_TRANSFER_UNITS * MAX_RAM_TRANSFERS;
            let dst_buf_size = offset + offset;
            for r in 0..repeat {
                let src = &SRC_BUF[..len];
                let mut dst_buf: [u8; DST_BUF_SIZE] = [0; DST_BUF_SIZE];
                let res = test_write(src, &mut dst_buf[..dst_buf_size], 0, offset, &mut spi, &crc);
                if res.is_err() {
                    error!(
                        "write_u8_dma: case #{} (iter {}/{}) FAILED — len={}, offset={}",
                        i,
                        r + 1,
                        repeat,
                        len,
                        offset
                    );
                    failed += 1;
                }
                dst_buf[..dst_buf_size].fill(0);
            }
        }
        if failed == 0 {
            Ok(())
        } else {
            error!("write_u8_dma: {} iteration(s) failed", failed);
            Err(())
        }
    }

    /// Interleaved transfers on two independent SPI DMA drivers (USART0 + USART1) to verify
    /// they function in parallel without interference.
    ///
    /// The first driver (`Spi<'static, Usart0>`) uses USART0 with DMA channels 1/0
    /// (TX/RX). The second (`Spi<'static, Usart1>`) uses USART1 with DMA channels 3/2
    /// (TX/RX). The RX channel always has a lower channel number (higher LDMA priority)
    /// than the TX channel, per the errata USART_E203 workaround.
    ///
    /// A subset of [`TRANSFER_CASES`] is run on both drivers in an alternating fashion:
    /// transfer on spi0, then transfer on spi1, repeat. Both are in loopback mode, so the
    /// received data must match the transmitted data. The test verifies both drivers produce
    /// correct results after all interleaved transfers.
    #[test]
    #[timeout(60)]
    fn transfer_interleaved(
        (mut spi0, mut spi1, crc): (Spi<'static, Usart0>, Spi<'static, Usart1>, Crc<u32>),
    ) -> Result<(), ()> {
        // Use a representative subset of cases that exercise different transfer sizes
        // (small, medium, large, asymmetric).
        const INTERLEAVED_CASES: &[FullDuplexCase] = &[
            FullDuplexCase {
                src_len: 1,
                dst_len: 1,
                offset: 10,
                repeat: 1,
            },
            FullDuplexCase {
                src_len: 16,
                dst_len: 16,
                offset: 10,
                repeat: 1,
            },
            FullDuplexCase {
                src_len: 1,
                dst_len: 16,
                offset: 10,
                repeat: 3,
            },
            // Large transfers that previously hung on USART1 (channels 2/3)
            FullDuplexCase {
                src_len: 256,
                dst_len: 256,
                offset: 10,
                repeat: 1,
            },
            FullDuplexCase {
                src_len: Descriptor::MAX_TRANSFER_UNITS,
                dst_len: Descriptor::MAX_TRANSFER_UNITS,
                offset: 10,
                repeat: 1,
            },
            // Asymmetric: TX larger than RX and vice versa
            FullDuplexCase {
                src_len: 256,
                dst_len: 1,
                offset: 10,
                repeat: 1,
            },
            FullDuplexCase {
                src_len: 1,
                dst_len: 256,
                offset: 10,
                repeat: 1,
            },
        ];

        const DST_BUF_OFFSET: usize = 10;
        const DST_BUF_SIZE: usize = 2 * DST_BUF_OFFSET + Descriptor::MAX_TRANSFER_UNITS;

        let mut failed: usize = 0;

        for (
            i,
            &FullDuplexCase {
                src_len,
                dst_len,
                offset,
                repeat,
            },
        ) in INTERLEAVED_CASES.iter().enumerate()
        {
            let dst_buf_size = offset + dst_len + offset;
            for r in 0..repeat {
                let src = &SRC_BUF[..src_len];

                // Start a non-blocking transfer on spi0 (DMA starts immediately).
                // Start a non-blocking transfer on spi0 (DMA starts immediately).
                let mut dst_buf0: [u8; DST_BUF_SIZE] = [0; DST_BUF_SIZE];
                // SAFETY: transfer_nb borrows the dst slice for the DMA transfer lifetime.
                // We use raw pointers to avoid the borrow checker conflict with
                // test_buffers, which reads dst_buf0 after flush_blocking has consumed
                // the SpiTransfer and the DMA transfer is complete.
                let transfer0 = unsafe {
                    let dst =
                        core::slice::from_raw_parts_mut(dst_buf0.as_mut_ptr().add(offset), dst_len);
                    spi0.transfer_nb(dst, &src[..src_len])
                };

                // Start a non-blocking transfer on spi1 (both DMAs now running in parallel).
                let mut dst_buf1: [u8; DST_BUF_SIZE] = [0; DST_BUF_SIZE];
                let transfer1 = unsafe {
                    let dst =
                        core::slice::from_raw_parts_mut(dst_buf1.as_mut_ptr().add(offset), dst_len);
                    spi1.transfer_nb(dst, &src[..src_len])
                };

                // Now block on both — they were started in parallel, so the DMA engines
                // overlap. flush_blocking consumes the SpiTransfer and returns the result.
                let res0 = match transfer0 {
                    Ok(t) => Spi::<Usart0>::flush_blocking(t),
                    Err(e) => Err(e),
                };
                if res0.is_err() {
                    error!(
                        "transfer_interleaved: spi0 case #{} (iter {}/{}) FAILED",
                        i,
                        r + 1,
                        repeat,
                    );
                    failed += 1;
                } else {
                    if test_buffers(src, &dst_buf0[..dst_buf_size], dst_len, offset, &crc).is_err()
                    {
                        error!(
                            "transfer_interleaved: spi0 case #{} (iter {}/{}) buffer check FAILED",
                            i,
                            r + 1,
                            repeat,
                        );
                        failed += 1;
                    }
                }

                let res1 = match transfer1 {
                    Ok(t) => Spi::<Usart1>::flush_blocking(t),
                    Err(e) => Err(e),
                };
                if res1.is_err() {
                    error!(
                        "transfer_interleaved: spi1 case #{} (iter {}/{}) FAILED",
                        i,
                        r + 1,
                        repeat,
                    );
                    failed += 1;
                } else {
                    if test_buffers(src, &dst_buf1[..dst_buf_size], dst_len, offset, &crc).is_err()
                    {
                        error!(
                            "transfer_interleaved: spi1 case #{} (iter {}/{}) buffer check FAILED",
                            i,
                            r + 1,
                            repeat,
                        );
                        failed += 1;
                    }
                }
            }
        }

        if failed == 0 {
            Ok(())
        } else {
            error!("transfer_interleaved: {} iteration(s) failed", failed);
            Err(())
        }
    }
}

/// Test [`SpiBus`] transfer
///
/// Values of `dst_buf`, `dst_len`, `dst_offset` must conform to:
///
/// ```rs,no_run
///     assert_eq!(dst_buf.len(), dst_offset + dst_len + dst_offset)
/// ```
fn test_transfer<T: SpiBus + ErrorType>(
    src: &[u8],
    dst_buf: &mut [u8],
    dst_len: usize,
    dst_offset: usize,
    spi: &mut T,
    crc: &Crc<u32>,
) -> Result<(), ()> {
    assert_eq!(dst_buf.len(), dst_offset + dst_len + dst_offset);
    let dst = &mut dst_buf[dst_offset..dst_offset + dst_len];

    if spi.transfer(dst, src).is_err() {
        return Err(());
    }
    if spi.flush().is_err() {
        return Err(());
    }

    test_buffers(src, dst_buf, dst_len, dst_offset, crc)
}

/// Test [`SpiBus`] read
///
/// Values of `dst_buf`, `dst_len`, `dst_offset` must conform to:
///
/// ```rs,no_run
///     assert_eq!(dst_buf.len(), dst_offset + dst_len + dst_offset)
/// ```
fn test_read<T: SpiBus + ErrorType>(
    src: &[u8],
    dst_buf: &mut [u8],
    dst_len: usize,
    dst_offset: usize,
    spi: &mut T,
    crc: &Crc<u32>,
) -> Result<(), ()> {
    assert_eq!(dst_buf.len(), dst_offset + dst_len + dst_offset);
    let dst = &mut dst_buf[dst_offset..dst_offset + dst_len];

    if spi.read(dst).is_err() {
        return Err(());
    }
    if spi.flush().is_err() {
        return Err(());
    }

    test_buffers(src, dst_buf, dst_len, dst_offset, crc)
}

/// Test [`SpiBus`] write
///
fn test_write<T: SpiBus + ErrorType>(
    src: &[u8],
    dst_buf: &mut [u8],
    dst_len: usize,
    dst_offset: usize,
    spi: &mut T,
    crc: &Crc<u32>,
) -> Result<(), ()> {
    if spi.write(src).is_err() {
        return Err(());
    }
    if spi.flush().is_err() {
        return Err(());
    }

    test_buffers(src, dst_buf, dst_len, dst_offset, crc)
}

/// Test the buffers after the SPI operation has completed
fn test_buffers(
    src: &[u8],
    dst_buf: &[u8],
    dst_len: usize,
    dst_offset: usize,
    crc: &Crc<u32>,
) -> Result<(), ()> {
    assert_eq!(dst_buf.len(), dst_offset + dst_len + dst_offset);
    let dst = &dst_buf[dst_offset..dst_offset + dst_len];

    // Only compare the common bytes of the two slices `src` and `dst`
    let crc_len = src.len().min(dst.len());
    if crc_len > 0 {
        crc.update(&src[..crc_len]);
        let src_crc = crc.finalize();
        crc.update(&dst[..crc_len]);
        let dst_crc = crc.finalize();

        // src and dest are identical
        if src_crc != dst_crc {
            error!(
                "CRCs don't match! Src: {} bytes, src_crc=0x{:X}, Dst: {} bytes, dst_crc=0x{:X}",
                src.len(),
                src_crc,
                dst.len(),
                dst_crc
            );
            error!("\t dst_buf = {=[?]}", dst_buf);
            return Err(());
        }
    }

    // Check filler bytes, if they exist
    let dst_filler_len = dst.len().saturating_sub(src.len());
    if dst_filler_len > 0 {
        let start_index = dst.len() - dst_filler_len;
        let end_index = dst.len();

        // filler bytes of `dst`
        for (i, b) in dst[start_index..end_index].iter().enumerate() {
            if *b != spi::TX_FILLER_BYTE {
                error!(
                    "Dst filler: expected 0x{:X}, found 0x{:X}, at index {}",
                    spi::TX_FILLER_BYTE,
                    *b,
                    i + start_index
                );
                return Err(());
            }
        }
    }

    // no bytes written before start of `dst`
    for b in &dst_buf[0..dst_offset] {
        if *b != 0 {
            error!(
                "Dst underflow: expected [0;_], found {=[?]}",
                &dst_buf[0..dst_offset]
            );
            return Err(());
        }
    }
    // no bytes written after end of `dst`
    for b in &dst_buf[dst_offset + dst_len..] {
        if *b != 0 {
            error!(
                "Dst overflow: expected [0;_], found {=[?]}",
                &dst_buf[dst_offset + dst_len..]
            );
            return Err(());
        }
    }

    Ok(())
}
