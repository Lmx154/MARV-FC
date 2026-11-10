# MARV V2 firmware

Independent drivers for the devices on the MARV V2 Rev 1 flight controller (RP2354B), and the
on-board tests that prove them. Phase one is data acquisition; this is the driver layer for it.

## Layout

```
crates/            one crate per device; no crate depends on another
  pio-sdmmc/       microSD, native four-bit SD bus
  icm45686/        TDK ICM-45686 IMU (SPI)
  lsm6dsk320x/     ST LSM6DSK320X IMU with high-g accelerometer (SPI)
  bmp581/          Bosch BMP581 barometer (I2C)
  bmm350/          Bosch BMM350 magnetometer (I2C)
  lc76g/           Quectel LC76G GPS (UART)
hil/               on-board tests, one package per driver; the only place that uses embassy-rp
docs/pinout.md     pin map of the board
docs/devices.md    register, timing and measured facts per device
tools/run.sh       builds, flashes and runs one on-board test
```

## Drivers

| Crate | Device | Needs from the application | Measured on the board |
|---|---|---|---|
| `icm45686` | ICM-45686 | `SpiDevice`, `DelayNs` | 6.4 kHz with no missed sample; 26 us per read at 8 MHz SPI |
| `lsm6dsk320x` | LSM6DSK320X | `SpiDevice`, `DelayNs` | 7.68 kHz setting (7540 Hz actual) with no missed sample; 23 us per read at 8 MHz SPI |
| `bmp581` | BMP581 | `I2c`, `DelayNs` | 496 Hz continuous; 242 us per read at 400 kHz I2C |
| `bmm350` | BMM350 | `I2c`, `DelayNs` | 400 Hz with no missed sample; 443 us per compensated read at 400 kHz I2C |
| `lc76g` | LC76G GPS | a buffered UART: `Read`, and `Write` for commands | 10 Hz, 100 fixes per 10 s with no checksum error; about 115 us to decode a sentence |
| `pio-sdmmc` | microSD | a `Bus` (the PIO + DMA bus for embassy-rp is included) | 9.1 MB/s read, 5.8 to 7.9 MB/s write at 25 MHz |

### What the four sensor drivers have in common

- Dependencies: `embedded-hal` 1.0, `embedded-hal-async` 1.0 and, optionally, `defmt`. No embassy
  crate, no HAL, no other driver.
- The crate root is the async driver. The `blocking` module is the same driver for blocking buses:
  it wraps the bus in an adapter that never pends and runs the same code, so nothing is written twice.
- Same shape: `new(bus)`, `init(&mut delay, &config)`, `read()` (one bus transaction, integers
  only), unit conversion as separate pure functions, public register access, `release()`.
- A driver owns no interrupt pin, timer or task. `init` configures the chip's data-ready output; the
  application waits on the pin. Every sample signalled by data-ready after `init` returns is valid.
- All device knowledge (register map, configuration codes, decoding, units) is pure code tested on
  the host.

```rust
// Async, for example in an embassy task:
let mut imu = icm45686::Icm45686::new(spi_device);
let config = icm45686::Config { int1_data_ready: true, ..Default::default() };
imu.init(&mut delay, &config).await?;
loop {
    int1.wait_for_rising_edge().await;
    let sample = imu.read().await?;
    let accel = sample.accel_ms2(config.accel_fs);
}

// Blocking, on any embedded-hal bus:
let mut imu = icm45686::blocking::Icm45686::new(spi_device);
imu.init(&mut delay, &icm45686::Config::default())?;
let sample = imu.read()?;
```

Each crate's own documentation has the same two sketches for its device.

## GPS

`lc76g` drives the Quectel LC76G on a UART. `embedded-hal` has no UART traits, so this crate depends on
`embedded-io` 0.7 and `embedded-io-async` 0.7 instead, and optionally `defmt`. There are three ways in:

- the async driver `Lc76g`, on `embedded_io_async::Read` (plus `Write` to send commands);
- `blocking::Lc76g`, the same code on the blocking `embedded_io` traits;
- the core with no UART at all: `Framer` (bytes in, checked sentences out), `Sentence::parse`,
  `FixBuilder` and `Command::write`, for firmware that drains its own DMA ring buffer.

```rust
let mut gps = lc76g::Lc76g::new(buffered_uart);           // 115200 baud 8N1
let config = lc76g::Config {
    nav_mode: lc76g::NavMode::Drone,
    ..lc76g::Config::with_fix_interval(100)               // 10 Hz
};
gps.init(&config).await?;
loop {
    let fix = gps.read_fix().await?;                      // one GGA + RMC pair of the same second
    if fix.valid {
        let (lat, lon, alt) = (fix.lat_e7, fix.lon_e7, fix.alt_mm);
    }
}
```

- A fix is integers only: 1e-7 degree, mm, mm/s, 0.01 degree. `read` returns each decoded sentence and
  `read_sentence` the raw text, for logging.
- Give the driver a buffered UART: the module sends bursts of about 1.4 kB.
- Above 1 Hz the satellite lists (GSA, GSV) may go out at most once per second, and `init` refuses a
  configuration that breaks this. This module's full default stream at 10 Hz is more than 115200 baud can
  carry, and a flooded module stops answering commands. `Config::with_fix_interval` sets the rates to fit.
- At 10 Hz the module's fix times are not on a 100 ms grid: steps between about 90 and 107 ms are normal.
  Use `fix.time` for timing, not the order of arrival.
- A fix reaches the application 0.07 to 0.18 s after the PPS edge of its second. The driver does not
  touch the PPS pin; the application time-stamps it.
- `init` never writes the module's flash, so settings last until the module loses power. `save_settings`
  is a separate call.
- The driver has no timer. On a silent line a read waits forever, so wrap calls in a timeout.
- Decoding costs about 115 us per sentence on the RP2350, under 1 % of one core at 10 Hz.

## SD card

`pio-sdmmc` has three layers. `proto` is the SD protocol as pure functions. `card` is the driver
(`SdCard`: identification, block reads and writes). Both are hardware-independent and reach the card
only through the `Bus` trait. The `pio` module (feature `embassy-rp`) is the bus for the RP2350:
one PIO state machine and one DMA channel. The state machine that moves the bits also makes the
clock, so a late DMA transfer or a busy CPU pauses the clock instead of corrupting data.

```rust
let Pio { mut common, sm0, .. } = Pio::new(p.PIO0, Irqs);
let bus = PioBus::new(
    &mut common, sm0, dma::Channel::new(p.DMA_CH0, Irqs),
    p.PIN_32, p.PIN_33,                          // CLK, CMD
    p.PIN_28, p.PIN_29, p.PIN_30, p.PIN_31,      // DAT0-3
    PioBusConfig::default(),
);
let mut card = SdCard::new(bus, Config::default());
let info = card.init().await?;
card.write_blocks(lba, &blocks).await?;          // blocks: &[pio_sdmmc::Block]
card.read_blocks(lba, &mut blocks).await?;
```

Measured on the board at 25 MHz, 1 MiB per run, SanDisk 64 GB card:

| Blocks per call | Write | Read |
|---|---|---|
| 1 | 0.4 MB/s | 1.3 MB/s |
| 8 | 3.0 MB/s | 4.8 MB/s |
| 64 | 6.8 MB/s | 8.4 MB/s |
| 256 | 5.8 to 7.9 MB/s | 9.1 MB/s |

- Write in large calls. A call costs a few commands on top of the data, and a single-block write
  costs about 1.2 ms of card programming time.
- The card sometimes pauses for tens of milliseconds while it reorganises itself. A 4 KiB write
  takes about 1.4 ms typically; the worst seen across runs is 33 ms. Whatever writes logs has to
  buffer across such a pause.
- The bus needs a PIO block whose pins all lie in GPIO16-47. embassy-rp rewrites that setting each
  time any state machine in the block is configured, so every other user of the same PIO block must
  have a pin at GPIO32 or above, or SD should have a PIO block to itself.
- The driver reads and writes blocks. Files need a filesystem on top; none is included.
- On a failed transfer the driver brings the card back to its idle state once and returns the error.
  It does not retry.

## Tests

On the host, everything except the PIO bus itself:

    cargo test --workspace

On the board, through a debug probe (the script takes a lock, so runs never overlap):

    tools/run.sh <binary> [timeout_seconds]

| Binary | What it checks |
|---|---|
| `alive` | tool chain, probe and board |
| `sd_bus` | the PIO bus alone: identification, data in both directions, a deliberately late DMA transfer, timing of each part |
| `sd_smoke` | the card driver on the real bus: 1 MiB write and read-back, throughput by call size, write latency |
| `icm45686_smoke`, `lsm6dsk320x_smoke`, `bmp581_smoke`, `bmm350_smoke` | identity, data-ready rate with missed-sample count, time per read, plausible values at rest, the blocking front-end |
| `lc76g_smoke` | firmware version, configuration to 10 Hz, fix rate and counters, plausible fixes, decode cost, PPS period, the blocking front-end, return to the module's defaults; never writes the module's flash |

The exit status is 0 when every check passed. Full logs are kept under `hil/logs/`.
`sd_bus` and `sd_smoke` overwrite blocks on the card.

## Not done

- SD: a filesystem, automatic retries, High Speed mode (the bus runs at Default Speed, 25 MHz).
- Sensors: FIFO and batch reads, self-test, motion and embedded functions, calibration of the
  installed sensors (the LSM6DSK320X high-g channel reads about 0.3 g high at rest on this board; the
  compass needs hard-iron and soft-iron calibration).
- GPS: baud-rate change and constellation selection (both restart the module), the module's accuracy
  estimate merged into the fix (the message is decoded on its own), time synchronisation from PPS, and
  recovery of a module that other software left flooded.
- Drivers for the PSRAM and the status LED.
