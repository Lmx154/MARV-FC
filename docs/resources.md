# MARV V2 Rev 1 firmware: resource plan

Part of [MARV V2 firmware](../README.md)

Version 2, 2026-11-13 (lead; updated for the two-core design), from a survey of every driver crate, `docs/pinout.md` and embassy-rp 0.10.0.
Purpose: every device runs in ONE program at the same time, so each peripheral, PIO block, DMA channel, interrupt
and executor is assigned once, here. The bench programs of the driver crates each chose their own and collide
(the same DMA channels 0 and 1 in five of them, PIO2 in three); this table replaces those choices in the
firmware. Status of each line: FIXED = dictated by the board; CHOSEN = decided here; OPEN = waits for the
foundation design. Run as one program so far (2026-11-14): the two-core skeleton (core 1 started with `spawn_core1`, USB with a
MAVLink node on core 0, `DMA_IRQ_0` off on core 0), the BMP581 barometer on core 0 (I2C1 without DMA, `I2C1_IRQ`, the data-ready
input GPIO21 through `IO_IRQ_BANK0`), the BMM350 magnetometer on core 0 (2026-11-15: I2C0 without DMA, `I2C0_IRQ`, GPIO12 SDA, GPIO13 SCL, data-ready GPIO4 through `IO_IRQ_BANK0`), the SAM-M10Q GNSS receiver on core 0 (UART1 buffered and interrupt driven without DMA, `UART1_IRQ`, TX GPIO24, RX GPIO25; UBX configuration from `GPS_RATE_MS` and `GPS_GNSS` sent to RAM at every start, restart and apply, never saved; PPS GPIO5 edges time-stamped), the battery monitor on core 0 (2026-11-15: the ADC without DMA, `ADC_IRQ_FIFO`, GPIO41 and GPIO42; GPIO40 unused), the ELRS receiver on core 0 (2026-11-15: UART0 buffered and interrupt driven without DMA, `UART0_IRQ`, TX GPIO14, RX GPIO17 with the pad's pull-up, task `rc_link` on the `SWI_IRQ_0` interrupt executor, `rc-port`) and IMU1, the ICM-45686, on core 1 (SPI0 at 8 MHz mode 0 with DMA channels 0 and 1 on
`DMA_IRQ_0`, INT1 on GPIO15 through core 1's `IO_IRQ_BANK0`, all built inside core 1's entry closure; start-up checks:
`DMA_IRQ_0` enabled on core 1, not on core 0), IMU2, the LSM6DSK320X, on core 1 (2026-11-15: SPI1 at 8 MHz mode 0
with DMA channels 2 and 3 on `DMA_IRQ_0`, INT1 on GPIO7 through core 1's `IO_IRQ_BANK0`, INT2 GPIO6 unused,
built inside core 1's entry closure next to IMU1; configured once at boot from `IMU2_ACC_FS`, `IMU2_GYR_FS`, `IMU2_ODR_HZ`, default 800 Hz, the high-g channel off) and the hardware watchdog (2026-11-15: fed by the monitor on `SWI_IRQ_1`,
see the table below), and the four ESCs' DShot stream on core 1 (2026-11-18, `src/esc.rs`: `dshot` `PioDshot` on PIO2 SM0, GPIO36-39,
reply capture on DMA channel 5 on `DMA_IRQ_0`, `PIO2_IRQ_0` bound and enabled on core 1 only, all built inside core 1's entry; Stop
frames from every boot and command 13 in EDT bursts, and the props-off motor test's capped throttle on one line (2026-11-18, run on the board from the station, README, Motor test); the telemetry run on the board 2026-11-18, 4cfb1f0 and 0d25909, logs
`20261118-191955` to `20261118-194637`, see the README, ESC telemetry); no
other device, PIO block or DMA channel of this plan has run in it yet.

## Fixed by the board (`docs/pinout.md`)

| Device | Pins | Peripheral | Driver crate |
|---|---|---|---|
| ICM-45686 IMU | SCK 18, MOSI 19, MISO 16, CS 20, INT1 15 | SPI0 | `icm45686` |
| LSM6DSK320X IMU | SCK 10, MOSI 11, MISO 8, CS 9, INT1 7, INT2 6 | SPI1 | `lsm6dsk320x` |
| BMP581 barometer | SDA 22, SCL 23, INT 21 | I2C1 | `bmp581` |
| BMM350 magnetometer (bench, magnetometer port) | SDA 12, SCL 13, INT 4 (servo pad S6) | I2C0 | `bmm350` |
| SAM-M10Q GNSS (pads T1/R1) | TX 24, RX 25, PPS 5 (servo pad S5) | UART1 | `sam-m10q` (`src/gnss.rs`: `BufferedUart`, 4096-byte RX buffer, 64-byte TX buffer, `UART1_IRQ` on core 0; PPS edges time-stamped) |
| ELRS receiver (pads T0/R0) | TX 14 (function 11), RX 17 | UART0 | `rc-port` |
| microSD | DAT 28-31, CLK 32, CMD 33 | PIO | `pio-sdmmc` |
| ESC telemetry in | 34 | PIO | `pio-uart` `UartRx` |
| ESC signal (DShot, passthrough) | 36-39 (M4..M1) | PIO | `dshot` (BUILT, `src/esc.rs`, core 1: driver line 0 = GPIO36 = M4 ... line 3 = GPIO39 = M1), `esc-passthrough` (not in the firmware) |
| Battery voltage, current | 40, 41, 42 (ADC0-2) | ADC | `adc-battery` |
| Status LED | 43 | PIO | `ws2812c` (BUILT, `src/led.rs`, core 0: PIO1 SM1) |
| PSRAM | CS on 0 | QMI CS1 | `qmi-psram` (BUILT, `src/psram.rs`, core 0: init at boot, canary, re-map after a settings save; holds `logger::RING` and `logdl::HANDOVER`) |
| USB | | USB | `usb-cdc`, `mavlink-device` |
| Servo pads | 2-5 | PWM slices 1 and 2 | (none yet; 4 and 5 are used as inputs on the bench) |

The firmware drives the I2C pins (GPIO22/23 and GPIO12/13) as plain GPIO for a bus clear (`src/busclear.rs`): at every boot before the first transaction on each bus, and before a restart; a reset of the MCU in the middle of a transfer can leave a sensor holding SDA low.

Spare: GPIO1 (outside the upper PIO window), GPIO26, 27 (inside it, but see the note on pins below 32), GPIO35,
44-47 (inside it; 44-47 are also ADC4-7).

## PIO (3 blocks, 4 state machines and 32 instruction words each)

On the RP2350B a PIO block sees a window of 32 pins: GPIO0-31 (GPIOBASE 0) or GPIO16-47 (GPIOBASE 16). That is
the chip. Every PIO signal on this board is on GPIO28-43, so every block runs with GPIOBASE = 16 and all of them
fit: nothing is lost today. Two limits follow. (1) The chip's: one block cannot serve a pin below 16 together
with a pin above 31. (2) The HAL's (as the driver crates' READMEs describe embassy-rp 0.10; not re-read here): the
window is chosen from the pins a state machine is configured with, so a state machine that uses only pins below
32 sets its block's window back to 0 and breaks its neighbours on pins 32 and up. So a new PIO user on GPIO16-31
(the spares 26 and 27, for example) goes into a block of its own or one whose other users are all below 32,
unless the HAL is given an explicit base.

| Block | User | State machine | Words | Notes |
|---|---|---|---|---|
| PIO0 | microSD (`pio-sdmmc`) | SM0 | 21 | BUILT (`src/sd.rs`, core 0, 2026-11-15): the SD task builds the whole `Pio` (SM1-3 unused and dropped) and builds it again for every start, recovery and restart. Takes `&mut Common` + one SM + one DMA channel |
| PIO1 | Status LED (`ws2812c`, PIO transport) | SM1 | 4 | BUILT (`src/led.rs`, core 0, 2026-11-18; run on the board 2026-11-18 and seen by the owner, README, Status LED): the LED task builds `Pio::new(PIO1)` and `PioTransport::new` on SM1 and GPIO43. Not PIO0: the SD task builds `Pio::new(PIO0)` again at every SD session (`src/sd.rs`), which resets the block's bookkeeping under any other user |
| PIO1 | ESC telemetry (`pio-uart` `UartRx`) | SM0 | 13 | CHOSEN, not built. Borrows `&mut Common`. 13 words, with the LED's 4: 17 of 32, SM2 and SM3 free. When it is built, the `UartRx` and the LED must come from the SAME `Pio::new(PIO1)` call (one `Common`, SM0 and SM1): a second `Pio::new` on this block would reset the state the first one set |
| PIO2 | DShot (`dshot` `PioDshot`) | SM0 | 15 | BUILT (`src/esc.rs`, core 1, 2026-11-18): `esc::driver` builds the whole `Pio` inside `core1_main` (SM1-3 dropped) and `PioDshot::new` at DShot300 with `clk_sys` read on core 0; never released. The driver needs SM0 and takes the block's `Common` by value, so nothing else lives in PIO2 while it runs |
| PIO2 | ESC passthrough (`pio-uart` `OneWire`) | SM0 | 27 | CHOSEN. EXCLUSIVE IN TIME with DShot (15 + 27 > 32): DShot is released, `OneWire` built for the session, then released and DShot rebuilt, as the passthrough demo does |

## DMA (16 channels; embassy-rp's on `DMA_IRQ_0`, the SD card's on `DMA_IRQ_1`)

| Channel | User | Notes |
|---|---|---|
| 0, 1 | SPI0 (ICM-45686) | Core 1, BUILT (`src/imu1.rs`, inside core 1's entry). The HAL's async SPI takes two channels (as every bench does; not checked in `spi.rs`) |
| 2, 3 | SPI1 (LSM6DSK320X) | Core 1, BUILT (`src/imu2.rs`, inside core 1's entry). One `bind_interrupts!` for channels 0-3 and 5 (`DmaIrqs`, `src/main.rs`); the `f4-negative` control builds channel 6 on core 0 |
| 4 | microSD | Core 0, BUILT (`src/sd.rs`, 2026-11-15): programmed by `pio-sdmmc` itself through the PAC (no embassy-rp `dma::Channel`), completing on `DMA_IRQ_1`, enabled only in core 0's NVIC (INTE1 = 1 << 4; checked at start-up with INTE0, INTE2, INTE3) |
| 5 | DShot reply capture | Core 1, BUILT (`src/esc.rs` `driver`, inside core 1's entry, 2026-11-18; an embassy-rp `dma::Channel` on `DMA_IRQ_0`) |
| 6-15 | free | The ADC (battery) runs WITHOUT DMA (it is on core 0: see the rule below); the flash driver needs none |

## Interrupts to bind

`DMA_IRQ_0` (core 1: channels 0-3 and 5), `DMA_IRQ_1` (core 0: channel 4, `pio_sdmmc::pio::DmaIrqHandler<SdWaker>`, priority P3, set by the driver), `PIO0_IRQ_0` (the SD bus's `Pio`), `PIO1_IRQ_0` (BUILT: `led::Irqs`, enabled by `Pio::new` on core 0, the status LED; run on the board 2026-11-18 and seen by the owner, README, Status LED), `PIO2_IRQ_0` (BUILT: `esc::Irqs`, enabled by `Pio::new` on core 1; start-up check: enabled on core 1, not on core 0), `I2C0_IRQ`, `I2C1_IRQ`, `UART0_IRQ` and
`UART1_IRQ` (buffered UARTs), `ADC_IRQ_FIFO`, `USBCTRL_IRQ`. Taken by the HAL itself: `TIMER0_IRQ_0` (the time
driver, alarm 0), `IO_IRQ_BANK0` (all `Input::wait_for_*`, no binding needed; priority P3), and `DMA_IRQ_0`'s
priority (P3). Data-ready and PPS inputs (GPIO15, 7, 6, 21, 4, 5) all go through the bank interrupt.

## Executors, priorities, cores

| Item | Assignment | Status |
|---|---|---|
| Core 0, the I/O core: thread-mode executor | `embassy_rp::init`, the time driver; usb_link (`usb-cdc`'s futures are not `Send`: built here), rc_link, baro, mag, gnss, batt, supervisor, actions, settings and flash_saver, beat0, led (BUILT 2026-11-18: `src/led.rs`, a 20 ms `Ticker`, PIO1 SM1, GPIO43; run on the board 2026-11-18 and seen by the owner, README, Status LED); later sd_logger | DECIDED (design, two-core amendment) |
| Core 0: `SWI_IRQ_1` interrupt executor | the watchdog's monitor only, priority 1 (BUILT: `src/watchdog.rs`) | DECIDED |
| Core 0: `SWI_IRQ_0` | the receiver task `rc_link` alone, priority P3 (0xC0), below the monitor's `SWI_IRQ_1` (P1) (2026-11-15) | DECIDED |
| Core 1, the FLIGHT core: thread-mode executor | imu1 (BUILT: `fw_flight::imu` over the ICM-45686), imu2 (BUILT: `fw_flight::imu` over the LSM6DSK320X), fbeat (liveness, BUILT), esc (BUILT 2026-11-18: one DShot exchange per 1 ms, paced with `SpinDelay`); the flight step runs inside imu1, once per sample, after the publish (BUILT 2026-11-19, `src/flight.rs`; since step 5 P6 with the sticks from `rc::FLIGHT_RC`, arming on channel 5, and the esc task sending its command). The esc task's wait is a clock poll that wakes itself, so core 1's executor no longer sleeps: a core-1 load figure must come from task timings, not idle time. Every flight peripheral is built inside core 1's entry closure so its interrupt lands there. No interrupt executor on core 1 (it could not be woken from core 0) | DECIDED |
| Core 1 stack | a `static`, 16 KB default, set from stack painting; MSPLIM guard set by `spawn_core1` | DECIDED |
| Hardware watchdog | owned and fed by core 0's monitor, 1600 ms period, `pause_on_debug(true)`, only while `BEAT0` and `PONG` both changed less than 200 ms ago; scratch registers 0 to 3 hold the reset record (layout version 2; 4 to 7 are the boot ROM's). BUILT | DECIDED |

## Internal SRAM (measured from the ELF, normal build, 2026-11-16)

| Item | Bytes | Note |
|---|---|---|
| `.ram_text`, the flight core's code, from 0x20000000 | 75 668 | selected in `memory.x`, copied from flash at boot (rule 10) |
| `.ram_text`, every constant (`.rodata`) | 35 592 | none left in flash (link ASSERT) |
| `.data` | 4 548 | |
| `.bss` | 103 972 | core 1's vector table (512 B at 0x2002b600) among it |
| `.uninit` | 1 024 | |
| static use | 221 220 | 0x20000000 to `__euninit` 0x20036024 |
| free, up to 0x20080000 | 303 068 (296 KiB) | core 0's stack grows down into it from the top; link ASSERT: at least 32 KiB (`_stack_start - _stack_end`); core 0's stack depth is not measured. `_stack_end` = `__euninit` rounded up to 8, MSPLIM is set to it at the start of `main` (fault on overflow) |

SD3 (2026-11-16, normal build): `.data` 39 388 (+34 840: the log ring `logger::RING`, 32 824 B,
and the listing `sd::SIZES`, 2 004 B, are in `.data`, not `.bss`, because their initial values are not all zero:
the ring's statistics window starts with its minimum at `u32::MAX`, the listing's empty slot is `u32::MAX`; they
cost the same RAM and their size again in flash; accepted by the lead on 2026-11-16 rather than changing the
host-tested ring), `.bss` 106 532 (+2 560: the header buffer 980 B, the collector's task 776 B with its copy of
the 26 logged parameter values, the SD task's future 19 432 B against 18 632), `.ram_text` 119 332 (+8 076: rodata and the
`LogRing` methods, which the existing `fw_core::xcore` rule puts in SRAM), `.text` 423 168 (+33 852). New core-0
thread-mode task `log_collect` (`src/logger.rs`, watchdog id LOG = 15, a 10 ms `Ticker`, never awaits the card).
At the merged head (after the clean-up moved before the log's opening): `.text` 485 104, `.ram_text` 121 764,
`.data` 39 388, `.bss` 106 732, `.uninit` 1 024; static SRAM 269 036 B, free 255 252 B (249 KiB). The move cost
about 62 KB of flash code (the file system's delete path is now part of the open sequence): not investigated.

SD4 (2026-11-16, normal build): `.text` 492 588 (+7 484), `.ram_text` 122 340 (+576, the
second `LogRing` size's methods), `.data` 43 536 (+4 148: the hand-over ring `logdl::HANDOVER`, 4 152 B, in
`.data` for the same reason as `logger::RING`), `.bss` 107 996 (+1 264: the read buffer `sd::READ_BUF` 900 B,
the request `logdl::REQUEST` 16 B, the wake signal, four counters, the SD task's and the USB task's larger
futures), `.uninit` 1 024; static SRAM 274 908 B, free 249 380 B (243 KiB). New statics, all core 0 only
(USB task and SD task): `logdl::HANDOVER` (one producer, the SD task; one consumer, the USB task),
`logdl::REQUEST` (written by the USB task, read by the SD task, under a critical-section mutex),
`logdl::WAKE` (a `Signal`), the counters `TRANSFERS`, `BYTES_SENT`, `STEP_MAX_US`, `LATE_MAX_US`. No new task,
interrupt, DMA channel or flight-core change.

F1 and F2 (2026-11-16, normal build, against `main` 6f26fc2: `.text` 492 588, `.ram_text` 122 340, `.data` 43 536,
`.bss` 107 996): `.text` 500 048 (+7 460), `.ram_text` 125 860 (+3 520), `.data` 43 892 (+356), `.bss` 108 652 (+656), `.uninit` 1 024;
static SRAM 279 428 B (the sum, as the earlier rows). The growth is mostly F1's 44-entry parameter table (`.ram_text`: the rule puts every constant in SRAM) and its
store values (`.bss`). F2 adds no static, no task, no interrupt and no DMA channel: one byte in the existing `rc::SHARED`
(`sticks_crsf`, copied under the lock the RC hook already takes) and one stream slot in the USB task's local array. `rc::manual_control`
runs in the USB link task on core 0 when the stream is due, once per message at most 50 times a second: one `SHARED` lock to copy the
sticks, then one store lock to copy 44 values by index (a 44-entry `Option<ParamValue>` array of about 530 B on the task's stack,
not in its future); the mapping is computed outside both locks. Nothing is added to core 1 (`tools/check-ram-code.py` passes
unchanged).

CRSF GPS frame (2026-11-16, normal build): `.text` 501 644, `.ram_text` 125 840, `.data` 43 892, `.bss` 108 564,
`.uninit` 1 024 (for reference the F1 and F2 row above: `.text` 500 048, `.ram_text` 125 860, `.data` 43 892, `.bss` 108 652; not measured
against `main` itself). The change adds one `Option<u64>` to the RC hook's own state (a local of the `rc_link` task, not a static), no task,
interrupt or DMA channel; `rc-port` moved to `fe55602`. Nothing is added to core 1 (`tools/check-ram-code.py` passes unchanged).

Radio sensor streams (2026-11-16, normal build, against the F1/F2 row above): `.text` 500 760 (+712), `.ram_text` 125 860 (+0), `.data` 43 892 (+0),
`.bss` 108 908 (+256), `.uninit` 1 024; static SRAM 279 684 B, free 244 604 B (238 KiB). `rc::SLOTS` is now 6 stream slots (200 B in the ELF; the
old size was not measured); the rest of the `.bss` growth is not investigated (the `rc_link` task's future is the likely place). No new task, static of its own, interrupt or DMA
channel; the three builders read `IMU1`, `IMU2`, `BARO` and `MAG` `latest` (a critical-section `Watch`: one copy of the sample each) from
the `SWI_IRQ_0` executor on core 0, as the radio's GPS_RAW_INT and SYS_STATUS already read `GNSS` and `BATT`. Nothing is added to core 1 (`tools/check-ram-code.py` passes unchanged).

IMU chip settings (2026-11-17, normal build): `.text` 503 048, `.ram_text` 126 520, `.data` 43 940, `.bss` 109 732, `.uninit` 1 024
(absolute, `arm-none-eabi-size -A`; the rows above are from other branches, so no delta is claimed). The change reads both IMUs' setups in `main` before `spawn_core1`. No new task,
interrupt, DMA channel or cross-core cell: each setup (the chip's `Config`, the two full scales and the period, plain data) is moved by value into `imu1::Parts` / `imu2::Parts`, i.e. into core 1's
entry closure, as the pins and DMA channels are; `fw_settings` code runs on core 0 only (the adapters on core 1 only read the plain data), `REQ_APPLY` stays reserved for the flight rows.
`tools/check-ram-code.py` passes unchanged (roots 4, 129 functions, 67 548 B in SRAM, 204 data objects).

Sensor orientation (2026-11-17, `wip-orientation`, normal build): `.text` 505 088, `.ram_text` 127 976, `.data` 43 980, `.bss` 109 620, `.uninit` 1 024
(absolute, `arm-none-eabi-size -A`; the row above is from another branch, so no delta is claimed). `fw_core::rotation` (a 3x3 table and two small functions) runs on core 1: `.text.*7fw_core8rotation*`
is in `memory.x`, and `ImuOffsets::publish` is `#[inline(always)]` into the two adapters. The board's orientation (`AHRS_ORIENTATION`) is read once in `imu_setups` and moved by value into
`ImuSetup.board`, i.e. into core 1's entry closure like the rest of the setup; core 0 keeps the same value in `calibration::Core0` (a field of the existing core-0 struct, never read by core 1). No new
task, interrupt, DMA channel, cross-core cell or shared static (the two new `static`s are `ParamDef` tables, constant data read on core 0). `tools/check-ram-code.py` passes (roots 4, 130 functions, 68 048 B in SRAM, 209 data objects).

ESC telemetry F2 (2026-11-18, `esc-telemetry`, normal build, `arm-none-eabi-size -A` and the linker symbols, against
82ee5e1 built the same way): `.ram_text` 128 292 -> 146 148, `.text` 504 596 -> 548 352, `.data` 43 988 -> 43 984,
`.bss` 109 996 -> 112 084, `.uninit` 8 192; code in SRAM 83 784 -> 100 760 B; static SRAM (to `__euninit`) 290 732 ->
310 740 B; free above `_stack_end` 233 552 -> 213 544 B, 180 776 B above the 32 KiB floor. New: one task (core 1's
`esc`), one static cell (`esc::ESC`, a `Seqlock<EscStatus>` of 696 B), the PIO2 block, DMA channel 5, `PIO2_IRQ_0`.
Most of the `.text` growth is the core-0 readers (`esc::row`, `message`, `stat`, each inlining the seqlock copy).
`tools/check-ram-code.py` passes (roots 5, 151 functions, 82 712 B in SRAM, 247 data objects).

Status LED (2026-11-18, `status-led`, normal build, `arm-none-eabi-size -A`, against 338709b built the same way):
`.ram_text` 150 076 -> 153 636, `.text` 667 132 -> 700 276, `.data` 44 852 -> 44 860, `.bss` 111 396 -> 112 136,
`.uninit` 8 192. New: one core-0 task (`led`, watchdog id 19), `PIO1_IRQ_0` bound on core 0, PIO1 SM1, GPIO43, one
`AtomicU8` (`led::EVENT`), one `ParamDef` table; no cross-core cell, nothing on core 1. About 20 KB of the `.text`
growth is `led::inputs` (the rows' reads inlined). `tools/check-ram-code.py` passes with the same 5 roots (155
functions, 84 906 B in SRAM, 248 data objects; 151, 83 700 B and 248 before).

Flight code in the step (2026-11-19, `first-pair` P4b, release build, `arm-none-eabi-size -A` and the linker symbols,
against ee647fb built the same way): `.ram_text` 154 516 -> 176 528 (+22 012: code +21 300, constants +712), `.text`
702 260 -> 702 168, `.data` 44 868, `.bss` 112 852 -> 114 604 (+1 752, the flight state in IMU1's task), `.uninit`
8 192; static SRAM to `__euninit` +23 768 B (0x2004e4d4 -> 0x200541ac); free above `_stack_end` 179 792 B. The
card's `MIX` is folded into `CardAllocator::allocate`'s literal pool (in SRAM). `tools/check-ram-code.py` passes
(roots 5, 185 functions, 107 010 B, 255 data objects; 155, 85 776 B and 250 before).

Flight logging (2026-11-19, `flight-candidate`, decision-flight-logging-20261119 packet 5, release build,
`arm-none-eabi-size -A` and the linker symbols, against 5cf573d built the same way): `.ram_text` 177 628 -> 184 620
(+6 992: code +5 664, constants +1 328), `.text` 727 296 -> 731 456, `.data` 44 972 -> 94 188 (+49 216: the main
ring `logger::RING` 32 824 -> 65 592 B at 0x2002e2c8, and the new flight ring `flight::LOG` 16 440 B at 0x2003ece8,
both in `.data` for the reason given at SD3), `.bss` 115 644 -> 116 124, `.uninit` 8 192; static SRAM to
`__euninit` +56 800 B (0x200549bc -> 0x2006279c); free above `_stack_end` 177 728 -> 120 928 B, 88 160 B above
the 32 KiB floor (the decision's 97 872 B started from the P4b figure and left out the code). New: one cross-core
cell (`flight::LOG`, rule 8); no task, interrupt or DMA channel. `tools/check-ram-code.py` passes (roots 5, 195
functions, 111 578 B, 256 data objects; 183, 106 536 B and 255 before).

SRAM8 and SRAM9 (4 KB each) are unused.

## Rules that follow

1. A write or erase of the in-package flash runs with ALL interrupts off, from RAM, with core 1 paused
   (`embassy_rp::flash`, `in_ram`), and afterwards the PSRAM mapping must be redone (`qmi-psram` `remap()`). While
   it runs no device is sampled, no DShot frame is sent, and the watchdog is not fed. Measured (2026-11-15,
   100 saves of one 4 KB sector: erase, then program of one 256-byte page): 30.6 to 44.8 ms, mean 40.2 ms,
   almost all of it the erase. Today the save runs inside the `245` command's action on the link task
   (`src/settings.rs`, ONLY on an explicit save), not in a separate `flash_saver` task; the PSRAM is mapped since
   decision-psram-offload-20261121 (W2, W3), so `settings::guarded` calls `psram::after_flash_op()` (`remap()`, then a canary
   check) right after each flash erase and program.
2. A bus call of a sensor driver is never cancelled (embassy-rp 0.10): the timeout goes around the data-ready
   wait, then the driver's `recover()` is called. Exception since 2026-11-15 (the owner's "Yes just do what
   PX4 and ardupilot do", amending I1.1): an I2C transaction of BARO or MAG that misses its deadline
   (ArduPilot's formula, 4 ms at 400 kHz, `src/i2cbus.rs`) is abandoned; the bus is then refused until the
   adapter has cleared it and rebuilt the controller and the driver.
3. DShot and ESC passthrough never hold PIO2 together; passthrough is only entered with the motors stopped.
4. The servo pads 4 and 5 carry the magnetometer's interrupt and the GNSS PPS on the bench: servo outputs on
   slice 2 are not available while that wiring stands.
5. Nothing may be added to PIO0, PIO1 or PIO2 that uses pins below GPIO32 only.
6. `DMA_IRQ_0` and the embassy-rp DMA channels belong to core 1: embassy-rp 0.10 puts every channel it builds
   on `DMA_IRQ_0` and its transfers abort with a read-modify-write of the shared `CHAN_ABORT`. `DMA_IRQ_0` is
   disabled on core 0 before core 1 starts, and core 0 never builds an embassy-rp `dma::Channel`. Core 0's one
   channel, the SD card's channel 4, goes through the driver's own path (decision-sd-logging Q1): set in INTE1
   only, completing on `DMA_IRQ_1` in core 0's NVIC, cleared through INTS1; core 0 never read-modify-writes a
   DMA register shared across channels (INTR, INTEn, INTFn, INTSn, CHAN_ABORT, MULTI_CHAN_TRIGGER): only SET/CLR
   alias writes and single-bit plain writes. Start-up checks: `DMA_IRQ_0` off on core 0 (F4), and after the SD
   bus is built INTE0 = core 1's channels (0x2f: 0-3 and 5), INTE1 = 1 << 4, INTE2 = INTE3 = 0, `DMA_IRQ_1` on in core 0 and off
   in core 1.
7. A driver is constructed on the core that owns its interrupt (the HAL enables the NVIC line in the
   constructor, on the calling core). The timer interrupt is always on core 0; a flash write must come from
   core 0 and pauses core 1.
8. The flight core schedules no timer at all, takes no critical-section primitive of its own and
   never logs a text line (no defmt); data crosses the cores only through lock-free cells (seqlock out of the flight
   core, triple buffer into it, atomic flags with a lock-free waker for requests, and one `LogRing` out of it for
   its log records). The seqlocks out of core 1: each IMU's status
   (`fw_flight::imu::ImuCells`), the ESC status (`esc::ESC`, 2026-11-18, writer taken once by the esc task) and the flight
   step's output (`flight::OUT`, 2026-11-19: step time, `MotorCommand`; writer taken once by IMU1's task; core-1
   internal: its one reader is the esc task, `try_read(1)` at every exchange, step 5 P6; nothing on core 0 reads it).
   Core 0 reads `esc::ESC` for `flight_spin` (admission, the LED). The atomic flags `settings::HANDSHAKE` are handed to
   the flight step's gate, which stores the armed state there; core 0 reads it in admission. The triple buffers into
   it: the IMU offsets (`calibration::OFFSETS`), the motor-test command (`esc::TEST`, a `TestCmd`; writer in core 0's
   `actions` state, reader taken before core 1 starts and moved into the esc task, which reads it at every exchange;
   2026-11-18) and the sticks (`rc::FLIGHT_RC`, an `RcSample`, step 5 P6; writer taken by the RC port's hooks on core 0,
   written once per CRSF stick report; reader taken before core 1 starts and moved into IMU1's task and its `Stepper`,
   which reads it once per step; the stick mapping runs on core 0) and the slow sensors (`flight::FLIGHT_BARO`,
   `FLIGHT_MAG`, `FLIGHT_GNSS`, a `BaroSample`, `MagSample` and `GnssSample`, decision-nav-eskf-20261120 section 4; each
   writer taken by its device's adapter on core 0 and written once per published sample through the `to_flight` hook;
   readers taken by IMU1's `Stepper` when it is built on core 1, read once per step). The `LogRing` out of it (`flight::LOG`, 16 KiB,
   decision-flight-logging-20261119, 2026-11-19): producer taken once by IMU1's `Stepper`, which pushes the step's
   records after it has written `flight::OUT` (wait-free, atomic loads and stores only: `head`, `tail` and `stat_req`
   loaded, the statistics seqlock written, then `head` stored, or for a record that does not fit `dropped` loaded
   and stored); consumer taken once by core 0's collector
   (`logger`), which relays every record into the main log ring every 10 ms tick, log open or not. The flight core's IMU tasks wait through `SpinDelay`
   (`src/imu1.rs`): under 50 us a busy wait (the ICM-45686's 10 us waits in a verify step, so the sample loop
   never yields inside one), from 50 us on clock polls with a yield between them. HARD RULE (lead's ruling,
   2026-11-15): no timer queue on the flight core at all, no `embassy_time::{Timer, Ticker, Delay}` and no
   `with_timeout` in anything core 1 runs. The mechanism: embassy-rp 0.10's `schedule_wake` runs the
   queue's expiry in the caller's context, so a timer scheduled on core 1 can wake an expired core-0 task;
   for a core-0 interrupt-executor task (the watchdog monitor, the RC port task) the waker is
   `embassy-executor-0.10.0/src/platform/cortex_m.rs` `__pender`, which pends the interrupt with
   `NVIC::pend` on the CALLING core, core 1, where it is never served: that task is dead from then on. With
   `embassy_time::Delay` the IMU1 self-test's ~7000 short waits ended twice in a watchdog reset with no record
   (2026-11-15, `logs/marv-v2-firmware-20261115-153508.log`); the two resets were not reproduced under
   instrumentation.
9. The flight core verifies incrementally (lead's ruling, 2026-11-15): the sample loop never runs a
   whole-configuration `verify`; after every K-th sample, never while a sample is waiting, it checks ONE
   register (`verify_step`: one or two SPI transactions, the ICM's indirect registers with their 10 us
   waits), K chosen per IMU for one pass in about 1 s (IMU1 K 24 over 33 registers, IMU2 K 25 over 32). The
   earlier rule, a yield after every transaction and busy wait, was withdrawn: it stretched a whole `verify`
   past one sample period, during which the task cannot read its own samples. Each IMU task's longest
   stretch without a yield (since boot and per 10 s window) is published and logged in its `[STAT]` line.
10. The flight core runs from internal SRAM (decision-ram-code-20261115; the owner: "run from PSRAM or RAM not
   flash"; the PSRAM sits behind the same XIP cache as the flash). R1: once `core1_main` has started, core 1
   makes no XIP access. Guards: `tools/check-ram-code.py`, run by `tools/run.sh` on every build before
   flashing (a wrong result flashes nothing); the link ASSERTs for gross failure; on the board
   `FEATURES=xip-park` with its negative control `xip-park,ram-negative` (0 accesses in a 1 s window with
   core 0 parked, 2026-11-16; again with the DShot task running, 2026-11-18, 0d25909: 0 accesses, log
   `20261118-194106`, control 184 035, log `20261118-194357`); run it again when the control loop lands. R2: core 1's VTOR points at
   its vector table in SRAM (start-up check). R3: no constants in flash (link ASSERT
   `__erodata == __srodata`). R4: at least 28 252 B (26 204 B of static call chain plus 2 KiB; memory.x's ASSERT) of SRAM above the statics for core 0's stack (link ASSERT on
   `_stack_end`, defined in `memory.x` as `__euninit` rounded up to 8; an overflow of RAM fails the link; at run
   time MSPLIM = `_stack_end` on core 0 and the bottom of `CORE1_STACK` on core 1, so an overflow of either stack
   is a HardFault, not a silent write into the statics). R5: the copy runs before anything uses SRAM code
   (`__wrap___pre_init`, assembly, wrapped ahead of embassy-rp's `__pre_init`; a violation faults at once on
   every boot). Maintenance: a new core-1 interrupt becomes a checker root and a pattern line in `memory.x`;
   a new flight crate gets a pattern line. DShot (2026-11-18) added the root `PIO2_IRQ_0` and the lines
   `.text.PIO2_IRQ_0`, `*5dshot*`, `*10embassy_rp3pio*`, `*8pio_core*` (named by the checker: `set_config` in
   `PioDshot::new`, called from `core1_main`), `*9fw_flight3esc*` and three narrow `esc` lines
   (`*16marv_v2_firmware3esc6driver*`, `*3esc4task*`, `*3esc11___task_task*`): the broad
   `*16marv_v2_firmware3esc*` also moved the core-0 readers of `src/esc.rs` (about 21 KB) into SRAM.
   `*10embassy_rp3pio*` also moves core 0's PIO0 code (the SD bus's, about 2.4 KB). Removing `*5dshot*`
   makes the checker FAIL (13 functions). The flight step (2026-11-19) added
   `*9fw_flight4step*`, `*9fw_flight3gnc*`, `*9fw_flight4gate*` and `*16marv_v2_firmware6flight*7Stepper*`. With today's stand-ins the whole step is inlined into IMU1's
   `ImuTask::once`, so the checker passes without them; with `Flight::step` forced out of line, removing them makes
   it FAIL (1 function, `Flight::step`). The flight code (step 5, P4b, 2026-11-19) added `*9fw_flight5alloc*`,
   `*9fw_flight6mahony*`, `*9fw_flight3pid*`, `*9fw_flight4quat*`, `*9fw_flight6tuning*`, `*7fw_core9handshake*`,
   `*4libm4math*`, and `.text.__aeabi_d*`, `__aeabi_f2d`, `__aeabi_i2d`, `__aeabi_ul2f` and
   `*17compiler_builtins5float*` (the soft double-precision code `libm`'s `sinf`/`cosf` use, and Mahony's u64-to-f32);
   the flight log (2026-11-19) added one line per `fw_core::dflog` encoder the `Stepper` calls (`encode_imu`, `nav`,
   `setp`, `dmd`, `mot`, `esc`, `rcs`; not the whole module, whose other encoders are core 0's): without them the
   checker FAILS (7 functions);
   `Flight::step` is now out of line in SRAM. Removing the `handshake`/`libm` line makes the checker FAIL (21
   functions: `sinf`, `cosf`, `sqrtf`, `roundf`, `Handshake::try_arm`, `disarm` and their callees). The card's
   constants need no line: every `.rodata` is in SRAM already. A pattern change is accepted only with a passing checker (v0 back-references and rustc's merging of
   identical functions make the patterns inexact: three lines exist only for merged functions' sections). A
   binary flashed without `tools/run.sh` skips the checker.

## Not yet decided or measured

Whether the two IMUs' SPI really need DMA at the rates used; stack and static RAM
per task; what the loaded I/O core does to the flight core (milestone FD4); the flight core's IMU figures under sustained card work (the shared XIP cache; facts-sd-logging C2) were measured again with the flight core in SRAM (rule 10): unchanged by the 16 MiB burst (facts-sd-logging C3, 2026-11-16).
A microSD logger needs a buffer that rides through the card's stalls: on the one card tested (SanDisk 64 GB, `pio-sdmmc`, 2026-11-11) a single write call sometimes took 30 to 132 ms; the driver does not retry.
