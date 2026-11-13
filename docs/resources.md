# MARV V2 Rev 1 firmware: resource plan

Version 2, 2026-11-13 (lead; updated for the two-core design), from a survey of every driver crate, `docs/pinout.md` and embassy-rp 0.10.0.
Purpose: every device runs in ONE program at the same time, so each peripheral, PIO block, DMA channel, interrupt
and executor is assigned once, here. The bench programs of the driver crates each chose their own and collide
(the same DMA channels 0 and 1 in five of them, PIO2 in three); this table replaces those choices in the
firmware. Status of each line: FIXED = dictated by the board; CHOSEN = decided here; OPEN = waits for the
foundation design. Nothing in this plan has run yet as one program.

## Fixed by the board (`docs/pinout.md`)

| Device | Pins | Peripheral | Driver crate |
|---|---|---|---|
| ICM-45686 IMU | SCK 18, MOSI 19, MISO 16, CS 20, INT1 15 | SPI0 | `icm45686` |
| LSM6DSK320X IMU | SCK 10, MOSI 11, MISO 8, CS 9, INT1 7, INT2 6 | SPI1 | `lsm6dsk320x` |
| BMP581 barometer | SDA 22, SCL 23, INT 21 | I2C1 | `bmp581` |
| BMM350 magnetometer (bench, magnetometer port) | SDA 12, SCL 13, INT 4 (servo pad S6) | I2C0 | `bmm350` |
| LC76G GNSS (bench, pads T1/R1) | TX 24, RX 25, PPS 5 (servo pad S5) | UART1 | `lc76g` |
| ELRS receiver (pads T0/R0) | TX 14 (function 11), RX 17 | UART0 | `rc-port` |
| microSD | DAT 28-31, CLK 32, CMD 33 | PIO | `pio-sdmmc` |
| ESC telemetry in | 34 | PIO | `pio-uart` `UartRx` |
| ESC signal (DShot, passthrough) | 36-39 (M4..M1) | PIO | `dshot`, `esc-passthrough` |
| Battery voltage, current | 40, 41, 42 (ADC0-2) | ADC | `adc-battery` |
| Status LED | 43 | PIO | `ws2812c` |
| PSRAM | CS on 0 | QMI CS1 | `qmi-psram` |
| USB | | USB | `usb-cdc`, `mavlink-device` |
| Servo pads | 2-5 | PWM slices 1 and 2 | (none yet; 4 and 5 are used as inputs on the bench) |

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
| PIO0 | microSD (`pio-sdmmc`) | SM0 | 21 | CHOSEN. Takes `&mut Common` + one SM + one DMA channel |
| PIO0 | Status LED (`ws2812c`, PIO transport) | SM1 | 4 | CHOSEN. Takes `&mut Common` + one SM. 25 of 32 words used; SM2, SM3 free |
| PIO1 | ESC telemetry (`pio-uart` `UartRx`) | SM0 | 13 | CHOSEN. Borrows `&mut Common`. 19 words and SM1-3 free |
| PIO2 | DShot (`dshot` `PioDshot`) | SM0 | 15 | CHOSEN. The driver needs SM0 and takes the block's `Common` by value, so nothing else lives in PIO2 while it runs |
| PIO2 | ESC passthrough (`pio-uart` `OneWire`) | SM0 | 27 | CHOSEN. EXCLUSIVE IN TIME with DShot (15 + 27 > 32): DShot is released, `OneWire` built for the session, then released and DShot rebuilt, as the passthrough demo does |

## DMA (16 channels, all on `DMA_IRQ_0`; one handler entry per channel in a single `bind_interrupts!`)

| Channel | User | Notes |
|---|---|---|
| 0, 1 | SPI0 (ICM-45686) | Core 1. The HAL's async SPI takes two channels (as every bench does; not checked in `spi.rs`) |
| 2, 3 | SPI1 (LSM6DSK320X) | Core 1 |
| 4 | microSD | BLOCKED: the driver requires one, the SD belongs on core 0, and DMA is flight-core-only for now (rule 6) |
| 5 | DShot reply capture | Core 1. The driver requires one |
| 6-15 | free | The ADC (battery) runs WITHOUT DMA (it is on core 0: see the rule below); the flash driver needs none |

## Interrupts to bind

`DMA_IRQ_0` (channels 0-6), `PIO0_IRQ_0`, `PIO1_IRQ_0`, `PIO2_IRQ_0`, `I2C0_IRQ`, `I2C1_IRQ`, `UART0_IRQ` and
`UART1_IRQ` (buffered UARTs), `ADC_IRQ_FIFO`, `USBCTRL_IRQ`. Taken by the HAL itself: `TIMER0_IRQ_0` (the time
driver, alarm 0), `IO_IRQ_BANK0` (all `Input::wait_for_*`, no binding needed; priority P3), and `DMA_IRQ_0`'s
priority (P3). Data-ready and PPS inputs (GPIO15, 7, 6, 21, 4, 5) all go through the bank interrupt.

## Executors, priorities, cores

| Item | Assignment | Status |
|---|---|---|
| Core 0, the I/O core: thread-mode executor | `embassy_rp::init`, the time driver; usb_link (`usb-cdc`'s futures are not `Send`: built here), rc_link, baro, mag, gnss, batt, supervisor, actions, settings and flash_saver, beat0; later led, sd_logger | DECIDED (design, two-core amendment) |
| Core 0: `SWI_IRQ_1` interrupt executor | the watchdog's monitor only | DECIDED |
| Core 0: `SWI_IRQ_0` | reserved (the receiver task, if UART overruns show it is needed) | DECIDED |
| Core 1, the FLIGHT core: thread-mode executor | imu1, imu2, fbeat (liveness); later the control loop, DShot, arming. Every flight peripheral is built inside core 1's entry closure so its interrupt lands there. No interrupt executor on core 1 (it could not be woken from core 0) | DECIDED |
| Core 1 stack | a `static`, 16 KB default, set from stack painting | DECIDED |
| Hardware watchdog | owned and fed by core 0's monitor, 1600 ms period, only while both cores' heartbeats advance | DECIDED |

## Rules that follow

1. A write or erase of the in-package flash runs with ALL interrupts off, from RAM, with core 1 paused
   (`embassy_rp::flash`, `in_ram`), and afterwards the PSRAM mapping must be redone (`qmi-psram` `remap()`). While
   it runs no device is sampled, no DShot frame is sent, and the watchdog is not fed. Duration not measured.
2. A bus call of a sensor driver is never cancelled (embassy-rp 0.10): the timeout goes around the data-ready
   wait, then the driver's `recover()` is called.
3. DShot and ESC passthrough never hold PIO2 together; passthrough is only entered with the motors stopped.
4. The servo pads 4 and 5 carry the magnetometer's interrupt and the GNSS PPS on the bench: servo outputs on
   slice 2 are not available while that wiring stands.
5. Nothing may be added to PIO0, PIO1 or PIO2 that uses pins below GPIO32 only.
6. DMA is used on the flight core (core 1) ONLY: embassy-rp 0.10 puts every channel on `DMA_IRQ_0` with one
   shared status register, so two cores would race. `DMA_IRQ_0` is disabled on core 0 before core 1 starts and
   core 0 builds no DMA channel. The microSD therefore waits for a way to route a channel to a second DMA
   interrupt line on core 0 (a HAL change).
7. A driver is constructed on the core that owns its interrupt (the HAL enables the NVIC line in the
   constructor, on the calling core). The timer interrupt is always on core 0; a flash write must come from
   core 0 and pauses core 1.
8. The flight core schedules no timer in its sample loops, takes no critical-section primitive of its own and
   never logs; data crosses the cores only through lock-free cells (seqlock out of the flight core, triple
   buffer into it, atomic flags with a lock-free waker for requests).

## Not yet decided or measured

Whether the two IMUs' SPI really need DMA at the rates used; the length of a flash erase; stack and static RAM
per task; what the loaded I/O core does to the flight core (milestone FD4); how the microSD gets its DMA channel.
