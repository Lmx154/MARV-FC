# MARV V2 Rev 1 pin map

GPIO assignments and the electrical facts a driver needs at each pin; per-device register facts are in `devices.md`. The netlist and the board silk are authoritative. All GPIO are 3.3 V logic and are not 5 V tolerant.

## MCU and clock

- RP2354B, QFN-80, 2 MB in-package flash. 12 MHz crystal Y1.
- On the QFN-80, ADC channels 0-7 are GPIO40-47 (channel 8 is the temperature sensor).
- Function numbers in the table: F1 SPI, F2 UART, F3 I2C, F4 PWM, F5 SIO, F6-F8 PIO0/1/2, F9 clock/QMI/trace, F10 USB, F11 UART aux.
- USB: D+/D- through 22 ohm each to USB-C J4; 5.1k on CC1/CC2.
- RESET button SW1: RUN to GND, 100k pull-up. BOOTSEL button SW2: QSPI_SS to GND through 1k R24.

## Per-GPIO table

| GPIO | Net | Function used | Status |
|---|---|---|---|
| 0 | FLASH_CS1 | QMI CS1n (F9) | PSRAM socket U24 chip select; 10k pull-up R35 |
| 1 | NC | GPIO | spare, no-connect |
| 2 | PWM8 | PWM1 A (F4) | servo S8, IO block row 10 |
| 3 | PWM7 | PWM1 B (F4) | servo S7, IO block row 9 |
| 4 | PWM6 | PWM2 A (F4) | servo S6, IO block row 8 |
| 5 | PWM5 | PWM2 B (F4) | servo S5, IO block row 7 |
| 6 | IMU2_INT2 | GPIO input | LSM6DSK320X INT2 |
| 7 | IMU2_INT1 | GPIO input | LSM6DSK320X INT1 |
| 8 | IMU2_MISO | SPI1 RX (F1) | LSM6DSK320X |
| 9 | IMU2_CS | SPI1 CSn (F1) | LSM6DSK320X hardware CSn, 10k pull-up R51 |
| 10 | IMU2_SCK | SPI1 SCK (F1) | LSM6DSK320X |
| 11 | IMU2_MOSI | SPI1 TX (F1) | LSM6DSK320X |
| 12 | MAG_SDA | I2C0 SDA (F3) | IO block row 5; 4.7k pull-up R25 |
| 13 | MAG_SCL | I2C0 SCL (F3) | IO block row 6; 4.7k pull-up R26 |
| 14 | GPS_RX | UART0 TX (F11 aux) | IO block row 3, silk T0; drives the GPS receiver input |
| 15 | ICM_INT1 | GPIO input | ICM-45686 INT1 |
| 16 | ICM_MISO | SPI0 RX (F1) | ICM-45686 |
| 17 | GPS_TX | UART0 RX (F2) | IO block row 4, silk R0; receives the GPS transmitter output |
| 18 | ICM_SCK | SPI0 SCK (F1) | ICM-45686 |
| 19 | ICM_MOSI | SPI0 TX (F1) | ICM-45686 |
| 20 | ICM_CS | GPIO (chip select) | ICM-45686; 10k pull-up R48 |
| 21 | BARO_INT | GPIO input | BMP581 INT |
| 22 | BARO_SDA | I2C1 SDA (F3) | BMP581; 4.7k pull-up R57 |
| 23 | BARO_SCL | I2C1 SCL (F3) | BMP581; 4.7k pull-up R50 |
| 24 | ELRS_RX | UART1 TX (F2) | IO block row 1, silk T1; MCU output |
| 25 | ELRS_TX | UART1 RX (F2) | IO block row 2, silk R1; MCU input |
| 26 | NC | GPIO | spare, no-connect |
| 27 | NC | GPIO | spare, no-connect |
| 28 | SD_DAT0 | PIO native SD DAT0 | microSD J11; 10k pull-up (R36-R40 cover CMD and DAT0-3) |
| 29 | SD_DAT1 | PIO native SD DAT1 | microSD J11; 10k pull-up (R36-R40) |
| 30 | SD_DAT2 | PIO native SD DAT2 | microSD J11; 10k pull-up (R36-R40) |
| 31 | SD_DAT3 | PIO native SD DAT3 | microSD J11; 10k pull-up (R36-R40) |
| 32 | SD_CLK_MCU | PIO native SD clock | through 22 ohm R58 to SD_CLK; no pull-up |
| 33 | SD_CMD | PIO native SD command | microSD J11; 10k pull-up (R36-R40) |
| 34 | ESC_TELEM_RX | PIO UART RX | ESC pad row, pad 2 (TX) |
| 35 | NC | GPIO | spare, no-connect |
| 36 | PWM4 | DShot via PIO | ESC M4, pad 3 |
| 37 | PWM3 | DShot via PIO | ESC M3, pad 4 |
| 38 | PWM2 | DShot via PIO | ESC M2, pad 5 |
| 39 | PWM1 | DShot via PIO | ESC M1, pad 6 |
| 40 | VBUS_SENSE | ADC0 | USB VBUS divider |
| 41 | CURR_SENSE | ADC1 | ESC current output, pad 1 |
| 42 | VBAT_SENSE | ADC2 | battery voltage divider |
| 43 | LED_DATA | PIO (WS2812C) | status LED D20 through 100 ohm R55 |
| 44 | IO_GPIO44 | GPIO / ADC4 | spare, IO block row 11, silk A44 |
| 45 | IO_GPIO45 | GPIO / ADC5 | spare, IO block row 12, silk A45 |
| 46 | IO_GPIO46 | GPIO / ADC6 | spare, IO block row 13, silk A46 |
| 47 | IO_GPIO47 | GPIO / ADC7 | spare, IO block row 14, silk A47 |

Net names follow the schematic's design intent: UART1 nets are named ELRS, UART0 nets are named GPS. A GPS_RX net is the GPS receiver input, driven by the MCU. The nets keep these names whatever is attached; see Bench configuration.

## Peripheral ledger

| Peripheral | GPIOs | Device |
|---|---|---|
| SPI0 | SCK 18, MOSI 19, MISO 16; CS 20 (plain GPIO) | ICM-45686 only (U21), 4-wire |
| SPI1 | SCK 10, MOSI 11, MISO 8, CSn 9 (hardware) | LSM6DSK320X only (U23), 4-wire |
| I2C0 | SDA 12, SCL 13 | external magnetometer port (IO block rows 5/6) |
| I2C1 | SDA 22, SCL 23 | BMP581 only, address 0x46 |
| Native 4-bit SD, PIO + DMA | DAT0-DAT3 28-31, CLK 32, CMD 33 | microSD J11 only |
| Sensor interrupts | BMP581 INT 21; ICM INT1 15; LSM6DSK320X INT1 7, INT2 6 | four separate nets and GPIOs |
| Card detect | none | the J11 socket has no detect switch |
| UART0 | TX 14 (F11 aux), RX 17 (F2) | IO block rows 3/4, silk T0/R0; hardware UART, no PIO UART |
| UART1 | TX 24, RX 25 | IO block rows 1/2, silk T1/R1 |
| Servo PWM | S5 GPIO5 (slice 2 B), S6 GPIO4 (slice 2 A), S7 GPIO3 (slice 1 B), S8 GPIO2 (slice 1 A) | IO block rows 7-10 |
| ESC outputs | M1 39, M2 38, M3 37, M4 36 | intended PIO DShot |
| ESC telemetry | 34 | PIO UART RX |
| ADC | 40 (ADC0), 41 (ADC1), 42 (ADC2) | VBUS, ESC current, battery voltage |
| Status LED | 43 | PIO, WS2812C |
| QMI CS1 | 0 | PSRAM socket U24 |
| Spare no-connect | 1, 26, 27, 35 | no-connect flags on U20 |
| USB, SWD | dedicated pins | J4, J10 |

## Connectors and silk labels

IO block: each row has J6 = signal, J7 = power, J8 = ground. All fourteen J7 power pins are V3V3_SYS.

| Row | GPIO | Silk | Function |
|---|---|---|---|
| 1 | 24 | T1 | UART1 TX (MCU output) |
| 2 | 25 | R1 | UART1 RX (MCU input) |
| 3 | 14 | T0 | UART0 TX (MCU output) |
| 4 | 17 | R0 | UART0 RX (MCU input) |
| 5 | 12 | - | magnetometer I2C0 SDA |
| 6 | 13 | - | magnetometer I2C0 SCL |
| 7 | 5 | S5 | servo PWM5 |
| 8 | 4 | S6 | servo PWM6 |
| 9 | 3 | S7 | servo PWM7 |
| 10 | 2 | S8 | servo PWM8 |
| 11-14 | 44-47 | A44-A47 | spare IO (ADC4-7) |

Silk names for rows 5/6 are not recorded in the sources.

ESC pad row J3 (back of the board), pads 1-8: CURR (GPIO41), TX (ESC telemetry, GPIO34), M4 (GPIO36), M3 (GPIO37), M2 (GPIO38), M1 (GPIO39), VIN / VBAT (pad 7, raw battery voltage, sensed on GPIO42), GND.

SWD header J10, 1x03 2.54 mm: SWCLK, GND, SWDIO (Debug Probe cable order).

## Electrical facts at the pins

| Item | Value |
|---|---|
| ICM-45686 CS (GPIO20) | 10k pull-up R48 to V3V3_ANA |
| LSM6DSK320X CS (GPIO9) | 10k pull-up R51 to V3V3_ANA |
| Sensor SPI clock and data | no pull resistors on either bus |
| BMP581 I2C | 4.7k pull-ups R57 (SDA), R50 (SCL) to V3V3_ANA; CSB tied to VDDIO (V3V3_ANA, I2C mode), SDO/ADDR to GND: address 0x46 |
| Magnetometer I2C0 | 4.7k pull-ups R25/R26 to V3V3_SYS; the module is on the V3V3_SYS power pin of the IO block |
| LSM6DSK320X pins SDx / SCx | SDx tied to V3V3_ANA, SCx to GND; 4-wire SPI |
| ICM-45686 pin 9 (INT2/FSYNC/CLKIN) | used as CLKIN, driven by the 32.000 kHz oscillator X1 (HCI 1521H-32K000J18DTSL, net ICM_CLKIN; Vcc and E/D on V3V3_ANA, 100 nF on Vcc); not connected to any MCU pin |
| ICM-45686 interrupts | INT1 only reaches the MCU (GPIO15); there is no INT2 on the MCU |
| microSD | 10k pull-ups to V3V3_SYS on CMD and DAT0-3 (R36-R40); none on CLK; GPIO32 reaches SD_CLK through 22 ohm R58 |
| PSRAM socket U24 | CS GPIO0, 10k pull-up R35; clock and SD0-3 are the MCU's dedicated QSPI pins shared with the in-package flash; supply V3V3_SYS; no pull-ups on SD2/SD3 |
| VBUS sense (GPIO40, ADC0) | 0.6 x VBUS (R29 10k top, R30 15k bottom, 100n) |
| Current sense (GPIO41, ADC1) | 1:1 through 1k R53 with 100n at the pin |
| VBAT sense (GPIO42, ADC2) | VBAT / 11 (R27 100k, R28 10k, 100n) |
| ADC_AVDD | V3V3_ANA (TPS7A2033 LDO), the same rail as the sensors |
| Status LED | WS2812C-2020 D20, data on GPIO43 through 100 ohm R55, powered from V5_SYS |

Rails: V3V3_ANA (sensors, their pull-ups, X1, ADC_AVDD) is enabled from V3V3_SYS, so it comes up after V3V3_SYS. V3V3_SYS powers the microSD, its pull-ups, the PSRAM socket and the IO block power pins.

## Constraints

- SD DAT0-DAT3 are consecutive in ascending GPIO order (a PIO `in pins` requirement). All six SD signals use GPIO28-33, in the GPIO16-47 PIO window. The LED and ESC PIO signals need the same window, so every PIO user shares GPIOBASE = 16.
- PWM frequency is shared per slice. Servo S5/S6 share slice 2 and S7/S8 share slice 1. A hardware-PWM fallback for the ESC outputs would put M4/M3 on slice 10 (shared with GPIO44/45) and M2/M1 on slice 11 (shared with GPIO46/47). PIO DShot does not use those slices.
- There is no SD card-detect.
- A pin alternate does not create an extra controller. UART0 is the controller on the T0/R0 port (GPIO14/17), so its TX/RX alternates on GPIO44/45 cannot be used at the same time. UART1 is the controller on the T1/R1 port (GPIO24/25), so the F11 UART1 TX/RX functions on GPIO26/27 cannot be used alongside it. SPI1 and I2C0 already serve onboard sensors, which limits the alternates of GPIO44/45. PWM11 A is shared between GPIO46 and GPIO38, PWM11 B between GPIO47 and GPIO39.

## Spare pins

| GPIO | Exposed | Alternates (see constraints above) |
|---|---|---|
| 44-47 | IO block rows 11-14 (A44-A47) | ADC4-7; PWM10 A/B on 44/45, PWM11 A/B on 46/47; UART0 TX on 44, RX on 45 |
| 1 | no-connect | SPI0 CSn, UART0 RX, I2C0 SCL, PWM0 B, GPIO |
| 26 | no-connect | SPI1 SCK, I2C1 SDA, PWM5 A, UART1 TX (F11 aux) |
| 27 | no-connect | SPI1 TX, I2C1 SCL, PWM5 B, UART1 RX (F11 aux) |
| 35 | no-connect | PWM9 B, PIO, UART0 RX (F11 aux), GPIO (PIO window 16-47) |

## Bench configuration

Attached by hand on the current board; the PCB itself provides the pins above.

- LC76G GPS on silk T1/R1: T1 = GPIO24 = UART1 TX (MCU output, to GPS RX); R1 = GPIO25 = UART1 RX (MCU input, from GPS TX). PPS on silk S5 = GPIO5, a plain GPIO input (S5 is otherwise the servo PWM5 pad).
- IS66WVS16M8FBLL PSRAM hand-fitted in socket U24 (SOIC-8): CS on GPIO0 (QMI CS1n), shared QSPI clock/data, supply V3V3_SYS.
- BMM350 magnetometer on the magnetometer port: SDA = GPIO12, SCL = GPIO13 (I2C0, 4.7k pull-ups to V3V3_SYS, powered from the V3V3_SYS pin of the IO block). INT on silk S6 = GPIO4, a plain GPIO input (S6 is otherwise the servo PWM6 pad).
- Motors (owner, 2026-11-13; four Emax ECO II 2807 1700KV, 14 magnets, on the GOKU G55M with AM32 2.20; directions seen in the spin test at 5 % throttle, no propellers): M1 = GPIO39 = rear left, counter-clockwise; M2 = GPIO38 = front left, clockwise; M3 = GPIO37 = rear right, clockwise; M4 = GPIO36 = front right, counter-clockwise. The direction is a setting of each ESC (reversed on M1's and M4's).
- T0/R0 (GPIO14/17, UART0) is assigned to the ELRS receiver (owner, 2026-11-13: "T0 R0 are the ELRS receiver"); whether it is attached yet, and its supply, are not recorded. The current-sense source pad is floating.
