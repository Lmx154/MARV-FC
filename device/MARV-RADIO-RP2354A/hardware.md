# MARV-RADIO-RP2354A Hardware

This target uses one firmware crate for two physical roles:

- `radio`: connected to the flight controller over UART0.
- `ground-station`: connected to a CP2102 USB-UART bridge over UART0.

Select the role at compile time with Cargo features. The default is `radio`.

```sh
cargo build -p marv-radio-rp2354a
cargo build -p marv-radio-rp2354a --no-default-features --features ground-station
```

## IREC 33 cm Mode C Build Configuration

For 33 cm Mode C operation, build both the rocket radio and any transmitting ground-station
radio with the amateur callsign and the FFRR-assigned SRAD center frequency:

```sh
MARV_AMATEUR_CALLSIGN=N0CALL MARV_LORA_FREQUENCY_HZ=902080000 cargo build -p marv-radio-rp2354a
MARV_AMATEUR_CALLSIGN=N0CALL MARV_LORA_FREQUENCY_HZ=902080000 cargo build -p marv-radio-rp2354a --no-default-features --features ground-station
```

The firmware uses a fixed 62.5 kHz LoRa bandwidth Mode C profile and transmits a clear
ASCII `DE <CALLSIGN>` station-ID frame every 9 minutes, ahead of normal telemetry or
command traffic. The build script validates that the occupied bandwidth fits within the
902.0-909.0 MHz SRAD range; the center frequency still must be the frequency assigned by
ESRA for the flight.

## Microcontroller

- RP2354A
- Embassy/HAL feature: `rp235xa`

## LoRa Radio

Waveshare SX1262 HF 915 MHz on SPI0.

| Signal | RP2354A GPIO |
| --- | --- |
| MISO | GP0 |
| CS | GP1 |
| SCK/CLK | GP2 |
| MOSI | GP3 |
| BUSY | GP4 |
| RESET | GP5 |
| RXEN | GP6 |
| TXEN | GP7 |
| DIO2 | GP8 |
| DIO1 | GP9 |

## Host UART

UART0 is the local host-facing serial link. The firmware role decides what is on the other side of this same electrical interface.

| Role | Peer | UART | TX | RX |
| --- | --- | --- | --- | --- |
| `radio` | MARV-FC | UART0 | GP12 | GP13 |
| `ground-station` | CP2102 | UART0 | GP12 | GP13 |

## Status LED

| Signal | RP2354A GPIO |
| --- | --- |
| SK6805-EC15 data | GP11 |

## Reserved

| GPIO | Purpose |
| --- | --- |
| GP10 | Reserved for board revision strapping or future debug use |
