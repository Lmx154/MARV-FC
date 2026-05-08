# MARV-RADIO-RP2354A Hardware

This target uses one firmware crate for two physical roles:

- `radio`: connected to the flight controller over UART0.
- `ground-station`: connected to a CP2102 USB-UART bridge over UART0.

Select the role at compile time with Cargo features. The default is `radio`.

```sh
cargo build -p marv-radio-rp2354a
cargo build -p marv-radio-rp2354a --no-default-features --features ground-station
```

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
