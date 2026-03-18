# Payload Controller

## Board

- RP2350 Pico 2

## Peripheral Entrypoints

### I2C0

- SDA: GP0
- SCL: GP1

### SPI0

- SCK: GP2
- MOSI: GP3
- MISO: GP4
- CS: GP5

### ADC0

- Pressure transducer input: GP29

## Devices

- I2C0: BMP388
- ADC0 / GP29: 0-150 PSI analog pressure transducer via voltage divider
- SPI0: microSD card
- Status LED: GP25
- Servo output: GP28

## Runtime Services

- USB CDC console
- BMP388 acquisition on I2C0
- Pressure transducer acquisition on ADC0 / GP29
- DS3240-style PWM servo deployment on GP28 when barometric altitude exceeds 8,000 ft
- SD sensor snapshot logging for BMP388 and pressure transducer samples
