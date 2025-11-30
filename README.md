# MARV (Modular Avionics for Rockets and Vehicles) Flight Controller

MARV is an avionics system aimed at growing my understanding of avionics system and is my exploration into the lowest level components of avionics. This repository is the firmware designed to be flashed into the MARV flight controller hardware. 


- sensors available
- devices available
- tasks
- simulation

## SD card config (CONFIG.TXT)

At boot the FC reads a simple key/value file `CONFIG.TXT` on the SD card (root). Current keys:

- `led_color` (RGB as comma-separated bytes, e.g. `0,16,0` for dim green)
- `imu_hz` (BMI088 sample rate in Hz)
- `mag_hz` (BMM350 sample rate in Hz)
- `baro_hz` (BMP390 sample rate in Hz)
- `gps_hz` (GPS poll rate in Hz)

Example:
```
led_color=0,16,0
imu_hz=1000
mag_hz=25
baro_hz=250
gps_hz=20
```

Missing keys fall back to defaults. Config is applied once at boot before tasks start.

TODO:

- create parameter based system

- create simple mavlink usbcdc portals for FC and GS
    - FC (Configuration mode)
        - GUI connect via serial in fc conf mode
        - communicate via mavlink
        - retrieve conf parameters
        - set conf parameters
    - GS (Normal mode)
        - GS connect via serial in GCS mode
        - communicate via mavlink
        - retrieve uav state
        - capable of real-time arming/disarming/control
        - real-time telemetry stream

- use i2c async for adxl375 driver
    - pending a working sensor to check

- re-factor lora link layer
    - connection logic rework
    - rssi based bandwidth throttling logic (currently having issues with many lost/misformed packets when gs and radio link extends past 1 meter)

- re-factor mavlink layer
    - code reusability pretty terrible rn

- add new sensor driver for adc devices
    - reusable generic adc driver
    - air pressure gauge
    - strain gauge
    - temperature probe

- ekf

- pid controller

- motor control
    - dshot comm from fc to esc
    - vehicle profiles
    - motor drivers for stepper, servo, brushless
