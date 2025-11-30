# MARV (Modular Avionics for Rockets and Vehicles) Flight Controller

MARV is an avionics system aimed at growing my understanding of avionics system and is my exploration into the lowest level components of avionics. This repository is the firmware designed to be flashed into the MARV flight controller hardware. 


- sensors available
- devices available
- tasks
- simulation

TODO:
- async spi SD blackbox storage logs and config

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
