# Hardware Pinouts for MARV Project PCB

This document outlines the hardware configurations and pin assignments for the MARV project, including the Flight Controller (FC), Radio, and Ground Station (GS). All microcontrollers (MCs) are Raspberry Pi Pico 2 boards based on the RP2350A.

## LoRa sync word

- This project uses a custom private LoRa sync word: `0x47`.
- The value is configured in `common/src/coms/transport/lora/rf_config.rs` and
  must match on both GS and radio.

## Flight Controller (FC)

- **Microcontroller (MC)**: Waveshare RP2350B Core - RP2350B

- **Sensors and Interfaces**:
  - **IMU - BMI088 (dual CS) & LSM6DSV32X** : SPI1 Shared 
    - MOSI: GP11
    - MISO: GP12
    - SCK: GP10
    - CS_ACCEL: GP13
    - CS_GYRO: GP14
    - CS_LSM: GP16
  - **microSD Card**: SPI0
    - MOSI: GP19
    - MISO: GP20
    - SCK: GP18
    - CS: GP21
  - **I2C0**: BMP581
    - SDA: GP8
    - SCL: GP9
    - BMP390 Address: 0x77
  - **I2C1**: BMM350, Ublox NEO M9N GPS
    - SDA: GP2
    - SCL: GP3
    - BMM350 Address: 0x14
    - Ublox NEO M9N GPS Address: 0x42
  - **RGB LED (SK6805-EC15 addressable LED)**:
    - GP6
  - **Buzzer (Active buzzer PWM)**:
    - GP5
  - **FC-RADIO Connection**: UART0
    - TX: GP14 
    - RX: GP15 
  - **FC-SBC Connection**: UART1
    - TX: GP20
    - RX: GP21
  # PWM pins 
    - **Rocket**
      - **Servo Controller**
        - AIR_BRK: GP
        - ROLL_CTRL1: GP
        - ROLL_CTRL2: GP
        - ROLL_CTRL3: GP
        - ROLL_CTRL4: GP
        - THVCTR_CTRL1: GP
        - THVCTR_CTRL2: GP
    - **Drone**
      - **Electronic Speed Controller**
        - MOTOR1: GP
        - MOTOR2: GP
        - MOTOR3: GP
        - MOTOR4: GP



## Radio/GS

- **Microcontroller (MC)**: Raspberry Pi Pico 2 - RP2350A

- **Sensors and Interfaces**:
  - **LoRa Radio - Waveshare SX1262 HF (915Mhz)**: SPI0
    - CS: GP1
    - CLK: GP2
    - MOSI: GP3 
    - MISO: GP0 
    - RESET: GP5 
    - BUSY: GP4
    - DIO2: GP8
    - DIO1: GP9
    - TXEN: GP7
    - RXEN: GP6 

  - **RADIO-FC Connection**: UART0
    - TX: GP12 
    - RX: GP13 

  - **RGB LED (SK6805-EC15 addressable LED)**:
    - GP11

# Pinouts do not matter for external sensors, so long as they are wired into the correct bus and the bus is specified during the sensor initialization. 
