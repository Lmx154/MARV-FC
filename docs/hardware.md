# Hardware Pinouts for MARV Project

This document outlines the hardware configurations and pin assignments for the MARV project, including the Flight Controller (FC), Radio, and Ground Station (GS). All microcontrollers (MCs) are Raspberry Pi Pico 2 boards based on the RP2350A.

## Flight Controller (FC)

- **Microcontroller (MC)**: Waveshare RP2350B Core - RP2350B

- **Sensors and Interfaces**:
  - **IMU - BMI088 (dual CS)**: SPI0
    - MOSI: GP3
    - MISO: GP0
    - SCK: GP2
    - CS_ACCEL: GP1
    - CS_GYRO: GP4
  - **microSD Card**: SPI1
    - MOSI: GP11
    - MISO: GP8
    - SCK: GP10
    - CS: GP9
  - **I2C0**: BMP390
    - SDA: GP12
    - SCL: GP13
    - BMP390 Address: 0x77
  - **I2C1**: BMM350, Ublox NEO M9N GPS
    - SDA: GP6
    - SCL: GP7
    - BMM350 Address: 0x14
    - Ublox NEO M9N GPS Address: 0x42
  - **RGB LED (SK6812/WS2812-style addressable LED)**:
    - GP16
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
        - AIR_BRK: GP22
        - ROLL_CTRL1: GP22
        - ROLL_CTRL2: GP22
        - ROLL_CTRL3: GP22
        - ROLL_CTRL4: GP22
        - THVCTR_CTRL1: GP22
        - THVCTR_CTRL2: GP22
    - **Drone**
      - **Electronic Speed Controller**
        - MOTOR1: GP22
        - MOTOR2: GP22
        - MOTOR3: GP22
        - MOTOR4: GP22



## Radio

- **Microcontroller (MC)**: Raspberry Pi Pico 2 - RP2350A

- **Sensors and Interfaces**:
  - **LoRa Radio - Waveshare SX1262 HF (915Mhz)**: SPI0
    - CS: GP5
    - CLK: GP2
    - MOSI: GP3 
    - MISO: GP4 
    - RESET: GP1 
    - BUSY: GP0
    - DIO2: GP7 
    - DIO1: GP6
    - TXEN: GP8
    - RXEN: GP9 

  - **RADIO-FC Connection**: UART0
    - TX: GP12 
    - RX: GP13 

  - **RGB LED (SK6812/WS2812-style addressable LED)**:
    - GP14

## Ground Station (GS)

- **Microcontroller (MC)**: Raspberry Pi Pico 2 - RP2350A

- **Sensors and Interfaces**:
  - **LoRa Radio - Waveshare SX1262 HF (915Mhz)**: SPI0
      - CS: GP5
      - CLK: GP2
      - MOSI: GP3 
      - MISO: GP4 
      - RESET: GP1 
      - BUSY: GP0
      - DIO2: GP7 
      - DIO1: GP6 
      - TXEN: GP8
      - RXEN: GP9

  - **RGB LED (SK6812/WS2812-style addressable LED)**:
    - GP14

# Pinouts do not matter for external sensors, so long as they are wired into the correct bus and the bus is specified during the sensor initialization. 

