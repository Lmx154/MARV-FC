# Hardware Pinouts for MARV Project PCB

This document outlines the hardware configurations and pin assignments for the MARV project, including the Flight Controller (FC), Radio, and Ground Station (GS). All microcontrollers (MCs) are Raspberry Pi Pico 2 boards based on the RP2350A.

## Flight Controller (FC)

- **Microcontroller (MC)**: RP2354B

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

  - **I2C1**: BMM350, Ublox NEO M9N GPS
    - SDA: GP2
    - SCL: GP3
    - BMM350 Address: 0x14
    - Ublox NEO M9N GPS Address: 0x42
    
  - **RGB LED (SK6805-EC15 addressable LED)**:
    - GP6

# External Pads 
  - **FC-RADIO Connection**: UART0
    - TX: GP0 
    - RX: GP1 
  - **FC-SBC Connection**: UART1
    - TX: GP4
    - RX: GP5
  - **External I2C0**: I2C0
    - SDA: GP8
    - SCL: GP9
  - **External I2C1**: I2C1
    - SDA: GP2
    - SCL: GP3
  - **External GPIO**: GPIO
    - IO1: GP28
    - IO2: GP29
    - IO3: GP30
    - IO4: GP31
  - **ESC Pads**
    - C/CURRENT_SENSE: 
    - 1/PWM1: GP39
    - 2/PWM2: GP38
    - 3/PWM3: GP35
    - 4/PWM4: GP36
    - T/TELEMETRY: GP37
  - **VTX Pads**
    - TX/VTX_TX: GP32
    - RX/VTX_RX: GP33



## Radio/GS

- **Microcontroller (MC)**: RP2354A

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
