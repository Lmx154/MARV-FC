//! Register map, names as in the datasheet (DS-000489 Rev 1.1, section 16 and 19).

/// First accelerometer data register, ACCEL_DATA_X1_UI. The 14 bytes from here are
/// accel X, Y, Z, gyro X, Y, Z, temperature, each a little-endian `i16`.
pub const ACCEL_DATA_X1_UI: u8 = 0x00;
/// First gyroscope data register, GYRO_DATA_X1_UI.
pub const GYRO_DATA_X1_UI: u8 = 0x06;
/// First temperature data register, TEMP_DATA1_UI.
pub const TEMP_DATA1_UI: u8 = 0x0C;
/// Power management: gyro mode in bits 3:2, accel mode in bits 1:0.
pub const PWR_MGMT0: u8 = 0x10;
/// INT1 status enables; bit 7 reset done, bit 2 data ready.
pub const INT1_CONFIG0: u8 = 0x16;
/// INT1 pin: bit 2 drive, bit 1 mode, bit 0 polarity.
pub const INT1_CONFIG2: u8 = 0x18;
/// INT1 status flags, cleared on read; bit 7 reset done, bit 2 data ready.
pub const INT1_STATUS0: u8 = 0x19;
/// Accelerometer full scale (bits 6:4) and output data rate (bits 3:0).
pub const ACCEL_CONFIG0: u8 = 0x1B;
/// Gyroscope full scale (bits 7:4) and output data rate (bits 3:0).
pub const GYRO_CONFIG0: u8 = 0x1C;
/// Real-time clock configuration; bit 5 RTC_MODE selects the external CLKIN clock.
pub const RTC_CONFIG: u8 = 0x26;
/// Device identity, reads [`WHO_AM_I_VALUE`].
pub const WHO_AM_I: u8 = 0x72;
/// Indirect register address, bits 15:8.
pub const IREG_ADDR_15_8: u8 = 0x7C;
/// Indirect register address, bits 7:0.
pub const IREG_ADDR_7_0: u8 = 0x7D;
/// Indirect register data.
pub const IREG_DATA: u8 = 0x7E;
/// Bit 1 SOFT_RST (self-clearing), bit 0 IREG_DONE.
pub const REG_MISC2: u8 = 0x7F;
/// Not in the datasheet; from TDK's reference driver. Needed to route pin 9 to CLKIN.
pub const IOC_PAD_SCENARIO_OVRD: u8 = 0x31;

/// Expected WHO_AM_I value.
pub const WHO_AM_I_VALUE: u8 = 0xE9;
/// REG_MISC2 value that starts a soft reset.
pub const SOFT_RST: u8 = 0x02;
/// INT1_STATUS0 / INT1_CONFIG0 bit 7: reset done.
pub const RESET_DONE: u8 = 0x80;
/// INT1_STATUS0 / INT1_CONFIG0 bit 2: data ready.
pub const DRDY: u8 = 0x04;
/// PWR_MGMT0 with accel and gyro in low-noise mode.
pub const PWR_MGMT0_LOW_NOISE: u8 = 0x0F;
/// INT1_CONFIG2: push-pull, pulsed, active high.
pub const INT1_CONFIG2_PUSH_PULL_PULSE_HIGH: u8 = 0x01;
/// RTC_CONFIG bit 5, RTC_MODE.
pub const RTC_MODE: u8 = 0x20;

/// IPREG_TOP1 base.
pub const IPREG_TOP1: u16 = 0xA200;
/// IPREG_SYS1 base.
pub const IPREG_SYS1: u16 = 0xA400;
/// IPREG_SYS2 base.
pub const IPREG_SYS2: u16 = 0xA500;
/// SMC_CONTROL_0: bit 4 ACCEL_LP_CLK_SEL, bit 3 TEMP_DIS.
pub const SMC_CONTROL_0: u16 = IPREG_TOP1 + 0x58;
/// SIFS_I3C_STC_CFG: bit 2 must be 0 for CLKIN.
pub const SIFS_I3C_STC_CFG: u16 = IPREG_TOP1 + 0x68;
/// IPREG_SYS1_REG_166, GYRO_SRC_CTRL in bits 6:5.
pub const IPREG_SYS1_REG_166: u16 = IPREG_SYS1 + 0xA6;
/// IPREG_SYS2_REG_123, ACCEL_SRC_CTRL in bits 1:0.
pub const IPREG_SYS2_REG_123: u16 = IPREG_SYS2 + 0x7B;
