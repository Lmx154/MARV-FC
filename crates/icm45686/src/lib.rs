//! Driver for the TDK InvenSense ICM-45686 six-axis IMU over SPI.
//!
//! Built on the `embedded-hal` 1.0 traits only: it works with any executor or none. The crate root
//! is the async driver ([`Icm45686`], `embedded-hal-async`); [`blocking`] is the same driver on the
//! blocking traits. The device knowledge (registers, configuration, decoding, unit conversion) is
//! pure code in [`reg`], [`config`] and [`sample`].
//!
//! The driver owns no interrupt pin and no timer: `init` can route data-ready to INT1 and the
//! application waits on the pin.
//!
//! Units: [`RawSample::accel_ms2`] is m/s^2, [`RawSample::gyro_rads`] is rad/s,
//! [`RawSample::temperature_c`] is degrees Celsius.
//!
//! Async:
//! ```ignore
//! let mut imu = icm45686::Icm45686::new(spi_device);
//! let config = icm45686::Config { int1_data_ready: true, ..Default::default() };
//! imu.init(&mut delay, &config).await?;
//! loop {
//!     int1.wait_for_rising_edge().await;
//!     let sample = imu.read().await?;
//!     let accel = sample.accel_ms2(config.accel_fs);
//! }
//! ```
//!
//! Blocking:
//! ```ignore
//! let mut imu = icm45686::blocking::Icm45686::new(spi_device);
//! imu.init(&mut delay, &icm45686::Config::default())?;
//! let sample = imu.read()?;
//! let temperature = sample.temperature_c();
//! ```
#![cfg_attr(not(test), no_std)]
#![forbid(unsafe_code)]

pub mod blocking;
pub mod config;
pub mod reg;
pub mod sample;

use embedded_hal_async::delay::DelayNs;
use embedded_hal_async::spi::{Operation, SpiDevice};

pub use config::{AccelFullScale, ClockSource, Config, GyroFullScale, Odr};
pub use sample::RawSample;

const READ: u8 = 0x80;
/// Minimum gap between indirect-register accesses is 4 us; this leaves margin.
const IREG_WAIT_US: u32 = 10;
/// Soft-reset polling: first wait, then up to `RESET_POLLS` polls this far apart.
const RESET_FIRST_WAIT_US: u32 = 1_000;
const RESET_POLL_US: u32 = 500;
const RESET_POLLS: u32 = 100;
/// Gyroscope start-up is 35 ms; `init` returns after this so that the first `read` is valid.
const START_UP_MS: u32 = 50;

/// Driver errors.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum Error<E> {
    /// SPI bus error.
    Bus(E),
    /// WHO_AM_I did not read 0xE9.
    WrongId {
        /// Value read.
        found: u8,
    },
    /// RESET_DONE did not appear after a soft reset.
    ResetTimeout,
    /// The configuration cannot be applied (output data rate not supported in low-noise mode).
    InvalidConfig,
}

/// ICM-45686 on an async SPI device (mode 0 or 3, up to 24 MHz).
pub struct Icm45686<SPI> {
    spi: SPI,
}

impl<SPI: SpiDevice> Icm45686<SPI> {
    /// Wraps the SPI device. Nothing is sent until `init`.
    pub const fn new(spi: SPI) -> Self {
        Self { spi }
    }

    /// Returns the SPI device.
    pub fn release(self) -> SPI {
        self.spi
    }

    /// Soft reset, identity check, clock, full scale and rate, low-noise mode on both sensors and the
    /// INT1 pin. Returns after the sensors have started up.
    pub async fn init(
        &mut self,
        delay: &mut impl DelayNs,
        config: &Config,
    ) -> Result<(), Error<SPI::Error>> {
        if !config.odr.low_noise() {
            return Err(Error::InvalidConfig);
        }
        self.soft_reset(delay).await?;
        let found = self.read_register(reg::WHO_AM_I).await?;
        if found != reg::WHO_AM_I_VALUE {
            return Err(Error::WrongId { found });
        }
        self.write_register(reg::INT1_CONFIG0, 0).await?;
        self.write_register(reg::INT1_CONFIG2, reg::INT1_CONFIG2_PUSH_PULL_PULSE_HIGH)
            .await?;
        if config.clock == ClockSource::ExternalClkin {
            self.external_clock(delay).await?;
        }
        self.write_register(
            reg::ACCEL_CONFIG0,
            config::accel_config0(config.accel_fs, config.odr),
        )
        .await?;
        self.write_register(
            reg::GYRO_CONFIG0,
            config::gyro_config0(config.gyro_fs, config.odr),
        )
        .await?;
        self.write_register(
            reg::INT1_CONFIG0,
            config::int1_config0(config.int1_data_ready),
        )
        .await?;
        self.write_register(reg::PWR_MGMT0, reg::PWR_MGMT0_LOW_NOISE)
            .await?;
        delay.delay_ms(START_UP_MS).await;
        Ok(())
    }

    /// Reads the latest sample in one SPI transaction.
    pub async fn read(&mut self) -> Result<RawSample, Error<SPI::Error>> {
        let mut buf = [0u8; 15];
        buf[0] = reg::ACCEL_DATA_X1_UI | READ;
        self.spi
            .transfer_in_place(&mut buf)
            .await
            .map_err(Error::Bus)?;
        let mut data = [0u8; 14];
        data.copy_from_slice(&buf[1..]);
        Ok(RawSample::decode(&data))
    }

    /// Reads one register.
    pub async fn read_register(&mut self, addr: u8) -> Result<u8, Error<SPI::Error>> {
        let mut b = [0u8; 1];
        self.read_registers(addr, &mut b).await?;
        Ok(b[0])
    }

    /// Reads `buf.len()` consecutive registers starting at `addr` in one transaction.
    pub async fn read_registers(
        &mut self,
        addr: u8,
        buf: &mut [u8],
    ) -> Result<(), Error<SPI::Error>> {
        let cmd = [addr | READ];
        self.spi
            .transaction(&mut [Operation::Write(&cmd), Operation::Read(buf)])
            .await
            .map_err(Error::Bus)
    }

    /// Writes one register.
    pub async fn write_register(&mut self, addr: u8, value: u8) -> Result<(), Error<SPI::Error>> {
        self.write_registers(addr, &[value]).await
    }

    /// Writes consecutive registers starting at `addr` in one transaction.
    pub async fn write_registers(
        &mut self,
        addr: u8,
        data: &[u8],
    ) -> Result<(), Error<SPI::Error>> {
        let cmd = [addr & !READ];
        self.spi
            .transaction(&mut [Operation::Write(&cmd), Operation::Write(data)])
            .await
            .map_err(Error::Bus)
    }

    /// Reads an indirect register (`reg::IPREG_*` base plus offset).
    pub async fn ireg_read(
        &mut self,
        delay: &mut impl DelayNs,
        addr: u16,
    ) -> Result<u8, Error<SPI::Error>> {
        let [hi, lo] = addr.to_be_bytes();
        self.write_registers(reg::IREG_ADDR_15_8, &[hi, lo]).await?;
        delay.delay_us(IREG_WAIT_US).await;
        let value = self.read_register(reg::IREG_DATA).await?;
        delay.delay_us(IREG_WAIT_US).await;
        Ok(value)
    }

    /// Writes an indirect register (`reg::IPREG_*` base plus offset).
    pub async fn ireg_write(
        &mut self,
        delay: &mut impl DelayNs,
        addr: u16,
        value: u8,
    ) -> Result<(), Error<SPI::Error>> {
        let [hi, lo] = addr.to_be_bytes();
        self.write_registers(reg::IREG_ADDR_15_8, &[hi, lo, value])
            .await?;
        delay.delay_us(IREG_WAIT_US).await;
        Ok(())
    }

    async fn ireg_modify(
        &mut self,
        delay: &mut impl DelayNs,
        addr: u16,
        mask: u8,
        value: u8,
    ) -> Result<(), Error<SPI::Error>> {
        let old = self.ireg_read(delay, addr).await?;
        self.ireg_write(delay, addr, (old & !mask) | (value & mask))
            .await
    }

    async fn soft_reset(&mut self, delay: &mut impl DelayNs) -> Result<(), Error<SPI::Error>> {
        self.read_register(reg::INT1_STATUS0).await?;
        self.write_register(reg::REG_MISC2, reg::SOFT_RST).await?;
        delay.delay_us(RESET_FIRST_WAIT_US).await;
        for _ in 0..RESET_POLLS {
            let status = self.read_register(reg::INT1_STATUS0).await?;
            if status != 0xFF && status & reg::RESET_DONE != 0 {
                return Ok(());
            }
            delay.delay_us(RESET_POLL_US).await;
        }
        Err(Error::ResetTimeout)
    }

    /// Sequence from TDK's reference driver that switches the sensor clock to pin 9 (CLKIN).
    async fn external_clock(&mut self, delay: &mut impl DelayNs) -> Result<(), Error<SPI::Error>> {
        self.write_register(reg::IOC_PAD_SCENARIO_OVRD, 0x06)
            .await?;
        self.ireg_modify(delay, reg::SIFS_I3C_STC_CFG, 0x04, 0x00)
            .await?;
        self.ireg_modify(delay, reg::IPREG_SYS2_REG_123, 0x03, 0x02)
            .await?;
        self.ireg_modify(delay, reg::IPREG_SYS1_REG_166, 0x60, 0x40)
            .await?;
        self.ireg_modify(delay, reg::SMC_CONTROL_0, 0x10, 0x10)
            .await?;
        let rtc = self.read_register(reg::RTC_CONFIG).await?;
        self.write_register(reg::RTC_CONFIG, rtc | reg::RTC_MODE)
            .await
    }
}

#[cfg(test)]
mod tests;
