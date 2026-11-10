use std::convert::Infallible;

use embedded_hal::delay::DelayNs;
use embedded_hal::spi::{ErrorType, Operation, SpiDevice};

use crate::config::*;
use crate::*;

#[test]
fn enums_encode_to_datasheet_codes() {
    let accel = [
        AccelFullScale::G32,
        AccelFullScale::G16,
        AccelFullScale::G8,
        AccelFullScale::G4,
        AccelFullScale::G2,
    ];
    for (i, fs) in accel.into_iter().enumerate() {
        assert_eq!(fs.code(), i as u8);
    }
    let gyro = [
        GyroFullScale::Dps4000,
        GyroFullScale::Dps2000,
        GyroFullScale::Dps1000,
        GyroFullScale::Dps500,
        GyroFullScale::Dps250,
        GyroFullScale::Dps125,
        GyroFullScale::Dps62_5,
        GyroFullScale::Dps31_25,
        GyroFullScale::Dps15_625,
    ];
    for (i, fs) in gyro.into_iter().enumerate() {
        assert_eq!(fs.code(), i as u8);
    }
    let odr = [
        (Odr::Hz6400, 0x3, 6400.0),
        (Odr::Hz3200, 0x4, 3200.0),
        (Odr::Hz1600, 0x5, 1600.0),
        (Odr::Hz800, 0x6, 800.0),
        (Odr::Hz400, 0x7, 400.0),
        (Odr::Hz200, 0x8, 200.0),
        (Odr::Hz100, 0x9, 100.0),
        (Odr::Hz50, 0xA, 50.0),
        (Odr::Hz25, 0xB, 25.0),
        (Odr::Hz12_5, 0xC, 12.5),
        (Odr::Hz6_25, 0xD, 6.25),
        (Odr::Hz3_125, 0xE, 3.125),
        (Odr::Hz1_5625, 0xF, 1.5625),
    ];
    for (o, code, hz) in odr {
        assert_eq!(o.code(), code);
        assert_eq!(o.hz(), hz);
        assert_eq!(o.low_noise(), code <= 0xC);
    }
    assert_eq!(accel_config0(AccelFullScale::G4, Odr::Hz200), 0x38);
    assert_eq!(gyro_config0(GyroFullScale::Dps250, Odr::Hz6400), 0x43);
}

#[test]
fn decode_little_endian() {
    let b = [
        0x01, 0x00, 0xFF, 0xFF, 0x00, 0x80, 0x34, 0x12, 0x00, 0x00, 0xFF, 0x7F, 0x80, 0x00,
    ];
    let s = RawSample::decode(&b);
    assert_eq!(s.accel, [1, -1, i16::MIN]);
    assert_eq!(s.gyro, [0x1234, 0, i16::MAX]);
    assert_eq!(s.temp, 128);
}

#[test]
fn unit_conversions() {
    let s = RawSample {
        accel: [8192, 0, 0],
        gyro: [0, 131, 0],
        temp: -128 * 25,
    };
    let a = s.accel_ms2(AccelFullScale::G4);
    assert!((a[0] - 9.80665).abs() < 1e-4);
    let g = s.gyro_rads(GyroFullScale::Dps250);
    assert!((g[1] - 1.0_f32.to_radians()).abs() < 1e-3);
    assert_eq!(s.temperature_c(), 0.0);
}

struct FakeSpi {
    regs: [u8; 256],
}

impl ErrorType for FakeSpi {
    type Error = Infallible;
}

impl SpiDevice for FakeSpi {
    fn transaction(&mut self, ops: &mut [Operation<'_, u8>]) -> Result<(), Infallible> {
        let mut cmd: Option<u8> = None;
        let mut step = |regs: &mut [u8; 256], byte: &mut u8, input: bool| match cmd {
            None => cmd = Some(*byte),
            Some(c) => {
                let a = (c & 0x7F) as usize;
                if c & 0x80 != 0 {
                    *byte = regs[a];
                } else if input {
                    regs[a] = *byte;
                }
                cmd = Some(c.wrapping_add(1));
            }
        };
        for op in ops {
            match op {
                Operation::Write(w) => w
                    .iter()
                    .for_each(|b| step(&mut self.regs, &mut b.clone(), true)),
                Operation::Read(r) => r.iter_mut().for_each(|b| step(&mut self.regs, b, false)),
                Operation::TransferInPlace(t) => {
                    t.iter_mut().for_each(|b| step(&mut self.regs, b, true))
                }
                _ => unreachable!(),
            }
        }
        Ok(())
    }
}

struct NoDelay;

impl DelayNs for NoDelay {
    fn delay_ns(&mut self, _ns: u32) {}
}

#[test]
fn blocking_init_then_read() {
    let mut regs = [0u8; 256];
    regs[reg::WHO_AM_I as usize] = reg::WHO_AM_I_VALUE;
    regs[reg::INT1_STATUS0 as usize] = reg::RESET_DONE;
    for (i, b) in regs[..14].iter_mut().enumerate() {
        *b = if i % 2 == 0 { i as u8 + 1 } else { 0 };
    }
    let mut imu = blocking::Icm45686::new(FakeSpi { regs });
    let config = Config {
        accel_fs: AccelFullScale::G8,
        gyro_fs: GyroFullScale::Dps500,
        odr: Odr::Hz200,
        clock: ClockSource::ExternalClkin,
        int1_data_ready: true,
    };
    imu.init(&mut NoDelay, &config).unwrap();
    let s = imu.read().unwrap();
    assert_eq!(s.accel, [1, 3, 5]);
    assert_eq!(s.gyro, [7, 9, 11]);
    assert_eq!(s.temp, 13);

    let r = imu.release().regs;
    assert_eq!(r[reg::ACCEL_CONFIG0 as usize], 0x28);
    assert_eq!(r[reg::GYRO_CONFIG0 as usize], 0x38);
    assert_eq!(r[reg::INT1_CONFIG0 as usize], 0x04);
    assert_eq!(r[reg::INT1_CONFIG2 as usize], 0x01);
    assert_eq!(r[reg::PWR_MGMT0 as usize], 0x0F);
    assert_eq!(r[reg::RTC_CONFIG as usize], 0x20);
    assert_eq!(r[reg::IOC_PAD_SCENARIO_OVRD as usize], 0x06);
    // Last indirect write: SMC_CONTROL_0 (0xA258), bit 4 set (the fake has one flat register file,
    // so the read-modify-write sees the previous write's data).
    assert_eq!(
        r[reg::IREG_ADDR_15_8 as usize..reg::IREG_DATA as usize],
        [0xA2, 0x58]
    );
    assert_eq!(r[reg::IREG_DATA as usize] & 0x10, 0x10);
}

#[test]
fn wrong_id_and_bad_odr() {
    let mut regs = [0u8; 256];
    regs[reg::INT1_STATUS0 as usize] = reg::RESET_DONE;
    let mut imu = blocking::Icm45686::new(FakeSpi { regs });
    assert_eq!(
        imu.init(&mut NoDelay, &Config::default()),
        Err(Error::WrongId { found: 0 })
    );
    let low_power = Config {
        odr: Odr::Hz6_25,
        ..Config::default()
    };
    assert_eq!(
        imu.init(&mut NoDelay, &low_power),
        Err(Error::InvalidConfig)
    );
}
