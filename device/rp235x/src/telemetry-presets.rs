use defmt::{Format, Formatter, debug, info, write};

#[derive(Clone, Copy)]
pub(crate) struct SensorRates {
    pub imu: u32,
    pub imu2: u32,
    pub mag: u32,
    pub baro: u32,
    pub gps: u32,
}

#[derive(Clone, Copy)]
pub(crate) struct TelemetryLogPreset {
    pub rates: bool,
    pub imu: bool,
    pub imu2: bool,
    pub mag: bool,
    pub baro: bool,
    pub gps: bool,
    pub seq: bool,
}

pub(crate) const DEBUG_PRESET: TelemetryLogPreset = TelemetryLogPreset {
    rates: false,
    imu: false,
    imu2: false,
    mag: false,
    baro: false,
    gps: true,
    seq: true,
};

pub(crate) const RATE_PRESET: TelemetryLogPreset = TelemetryLogPreset {
    rates: true,
    imu: false,
    imu2: false,
    mag: false,
    baro: true,
    gps: true,
    seq: false,
};

pub(crate) fn log_debug_sample(s: &super::SensorsState, preset: TelemetryLogPreset) {
    debug!(
        "{}",
        TelemetryLine {
            sample: s,
            rates: None,
            preset,
        }
    );
}

pub(crate) fn log_rates_sample(
    s: &super::SensorsState,
    rates: SensorRates,
    preset: TelemetryLogPreset,
) {
    info!(
        "{}",
        TelemetryLine {
            sample: s,
            rates: Some(rates),
            preset,
        }
    );
}

struct TelemetryLine<'a> {
    sample: &'a super::SensorsState,
    rates: Option<SensorRates>,
    preset: TelemetryLogPreset,
}

impl Format for TelemetryLine<'_> {
    fn format(&self, fmt: Formatter) {
        let s = self.sample;
        let mut wrote_any = false;

        if self.preset.rates {
            if let Some(r) = self.rates {
                write!(
                    fmt,
                    "rates i:{} i2:{} m:{} b:{} g:{}",
                    r.imu, r.imu2, r.mag, r.baro, r.gps
                );
                wrote_any = true;
            }
        }

        if self.preset.imu {
            if wrote_any {
                write!(fmt, " | ");
            }
            write!(
                fmt,
                "imu a[{},{},{}] g[{},{},{}]",
                s.imu.accel[0],
                s.imu.accel[1],
                s.imu.accel[2],
                s.imu.gyro[0],
                s.imu.gyro[1],
                s.imu.gyro[2]
            );
            wrote_any = true;
        }

        if self.preset.imu2 {
            if wrote_any {
                write!(fmt, " | ");
            }
            write!(
                fmt,
                "imu2 a[{},{},{}] g[{},{},{}]",
                s.imu2.accel[0],
                s.imu2.accel[1],
                s.imu2.accel[2],
                s.imu2.gyro[0],
                s.imu2.gyro[1],
                s.imu2.gyro[2]
            );
            wrote_any = true;
        }

        if self.preset.mag {
            if wrote_any {
                write!(fmt, " | ");
            }
            write!(
                fmt,
                "mag [{},{},{}]",
                s.mag.xyz[0], s.mag.xyz[1], s.mag.xyz[2]
            );
            wrote_any = true;
        }

        if self.preset.baro {
            if wrote_any {
                write!(fmt, " | ");
            }
            write!(
                fmt,
                "baro t={}c p={}Pa",
                (s.baro.t_c_x100 as f32) / 100.0,
                s.baro.p_pa
            );
            wrote_any = true;
        }

        if self.preset.gps {
            if wrote_any {
                write!(fmt, " | ");
            }
            if let Some(fix) = s.gps.fix {
                write!(
                    fmt,
                    "gps fix {} sats {} lat {}.{:07} lon {}.{:07} alt {}m",
                    fix.fix_type,
                    fix.satellites,
                    fix.latitude / 10_000_000,
                    (fix.latitude % 10_000_000).abs(),
                    fix.longitude / 10_000_000,
                    (fix.longitude % 10_000_000).abs(),
                    fix.altitude / 1000
                );
            } else {
                write!(fmt, "gps no-fix");
            }
            wrote_any = true;
        }

        if self.preset.seq {
            if wrote_any {
                write!(fmt, " | ");
            }
            write!(
                fmt,
                "seq i:{} i2:{} m:{} b:{} g:{}",
                s.imu.seq, s.imu2.seq, s.mag.seq, s.baro.seq, s.gps.seq
            );
            wrote_any = true;
        }

        if !wrote_any {
            write!(fmt, "telemetry(no-fields)");
        }
    }
}
