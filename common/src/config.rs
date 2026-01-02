//! Simple key/value config loader for SD-backed parameters.
//! Designed to be extendable as more fields are added.

use core::fmt::Write;
use heapless::String;

#[derive(Copy, Clone, Debug, defmt::Format)]
pub struct Config {
    pub led_color: [u8; 3],
    pub imu_hz: u16,
    pub mag_hz: u16,
    pub baro_hz: u16,
    pub gps_hz: u16,
}

impl Config {
    pub const fn default() -> Self {
        Self {
            led_color: [0, 16, 0],
            imu_hz: 1000,
            mag_hz: 25,
            baro_hz: 250,
            gps_hz: 20,
        }
    }

    /// Parse a config text blob (key=value per line). Unknown keys are ignored.
    pub fn from_bytes(raw: &[u8]) -> Self {
        let mut cfg = Self::default();
        for line in raw.split(|&b| b == b'\n') {
            let line = trim(line);
            if line.is_empty() || line.starts_with(b"#") {
                continue;
            }
            if let Some((k, v)) = split_kv(line) {
                apply_kv(&mut cfg, k, v);
            }
        }
        cfg
    }

    /// Render the config to a static string buffer for writing to disk.
    pub fn to_bytes(&self) -> String<128> {
        let mut s: String<128> = String::new();
        let _ = write!(
            s,
            "led_color={},{},{}\nimu_hz={}\nmag_hz={}\nbaro_hz={}\ngps_hz={}\n",
            self.led_color[0],
            self.led_color[1],
            self.led_color[2],
            self.imu_hz,
            self.mag_hz,
            self.baro_hz,
            self.gps_hz
        );
        s
    }
}

fn apply_kv(cfg: &mut Config, key: &[u8], val: &[u8]) {
    match key {
        b"led_color" => {
            if let Some(rgb) = parse_rgb(val) {
                cfg.led_color = rgb;
            }
        }
        b"imu_hz" => {
            if let Some(v) = parse_u16(val) {
                cfg.imu_hz = v.max(1);
            }
        }
        b"mag_hz" => {
            if let Some(v) = parse_u16(val) {
                cfg.mag_hz = v.max(1);
            }
        }
        b"baro_hz" => {
            if let Some(v) = parse_u16(val) {
                cfg.baro_hz = v.max(1);
            }
        }
        b"gps_hz" => {
            if let Some(v) = parse_u16(val) {
                cfg.gps_hz = v.max(1);
            }
        }
        _ => {}
    }
}

fn parse_u16(raw: &[u8]) -> Option<u16> {
    let mut val: u32 = 0;
    let mut seen = false;
    for &b in raw {
        if b.is_ascii_digit() {
            seen = true;
            val = val.saturating_mul(10).saturating_add((b - b'0') as u32);
        } else {
            break;
        }
    }
    if seen { Some(val as u16) } else { None }
}

fn parse_rgb(raw: &[u8]) -> Option<[u8; 3]> {
    let mut out = [0u8; 3];
    let mut idx = 0usize;
    let mut acc: u16 = 0;
    let mut seen_digit = false;
    for &b in raw {
        if b == b',' {
            if idx >= 3 || !seen_digit {
                return None;
            }
            out[idx] = acc.min(255) as u8;
            idx += 1;
            acc = 0;
            seen_digit = false;
        } else if b.is_ascii_digit() {
            seen_digit = true;
            acc = acc.saturating_mul(10).saturating_add((b - b'0') as u16);
        } else {
            break;
        }
    }
    if idx == 2 && seen_digit {
        out[2] = acc.min(255) as u8;
        Some(out)
    } else {
        None
    }
}

fn split_kv(line: &[u8]) -> Option<(&[u8], &[u8])> {
    let mut iter = line.splitn(2, |&b| b == b'=');
    let k = iter.next()?;
    let v = iter.next().unwrap_or(&[]);
    Some((trim(k), trim(v)))
}

fn trim(mut s: &[u8]) -> &[u8] {
    while let Some((f, rest)) = s.split_first() {
        if !f.is_ascii_whitespace() {
            break;
        }
        s = rest;
    }
    while let Some((l, rest)) = s.split_last() {
        if !l.is_ascii_whitespace() {
            break;
        }
        s = rest;
    }
    s
}
