//! Writes a small, complete sample log in the on-board format, to read back with `mavlogdump.py`.
//!
//! `cargo host-run`-style: `cargo run --target x86_64-unknown-linux-gnu -p fw-core --example
//! dflog_sample -- /path/to/sample.BIN`. Contents: the header sequence, 25 PARM, 3 MSG, then 10 s
//! of data at the design's rates (IMU 0 and 1 at 25 Hz each, BARO 50 Hz, MAG 100 Hz, GPS 1 Hz,
//! BAT 10 Hz, RCIN 10 Hz, DEV 7 rows at 1 Hz, DSF 1 Hz), with `TimeUS` increasing.

use fw_core::dflog::{
    Dev, Dsf, Gps, HEADER_SEQUENCE_LEN, encode_baro, encode_bat, encode_dev, encode_dsf,
    encode_gps, encode_imu, encode_mag, encode_msg, encode_parm, encode_rcin, gps_week_ms,
    write_header,
};
use std::io::Write;

/// Appends the record that `f` encodes into a scratch buffer.
fn push(file: &mut Vec<u8>, f: impl FnOnce(&mut [u8]) -> usize) -> usize {
    let mut b = [0u8; 128];
    let n = f(&mut b);
    assert!(n > 0, "record does not fit the scratch buffer");
    file.extend_from_slice(&b[..n]);
    n
}

/// Wave in [-1, 1] of period `period_us` (a triangle: no libm in the example either).
fn tri(t_us: u64, period_us: u64) -> f32 {
    let p = (t_us % period_us) as f32 / period_us as f32;
    if p < 0.5 {
        4.0 * p - 1.0
    } else {
        3.0 - 4.0 * p
    }
}

fn main() {
    let path = std::env::args()
        .nth(1)
        .expect("usage: dflog_sample <out.BIN>");
    let mut file: Vec<u8> = Vec::new();

    let mut head = [0u8; HEADER_SEQUENCE_LEN];
    assert_eq!(write_header(&mut head), HEADER_SEQUENCE_LEN);
    file.extend_from_slice(&head);
    let header_len = file.len();

    for i in 0..25u32 {
        let name = format!("PARAM_{i:02}");
        push(&mut file, |b| {
            encode_parm(
                b,
                1_000 + u64::from(i) * 10,
                name.as_bytes(),
                i as f32 * 0.5,
                0.0,
            )
        });
    }
    let start_burst = file.len();
    for (i, text) in ["MARV V2 boot", "IMU1 ready", "GPS SAM-M10Q ready"]
        .iter()
        .enumerate()
    {
        push(&mut file, |b| {
            encode_msg(b, 2_000 + i as u64 * 100, text.as_bytes())
        });
    }

    let (week, ms0) = gps_week_ms(2026, 11, 15, 12, 0, 0, 0);
    let mut counts = [0u32; 19];
    let data_start = file.len();
    for ms in 0..10_000u64 {
        let t = 3_000_000 + ms * 1_000;
        if ms % 40 == 0 {
            for inst in 0..2u8 {
                let w = tri(t, 2_000_000 + u64::from(inst) * 500_000);
                counts[3] += 1;
                push(&mut file, |b| {
                    encode_imu(
                        b,
                        t,
                        inst,
                        [0.1 * w, -0.05 * w, 0.02],
                        [0.2 * w, 0.1, -9.80665],
                        31.0 + 0.5 * f32::from(inst),
                    )
                });
            }
        }
        if ms % 20 == 0 {
            counts[4] += 1;
            push(&mut file, |b| {
                encode_baro(b, t, 0, 101_325.0 - 2.0 * tri(t, 5_000_000), 24.5)
            });
        }
        if ms % 10 == 0 {
            counts[5] += 1;
            let w = tri(t, 4_000_000);
            push(&mut file, |b| {
                encode_mag(
                    b,
                    t,
                    0,
                    [(250.0 + 20.0 * w) as i16, -80, (420.0 - 10.0 * w) as i16],
                )
            });
        }
        if ms % 100 == 0 {
            counts[7] += 1;
            push(&mut file, |b| {
                encode_bat(b, t, 0, 12.4 - 0.001 * ms as f32, 3.2)
            });
            counts[8] += 1;
            let mut ch = [1500u16; 14];
            ch[2] = 1000 + (ms / 10) as u16;
            push(&mut file, |b| encode_rcin(b, t, &ch));
        }
        if ms % 1_000 == 0 {
            let s = (ms / 1_000) as u32;
            counts[6] += 1;
            let g = Gps {
                inst: 0,
                status: 3,
                gms: ms0 + s * 1_000,
                gwk: week,
                nsats: 14,
                hdop_x100: 85,
                lat_e7: 411_234_567 + s as i32 * 10,
                lng_e7: -86_543_210,
                alt_cm: 65_230,
                speed_m_s: 0.1 * s as f32,
                course_deg: 90.0,
            };
            push(&mut file, |b| encode_gps(b, t, &g));
            for id in 0..7u8 {
                counts[9] += 1;
                let d = Dev {
                    id,
                    state: 1,
                    err: 0,
                    samples: s * 25 * u32::from(id + 1),
                    errors: 0,
                    timeouts: u32::from(id == 3 && s > 5),
                    recoveries: 0,
                    attempts: 0,
                    overflows: 0,
                };
                push(&mut file, |b| encode_dev(b, t, &d));
            }
            counts[10] += 1;
            let d = Dsf {
                dropped: 0,
                blocks: (file.len() / 512) as u32,
                bytes: file.len() as u32,
                free_min: 30_000,
                free_max: 32_700,
                free_avg: 32_000,
            };
            push(&mut file, |b| encode_dsf(b, t, &d));
        }
    }

    std::fs::File::create(&path)
        .and_then(|mut f| f.write_all(&file))
        .expect("write the sample");
    let data = file.len() - data_start;
    println!("file {} bytes", file.len());
    println!("header sequence {header_len} bytes; with 25 PARM (start burst) {start_burst} bytes");
    println!("with the 3 MSG {data_start} bytes");
    println!("10 s of data {data} bytes = {} bytes/s", data / 10);
    println!(
        "records per type [FMT,PARM,MSG,IMU,BARO,MAG,GPS,BAT,RCIN,DEV,DSF,NAV,SETP,DMD,MOT,ESC,RCS,FDS,GPA] = {counts:?}"
    );
}
