use std::env;
use std::fs::File;
use std::io::Write;
use std::path::PathBuf;

const DEFAULT_LORA_FREQUENCY_HZ: u32 = 902_080_000;
const SRAD_LOWER_HZ: u32 = 902_000_000;
const SRAD_UPPER_HZ: u32 = 909_000_000;
const MODE_C_BANDWIDTH_HZ: u32 = 62_500;

fn main() {
    let out = PathBuf::from(env::var_os("OUT_DIR").unwrap());

    File::create(out.join("memory.x"))
        .unwrap()
        .write_all(include_bytes!("memory.x"))
        .unwrap();

    let frequency_hz = lora_frequency_hz();
    let callsign = amateur_callsign();
    let mut generated = File::create(out.join("radio_build_config.rs")).unwrap();
    writeln!(
        generated,
        "pub const LORA_FREQUENCY_HZ: u32 = {frequency_hz};"
    )
    .unwrap();
    writeln!(
        generated,
        "pub const AMATEUR_CALLSIGN: &str = {:?};",
        callsign
    )
    .unwrap();

    println!("cargo:rustc-link-search={}", out.display());
    println!("cargo:rerun-if-env-changed=MARV_LORA_FREQUENCY_HZ");
    println!("cargo:rerun-if-env-changed=MARV_AMATEUR_CALLSIGN");
    println!("cargo:rerun-if-changed=memory.x");
    println!("cargo:rustc-link-arg-bins=--nmagic");
    println!("cargo:rustc-link-arg-bins=-Tlink.x");
    println!("cargo:rustc-link-arg-bins=-Tdefmt.x");
}

fn lora_frequency_hz() -> u32 {
    let Some(raw) = env::var("MARV_LORA_FREQUENCY_HZ").ok() else {
        println!(
            "cargo:warning=MARV_LORA_FREQUENCY_HZ not set; using unassigned Mode C default {DEFAULT_LORA_FREQUENCY_HZ} Hz for bench builds"
        );
        return DEFAULT_LORA_FREQUENCY_HZ;
    };

    let frequency_hz = raw.parse::<u32>().unwrap_or_else(|_| {
        panic!("MARV_LORA_FREQUENCY_HZ must be an integer frequency in Hz, got {raw:?}")
    });
    validate_srad_frequency(frequency_hz);
    frequency_hz
}

fn validate_srad_frequency(frequency_hz: u32) {
    let half_bandwidth_hz = MODE_C_BANDWIDTH_HZ / 2;
    let lower_edge_hz = frequency_hz
        .checked_sub(half_bandwidth_hz)
        .expect("MARV_LORA_FREQUENCY_HZ is too low");
    let upper_edge_hz = frequency_hz
        .checked_add(half_bandwidth_hz)
        .expect("MARV_LORA_FREQUENCY_HZ is too high");

    if lower_edge_hz < SRAD_LOWER_HZ || upper_edge_hz > SRAD_UPPER_HZ {
        panic!(
            "MARV_LORA_FREQUENCY_HZ={frequency_hz} with {MODE_C_BANDWIDTH_HZ} Hz bandwidth does not fit inside the 902.0-909.0 MHz IREC 33 cm SRAD range"
        );
    }
}

fn amateur_callsign() -> String {
    let Some(raw) = env::var("MARV_AMATEUR_CALLSIGN").ok() else {
        println!(
            "cargo:warning=MARV_AMATEUR_CALLSIGN not set; station ID frames will be disabled until a callsign is supplied"
        );
        return String::new();
    };

    let callsign = raw.trim().to_ascii_uppercase();
    if callsign.is_empty() {
        println!(
            "cargo:warning=MARV_AMATEUR_CALLSIGN is empty; station ID frames will be disabled"
        );
        return callsign;
    }
    if callsign.len() > 16 {
        panic!("MARV_AMATEUR_CALLSIGN must be 16 ASCII characters or fewer");
    }
    if !callsign
        .bytes()
        .all(|byte| byte.is_ascii_alphanumeric() || byte == b'/')
    {
        panic!("MARV_AMATEUR_CALLSIGN may only contain ASCII letters, digits, and '/'");
    }

    callsign
}
