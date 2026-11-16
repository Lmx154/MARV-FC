//! Puts the board's `memory.x` (next to this file) on the linker search path and passes the
//! linker scripts every on-target binary needs.

use std::env;
use std::fs;
use std::path::PathBuf;

fn main() {
    let out = PathBuf::from(env::var_os("OUT_DIR").unwrap());
    fs::write(out.join("memory.x"), include_bytes!("memory.x")).unwrap();
    println!("cargo:rustc-link-search={}", out.display());
    println!("cargo:rerun-if-changed=memory.x");

    println!("cargo:rustc-link-arg-bins=--nmagic");
    println!("cargo:rustc-link-arg-bins=-Tlink.x");
    println!("cargo:rustc-link-arg-bins=-Tdefmt.x");
    // The flight core's code is copied to SRAM before anything runs (src/ram.rs): embassy-rp already
    // defines `__pre_init`, so cortex-m-rt's call goes to `__wrap___pre_init`, which copies and then
    // continues in embassy-rp's (`__real___pre_init`).
    println!("cargo:rustc-link-arg-bins=--wrap=__pre_init");
}
