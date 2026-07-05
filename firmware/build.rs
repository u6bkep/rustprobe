use std::env;
use std::fs::File;
use std::io::Write;
use std::path::PathBuf;

fn main() {
    let rp2040 = env::var("CARGO_FEATURE_RP2040").is_ok();

    let out = &PathBuf::from(env::var_os("OUT_DIR").unwrap());
    let memory_x: &[u8] = if rp2040 {
        include_bytes!("memory-rp2040.x")
    } else {
        include_bytes!("memory-rp2350.x")
    };
    File::create(out.join("memory.x")).unwrap().write_all(memory_x).unwrap();
    println!("cargo:rustc-link-search={}", out.display());
    println!("cargo:rerun-if-changed=memory-rp2040.x");
    println!("cargo:rerun-if-changed=memory-rp2350.x");

    println!("cargo:rustc-link-arg-bins=--nmagic");
    println!("cargo:rustc-link-arg-bins=-Tlink.x");
    if rp2040 {
        // Places the .boot2 second-stage bootloader (provided by embassy-rp).
        println!("cargo:rustc-link-arg-bins=-Tlink-rp.x");
    }
    println!("cargo:rustc-link-arg-bins=-Tdefmt.x");
}
