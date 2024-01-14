#![allow(unused_variables)]
#![allow(dead_code)]
#![allow(unused_imports)]
use std::env;
use std::path::PathBuf;
fn main() {
    #[cfg(not(feature = "std"))]
    {
        println!("cargo:rustc-link-arg-bins=--nmagic");
        println!("cargo:rustc-link-arg-bins=-Tlink.x");
    }
    #[cfg(feature = "defmt")]
    {
        println!("cargo:rustc-link-arg-bins=-Tdefmt.x");
    }

    return;
}
