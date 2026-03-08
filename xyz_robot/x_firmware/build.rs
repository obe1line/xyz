
fn main() {
    println!("cargo:rerun-if-changed=build.rs");
    // println!("cargo:rustc-link-lib=static=powerSTEP01");
    println!("cargo:rustc-link-arg-bins=-Map=target/thumbv7em-none-eabihf/x-firmware.map");
}