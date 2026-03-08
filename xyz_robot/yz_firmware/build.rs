// For build script info see: https://doc.rust-lang.org/cargo/reference/build-scripts.html
fn main() {
    if cfg!(feature = "embedded") {
        println!("cargo:rustc-link-arg-bins=--nmagic");
        println!("cargo:rustc-link-arg-bins=-Tlink.x");
        println!("cargo:rustc-link-arg-bins=-Tdefmt.x");
        println!("cargo:rustc-link-arg-bins=-Map=target/thumbv7em-none-eabihf/y-firmware.map");
    }
}
