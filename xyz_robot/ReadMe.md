### Dependencies
Nightly build of Rust (https://rustup.rs/) - this is due to the use of the per-package-target Cargo feature.

``` bash
rustup install nightly
rustup override set nightly
rustup toolchain list

# ARM target with FPU support
rustup target add thumbv7em-none-eabihf

# ARM targets without FPU support
rustup target add thumbv7em-none-eabi
rustup target add thumbv6m-none-eabi

# probe-rs
# Windows
irm https://github.com/probe-rs/probe-rs/releases/latest/download/probe-rs-tools-installer.ps1 | iex

# Linux
curl --proto '=https' --tlsv1.2 -LsSf https://github.com/probe-rs/probe-rs/releases/latest/download/probe-rs-tools-installer.sh | sh
# or if the above fails:
cargo install probe-rs-tools

# cargo-make
cargo install --force cargo-make

```
### Building
The parser module must be built first.
Any problems, add the "--verbose" flag to the commands for more information.
```cargo
# with cargo make (recommended)
cargo make fw
```
or, to build the parser and firmware separately:
```cargo
cd xyz_parser
cargo build --color=always --features=embedded --target thumbv7em-none-eabihf
cd ../x_firmware
cargo build --color=always --features=embedded --target thumbv7em-none-eabihf
```

### Flashing (via STLink SWD)
```cargo
# uses cargo flash to write a firmware as "probe-rs run" does not work correctly (TODO: find the command parameters to fix it). For now, separate commands work okay.
cargo flash
#or
cargo make flash_x
```

### Flashing with ST Cube Programmer
Writing the firmware with the cube programmer does not work unless the filename ends with ".elf". This is an annoying bug that I haven't written scripts to avoid, so remember to add one in the meantime if you are writing it directly. To generate a ".elf" in the build requires a custom target creating by the look of it.

### Running (via STLink SWD)
```cargo
# run the built firmware with RTT output
# uses "runner" setting from .cargo/config.toml to attach
cargo run
```

### Sending commands (via USB)
```
cargo make attach
# in another term...
tools/uart_cli/target/debug/uart-cli /dev/ttyUSB1
```

### Clean builds
```cargo
cargo clean
```

### Documentation
```cargo
# generate documentation
cargo doc

# open the generated documentation in the browser
cargo doc --open

### Unit tests
Note that the clean is required only if the previous build was not a unit test.
```cargo
# parser unit tests (on Windows)
cargo clean
cargo test -p xyz_parser --color=always --no-default-features --features=std --target=x86_64-pc-windows-msvc

# parser unit tests (on Linux)
cargo clean
cargo test -p xyz_parser --color=always --no-default-features --features=std --target=x86_64-unknown-linux-gnu

# TODO: fix this test build
cargo clean
cargo build -p x_firmware --color=always --no-default-features --features=xyz_std --target=x86_64-pc-windows-msvc
```
