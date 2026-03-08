Pros, cons, and notes on using Rust for embedded systems and other applications.
# Rust Notes
## Pros
TODO
  
## Cons
Lifetimes and borrowing can be complex and difficult to understand for beginners.  

TODO

============================  
Dumping ground below here

## Why Rust rather than C/C++?
* Memory safety without a garbage collector.
* Modern language features (pattern matching, algebraic data types, etc.).
* Strong type system with type inference.
* Excellent tooling (Cargo, rustfmt, clippy).
* Growing ecosystem and community support.
* Growing adoption in industry (e.g., Mozilla, Dropbox, Cloudflare).
* Interoperability with C/C++ code.
* Performance comparable to C/C++.
* Built-in support for concurrency and parallelism.
* Focus on developer productivity and code maintainability.
* Strong emphasis on documentation and testing.
* Active development and regular updates.
* Good error messages and compiler diagnostics.
* Ownership model that prevents data errors at compile time.
* Cross-platform support.
* Integration with modern development workflows (CI/CD, containerization).
* Rich standard library.
* Support for functional programming paradigms.
* First-class support for WebAssembly.
* Strong focus on safety and correctness.
* Excellent support for embedded systems.
* Built-in package manager (Cargo) for easy dependency management.
* Ability to write low-level code when needed (unsafe Rust).
* Good interoperability with other languages (FFI).

## Documentation Generation
* Rust has built-in support for documentation generation using `rustdoc`.
* Documentation can include Markdown formatting, code examples, and links to other items.
* Code examples in documentation comments can be tested using "doc-tests".
* Documentation can be generated using the command `cargo doc`.
* The generated documentation can be viewed in a web browser using 'cargo doc --open'.

# Commercial Use of Rust
https://onevariable.com/blog/embedded-rust-production/
Akiles is the maintainer of the Embassy Rust async framework which they use in their firmware devices.

AI Overview
Commercial users of the Embassy Rust async framework include Samsung (SmartThings), Akiles (office/hotel key devices), Kelvin (smart radiator covers), Sonos (voice assistant), and Elektron (audio products), among others, who use it for embedded firmware and applications requiring low power and multitasking capabilities. These companies leverage Embassy's async nature, hardware abstraction layers (HALs), and features like low-power modes and networking to build safe, energy-efficient, and modern embedded systems for various consumer and industrial products.  
Examples of Commercial Users and Products
Samsung: Uses Rust, including the Embassy framework, for firmware in its SmartThings platform.
Akiles: Employs Rust for the firmware in their office and hotel key devices.
Kelvin: Integrates Embassy Rust for the firmware of their smart radiator covers.
Sonos: Developed its on-device Sonos Voice Control entirely in Rust using Embassy.
Elektron: Has advertised for Rust Audio Developers, indicating significant use of Rust in their audio product codebases, likely on embedded Linux systems.
Why Embassy is Attractive for Commercial Use
Async Rust: The framework uses Rust's async capabilities to create energy-efficient, non-blocking code, crucial for embedded systems.
Hardware Abstraction: Embassy provides hardware abstraction layers (HALs) that simplify interaction with diverse hardware, allowing for more portable and idiomatic Rust APIs.
Low-Power Operation: The framework is designed to be low-power-ready, automatically putting the core to sleep when idle, which is essential for battery-powered devices.
Networking Support: Embassy includes extensive support for networking protocols like Ethernet, IP, TCP, UDP, and DHCP.
Community and Ecosystem: A strong community contributes to the ecosystem, providing numerous crates and support for popular microcontrollers, notes embassy.dev.
