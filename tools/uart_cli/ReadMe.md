### UART_CLI - Command Line Tecan Utility

## Building
Lots of commented code, so there will be warnings.  
Ignore them for now.
```cargo
cargo build
```

## Running
Send commands to COM4 - hardcoded in the first version.  
Hardcoded to move 100 steps left, delay for 2sec, move 100 steps right and repeat 4 times.  
```cargo
cargo run COM4
```