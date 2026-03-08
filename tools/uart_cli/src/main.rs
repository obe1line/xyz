extern crate core;

use std::io;
use std::fs::File;
use std::io::{BufRead, Read, Write};
use std::sync::{mpsc, Arc};
use std::thread;
use std::time::Duration;
use env_logger::Env;
use serialport5::{SerialPort, SerialPortBuilder};
use heapless::Vec;
use log::{info, error};

mod xyz_parser;
use crate::xyz_parser::{ErrorCode, XYZMessage, XYZMessageParser};

// // TODO: fetch from a file
// const COMMANDS: [&str; 1] = [
//     // "FWD",
//     // "FWD",
//     // "FWD",
//     // "FWD",
//     // "BCK",
//     // "BCK",
//     // "BCK",
//     // "BCK",
//     // "FWD",
//     // "BCK",
//     "RV",
//     // "18PI",
//     // "18XA 300",
//     // "18SF1 1000 500 500",
// ]; // Example command, can be extended

pub fn read_serial_loop(read_port: Arc<SerialPort>) {
    info!("Starting serial loop");
    let mut raw_buf: [u8;32] = [0x00u8; 32]; // Buffer for raw data
    let mut buf: Vec<u8, 32> = Vec::new(); // Buffer to pass to parser
    let mut need_more_data = true;
    let mut parser = XYZMessageParser::new();
    let mut port = read_port.as_ref();
    let mut expect_ack = false; // Flag to expect an ACK response

    loop {
        // Read data into the buffer if more is required
        if need_more_data {
            // info!("buf is empty, reading data...buf.len() = {}", buf.len());
            let data_read_count = port.read(&mut raw_buf).unwrap();
            buf.extend_from_slice(&raw_buf[..data_read_count]).ok();
            info!("data_read_count: {}, buf.len(): {}", data_read_count, buf.len());
        }

        let vl = buf.len();
        if vl > 0 {
            for b in buf.iter() {
                info!("  buf byte: {:02X}", b);
            }
            // info!("Adding {} bytes to parser", vl);
            let byte_count = parser.add_data(&buf, vl);
            // info!("byte_count = {}", byte_count);
            for _ in 0..byte_count {
                // remove processed bytes from the buffer
                buf.remove(0);
            }
            // info!("After truncate, buf.len() = {}", buf.len());
        }

        // parse the buffer to see if there is a complete message
        let mut no_error = true;
        while no_error {
            let msg = parser.parse();
            no_error = false; // Reset no_error for the next iteration

            match msg.error_code {
                ErrorCode::NoError => {
                    let msg_string = String::from_utf8_lossy(&msg.message);
                    info!("Message parsed successfully:");
                    info!("  Control: {:02X}", msg.control);
                    info!("  Arm Address: {}", msg.arm_adr as char);
                    info!("  Device Address: {}", msg.device_adr as char);
                    info!("  Message: {:?}", msg.message);
                    info!("  Message as str: {:?}", msg_string);
                    info!("  VRC: {:02X}", msg.vrc);
                    no_error = true;

                    if expect_ack {
                        if msg.control == XYZMessage::ACK {
                            info!("Expecting ACK response, skipping further processing");
                            expect_ack = false; // Reset the flag after processing
                            continue; // Skip further processing for this message
                        }
                        else {
                            // TODO: handle unexpected message type
                            info!("Expected an ACK but got another message type - skipping for now");
                            expect_ack = false;
                            parser.clear_buffer();
                            continue;
                        }
                    }

                    if msg.control != XYZMessage::ACK {
                        info!("Processing non-ACK message (answer?), sending ACK response");
                        // send an ACK response if it is a response message
                        let mut ack = XYZMessage::new_ack();
                        ack.arm_adr = msg.arm_adr;
                        ack.device_adr = msg.device_adr;
                        port.write_all(ack.encode().as_mut_slice()).unwrap();
                    }

                    continue; // Process next message
                },
                ErrorCode::PartialMessageReceived => {
                    info!("Partial message received, waiting for more data");
                    need_more_data = true;
                    continue; // Continue to wait for more data
                },
                ErrorCode::InvalidStartByte => {
                    error!("Invalid start byte received");
                    parser.skip_to_next_start_byte();
                    // TODO: send error response packet
                },
                ErrorCode::VRCMismatch => {
                    error!("VRC mismatch detected");
                    parser.clear_buffer();
                    // TODO: send error response packet
                },
                ErrorCode::MessageBufferOverflow => {
                    error!("Message buffer overflow");
                    parser.clear_buffer();
                    // TODO: send error response packet
                }
            }

            // // Send received data to the transmit task via the channel
            // let mut data: Vec<u8, 255> = Vec::new();
            // if no_error {
            //     data.push(0x40u8).ok();
            //     data.push(0x40u8).ok();
            //     data.push(0x40u8).ok();
            // }
            // else {
            //     data.push(0xFFu8).ok(); // Error code
            //     data.push(0x40u8).ok();	// Answer Block
            // }
            // let response = XYZMessage::new(ErrorCode::NoError, msg.control, msg.arm_adr,
            //                                msg.device_adr, data, msg.vrc);
            // let mut port = port.lock().unwrap();
            // port.write_all(response.encode().as_mut_slice()).unwrap();
            // UART2_CHANNEL.send(response).await;
        }
        // // Read data from the serial port
        // match port.read(&mut buffer) {
        //     Ok(0) => return,
        //     Ok(n) => {
        //         println!("buffer: {:?}", buffer[..n].to_vec());
        //         // Process the received data
        //         if buffer[0] == 0x02 && buffer[1] == 0x40 {
        //             // acknowledgment message
        //             println!("Got ACK");
        //         } else {
        //             // send ACK
        //             let ack = vec![0x02u8, 0x40u8, 0x00u8, 0x01u8, 0x03u8, 0x40u8];
        //             if let Err(e) = port.write_all(&ack) {
        //                 println!("Failed to send ACK: {:?}", e);
        //             } else {
        //                 println!("Sent ACK response: {:?}", ack);
        //             }
        // 
        //             // Handle message and send a response
        //             // TODO: Implement message handling logic
        //             println!("Received message: {:?}", &buffer[..n]);
        //             // send a response
        //             let ack = vec![0x02u8, 0x66u8, 0x00u8, 0x01u8, 0x03u8, 0x40u8];
        //             if let Err(e) = port.write_all(&ack) {
        //                 println!("Failed to send response: {:?}", e);
        //             } else {
        //                 println!("Sent response: {:?}", ack);
        //             }
        //         }
        //     }
        //     Err(ref e) if e.kind() == std::io::ErrorKind::TimedOut => continue,
        //     Err(e) => {
        //         println!("Failed to read: {}", e);
        //         break;
        //     }
        // }
    }
}

fn main() {
    env_logger::Builder::from_env(Env::default().default_filter_or("debug")).init();

    // check the available serial ports
    let ports = serialport5::available_ports().expect("No ports found!");
    for p in ports {
        info!("Detected port: {}", p.port_name);
    }

    // get the serial port path from command line arguments
    let args: Vec<String, 5> = std::env::args().collect();
    if args.len() < 2 {
        info!("Usage: uart_cli <serial port path> [<command_file>]");
        return;
    }
    let port_name = &args[1];
    let mut command_file = ""; // default to no command file
    // optional command file
    if args.len() > 2 {
        info!("Using command file: {}", &args[2]);
        command_file = &args[2];
    }

    info!("Using serial port: {}", port_name);

    let baud_rate = 9600;

    // Open the serial port
    let port = SerialPortBuilder::new()
        .baud_rate(baud_rate)
        .open(port_name);
    let port = match port {
        Ok(p) => p,
        Err(e) => {
            error!("Failed to open serial port {}: {}", port_name, e);
            return;
        }
    };

    let shared_port = Arc::new(port); // Wrap in Arc for shared access
    let writer_port = Arc::clone(&shared_port);
    let reader_port = Arc::clone(&shared_port);

    thread::spawn(move || {
        read_serial_loop(reader_port);
    });

    // send data to the serial port

    //let mut serial_ptr = serial_stream.try_clone().expect("Failed to clone serial port");

    // let serial_port = SerialPort::open(port_path, baud_rate)
    //     .expect("Failed to open serial port");;
    // let read_port = serial_port.try_clone().unwrap(); // Clone for reading

    // let shared_port = Arc::new(serial_port); // Wrap in Arc for shared access
    // let writer_port = Arc::clone(&shared_port);
    // let reader_port = Arc::clone(&shared_port);

    // Channel for sending data to the writer task
    let (sender, receiver) = mpsc::channel::<Vec<u8, 32>>();

    // Spawn a message handler task
    thread::spawn(move || {
        write_messages_loop(writer_port, receiver);
    });

    fn write_messages_loop(write_port: Arc<SerialPort>, receiver: mpsc::Receiver<Vec<u8, 32>>) {
        let mut data = [0x00u8; 255];
        let mut port = write_port.as_ref();
        loop {
            match receiver.recv() {
                Ok(msg) => {
                    let count = msg.len();
                    data[..count].copy_from_slice(&msg);
                    // info!("Sending {} bytes: {:?}", msg.len(), &msg);
                    // info!("Sending {} bytes: {:?}", msg.len(), &msg);
                    let mut hex_string = String::new();
                    hex_string.push_str("Sending: ");
                    for byte in msg.iter() {
                        hex_string.push_str(&format!("{:02x} ", byte));
                    }
                    hex_string.push('\n');
                    hex_string.shrink_to_fit();
                    info!("{}", hex_string);

                    // // if false {
                    //     // slow...
                    //     for byte in data[..count].iter() {
                    //         //info!("Sending byte: {:02X}", byte);
                    //         let byte_buf = [*byte];
                    //         if let Err(e) = port.write_all(&byte_buf) {
                    //             error!("Failed to send byte: {:?}", e);
                    //             return;
                    //         }
                    //         // if no delay, QP crashes
                    //         thread::sleep(Duration::from_millis(100)); // QP gets all data, but slow
                    //         // thread::sleep(Duration::from_millis(10)); // QP gets all data, but slow
                    //     }
                    // // }
                    // else {
                    // too fast for QP and single byte receive...
                    match port.write_all(&data[..count]) {
                        Ok(_) => {
                            // for byte in data[..count].iter() {
                            //     info!("Sent byte: {:02X}", byte);
                            // }
                            // info!("Sent bytes: {:?}", &data[..count]);

                        }
                        Err(e) => {
                            error!("Failed to write to serial port: {}", e);
                        }
                    }
                    // }
                    //thread::sleep(Duration::from_millis(1000));
                }
                Err(e) => {
                    error!("Channel receive error: {}", e);
                    break;
                }
            }
        }
    }

    let mut commands = std::vec::Vec::new();
    // load in commands from a file if they exist, defaults to RV command
    if !command_file.is_empty() {
        let file = File::open(command_file).expect("Failed to open command file");
        let reader = io::BufReader::new(file);

        // Iterate over the lines in the file
        for line_result in reader.lines() {
            if line_result.is_err() {
                error!("Error reading line from command file: {:?}", line_result.err());
                continue; // Skip lines that can't be read
            }
            // Handle potential errors for each line
            let line_raw = line_result.unwrap();
            if line_raw.is_empty() || line_raw.as_str().starts_with('#') {
                continue; // Skip empty lines and comments
            }
            commands.push(line_raw);
        }
        run_command_loop(commands, sender);
    } else {
        // default commands
        commands.push("RV".to_string());
        run_command_loop(commands, sender);
    }
    pub fn run_command_loop(commands: std::vec::Vec<String>, sender: mpsc::Sender<Vec<u8, 32>>) {
        for command in commands.into_iter() {
            let control = 0x45u8;
            let arm_adr: u8 = '1' as u8; // Example arm address, using count to vary it
            let device_adr: u8 = '2' as u8; // Example device address
            let msg_block: &[u8] = command.as_bytes();

            let mut data: Vec<u8, 32> = Vec::new();
            data.push(0x02u8).ok(); // STX (Start of Text)
            data.push(control).ok();
            data.push(arm_adr).ok();
            data.push(device_adr).ok();
            data.extend_from_slice(msg_block).ok();
            data.push(0x03u8).ok(); // ETX (End of Text)

            // calculate VRC (Variable Redundancy Check)
            let mut vrc = 0u8;
            for byte in data.as_slice() {
                vrc ^= byte; // Calculate VRC by XORing all bytes
            }

            // if vrc & 0x01 > 0 {
            //     let corrupt = vrc | 0x05; // Corrupt the VRC for testing
            //     info!("Corrupting message data: {:?} to {:?}", &vrc, &corrupt);
            //     vrc = corrupt;
            // }
            data.push(vrc).ok();

            // add the command to the channel for sending
            sender.send(data).ok();

            // thread::sleep(Duration::from_millis(1000)); // QP gets all data, but slow

            // // Slow it down for testing
            // // send a byte at a time, waiting a bit between bytes
            // let count = data.len();
            // info!("Sending command: '{}', {} bytes: {:?}", command, count, data);
            // for byte in data.iter() {
            //     let byte_buf = [*byte];
            //     if let Err(e) = port.write_all(&byte_buf) {
            //         error!("Failed to send byte: {:?}", e);
            //         return;
            //     }
            //     // if no delay, QP crashes
            //     // thread::sleep(Duration::from_millis(1));     // QP gets 1st and 4th packets only
            //     thread::sleep(Duration::from_millis(10)); // QP gets all data, but slow
            // }

            // if let Err(e) = port.write_all(data.as_mut_slice()) {
            //     eprintln!("Failed to send data: {:?}", e);
            //     return;
            // }

            // info!("Reading ACK");
            //
            // // read the ACK
            // let mut ack_buffer = [0; 512];
            // match port.read(&mut ack_buffer) {
            //     Ok(0) => {
            //         info!("No data read from serial port.");
            //         return;
            //     }
            //     Ok(n) => {
            //         info!("Received ACK: {:?}", &ack_buffer[..n]);
            //     }
            //     Err(ref e) if e.kind() == std::io::ErrorKind::TimedOut => {
            //         error!("Read timed out, no data received.");
            //         continue;
            //     }
            //     Err(e) => {
            //         error!("Failed to read ACK: {}", e);
            //         return;
            //     }
            // }

            // slow down the loop between commands - as not waiting for motor busy yet
            // thread::sleep(Duration::from_millis(1000));
            thread::sleep(Duration::from_millis(500));
        }

        // wait for flush etc
        thread::sleep(Duration::from_secs(2));
    }
}
