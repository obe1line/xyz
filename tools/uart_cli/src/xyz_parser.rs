// use defmt::{error, info};
// compatibility with the embedded code
macro_rules! info {
    // This rule matches any arguments passed to info! and forwards them to println!
    ($($arg:tt)*) => {
        println!($($arg)*);
    };
}
macro_rules! error {
    // This rule matches any arguments passed to error! and forwards them to println!
    ($($arg:tt)*) => {
        println!($($arg)*);
    };
}

use heapless::Vec;


#[derive(Debug, Default)]
pub enum ErrorCode {
    #[default]
    NoError,
    PartialMessageReceived,
    InvalidStartByte,
    VRCMismatch,
    MessageBufferOverflow,
}

#[derive(Debug, Default)]
/// Represents a message in the XYZ protocol.
pub struct XYZMessage {
    pub error_code: ErrorCode, // Error code if any
    pub control: u8, // Control byte
    pub arm_adr: u8,     // Arm byte
    pub device_adr: u8,  // Device byte
    pub message: Vec::<u8, 255>, // Message byte (not used in this example)
    pub vrc: u8, // VRC byte
}

impl XYZMessage {
    pub const ACK: u8 = 0x40u8; // Acknowledge byte
    
    pub fn new_ack() -> XYZMessage {
        let msg = XYZMessage {
            error_code: ErrorCode::NoError,
            control: Self::ACK,
            arm_adr: 0x00, // Example arm address
            device_adr: 0x01, // Example device address
            message: Vec::new(), // Empty message for ACK
            vrc: 0, // Placeholder for VRC, will be calculated later
        };
        msg
    }
}

impl XYZMessage {
    pub fn encode(&self) -> Vec<u8, 255> {
        let mut encoded: Vec<u8, 255> = Vec::new();
        encoded.push(0x02u8).ok(); // STX (Start of Text)
        encoded.push(self.control).ok();
        encoded.push(self.arm_adr).ok();
        encoded.push(self.device_adr).ok();
        encoded.extend_from_slice(&self.message).ok(); // Append message bytes
        encoded.push(0x03u8).ok(); // ETX (End of Text)
        // Calculate VRC (Variable Redundancy Check)
        let mut vrc_calc = 0u8;
        for &byte in encoded.iter() {
            vrc_calc ^= byte; // Calculate VRC by XORing all bytes
        }
        encoded.push(vrc_calc).ok(); // Append the VRC byte
        encoded
    }
}

impl XYZMessage {
    pub fn new(error_code: ErrorCode, control: u8, arm_adr: u8, device_adr: u8, 
               message: Vec<u8, 255>, vrc: u8) -> XYZMessage {
        XYZMessage {
            error_code,
            control,
            arm_adr,
            device_adr,
            message,
            vrc
        }
    }
}

impl XYZMessage {
    fn new_error(err: ErrorCode) -> XYZMessage {
        let msg = XYZMessage {
            error_code: err,
            control: 0, // Placeholder, will be set later
            arm_adr: 0, // Placeholder, will be set later
            device_adr: 0, // Placeholder, will be set later
            message: Vec::new(), // Placeholder, will be set later
            vrc: 0, // Placeholder, will be set later
        };
        msg
    }
}

pub struct XYZMessageParser{
    buffer: Vec::<u8, 255>
}

impl XYZMessageParser {
    const STX: u8 = 0x02; // Start of Text
    const ETX: u8 = 0x03; // End of Text
    const MIN_MSG_SIZE: usize = 6; // STX, Control, Arm, Device, Message (missing), ETX, VRC

    pub fn new() -> Self {
        XYZMessageParser {
            buffer: Vec::new(), // Initialize with empty buffer
        }
    }

    pub fn clear_buffer(&mut self) {
        // Clear the internal buffer
        self.buffer.clear();
    }

    /// Adds data to the internal buffer.
    /// If the data exceeds the buffer capacity, it will only add up to the maximum allowed size.
    /// Returns the number of bytes added to the buffer.
    pub fn add_data(&mut self, data: &[u8], byte_count: usize) -> usize {
        let copy_len = byte_count.min(255 - self.buffer.len());
        // Append the received data to the internal buffer
        if copy_len > 0 {
            self.buffer.extend_from_slice(&data[..copy_len]).ok();
        }
        copy_len
    }

    pub fn parse(&mut self) -> XYZMessage {
        let mut msg: XYZMessage = Default::default();

        // Not enough data for a complete message?
        info!("Buffer length: {}", self.buffer.len());
        if self.buffer.len() < Self::MIN_MSG_SIZE {
            return XYZMessage::new_error(ErrorCode::PartialMessageReceived);
        }

        // Check if the buffer contains a complete message
        if self.buffer[0] != Self::STX {
            return XYZMessage::new_error(ErrorCode::InvalidStartByte);
        }

        msg.control = self.buffer[1];
        msg.arm_adr = self.buffer[2];
        msg.device_adr = self.buffer[3];

        // get the message block
        let mut found = false;
        let mut pos = 4;
        while pos < self.buffer.len() {
            let ch = self.buffer[pos];
            if ch == Self::ETX {
                found = true;
                break;
            }
            // copy into message
            if msg.message.push(ch).is_err() {
                error!("Message buffer overflow");
                return XYZMessage::new_error(ErrorCode::MessageBufferOverflow);
            }
            // move to the next byte
            pos += 1;
        }
        if !found {
            return XYZMessage::new_error(ErrorCode::PartialMessageReceived);
        }

        pos += 1; // Move to the VRC byte
        if pos >= self.buffer.len() {
            return XYZMessage::new_error(ErrorCode::PartialMessageReceived);
        }
        msg.vrc = self.buffer[pos]; // checksum byte

        // check that the VRC is correct
        let mut vrc_calc = 0u8;
        for &byte in &self.buffer[0..pos] {
            vrc_calc ^= byte; // Calculate VRC by XORing all bytes
        }
        if vrc_calc != msg.vrc {
            error!("VRC mismatch: calculated {} but found {}", vrc_calc, msg.vrc);
            for &byte in self.buffer.iter() {
                error!("Buffer byte: {}", byte);
            }
            return XYZMessage::new_error(ErrorCode::VRCMismatch);
        }

        // Remove the processed message from the buffer
        for _ in 0..pos+1 {
            self.buffer.remove(0);
        }

        info!("Parsed message. Arm_adr = {}", msg.arm_adr);
        msg
    }

    pub fn skip_to_next_start_byte(&mut self) {
        while self.buffer.len() > 0 && self.buffer[0] != Self::STX {
            // Remove bytes until we find STX or the buffer is empty
            self.buffer.remove(0);
        }
    }
}