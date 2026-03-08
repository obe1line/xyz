use heapless::Vec;
use crate::prelude::rust_2024::derive;
use core::fmt::Debug;
use crate::default::Default;
use crate::cmp::PartialEq;
use crate::concat;
use core::stringify;
use core::include_str;
use simple_mermaid::mermaid;
use itoa;

#[cfg(feature = "embedded")]
use { defmt::{info, Format} };

#[cfg(not(feature = "embedded"))]
use { log::{info} };

#[cfg_attr(feature = "embedded", derive(Format))]
#[derive(Debug, Default, PartialEq, Copy, Clone)]
pub enum ErrorCode {
    #[default]
    NoError,
    PartialMessageReceived,
    InvalidStartByte,
    VRCMismatch,
    MessageBufferOverflow,
    NoMessagesAvailable,
    InvalidArmAddress,
    InvalidDeviceAddress,
    InvalidCommand,
}

#[derive(Debug)]
#[derive(Default)]
pub struct XYZCommand {
    pub cmd: heapless::String<{ XYZCommand::MAX_COMMAND_LENGTH }>,
    pub params: [i32; XYZCommand::MAX_PARAMS],
    pub num_params: u8,
}

impl XYZCommand {
    pub fn get_param(&self, index: u8) -> Option<i32> {
        if index < self.num_params {
            Some(self.params[index as usize])
        } else {
            None
        }
    }
}

impl XYZCommand {
    pub fn new() -> XYZCommand {
        let command = XYZCommand {
            cmd: heapless::String::new(),
            params: [0; XYZCommand::MAX_PARAMS],
            num_params: 0,
        };
        command
    }

    pub fn encode(&self) -> Vec<u8, { XYZMessage::MESSAGE_BUFFER_SIZE }> {
        let mut data = Vec::<u8, { XYZMessage::MESSAGE_BUFFER_SIZE }>::new();
        for byte in self.cmd.as_str().as_bytes() {
            data.push(*byte).ok();
        }
        // add parameters
        let mut buffer = itoa::Buffer::new();
        for i in 0..self.num_params {
            data.push(b' ').ok(); // space before parameter
            let param_str = buffer.format(self.params[i as usize]);
            for byte in param_str.as_bytes() {
                data.push(*byte).ok();
            }
        }
        data
    }

    pub fn decode(data: &[u8]) -> Result<XYZCommand, ErrorCode> {
        let mut command = XYZCommand::new();
        // ensure data is valid ASCII
        if !data.is_ascii() {
            info!("Not ASCII: {:?}", data);
            return Err(ErrorCode::InvalidCommand);
        }

        // split on spaces to get command and parameters
        let pieces = data.split(|&b| b == b' ').collect::<Vec<&[u8], 5>>();
        command.num_params = pieces.len() as u8 - 1; // first piece is command, rest are parameters
        // parse the command
        if let Some(cmd_bytes) = pieces.get(0) {
            if let Ok(cmd_str) = core::str::from_utf8(cmd_bytes) {
                command.cmd.push_str(cmd_str).map_err(|_| ErrorCode::InvalidCommand)?;
            } else {
                info!("Invalid command string: {:?}", cmd_bytes);
                return Err(ErrorCode::InvalidCommand);
            }
        } else {
            info!("No command found in data: {:?}", data);
            return Err(ErrorCode::InvalidCommand);
        }
        // parse the parameters
        for (i, param_bytes) in pieces.iter().skip(1).enumerate() {
            if let Ok(param_str) = core::str::from_utf8(param_bytes) {
                if let Ok(param_val) = param_str.parse::<i32>() {
                    if i < XYZCommand::MAX_PARAMS {
                        command.params[i] = param_val;
                    } else {
                        info!("Too many parameters: {:?}", pieces.len());
                        return Err(ErrorCode::InvalidCommand);
                    }
                } else {
                    info!("Invalid parameter value: {:?}", param_str);
                    return Err(ErrorCode::InvalidCommand);
                }
            } else {
                info!("Invalid parameter string: {:?}", param_bytes);
                return Err(ErrorCode::InvalidCommand);
            }
        }
        Ok(command)
    }
}

impl XYZCommand {
    pub const MAX_COMMAND_LENGTH: usize = 20; // Maximum length of a command string
    pub const MAX_PARAMS: usize = 4; // Maximum number of parameters
}

/// Represents a transaction in the XYZ protocol.
///
/// Fields:
/// - `arm_address`: The address of the arm.
/// - `device_address`: The address of the device.
/// - `cmd_message`: The command message to be sent.
/// - `cmd_ack`: The acknowledgment for the command message.
/// - `rsp_message`: The response message received.
/// - `rsp_ack`: The acknowledgment for the response message.
///
#[doc = mermaid!("../../../diagrams/xyz_messages.mermaid")]
pub struct Transaction {
    pub arm_address: u8,
    pub device_address: u8,
    pub cmd_message: XYZMessage,
    pub cmd_ack: XYZMessage,
    pub rsp_message: XYZMessage,
    pub rsp_ack: XYZMessage,
}

pub struct PumpCommand {
    pub error_code: ErrorCode,
    pub pump_adr: u8,
    pub sequence_num: u8,
    pub message_data: Vec<u8, { PumpCommand::MESSAGE_BUFFER_SIZE }>,
    pub vrc: u8,
}

#[cfg(feature = "embedded")]
impl defmt::Format for PumpCommand {
    fn format(&self, _fmt: defmt::Formatter) {
    }
}

impl PumpCommand {
    // TODO: Retry bit not implemented.
    pub const SEQUENCE_MASK: u8 = 0x0F;
    #[cfg(test)]
    pub const MESSAGE_BUFFER_SIZE: usize = 32;
    #[cfg(not(test))]
    pub const MESSAGE_BUFFER_SIZE: usize = 255;

    pub fn new(error_code: ErrorCode, pump_adr: u8, sequence_num: u8,
               message_data: Vec<u8, { PumpCommand::MESSAGE_BUFFER_SIZE }>,
               vrc: u8) -> PumpCommand {
        PumpCommand {
            error_code,
            pump_adr,
            sequence_num,
            message_data,
            vrc
        }
    }

    pub fn new_from_xyz(xyz: XYZMessage) -> PumpCommand {
        let mut message_data = Vec::new();
        message_data.extend_from_slice(&xyz.message_data).ok();
        PumpCommand {
            error_code: xyz.error_code,
            pump_adr: xyz.device_adr,
            sequence_num: xyz.control & Self::SEQUENCE_MASK,
            message_data,
            vrc: 0,
        }
    }

    pub fn new_ack(pump_adr: u8, sequence_num: u8) -> PumpCommand {
        PumpCommand {
            error_code: ErrorCode::NoError,
            pump_adr,
            sequence_num,
            message_data: Vec::new(),
            vrc: 0,
        }
    }
}

impl PumpCommand {
    pub fn encode(&self) -> Vec<u8, 255> {
        let mut encoded: Vec<u8, 255> = Vec::new();
        encoded.push(0x02u8).ok(); // STX
        encoded.push(self.pump_adr).ok();
        encoded.push(self.sequence_num).ok();
        encoded.extend_from_slice(&self.message_data).ok();
        encoded.push(0x03u8).ok(); // ETX
        let vrc_calc = calculate_vrc(&encoded);
        encoded.push(vrc_calc).ok();
        encoded
    }
}

pub struct PumpResponse {
    pub error_code: ErrorCode,
    pub master_adr: u8,
    pub status: u8,
    pub message_data: Vec<u8, { PumpResponse::MESSAGE_BUFFER_SIZE }>,
    pub vrc: u8,
}

#[cfg(feature = "embedded")]
impl defmt::Format for PumpResponse {
    fn format(&self, _fmt: defmt::Formatter) {
    }
}

impl PumpResponse {
    pub const STATUS_MASK: u8 = 0x0F;
    #[cfg(test)]
    pub const MESSAGE_BUFFER_SIZE: usize = 32;
    #[cfg(not(test))]
    pub const MESSAGE_BUFFER_SIZE: usize = 255;

    pub fn new(error_code: ErrorCode, master_adr: u8, status: u8,
               message_data: Vec<u8, { PumpResponse::MESSAGE_BUFFER_SIZE }>,
               vrc: u8) -> PumpResponse {
        PumpResponse {
            error_code,
            master_adr,
            status,
            message_data,
            vrc
        }
    }

    pub fn new_from_xyz(xyz: XYZMessage) -> PumpResponse {
        let mut message_data = Vec::new();
        message_data.extend_from_slice(&xyz.message_data).ok();
        PumpResponse {
            error_code: xyz.error_code,
            master_adr: xyz.device_adr,
            status: xyz.control & Self::STATUS_MASK,
            message_data,
            vrc: 0,
        }
    }
}

impl PumpResponse {
    pub fn encode(&self) -> Vec<u8, 255> {
        let mut encoded: Vec<u8, 255> = Vec::new();
        encoded.push(0x02u8).ok(); // STX
        encoded.push(self.master_adr).ok();
        encoded.push(self.status).ok();
        encoded.extend_from_slice(&self.message_data).ok();
        encoded.push(0x03u8).ok(); // ETX
        let vrc_calc = calculate_vrc(&encoded);
        encoded.push(vrc_calc).ok();
        encoded
    }
}

/// Represents a message in the XYZ protocol.
/// Fields:
/// - `error_code`: The error code if any.
/// - `control`: The control byte.
/// - `arm_adr`: The arm address byte.
/// - `device_adr`: The device address byte.
/// - `messages`: A vector of messages per device, each message is a fixed-size vector.
/// - `vrc`: The VRC (Vertical Redundancy Check) byte.
#[doc = mermaid!("../../../diagrams/xyz_binary_packet.mermaid")]
#[derive(Debug, Default)]
pub struct XYZMessage {
    pub error_code: ErrorCode, // Error code if any
    pub control: u8, // Control byte
    pub arm_adr: u8,     // Arm byte
    pub device_adr: u8,  // Device byte
    pub message_data: Vec::<u8, { XYZMessage::MESSAGE_BUFFER_SIZE }>,
    pub vrc: u8, // VRC byte
}

#[cfg(feature = "embedded")]
impl defmt::Format for XYZMessage {
    fn format(&self, fmt: defmt::Formatter) {
        let msg_str = core::str::from_utf8(&self.message_data).ok().unwrap();
        // TODO: implement Format for Vec
        defmt::write!(fmt, "XYZMessage {{ error_code: {:?}, control: 0x{:02X}, arm_adr: 0x{:02X} ('{}'), device_adr: 0x{:02X} ('{}'), message_data: {}, vrc: 0x{:02X} }}",
            self.error_code,
            self.control,
            self.arm_adr, self.arm_adr as char,
            self.device_adr, self.device_adr as char,
            msg_str,
            self.vrc
        );
    }

}

impl XYZMessage {
    #[cfg(test)]
    pub const MESSAGE_BUFFER_SIZE: usize = 32; // Increased from 8 to accommodate larger test messages
    #[cfg(not(test))]
    pub const MESSAGE_BUFFER_SIZE: usize = 255; // Maximum size of a message buffer
    pub const SEQ_MASK: u8 = 0x07u8; // Sequence number mask (lower 3 bits)

    pub const ACK: u8 = 0x40u8; // Acknowledge byte
    // pub const CONTROL_BASE : u8 = 0x40u8; // Base control byte is b01000000
    // pub const CONTROL_REP: u8 = 0x08u8; // Message is a retransmission if this bit is set
    // pub const CONTROL_SEQ_MASK: u8 = 0xF8u8; // Control sequence mask for the lower 3 bits
    // control sequence number bits are lower 3 bits (values 1..7)

    pub fn new_ack(arm_adr: u8, device_adr: u8) -> XYZMessage {
        let msg = XYZMessage {
            error_code: ErrorCode::NoError,
            control: Self::ACK,
            arm_adr,
            device_adr,
            message_data: Vec::new(),
            vrc: 0,
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
        // TODO: Handle multiple messages per device
        encoded.extend_from_slice(&self.message_data).ok(); // Append message bytes
        encoded.push(0x03u8).ok(); // ETX (End of Text)
        let vrc_calc = calculate_vrc(&encoded);
        encoded.push(vrc_calc).ok(); // Append the VRC byte
        encoded
    }
}

pub fn calculate_vrc(data: &[u8]) -> u8 {
    let mut vrc_calc = 0u8;
    for &byte in data.iter() {
        vrc_calc ^= byte; // Calculate VRC by XORing all bytes
    }
    vrc_calc
}

impl XYZMessage {
    pub fn new(error_code: ErrorCode, control: u8, arm_adr: u8, device_adr: u8,
               message_data: Vec<u8, {XYZMessage::MESSAGE_BUFFER_SIZE}>,
               vrc: u8) -> XYZMessage {
        let msg = XYZMessage {
            error_code,
            control,
            arm_adr,
            device_adr,
            message_data,
            vrc
        };
        msg
    }

    pub fn create_answer(in_msg: &XYZMessage, data: Vec<u8, 255>, error_code: u8) -> XYZMessage {
        // answer message:
        // Control byte: b01010001 (0x51)
        //    Bit 7: 0 (always 0)
        //    Bit 6: 1 (always 1)
        //    Bit 5: 0 (arm address ok)
        //    Bit 4: 1 (done - completed without error)
        //    Bit 3: 0 (not a retried message)
        //    Bit 2-0: 0x01-0x0F (sequence number to device)
        let mut answer_data = Vec::new();
        let mut ctrl_byte: u8 = 0x50; // answer control byte
        answer_data.extend(data);
        if error_code != 0 {
            // insert error code at start of data
            ctrl_byte = 0x40; // clear done bit to indicate error
            answer_data.insert(0, error_code).expect("Failed to insert error code");
        }
        XYZMessage::new(
            crate::ErrorCode::NoError,
            (in_msg.control & XYZMessage::SEQ_MASK) | ctrl_byte,  // send back the original sequence number
            in_msg.arm_adr,
            in_msg.device_adr,
            answer_data,
            0, // VRC will be calculated in encode()
        )
    }
}

impl XYZMessage {
    pub fn new_error(err: ErrorCode) -> XYZMessage {
        let mut msg = XYZMessage::default();
        msg.error_code = err;
        msg
    }
}

#[cfg(test)]
mod xyz_message_tests {
    use super::*;

    #[test]
    fn test_error_code_equality() {
        // check that all items in the enum are equal to themselves (the PartialEq trait)
        assert_eq!(ErrorCode::NoError, ErrorCode::NoError);
        assert_eq!(ErrorCode::PartialMessageReceived, ErrorCode::PartialMessageReceived);
        assert_eq!(ErrorCode::InvalidStartByte, ErrorCode::InvalidStartByte);
        assert_eq!(ErrorCode::VRCMismatch, ErrorCode::VRCMismatch);
        assert_eq!(ErrorCode::MessageBufferOverflow, ErrorCode::MessageBufferOverflow);
        assert_eq!(ErrorCode::NoMessagesAvailable, ErrorCode::NoMessagesAvailable);
        assert_eq!(ErrorCode::InvalidArmAddress, ErrorCode::InvalidArmAddress);
        assert_eq!(ErrorCode::InvalidDeviceAddress, ErrorCode::InvalidDeviceAddress);
        assert_ne!(ErrorCode::NoError, ErrorCode::PartialMessageReceived);
    }

    #[test]
    fn test_xyz_message_creation() {
        let message_data: Vec<u8, { XYZMessage::MESSAGE_BUFFER_SIZE }> = Vec::new();
        let msg = XYZMessage::new(
            ErrorCode::NoError,
            XYZMessage::ACK,
            '1' as u8,
            '2' as u8,
            message_data,
            0, // Placeholder for VRC, will be calculated later
        );
        assert_eq!(msg.error_code, ErrorCode::NoError);
        assert_eq!(msg.control, XYZMessage::ACK);
        assert_eq!(msg.arm_adr, '1' as u8);
        assert_eq!(msg.device_adr, '2' as u8);
        assert!(msg.message_data.is_empty());
        assert_eq!(msg.vrc, 0); // VRC is initially 0
    }

    #[test]
    fn test_xyz_message_encode() {
        let message_data: Vec<u8, { XYZMessage::MESSAGE_BUFFER_SIZE }> = Vec::from_slice(&[0x41, 0x42, 0x43]).unwrap();
        let msg = XYZMessage::new(
            ErrorCode::NoError,
            0x06,
            '3' as u8,
            '4' as u8,
            message_data,
            0, // Placeholder for VRC, will be calculated later
        );
        let encoded = msg.encode();
        assert_eq!(encoded.len(), 9); // STX + control + arm_adr
        assert_eq!(encoded[0], 0x02); // STX
        assert_eq!(encoded[1], 0x06); // Control byte
        assert_eq!(encoded[2], '3' as u8); // Arm address
        assert_eq!(encoded[3], '4' as u8); // Device address
        assert_eq!(encoded[4], 'A' as u8); // First message byte
        assert_eq!(encoded[5], 'B' as u8); // Second message byte
        assert_eq!(encoded[6], 'C' as u8); // Third message byte
        assert_eq!(encoded[7], 0x03); // ETX
        assert_eq!(encoded[8], 0x40); // VRC
    }

    #[test]
    fn test_new_ack_and_new_error() {
        let ack = XYZMessage::new_ack(1, 2);
        assert_eq!(ack.error_code, ErrorCode::NoError);
        assert_eq!(ack.control, XYZMessage::ACK);
        assert_eq!(ack.arm_adr, 1);
        assert_eq!(ack.device_adr, 2);
        assert_eq!(ack.vrc, 0);
        assert!(ack.message_data.is_empty());

        let err = XYZMessage::new_error(ErrorCode::InvalidDeviceAddress);
        assert_eq!(err.error_code, ErrorCode::InvalidDeviceAddress);
        // other fields should be default (0 or empty)
        assert_eq!(err.control, 0);
        assert_eq!(err.arm_adr, 0);
        assert_eq!(err.device_adr, 0);
        assert_eq!(err.vrc, 0);
        assert!(err.message_data.is_empty());
    }

    #[test]
    fn test_encode_empty_message() {
        let empty_data: Vec<u8, { XYZMessage::MESSAGE_BUFFER_SIZE }> = Vec::new();
        let msg = XYZMessage::new(ErrorCode::NoError, 0x00, '1' as u8, '1' as u8, empty_data, 0);
        let encoded = msg.encode();
        // Should be STX, control, arm, dev, ETX, VRC -> 6 bytes
        assert_eq!(encoded.len(), 6);
        assert_eq!(encoded[0], 0x02);
        assert_eq!(encoded[1], 0x00);
        assert_eq!(encoded[2], '1' as u8);
        assert_eq!(encoded[3], '1' as u8);
        assert_eq!(encoded[4], 0x03);
        let expected_vrc = 0x02 ^ 0x00 ^ ('1' as u8) ^ ('1' as u8) ^ 0x03;
        assert_eq!(encoded[5], expected_vrc);
    }

    #[test]
    fn test_pump_command_encode() {
        let message_data: Vec<u8, { PumpCommand::MESSAGE_BUFFER_SIZE }> = Vec::from_slice(&[0x41, 0x42]).unwrap();
        let msg = PumpCommand::new(
            ErrorCode::NoError,
            'P' as u8,
            0x01,
            message_data,
            0,
        );
        let encoded = msg.encode();
        // STX, pump_adr, sequence, 'A', 'B', ETX, VRC -> 7 bytes
        assert_eq!(encoded.len(), 7);
        assert_eq!(encoded[0], 0x02); // STX
        assert_eq!(encoded[1], 'P' as u8); // Pump address
        assert_eq!(encoded[2], 0x01); // Sequence number
        assert_eq!(encoded[3], 0x41); // 'A'
        assert_eq!(encoded[4], 0x42); // 'B'
        assert_eq!(encoded[5], 0x03); // ETX
        let expected_vrc = 0x02 ^ ('P' as u8) ^ 0x01 ^ 0x41 ^ 0x42 ^ 0x03;
        assert_eq!(encoded[6], expected_vrc);
    }

    #[test]
    fn test_pump_command_new_from_xyz() {
        let message_data: Vec<u8, { XYZMessage::MESSAGE_BUFFER_SIZE }> = Vec::from_slice(&[0x11, 0x22]).unwrap();
        let xyz = XYZMessage::new(
            ErrorCode::NoError,
            0x46,
            '3' as u8,
            'D' as u8,
            message_data,
            0,
        );
        let pump = PumpCommand::new_from_xyz(xyz);
        assert_eq!(pump.pump_adr, 'D' as u8);
        assert_eq!(pump.sequence_num, 0x06);
        assert_eq!(pump.message_data.len(), 2);
        assert_eq!(pump.message_data[0], 0x11);
        assert_eq!(pump.message_data[1], 0x22);
        assert_eq!(pump.error_code, ErrorCode::NoError);
    }

    #[test]
    fn test_decode_xyz_to_pump() {
        use crate::components::xyz_parser::XYZMessageParser;
        let mut parser = XYZMessageParser::new();
        let data = [
            0x02, 0x44, 0x31, 0x31, 0x4c, 0x31, 0x30, 0x76, 0x35, 0x30, 0x56, 0x31, 0x32, 0x30,
            0x63, 0x35, 0x30, 0x4b, 0x33, 0x41, 0x31, 0x32, 0x52, 0x03, 0x10,
        ];

        parser.add_data(&data, data.len());
        let xyz = parser.parse();

        assert_eq!(xyz.error_code, ErrorCode::NoError);
        assert_eq!(xyz.control, 0x44);
        assert_eq!(xyz.arm_adr, 0x31); // '1'
        assert_eq!(xyz.device_adr, 0x31); // '1'
        assert_eq!(xyz.vrc, 0x10);

        let pump = PumpCommand::new_from_xyz(xyz);
        assert_eq!(pump.pump_adr, 0x31);
        assert_eq!(pump.sequence_num, 0x04);
        assert_eq!(pump.error_code, ErrorCode::NoError);
        // data should be "L10v50V120c50K3A12R"
        assert_eq!(pump.message_data.as_slice(), &data[4..23]);
    }

    #[test]
    fn test_pump_command_decode() {
        let pump_adr = 'P' as u8;
        let sequence_num = 0x05;
        let ack = PumpCommand::new_ack(pump_adr, sequence_num);

        assert_eq!(ack.pump_adr, pump_adr);
        assert_eq!(ack.sequence_num, sequence_num);
        assert_eq!(ack.message_data.len(), 0);
        assert_eq!(ack.error_code, ErrorCode::NoError);

        let encoded = ack.encode();
        // STX, pump_adr, sequence, ETX, VRC -> 5 bytes
        assert_eq!(encoded.len(), 5);
        assert_eq!(encoded[0], 0x02); // STX
        assert_eq!(encoded[1], pump_adr);
        assert_eq!(encoded[2], sequence_num);
        assert_eq!(encoded[3], 0x03); // ETX
        let expected_vrc = 0x02 ^ pump_adr ^ sequence_num ^ 0x03;
        assert_eq!(encoded[4], expected_vrc);
    }

    #[test]
    fn test_pump_response_encode() {
        let message_data: Vec<u8, { PumpResponse::MESSAGE_BUFFER_SIZE }> = Vec::from_slice(&[0x30, 0x31]).unwrap();
        let msg = PumpResponse::new(
            ErrorCode::NoError,
            'M' as u8,
            0x02,
            message_data,
            0,
        );
        let encoded = msg.encode();
        // STX, master_adr, status, '0', '1', ETX, VRC -> 7 bytes
        assert_eq!(encoded.len(), 7);
        assert_eq!(encoded[0], 0x02); // STX
        assert_eq!(encoded[1], 'M' as u8); // Master address
        assert_eq!(encoded[2], 0x02); // Status
        assert_eq!(encoded[3], 0x30); // '0'
        assert_eq!(encoded[4], 0x31); // '1'
        assert_eq!(encoded[5], 0x03); // ETX
        let expected_vrc = 0x02 ^ ('M' as u8) ^ 0x02 ^ 0x30 ^ 0x31 ^ 0x03;
        assert_eq!(encoded[6], expected_vrc);
    }
}
