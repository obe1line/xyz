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

#[cfg(feature = "std")]
use { log::{info} };

#[cfg(all(not(feature = "embedded"), not(feature = "std")))]
macro_rules! info {
    ($($arg:tt)*) => {};
}

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

#[cfg_attr(feature = "embedded", derive(Format))]
#[derive(Debug, Default, PartialEq, Copy, Clone)]
pub enum CavroDeviceType {
    #[default]
    XYZ,
    PUMP,
}

#[derive(Debug, Default, PartialEq, Clone)]
pub struct XYZCommand {
    pub error_code: ErrorCode,
    pub repeat: bool,
    pub sequence_number: u8,
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
            error_code: ErrorCode::NoError,
            repeat: false,
            sequence_number: 0u8,
            cmd: heapless::String::new(),
            params: [0; XYZCommand::MAX_PARAMS],
            num_params: 0,
        };
        command
    }

    pub fn encode(&self) -> Vec<u8, { CavroMessage::MESSAGE_BUFFER_SIZE }> {
        let mut data = Vec::<u8, { CavroMessage::MESSAGE_BUFFER_SIZE }>::new();
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

    pub fn decode(xyz: XYZMessage) -> XYZCommand {
        let mut command = XYZCommand::new();

        command.sequence_number = xyz.control & 0x07;
        command.repeat = (xyz.control & 0x08) != 0;

        let data = xyz.message_data;
        // ensure data is valid ASCII
        if !data.is_ascii() {
            info!("Not ASCII: {:?}", data.as_slice());
            command.error_code = ErrorCode::InvalidCommand;
            return command;
        }

        // split on spaces to get command and parameters
        let pieces = data.split(|&b| b == b' ').collect::<Vec<&[u8], 5>>();
        command.num_params = pieces.len() as u8 - 1; // first piece is command, rest are parameters
        // parse the command
        if let Some(cmd_bytes) = pieces.get(0) {
            if let Ok(cmd_str) = core::str::from_utf8(cmd_bytes) {
                command.cmd.push_str(cmd_str).ok();
            } else {
                info!("Invalid command string: {:?}", cmd_bytes);
                command.error_code = ErrorCode::InvalidCommand;
                return command;
            }
        } else {
            info!("No command found in data: {:?}", data.as_slice());
            command.error_code = ErrorCode::InvalidCommand;
            return command;
        }
        // parse the parameters
        for (i, param_bytes) in pieces.iter().skip(1).enumerate() {
            if let Ok(param_str) = core::str::from_utf8(param_bytes) {
                if let Ok(param_val) = param_str.parse::<i32>() {
                    if i < XYZCommand::MAX_PARAMS {
                        command.params[i] = param_val;
                    } else {
                        info!("Too many parameters: {:?}", pieces.len());
                        command.error_code = ErrorCode::InvalidCommand;
                        return command;
                    }
                } else {
                    info!("Invalid parameter value: {:?}", param_str);
                    command.error_code = ErrorCode::InvalidCommand;
                    return command;
                }
            } else {
                info!("Invalid parameter string: {:?}", param_bytes);
                command.error_code = ErrorCode::InvalidCommand;
                return command;
            }
        }
        command
    }
}

impl XYZCommand {
    pub const MAX_COMMAND_LENGTH: usize = 20; // Maximum length of a command string
    pub const MAX_PARAMS: usize = 4; // Maximum number of parameters
}

#[derive(Debug, Default, PartialEq, Clone)]
pub struct CavroMessage {
    pub error_code: ErrorCode,
    pub message_data: Vec<u8, { CavroMessage::MESSAGE_BUFFER_SIZE }>,
    pub vrc: u8,
    pub vrc_calculated: u8,
}

#[derive(Debug, PartialEq, Clone)]
pub enum CavroCommand {
    XYZ(XYZCommand),
    Pump(PumpCommand),
}

impl CavroMessage {
    #[cfg(test)]
    pub const MESSAGE_BUFFER_SIZE: usize = 32;
    #[cfg(not(test))]
    pub const MESSAGE_BUFFER_SIZE: usize = 255;

    pub const STX: u8 = 0x02;
    pub const ETX: u8 = 0x03;

    pub fn new_error(error_code: ErrorCode) -> CavroMessage {
        CavroMessage {
            error_code,
            message_data: Vec::new(),
            vrc: 0,
            vrc_calculated: 0,
        }
    }

    pub fn new(message_data: Vec<u8, { CavroMessage::MESSAGE_BUFFER_SIZE }>) -> CavroMessage {
        CavroMessage {
            error_code: ErrorCode::NoError,
            message_data,
            vrc: 0,
            vrc_calculated: 0,
        }
    }

    pub fn decode(message_data: Vec<u8, { CavroMessage::MESSAGE_BUFFER_SIZE }>) -> Result<CavroMessage, ErrorCode> {
        let msg_len = message_data.len();
        // pump command: <pmp_adr><seq>[data]
        // xyz command:  <ctrl><arm><dev>[data]
        if msg_len < 2 {
            // we should not get this if the receiver reads from 0x02 to 0x03 and an extra VRC byte
            // but, in case it isn't checking, return an error
            return Err(ErrorCode::InvalidCommand)
        }
        
        let vrc = message_data[msg_len - 1];
        // check the vrc
        let (_end, data_no_vrc) = message_data.split_last().unwrap();
        let vrc_calculated = calculate_vrc(data_no_vrc);
        let error_code = if vrc_calculated != vrc { ErrorCode::VRCMismatch } else { ErrorCode::NoError };

        let (_start, data_with_end) = message_data.split_first().unwrap();
        let (data, _end) = data_with_end.split_last_chunk::<2>().unwrap();
        let mut message_data = Vec::new();
        message_data.extend_from_slice(data).ok();

        Ok(CavroMessage {
            error_code,
            message_data,
            vrc,
            vrc_calculated,
        })
    }

    pub fn decode_to_xyz_command(&self) -> XYZCommand {
        let xyz = XYZMessage::decode(self.message_data.clone());
        XYZCommand::decode(xyz)
    }

    pub fn decode_to_pump_command(&self) -> PumpCommand {
        let pump = PumpCommand::decode(self.message_data.clone());
        pump
    }

    pub fn encode(&self) -> Vec<u8, 255> {
        let mut encoded: Vec<u8, 255> = Vec::new();
        encoded.push(Self::STX).ok();
        encoded.extend_from_slice(&self.message_data).ok();
        encoded.push(Self::ETX).ok();
        let vrc_calc = calculate_vrc(&encoded);
        encoded.push(vrc_calc).ok();
        encoded
    }
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
pub struct XYZTransaction {
    pub arm_address: u8,
    pub device_address: u8,
    pub cmd_message: XYZMessage,
    pub cmd_ack: XYZMessage,
    pub rsp_message: XYZMessage,
    pub rsp_ack: XYZMessage,
}

#[derive(Debug, PartialEq, Clone)]
pub struct PumpCommand {
    pub error_code: ErrorCode,
    pub pump_address: u8,
    pub sequence_num: u8,
    pub message_data: Vec<u8, { CavroMessage::MESSAGE_BUFFER_SIZE }>,
}

impl PumpCommand {
    // TODO: Retry bit not implemented.
    pub const SEQUENCE_MASK: u8 = 0x0F;

    pub fn new(error_code: ErrorCode, pump_address: u8, sequence_num: u8,
               message_data: Vec<u8, { CavroMessage::MESSAGE_BUFFER_SIZE }>) -> PumpCommand {
        PumpCommand {
            error_code,
            pump_address,
            sequence_num,
            message_data
        }
    }

    pub fn new_ack(pump_address: u8) -> PumpCommand {
        PumpCommand {
            error_code: ErrorCode::NoError,
            pump_address,
            sequence_num: 0x40,
            message_data: Vec::new(),
        }
    }
    
    pub fn decode(message_data: Vec<u8, { CavroMessage::MESSAGE_BUFFER_SIZE }>) -> PumpCommand {
        let (header, data) = message_data.split_first_chunk::<2>().unwrap();
        let pump_address = header[0];
        let sequence_num = header[1];
        let mut message_data = Vec::new();
        message_data.extend_from_slice(data).ok();
        PumpCommand {
            error_code: ErrorCode::NoError,
            pump_address,
            sequence_num,
            message_data,
        }
    }
}

impl PumpCommand {
    pub fn encode(&self) -> Vec<u8, 255> {
        // encode the data in the pump format
        let mut encoded: Vec<u8, 255> = Vec::new();
        encoded.push(self.pump_address).ok();
        encoded.push(self.sequence_num).ok();
        encoded.extend_from_slice(&self.message_data).ok();
        encoded
    }
}

pub struct PumpResponse {
    pub error_code: ErrorCode,
    pub master_adr: u8,
    pub status: u8,
    pub message_data: Vec<u8, { CavroMessage::MESSAGE_BUFFER_SIZE }>,
}

impl PumpResponse {
    pub const STATUS_MASK: u8 = 0x0F;

    pub fn new(error_code: ErrorCode, master_adr: u8, status: u8,
               message_data: Vec<u8, { CavroMessage::MESSAGE_BUFFER_SIZE }>) -> PumpResponse {
        PumpResponse {
            error_code, master_adr, status, message_data
        }
    }

    pub fn decode(cavro: CavroMessage) -> PumpResponse {
        PumpResponse {
            error_code: cavro.error_code,
            master_adr: 0,
            status: 0,
            message_data: cavro.message_data,
        }
    }
}

/// Represents a message in the XYZ protocol.
/// Fields:
/// - `cavro`: The underlying Cavro message.
/// - `arm_adr`: The arm address byte.
#[doc = mermaid!("../../../diagrams/xyz_binary_packet.mermaid")]
#[derive(Debug, Default)]
pub struct XYZMessage {
    pub error_code: ErrorCode,
    pub control: u8,
    pub arm_address: u8,
    pub device_address: u8,
    pub message_data: Vec<u8, { CavroMessage::MESSAGE_BUFFER_SIZE }>,
}

impl XYZMessage {
    pub const XYZ_DEV_ADR: u8 = 8;
    pub const PUMP_DEV_ADR_MIN: u8 = 0x31;
    pub const PUMP_DEV_ADR_MAX: u8 = 0x34;

    pub fn is_pump_device(&self) -> bool {
        (self.device_address >= Self::PUMP_DEV_ADR_MIN) &&
            (self.device_address <= Self::PUMP_DEV_ADR_MAX)
    }

    pub fn is_xyz_device(&self) -> bool {
        self.device_address == Self::XYZ_DEV_ADR
    }
}

impl XYZMessage {
    pub const MESSAGE_BUFFER_SIZE: usize = CavroMessage::MESSAGE_BUFFER_SIZE;
    pub const SEQ_MASK: u8 = 0x07u8; // Sequence number mask (lower 3 bits)

    pub const ACK: u8 = 0x40u8; // Acknowledge byte
    // pub const CONTROL_BASE : u8 = 0x40u8; // Base control byte is b01000000
    // pub const CONTROL_REP: u8 = 0x08u8; // Message is a retransmission if this bit is set
    // pub const CONTROL_SEQ_MASK: u8 = 0xF8u8; // Control sequence mask for the lower 3 bits
    // control sequence number bits are lower 3 bits (values 1..7)

    pub fn new_ack(arm_address: u8, device_address: u8) -> XYZMessage {
        let msg = XYZMessage {
            error_code: ErrorCode::NoError,
            control: Self::ACK,
            arm_address,
            device_address,
            message_data: Vec::new(),
        };
        msg
    }
}

impl XYZMessage {
    pub fn encode(&self) -> Vec<u8, 255> {
        let mut encoded: Vec<u8, 255> = Vec::new();
        encoded.push(self.control).ok();
        encoded.push(self.arm_address).ok();
        encoded.push(self.device_address).ok(); // device_adr in XYZ
        encoded.extend_from_slice(&self.message_data).ok();
        encoded
    }

    pub fn decode(message_data: Vec<u8, { CavroMessage::MESSAGE_BUFFER_SIZE }>) -> XYZMessage {
        let (header, data) = message_data.split_first_chunk::<3>().unwrap();
        let control = header[0];
        let arm_address = header[1];
        let device_address = header[2];
        let mut message_data = Vec::new();
        message_data.extend_from_slice(data).ok();

        let msg = XYZMessage {
            error_code: ErrorCode::NoError,
            control,
            arm_address,
            device_address,
            message_data,
        };
        msg
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
    pub fn new(error_code: ErrorCode, control: u8, arm_address: u8, device_address: u8,
               message_data: Vec<u8, { CavroMessage::MESSAGE_BUFFER_SIZE }>) -> XYZMessage {
        XYZMessage {
            error_code, control, arm_address, device_address, message_data,
        }
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
            ErrorCode::NoError,
            (in_msg.control & XYZMessage::SEQ_MASK) | ctrl_byte,  // send back the original sequence number
            in_msg.arm_address,
            in_msg.device_address,
            answer_data,
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
    fn test_cavro_message_decode() {
        let message_data: Vec<u8, { CavroMessage::MESSAGE_BUFFER_SIZE }> = Vec::from_slice(&[0x02, 0x41, 0x42, 0x43, 0x03, 0x41]).unwrap();
        let msg = CavroMessage::decode(message_data).ok().unwrap();
        assert_eq!(msg.error_code, ErrorCode::NoError);
    }

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
        let message_data: Vec<u8, { CavroMessage::MESSAGE_BUFFER_SIZE }> = Vec::new();
        let msg = XYZMessage::new(
            ErrorCode::NoError,
            XYZMessage::ACK,
            '1' as u8,
            '2' as u8,
            message_data,
        );
        assert_eq!(msg.error_code, ErrorCode::NoError);
        assert_eq!(msg.control, XYZMessage::ACK);
        assert_eq!(msg.arm_address, '1' as u8);
        assert_eq!(msg.device_address, '2' as u8);
        assert!(msg.message_data.is_empty());
    }

    #[test]
    fn test_xyz_message_encode() {
        let message_data: Vec<u8, { CavroMessage::MESSAGE_BUFFER_SIZE }> = Vec::from_slice(&[0x41, 0x42, 0x43]).unwrap();
        let msg = XYZMessage::new(
            ErrorCode::NoError,
            0x06,
            '3' as u8,
            '4' as u8,
            message_data,
        );
        let encoded = msg.encode();
        assert_eq!(encoded.len(), 6);
        assert_eq!(encoded[0], 0x06); // Control byte
        assert_eq!(encoded[1], '3' as u8); // Arm address
        assert_eq!(encoded[2], '4' as u8); // Device address
        assert_eq!(encoded[3], 'A' as u8); // First message byte
        assert_eq!(encoded[4], 'B' as u8); // Second message byte
        assert_eq!(encoded[5], 'C' as u8); // Third message byte
    }

    #[test]
    fn test_new_ack_and_new_error() {
        let ack = XYZMessage::new_ack(1, 2);
        assert_eq!(ack.error_code, ErrorCode::NoError);
        assert_eq!(ack.control, XYZMessage::ACK);
        assert_eq!(ack.arm_address, 1);
        assert_eq!(ack.device_address, 2);
        assert!(ack.message_data.is_empty());

        let err = XYZMessage::new_error(ErrorCode::InvalidDeviceAddress);
        assert_eq!(err.error_code, ErrorCode::InvalidDeviceAddress);
        // other fields should be default (0 or empty)
        assert_eq!(err.control, 0);
        assert_eq!(err.arm_address, 0);
        assert_eq!(err.device_address, 0);
        assert!(err.message_data.is_empty());
    }

    #[test]
    fn test_decode_xyz_to_xyz_command() {
        let message_data: Vec<u8, { CavroMessage::MESSAGE_BUFFER_SIZE }> = Vec::from_slice(&[0x02, 0x4A, 0x36, 0x37, 0x41, 0x42, 0x41, 0x42, 0x03, 0x11]).unwrap();
        let cavro = CavroMessage::decode(message_data).ok().unwrap();
        let xyz = cavro.decode_to_xyz_command();
        assert_eq!(xyz.cmd, "ABAB");
    }

    #[test]
    fn test_encode_empty_message() {
        let msg = PumpCommand::new(
            ErrorCode::NoError,
            'R' as u8,
            0x02,
            Vec::new(),
        );
        let encoded = msg.encode();
        assert_eq!(encoded.len(), 2);
        assert_eq!(encoded[0], 'R' as u8); // Pump address
        assert_eq!(encoded[1], 0x02); // Sequence number
    }

    #[test]
    fn test_pump_command_encode() {
        let message_data: Vec<u8, { CavroMessage::MESSAGE_BUFFER_SIZE }> = Vec::from_slice(&[0x41, 0x42]).unwrap();
        let msg = PumpCommand::new(
            ErrorCode::NoError,
            'P' as u8,
            0x01,
            message_data,
        );
        let encoded = msg.encode();
        assert_eq!(encoded.len(), 4);
        assert_eq!(encoded[0], 'P' as u8); // Pump address
        assert_eq!(encoded[1], 0x01); // Sequence number
        assert_eq!(encoded[2], 0x41); // data
        assert_eq!(encoded[3], 0x42);
    }

    // #[test]
    // fn test_pump_command_decode() {
    //     let message_data: Vec<u8, { CavroMessage::MESSAGE_BUFFER_SIZE }> = Vec::from_slice(&[0x02, 0x13, 0x06, 0x03, 0x11]).unwrap();
    //     let cavro = CavroMessage::decode(message_data);
    //     //let pump = PumpCommand::decode(cavro);
    //     let msg = cavro.decode_message();
    //     assert_eq!(pump.pump_address, 0x13);
    //     assert_eq!(pump.sequence_num, 0x06);
    //     assert_eq!(pump.message_data.len(), 0);
    //     assert_eq!(pump.error_code, ErrorCode::NoError);
    // }

    #[test]
    fn test_decode_to_command() {
        // Test XYZ Command
        let xyz_data: Vec<u8, { CavroMessage::MESSAGE_BUFFER_SIZE }> = Vec::from_slice(&[0x02, 0x41, 0x31, 0x38, 0x4d, 0x31, 0x30, 0x03, 0x05]).unwrap();

        let vrc_calc = calculate_vrc(&xyz_data[0..xyz_data.len()-1]);
        let vrc = xyz_data[xyz_data.len() - 1];
        assert_eq!(vrc, vrc_calc);

        let cavro = CavroMessage::decode(xyz_data).ok().unwrap();
        assert_eq!(cavro.error_code, ErrorCode::NoError);
        assert_eq!(cavro.vrc_calculated, cavro.vrc);
        //assert_eq!(cavro.device_type, CavroDeviceType::XYZ);
        //let msg = XYZMessage::decode(cavro);

        let xyz = cavro.decode_to_xyz_command();
        assert_eq!(xyz.cmd, "M10");
        assert_eq!(xyz.sequence_number, 1);

        // Test Pump Command
        let pump_data: Vec<u8, { CavroMessage::MESSAGE_BUFFER_SIZE }> = Vec::from_slice(&[0x02, 0x41, 0x31, 0x32, 0x01, 0x31, 0x30, 0x03, 0x43]).unwrap();
        let vrc_calc = calculate_vrc(&pump_data[0..pump_data.len()-1]);
        let vrc = pump_data[pump_data.len() - 1];
        assert_eq!(vrc, vrc_calc);

        let cavro = CavroMessage::decode(pump_data).ok().unwrap();
        let pump = cavro.decode_to_pump_command();
        assert_eq!(pump.pump_address, 0x41);
        assert_eq!(pump.sequence_num, 0x31);
        assert_eq!(pump.message_data.as_slice(), &[0x32, 0x01, 0x31, 0x30]);
    }

    #[test]
    fn test_decode_xyz_to_pump_command() {
        let message_data: Vec<u8, { CavroMessage::MESSAGE_BUFFER_SIZE }> = Vec::from_slice(&[
            0x02, 0x44, 0x31, 0x31, 0x4c, 0x31, 0x30, 0x76, 0x35, 0x30, 0x56, 0x31, 0x32, 0x30,
            0x63, 0x35, 0x30, 0x4b, 0x33, 0x41, 0x31, 0x32, 0x52, 0x03, 0x10]).unwrap();
        let _cavro = CavroMessage::decode(message_data);
    }
}
