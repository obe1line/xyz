use crate::components::xyz_message;
pub use xyz_message::{XYZCommand, XYZMessage, ErrorCode, CavroMessage, PumpCommand};

use heapless::{Vec};
use core::default::Default;

#[cfg(feature = "embedded")]
use {
    defmt::{info, error},
};

#[cfg(not(feature = "embedded"))]
use {
    log::{info, error},
};

pub struct CavroMessageParser {
    buffer: Vec::<u8, 255>,
    error_code: ErrorCode,
}

impl CavroMessageParser {
    pub fn version(&self) -> &'static str {
        "0.1.0"
    }
}

impl CavroMessageParser {
    const STX: u8 = 0x02; // Start of Text
    const ETX: u8 = 0x03; // End of Text
    const MIN_MSG_SIZE: usize = 5; // STX, ADR, CTRL, ETX, VRC

    pub fn new() -> Self {
        CavroMessageParser {
            buffer: Vec::new(),
            error_code: ErrorCode::NoError,
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
        if copy_len < byte_count {
            error!("Buffer overflow: only {} of {} bytes added", copy_len, byte_count);
            self.error_code = ErrorCode::MessageBufferOverflow;
        }
        copy_len
    }

    /// Parses the internal buffer and returns a CavroMessage.
    /// Once message data is parsed, it removes the processed bytes from the buffer.
    /// If the buffer does not contain a complete message, it returns an error message.
    /// If the message is successfully parsed, it returns a valid CavroMessage.
    /// If the message is invalid, it returns a CavroMessage with an appropriate error code.
    /// The message is considered valid if it starts with STX, ends with ETX,
    /// and has a valid VRC (Vertical Redundancy Check).
    pub fn parse(&mut self) -> CavroMessage {
        // first check and exit if there is an overflow message from add_data
        let error_code = self.error_code;
        if error_code == ErrorCode::MessageBufferOverflow {
            self.error_code = ErrorCode::NoError;
            return CavroMessage::new_error(error_code);
        }

        // Not enough data for a complete message?
        info!("Buffer length: {}", self.buffer.len());
        if self.buffer.len() < Self::MIN_MSG_SIZE {
            self.error_code = ErrorCode::PartialMessageReceived;
            return CavroMessage::new_error(ErrorCode::PartialMessageReceived,);
        }

        // Check if the buffer contains a complete message
        if self.buffer[0] != Self::STX {
            self.skip_to_next_start_byte();
            self.error_code = ErrorCode::InvalidStartByte;
            return CavroMessage::new_error(ErrorCode::InvalidStartByte);
        }

        let mut cavro: CavroMessage = Default::default();

        // get the message block
        let mut found = false;
        let mut pos = 1;
        while pos < self.buffer.len() {
            let &ch = &self.buffer[pos];
            if ch == Self::ETX {
                found = true;
                break;
            }
            // copy into message
            if cavro.message_data.push(ch).is_err() {
                error!("Message buffer overflow");
                cavro.error_code = ErrorCode::MessageBufferOverflow;
                return cavro;
            }
            // move to the next byte
            pos += 1;
        }
        if !found {
            cavro.error_code = ErrorCode::PartialMessageReceived;
            return cavro;
        }

        pos += 1; // Move to the VRC byte
        if pos >= self.buffer.len() {
            cavro.error_code = ErrorCode::PartialMessageReceived;
            return cavro;
        }
        cavro.vrc = self.buffer[pos]; // checksum byte

        // check that the VRC is correct
        let mut vrc_calc = 0u8;
        for &byte in &self.buffer[0..pos] {
            vrc_calc ^= byte; // Calculate VRC by XORing all bytes
        }
        if vrc_calc != cavro.vrc {
            error!("VRC mismatch: calculated {} but found {}", vrc_calc, cavro.vrc);
            for &byte in self.buffer.iter() {
                error!("Buffer byte: {}", byte);
            }
            cavro.error_code = ErrorCode::VRCMismatch;
            return cavro;
        }

        // Remove the processed message from the buffer
        for _ in 0..pos+1 {
            self.buffer.remove(0);
        }

        cavro
    }

    pub fn skip_to_next_start_byte(&mut self) {
        while self.buffer.len() > 0 && self.buffer[0] != Self::STX {
            // Remove bytes until we find STX or the buffer is empty
            self.buffer.remove(0);
        }
    }
}


//========================================================

#[cfg(test)]
#[cfg(feature = "std")]
mod parser_tests {
    use super::*;

    #[test]
    fn test_parser_version() {
        let parser = CavroMessageParser::new();
        assert_eq!(parser.version(), "0.1.0");
    }

    #[test]
    fn test_parser_empty_buffer() {
        let mut parser = CavroMessageParser::new();
        let msg = parser.parse();
        assert_eq!(msg.error_code, ErrorCode::PartialMessageReceived);
    }

    #[test]
    fn test_parser_invalid_start_byte() {
        let mut parser = CavroMessageParser::new();
        let data = [0x02, 0x01, '1' as u8, '1' as u8, 0x04, 0x05, 0x03, 0x06]; // valid looking but we want to test skip
        // parser.add_data([0x01...])
        let bad_data = [0x01, 0x01];
        parser.add_data(&bad_data, bad_data.len());
        parser.add_data(&data, data.len());
        let msg = parser.parse();
        assert_eq!(msg.error_code, ErrorCode::InvalidStartByte);
    }

    #[test]
    fn test_parser_partial_message() {
        let mut parser = CavroMessageParser::new();
        let data = [0x02, 0x01, '1' as u8, '1' as u8, 0x04, 0x05]; // Missing ETX and VRC
        parser.add_data(&data, data.len());
        let msg = parser.parse();
        assert_eq!(msg.error_code, ErrorCode::PartialMessageReceived);
    }

    #[test]
    fn test_parser_vrc_mismatch() {
        let mut parser = CavroMessageParser::new();
        let data = [0x02, 0x01, '1' as u8, '1' as u8, 0x03, 0x07]; // STX, ADR, CTRL, ETX, VRC
        parser.add_data(&data, data.len());
        let msg = parser.parse();
        assert_eq!(msg.error_code, ErrorCode::VRCMismatch);
    }

    #[test]
    fn test_parser_multiple_messages() {
        let mut parser = CavroMessageParser::new();
        let data1 = [0x02, 0x41, '2' as u8, 0x07, 0x42, 0x43, 0x03, 0x02 ^ 0x07 ^ ('2' as u8) ^ 0x41 ^ 0x42 ^ 0x43 ^ 0x03];
        let data2 = [0x02, 0x06, '4' as u8, 0x41, 0x42, 0x43, 0x03, 0x02 ^ 0x06 ^ ('4' as u8) ^ 0x41 ^ 0x42 ^ 0x43 ^ 0x03];

        parser.add_data(&data1, data1.len());
        parser.add_data(&data2, data2.len());

        let msg1: CavroMessage = parser.parse();
        assert_eq!(msg1.error_code, ErrorCode::NoError);
        let cmd1: XYZMessage = XYZMessage::decode(msg1);
        assert_eq!(cmd1.device_address, 0x07);
        assert_eq!(cmd1.control, 0x41);

        // let msg2 = parser.parse();
        // assert_eq!(msg2.error_code, ErrorCode::NoError);
        // assert_eq!(msg2.address, 0x06);
        // assert_eq!(msg2.control, '4' as u8);
    }

    #[test]
    fn test_clear_buffer_and_add_data_overflow_count() {
        let mut parser = CavroMessageParser::new();
        let large = [0xFFu8; 200];
        let added1 = parser.add_data(&large, large.len());
        assert_eq!(added1, 200);
        assert_eq!(parser.buffer.len(), 200);
        // now add 100 more bytes but buffer has only 55 capacity left
        let more = [0xAAu8; 100];
        let added2 = parser.add_data(&more, more.len());
        assert_eq!(added2, 55);
        assert_eq!(parser.buffer.len(), 255);
        // clear and ensure empty
        parser.clear_buffer();
        assert_eq!(parser.buffer.len(), 0);
    }

    #[test]
    fn test_add_data_with_custom_count() {
        let mut parser = CavroMessageParser::new();
        let data = [1,2,3,4,5,6,7,8,9,10];
        let added = parser.add_data(&data, 5);
        assert_eq!(added, 5);
        assert_eq!(&parser.buffer[..], &[1,2,3,4,5]);
    }

    // #[test]
    // fn test_skip_to_next_start_byte_behavior() {
    //     let mut parser = CavroMessageParser::new();
    //     // Junk bytes before STX
    //     parser.add_data(&[0x00, 0xFF, 0x01, 0x7E], 4);
    //     // Now add a minimal valid message: STX, addr=0x00, ctrl='1', ETX, VRC
    //     let mut msg = vec![0x02, 0x00, '1' as u8, 0x03];
    //     let vrc = msg.as_slice().iter().fold(0u8, |acc, b| acc ^ b);
    //     msg.push(vrc);
    //     let msg = msg.as_slice();
    //     parser.add_data(&msg, msg.len());
    //     // First parse returns InvalidStartByte but trims buffer to STX
    //     let first = parser.parse();
    //     assert_eq!(first.error_code, ErrorCode::InvalidStartByte);
    //     // Second parse should parse the valid message now at the front
    //     let out = parser.parse();
    //     assert_eq!(out.error_code, ErrorCode::NoError);
    //     assert_eq!(out.address, 0x00);
    //     assert_eq!(out.control, '1' as u8);
    //     // buffer should be empty after parsing
    //     let out2 = parser.parse();
    //     assert_eq!(out2.error_code, ErrorCode::PartialMessageReceived);
    // }

    // #[test]
    // fn test_parser_success_min_message_and_invalid_device() {
    //     let mut parser = CavroMessageParser::new();
    //     // First, a minimal valid message: STX, ADR, CTRL, ETX, VRC
    //     let mut pkt1 = vec![0x02, 0x32, 0x39, 0x03];
    //     let vrc1 = pkt1.as_slice().iter().fold(0u8, |acc, b| acc ^ b);
    //     pkt1.push(vrc1);
    //     let pkt1 = pkt1.as_slice();
    //     parser.add_data(&pkt1, pkt1.len());
    //     let ok = parser.parse();
    //     assert_eq!(ok.error_code, ErrorCode::NoError);
    //     assert_eq!(ok.address, 0x32);
    //     assert_eq!(ok.control, 0x39);
    // }

    #[test]
    fn test_parser_message_buffer_overflow() {
        let mut parser = CavroMessageParser::new();
        // Construct a packet with more data bytes than the (test) MESSAGE_BUFFER_SIZE (=32)
        let mut data = vec![0x02, 0x00, '1' as u8];
        // Push many message bytes so that push into message_data overflows
        data.extend_from_slice(&[0x55u8; 256]); // Exceed any buffer
        // Add ETX so the loop would normally stop, but we should overflow before reaching it
        data.push(0x03);
        // No VRC provided because we expect early return due to overflow
        let bytes_added =parser.add_data(&data.as_slice(), data.len());
        // We expect only 255 bytes to be added to the buffer
        assert_eq!(bytes_added, 255);
        // parse should fail since the add_data failed to add all bytes
        let msg = parser.parse();
        assert_eq!(msg.error_code, ErrorCode::MessageBufferOverflow);
    }

    #[test]
    fn test_parser_missing_vrc_after_etx() {
        let mut parser = CavroMessageParser::new();
        // STX, addr, ctrl, data, ETX, but NO VRC
        let data = [0x02, 0x00, '1' as u8, 0x41, 0x03];
        parser.add_data(&data, data.len());
        let msg = parser.parse();
        // Should reach the pos >= len path and return PartialMessageReceived
        assert_eq!(msg.error_code, ErrorCode::PartialMessageReceived);
    }

    // #[test]
    // fn test_parser_command_validation() {
    //     // construct an XYZ_Message with a command
    //     let mut cmd = Vec::<u8, { CavroMessage::MESSAGE_BUFFER_SIZE }>::new();
    //     for &b in b"PA 300 500 700" {
    //         cmd.push(b).unwrap();
    //     }
    //     let msg = XYZMessage::new(
    //         ErrorCode::NoError,
    //         0x07,
    //         '1' as u8,
    //         '1' as u8,
    //         cmd,
    //     );
    //
    //     let mut parser = CavroMessageParser::new();
    //     let data = msg.encode();
    //     let data = data.as_slice();
    //     parser.add_data(data, data.len());
    //     let cavro = parser.parse();
    //     assert_eq!(cavro.error_code, ErrorCode::NoError);
    //     assert_eq!(cavro.address, 0x07);
    //     assert_eq!(cavro.control, '1' as u8); // In XYZ encode, address is control, control is arm_adr, device_adr is next... wait.
    // }

    #[test]
    fn test_parser_command_extraction() {
        let message_data: heapless::Vec<u8, { CavroMessage::MESSAGE_BUFFER_SIZE }> = heapless::Vec::from_slice(
            &[0x02, 0x41, 0x42, 0x43,
                'P' as u8, 'A' as u8, ' ' as u8,
                '3' as u8, '0' as u8, '0' as u8, ' ' as u8,
                '5' as u8, '0' as u8, '0' as u8, ' ' as u8,
                '7' as u8, '0' as u8, '0' as u8, 0x03, 0x41]).unwrap();

        let msg = CavroMessage::decode(message_data);
        let xyz = XYZMessage::decode(msg);
        let command = XYZCommand::decode(xyz);
        assert_eq!(command.error_code, ErrorCode::NoError);
        assert_eq!(command.cmd, "PA");
        assert_eq!(command.num_params, 3);
        assert_eq!(command.get_param(0), Some(300));
        assert_eq!(command.get_param(1), Some(500));
        assert_eq!(command.get_param(2), Some(700));
    }

    // #[test]
    // fn test_command_parameter_encoding_and_decoding() {
    //     let mut cmd_string = heapless::String::new();
    //     cmd_string.insert_str(0, "PR").ok();
    //     let command = XYZCommand {
    //         error_code: ErrorCode::NoError,
    //         cmd: cmd_string,
    //         num_params: 2,
    //         params: [123, -456, 0, 0],
    //     };
    //     let encoded = command.encode();
    //     assert_eq!(&encoded[..], b"PR 123 -456");
    //
    //     let command_decoded = XYZCommand::decode(encoded.as_slice()).unwrap();
    //     assert_eq!(command_decoded.cmd, "PR");
    //     assert_eq!(command_decoded.num_params, 2);
    //     assert_eq!(command_decoded.get_param(0), Some(123));
    //     assert_eq!(command_decoded.get_param(1), Some(-456));
    // }

    // #[test]
    // fn test_parser_decode_pump_command() {
    //     let mut parser = CavroMessageParser::new();
    //     let mut data: heapless::Vec<u8, 32> = heapless::Vec::new();
    //     // STX, ADR, SEQ, 'L', '1', '0', ETX, VRC
    //     data.extend_from_slice(&[0x02, 0x31, 0x01, 0x4c, 0x31, 0x30, 0x03]).unwrap();
    //     let vrc = data.as_slice().iter().fold(0u8, |acc, b| acc ^ b);
    //     data.push(vrc).unwrap();
    //
    //     parser.add_data(&data.as_slice(), data.len());
    //     let cavro = parser.parse();
    //     assert_eq!(cavro.error_code, ErrorCode::NoError);
    //     assert_eq!(cavro.address, 0x31);
    //     assert_eq!(cavro.control, 0x01);
    //     assert_eq!(cavro.message_data.as_slice(), b"L10");
    // }
}