#![cfg_attr(feature="embedded", no_std)]
pub mod components;
pub use crate::components::xyz_parser::{CavroMessageParser};
pub use crate::components::xyz_message::{XYZMessage, XYZCommand, ErrorCode, CavroMessage, PumpCommand};
#[cfg(feature = "embedded")]
pub use crate::components::receiver::{common_message_receiver};

use core::*;

#[cfg(feature = "embedded")]
use {
    defmt::error,
};

#[cfg(not(feature = "embedded"))]
use {
    log::error,
};

// TODO: Add ability to handle transactions with data from uart, and passthrough to the second uart
// Test by using 2 tasks sending messages to each other.

// TODO: wrap the channel in a struct so std and no_std have the same api
// static UART2_CHANNEL: Channel<CriticalSectionRawMutex, XYZMessage, 1> = Channel::new();

pub struct XYZRobot {
    // shutdown_mutex: Arc<(Mutex<bool>, Condvar)>,
    // raw_data_tx: Sender<u8>,
    // raw_data_rx: Receiver<u8>,
    parser_xyz: CavroMessageParser,
}

impl XYZRobot {
    pub fn new() -> Self {
        // let (raw_data_tx, raw_data_rx) = channel::<u8>();
        XYZRobot {
            // shutdown_mutex: Arc::new((Mutex::new(true), Condvar::new())),
            // raw_data_tx,
            // raw_data_rx,
            parser_xyz: CavroMessageParser::new(),
        }
    }

    pub fn version(&self) -> &'static str {
        self.parser_xyz.version()
    }

    pub fn add_data(&mut self, data: &[u8], count: usize) {
        self.parser_xyz.add_data(data, count);
    }

    pub fn process_next_message(&mut self) -> CavroMessage {
        // Check if there is more data
        let msg = self.parser_xyz.parse();
        match msg.error_code {
            ErrorCode::NoError => {
                // Handle successful message
                // info!("Received full message: {:?}", msg);
                msg
            },
            ErrorCode::PartialMessageReceived => {
                // go back for more data
                msg
            }
            _ => {
                // Handle error
                error!("Error in message: {:?}", msg.error_code);
                msg
            }
        }
    }
}

#[cfg(test)]
mod library_tests {
    use super::*;

    #[test]
    fn parser_version_is_correct() {
        let parser = XYZRobot::new();
        assert_eq!(parser.version(), "0.1.0");
    }

    #[test]
    fn parser_add_data() {
        let mut robot = XYZRobot::new();

        let mut data1a: heapless::Vec<u8, 16> = heapless::Vec::new();
        let mut data1b: heapless::Vec<u8, 16> = heapless::Vec::new();
        let mut data2a: heapless::Vec<u8, 16> = heapless::Vec::new();
        let mut data2b: heapless::Vec<u8, 16> = heapless::Vec::new();
        data1a.extend_from_slice(&[0x02, 0x07, '1' as u8, '2' as u8]).unwrap();
        data1b.extend_from_slice(&[0x41, 0x42, 0x43, 0x03, 0x45]).unwrap();
        data2a.extend_from_slice(&[0x02, 0x06, '3' as u8, '4' as u8]).unwrap();
        data2b.extend_from_slice(&[0x41, 0x42, 0x43, 0x03, 0x40]).unwrap();

        robot.add_data(&data1a, data1a.len());
        let msg = robot.process_next_message();
        assert_eq!(msg.error_code, ErrorCode::PartialMessageReceived);

        robot.add_data(&data1b, data1b.len());
        let msg = robot.process_next_message();
        assert_eq!(msg.error_code, ErrorCode::NoError);

        robot.add_data(&data2a, data2a.len());
        let msg = robot.process_next_message();
        assert_eq!(msg.error_code, ErrorCode::PartialMessageReceived);

        robot.add_data(&data2b, data2b.len());
        let msg = robot.process_next_message();
        assert_eq!(msg.error_code, ErrorCode::NoError);
    }
}
