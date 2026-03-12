#[cfg(feature = "embedded")]
use {
    defmt::{error, info},
    embassy_stm32::mode::Async,
    embassy_stm32::usart::UartRx,
    embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex,
    embassy_sync::channel::Sender,
    crate::components::xyz_parser::CavroMessageParser,
    crate::components::xyz_message::{CavroMessage, XYZMessage},
};

#[cfg(feature = "embedded")]
pub async fn common_message_receiver<const IN_SIZE: usize, const OUT_SIZE: usize>(
    name: &str,
    usart_rx: UartRx<'static, Async>,
    processing: Sender<'static, CriticalSectionRawMutex, CavroMessage, IN_SIZE>,
    filter_acks: bool,
    outgoing_ack_target: Option<Sender<'static, CriticalSectionRawMutex, CavroMessage, OUT_SIZE>>,
    outgoing_msg_target: Option<Sender<'static, CriticalSectionRawMutex, CavroMessage, OUT_SIZE>>,
) -> ! {
    let mut rx_dma_buf = [0u8; 256];
    let mut ring_rx = usart_rx.into_ring_buffered(&mut rx_dma_buf);
    let mut buffer = [0u8; 64];
    let mut parser = CavroMessageParser::new();

    loop {
        info!("{}: Reading from UART", name);
        let n: usize = match ring_rx.read(&mut buffer).await {
            Ok(n) => n,
            Err(_e) => {
                error!("{}: Error reading from UART", name);
                continue;
            }
        };

        if n > 0 {
            info!("{}: Read {} bytes from UART", name, n);
            parser.add_data(&buffer, n);
            loop {
                let cavro = parser.parse();
                if cavro.error_code == crate::components::xyz_message::ErrorCode::PartialMessageReceived {
                    break;
                }

                if cavro.error_code == crate::components::xyz_message::ErrorCode::NoError {
                    let xyz = XYZMessage::decode(cavro.message_data.clone());
                    if xyz.control == XYZMessage::ACK {
                        if filter_acks {
                            info!("{}: Received ACK message - ignoring", name);
                            continue;
                        }
                    }

                    if xyz.control != XYZMessage::ACK {
                        if let Some(ref ack_target) = outgoing_ack_target {
                            // we have an answer - send ack to the source
                            info!("{}: Queuing ACK for source", name);
                            let ack_xyz = XYZMessage::new_ack(xyz.arm_address, xyz.device_address);
                            let ack_message = CavroMessage::new(ack_xyz.encode());
                            if let Err(e) = ack_target.try_send(ack_message) {
                                info!("{}: Failed to send ACK to channel: {:?}", name, e);
                            }
                        }
                        if let Some(ref msg_target) = outgoing_msg_target {
                            // and pass the answer to the other target
                            info!("{}: Queuing message for other target", name);
                            let up_message = CavroMessage::new(xyz.encode());
                            if let Err(e) = msg_target.try_send(up_message) {
                                info!("{}: Failed to send message to target channel: {:?}", name, e);
                            }
                        }
                    }

                    info!("{}: Queuing message for processing", name);
                    let cavro_msg = CavroMessage::new(xyz.encode());
                    if let Err(e) = processing.try_send(cavro_msg) {
                        info!("{}: Failed to send message to processing channel: {:?}", name, e);
                    }
                } else {
                    info!("{}: Message parser error code: {:?}", name, cavro.error_code);
                    parser.clear_buffer();
                    break;
                }
            }
        }
    }
}
