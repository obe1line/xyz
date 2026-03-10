#![cfg_attr(feature="embedded", no_std, no_main)]

use xyz_motor::{MotorController, StepperController, PowerStepControl, MOTOR_DIR_BACKWARD, MOTOR_DIR_FORWARD, MotorDirection};
use xyz_parser::{CavroMessage, CavroMessageParser, XYZMessage, XYZCommand, PumpCommand};
use fixed::types::extra::U3;

#[cfg(feature = "embedded")]
use {
    defmt::{error, info, warn},
    embassy_executor::Spawner,
    embassy_stm32::gpio::{Level, Input, Output, Pull, Speed},
    embassy_stm32::{bind_interrupts, peripherals, Peripherals, usart},
    embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex,
    embassy_time::{Timer, Duration, block_for},
    embassy_sync::channel::Channel,
    embassy_stm32::mode::Async,
    embassy_stm32::usart::{UartTx, UartRx},
    embassy_stm32::spi::Spi,
    embassy_stm32::mode::Blocking,
    embassy_stm32::spi::mode::Master,
    heapless::Vec,
    panic_probe as _,
};

#[cfg(feature = "embedded-rtt")]
use defmt_rtt as _;

#[cfg(feature = "embedded-serial")]
use defmt_serial as _;

#[cfg(not(feature = "embedded"))]
use {
    log::{error, info},
    xyz_parser::{XYZMessageParser, XYZMessage},
    xyz_motor::{Motor, PowerStepControl},
};

const X_MOTOR_DEVICE_ID: u8 = 0;

#[cfg(feature = "embedded")]
#[embassy_executor::task]
async fn heartbeat_task(mut led: Output<'static>) -> ! {
    loop {
        led.toggle();
        Timer::after(Duration::from_millis(1000)).await;
    }
}

fn uart_config() -> usart::Config {
    let mut config = usart::Config::default();
    config.baudrate = 9600;
    config.data_bits = usart::DataBits::DataBits8;
    config.parity = usart::Parity::ParityNone;
    config.stop_bits = usart::StopBits::STOP1;
    config
}

// max message size for message channels
const IN_CHANNEL_MSG_SIZE: usize = 8;
const OUT_CHANNEL_MSG_SIZE: usize = 8;

static IN_UPSTREAM_MSG_CHANNEL: Channel<CriticalSectionRawMutex, CavroMessage, IN_CHANNEL_MSG_SIZE> = Channel::new();
static OUT_UPSTREAM_MSG_CHANNEL: Channel<CriticalSectionRawMutex, CavroMessage, OUT_CHANNEL_MSG_SIZE> = Channel::new();

// static IN_PUMP_MSG_CHANNEL: Channel<CriticalSectionRawMutex, CavroMessage, OUT_CHANNEL_MSG_SIZE> = Channel::new();
static OUT_PUMP_MSG_CHANNEL: Channel<CriticalSectionRawMutex, CavroMessage, OUT_CHANNEL_MSG_SIZE> = Channel::new();

static IN_DOWNSTREAM_MSG_CHANNEL: Channel<CriticalSectionRawMutex, CavroMessage, IN_CHANNEL_MSG_SIZE> = Channel::new();
static OUT_DOWNSTREAM_MSG_CHANNEL: Channel<CriticalSectionRawMutex, CavroMessage, OUT_CHANNEL_MSG_SIZE> = Channel::new();

// static ACK_SIGNAL: embassy_sync::signal::Signal<CriticalSectionRawMutex, bool> = embassy_sync::signal::Signal::new();
// static ANSWER_SIGNAL: embassy_sync::signal::Signal<CriticalSectionRawMutex, bool> = embassy_sync::signal::Signal::new();

macro_rules! validate_params{
    ($params_len:expr, $expected_len:expr) => {
        if $params_len != $expected_len {
            error!("Invalid parameters for command");
            // send back an error
            continue;
        }
    };
}

macro_rules! validate_position{
    ($x:expr, $y:expr, $z:expr) => {
        if $x > 10000u32 || $y > 10000u32 || $z > 2000u32 {
            error!("Invalid position: {}, {}, {}", $x, $y, $z);
            // send back an error
            continue;
        }
    };
}

// #[cfg(feature = "embedded")]
// #[embassy_executor::task]
// async fn downstream_message_handler_task() -> ! {
//     let receiver = IN_DOWNSTREAM_MSG_CHANNEL.receiver();
//
//     loop {
//         let message = receiver.receive().await;
//         if message.control == XYZMessage::ACK {
//             info!("Received an ack from downstream channel - not passing upstream");
//         }
//         else {
//             info!("Received an answer from the downstream channel - send ack only for now, no passthrough upstream");
//             let ack_message = XYZMessage::new_ack(message.arm_adr, message.device_adr);
//             OUT_DOWNSTREAM_MSG_CHANNEL.sender().try_send(ack_message).unwrap();
//
//             // TODO: only pass upstream if it's not an internal command - how do we know? control bits that are unused?
//             // OUT_UPSTREAM_MSG_CHANNEL.sender().try_send(message).unwrap();
//         }
//     }
// }

#[cfg(feature = "embedded")]
#[embassy_executor::task]
async fn upstream_message_handler_task(power_step_controller: &'static mut PowerStepControl<'static, XYZStepperController>,
                                       motor_home: Input<'static>,
                                       motor_status_24v: Input<'static>,
                                       pump_outgoing: embassy_sync::channel::Sender<'static, CriticalSectionRawMutex, CavroMessage, IN_CHANNEL_MSG_SIZE>,
                                       upstream_outgoing: embassy_sync::channel::Sender<'static, CriticalSectionRawMutex, CavroMessage, IN_CHANNEL_MSG_SIZE>) -> ! {
    let receiver = IN_UPSTREAM_MSG_CHANNEL.receiver();

    /// position wrapper using Cell to allow interior mutability
    /// will add functionality to convert between -ve coordinate space later
    /// Origin at 0,0,0 is this robot's absolute home position but the instrument's home
    /// position is at some offset from this.
    use core::cell::Cell;
    struct Position {
        x: Cell<u32>,
        // y: Cell<u32>,
        // z: Cell<u32>,
    }
    impl Position {
        fn new(x: u32, _y: u32, _z: u32) -> Self {
            Self {
                x: Cell::new(x),
                // y: Cell::new(_y),
                // z: Cell::new(_z),
            }
        }
    }

    let current_position = Position::new(0, 0, 0);

    // wait for downstream board to send an ack and answer
    // TODO: validate it matches the command we sent
    async fn wait_for_ack_and_answer(receiver: embassy_sync::channel::Receiver<'static, CriticalSectionRawMutex, CavroMessage, OUT_CHANNEL_MSG_SIZE>,
                                     delay_ms: u64,
                                     timeout: u16) -> bool {
        info!("Waiting for YZ board to respond to command...");
        // wait for a message from YZ board
        let mut retry_count = timeout;
        while receiver.is_empty() && retry_count > 0 {
            Timer::after_millis(delay_ms).await;
            retry_count -= 1;
        }
        // timeout?
        if retry_count == 0 {
            error!("Timeout waiting for YZ ack for command");
            return false;
        }
        else {
            let cavro = receiver.receive().await;
            let ack = XYZMessage::decode(cavro.message_data.clone());
            if ack.control == XYZMessage::ACK {
                info!("Received ack from YZ for command: {:?}", ack);
            }
            else {
                warn!("Received a msg when expecting ack from YZ for command: {:?}", ack);
            }
        }

        // wait for a message from YZ board
        let mut retry_count = timeout;
        while receiver.is_empty() && retry_count > 0 {
            Timer::after_millis(delay_ms).await;
            retry_count -= 1;
        }
        // timeout?
        if retry_count == 0 {
            error!("Timeout waiting for YZ answer for PA command");
            return false;
        }
        else {
            let cavro = receiver.receive().await;
            let answer = XYZMessage::decode(cavro.message_data.clone());
            info!("Received answer from YZ for PA command: {:?}", answer);
            info!("Sending ack to answer from YZ board");
            let ack_xyz = XYZMessage::new_ack(answer.arm_address, answer.device_address);
            let ack = CavroMessage::new(ack_xyz.encode());
            OUT_DOWNSTREAM_MSG_CHANNEL.sender().try_send(ack).unwrap();
        }

        true
    }

    let mut motor_needs_reset = false;
    loop {
        let cavro = receiver.receive().await;
        let xyz = XYZMessage::decode(cavro.message_data.clone());

        // send ack upstream as we are now processing the message content
        let ack_xyz = XYZMessage::new_ack(xyz.arm_address, xyz.device_address);
        let ack_message = CavroMessage::new(ack_xyz.encode());
        info!("Sending upstream ACK data: {:?}", ack_message.encode());
        let result = upstream_outgoing.try_send(ack_message);
        if result.is_err() {
            error!("Error sending ACK upstream")
        } else {
            info!("Sending ACK upstream");
        }

        // pump command?
        if xyz.is_pump_device() {
            info!("Received pump message from upstream");
            let pump_command = PumpCommand::decode_from_xyz(xyz);
            // send to the pump channel
            let pump_cavro_message = CavroMessage::new(pump_command.encode());
            pump_outgoing.try_send(pump_cavro_message).unwrap();
            continue;
        }

        // the upstream port is always xyz messages
        let command = XYZCommand::decode(xyz);
        // TODO: check for errors in decoding

        if command.cmd.len() == 0 {
            error!("No command to run - ignoring");
            continue;
        }

        info!("Motor 24V power: {}", motor_status_24v.is_high());
        if !motor_status_24v.is_high() {
            error!("Motor power is off - cannot process command");
            motor_needs_reset = true;
            continue;
        }

        // has the motor power just come back on?
        if motor_needs_reset {
            info!("Motor power has been restored - resetting motor controller");
            power_step_controller.initialize();
            motor_needs_reset = false;
        }

        // TODO: this is a common function - refactor later
        fn wait_for_motor(
            motor_id: u8,
            power_step_controller: &mut PowerStepControl<'static, XYZStepperController>,
        ) {
            info!("Waiting for motor to be ready...");
            let mut timeout_ms = 5000;
            while power_step_controller.Powerstep01_IsDeviceBusy(motor_id) {
                block_for(Duration::from_millis(100));
                timeout_ms -= 100;
                if timeout_ms == 0 {
                    error!("Timeout waiting for motor to be ready");
                    break;
                }
            }
            info!("Motor is ready");
        }

        let xyz = XYZMessage::decode(cavro.message_data.clone());
        info!("Processing command: {}, params: {:?}", command.cmd.as_str(), command.params);
        match command.cmd.as_str() {
            "RV" => {
                validate_params!(command.num_params, 0);
                let xyz_response = XYZMessage::create_answer(&xyz, Vec::from_slice(b"242").unwrap(), 0);
                let response_message = CavroMessage::new(xyz_response.encode());
                let result = upstream_outgoing.try_send(response_message);
                if result.is_err() {
                    error!("Error sending RV response");
                }
            }
            // "&" => {
            //     // Pump version command
            //     let xyz_response = XYZMessage::create_answer(&xyz, Vec::from_slice(b"XL-3000 Hi Res 8 port April 1996 P/N 726950_B").unwrap(), 0);
            //     let response_message = CavroMessage::new(xyz_response.encode());
            //     OUT_UPSTREAM_MSG_CHANNEL.sender().try_send(response_message).unwrap();
            // }
            "PA" => {
                wait_for_motor(X_MOTOR_DEVICE_ID, power_step_controller);
                // TODO: set control bit 7 to indicate an internal command where the response is not sent upstream
                // E.g. PA command sent from X board to YZ board to move Y and Z only, response is not sent upstream as the X move has to be done last.
                validate_params!(command.num_params, 3);
                let x_pos: u32 = command.params[0] as u32;
                let y_pos: u32 = command.params[1] as u32;
                let z_pos: u32 = command.params[2] as u32;
                validate_position!(x_pos, y_pos, z_pos);
                info!("Set absolute position: x={}, y={}, z={}", x_pos, y_pos, z_pos);
                // move y and z first by sending to YZ board
                let xyz_pi = XYZMessage::new(
                    xyz_parser::ErrorCode::NoError,
                    xyz.control,
                    b'1',
                    b'2',
                    Vec::from_slice(command.encode().as_slice()).unwrap(),
                );
                let pi_message = CavroMessage::new(xyz_pi.encode());
                OUT_DOWNSTREAM_MSG_CHANNEL.sender().try_send(pi_message).unwrap();

                let yz_receiver = IN_DOWNSTREAM_MSG_CHANNEL.receiver();
                wait_for_ack_and_answer(yz_receiver, 10, 50).await;

                // move x
                let dir: MotorDirection;
                let x_steps: u32;
                if x_pos < current_position.x.get() {
                    dir = MOTOR_DIR_BACKWARD;
                    x_steps = current_position.x.get() - x_pos;
                }
                else {
                    dir = MOTOR_DIR_FORWARD;
                    x_steps = x_pos - current_position.x.get();
                }

                if x_steps > 0 {
                    info!("Moving X motor: {} steps, dir {}", x_steps, dir);
                    power_step_controller.Powerstep01_CmdMove(X_MOTOR_DEVICE_ID, dir, x_steps).ok();
                    current_position.x.set(x_pos);
                    info!("New X position: {}", current_position.x.get());
                }

                // TODO: blank response for now
                let response_xyz = XYZMessage::create_answer(&xyz, Vec::new(), 0);
                let response_message = CavroMessage::new(response_xyz.encode());
                OUT_UPSTREAM_MSG_CHANNEL.sender().try_send(response_message).unwrap();
            }
            "PI" | "XI" => {
                wait_for_motor(X_MOTOR_DEVICE_ID, power_step_controller);
                if command.cmd == "PI" {
                    // initialise all axis to zero position - Z first, then Y, then X
                    let response_xyz = XYZMessage::new(
                        xyz_parser::ErrorCode::NoError,
                        xyz.control,
                        // message.control | 0x80, // mark as internal command TODO: move to field in XYZMessage
                        '1' as u8,
                        '2' as u8,
                        Vec::from_slice(b"PI").unwrap(),
                    );
                    let response_message = CavroMessage::new(response_xyz.encode());
                    OUT_DOWNSTREAM_MSG_CHANNEL.sender().try_send(response_message).unwrap();
                    // TODO: Should X keep track of y and z? Currently X board has no knowledge of y and z position

                    // // wait for the ack and response signals
                    // let ack_received = ACK_SIGNAL.wait().await;
                    // info!("ACK received for PI command: {}", ack_received);
                    // let answer_received = ANSWER_SIGNAL.wait().await;
                    // info!("Answer received for PI command: {}", answer_received);
                }

                // TODO: blank response for now
                let response_xyz = XYZMessage::create_answer(&xyz, Vec::new(), 0);
                let response_message = CavroMessage::new(response_xyz.encode());
                OUT_UPSTREAM_MSG_CHANNEL.sender().try_send(response_message).unwrap();

                let mut cmd_speed = 400; // default speed if not provided
                if command.num_params == 1 {
                    // validate params: 5-400 or 5-800 for Y/Z
                    let param_speed = command.params[0] as u32;
                    if !(5..=800).contains(&param_speed) {
                        error!("Invalid speed parameter for XI/PI command. Falling back to default: {}", param_speed);
                        // TODO: send back an error
                        // continue;
                    }
                    else {
                        cmd_speed = param_speed;
                    }
                }

                //==============================
                // initialize X motor
                //==============================
                info!("Initialize X motor home position.");
                use fixed::FixedU32;
                let fixed_speed: FixedU32<U3> = FixedU32::const_from_int(cmd_speed);
                let speed = fixed_speed.to_bits();        // this is steps per sec in fixed point 0:28
                info!("Setting speed {} as {}", cmd_speed, speed);
                power_step_controller.Powerstep01_CmdGoUntil(X_MOTOR_DEVICE_ID, MOTOR_DIR_BACKWARD, speed).expect("TODO: panic message");
                // TODO: better to use with_timeout from embassy_time. For now, simple counter
                let mut counter = 0;
                loop {
                    // TODO: Error case. What if it never hits the home switch?
                    // simple counter timeout for 200 * 100ms = 20 seconds
                    if counter > 200 {
                        error!("Timeout waiting for X motor to hit home switch");
                        break;
                    }

                    let at_home = motor_home.is_low();
                    info!("Motor 1 at_home: {}", at_home);
                    if at_home {
                        info!("Motor 1 has triggered home switch");
                        // set the position to 0 in the power step chip
                        // this is done automatically when the CmdReleaseSw is called with ACT=0 (default)
                        // power_step_controller.Powerstep01_ResetPos(X_MOTOR_DEVICE_ID).ok();
                        // set the current x position to 0 as we are at true home
                        current_position.x.set(0);
                        info!("Motor 1 is moving out of switch");
                        // move away from the home position slightly
                        power_step_controller.Powerstep01_CmdReleaseSw(X_MOTOR_DEVICE_ID, MOTOR_DIR_FORWARD).expect("TODO: panic message");
                        // power_step_controller.Powerstep01_CmdMove(X_MOTOR_DEVICE_ID, MOTOR_DIR_FORWARD, 100).expect("TODO: panic message");
                        break;
                    }

                    info!("Motor 1 not at home position, waiting 100ms");
                    Timer::after_millis(100).await;
                    counter += 1;
                }

                // TODO: blank response for now
                let response_xyz = XYZMessage::create_answer(&xyz, Vec::new(), 0);
                let response_message = CavroMessage::new(response_xyz.encode());
                OUT_UPSTREAM_MSG_CHANNEL.sender().try_send(response_message).unwrap();
            }
            "XR" => {
                wait_for_motor(X_MOTOR_DEVICE_ID, power_step_controller);
                validate_params!(command.num_params, 1);

                let steps_and_dir: i32 = command.params[0];
                let steps: u32 = steps_and_dir.unsigned_abs();
                let dir = if steps_and_dir < 0 { MOTOR_DIR_BACKWARD } else { MOTOR_DIR_FORWARD };
                info!("Moving X motor: {} steps, dir {}", steps, dir);
                power_step_controller.Powerstep01_CmdMove(X_MOTOR_DEVICE_ID, dir, steps).ok();
                // update current position
                if dir == MOTOR_DIR_BACKWARD {
                    current_position.x.set(current_position.x.get().wrapping_sub(steps));
                } else {
                    current_position.x.set(current_position.x.get().wrapping_add(steps));
                }
                info!("New X position: {}", current_position.x.get());

                // TODO: blank response for now
                let response_xyz = XYZMessage::create_answer(&xyz, Vec::new(), 0);
                let response_message = CavroMessage::new(response_xyz.encode());
                OUT_UPSTREAM_MSG_CHANNEL.sender().try_send(response_message).unwrap();
            },
            "YR" | "ZR" | "YI" | "ZI"
            => {
                info!("YR/ZR/YI/ZI command - pass to YZ board");
                let message = CavroMessage::new(xyz.encode());
                OUT_DOWNSTREAM_MSG_CHANNEL.sender().try_send(message).unwrap();
                // ack and response will be sent by the downstream handler
            },
            _ => {
                info!("Unknown command: {}", command.cmd.as_str());
                info!("HACK - sending response regardless");
                // TODO: blank response for now
                let response_xyz = XYZMessage::create_answer(&xyz, Vec::new(), 0);
                let response_message = CavroMessage::new(response_xyz.encode());
                OUT_UPSTREAM_MSG_CHANNEL.sender().try_send(response_message).unwrap();
            },
        }
    }
}

#[cfg(feature = "embedded")]
#[embassy_executor::task(pool_size = 1)]
async fn upstream_message_receiver(
    usart_rx: UartRx<'static, Async>,
    processing: embassy_sync::channel::Sender<'static, CriticalSectionRawMutex, CavroMessage, IN_CHANNEL_MSG_SIZE>,
) -> ! {
    let mut rx_dma_buf = [0u8; 256];
    let mut ring_rx = usart_rx.into_ring_buffered(&mut rx_dma_buf);
    let mut buffer = [0u8; 64];
    let mut parser = CavroMessageParser::new();

    loop {
        info!("Reading from upstream UART");
        let n: usize = match ring_rx.read(&mut buffer).await {
            Ok(n) => {n}
            Err(_e) => {
                error!("Error reading from upstream UART");
                continue
            }
        };
        if n > 0 {
            info!("Read {} bytes from upstream UART", n);
            // dump the buffer
            info!("{}", buffer[0..n]);
            parser.add_data(&buffer, n);
            let cavro = parser.parse();
            info!("Cavro parser error code: {:?}", cavro.error_code);
            if cavro.error_code == xyz_parser::ErrorCode::NoError {
                let message = XYZMessage::decode(cavro.message_data.clone());
                if message.control == XYZMessage::ACK {
                    info!("Received ACK message on upstream - ignoring for now");
                    continue;
                }
                
                // process the message in another task, which sends an ack and response when done
                info!("Queuing upstream message for processing");
                let cavro_message = CavroMessage::new(message.encode());
                if let Err(e) = processing.try_send(cavro_message) {
                    info!("Failed to send message to channel: {:?}", e);
                }
            }
        }
    }
}

#[cfg(feature = "embedded")]
#[embassy_executor::task(pool_size = 1)]
async fn downstream_message_receiver(
    usart_rx: UartRx<'static, Async>,
    // processing: embassy_sync::channel::Sender<'static, CriticalSectionRawMutex, XYZMessage, IN_CHANNEL_MSG_SIZE>,
    outgoing_downstream: embassy_sync::channel::Sender<'static, CriticalSectionRawMutex, CavroMessage, IN_CHANNEL_MSG_SIZE>,
    outgoing_upstream: embassy_sync::channel::Sender<'static, CriticalSectionRawMutex, CavroMessage, IN_CHANNEL_MSG_SIZE>,
) -> ! {
    let mut rx_dma_buf = [0u8; 256];
    let mut ring_rx = usart_rx.into_ring_buffered(&mut rx_dma_buf);
    let mut buffer = [0u8; 64];
    let mut parser = CavroMessageParser::new();

    loop {
        info!("Reading from downstream UART");
        let n: usize = match ring_rx.read(&mut buffer).await {
            Ok(n) => {n}
            Err(_e) => {
                error!("Error reading from downstream UART");
                continue
            }
        };
        if n > 0 {
            info!("Read {} bytes from downstream UART", n);
            // dump the buffer
            info!("{}", buffer[0..n]);
            parser.add_data(&buffer, n);
            let cavro = parser.parse();
            info!("Cavro parser error code: {:?}", cavro.error_code);
            if cavro.error_code == xyz_parser::ErrorCode::NoError {
                let xyz = XYZMessage::decode(cavro.message_data);
                if xyz.control == XYZMessage::ACK {
                    info!("Received ACK message on downstream - ignoring as already acknowledged by upstream receiver");
                }
                else {
                    // we have an answer from downstream - send ack to downstream and pass upstream
                    info!("Queuing ACK for downstream");
                    let ack_xyz = XYZMessage::new_ack(xyz.arm_address, xyz.device_address);
                    let ack_message = CavroMessage::new(ack_xyz.encode());
                    outgoing_downstream.try_send(ack_message).unwrap();

                    // pass the response upstream
                    info!("Queuing message for upstream");
                    let up_message = CavroMessage::new(xyz.encode());
                    outgoing_upstream.try_send(up_message).unwrap();
                }
            }
        }
    }
}

#[cfg(feature = "embedded")]
#[embassy_executor::task(pool_size = 1)]
async fn upstream_message_sender(
    mut upstream_usart_tx: UartTx<'static, Async>,
    receiver: embassy_sync::channel::Receiver<'static, CriticalSectionRawMutex, CavroMessage, OUT_CHANNEL_MSG_SIZE>,
) -> ! {
    loop {
        let message = receiver.receive().await;
        let message_bytes = message.encode();
        upstream_usart_tx.write(message_bytes.as_slice()).await.expect("Failed to write message to UART");
        info!("Sent message to upstream UART: {}", message);
    }
}

#[cfg(feature = "embedded")]
#[embassy_executor::task(pool_size = 1)]
async fn pump_message_sender(
    mut rs485_enable: Output<'static>,
    mut pump_rs485_tx: UartTx<'static, Async>,
    mut pump_rs485_rx: UartRx<'static, Async>,
    receiver: embassy_sync::channel::Receiver<'static, CriticalSectionRawMutex, CavroMessage, OUT_CHANNEL_MSG_SIZE>,
) -> ! {
    loop {
        let message = receiver.receive().await;
        let message_bytes = message.encode();
        // set the RS485 enable pin high to transmit and then low to receive again.
        rs485_enable.set_high();
        pump_rs485_tx.blocking_write(&message_bytes).expect("TODO: panic message");
        pump_rs485_tx.blocking_flush().expect("TODO: panic message");
        pump_rs485_tx.write(message_bytes.as_slice()).await.expect("Failed to write message to RS485");
        info!("Sent message to pump RS485: {}", message_bytes);

        // receive ack and response
        rs485_enable.set_low();
        let mut buffer = [0u8; 64];
        let mut data_len = 0;
        while data_len == 0 {
            let pump_reply = pump_rs485_rx.read_until_idle(&mut buffer).await;
            if pump_reply.is_ok() {
                data_len = pump_reply.unwrap();
            } else {
                info!("Waiting for pump ack");
            }
            info!("pump: ok: {:?} len: {:?} rcv: {:?}", pump_reply.is_ok(), data_len, buffer[0..data_len]);
        }
    }
}

#[cfg(feature = "embedded")]
#[embassy_executor::task(pool_size = 1)]
async fn downstream_message_sender(
    mut downstream_usart_tx: UartTx<'static, Async>,
    receiver: embassy_sync::channel::Receiver<'static, CriticalSectionRawMutex, CavroMessage, OUT_CHANNEL_MSG_SIZE>,
) -> ! {
    loop {
        let message = receiver.receive().await;
        let message_bytes = message.encode();
        downstream_usart_tx.write(message_bytes.as_slice()).await.expect("Failed to write message to UART");
        info!("Sent message to downstream UART: {}", message);
    }
}

struct XMotorController {
    motor_cs: Output<'static>,
    motor_reset: Output<'static>,
    motor_stop: Output<'static>,
}
impl XMotorController {
    fn new(motor_cs: Output<'static>,
           motor_reset: Output<'static>,
           motor_stop: Output<'static>,
    ) -> Self {
        Self {
            motor_cs,
            motor_reset,
            motor_stop
        }
    }
}
impl MotorController for XMotorController {
    fn reset(&mut self, reset: bool) {
        info!("MotorController reset (active low): reset {}", reset);
        if reset {
            self.motor_reset.set_low();
        } else {
            self.motor_reset.set_high();
        }
    }

    fn stop(&mut self, stop: bool) {
        info!("MotorController stop: stop {}", stop);
        if stop {
            self.motor_stop.set_low();
        } else {
            self.motor_stop.set_high();
        }
    }

    fn chip_select(&mut self, select: bool) {
        if select {
            self.motor_cs.set_low();
        } else {
            self.motor_cs.set_high();
        }
    }
}

struct XYZStepperController {
    spi: Spi<'static, Blocking, Master>,
}
impl XYZStepperController {
    fn new(spi: Spi<'static, Blocking, Master>) -> Self {
        Self {
            spi,
        }
    }
}
impl StepperController for XYZStepperController {
    fn enable_stepper_driver(&mut self, _enable: bool) {
        info!("enable_stepper_driver: Not implemented");
    }

    fn spi_transfer(&mut self, tx: &u8, rx: &mut u8) -> Result<(), u8> {
        let mut data_value = *tx;
        let data = core::slice::from_mut(&mut data_value);
        // info!("spi_tx_rx: Transmitting byte {:02X}", tx);
        match self.spi.blocking_transfer_in_place(data) {
            Ok(_) => {
                *rx = data[0];
                // info!("spi_tx_rx: Received byte {:02X}", rx);
            }
            Err(_) => {
                error!("SPI transfer error");
            }
        };

        Ok(()) // success
    }

    fn reset(&mut self, enable: bool) {
        info!("stepper reset: Not implemented: enable={}", enable);
    }

    fn delay_ms(&mut self, ms: u32) {
        block_for(Duration::from_millis(ms as u64));
    }
}

#[cfg(feature = "embedded")]
#[embassy_executor::main]
async fn main(spawner: Spawner) {
    info!("Starting up x_firmware...");
    let p: Peripherals = embassy_stm32::init(Default::default());

    // TODO: create a new executor task to blink the heartbeat LED at lower priority
    // TODO: flash the error code on the LED if init fails?
    // start the LED heartbeat task
    let heartbeat_led = Output::new(p.PB1, Level::High, Speed::Low);
    spawner.spawn(heartbeat_task(heartbeat_led)).unwrap();

    //==================================
    // Configure USART3 for debug output
    //==================================
    // debug usart3: TX = PB10/DMA-1_3, RX = PB11/DMA-1_1
    bind_interrupts!(struct Irqs3 {
        USART3 => usart::InterruptHandler<peripherals::USART3>;
    });

    let usart3 = usart::Uart::new(
        p.USART3,
        p.PB11,      // RX
        p.PB10,      // TX
        Irqs3,
        p.DMA1_CH3, // TX DMA
        p.DMA1_CH1, // RX DMA
        uart_config()).expect("Failed to configure uart");
    let (usart3_tx, _usart3_rx) = usart3.split();

    let _usart3_tx_static: &'static mut UartTx<'static, Async> = cortex_m::singleton!(
        : UartTx<'static, Async> = usart3_tx
    ).expect("Failed to create static reference for usart3_tx");

    // This enables defmt logging over USART3, it will redirect all defmt logs to this UART
    // so RTT logs will not appear if this is enabled.
    #[cfg(feature = "defmt-serial")]
    defmt_serial::defmt_serial(_usart3_tx_static);

    //============================================
    // Configure USART1 for upstream communication
    //============================================
    // upstream usart1: TX = PA9/DMA-2_7, RX = PA10/DMA-2_5
    bind_interrupts!(struct Irqs1 {
        USART1 => usart::InterruptHandler<peripherals::USART1>;
    });

    // TODO: flash the error code on the LED if usart init fails
    let usart1 = usart::Uart::new(
        p.USART1,
        p.PA10,      // RX
        p.PA9,      // TX
        Irqs1,
        p.DMA2_CH7, // TX DMA
        p.DMA2_CH5, // RX DMA
        uart_config()).expect("Failed to configure uart");
    let (upstream_tx, upstream_rx) = usart1.split();

    //==============================================
    // Configure USART2 for downstream communication
    //==============================================
    // downstream usart2: TX = PA2/DMA-1_7, RX = PA3/DMA-1_5
    bind_interrupts!(struct Irqs2 {
        USART2 => usart::InterruptHandler<peripherals::USART2>;
    });

    // needs to be enable tor YZ board to start due to floating
    let usart2 = usart::Uart::new(
        p.USART2,
        p.PA3,      // RX
        p.PA2,      // TX
        Irqs2,
        p.DMA1_CH6, // TX DMA
        p.DMA1_CH5, // RX DMA
        uart_config()).expect("Failed to configure uart");
    let (downstream_tx, downstream_rx) = usart2.split();

    //=========================================
    // Configure USART6 for RS485 communication
    //=========================================
    // USART6: TX = PC6/DMA-2_6, RX = PC7/DMA-2_1
    bind_interrupts!(struct Irqs6 {
        USART6 => usart::InterruptHandler<peripherals::USART6>;
    });

    let usart6 = usart::Uart::new(
        p.USART6,
        p.PC7,      // RX
        p.PC6,      // TX
        Irqs6,
        p.DMA2_CH6, // TX DMA
        p.DMA2_CH1, // RX DMA
        uart_config()).expect("Failed to configure uart");
    let (pump_tx, pump_rx) = usart6.split();

    // RS485 enable pin is on PC14
    let rs485_enable = Output::new(p.PC14, Level::Low, Speed::Low);

    //=========================================
    // Configure SPI1 for X-motor communication
    //=========================================
    // Configure SPI1 pins: SCK = PB3, MISO = PB4, MOSI = PB5
    let spi_config = embassy_stm32::spi::Config::default();
    let m_spi = p.SPI1;
    let sck = p.PB3;
    let miso = p.PB4;
    let mosi = p.PB5;
    let motor_spi = Spi::new_blocking(m_spi, sck, mosi, miso, spi_config);

    let mut motor1_home_enable = Output::new(p.PB8, Level::High, Speed::Low);
    motor1_home_enable.set_low();
    let motor1_home = Input::new(p.PC5, Pull::None);

    let motor1_reset = Output::new(p.PC1, Level::Low, Speed::Low);
    let motor1_cs = Output::new(p.PD2, Level::High, Speed::Low);
    let motor1_stop = Output::new(p.PC0, Level::Low, Speed::Low);

    // interlock cuts power to the motors, so need to handle recovery
    let motor_status_24v = Input::new(p.PC15, Pull::None);

    info!("Creating singleton XMotorController...");
    let motor_controller: &'static mut XMotorController = cortex_m::singleton!(
        : XMotorController = XMotorController::new(motor1_cs, motor1_reset, motor1_stop)
    ).expect("Failed creating singleton XMotorController");


    info!("creating motors...");
    let motors = [motor_controller as &mut dyn MotorController];

    info!("Creating singleton XYZStepperController...");
    let stepper_controller_opt: &'static mut Option<XYZStepperController> = cortex_m::singleton!(
        : Option<XYZStepperController> = None
    ).expect("Failed creating singleton Option<XYZStepperController>");
    if stepper_controller_opt.is_none() {
        *stepper_controller_opt = Some(XYZStepperController::new(motor_spi));
    }
    let stepper_controller: &'static mut XYZStepperController = stepper_controller_opt.as_mut().unwrap();

    info!("Creating singleton PowerStepControl...");
    let power_ctrl_opt: &'static mut Option<PowerStepControl<'static, XYZStepperController>> = cortex_m::singleton!(
        : Option<PowerStepControl<'static, XYZStepperController>> = None
    ).expect("Failed creating singleton Option<PowerStepControl>");
    if power_ctrl_opt.is_none() {
        *power_ctrl_opt = Some(PowerStepControl::new(motors, stepper_controller));
    }
    let power_ctrl: &'static mut PowerStepControl<'static, XYZStepperController> = power_ctrl_opt.as_mut().unwrap();

    // initialize the power-step controller and all the motors
    info!("Initializing PowerStepControl...");
    power_ctrl.initialize();
    power_ctrl.read_all_registers(X_MOTOR_DEVICE_ID);

    info!("Starting upstream message handler task...");
    spawner.spawn(upstream_message_handler_task(power_ctrl, motor1_home, motor_status_24v,
                                                OUT_PUMP_MSG_CHANNEL.sender(),
                                                OUT_UPSTREAM_MSG_CHANNEL.sender(),
                                                )).unwrap();

    // TODO: pump message handler
    // TODO: downstream handler not needed for now - using wait for ack function
    // info!("Starting downstream message handler task...");
    // spawner.spawn(downstream_message_handler_task()).unwrap();

    info!("Starting message receiver task...upstream");
    spawner.spawn(upstream_message_receiver(upstream_rx,
                                        IN_UPSTREAM_MSG_CHANNEL.sender(),   // data to process
    )).unwrap();

    info!("Starting message receiver task...downstream");
    spawner.spawn(downstream_message_receiver(downstream_rx,
                                              OUT_DOWNSTREAM_MSG_CHANNEL.sender(),  // response and ack
                                              OUT_UPSTREAM_MSG_CHANNEL.sender(),   // for passing upstream
    )).unwrap();

    info!("Starting message sender task... upstream");
    spawner.spawn(upstream_message_sender(upstream_tx,
                                          OUT_UPSTREAM_MSG_CHANNEL.receiver())).unwrap();

    info!("Starting message sender task... pump");
    spawner.spawn(pump_message_sender(rs485_enable, pump_tx, pump_rx,
                                      OUT_PUMP_MSG_CHANNEL.receiver())).unwrap();

    info!("Starting message sender task...downstream");
    spawner.spawn(downstream_message_sender(downstream_tx,
                                            OUT_DOWNSTREAM_MSG_CHANNEL.receiver())).unwrap();

    loop {
        // let data = "USART1.".as_bytes();
        // usart1.write(&data).await.expect("TODO: panic message");

        // info!("Waiting 2 sec");
        Timer::after_millis(2000).await;
    }
}

#[cfg(not(feature = "embedded"))]
fn main() {
    // Initialize the CavroMessageParser
    let parser = CavroMessageParser::new();

    // Print the version of the parser
    info!("CavroMessageParser version: {}", parser.version());
}