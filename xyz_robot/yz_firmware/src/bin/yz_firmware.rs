#![cfg_attr(feature="embedded", no_std, no_main)]

use xyz_motor::{MotorController, StepperController, PowerStepControl, MOTOR_DIR_BACKWARD, MOTOR_DIR_FORWARD, MotorDirection};
use xyz_parser::{XYZCommand, XYZMessage, CavroMessage, CavroMessageParser};
use fixed::types::extra::U3;
use embassy_stm32::spi::mode::Master;


#[cfg(feature = "embedded")]
use {
    defmt_rtt as _,
    defmt::{error, info},
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
    heapless::Vec,
    panic_probe as _,
};

#[cfg(not(feature = "embedded"))]
use {
    log::{error, info},
};

// devices are on different SPI buses, so both can use device ID 0
const Y_MOTOR_DEVICE_ID: u8 = 0;
const Z_MOTOR_DEVICE_ID: u8 = 0;

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
static IN_DOWNSTREAM_MSG_CHANNEL: Channel<CriticalSectionRawMutex, CavroMessage, IN_CHANNEL_MSG_SIZE> = Channel::new();
static OUT_DOWNSTREAM_MSG_CHANNEL: Channel<CriticalSectionRawMutex, CavroMessage, OUT_CHANNEL_MSG_SIZE> = Channel::new();

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

#[cfg(feature = "embedded")]
#[embassy_executor::task]
async fn message_handler_task(power_step_controller_y: &'static mut PowerStepControl<'static, XYZStepperController>,
                              power_step_controller_z: &'static mut PowerStepControl<'static, XYZStepperController>,
                              motor_home: Input<'static>,
                              motor_status_24v: Input<'static>,
                              _outgoing_to_lls: embassy_sync::channel::Sender<'static, CriticalSectionRawMutex, CavroMessage, OUT_CHANNEL_MSG_SIZE>,
                              upstream_outgoing: embassy_sync::channel::Sender<'static, CriticalSectionRawMutex, CavroMessage, IN_CHANNEL_MSG_SIZE>
) -> ! {
    let receiver = IN_UPSTREAM_MSG_CHANNEL.receiver();

    /// position wrapper using Cell to allow interior mutability
    /// will add functionality to convert between -ve coordinate space later
    /// Origin at 0,0,0 is this robot's absolute home position but the instrument's home
    /// position is at some offset from this.
    use core::cell::Cell;
    struct Position {
        x: Cell<u32>,
        y: Cell<u32>,
        // z: Cell<u32>,
    }
    impl Position {
        fn new(x: u32, y: u32, _z: u32) -> Self {
            Self {
                x: Cell::new(x),
                y: Cell::new(y),
                // z: Cell::new(z),
            }
        }
    }

    // will not use x on this board
    let current_position = Position::new(0, 0, 0);

    fn send_response_upstream(in_msg: &XYZMessage, data: Vec<u8, 255>, error_code: u8) {
        let xyz = XYZMessage::create_answer(in_msg, data, error_code);
        let response_message = CavroMessage::new(xyz.encode());
        OUT_UPSTREAM_MSG_CHANNEL.sender().try_send(response_message).unwrap();
    }

    let mut motor_needs_reset = false;
    loop {
        let cavro = receiver.receive().await;
        let xyz = XYZMessage::decode(cavro.message_data.clone());

        // send ack upstream as we have 24V power and are now processing the message content
        let xyz_ack = XYZMessage::new_ack(xyz.arm_address, xyz.device_address);
        let ack = CavroMessage::new(xyz_ack.encode());
        upstream_outgoing.try_send(ack).unwrap();

        let command = XYZCommand::decode(xyz);
        // TODO: check for errors

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
            info!("Motor power has been restored - resetting motor controllers");
            power_step_controller_z.initialize();
            power_step_controller_y.initialize();
            motor_needs_reset = false;
        }

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

        info!("Processing command: {}, params: {:?}", command.cmd.as_str(), command.params);
        match command.cmd.as_str() {
            // "BL" => {
            //     // process the message in sender that owns the outgoing uart
            //     // as it needs to send a break signal.
            //     outgoing_to_lls.try_send(message).unwrap();
            // }
            "PA" => {
                wait_for_motor(Y_MOTOR_DEVICE_ID, power_step_controller_y);
                wait_for_motor(Z_MOTOR_DEVICE_ID, power_step_controller_z);

                info!("Num params: {}", command.num_params);
                validate_params!(command.num_params, 3);
                let x_pos: u32 = command.params[0] as u32;
                let y_pos: u32 = command.params[1] as u32;
                let z_pos: u32 = command.params[2] as u32;
                validate_position!(x_pos, y_pos, z_pos);
                info!("Set absolute position: x={}, y={}, z={}", x_pos, y_pos, z_pos);
                // move y
                let dir: MotorDirection;
                let y_steps: u32;
                if y_pos < current_position.y.get() {
                    dir = MOTOR_DIR_BACKWARD;
                    y_steps = current_position.y.get() - y_pos;
                }
                else {
                    dir = MOTOR_DIR_FORWARD;
                    y_steps = y_pos - current_position.y.get();
                }

                if y_steps > 0 {
                    info!("Moving Y motor: {} steps, dir {}", y_steps, dir);
                    power_step_controller_y.Powerstep01_CmdMove(Y_MOTOR_DEVICE_ID, dir, y_steps).ok();
                    current_position.y.set(y_pos);
                    info!("New Y position: {}", current_position.y.get());
                }

                // TODO: blank response for now
                let xyz = XYZMessage::decode(cavro.message_data.clone());
                send_response_upstream(&xyz, Vec::new(), 0);
            }
            "PI" | "YI" | "ZI" => {
                wait_for_motor(Y_MOTOR_DEVICE_ID, power_step_controller_y);
                wait_for_motor(Z_MOTOR_DEVICE_ID, power_step_controller_z);

                if command.cmd.as_str() == "YI" {
                    // TODO: check if Z homed and if not, return error
                    // for now, drop through and assume it is homed
                }

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

                // home Z-axis first
                if command.cmd.as_str() == "PI" || command.cmd.as_str() == "ZI" {
                    // Z-axis initialize home
                    info!("Initialize Z motor home position.");
                    use fixed::FixedU32;
                    let fixed_speed: FixedU32<U3> = FixedU32::const_from_int(cmd_speed);
                    let speed = fixed_speed.to_bits();        // this is steps per sec in fixed point 0:28
                    info!("Setting speed {} as {}", cmd_speed, speed);
                    // let steps = 10; // small increments
                    // // TODO: replace with homing logic
                    // while motor_home_z.is_low() {
                    //     power_step_controller_z.Powerstep01_CmdMove(Z_MOTOR_DEVICE_ID, MOTOR_DIR_FORWARD, steps).expect("TODO: panic message");
                    //     Timer::after_millis(100).await;
                    // }
                    // info!("Z motor has triggered home switch");
                    info!("Z Homing not implemented yet - assuming at home position");
                }

                // next home Y-axis
                if command.cmd.as_str() == "PI" || command.cmd.as_str() == "YI" {
                    // Y-axis initialize home
                    //==============================
                    // initialize X motor
                    //==============================
                    info!("Initialize Y motor home position.");
                    use fixed::FixedU32;
                    let fixed_speed: FixedU32<U3> = FixedU32::const_from_int(cmd_speed);
                    let speed = fixed_speed.to_bits();        // this is steps per sec in fixed point 0:28
                    info!("Setting speed {} as {}", cmd_speed, speed);
                    power_step_controller_y.Powerstep01_CmdGoUntil(Y_MOTOR_DEVICE_ID, MOTOR_DIR_BACKWARD, speed).expect("TODO: panic message");
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

                            // set the current x position to 0 as we are at true home
                            current_position.x.set(0);
                            info!("Motor 1 is moving out of switch");
                            // move away from the home position slightly
                            power_step_controller_y.Powerstep01_CmdReleaseSw(Y_MOTOR_DEVICE_ID, MOTOR_DIR_FORWARD).expect("TODO: panic message");
                            break;
                        }

                        info!("Motor 1 not at home position, waiting 100ms");
                        Timer::after_millis(100).await;
                        counter += 1;
                    }
                }
                // TODO: blank response for now
                let xyz = XYZMessage::decode(cavro.message_data.clone());
                send_response_upstream(&xyz, Vec::new(), 0);
            }
            "YR" | "ZR" => {
                validate_params!(command.num_params, 1);
                let is_z = command.cmd.as_str() == "ZR";
                let steps_and_dir: i32 = command.params[0];
                let steps: u32 = steps_and_dir.abs() as u32;
                let dir = if steps_and_dir < 0 { MOTOR_DIR_BACKWARD } else { MOTOR_DIR_FORWARD };
                if is_z {
                    wait_for_motor(Z_MOTOR_DEVICE_ID, power_step_controller_z);
                    info!("Moving Z motor: {} steps, dir {}", steps, dir);
                    power_step_controller_z.Powerstep01_CmdMove(Z_MOTOR_DEVICE_ID, dir, steps).ok();
                } else {
                    wait_for_motor(Y_MOTOR_DEVICE_ID, power_step_controller_y);
                    info!("Moving Y motor: {} steps, dir {}", steps, dir);
                    power_step_controller_y.Powerstep01_CmdMove(Y_MOTOR_DEVICE_ID, dir, steps).ok();
                    // update current position
                    if dir == MOTOR_DIR_BACKWARD {
                        current_position.y.set(current_position.y.get().wrapping_sub(steps));
                    } else {
                        current_position.y.set(current_position.y.get().wrapping_add(steps));
                    }
                    info!("New Y position: {}", current_position.y.get());
                }

                // TODO: blank response for now
                let xyz = XYZMessage::decode(cavro.message_data.clone());
                send_response_upstream(&xyz, Vec::new(), 0);
            },
            _ => info!("Unknown command: {}", command.cmd.as_str()),
        }
    }
}

#[cfg(feature = "embedded")]
#[embassy_executor::task(pool_size = 1)]
async fn downstream_message_receiver(
    usart_rx: UartRx<'static, Async>,
    processing: embassy_sync::channel::Sender<'static, CriticalSectionRawMutex, CavroMessage, IN_CHANNEL_MSG_SIZE>,
) -> ! {
    let mut rx_dma_buf = [0u8; 256];
    let mut ring_rx = usart_rx.into_ring_buffered(&mut rx_dma_buf);
    let mut buffer = [0u8; 64];
    let mut parser = CavroMessageParser::new();

    loop {
        info!("Reading from UART");
        let n: usize = match ring_rx.read(&mut buffer).await {
            Ok(n) => {n}
            Err(_e) => {
                error!("Error reading from UART");
                continue
            }
        };
        if n > 0 {
            info!("Read {} bytes from UART", n);
            parser.add_data(&buffer, n);
            let cavro = parser.parse();
            info!("Message parser error code: {:?}", cavro.error_code);
            if cavro.error_code == xyz_parser::ErrorCode::NoError {
                let xyz = XYZMessage::decode(cavro.message_data.clone());
                if xyz.control == XYZMessage::ACK {
                    info!("Received ACK message - ignoring for now");
                    continue;
                }

                // process the message in another task, which sends an ack and response when done
                info!("Queuing upstream message for processing");
                let cavro_msg = CavroMessage::new(xyz.encode());
                if let Err(e) = processing.try_send(cavro_msg) {
                    info!("Failed to send message to channel: {:?}", e);
                }
            }
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
        info!("Reading from UART");
        let n: usize = match ring_rx.read(&mut buffer).await {
            Ok(n) => {n}
            Err(_e) => {
                error!("Error reading from UART");
                continue
            }
        };
        if n > 0 {
            info!("Read {} bytes from UART", n);
            parser.add_data(&buffer, n);
            let cavro = parser.parse();
            info!("Message parser error code: {:?}", cavro.error_code);
            if cavro.error_code == xyz_parser::ErrorCode::NoError {
                let xyz = XYZMessage::decode(cavro.message_data);
                if xyz.control == XYZMessage::ACK {
                    info!("Received ACK message - ignoring for now");
                    continue;
                }

                // process the message in another task, which sends an ack and response when done
                info!("Queuing upstream message for processing");
                let cavro_msg = CavroMessage::new(xyz.encode());
                if let Err(e) = processing.try_send(cavro_msg) {
                    info!("Failed to send message to channel: {:?}", e);
                }
            }
        }
    }
}

#[cfg(feature = "embedded")]
#[embassy_executor::task(pool_size = 2)]
async fn message_sender_task(
    task_id: u8,
    mut usart_tx: UartTx<'static, Async>,
    receiver: embassy_sync::channel::Receiver<'static, CriticalSectionRawMutex, CavroMessage, OUT_CHANNEL_MSG_SIZE>,
) -> ! {
    loop {
        let xyz = receiver.receive().await;
        //let command = XYZCommand::decode(xyz);

        // // check for an LLS boot request
        // if command.cmd == "BL" {
        //     info!("Sending break signal for LLS boot");
        //     usart_tx.send_break();
        //     // wait a bit after the break
        //     Timer::after(Duration::from_millis(10)).await;
        //     // TODO: send a response back to the X board.
        //     continue;
        // }

        let message_bytes = xyz.encode();
        usart_tx.write(&message_bytes.as_slice()).await.expect("Failed to write message to UART");
        info!("Sent message from sender task {}: {:?}", task_id, xyz);
    }
}

//////////////////////////////////////////////////////////////////////////////////////////////
// TODO: This can be refactored with the XMotorController as they are the same
//////////////////////////////////////////////////////////////////////////////////////////////
struct YZMotorController {
    motor_cs: Output<'static>,
    motor_reset: Output<'static>,
    motor_stop: Output<'static>,
}
impl YZMotorController {
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
impl MotorController for YZMotorController {
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

//////////////////////////////////////////////////////////////////////////////////////////////
// TODO: This can be moved into a common embedded module
//////////////////////////////////////////////////////////////////////////////////////////////
struct XYZStepperController {
    spi: Spi<'static, Blocking, Master>,
}
impl<'a> XYZStepperController {
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
    info!("Starting up yz_firmware...");
    let p: Peripherals = embassy_stm32::init(Default::default());

    // TODO: create a new executor task to blink the heartbeat LED at lower priority
    // TODO: flash the error code on the LED if init fails?
    // start the LED heartbeat task
    let heartbeat_led = Output::new(p.PB1, Level::High, Speed::Low);
    let result = spawner.spawn(heartbeat_task(heartbeat_led));
    match result {
        Ok(_) => info!("Started heartbeat task"),
        Err(e) => error!("Failed to start heartbeat task: {:?}", e),
    };

    // let mut lls_boot_pin = Flex::new(p.PA2);    // Push-pull output
    // lls_boot_pin.set_as_output(Speed::Low);
    // loop {
    //     info!("BOOT pin PA2 - Z test low");
    //     lls_boot_pin.set_low();    // send break
    //     Timer::after_millis(100).await;
    //     lls_boot_pin.set_high();    // send break
    //     Timer::after_millis(100).await;
    //     info!("BOOT pin PA2 - Z test high");
    //
    //     // wait 10 seconds
    //     Timer::after_millis(10000).await;
    // }
    
    //==================================
    // Configure USART3 for debug output
    //==================================
    // debug usart3: TX = PB10/DMA-1_3, RX = PB11/DMA-1_1
    bind_interrupts!(struct Irqs3 {
        USART3 => usart::InterruptHandler<peripherals::USART3>;
    });

    let mut _usart3 = usart::Uart::new(
        p.USART3,
        p.PB11,      // RX
        p.PB10,      // TX
        Irqs3,
        p.DMA1_CH3, // TX DMA
        p.DMA1_CH1, // RX DMA
        uart_config()).expect("Failed to configure uart");

    // loop {
    //     // read and discard any data on the debug port
    //     let mut buf = [0u8; 32];
    //     match _usart3.read_until_idle(&mut buf).await {
    //         Ok(n) if n > 0 => {
    //             info!("Debug USART3 read {} bytes: {:?}", n, &buf[..n]);
    //         }
    //         _ => {
    //             info!("Error or no data on debug USART3");
    //         }
    //     }
    // }

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

    // TODO: flash the error code on the LED if usart init fails
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
    let (_usart6_tx, _usart6_rx) = usart6.split();

    // RS485 enable pin is on PC14
    let _rs485_enable = Output::new(p.PC14, Level::Low, Speed::Low);

    //////////////////////////////////////////////////////////////////////////////////////////////
    // A lot of this code could go into a common embedded module as pin assignments same as X?
    //////////////////////////////////////////////////////////////////////////////////////////////

    //=========================================
    // Configure SPI1 for Y-motor communication
    //=========================================
    // Configure SPI1 pins: SCK = PB3, MISO = PB4, MOSI = PB5
    let spi_config = embassy_stm32::spi::Config::default();
    let m_spi = p.SPI1;
    let sck = p.PB3;
    let miso = p.PB4;
    let mosi = p.PB5;
    let motor_spi_1 = Spi::new_blocking(m_spi, sck, mosi, miso, spi_config);

    //=========================================
    // Configure SPI2 for Z-motor communication
    //=========================================
    // Configure SPI2 pins: SCK = PB13, MISO = PB14, MOSI = PB15
    let spi_config = embassy_stm32::spi::Config::default();
    let m_spi = p.SPI2;
    let sck = p.PB13;
    let miso = p.PB14;
    let mosi = p.PB15;
    let motor_spi_2 = Spi::new_blocking(m_spi, sck, mosi, miso, spi_config);

    let _motor1_busy = Input::new(p.PB6, Pull::None);
    let _motor2_busy = Input::new(p.PC4, Pull::None);
    let mut motor1_home_enable = Output::new(p.PB8, Level::High, Speed::Low);
    let mut motor2_home_enable = Output::new(p.PB9, Level::High, Speed::Low);
    motor1_home_enable.set_low();
    motor2_home_enable.set_low();

    // active low motor stop
    let motor1_stop = Output::new(p.PC0, Level::Low, Speed::Low);
    let motor2_stop = Output::new(p.PC9, Level::Low, Speed::Low);

    let motor1_home = Input::new(p.PC5, Pull::None);
    // Z motor does not have a home flag

    // interlock cuts power to the motors, so need to handle recovery
    let motor_status_24v = Input::new(p.PC15, Pull::None);

    info!("Creating Y singleton YMotorController...");
    let motor1_reset = Output::new(p.PC1, Level::Low, Speed::Low);
    let motor1_cs = Output::new(p.PD2, Level::High, Speed::Low);
    let motor1_controller: &'static mut YZMotorController = cortex_m::singleton!(
        : YZMotorController = YZMotorController::new(motor1_cs, motor1_reset, motor1_stop)
    ).expect("Failed creating singleton");

    info!("Creating Z singleton YMotorController...");
    let motor2_reset = Output::new(p.PC2, Level::Low, Speed::Low);
    let motor2_cs = Output::new(p.PB12, Level::High, Speed::Low);
    let motor2_controller: &'static mut YZMotorController = cortex_m::singleton!(
        : YZMotorController = YZMotorController::new(motor2_cs, motor2_reset, motor2_stop)
    ).expect("Failed creating singleton");

    //=========================================
    // Y Motor
    //=========================================
    let motors = [motor1_controller as &mut dyn MotorController];

    // TODO: create a macro for creating motor controllers and power-step controllers to reduce code duplication

    info!("Creating Y singleton XYZStepperController...");
    let stepper_controller_y_opt: &'static mut Option<XYZStepperController> = cortex_m::singleton!(
        : Option<XYZStepperController> = None
    ).expect("Failed creating singleton");
    if stepper_controller_y_opt.is_none() {
        *stepper_controller_y_opt = Some(XYZStepperController::new(motor_spi_1));
    }
    let stepper_controller_y: &'static mut XYZStepperController = stepper_controller_y_opt.as_mut().unwrap();

    info!("Creating Y PowerStepControl...");
    let power_ctrl_y_opt: &'static mut Option<PowerStepControl<'static, XYZStepperController>> = cortex_m::singleton!(
        : Option<PowerStepControl<'static, XYZStepperController>> = None
    ).expect("Failed creating singleton");
    if power_ctrl_y_opt.is_none() {
        *power_ctrl_y_opt = Some(PowerStepControl::new(motors, stepper_controller_y));
    }
    let power_ctrl_y: &'static mut PowerStepControl<'static, XYZStepperController> = power_ctrl_y_opt.as_mut().unwrap();

    //=========================================
    // Z Motor
    //=========================================
    let motors = [motor2_controller as &mut dyn MotorController];

    info!("Creating Y singleton XYZStepperController...");
    let stepper_controller_z_opt: &'static mut Option<XYZStepperController> = cortex_m::singleton!(
        : Option<XYZStepperController> = None
    ).expect("Failed creating singleton");
    if stepper_controller_z_opt.is_none() {
        *stepper_controller_z_opt = Some(XYZStepperController::new(motor_spi_2));
    }
    let stepper_controller_z: &'static mut XYZStepperController = stepper_controller_z_opt.as_mut().unwrap();

    info!("Creating Y singleton PowerStepControl...");
    let power_ctrl_z_opt: &'static mut Option<PowerStepControl<'static, XYZStepperController>> = cortex_m::singleton!(
        : Option<PowerStepControl<'static, XYZStepperController>> = None
    ).expect("Failed creating singleton");
    if power_ctrl_z_opt.is_none() {
        *power_ctrl_z_opt = Some(PowerStepControl::new(motors, stepper_controller_z));
    }
    let power_ctrl_z: &'static mut PowerStepControl<'static, XYZStepperController> = power_ctrl_z_opt.as_mut().unwrap();

    // initialize the power-step controller and all the motors
    info!("Initializing PowerStepControl...");
    power_ctrl_y.initialize();
    power_ctrl_z.initialize();

    // // Home the x motor
    // info!("Homing X motor...");
    // motor1_home_enable.set_high();
    // power_ctrl.Powerstep01_CmdGoHome(X_MOTOR_DEVICE_ID).ok();
    // motor1_home_enable.set_low();

    power_ctrl_y.read_all_registers(Y_MOTOR_DEVICE_ID);
    power_ctrl_z.read_all_registers(Y_MOTOR_DEVICE_ID);

    // LLS boot pin is on PA2, Error LED is on PA8
    // let boot_pin = Output::new(p.PA2, Level::Low, Speed::Low);  // UART2 TX pin has ownership
    // TODO: use error led when bootloader
    // let error_led = Output::new(p.PA8, Level::Low, Speed::Low);

    // start a task to handle messages via a channel
    info!("Starting message handler task...");
    spawner.spawn(message_handler_task(power_ctrl_y, power_ctrl_z, motor1_home, motor_status_24v,
                                       OUT_DOWNSTREAM_MSG_CHANNEL.sender(),
                                       OUT_UPSTREAM_MSG_CHANNEL.sender(),       // ack
    )).unwrap();

    info!("Starting message receiver task...upstream");
    spawner.spawn(upstream_message_receiver(upstream_rx,
                                        IN_UPSTREAM_MSG_CHANNEL.sender(),   // data to process
    )).unwrap();

    info!("Starting message receiver task...downstream");
    spawner.spawn(downstream_message_receiver(downstream_rx,
                                        IN_DOWNSTREAM_MSG_CHANNEL.sender(),   // data to process
    )).unwrap();

    // spawn a task to send message responses upstream
    info!("Starting message sender task...");
    spawner.spawn(message_sender_task(1, upstream_tx,
                                      OUT_UPSTREAM_MSG_CHANNEL.receiver())).unwrap();

    // spawn a task to send messages downstream to the YZ board
    info!("Starting message sender task...");
    spawner.spawn(message_sender_task(2, downstream_tx,
                                      OUT_DOWNSTREAM_MSG_CHANNEL.receiver())).unwrap();


    loop {
        // let data = "USART1.".as_bytes();
        // usart1.write(&data).await.expect("TODO: panic message");

        // This uart is connected to RS485, so need to set the enable pin high to transmit
        // and then low to receive again.
        // let data = "USART6.".as_bytes();
        // rs485_enable.set_high();
        // usart6.blocking_write(&data).expect("TODO: panic message");
        // usart6.blocking_flush().expect("TODO: panic message");
        // rs485_enable.set_low();

        // info!("Waiting 2 sec");
        Timer::after_millis(2000).await;
    }
}

#[cfg(not(feature = "embedded"))]
fn main() {
    // Initialize the XYZMessageParser
    let parser = XYZMessageParser::new();

    // Print the version of the parser
    info!("XYZMessageParser version: {}", parser.version());
}