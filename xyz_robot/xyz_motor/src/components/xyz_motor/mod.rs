#![allow(non_upper_case_globals)]
#![allow(non_camel_case_types)]
#![allow(non_snake_case)]

use core::include;
use core::ptr::{null_mut};

include!("./generated/powerstep01.rs");

#[cfg(feature = "embedded")]
use {
    defmt::{ error, info },
    embassy_stm32::mode::Blocking,
    embassy_stm32::spi::Spi,
    embassy_stm32::gpio::Output,
    embassy_stm32::usart::UartTx,
    embassy_stm32::mode::Async,
    embassy_time::{block_for, Duration},
};

#[cfg(not(feature = "embedded"))]
pub mod mocks;

#[cfg(not(feature = "embedded"))]
use {
    log::{ error, info },
    crate::components::xyz_motor::mocks::{Spi, Blocking, Output, UartTx, Async, Duration, block_for},
};

// // Constants
// pub const MOTOR_STATUS_REG: u8 = 0x1B;
// pub const MOTOR_CONFIG_REG: u8 = 0x1A;
// pub const COMMAND_MASK: u8 = 0xFA;
// pub const POWERSTEP01_CMD_ARG_MAX_NB_BYTES: usize = 4;
//
// Commands
#[repr(u8)]
#[derive(Copy, Clone)]
pub enum Powerstep01Command {
    Nop = 0x00,
    SetParamAbsPos = 0x00 + 0x01,
    SetParamElPos = 0x00 + 0x02,
    SetParamMark = 0x00 + 0x03,
    SetParamSpeed = 0x00 + 0x04,
    SetParamAcc = 0x00 + 0x05,
    SetParamDec = 0x00 + 0x06,
    SetParamMaxSpeed = 0x00 + 0x07,
    SetParamMinSpeed = 0x00 + 0x08,
    SetParamKValHoldOrTValHold = 0x00 + 0x09,
    SetParamKValRunOrTValRun = 0x00 + 0x0A,
    SetParamKValAccOrTValAcc = 0x00 + 0x0B,
    SetParamKValDecOrTValDec = 0x00 + 0x0C,
    SetParamIntSpeed = 0x00 + 0x0D,
    SetParamStSlpOrTFast = 0x00 + 0x0E,
    SetParamFnSlpAccOrTonMin = 0x00 + 0x0F,
    SetParamFnSlpDecOrToffMin = 0x00 + 0x10,
    SetParamKTherm = 0x00 + 0x11,
    SetParamAdcOut = 0x00 + 0x12,
    SetParamOcdTh = 0x00 + 0x13,
    SetParamStallTh = 0x00 + 0x14,
    SetParamFsSpd = 0x00 + 0x15,
    SetParamStepMode = 0x00 + 0x16,
    SetParamAlarmEn = 0x00 + 0x17,
    SetParamGateCfg1 = 0x00 + 0x18,
    SetParamGateCfg2 = 0x00 + 0x19,
    SetParamConfig = 0x00 + 0x1A,
    SetParamStatus = 0x00 + 0x1B,
    // GetParam commands
    GetParamAbsPos = 0x20 + 0x01,
    GetParamElPos = 0x20 + 0x02,
    GetParamMark = 0x20 + 0x03,
    GetParamSpeed = 0x20 + 0x04,
    GetParamAcc = 0x20 + 0x05,
    GetParamDec = 0x20 + 0x06,
    GetParamMaxSpeed = 0x20 + 0x07,
    GetParamMinSpeed = 0x20 + 0x08,
    GetParamKValHold = 0x20 + 0x09,
    GetParamKValRun = 0x20 + 0x0A,
    GetParamKValAcc = 0x20 + 0x0B,
    GetParamKValDec = 0x20 + 0x0C,
    GetParamIntSpeed = 0x20 + 0x0D,
    GetParamStSlpOrTFast = 0x20 + 0x0E,
    GetParamFnSlpAccOrTonMin = 0x20 + 0x0F,
    GetParamFnSlpDecOrToffMin = 0x20 + 0x10,
    GetParamKTherm = 0x20 + 0x11,
    GetParamAdcOut = 0x20 + 0x12,
    GetParamOcdTh = 0x20 + 0x13,
    GetParamStallTh = 0x20 + 0x14,
    GetParamFsSpd = 0x20 + 0x15,
    GetParamStepMode = 0x20 + 0x16,
    GetParamAlarmEn = 0x20 + 0x17,
    GetParamGateCfg1 = 0x20 + 0x18,
    GetParamGateCfg2 = 0x20 + 0x19,
    GetParamConfig = 0x20 + 0x1A,
    GetParamStatus = 0x20 + 0x1B,
    RunRev = 0x50,
    RunFwd = 0x51,
    StepClockRev = 0x58,
    StepClockFwd = 0x59,
    MoveRev = 0x40,
    MoveFwd = 0x41,
    GoTo = 0x60,
    GoToDirFwd = 0x68,
    GoToDirRev = 0x69,
    GoUntilRev = 0x82,
    GoUntilRevAct = 0x8A,
    GoUntilFwd = 0x83,
    GoUntilFwdAct = 0x8B,
    ReleaseSwRev = 0x92,
    ReleaseSwRevAct = 0x9A,
    ReleaseSwFwd = 0x93,
    ReleaseSwFwdAct = 0x9B,
    GoHome = 0x70,
    GoMark = 0x78,
    ResetPos = 0xD8,
    ResetDevice = 0xC0,
    SoftStop = 0xB0,
    HardStop = 0xB8,
    SoftHiz = 0xA0,
    HardHiz = 0xA8,
    GetStatus = 0xD0,
    ReservedCmd1 = 0xEB,
    ReservedCmd2 = 0xF8,
}

// Masks
pub const POWERSTEP01_ABS_POS_VALUE_MASK: u32 = 0x003F_FFFF;
pub const POWERSTEP01_ABS_POS_SIGN_BIT_MASK: u32 = 0x0020_0000;

// Status masks
pub const POWERSTEP01_STATUS_BUSY: u16 = 0x0002;

// Motor struct

#[repr(C)]
pub struct XYZMotor<'a> {
    spi: Spi<'a, Blocking>,
    tx_buf: [u8; POWERSTEP01_CMD_ARG_MAX_NB_BYTES as usize],
    rx_buf: [u8; POWERSTEP01_CMD_ARG_MAX_NB_BYTES as usize],
    motor_x: Option<&'a XYZMotor<'static>>,
    n_reset: Output<'static>,
    motor_cs: Output<'static>,
    usart_dbg: UartTx<'static, Async>,
    rs485_enable: Output<'static>,
}

static mut GLOBAL_MOTOR_PTR: *mut XYZMotor<'static> = null_mut();

impl<'a> XYZMotor<'static> {
    pub fn new(spi: Spi<'static, Blocking>, n_reset: Output<'static>, motor_cs: Output<'static>,
               uart_dbg: UartTx<'static, Async>, rs485_enable: Output<'static>) -> Self {
        Self {
            spi,
            tx_buf: [0; POWERSTEP01_CMD_ARG_MAX_NB_BYTES as usize],
            rx_buf: [0; POWERSTEP01_CMD_ARG_MAX_NB_BYTES as usize],
            motor_x: None,
            n_reset,
            motor_cs,
            uart_dbg,
            rs485_enable,
        }
    }

    pub fn blocking_delay_ms(&mut self, ms: u32) {
        // blocking delay for ms milliseconds
        block_for(Duration::from_millis(ms as u64))
    }

    pub fn spi_tx_rx(&mut self, pByteToTransmit: &mut u8, pByteToReceive: &mut u8, _nb_devices: u8) -> u8 {
        let mut data_value = *pByteToTransmit;
        let data = core::slice::from_mut(&mut data_value);
        info!("spi_tx_rx: Transmitting byte {:02X}", pByteToTransmit);
        // self.rs485_enable.set_high();
        // let mut high_byte: [u8;2] = [0;2];
        // high_byte[0] = 0x30 + (*pByteToTransmit >> 4) & 0x0F;
        // high_byte[1] = 0x30 + *pByteToTransmit & 0x0F;
        // let _ = self.usart_dbg.blocking_write(&high_byte);
        // self.usart_dbg.blocking_flush().expect("TODO: panic message");
        // self.rs485_enable.set_low();

        self.motor_cs.set_low();
        match self.spi.blocking_transfer_in_place(data) {
            Ok(_) => {
                *pByteToReceive = data[0];
                info!("spi_tx_rx: Received byte {:02X}", pByteToReceive);
            }
            Err(_) => {
                error!("SPI transfer error");
            }
        };
        self.motor_cs.set_high();

        0 // success
    }

    #[allow(non_snake_case)]
    #[unsafe(no_mangle)]
    pub extern "C" fn Powerstep01_Board_GpioInit(device_id: u8) {
        // initialization the GPIO for the Powerstep01 motor driver board
        info!("Powerstep01_Board_GpioInit on device {}", device_id);
    }

    #[allow(non_snake_case)]
    #[unsafe(no_mangle)]
    pub extern "C" fn Powerstep01_Board_SpiInit() -> u8 {
        info!("Powerstep01_Board_SpiInit");
        0   // success
    }

    #[allow(non_snake_case)]
    #[unsafe(no_mangle)]
    pub extern "C" fn Powerstep01_Board_StepClockInit() {
        info!("Powerstep01_Board_StepClockInit");
    }

    #[allow(non_snake_case)]
    #[unsafe(no_mangle)]
    pub extern "C" fn Powerstep01_Board_ReleaseReset(device_id: u8) {
        info!("Powerstep01_Board_ReleaseReset on device {}", device_id);
        unsafe { (*GLOBAL_MOTOR_PTR).n_reset.set_high(); }
    }

    #[allow(non_snake_case)]
    #[unsafe(no_mangle)]
    pub extern "C" fn Powerstep01_Board_Delay(delay: u32) {
        info!("Powerstep01_Board_Delay: {} ms", delay);
        unsafe { (*GLOBAL_MOTOR_PTR).blocking_delay_ms(delay); }
    }

    #[allow(non_snake_case)]
    #[unsafe(no_mangle)]
    pub extern "C" fn Powerstep01_Board_BUSY_PIN_GetState() -> u32 {
        info!("Powerstep01_Board_BUSY_PIN_GetState");
        0   // not busy
    }

    #[allow(non_snake_case)]
    #[unsafe(no_mangle)]
    pub extern "C" fn Powerstep01_Board_FLAG_PIN_GetState() -> u32 {
        info!("Powerstep01_Board_FLAG_PIN_GetState");
        0   // no flag
    }

    #[allow(non_snake_case)]
    #[unsafe(no_mangle)]
    pub extern "C" fn Powerstep01_Board_EnableIrq() {
        info!("Powerstep01_Board_EnableIrq")
    }

    #[allow(non_snake_case)]
    #[unsafe(no_mangle)]
    pub extern "C" fn Powerstep01_Board_DisableIrq() {
        info!("Powerstep01_Board_DisableIrq")
    }

    #[allow(non_snake_case)]
    #[unsafe(no_mangle)]
    pub extern "C" fn Powerstep01_Board_Reset(device_id: u8) {
        info!("Powerstep01_Board_Reset on device {}", device_id);
        unsafe { (*GLOBAL_MOTOR_PTR).n_reset.set_low(); }
    }

    #[allow(non_snake_case)]
    #[unsafe(no_mangle)]
    pub extern "C" fn Powerstep01_Board_StartStepClock(device_id: u8) {
        info!("Powerstep01_Board_StartStepClock on device {}", device_id);
    }

    #[allow(non_snake_case)]
    #[unsafe(no_mangle)]
    pub extern "C" fn Powerstep01_Board_StopStepClock() {
        info!("Powerstep01_Board_StopStepClock");
    }

    #[allow(non_snake_case)]
    #[unsafe(no_mangle)]
    pub extern "C" fn Powerstep01_Board_SpiWriteBytes(pByteToTransmit: &mut u8, pReceivedByte: &mut u8, nb_devices: u8) -> u8 {
        info!("Powerstep01_Board_SpiWriteBytes: nb_devices = {}", nb_devices);
        unsafe { (*GLOBAL_MOTOR_PTR).spi_tx_rx(pByteToTransmit, pReceivedByte, nb_devices) }
    }

    fn cmd_set_param(&mut self, device_id: u8, param: Powerstep01Command, value: u32) {
        info!("Calling Powerstep01_CmdSetParam: param={:?}, value=0x{:X}", "TODO", value);
        unsafe { Powerstep01_CmdSetParam(device_id, param as u8 as u32, value) }
    }

    pub fn initialize_driver(&mut self, num_devices: u8) {
        // external C functions need a static pointer to the motor instance
        info!("Setting GLOBAL_MOTOR_PTR");
        unsafe { GLOBAL_MOTOR_PTR = self; }

        // number of devices on the SPI bus
        info!("Calling Powerstep01_SetNbDevices: num_devices={}", num_devices);
        unsafe { Powerstep01_SetNbDevices(num_devices); }

        info!("Calling Powerstep01_Init(0)");
        unsafe { Powerstep01_Init(null_mut()); }
    }

    // TODO" move this to the board firmware as specific to each motor
    pub fn init_motor(&mut self, device_id: u8) {
        self.hard_hi_z(device_id);

        self.cmd_set_param(device_id, Powerstep01Command::SetParamAlarmEn, 0xEF);
        self.cmd_get_status(device_id);

        // self.cmd_set_param(device_id, Powerstep01Command::SetParamStepMode, 0x09);   // Current mode, half step
        self.cmd_set_param(device_id, Powerstep01Command::SetParamStepMode, 0x0C);      // Current mode, 1/16 step
        self.cmd_get_status(device_id);

        self.cmd_set_param(device_id, Powerstep01Command::SetParamFnSlpAccOrTonMin, 0x0F);  // 4
        self.cmd_get_status(device_id);

        self.cmd_set_param(device_id, Powerstep01Command::SetParamFnSlpDecOrToffMin, 0x17);  // 17
        self.cmd_get_status(device_id);

        self.cmd_set_param(device_id, Powerstep01Command::SetParamFsSpd, 0x03FF);
        self.cmd_get_status(device_id);

        self.cmd_set_param(device_id, Powerstep01Command::SetParamGateCfg1, 0x00C6);
        self.cmd_get_status(device_id);

        self.cmd_set_param(device_id, Powerstep01Command::SetParamGateCfg2, 0x20);
        self.cmd_get_status(device_id);

        self.cmd_set_param(device_id, Powerstep01Command::SetParamStSlpOrTFast, 0x35);  // 33
        self.cmd_get_status(device_id);

        self.cmd_set_param(device_id, Powerstep01Command::SetParamKValRunOrTValRun, 0x16);
        self.cmd_get_status(device_id);

        self.cmd_set_param(device_id, Powerstep01Command::SetParamKValAccOrTValAcc, 0x16);
        self.cmd_get_status(device_id);

        for value in [0x01, 0x02, 0x05] {
            self.cmd_set_param(device_id, Powerstep01Command::SetParamKValHoldOrTValHold, value);
            self.cmd_get_status(device_id);
            self.blocking_delay_ms(10);
        }

        self.cmd_set_param(device_id, Powerstep01Command::SetParamKValDecOrTValDec, 0x16);
        self.cmd_get_status(device_id);

        // set the over current threshold to v low
        self.cmd_set_param(device_id, Powerstep01Command::SetParamOcdTh, 0x05);
        self.cmd_get_status(device_id);

        // X, Y
        self.cmd_set_param(device_id, Powerstep01Command::SetParamAcc, 0x146);
        self.cmd_set_param(device_id, Powerstep01Command::SetParamDec, 0x146);
        self.cmd_set_param(device_id, Powerstep01Command::SetParamMaxSpeed, 0x80);
        // Z
        // self.cmd_set_param(device_id, Powerstep01Command::SetParamAcc, 0x408);
        // self.cmd_set_param(device_id, Powerstep01Command::SetParamDec, 0x408);
        // self.cmd_set_param(device_id, Powerstep01Command::SetParamMaxSpeed, 0x84);

        // self.cmd_set_param(device_id, Powerstep01Command::SetParamMaxSpeed, 0x0106);
        self.cmd_get_status(device_id);
        self.cmd_set_param(device_id, Powerstep01Command::SetParamConfig, 0x1680);

        self.hard_stop(device_id);

        // clear status register
        self.cmd_get_status(device_id);
    }

    pub fn fetch_and_clear_all_status(&mut self) {
        info!("Calling Powerstep01_CmdGetStatus");
        unsafe { Powerstep01_FetchAndClearAllStatus() };
    }

    pub fn hard_hi_z(&mut self, device_id: u8) {
        info!("Calling Powerstep01_CmdHardHiZ");
        unsafe { Powerstep01_CmdHardHiZ(device_id); }
    }

    pub fn hard_stop(&mut self, device_id: u8) {
        info!("Calling Powerstep01_CmdHardStop");
        unsafe { Powerstep01_CmdHardStop(device_id); }
    }

    pub async fn get_position(&mut self, device_id: u8) -> i32 {
        info!("Calling Powerstep01_CmdGetParam: ABS_POS");
        let abs_pos = self.cmd_get_param(device_id, Powerstep01Command::GetParamAbsPos);
        self.convert_position(abs_pos)
    }

    pub fn move_home(&mut self, device_id: u8) {
        info!("Calling Powerstep01_CmdGoHome");
        unsafe { Powerstep01_CmdGoHome(device_id); }
    }

    pub fn move_steps_in_direction(&mut self, device_id: u8, direction: motorDir_t, steps: u32) {
        info!("Calling Powerstep01_CmdMove: direction={}, steps={}", direction, steps);
        unsafe { Powerstep01_CmdMove(device_id, direction, steps & POWERSTEP01_ABS_POS_VALUE_MASK); }
    }

    pub fn cmd_get_status(&mut self, device_id: u8) -> u16 {
        info!("Calling Powerstep01_CmdGetStatus");
        unsafe { Powerstep01_CmdGetStatus(device_id) }
    }

    pub fn cmd_get_param(&mut self, device_id: u8, param: Powerstep01Command) -> u32 {
        info!("Calling Powerstep01_CmdGetParam: param={:?}", param as u8);
        unsafe { Powerstep01_CmdGetParam(device_id, param as u8 as u32) }
    }

    pub fn convert_position(&mut self, abs_position_reg: u32) -> i32 {
        info!("Calling Powerstep01_CmdConvertPosition: abs_position_reg=0x{:X}", abs_position_reg);
        if abs_position_reg & POWERSTEP01_ABS_POS_SIGN_BIT_MASK != 0 {
            let reg = !abs_position_reg + 1;
            let val = (reg & POWERSTEP01_ABS_POS_VALUE_MASK) as i32;
            -val
        } else {
            abs_position_reg as i32
        }
    }

    pub fn device_busy(&mut self, device_id: u8) -> bool {
        info!("Checking if device {} is busy", device_id);
        (self.cmd_get_status(device_id) & POWERSTEP01_STATUS_BUSY) == 0
    }

    pub fn read_all_registers(&mut self, device_id: u8) {
        let speed = self.cmd_get_param(device_id, Powerstep01Command::GetParamSpeed);
        info!("SPEED (0x04): 0x{:04X}", speed);

        let acc = self.cmd_get_param(device_id, Powerstep01Command::GetParamAcc);
        info!("ACC (0x05): 0x{:04X}", acc);

        let dec = self.cmd_get_param(device_id, Powerstep01Command::GetParamDec);
        info!("DEC (0x06): 0x{:04X}", dec);

        let max_speed = self.cmd_get_param(device_id, Powerstep01Command::GetParamMaxSpeed);
        info!("MAX_SPEED (0x07): 0x{:04X}", max_speed);

        let min_speed = self.cmd_get_param(device_id, Powerstep01Command::GetParamMinSpeed);
        info!("MIN_SPEED (0x08): 0x{:04X}", min_speed);

        let tval_run = self.cmd_get_param(device_id, Powerstep01Command::SetParamKValRunOrTValRun);
        info!("TVAL_RUN (0x0A): 0x{:02X}", tval_run);

        let tval_acc = self.cmd_get_param(device_id, Powerstep01Command::SetParamKValAccOrTValAcc);
        info!("TVAL_ACC (0x0B): 0x{:02X}", tval_acc);

        let tval_dec = self.cmd_get_param(device_id, Powerstep01Command::SetParamKValDecOrTValDec);
        info!("TVAL_DEC (0x0C): 0x{:02X}", tval_dec);

        let t_fast = self.cmd_get_param(device_id, Powerstep01Command::GetParamStSlpOrTFast);
        info!("T_FAST (0x0E): 0x{:02X}", t_fast);

        let ton_min = self.cmd_get_param(device_id, Powerstep01Command::GetParamFnSlpAccOrTonMin);
        info!("TON_MIN (0x0F): 0x{:02X}", ton_min);

        let toff_min = self.cmd_get_param(device_id, Powerstep01Command::GetParamFnSlpDecOrToffMin);
        info!("TOFF_MIN (0x0F): 0x{:02X}", toff_min);

        let ocd_th = self.cmd_get_param(device_id, Powerstep01Command::GetParamOcdTh);
        info!("OCD_TH (0x13): 0x{:02X}", ocd_th);

        let fs_spd = self.cmd_get_param(device_id, Powerstep01Command::GetParamFsSpd);
        info!("FS_SPD (0x15): 0x{:04X}", fs_spd);

        let step_mode = self.cmd_get_param(device_id, Powerstep01Command::GetParamStepMode);
        info!("STEP_MODE (0x16): 0x{:02X}", step_mode);

        let alarm_en = self.cmd_get_param(device_id, Powerstep01Command::GetParamAlarmEn);
        info!("ALARM_EN (0x17): 0x{:02X}", alarm_en);

        let gatecfg1 = self.cmd_get_param(device_id, Powerstep01Command::GetParamGateCfg1);
        info!("GATECFG1 (0x18): 0x{:04X}", gatecfg1);

        let gatecfg2 = self.cmd_get_param(device_id, Powerstep01Command::GetParamGateCfg2);
        info!("GATECFG2 (0x19): 0x{:02X}", gatecfg2);

        let config = self.cmd_get_param(device_id, Powerstep01Command::GetParamConfig);
        info!("CONFIG (0x1A): 0x{:04X}", config);

        let status = self.cmd_get_param(device_id, Powerstep01Command::GetParamStatus);
        info!("STATUS (0x1B): 0x{:04X}", status);
    }
}
