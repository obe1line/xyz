#![cfg_attr(feature="embedded", no_std)]

#[cfg(test)]
mod motor_unit_tests;

#[cfg(feature = "embedded")]
use {
    defmt::{error, info},
};

#[cfg(not(feature = "embedded"))]
use {
    log::{error, info},
};

// const MAX_MOTORS: usize = 2;    // Maximum number of motors supported per controller
const MAX_MOTORS: usize = 1;    // Maximum number of motors supported per controller

pub type MotorId = u8;
pub const MOTOR_ID_0: MotorId = 0;
pub const MOTOR_ID_1: MotorId = 1;
pub const MOTOR_ID_ERR: MotorId = 0xFF;

pub type MotorDirection = u8;
pub const MOTOR_DIR_BACKWARD: MotorDirection = 0;
pub const MOTOR_DIR_FORWARD: MotorDirection = 1;

pub type AbsPosMask = u32;
pub const POWERSTEP01_ABS_POS_VALUE_MASK: AbsPosMask = 0x003FFFFF;
pub const POWERSTEP01_ABS_POS_SIGN_BIT_MASK: AbsPosMask = 0x00200000;

pub type PowerStepCommands = u8;
pub const POWERSTEP01_NOP: PowerStepCommands = 0x00;
pub const POWERSTEP01_SET_PARAM: PowerStepCommands = 0x00;
pub const POWERSTEP01_GET_PARAM: PowerStepCommands = 0x20;
pub const POWERSTEP01_RUN: PowerStepCommands = 0x50;
pub const POWERSTEP01_STEP_CLOCK: PowerStepCommands = 0x58;
pub const POWERSTEP01_MOVE: PowerStepCommands = 0x40;
pub const POWERSTEP01_GO_TO: PowerStepCommands = 0x60;
pub const POWERSTEP01_GO_TO_DIR: PowerStepCommands = 0x68;
pub const POWERSTEP01_GO_UNTIL: PowerStepCommands = 0x82;
pub const POWERSTEP01_GO_UNTIL_ACT_CPY: PowerStepCommands = 0x8A;
pub const POWERSTEP01_RELEASE_SW: PowerStepCommands = 0x92;
pub const POWERSTEP01_GO_HOME: PowerStepCommands = 0x70;
pub const POWERSTEP01_GO_MARK: PowerStepCommands = 0x78;
pub const POWERSTEP01_RESET_POS: PowerStepCommands = 0xD8;
pub const POWERSTEP01_RESET_DEVICE: PowerStepCommands = 0xC0;
pub const POWERSTEP01_SOFT_STOP: PowerStepCommands = 0xB0;
pub const POWERSTEP01_HARD_STOP: PowerStepCommands = 0xB8;
pub const POWERSTEP01_SOFT_HIZ: PowerStepCommands = 0xA0;
pub const POWERSTEP01_HARD_HIZ: PowerStepCommands = 0xA8;
pub const POWERSTEP01_GET_STATUS: PowerStepCommands = 0xD0;
pub const POWERSTEP01_RESERVED_CMD1: PowerStepCommands = 0xEB;
pub const POWERSTEP01_RESERVED_CMD2: PowerStepCommands = 0xF8;

pub const POWERSTEP01_CMD_ARG_MAX_NB_BYTES: usize = 4;
pub const POWERSTEP01_CMD_ARG_NB_BYTES_NOP: usize = 1;
pub const POWERSTEP01_CMD_ARG_NB_BYTES_RUN: usize = 4;
pub const POWERSTEP01_CMD_ARG_NB_BYTES_STEP_CLOCK: usize = 1;
pub const POWERSTEP01_CMD_ARG_NB_BYTES_MOVE: usize = 4;
pub const POWERSTEP01_CMD_ARG_NB_BYTES_GO_TO: usize = 4;
pub const POWERSTEP01_CMD_ARG_NB_BYTES_GO_TO_DIR: usize = 4;
pub const POWERSTEP01_CMD_ARG_NB_BYTES_GO_UNTIL: usize = 4;
pub const POWERSTEP01_CMD_ARG_NB_BYTES_RELEASE_SW: usize = 1;
pub const POWERSTEP01_CMD_ARG_NB_BYTES_GO_HOME: usize = 1;
pub const POWERSTEP01_CMD_ARG_NB_BYTES_GO_MARK: usize = 1;
pub const POWERSTEP01_CMD_ARG_NB_BYTES_RESET_POS: usize = 1;
pub const POWERSTEP01_CMD_ARG_NB_BYTES_RESET_DEVICE: usize = 1;
pub const POWERSTEP01_CMD_ARG_NB_BYTES_SOFT_STOP: usize = 1;
pub const POWERSTEP01_CMD_ARG_NB_BYTES_HARD_STOP: usize = 1;
pub const POWERSTEP01_CMD_ARG_NB_BYTES_SOFT_HIZ: usize = 1;
pub const POWERSTEP01_CMD_ARG_NB_BYTES_HARD_HIZ: usize = 1;
pub const POWERSTEP01_CMD_ARG_NB_BYTES_GET_STATUS: usize = 1;
pub const POWERSTEP01_RSP_NB_BYTES_GET_STATUS: usize = 2;

pub type PowerStepParams = u8;
pub const POWERSTEP01_ABS_POS: PowerStepParams = 0x01;
pub const POWERSTEP01_EL_POS: PowerStepParams = 0x02;
pub const POWERSTEP01_MARK: PowerStepParams = 0x03;
pub const POWERSTEP01_SPEED: PowerStepParams = 0x04;
pub const POWERSTEP01_ACC: PowerStepParams = 0x05;
pub const POWERSTEP01_DEC: PowerStepParams = 0x06;
pub const POWERSTEP01_MAX_SPEED: PowerStepParams = 0x07;
pub const POWERSTEP01_MIN_SPEED: PowerStepParams = 0x08;
pub const POWERSTEP01_KVAL_HOLD: PowerStepParams = 0x09;
pub const POWERSTEP01_KVAL_RUN: PowerStepParams = 0x0A;
pub const POWERSTEP01_KVAL_ACC: PowerStepParams = 0x0B;
pub const POWERSTEP01_KVAL_DEC: PowerStepParams = 0x0C;
pub const POWERSTEP01_INT_SPD: PowerStepParams = 0x0D;
pub const POWERSTEP01_ST_SLP: PowerStepParams = 0x0E;
pub const POWERSTEP01_FN_SLP_ACC: PowerStepParams = 0x0F;
pub const POWERSTEP01_FN_SLP_DEC: PowerStepParams = 0x10;
pub const POWERSTEP01_K_THERM: PowerStepParams = 0x11;
pub const POWERSTEP01_ADC_OUT: PowerStepParams = 0x12;
pub const POWERSTEP01_OCD_TH: PowerStepParams = 0x13;
pub const POWERSTEP01_STALL_TH: PowerStepParams = 0x14;
pub const POWERSTEP01_FS_SPD: PowerStepParams = 0x15;
pub const POWERSTEP01_STEP_MODE: PowerStepParams = 0x16;
pub const POWERSTEP01_ALARM_EN: PowerStepParams = 0x17;
pub const POWERSTEP01_GATECFG1: PowerStepParams = 0x18;
pub const POWERSTEP01_GATECFG2: PowerStepParams = 0x19;
pub const POWERSTEP01_CONFIG: PowerStepParams = 0x1A;
pub const POWERSTEP01_STATUS: PowerStepParams = 0x1B;

pub const DAISY_CHAIN_COMMAND_MASK: u8 = 0xFA;

//===========================================================
// TODO: refactor this
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
    GetParamFnSlpDec = 0x20 + 0x10,
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

// Status masks
pub const POWERSTEP01_STATUS_BUSY: u16 = 0x0002;
//===========================================================

pub trait MotorController {
    fn reset(&mut self, enable: bool);
    fn stop(&mut self, stop: bool);
    fn chip_select(&mut self, select: bool);
}

pub trait StepperController {
    fn enable_stepper_driver(&mut self, enable: bool);
    fn spi_transfer(&mut self, tx: &u8, rx: &mut u8) -> Result<(), u8>;
    fn reset(&mut self, enable: bool);
    fn delay_ms(&mut self, ms: u32);
}

// pub struct Motors<'a> {
//     motors: [&'a mut dyn MotorController; MAX_MOTORS],
// }
// impl<'a> Motors<'a> {
//     pub fn new(motors: [&'a mut dyn MotorController; MAX_MOTORS]) -> Self {
//         Motors {
//             motors,
//         }
//     }
// }
// impl<'a> Motors<'a> {

pub struct PowerStepControl<'a, StepperControlT> {
    controller: &'a mut StepperControlT,
    motors: [&'a mut dyn MotorController; MAX_MOTORS],
    spi_tx_buf: [[u8; POWERSTEP01_CMD_ARG_MAX_NB_BYTES]; MAX_MOTORS],
    spi_rx_buf: [[u8; POWERSTEP01_CMD_ARG_MAX_NB_BYTES]; MAX_MOTORS],
}

impl<'a, StepperControlT: StepperController> PowerStepControl<'a, StepperControlT> {
    /// Create a new motor controller object.
    /// # Arguments
    /// * `num_motors` - The number of motors to manage (maximum 2).
    /// * `motors` - A `Motors` struct containing the motor instances.
    /// * `controller` - A mutable reference to an object implementing the `StepperController` trait.
    ///
    /// Returns a `PowerStepControl` instance.
    /// # Examples
    /// ```
    /// todo!("Add example usage");
    /// ```
    pub fn new(motors: [&'a mut dyn MotorController; MAX_MOTORS],
               controller: &'a mut StepperControlT
    ) -> Self {
        PowerStepControl {
            motors,
            controller,
            spi_tx_buf: [[0; POWERSTEP01_CMD_ARG_MAX_NB_BYTES]; MAX_MOTORS],
            spi_rx_buf: [[0; POWERSTEP01_CMD_ARG_MAX_NB_BYTES]; MAX_MOTORS],
        }
    }

    pub fn num_motors(&self) -> u8 {
        self.motors.len() as u8
    }

    fn motor_exists(&self, motor_id: MotorId) -> bool {
        motor_id < MAX_MOTORS as u8
    }

    fn stop_motor(&mut self, motor_id: MotorId, stop: bool) {
        if self.motor_exists(motor_id) {
            let m: &mut dyn MotorController = self.motors[motor_id as usize];
            m.stop(stop);
        } else {
            error!("Motor ID {} not found during stop", motor_id);
        }
    }

    fn reset_motor(&mut self, motor_id: MotorId, reset: bool) {
        if self.motor_exists(motor_id) {
            let m: &mut dyn MotorController = self.motors[motor_id as usize];
            m.reset(reset);
        } else {
            error!("Motor ID {} not found during enable", motor_id);
        }
    }

    fn chip_select_motor(&mut self, motor_id: MotorId, select: bool) {
        if self.motor_exists(motor_id) {
            let m: &mut dyn MotorController = self.motors[motor_id as usize];
            m.chip_select(select);
        } else {
            error!("Motor ID {} not found during chip_select", motor_id);
        }
    }

    pub fn initialize(&mut self) {
        self.Powerstep01_Init();
        for motor_id in 0..self.num_motors() {
            self.init_motor(motor_id);
        }
    }

    pub fn init_motor(&mut self, motor_id: u8) {
        // TODO: Fix this HACK as motor_id is motor_index and X is the same as Y (0), Z (1)
        let mut motor_number = 0;   // X (0)
        if self.motors.len() > 1 {
            motor_number = motor_id + 1;   // Y (1) or Z (2)
        }

        if motor_id >= self.num_motors() {
            error!("Motor ID {} not found during initialization", motor_id);
            return;
        }

        info!("Enabling motor ID {}", motor_id);
        self.stop_motor(motor_id, false);   // release the motor stop pin
        self.reset_motor(motor_id, false);

        // hard hiz
        self.hard_hi_z(motor_id);

        // turn off uvo_adc flag switch and stall detection
        self.cmd_set_param(motor_id, Powerstep01Command::SetParamAlarmEn, 0xEF);
        self.cmd_get_status(motor_id);

        // 1/16 microstep mode current configuration
        self.cmd_set_param(motor_id, Powerstep01Command::SetParamStepMode, 0x0C);
        self.cmd_get_status(motor_id);

        // X motor
        match motor_number {
            0 => {
                info!("Configuring X motor");
                //	G1
                self.cmd_set_param(motor_id, Powerstep01Command::SetParamGateCfg1, 0x00C6);
                self.cmd_get_status(motor_id);

                self.cmd_set_param(motor_id, Powerstep01Command::SetParamStSlpOrTFast, 0x33);
                self.cmd_get_status(motor_id);
                //TVAL_RUN
                self.cmd_set_param(motor_id, Powerstep01Command::SetParamKValRunOrTValRun, 0x1A);
                self.cmd_get_status(motor_id);
                // TVAL_ACC
                self.cmd_set_param(motor_id, Powerstep01Command::SetParamKValAccOrTValAcc, 0x1A);
                self.cmd_get_status(motor_id);
                //TVAL_HOLD
                self.cmd_set_param(motor_id, Powerstep01Command::SetParamKValHoldOrTValHold, 0x01);
                self.cmd_get_status(motor_id);
                self.Powerstep01_Board_Delay(10);
                self.cmd_get_status(motor_id);
                self.cmd_set_param(motor_id, Powerstep01Command::SetParamKValHoldOrTValHold, 0x02);
                self.cmd_get_status(motor_id);
                self.Powerstep01_Board_Delay(10);
                self.cmd_get_status(motor_id);
                self.cmd_set_param(motor_id, Powerstep01Command::SetParamKValHoldOrTValHold, 0x05);
                self.cmd_get_status(motor_id);
                //TVAL_DEC
                self.cmd_set_param(motor_id, Powerstep01Command::SetParamKValDecOrTValDec, 0x1A);
                self.cmd_get_status(motor_id);

                // Set the ton_min time
                self.cmd_set_param(motor_id, Powerstep01Command::SetParamFnSlpAccOrTonMin, 0x0F);
                self.cmd_get_status(motor_id);

                // Set the toff_min time
                self.cmd_set_param(motor_id, Powerstep01Command::SetParamFnSlpDecOrToffMin, 0x17);
                self.cmd_get_status(motor_id);

                //	G2
                self.cmd_set_param(motor_id, Powerstep01Command::SetParamGateCfg2, 0x20);
                self.cmd_get_status(motor_id);

                // set the over current threshold to around 6A
                self.cmd_set_param(motor_id, Powerstep01Command::SetParamOcdTh, 0x04);
                self.cmd_get_status(motor_id);

                //     Set the maximum speed
                // TODO: ? self.cmd_set_param(motor_id, Powerstep01Command::SetParamMinSpeed, 0x78);
                self.cmd_set_param(motor_id, Powerstep01Command::SetParamMaxSpeed, 0x97);
                self.cmd_get_status(motor_id);
                //     Set the maximum speed
                self.cmd_set_param(motor_id, Powerstep01Command::SetParamAcc, 0x0190);
                self.cmd_get_status(motor_id);
                //     Set the maximum speed
                self.cmd_set_param(motor_id, Powerstep01Command::SetParamDec, 0x0190);
                self.cmd_get_status(motor_id);

                // CONFIG
                self.cmd_set_param(motor_id, Powerstep01Command::SetParamConfig, 0x1680);
                self.hard_stop(motor_id);

                // clear status register
                self.cmd_get_status(motor_id);
            }
            1 => {
                info!("Configuring Y motor");
                // Set the ton_min time
                self.cmd_set_param(motor_id, Powerstep01Command::SetParamFnSlpAccOrTonMin, 0x0F);
                self.cmd_get_status(motor_id);

                // Set the toff_min time
                self.cmd_set_param(motor_id, Powerstep01Command::SetParamFnSlpDecOrToffMin, 0x17);
                self.cmd_get_status(motor_id);

                // FS_SPD
                self.cmd_set_param(motor_id, Powerstep01Command::SetParamFsSpd, 0x03FF);
                self.cmd_get_status(motor_id);
                // Set the gate drivers
                //	G1
                self.cmd_set_param(motor_id, Powerstep01Command::SetParamGateCfg1, 0x00C6);
                self.cmd_get_status(motor_id);
                //	G2
                self.cmd_set_param(motor_id, Powerstep01Command::SetParamGateCfg2, 0x0020);
                self.cmd_get_status(motor_id);
                // T_FAST
                self.cmd_set_param(motor_id, Powerstep01Command::SetParamStSlpOrTFast, 0x35);
                self.cmd_get_status(motor_id);

                //TVAL_RUN
                self.cmd_set_param(motor_id, Powerstep01Command::SetParamKValRunOrTValRun, 0x16);
                self.cmd_get_status(motor_id);
                // TVAL_ACC
                self.cmd_set_param(motor_id, Powerstep01Command::SetParamKValAccOrTValAcc, 0x16);
                self.cmd_get_status(motor_id);

                //TVAL_HOLD
                self.cmd_set_param(motor_id, Powerstep01Command::SetParamKValHoldOrTValHold, 0x01);
                self.cmd_get_status(motor_id);
                self.Powerstep01_Board_Delay(10);
                self.cmd_get_status(motor_id);
                self.cmd_set_param(motor_id, Powerstep01Command::SetParamKValHoldOrTValHold, 0x02);
                self.cmd_get_status(motor_id);
                self.Powerstep01_Board_Delay(10);
                self.cmd_get_status(motor_id);
                self.cmd_set_param(motor_id, Powerstep01Command::SetParamKValHoldOrTValHold, 0x05);
                self.cmd_get_status(motor_id);
                //TVAL_DEC
                self.cmd_set_param(motor_id, Powerstep01Command::SetParamKValDecOrTValDec, 0x16);
                self.cmd_get_status(motor_id);

                // set the over current threshold to v low
                self.cmd_set_param(motor_id, Powerstep01Command::SetParamOcdTh, 0x05);
                self.cmd_get_status(motor_id);

                // set acc and dec //2BC
                self.cmd_set_param(motor_id, Powerstep01Command::SetParamAcc, 0x146);
                self.cmd_get_status(motor_id);
                self.cmd_set_param(motor_id, Powerstep01Command::SetParamDec, 0x146);
                self.cmd_get_status(motor_id);
                self.cmd_set_param(motor_id, Powerstep01Command::SetParamMaxSpeed, 0x80);
                self.cmd_get_status(motor_id);

                // CONFIG
                self.cmd_set_param(motor_id, Powerstep01Command::SetParamConfig, 0x1680);
                self.hard_stop(motor_id);

                // clear status register
                self.cmd_get_status(motor_id);
            }
            2 => {
                info!("Configuring Z motor");
                // Set the ton_min time
                self.cmd_set_param(motor_id, Powerstep01Command::SetParamFnSlpAccOrTonMin, 0x0F);
                self.cmd_get_status(motor_id);

                // Set the toff_min time
                self.cmd_set_param(motor_id, Powerstep01Command::SetParamFnSlpDecOrToffMin, 0x17);
                self.cmd_get_status(motor_id);

                // FS_SPD
                self.cmd_set_param(motor_id, Powerstep01Command::SetParamFsSpd, 0x03FF);
                self.cmd_get_status(motor_id);
                // Set the gate drivers
                //	G1
                self.cmd_set_param(motor_id, Powerstep01Command::SetParamGateCfg1, 0x00C6);
                self.cmd_get_status(motor_id);
                //	G2
                self.cmd_set_param(motor_id, Powerstep01Command::SetParamGateCfg2, 0x0020);
                self.cmd_get_status(motor_id);
                // T_FAST
                self.cmd_set_param(motor_id, Powerstep01Command::SetParamStSlpOrTFast, 0x35);
                self.cmd_get_status(motor_id);

                //TVAL_RUN
                self.cmd_set_param(motor_id, Powerstep01Command::SetParamKValRunOrTValRun, 0x16);
                self.cmd_get_status(motor_id);
                // TVAL_ACC
                self.cmd_set_param(motor_id, Powerstep01Command::SetParamKValAccOrTValAcc, 0x16);
                self.cmd_get_status(motor_id);

                //TVAL_HOLD
                self.cmd_set_param(motor_id, Powerstep01Command::SetParamKValHoldOrTValHold, 0x01);
                self.cmd_get_status(motor_id);
                self.Powerstep01_Board_Delay(10);
                self.cmd_get_status(motor_id);
                self.cmd_set_param(motor_id, Powerstep01Command::SetParamKValHoldOrTValHold, 0x02);
                self.cmd_get_status(motor_id);
                self.Powerstep01_Board_Delay(10);
                self.cmd_get_status(motor_id);
                self.cmd_set_param(motor_id, Powerstep01Command::SetParamKValHoldOrTValHold, 0x05);
                self.cmd_get_status(motor_id);
                //TVAL_DEC
                self.cmd_set_param(motor_id, Powerstep01Command::SetParamKValDecOrTValDec, 0x16);
                self.cmd_get_status(motor_id);

                // set the over current threshold to v low
                self.cmd_set_param(motor_id, Powerstep01Command::SetParamOcdTh, 0x05);
                self.cmd_get_status(motor_id);

                // set acc and dec //2BC
                self.cmd_set_param(motor_id, Powerstep01Command::SetParamAcc, 0x408);
                self.cmd_get_status(motor_id);
                self.cmd_set_param(motor_id, Powerstep01Command::SetParamDec, 0x408);
                self.cmd_get_status(motor_id);
                self.cmd_set_param(motor_id, Powerstep01Command::SetParamMaxSpeed, 0x84);
                self.cmd_get_status(motor_id);

                // CONFIG
                self.cmd_set_param(motor_id, Powerstep01Command::SetParamConfig, 0x1680);
                self.hard_stop(motor_id);

                // clear status register
                self.cmd_get_status(motor_id);
            }
            _ => {
                error!("Invalid motor number {}", motor_number);
            }
        }

// TODO
//         self.cmd_set_param(motor_id, Powerstep01Command::SetParamFnSlpDecOrTonMin, 0x0F);
//         self.cmd_get_status(motor_id);
//         self.cmd_set_param(motor_id, Powerstep01Command::SetParamFsSpd, 0x03FF);
//         self.cmd_get_status(motor_id);
//         self.cmd_set_param(motor_id, Powerstep01Command::SetParamGateCfg1, 0x0096);
//         self.cmd_get_status(motor_id);
//         self.cmd_set_param(motor_id, Powerstep01Command::SetParamGateCfg2, 0x20);
//         self.cmd_get_status(motor_id);
//         self.cmd_set_param(motor_id, Powerstep01Command::SetParamStSlpOrTFast, 0x35);
//         self.cmd_get_status(motor_id);
//         self.cmd_set_param(motor_id, Powerstep01Command::SetParamKValRunOrTValRun, 0x03);
//         self.cmd_get_status(motor_id);
//         self.cmd_set_param(motor_id, Powerstep01Command::SetParamKValAccOrTValAcc, 0x03);
//         self.cmd_get_status(motor_id);
//         self.cmd_set_param(motor_id, Powerstep01Command::SetParamKValHoldOrTValHold, 0x03);
//         self.cmd_get_status(motor_id);
//         self.cmd_set_param(motor_id, Powerstep01Command::SetParamKValDecOrTValDec, 0x03);
//         self.cmd_get_status(motor_id);
//         self.cmd_set_param(motor_id, Powerstep01Command::SetParamOcdTh, 0x01);
//         self.cmd_get_status(motor_id);
//         self.cmd_set_param(motor_id, Powerstep01Command::SetParamMaxSpeed, 0x0106);
//         self.cmd_get_status(motor_id);
//         self.cmd_set_param(motor_id, Powerstep01Command::SetParamConfig, 0x2280);
//         self.hard_stop(motor_id);
//         self.cmd_get_status(motor_id);
    }

    pub fn fetch_and_clear_all_status(&mut self) {
        info!("Calling Powerstep01_CmdGetStatus");
        // self.Powerstep01_FetchAndClearAllStatus();
    }

    pub fn hard_hi_z(&mut self, device_id: u8) {
        self.Powerstep01_CmdHardHiZ(device_id).ok();
    }

    pub fn hard_stop(&mut self, device_id: u8) {
        self.Powerstep01_SendCommand(device_id, POWERSTEP01_HARD_STOP, 0).ok();
    }

    pub async fn get_position(&mut self, device_id: u8) -> i32 {
        info!("Calling Powerstep01_CmdGetParam: ABS_POS");
        let abs_pos = self.cmd_get_param(device_id, Powerstep01Command::GetParamAbsPos);
        self.convert_position(abs_pos)
    }

    pub fn cmd_get_status(&mut self, _device_id: u8) -> u16 {
        // unsafe { Powerstep01_CmdGetStatus(device_id) }
        0
    }

    pub fn cmd_get_param(&mut self, device_id: u8, param: Powerstep01Command) -> u32 {
        self.Powerstep01_CmdGetParam(device_id, param as u8)
    }

    fn cmd_set_param(&mut self, device_id: u8, param: Powerstep01Command, value: u32) {
        self.Powerstep01_CmdSetParam(device_id, param as u8, value)
    }

    pub fn convert_position(&mut self, abs_position_reg: u32) -> i32 {
        if abs_position_reg & POWERSTEP01_ABS_POS_SIGN_BIT_MASK != 0 {
            let reg = !abs_position_reg + 1;
            let val = (reg & POWERSTEP01_ABS_POS_VALUE_MASK) as i32;
            -val
        } else {
            abs_position_reg as i32
        }
    }

    pub fn device_busy(&mut self, device_id: u8) -> bool {
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

    #[allow(non_snake_case)]
    pub fn Powerstep01_ErrorHandler(error_code: u16) {
        error!("Powerstep01 Error: {}", error_code);
    }

    #[allow(non_snake_case)]
    pub fn Powerstep01_WriteBytes(&mut self, motor_id: u8, pByteToTransmit: &u8, pReceivedByte: &mut u8) -> Result<(), u8> {
        self.chip_select_motor(motor_id, true);
        let result = self.controller.spi_transfer(pByteToTransmit, pReceivedByte);
        self.chip_select_motor(motor_id, false);
        result
    }

    #[allow(non_snake_case)]
    pub fn Powerstep01_SendCommand(&mut self, motor_id: u8, command: u8, mut value: u32) -> Result<(), u8> {
        let mut num_arg_bytes: usize = 0;
        let spi_id = (self.num_motors() - 1 - motor_id) as usize;

        // clear tx buffer for this motor
        for i in 0..POWERSTEP01_CMD_ARG_MAX_NB_BYTES {
            self.spi_tx_buf[spi_id][i] = POWERSTEP01_NOP;
        }

        let masked_command = command & DAISY_CHAIN_COMMAND_MASK;
        if masked_command == POWERSTEP01_GO_TO_DIR {
            value &= POWERSTEP01_ABS_POS_VALUE_MASK;
        }
        match masked_command
        {
            POWERSTEP01_GO_TO |
            POWERSTEP01_GO_TO_DIR |
            POWERSTEP01_RUN |
            POWERSTEP01_MOVE |
            POWERSTEP01_GO_UNTIL |
            POWERSTEP01_GO_UNTIL_ACT_CPY => {
                self.spi_tx_buf[spi_id][0]= command;
                self.spi_tx_buf[spi_id][1]= (value >> 16) as u8;
                self.spi_tx_buf[spi_id][2]= (value >> 8) as u8;
                self.spi_tx_buf[spi_id][3]= value as u8;
                num_arg_bytes = 3;
            }
            _ => {
                self.spi_tx_buf[spi_id][0] = POWERSTEP01_NOP;
                self.spi_tx_buf[spi_id][1] = POWERSTEP01_NOP;
                self.spi_tx_buf[spi_id][2] = POWERSTEP01_NOP;
                self.spi_tx_buf[spi_id][3] = command;
            }
        }

        self.write_tx_buffer_to_spi(motor_id, num_arg_bytes, spi_id);

        Ok(())
    }

    #[allow(non_snake_case)]
    pub fn Powerstep01_CmdSetParam(&mut self, motor_id: u8, param: u8, value: u32) {
        info!("Calling Powerstep01_CmdSetParam: param={:?}, value=0x{:X}", param, value);
        let num_arg_bytes: usize;
        let spi_id = (self.num_motors() - 1 - motor_id) as usize;

        // clear tx buffer for this motor
        for i in 0..POWERSTEP01_CMD_ARG_MAX_NB_BYTES {
            self.spi_tx_buf[spi_id][i] = POWERSTEP01_NOP;
        }

        match param {
            POWERSTEP01_ABS_POS |
            POWERSTEP01_MARK => {
                self.spi_tx_buf[spi_id][0] = POWERSTEP01_SET_PARAM | (param as u8);
                self.spi_tx_buf[spi_id][1] = (value >> 16) as u8;
                self.spi_tx_buf[spi_id][2] = (value >> 8) as u8;
                num_arg_bytes = 3;
            }
            POWERSTEP01_EL_POS| POWERSTEP01_ACC| POWERSTEP01_DEC|
            POWERSTEP01_MAX_SPEED| POWERSTEP01_MIN_SPEED| POWERSTEP01_FS_SPD |
            POWERSTEP01_INT_SPD | POWERSTEP01_CONFIG | POWERSTEP01_GATECFG1 => {
                self.spi_tx_buf[spi_id][1] = POWERSTEP01_SET_PARAM | (param as u8);
                self.spi_tx_buf[spi_id][2] = (value >> 8) as u8;
                num_arg_bytes = 2;
            }
            _ => {
                self.spi_tx_buf[spi_id][2] = POWERSTEP01_SET_PARAM | (param as u8);
                num_arg_bytes = 1;
            }
        }

        self.spi_tx_buf[spi_id][3] = (value) as u8;

        self.write_tx_buffer_to_spi(motor_id, num_arg_bytes, spi_id);
    }

    #[allow(non_snake_case)]
    pub fn Powerstep01_CmdGetParam(&mut self, motor_id: u8, param: u8) -> u32 {
        info!("Calling Powerstep01_CmdGetParam: param={:?}", param);
        let num_arg_bytes: usize;
        let spi_id = (self.num_motors() - 1 - motor_id) as usize;
        // clear tx and rx buffers for this motor
        for i in 0..POWERSTEP01_CMD_ARG_MAX_NB_BYTES {
            self.spi_tx_buf[spi_id][i] = POWERSTEP01_NOP;
            self.spi_rx_buf[spi_id][i] = 0;
        }

        match param {
            POWERSTEP01_ABS_POS | POWERSTEP01_MARK | POWERSTEP01_SPEED => {
                self.spi_tx_buf[spi_id][0] = POWERSTEP01_GET_PARAM | param;
                num_arg_bytes = 3;
            }
            POWERSTEP01_EL_POS |
            POWERSTEP01_ACC |
            POWERSTEP01_DEC |
            POWERSTEP01_MAX_SPEED |
            POWERSTEP01_MIN_SPEED |
            POWERSTEP01_FS_SPD |
            POWERSTEP01_INT_SPD |
            POWERSTEP01_CONFIG |
            POWERSTEP01_GATECFG1 |
            POWERSTEP01_STATUS => {
                self.spi_tx_buf[spi_id][1] = POWERSTEP01_GET_PARAM | param;
                num_arg_bytes = 2;
            }
            _ => {
                self.spi_tx_buf[spi_id][2] = POWERSTEP01_GET_PARAM | param;
                num_arg_bytes = 1;
            }
        }

        self.write_tx_buffer_to_spi(motor_id, num_arg_bytes, spi_id);

        // get the returned value
        let value3 = self.spi_tx_buf[spi_id][3] as u32;
        let value2 = (self.spi_tx_buf[spi_id][2] as u32) << 8;
        let value1 = (self.spi_tx_buf[spi_id][1] as u32) << 16;
        value1 | value2 | value3
    }

    fn write_tx_buffer_to_spi(&mut self, motor_id: u8, num_arg_bytes: usize, spi_id: usize) {
        // write the bytes
        for id in (POWERSTEP01_CMD_ARG_MAX_NB_BYTES - 1 - num_arg_bytes)..POWERSTEP01_CMD_ARG_MAX_NB_BYTES {
            let data_to_send = self.spi_tx_buf[spi_id][id];
            let mut data_to_recv = self.spi_rx_buf[spi_id][id];
            self.Powerstep01_WriteBytes(motor_id, &data_to_send,
                                        &mut data_to_recv).expect("Powerstep01_WriteBytes failed");
            self.spi_rx_buf[spi_id][id] = data_to_recv;
        }
    }

    #[allow(non_snake_case)]
    pub fn Powerstep01_CmdMove(&mut self, motor_id: u8,
                               direction: MotorDirection,
                               n_step: u32) -> Result<(), u8> {
        self.Powerstep01_SendCommand(motor_id,
                                     POWERSTEP01_MOVE as u8 | direction as u8,
                                     n_step)
    }

    #[allow(non_snake_case)]
    pub fn Powerstep01_ResetPos(&mut self, motor_id: u8) -> Result<(), u8> {
        self.Powerstep01_SendCommand(motor_id, POWERSTEP01_RESET_POS as u8, 0)
    }

    #[allow(non_snake_case)]
    pub fn Powerstep01_CmdGoHome(&mut self, motor_id: u8) -> Result<(), u8> {
        self.Powerstep01_SendCommand(motor_id, POWERSTEP01_GO_HOME as u8, 0)
    }

    #[allow(non_snake_case)]
    pub fn Powerstep01_CmdReleaseSw(&mut self, motor_id: u8,
                                  direction: MotorDirection) -> Result<(), u8> {
        self.Powerstep01_SendCommand(motor_id,
                                     POWERSTEP01_RELEASE_SW as u8 | direction as u8,
                                     0)
    }

    #[allow(non_snake_case)]
    pub fn Powerstep01_CmdGoUntil(&mut self, motor_id: u8,
                                  direction: MotorDirection,
                                  speed: u32) -> Result<(), u8> {
        self.Powerstep01_SendCommand(motor_id,
                                     POWERSTEP01_GO_UNTIL as u8 | direction as u8,
                                     speed)
    }

    #[allow(non_snake_case)]
    pub fn Powerstep01_CmdHardHiZ(&mut self, motor_id: u8) -> Result<(), u8> {
        self.Powerstep01_SendCommand(motor_id, POWERSTEP01_HARD_HIZ as u8, 0)
    }

    #[allow(non_snake_case)]
    pub fn Powerstep01_CmdSoftHiZ(&mut self, motor_id: u8) -> Result<(), u8> {
        self.Powerstep01_SendCommand(motor_id, POWERSTEP01_SOFT_HIZ as u8, 0)
    }

    #[allow(non_snake_case)]
    pub fn Powerstep01_Board_ReleaseReset(&mut self) {
        self.controller.reset(false);
    }

    #[allow(non_snake_case)]
    pub fn Powerstep01_Board_MotorStop(&mut self, motor_id: u8) {
        self.stop_motor(motor_id, true);
    }

    #[allow(non_snake_case)]
    pub fn Powerstep01_Board_MotorStart(&mut self, motor_id: u8) {
        self.stop_motor(motor_id, true);
    }

    #[allow(non_snake_case)]
    pub fn Powerstep01_Reset(&mut self) {
        self.controller.reset(true);
    }

    #[allow(non_snake_case)]
    pub fn Powerstep01_Board_Delay(&mut self, milliseconds: u32) {
        self.controller.delay_ms(milliseconds);
    }

    #[allow(non_snake_case)]
    pub fn Powerstep01_Init(&mut self)
    {
        info!("Powerstep01_Init");
        //   // configure the step clock
        //   Powerstep01_Board_StepClockInit();

        // standby-reset deactivation
        info!("Powerstep01_Board_ReleaseReset");
        self.Powerstep01_Board_ReleaseReset();

        // delay after reset
        info!("Powerstep01_Board_Delay");
        self.Powerstep01_Board_Delay(1);

        // Set all registers to their predefined values from powerstep01_target_config.h
        // Powerstep01_SetRegisterToPredefinedValues();
        // Powerstep01_SetRegisterToPredefinedValues(powerstep01DriverInstance);
        // or
        // Powerstep01_SetDeviceParamsToGivenValues(powerstep01DriverInstance, (powerstep01_Init_u_t*)pInit);

        // Put the Powerstep01 in HiZ state
        // Powerstep01_CmdHardHiZ();

        // clear status for each motor
        self.Powerstep01_FetchAndClearAllStatus();
    }

    #[allow(non_snake_case)]
    pub fn Powerstep01_CmdGetStatus(&mut self, motor_id: u8) -> u16 {
        info!("Powerstep01_CmdGetStatus");
        let num_arg_bytes = POWERSTEP01_CMD_ARG_MAX_NB_BYTES - 1;
        let spi_id = (self.num_motors() - 1 - motor_id) as usize;
        self.spi_tx_buf[spi_id][0] = POWERSTEP01_GET_STATUS;
        self.spi_tx_buf[spi_id][1] = POWERSTEP01_NOP;
        self.spi_tx_buf[spi_id][2] = POWERSTEP01_NOP;
        self.spi_tx_buf[spi_id][3] = POWERSTEP01_NOP;
        self.spi_rx_buf[spi_id][0] = 0;
        self.spi_rx_buf[spi_id][1] = 0;
        self.spi_rx_buf[spi_id][2] = 0;
        self.spi_rx_buf[spi_id][3] = 0;

        self.write_tx_buffer_to_spi(motor_id, num_arg_bytes, spi_id);
        ((self.spi_rx_buf[spi_id][1]as u16) << 8) | (self.spi_rx_buf[spi_id][2] as u16)
    }

    #[allow(non_snake_case)]
    pub fn Powerstep01_FetchAndClearAllStatus(&mut self) {
        info!("Powerstep01_FetchAndClearAllStatus");
        for motor_id in 0..self.num_motors() {
            let status = self.Powerstep01_CmdGetStatus(motor_id);
            info!("Motor ID {} Status: 0x{:04X}", motor_id, status);
        }
    }

    #[allow(non_snake_case)]
    pub fn Powerstep01_IsDeviceBusy(&mut self, motor_id: u8) -> bool {
        info!("Powerstep01_IsDeviceBusy");
        let status = self.Powerstep01_CmdGetStatus(motor_id);
        info!("Motor ID {} Status: 0x{:04X}", motor_id, status);
        (status & POWERSTEP01_STATUS_BUSY) == 0     // BUSY bit is 0 when busy
    }
}
