#[cfg(test)]
mod motor_unit_tests {
    use crate::*;   // import everything from the main crate

    struct MockMotor {
        chip_select: bool,
        reset_flag: bool,
    }
    impl MockMotor {
        fn new() -> Self {
            MockMotor {
                chip_select: false,
                reset_flag: false
            }
        }
    }
    impl MotorController for MockMotor {
        fn reset(&mut self, reset: bool) {
            self.reset_flag = reset
        }

        fn chip_select(&mut self, select: bool) {
            self.chip_select = select
        }
    }

    struct MockStepperController {
        transfer_called: bool,
        tx_data: Vec<u8>, // to capture the data sent via SPI
    }
    impl MockStepperController {
        fn new() -> Self {
            MockStepperController {
                transfer_called: false,
                tx_data: Vec::new(),
            }
        }
    }
    impl StepperController for MockStepperController {
        fn enable_stepper_driver(&mut self, _enable: bool) {}
        fn spi_transfer(&mut self, _tx: &u8, _rx: &mut u8) -> Result<(), u8> {
            self.transfer_called = true;
            self.tx_data.push(*_tx);
            Ok(())
        }
        fn reset(&mut self, _enable: bool) {}
        fn delay_ms(&mut self, _ms: u32) {}
    }

    #[test]
    fn test_motor_init() {
        let mut motor1 = MockMotor::new();
        let motors = [
            &mut motor1 as &mut dyn MotorController,
        ];
        let mut stepper_controller = MockStepperController::new();
        let mut stepper = PowerStepControl::new(motors, &mut stepper_controller);
        assert_eq!(stepper.num_motors(), 2);

        stepper.init_motor(MOTOR_ID_0);
        // TODO: Add assertions to verify motor initialization state
    }

    #[test]
    fn test_motor_enable() {
        let mut motor = MockMotor::new();
        motor.reset(true);
        assert_eq!(motor.reset_flag, true);
        motor.reset(false);
        assert_eq!(motor.reset_flag, false);
    }

    #[test]
    fn test_motor_spi_transfer() {
        let mut motor1 = MockMotor::new();
        let motors = [
            &mut motor1 as &mut dyn MotorController,
        ];
        // let motor = MockMotorController::new();
        let mut stepper_controller = MockStepperController::new();
        let mut stepper = PowerStepControl::new(motors, &mut stepper_controller);
        // let mut stepper_controller = MockStepperController::new();
        // let mut stepper = PowerStepControl::new(1, &mut stepper_controller);
        // let motor_id = stepper.add_motor(MockMotorController::new());
        // assert_eq!(motor_id, MOTOR_ID_0);

        let result = stepper.Powerstep01_CmdGoHome(MOTOR_ID_0);
        assert!(result.is_ok());
        assert!(stepper.controller.transfer_called);
        let data = stepper.controller.tx_data.as_slice();
        assert_eq!(data.len(), 1);
        assert_eq!(data.get(0), Some(&0x70u8));

        stepper.Powerstep01_CmdMove(MOTOR_ID_0, MOTOR_DIR_FORWARD, 100).ok();
        let data = stepper.controller.tx_data.as_slice();
        assert_eq!(data.len(), 5);
        assert_eq!(data.get(0), Some(&0x70u8));
        assert_eq!(data.get(1), Some(&0x41u8));
        assert_eq!(data.get(2), Some(&0x00u8));
        assert_eq!(data.get(3), Some(&0x00u8));
    }
}