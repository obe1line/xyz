#![no_std]
#![no_main]
use core::fmt;
use cortex_m::prelude::_embedded_hal_blocking_serial_Write;
use defmt::{info};
use embassy_executor::Spawner;
use embassy_futures::select::select;
use embassy_stm32::{bind_interrupts, i2c, peripherals, usart, Peri, Peripherals};
use embassy_stm32::exti::ExtiInput;
use embassy_stm32::gpio::{Output, Level, Speed, Pull};
use embassy_stm32::i2c::{I2c};
use embassy_stm32::mode::{Async, Blocking};
use embassy_stm32::usart::Uart;
use embassy_time::{Duration, Ticker, Timer};
use {defmt_rtt as _, panic_probe as _};

bind_interrupts!(struct IrqsI2C1 {
    I2C1 => i2c::ErrorInterruptHandler<peripherals::I2C1>,
            i2c::EventInterruptHandler<peripherals::I2C1>;
});

const LLS_DEVICE_I2C_ADDR: u8 = 0x2B; // LLS device I2C address (ADDR pin high)

#[cfg(feature = "embedded")]
#[embassy_executor::task]
async fn heartbeat_task(mut led: Output<'static>) -> ! {
    loop {
        led.toggle();
        Timer::after(Duration::from_millis(1000)).await;
    }
}

async fn uart_msg(uart1: &mut Uart<'_, Async>, args: fmt::Arguments<'_>) {
    let mut out_buffer: [u8; 64] = [0; 64];
    let s = format_no_std::show(&mut out_buffer, args).expect("write buffer failed");
    uart1.blocking_write(s.as_bytes()).expect("buffered write_all failed");
    uart1.flush().await.expect("buffered flush failed");
}

async fn uart_dbg(uart2: &mut Uart<'_, Blocking>, args: fmt::Arguments<'_>) {
    let mut out_buffer: [u8; 64] = [0; 64];
    let s = format_no_std::show(&mut out_buffer, args).expect("write buffer failed");
    uart2.bwrite_all(s.as_bytes()).expect("buffered write_all failed");
    uart2.bflush().expect("buffered flush failed");
}

#[embassy_executor::main]
async fn main(spawner: Spawner) {
    let p: Peripherals = embassy_stm32::init(Default::default());

    // start the heartbeat LED blinker task
    let heartbeat_led = Output::new(p.PB1, Level::High, Speed::Low);
    let result = spawner.spawn(heartbeat_task(heartbeat_led));
    match result {
        Ok(_) => info!("Started heartbeat task"),
        Err(e) => error!("Failed to start heartbeat task: {:?}", e),
    };

    let i2c1_scl = p.PB8; // I2C1_SCL
    let i2c1_sda = p.PB9; // I2C1_SDA

    let mut i2c1 = I2c::new(
        p.I2C1,
        i2c1_scl,
        i2c1_sda,
        IrqsI2C1,
        p.DMA1_CH2,     // I2C1_TX (DMAMUX1)
        p.DMA1_CH3,     // I2C1_RX (DMAMUX1)
        Default::default(),
    );

    // UART1 setup
    let uart1_tx = p.PA9;
    let uart1_rx = p.PA10;
    bind_interrupts!(struct IrqsUart1 {
		 USART1 => usart::InterruptHandler<peripherals::USART1>;
	});
    let uart1_irq = IrqsUart1;
    let tx1_dma = p.DMA1_CH4;
    let rx1_dma = p.DMA1_CH5;
    let mut cfg1 = usart::Config::default();
    cfg1.baudrate = 9600;
    cfg1.data_bits= usart::DataBits::DataBits8;
    cfg1.parity = usart::Parity::ParityNone;
    cfg1.stop_bits = usart::StopBits::STOP1;
    let mut uart1 = usart::Uart::new(p.USART1, uart1_rx, uart1_tx,
                                     uart1_irq, tx1_dma, rx1_dma, cfg1).expect("uart1 failed");

    // UART2 setup
    let uart2_tx = p.PA2;
    let uart2_rx = p.PA3;
    bind_interrupts!(struct Irqs2 {
		 USART2 => usart::InterruptHandler<peripherals::USART2>;
	});
    let mut cfg2 = usart::Config::default();
    cfg2.baudrate = 9600;
    cfg2.data_bits= usart::DataBits::DataBits8;
    cfg2.parity = usart::Parity::ParityNone;
    cfg2.stop_bits = usart::StopBits::STOP1;
    let mut uart2 = usart::Uart::new_blocking(p.USART2, uart2_rx, uart2_tx, cfg2).expect("uart2 failed");

    // wait for 10 seconds before starting
    Timer::after_secs(10).await;

    let version = "LLS v0.1.1";
    uart_msg(&mut uart1, format_args!("{} started\r\n", version)).await;
    uart_dbg(&mut uart2, format_args!("{} started\r\n", version)).await;

    // release notes
    uart_dbg(&mut uart2, format_args!("...Fix: Corrected the I2C address on writes.\r\n")).await;
    uart_dbg(&mut uart2, format_args!("...Add: Read and report the device and manufacturer.\r\n")).await;
    uart_dbg(&mut uart2, format_args!("...Add: Configuration and sampling loop.\r\n")).await;
    uart_dbg(&mut uart2, format_args!("\r\n")).await;


    // check for I2C device presence
    for addr in 0..=127 {
        if let Ok(_) = i2c1.write(addr, &[0]).await {
            info!("Found I2C device at address: 0x{:02x}", addr);
            uart_msg(&mut uart1, format_args!("Found I2C device at address: 0x{:02x}\r\n", addr)).await;
            uart_dbg(&mut uart2, format_args!("Found I2C device at address: 0x{:02x}\r\n", addr)).await;
        }
    }

    // start the LLS sampler task
    // _spawner.spawn(lls_sampler(i2c1)).unwrap();

    // FDC2112 register addresses
    const REG_CONFIG: u8 = 0x1A;
    const REG_RESET_DEV: u8 = 0x1C;
    const REG_MANUFACTURER_ID: u8 = 0x7E;
    const REG_DEVICE_ID: u8 = 0x7F;
    const REG_RCOUNT_CH0: u8 = 0x08;			// Reference count for channel 0
    const REG_MUX_CONFIG: u8 = 0x1B;			// MUX configuration register
    const REG_DRIVE_CURRENT_CH0: u8 = 0x1E;
    const REG_ERROR_CONFIG: u8 = 0x19;			// Error configuration register
    const REG_CLOCK_DIVIDERS_CH0: u8 = 0x14;	// Clock divider for channel 0
    const REG_SETTLECOUNT_CH0: u8 = 0x10;		// Settle count for channel 0
    const REG_STATUS: u8 = 0x18;				// Status register
    const REG_DATA_CH0: u8 = 0x00;				// Channel 0 conversion result
    const REG_DATA_LSB_CH0: u8 = 0x01;			// Channel 0 conversion result LSB

    // Initialize the LLS device
    uart_dbg(&mut uart2, format_args!("Initializing LLS device...\r\n")).await;

    // Reset. At which point the device is in sleep mode - required when changing configuration
    match i2c1.blocking_write(LLS_DEVICE_I2C_ADDR, &[REG_RESET_DEV, 0x80, 0x00]) {
        Ok(_) => uart_dbg(&mut uart2, format_args!("LLS device reset\r\n")).await,
        Err(e) => uart_dbg(&mut uart2, format_args!("LLS device reset failed: {}\r\n", e)).await
    };

    // Wait for the device to reset
    Timer::after_millis(1000).await;

    // Read the device identifier
    let mut read_buffer: [u8; 2] = [0; 2];
    match i2c1.blocking_write_read(LLS_DEVICE_I2C_ADDR, &[REG_DEVICE_ID], &mut read_buffer) {
        Ok(_) => {
            uart_dbg(&mut uart2, format_args!("LLS device id: 0x{:02x}{:02x}\r\n", read_buffer[0], read_buffer[1])).await;
        },
        Err(e) => {
            uart_dbg(&mut uart2, format_args!("LLS device id read failed: {}\r\n", e)).await;
        }
    }

    // Read the manufacturer identifier
    match i2c1.blocking_write_read(LLS_DEVICE_I2C_ADDR, &[REG_MANUFACTURER_ID], &mut read_buffer) {
        Ok(_) => {
            uart_dbg(&mut uart2, format_args!("LLS manufacturer id: 0x{:02x}{:02x}\r\n", read_buffer[0], read_buffer[1])).await;
        },
        Err(e) => {
            uart_dbg(&mut uart2, format_args!("LLS manufacturer id read failed: {}\r\n", e)).await;
        }
    }

    // Set the conversion counts for channel 0
    let cfg_upper : u8 = 0x04;	// 1024 * (16/fref0)
    let cfg_lower : u8 = 0x00;	//
    match i2c1.blocking_write(LLS_DEVICE_I2C_ADDR, &[REG_RCOUNT_CH0, cfg_upper, cfg_lower]) {
        Ok(_) => uart_dbg(&mut uart2, format_args!("LLS conversion count CH0 set to 0x{:02x}{:02x}\r\n", cfg_upper, cfg_lower)).await,
        Err(e) => uart_dbg(&mut uart2, format_args!("LLS REG_RCOUNT_CH0 failed: {}\r\n", e)).await
    };

    // Set the MUX to channel 0
    let cfg_upper : u8 = 0x02;	// Continuous conversions on CONFIG.ACTIVE_CHAN (CH0)
    let cfg_lower : u8 = 0x0D;	// 10MHz input de-glitch filter bandwidth
    match i2c1.blocking_write(LLS_DEVICE_I2C_ADDR, &[REG_MUX_CONFIG, cfg_upper, cfg_lower]) {
        Ok(_) => uart_dbg(&mut uart2, format_args!("LLS MUX set\r\n")).await,
        Err(e) => uart_dbg(&mut uart2, format_args!("LLS MUX set failed: {}\r\n", e)).await
    };

    // Set the drive current for channel 0
    let cfg_upper : u8 = 0x78;	// 0.146mA
    let cfg_lower : u8 = 0x00;
    match i2c1.blocking_write(LLS_DEVICE_I2C_ADDR, &[REG_DRIVE_CURRENT_CH0, cfg_upper, cfg_lower]) {
        Ok(_) => uart_dbg(&mut uart2, format_args!("LLS CH0 drive current set\r\n")).await,
        Err(e) => uart_dbg(&mut uart2, format_args!("LLS CH0 drive current set failed: {}\r\n", e)).await
    };

    // Enable data ready interrupt
    let cfg_upper : u8 = 0x00;
    let cfg_lower : u8 = 0x01;	// DRDY_2INT = Assert INTB pin and update status register on data ready
    match i2c1.blocking_write(LLS_DEVICE_I2C_ADDR, &[REG_ERROR_CONFIG, cfg_upper, cfg_lower]) {
        Ok(_) => uart_dbg(&mut uart2, format_args!("LLS data-ready behaviour set\r\n")).await,
        Err(e) => uart_dbg(&mut uart2, format_args!("LLS data ready behaviour set failed: {}\r\n", e)).await
    };

    // Set the clock divider for channel 0
    let cfg_upper: u8 = 0x10;	// Divide by 1 (for frequencies 0.01MHz to 8.75MHz)
    let cfg_lower: u8 = 0x00;
    match i2c1.blocking_write(LLS_DEVICE_I2C_ADDR, &[REG_CLOCK_DIVIDERS_CH0, cfg_upper, cfg_lower]) {
        Ok(_) => uart_dbg(&mut uart2, format_args!("LLS clock divider CH0 set\r\n")).await,
        Err(e) => uart_dbg(&mut uart2, format_args!("LLS clock divider CH0 set failed: {}\r\n", e)).await
    };

    // Set the settling time for channel 0
    let cfg_upper: u8 = 0x04;	// check this value
    let cfg_lower: u8 = 0x00;
    match i2c1.blocking_write(LLS_DEVICE_I2C_ADDR, &[REG_SETTLECOUNT_CH0, cfg_upper, cfg_lower]) {
        Ok(_) => uart_dbg(&mut uart2, format_args!("LLS CH0 settle count set\r\n")).await,
        Err(e) => uart_dbg(&mut uart2, format_args!("LLS CH0 settle count set failed: {}\r\n", e)).await
    };

    // Switch from sleep to active mode
    //
    let cfg_upper: u8 = 0x1E;	// Active, continuous conversions on CH0, Ref freq on CLKIN pin
    let cfg_lower: u8 = 0x01;	// low power, assert INTB on status reg update, normal current
    match i2c1.blocking_write(LLS_DEVICE_I2C_ADDR, &[REG_CONFIG, cfg_upper, cfg_lower]) {
        Ok(_) => uart_dbg(&mut uart2, format_args!("LLS device running\r\n")).await,
        Err(e) => uart_dbg(&mut uart2, format_args!("LLS device config failed: {}\r\n", e)).await
    };

    let mut lls_data_ready = ExtiInput::new(p.PB5, p.EXTI5, Pull::Up);

    let mut count: u32 = 0;
    loop {
        count += 1;
        uart_msg(&mut uart1, format_args!("Count: {}\r\n", count)).await;
        uart_dbg(&mut uart2, format_args!("Count: {}\r\n", count)).await;

        // Wait for the data ready interrupt to trigger
        // This will block until the interrupt is triggered
        let mut triggered = false;
        select(
            async {
                // Wait for the data ready interrupt to trigger
                lls_data_ready.wait_for_rising_edge().await;
                triggered = true;
            },
            async {
                // Wait for 100 milliseconds between samples
                Timer::after(Duration::from_millis(100)).await;
            }
        ).await;

        if !triggered {
            uart_dbg(&mut uart2, format_args!("LLS data ready timed out\r\n")).await;
            // TODO: once interrupts working, uncomment the next line to only sampling when ready
            // continue;
        }

        // Example output:
        // Status: 0x0048			= No errors, Data Ready, Conversion data unread
        // Data CH0: 0x020e			= CH0 value is 0x20eeb08
        // Data LSB CH0: 0xeb08
        let mut read_buffer: [u8; 2] = [0; 2];
        match i2c1.blocking_write_read(LLS_DEVICE_I2C_ADDR, &[REG_STATUS], &mut read_buffer) {
            Ok(_) => {
                uart_dbg(&mut uart2, format_args!("LLS REG_STATUS: 0x{:02x}{:02x}\r\n", read_buffer[0], read_buffer[1])).await;
            },
            Err(e) => {
                uart_dbg(&mut uart2, format_args!("LLS REG_STATUS read failed: {}\r\n", e)).await;
            }
        }

        match i2c1.blocking_write_read(LLS_DEVICE_I2C_ADDR, &[REG_DATA_CH0], &mut read_buffer) {
            Ok(_) => {
                uart_dbg(&mut uart2, format_args!("LLS REG_DATA_CH0: 0x{:02x}{:02x}\r\n", read_buffer[0], read_buffer[1])).await;
            },
            Err(e) => {
                uart_dbg(&mut uart2, format_args!("LLS REG_DATA_CH0 read failed: {}\r\n", e)).await;
            }
        }
        match i2c1.blocking_write_read(LLS_DEVICE_I2C_ADDR, &[REG_DATA_LSB_CH0], &mut read_buffer) {
            Ok(_) => {
                uart_dbg(&mut uart2, format_args!("LLS REG_DATA_LSB_CH0: 0x{:02x}{:02x}\r\n", read_buffer[0], read_buffer[1])).await;
            },
            Err(e) => {
                uart_dbg(&mut uart2, format_args!("LLS REG_DATA_LSB_CH0 read failed: {}\r\n", e)).await;
            }
        }
    }
}
