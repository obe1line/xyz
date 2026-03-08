// This is a mock implementation of the embassy interfaces for testing purposes.
// Required so that the motor code builds and unit tests can be run.
// Otherwise, fails because SPI is not implemented for the target architecture.
// TODO: Modify so the SPI has a trait that can be mocked.
use core::fmt::Debug;
use log::info;

///////////////////
pub trait SealedMode {}

pub trait Word {
    /// Word size
    fn size() -> u16;
    /// Amount of bits of this word size.
    fn bits() -> usize;
}

impl Word for u8 {
    fn size() -> u16 {
        1
    }
    fn bits() -> usize {
        8
    }
}

pub struct Blocking;
impl SealedMode for Blocking {}

#[derive(Debug, PartialEq, Eq, Clone, Copy)]
pub enum Error {
    /// Invalid framing.
    Framing,
    /// CRC error (only if hardware CRC checking is enabled).
    Crc,
    /// Mode fault
    ModeFault,
    /// Overrun.
    Overrun,
}

pub struct Spi<'d, M: SealedMode> {
    _mode: core::marker::PhantomData<&'d M>,
}

// #[automock]
impl<'d> Spi<'d, Blocking> {
    pub fn new_blocking(
    ) -> Self {
        Spi::<Blocking> {
            _mode: core::marker::PhantomData,
        }
    }
    pub fn blocking_transfer_in_place<W: Word + Debug + Copy>(&mut self, read_write: &mut [W]) -> Result<(), Error> {
        read_write[0] = read_write[0]; // Mock implementation
        Ok(())
    }

    pub fn blocking_transfer<W: Word + Debug + Copy>(&mut self, read: &mut [W], write: &[W]) -> Result<(), Error> {
        // Mock implementation of blocking SPI transfer
        // info!("Mock SPI blocking transfer");
        // info!("Read buffer before transfer: {:?}", read);
        // info!("Write buffer: {:?}", write);
        // For demonstration, let's just copy write to read (up to the length of read)
        let len = core::cmp::min(read.len(), write.len());
        for i in 0..len {
            read[i] = write[i];
        }
        // info!("Read buffer after transfer: {:?}", read);
        Ok(())
    }
}

pub struct Output<'a> {
    _pin: core::marker::PhantomData<&'a ()>,
}

impl<'a> Output<'a> {
    pub fn new_fake() -> Self {
        Output {
            _pin: core::marker::PhantomData,
        }
    }
    
    pub fn set_high(&mut self) {
        // Mock implementation of setting pin high
        info!("Mock pin set high");
    }
    pub fn set_low(&mut self) {
        // Mock implementation of setting pin low
        info!("Mock pin set low");
    }
}

pub struct Async;
pub struct UartTx<'a, Async> {
    _mode: core::marker::PhantomData<&'a Async>,
}
impl<'a> UartTx<'a, Async> {
    pub fn write(&mut self, _data: &[u8]) {
        // Mock implementation of UART write
        info!("Mock UART write");
    }
}
pub struct Duration;
impl Duration {
    pub fn from_millis(_ms: u64) -> Self {
        Duration
    }
}

pub fn block_for(_duration: Duration) {
    // Mock implementation of blocking for a duration
    info!("Mock blocking for duration");
}