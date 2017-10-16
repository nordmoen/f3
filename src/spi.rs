//! Serial Peripheral Interface
//!
//! You can use the `Spi` interface with these SPI instances
//!
//! # SPI1
//! - NSS = PA4
//! - SCK = PA5
//! - MISO = PA6
//! - MOSI = PA7
//!
//! # SPI2
//! - NSS = PB12
//! - SCK = PB13
//! - MISO = PB14
//! - MOSI = PB15

use core::any::Any;
use core::ops::Deref;
use core::ptr;

use hal;
use nb;
use stm32f30x::{spi1, gpioa, SPI1, GPIOA, RCC};

/// SPI instance that can be used with the `Spi` interface
pub unsafe trait SPI: Deref<Target = spi1::RegisterBlock> {
    /// GPIO block associated to this SPI instance
    type GPIO: Deref<Target = gpioa::RegisterBlock>;
}

unsafe impl SPI for SPI1 {
    type GPIO = GPIOA;
}

/// SPI result
pub type Result<T> = ::core::result::Result<T, nb::Error<Error>>;

/// SPI error
#[derive(Debug)]
pub enum Error {
    /// Overrun occurred
    Overrun,
    /// Mode fault occurred
    ModeFault,
    /// CRC error
    Crc,
    #[doc(hidden)]
    _Extensible,
}

/// Serial Peripheral Interface
pub struct Spi<'a, S>(pub &'a S)
where
    S: Any + SPI;

impl<'a, S> Spi<'a, S>
where
    S: Any + SPI,
{
    /// Initialize the SPI
    pub fn init(&self, gpio: &S::GPIO, rcc: &RCC) {
        let spi = self.0;
        // Enable GPIOA
        rcc.ahbenr.modify(|_, w| w.iopaen().enabled());
        // Enable SPI1
        rcc.apb2enr.modify(|_, w| w.spi1en().enabled());

        // SCK = PA5 = Alternate function push pull
        // MISO = PA6 = Floating input
        // MOSI = PA7 = Alternate function push pull
        gpio.afrl.modify(|_, w| unsafe {
            w.afrl6().bits(5)
                .afrl5().bits(5)
                .afrl7().bits(5)
        });
        gpio.moder.modify(|_, w| {
            w.moder5().alternate()
                .moder6().alternate()
                .moder7().alternate()
        });
        // cpha: second clock transition is the first data capture
        // cpol: CK to 1 when idle
        // mstr: master configuration
        // br: 1 MHz frequency
        // lsbfirst: MSB first
        // ssm: disable software slave management
        // dff: 8 bit frames
        // bidimode: 2-line unidirectional
        spi.cr1.write(|w| unsafe {
            w.cpha().clear_bit()
                .cpol().clear_bit()
                .mstr().set_bit()
                .br().bits(0b010)
                .lsbfirst().clear_bit()
                .ssi().set_bit()
                .ssm().set_bit()
                .rxonly().clear_bit()
                .bidimode().clear_bit()
        });
        // Enable SS output
        spi.cr2.modify(|_, w| unsafe {
            w.ssoe().clear_bit()
                .frxth().set_bit()
                .ds().bits(0b0111)
        });
    }

    /// Disable SPI
    ///
    /// **NOTE** This drives the NSS pin high
    pub fn disable(&self) {
        self.0.cr1.modify(|_, w| w.spe().clear_bit());
    }

    /// Enable SPI
    ///
    /// **NOTE** This drives the NSS pin low
    pub fn enable(&self) {
        self.0.cr1.modify(|_, w| w.spe().set_bit());
    }
}

impl<'a, S> hal::Spi<u8> for Spi<'a, S>
where
    S: Any + SPI,
{
    type Error = Error;

    fn read(&self) -> Result<u8> {
        let spi = self.0;
        // Read status register
        let sr = spi.sr.read();
        // Check for errors on read
        if sr.ovr().bit_is_set() {
            Err(nb::Error::Other(Error::Overrun))
        } else if sr.modf().bit_is_set() {
            Err(nb::Error::Other(Error::ModeFault))
        } else if sr.crcerr().bit_is_set() {
            Err(nb::Error::Other(Error::Crc))
        } else if sr.rxne().bit_is_set() {
            //Ok(spi.dr.read().dr().bits() as u8)
            Ok(unsafe {
                ptr::read_volatile(&spi.dr as *const _ as *const u8)
            })
        } else {
            // There was no error, but there was no data either
            Err(nb::Error::WouldBlock)
        }
    }

    fn send(&self, byte: u8) -> Result<()> {
        let spi = self.0;
        // Read status register
        let sr = spi.sr.read();
        // Check for errors on read
        if sr.ovr().bit_is_set() {
            Err(nb::Error::Other(Error::Overrun))
        } else if sr.modf().bit_is_set() {
            Err(nb::Error::Other(Error::ModeFault))
        } else if sr.crcerr().bit_is_set() {
            Err(nb::Error::Other(Error::Crc))
        } else if sr.txe().bit_is_set() {
            unsafe {
                //spi.dr.write(|w| w.dr().bits(byte as u16))
                ptr::write_volatile(&spi.dr as *const _ as *mut u8, byte)
            }
            Ok(())
        } else {
            // There was no error, but SPI not ready to write yet
            Err(nb::Error::WouldBlock)
        }
    }
}
