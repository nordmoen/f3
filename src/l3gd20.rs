//! L3GD20 - Gyroscope
//!
//! This sensor is connected to the SPI bus via these pins:
//!
//! - Serial Clock (`SCK`) pin - `PA5`
//! - Master Input (`MISO`) pin - `PA6`
//! - Master Output (`MOSI`) pin - `PA7`
//! - `CS` pin - `PE3`

use core::default::Default;
use core::num;
use hal::Spi;
use nb;
use spi;
use stm32f30x::{SPI1, GPIOE};

/// Read/Write multiple registers
const MS: u8            = 1 << 6;
/// Sensor ID stored in `WHO_AM_I` register
const SENSOR_ID: u8     = 0b11010100;
/// Junk data used to initiate sensor read
const JUNK_DATA: u8     = 0x00;
/// Command for `CTRL_REG1` to enable sensor on all axis
const ENABLE_SENSOR: u8 = 0x0F;
/// Enable interrupt on GPIOE pin 1 when Data is ReaDY
const ENABLE_DRDY: u8   = 0x08;
/// Block Data Update
//const BDU: u8           = 1 << 7;

/// Gyroscope result
pub type Result<T> = ::core::result::Result<T, nb::Error<Error>>;

/// Gyroscope
pub struct L3GD20<'a>(pub &'a spi::Spi<'a, SPI1>, pub &'a GPIOE);

/// Gyroscope measurement
pub struct Measurement {
    /// Angular X velocity in RAD/S
    pub x: f32,
    /// Angular Y velocity in RAD/S
    pub y: f32,
    /// Angular Z velocity in RAD/S
    pub z: f32,
}

/// Gyroscope errors
#[derive(Debug)]
pub enum Error {
    /// SPI communication error
    Spi(spi::Error),
}

/// Output Data Rate
#[repr(u8)]
pub enum ODR {
    /// Output at 95 Hz
    Hz95  = 0x00,
    /// Output at 190 Hz
    Hz190 = 0x40,
    /// Output at 380 Hz
    Hz380 = 0x80,
    /// Output at 760 Hz
    Hz760 = 0xC0,
}

/// Cut-Off frequency low pass filter
#[repr(u8)]
pub enum CutOff {
    /// For `ODR` of 760 cut-off at 30
    Freq30  = 0x00,
    /// For `ODR` of 760 cut-off at 35
    Freq35  = 0x10,
    /// For `ODR` of 760 cut-off at 50
    Freq50  = 0x20,
    /// For `ODR` of 760 cut-off at 100
    Freq100 = 0x30,
}

/// Full scale selection
#[repr(u8)]
pub enum ScaleSelection {
    /// 250 degrees per second
    Dps250  = 0x00,
    /// 500 degrees per second
    Dps500  = 0x10,
    /// 2000 degrees per second
    Dps2000 = 0x30,
}

impl ScaleSelection {
    fn scale_factor(&self) -> f32 {
        let mdps: f32 = match *self {
            ScaleSelection::Dps250  => 8.75e-3,
            ScaleSelection::Dps500  => 17.5e-3,
            ScaleSelection::Dps2000 => 70e-3,
        };
        num::Float::to_radians(mdps)
    }
}

/// Configuration of Gyroscope
// TODO: Implement builder pattern for struct for easier configuration
pub struct Config {
    /// Output data frequency
    pub odr: ODR,
    /// Low-pass filter cut-off frequency
    pub cut_off: CutOff,
    /// Enable interrupt on `INT1` pin (GPIOE pin 0)
    pub interrupt: bool,
    /// Sensitivity range
    pub scale: ScaleSelection,
}

impl Default for Config {
    fn default() -> Config {
        Config {
            odr: ODR::Hz380,
            cut_off: CutOff::Freq50,
            interrupt: true,
            scale: ScaleSelection::Dps2000,
        }
    }
}

/// Sensor status
pub struct Status {
    /// X, Y, Z - axis data overrun
    pub sensor_overrun: bool,
    /// Z - axis overrun
    pub z_overrun: bool,
    /// Y - axis overrun
    pub y_overrun: bool,
    /// X - axis overrun
    pub x_overrun: bool,
    /// Is new data available
    pub new_data: bool,
    /// New data for Z - axis
    pub z_new: bool,
    /// New data for Y - axis
    pub y_new: bool,
    /// New data for X - axis
    pub x_new: bool,
}

/// Register mapping for Gyroscope
#[allow(missing_docs, non_camel_case_types)]
#[allow(dead_code)] // TODO: Remove this!
#[repr(u8)]
enum Register {
    WHO_AM_I   = 0x0F,
    CTRL_REG1  = 0x20,
    CTRL_REG3  = 0x22,
    CTRL_REG4  = 0x23,
    OUT_TEMP   = 0x26,
    STATUS_REG = 0x27,
    OUT_X_L    = 0x28,
}

impl<'a> L3GD20<'a>
{
    /// Initialize Gyroscope
    pub fn init(&self, cfg: Config) -> Result<()> {
        let gpio = self.1;
        // NOTE: `CS` pin for Gyroscope is GPIOE pin 3
        gpio.moder.write(|w| w.moder3().output());
        gpio.bsrr.write(|w| w.bs3().set());
        // Debug check to test device ID, also good test for device
        // read logic
        assert!(self.check_id()?);
        self.config_reg3(cfg.interrupt)?;
        self.config_reg4(cfg.scale)?;
        // Enable sensor at the end according to L3GD20 programming guide
        self.config_reg1(cfg.odr, cfg.cut_off)?;
        Ok(())
    }

    /// Check `WHO_AM_I` register
    pub fn check_id(&self) -> Result<bool> {
        let mut id = [0];
        self.read(Register::WHO_AM_I, &mut id)?;
        // Compare to Data sheet ID
        Ok(id[0] == SENSOR_ID)
    }

    /// Read sensor temperature
    ///
    /// Read `OUT_TEMP` register of sensor
    pub fn temp(&self) -> Result<i8> {
        let mut data = [0];
        self.read(Register::OUT_TEMP, &mut data)?;
        Ok(data[0] as i8)
    }

    /// Read sensor status
    ///
    /// Read `STATUS_REG` register of sensor
    pub fn status(&self) -> Result<Status> {
        let mut data = [0];
        self.read(Register::STATUS_REG, &mut data)?;
        Ok(Status {
            sensor_overrun: data[0] & 128 != 0,
            z_overrun: data[0] & 64 != 0,
            y_overrun: data[0] & 32 != 0,
            x_overrun: data[0] & 16 != 0,
            new_data: data[0] & 8 != 0,
            z_new: data[0] & 4 != 0,
            y_new: data[0] & 2 != 0,
            x_new: data[0] & 1 != 0,

        })
    }

    /// Read angular velocity
    pub fn measure(&self, dps: ScaleSelection) -> Result<Measurement> {
        let scale = dps.scale_factor();
        // Read 2 bytes for 3 axis
        let mut data = [0; 2 * 3];
        self.read(Register::OUT_X_L, &mut data)?;
        // Cast values to u16 in preparation to combine
        let out_x_l = data[0] as u16;
        let out_x_h = data[1] as u16;
        let out_y_l = data[2] as u16;
        let out_y_h = data[3] as u16;
        let out_z_l = data[4] as u16;
        let out_z_h = data[5] as u16;
        // Return measurement with scaled values
        Ok(Measurement {
            x: ((out_x_h << 8) + out_x_l) as i16 as f32 * scale,
            y: ((out_y_h << 8) + out_y_l) as i16 as f32 * scale,
            z: ((out_z_h << 8) + out_z_l) as i16 as f32 * scale,
        })
    }

    /// Configure `CTRL_REG1`
    ///
    /// This will enable power and activate all sensor axis,
    /// the arguments control data rate and low-pass filter
    fn config_reg1(&self, odr: ODR, cut_off: CutOff) -> Result<()> {
        // TODO: Support individual axis selection
        let cmd = odr as u8 | cut_off as u8 | ENABLE_SENSOR;
        self.write(Register::CTRL_REG1, &[cmd])
    }

    /// Configure `CTRL_REG3`
    ///
    /// Enable interrupt on data-ready, `DRDY`, (`INT2`, GPIOE Pin 1)
    fn config_reg3(&self, interrupt: bool) -> Result<()> {
        // TODO: Add support for interrupt on PIN 1
        if interrupt {
            // Enable `DRDY` in `INT2`
            self.write(Register::CTRL_REG3, &[ENABLE_DRDY])?;
        }
        Ok(())
    }

    /// Configure `CTRL_REG4`
    fn config_reg4(&self, scale: ScaleSelection) -> Result<()> {
        let cmd = scale as u8;
        self.write(Register::CTRL_REG4, &[cmd])
    }

    /// Enable communication with L3GD20
    ///
    /// **NOTE** This drives the `CS` pin low
    fn enable(&self) {
        let gpio = self.1;
        gpio.bsrr.write(|w| w.br3().set_bit());
    }

    /// Disable communication with L3GD20
    ///
    /// **NOTE** This drives the `CS` pin high
    fn disable(&self) {
        let gpio = self.1;
        gpio.bsrr.write(|w| w.bs3().set_bit());
    }

    /// Write to register
    ///
    /// All bytes in `bytes` are
    /// written starting at register `reg` incremented by one for each value.
    fn write(&self, reg: Register, bytes: &[u8]) -> Result<()> {
        const WRITE: u8 = 0 << 7;
        let reg = reg as u8;
        let cmd = if bytes.len() > 1 { WRITE | MS } else { WRITE };
        let spi = self.0;
        // Drive `CS` low to communicate with Gyroscope
        self.enable();
        // Tell device we want to write, possible multiple, starting at `reg`
        block!(spi.send(cmd | reg)).map_err(Error::Spi).map_err(nb::Error::Other)?;
        // Need to read back to get device to read
        block!(spi.read()).map_err(Error::Spi).map_err(nb::Error::Other)?;
        for byte in bytes {
            block!(spi.send(*byte)).map_err(Error::Spi).map_err(nb::Error::Other)?;
            block!(spi.read()).map_err(Error::Spi).map_err(nb::Error::Other)?;
        }
        // Drive `CS` high to end communication
        self.disable();
        Ok(())
    }

    /// Read from register
    ///
    /// Read bytes from register starting at `reg` for as many bytes
    /// as `bytes` can store.
    fn read(&self, reg: Register, bytes: &mut [u8]) -> Result<()> {
        const READ: u8 = 1 << 7;
        let reg = reg as u8;
        let cmd = if bytes.len() > 1 { READ | MS } else { READ };
        let spi = self.0;
        // Drive `CS` low to communicate with Gyroscope
        self.enable();
        // Tell device we want to read, possible multiple, starting at `reg`
        block!(spi.send(cmd | reg)).map_err(Error::Spi).map_err(nb::Error::Other)?;
        // Need to read back to get device to read
        block!(spi.read()).map_err(Error::Spi).map_err(nb::Error::Other)?;
        for byte in bytes {
            // Send junk data to initiate read
            block!(spi.send(JUNK_DATA)).map_err(Error::Spi).map_err(nb::Error::Other)?;
            *byte = block!(spi.read()).map_err(Error::Spi).map_err(nb::Error::Other)?;
        }
        // Drive `CS` high to end communication
        self.disable();
        Ok(())
    }
}
