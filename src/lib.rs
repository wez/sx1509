/*!
A platform agnostic Rust friver for the [sx1509], based on the
[`embedded-hal`] traits.

## The Device

The [sx1509] is a GPIO expander that also providers LED driver
and keypad scanning functionality.  The device has two banks
of 8 GPIO pins.

## Supported features

At the time of writing, the support is very bare bones: you can
reset the device and configure basic GPIO functions by operating
on a bank at a time.

## Future

There's scope for some interesting API design choices; for example,
it would be nice to expose the individual GPIO pins as instances
that implement the GPIO traits from [`embedded-hal`] so that they
could in turn be used to interface with other drivers.

The keypad and LED driver functionality has some potential for
nice type safe API design by moving pins/banks into different
modes.

Care needs to be taken to find and ergnomic and zero cost
way to express these APIs.

For the moment we just have a fairly dumb pass through!

## Usage

Import this crate an an `embedded_hal` implementation:

```
extern crate atsamd21_hal as hal;
extern crate sx1509;
```

Initialize the I2C bus (differs between the various hal implementations):

```no_run
let mut i2c = I2CMaster3::new(
    &clocks,
    400.khz(),
    peripherals.SERCOM3,
    &mut peripherals.PM,
    &mut peripherals.GCLK,
    // Metro M0 express has I2C on pins PA22, PA23
    Sercom3Pad0::Pa22(pins.pa22.into_function_c(&mut pins.port)),
    Sercom3Pad1::Pa23(pins.pa23.into_function_c(&mut pins.port)),
);
```

and then instantiate the driver:

```no_run
let mut expander = sx1509::Sx1509::new(&mut i2c, sx1509::DEFAULT_ADDRESS);
expander.borrow(&mut i2c).software_reset()?;
expander.borrow(&mut i2c).set_bank_a_direction(0)?;
// read the pins from bank a
let pins = expander.borrow(&mut i2c).get_bank_a_data()?;
```

Note that you must `borrow` the i2c bus for the duration of each operation.
This allows the bus to be shared and used to address other devices in between
operations.

It is possible to take ownership of the bus for a longer period of time.  That
is required for `Future` based usage.  While `Future` based usage isn't supported
at this time, we do have support for this mode of operation:

```no_run
let mut owned = expander.take(i2c);

// Now you can borrow and perform the IO when you are ready:
let pins = owned.borrow().get_bank_a_data()?;

// and relinquish the bus when done
let (expander, i2c) = owned.release();
```

*/
#![no_std]
extern crate embedded_hal as hal;
use core::marker::PhantomData;

use hal::blocking::i2c::{Write, WriteRead};

#[derive(Copy, Clone)]
pub enum Register {
    /// Input buffer disable register - I/O[15-8] (Bank B) 0000 0000
    RegInputDisableB = 0x00,
    /// Input buffer disable register - I/O[7-0] (Bank A) 0000 0000
    RegInputDisableA = 0x01,
    /// Output buffer long slew register - I/O[15-8] (Bank B) 0000 0000
    RegLongSlewB = 0x02,
    /// Output buffer long slew register - I/O[7-0] (Bank A) 0000 0000
    RegLongSlewA = 0x03,
    /// Output buffer low drive register - I/O[15-8] (Bank B) 0000 0000
    RegLowDriveB = 0x04,
    /// Output buffer low drive register - I/O[7-0] (Bank A) 0000 0000
    RegLowDriveA = 0x05,
    /// Pull-up register - I/O[15-8] (Bank B) 0000 0000
    RegPullUpB = 0x06,
    /// Pull-up register - I/O[7-0] (Bank A) 0000 0000
    RegPullUpA = 0x07,
    /// Pull-down register - I/O[15-8] (Bank B) 0000 0000
    RegPullDownB = 0x08,
    /// Pull-down register - I/O[7-0] (Bank A) 0000 0000
    RegPullDownA = 0x09,
    /// Open drain register - I/O[15-8] (Bank B) 0000 0000
    RegOpenDrainB = 0x0A,
    /// Open drain register - I/O[7-0] (Bank A) 0000 0000
    RegOpenDrainA = 0x0B,
    /// Polarity register - I/O[15-8] (Bank B) 0000 0000
    RegPolarityB = 0x0C,
    /// Polarity register - I/O[7-0] (Bank A) 0000 0000
    RegPolarityA = 0x0D,
    /// Direction register - I/O[15-8] (Bank B) 1111 1111
    RegDirB = 0x0E,
    /// Direction register - I/O[7-0] (Bank A) 1111 1111
    RegDirA = 0x0F,
    /// Data register - I/O[15-8] (Bank B) 1111 1111*
    RegDataB = 0x10,
    /// Data register - I/O[7-0] (Bank A) 1111 1111*
    RegDataA = 0x11,
    /// Interrupt mask register - I/O[15-8] (Bank B) 1111 1111
    RegInterruptMaskB = 0x12,
    /// Interrupt mask register - I/O[7-0] (Bank A) 1111 1111
    RegInterruptMaskA = 0x13,
    /// Sense register for I/O[15:12] 0000 0000
    RegSenseHighB = 0x14,
    /// Sense register for I/O[11:8] 0000 0000
    RegSenseLowB = 0x15,
    /// Sense register for I/O[7:4] 0000 0000
    RegSenseHighA = 0x16,
    /// Sense register for I/O[3:0] 0000 0000
    RegSenseLowA = 0x17,
    /// Interrupt source register - I/O[15-8] (Bank B) 0000 0000
    RegInterruptSourceB = 0x18,
    /// Interrupt source register - I/O[7-0] (Bank A) 0000 0000
    RegInterruptSourceA = 0x19,
    /// Event status register - I/O[15-8] (Bank B) 0000 0000
    RegEventStatusB = 0x1A,
    /// Event status register - I/O[7-0] (Bank A) 0000 0000
    RegEventStatusA = 0x1B,
    /// Level shifter register 0000 0000
    RegLevelShifter1 = 0x1C,
    /// Level shifter register 0000 0000
    RegLevelShifter2 = 0x1D,
    /// Clock management register 0000 0000
    RegClock = 0x1E,
    /// Miscellaneous device settings register 0000 0000
    RegMisc = 0x1F,
    /// LED driver enable register - I/O[15-8] (Bank B) 0000 0000
    RegLEDDriverEnableB = 0x20,
    /// LED driver enable register - I/O[7-0] (Bank A) 0000 0000
    RegLEDDriverEnableA = 0x21,
    /// Debounce configuration register 0000 0000
    RegDebounceConfig = 0x22,
    /// Debounce enable register - I/O[15-8] (Bank B) 0000 0000
    RegDebounceEnableB = 0x23,
    /// Debounce enable register - I/O[7-0] (Bank A) 0000 0000
    RegDebounceEnableA = 0x24,
    /// Key scan configuration register 0000 0000
    RegKeyConfig1 = 0x25,
    /// Key scan configuration register 0000 0000
    RegKeyConfig2 = 0x26,
    /// Key value (column) 1111 1111
    RegKeyData1 = 0x27,
    /// Key value (row) 1111 1111
    RegKeyData2 = 0x28,
    /// ON time register for I/O[0] 0000 0000
    RegTOn0 = 0x29,
    /// ON intensity register for I/O[0] 1111 1111
    RegIOn0 = 0x2A,
    /// OFF time/intensity register for I/O[0] 0000 0000
    RegOff0 = 0x2B,
    /// ON time register for I/O[1] 0000 0000
    RegTOn1 = 0x2C,
    /// ON intensity register for I/O[1] 1111 1111
    RegIOn1 = 0x2D,
    /// OFF time/intensity register for I/O[1] 0000 0000
    RegOff1 = 0x2E,
    /// ON time register for I/O[2] 0000 0000
    RegTOn2 = 0x2F,
    /// ON intensity register for I/O[2] 1111 1111
    RegIOn2 = 0x30,
    /// OFF time/intensity register for I/O[2] 0000 0000
    RegOff2 = 0x31,
    /// ON time register for I/O[3] 0000 0000
    RegTOn3 = 0x32,
    /// ON intensity register for I/O[3] 1111 1111
    RegIOn3 = 0x33,
    /// OFF time/intensity register for I/O[3] 0000 0000
    RegOff3 = 0x34,
    /// ON time register for I/O[4] 0000 0000
    RegTOn4 = 0x35,
    /// ON intensity register for I/O[4] 1111 1111
    RegIOn4 = 0x36,
    /// OFF time/intensity register for I/O[4] 0000 0000
    RegOff4 = 0x37,
    /// Fade in register for I/O[4] 0000 0000
    RegTRise4 = 0x38,
    /// Fade out register for I/O[4] 0000 0000
    RegTFall4 = 0x39,
    /// ON time register for I/O[5] 0000 0000
    RegTOn5 = 0x3A,
    /// ON intensity register for I/O[5] 1111 1111
    RegIOn5 = 0x3B,
    /// OFF time/intensity register for I/O[5] 0000 0000
    RegOff5 = 0x3C,
    /// Fade in register for I/O[5] 0000 0000
    RegTRise5 = 0x3D,
    /// Fade out register for I/O[5] 0000 0000
    RegTFall5 = 0x3E,
    /// ON time register for I/O[6] 0000 0000
    RegTOn6 = 0x3F,
    /// ON intensity register for I/O[6] 1111 1111
    RegIOn6 = 0x40,
    /// OFF time/intensity register for I/O[6] 0000 0000
    RegOff6 = 0x41,
    /// Fade in register for I/O[6] 0000 0000
    RegTRise6 = 0x42,
    /// Fade out register for I/O[6] 0000 0000
    RegTFall6 = 0x43,
    /// ON time register for I/O[7] 0000 0000
    RegTOn7 = 0x44,
    /// ON intensity register for I/O[7] 1111 1111
    RegIOn7 = 0x45,
    /// OFF time/intensity register for I/O[7] 0000 0000
    RegOff7 = 0x46,
    /// Fade in register for I/O[7] 0000 0000
    RegTRise7 = 0x47,
    /// Fade out register for I/O[7] 0000 0000
    RegTFall7 = 0x48,
    /// ON time register for I/O[8] 0000 0000
    RegTOn8 = 0x49,
    /// ON intensity register for I/O[8] 1111 1111
    RegIOn8 = 0x4A,
    /// OFF time/intensity register for I/O[8] 0000 0000
    RegOff8 = 0x4B,
    /// ON time register for I/O[9] 0000 0000
    RegTOn9 = 0x4C,
    /// ON intensity register for I/O[9] 1111 1111
    RegIOn9 = 0x4D,
    /// OFF time/intensity register for I/O[9] 0000 0000
    RegOff9 = 0x4E,
    /// ON time register for I/O[10] 0000 0000
    RegTOn10 = 0x4F,
    /// ON intensity register for I/O[10] 1111 1111
    RegIOn10 = 0x50,
    /// OFF time/intensity register for I/O[10] 0000 0000
    RegOff10 = 0x51,
    /// ON time register for I/O[11] 0000 0000
    RegTOn11 = 0x52,
    /// ON intensity register for I/O[11] 1111 1111
    RegIOn11 = 0x53,
    /// OFF time/intensity register for I/O[11] 0000 0000
    RegOff11 = 0x54,
    /// ON time register for I/O[12] 0000 0000
    RegTOn12 = 0x55,
    /// ON intensity register for I/O[12] 1111 1111
    RegIOn12 = 0x56,
    /// OFF time/intensity register for I/O[12] 0000 0000
    RegOff12 = 0x57,
    /// Fade in register for I/O[12] 0000 0000
    RegTRise12 = 0x58,
    /// Fade out register for I/O[12] 0000 0000
    RegTFall12 = 0x59,
    /// ON time register for I/O[13] 0000 0000
    RegTOn13 = 0x5A,
    /// ON intensity register for I/O[13] 1111 1111
    RegIOn13 = 0x5B,
    /// OFF time/intensity register for I/O[13] 0000 0000
    RegOff13 = 0x5C,
    /// Fade in register for I/O[13] 0000 0000
    RegTRise13 = 0x5D,
    /// Fade out register for I/O[13] 0000 0000
    RegTFall13 = 0x5E,
    /// ON time register for I/O[14] 0000 0000
    RegTOn14 = 0x5F,
    /// ON intensity register for I/O[14] 1111 1111
    RegIOn14 = 0x60,
    /// OFF time/intensity register for I/O[14] 0000 0000
    RegOff14 = 0x61,
    /// Fade in register for I/O[14] 0000 0000
    RegTRise14 = 0x62,
    /// Fade out register for I/O[14] 0000 0000
    RegTFall14 = 0x63,
    /// ON time register for I/O[15] 0000 0000
    RegTOn15 = 0x64,
    /// ON intensity register for I/O[15] 1111 1111
    RegIOn15 = 0x65,
    /// OFF time/intensity register for I/O[15] 0000 0000
    RegOff15 = 0x66,
    /// Fade in register for I/O[15] 0000 0000
    RegTRise15 = 0x67,
    /// Fade out register for I/O[15] 0000 0000
    RegTFall15 = 0x68,
    /// High input enable register - I/O[15-8] (Bank B) 0000 0000
    RegHighInputB = 0x69,
    /// High input enable register - I/O[7-0] (Bank A) 0000 0000
    RegHighInputA = 0x6A,
    RegReset = 0x7D,
}

/// The default address of the SparkFun board, with no jumpers
/// applied to alter its address.
pub const DEFAULT_ADDRESS: u8 = 0x3e;

/// The `Sx1509` struct encapsulates the basic data about the device.
/// You need to `borrow` or `take` the I2C bus to issue IO.
pub struct Sx1509<I2C>
where
    I2C: WriteRead + Write,
{
    address: u8,
    i2c: PhantomData<I2C>,
}

/// The `Owned` struct encapsulates the device and owns the bus.
/// You need to `borrow` to issue IO, or `release` to release
/// ownership of the bus.
pub struct Owned<I2C>
where
    I2C: WriteRead + Write,
{
    i2c: I2C,
    state: Sx1509<I2C>,
}

/// The `Borrowed` struct encapsulates the device and a borrowed
/// bus.   This is the struct which holds the actual `impl` for
/// the device driver.
pub struct Borrowed<'a, I2C: 'a>
where
    I2C: WriteRead + Write,
{
    i2c: &'a mut I2C,
    state: &'a mut Sx1509<I2C>,
}

impl<I2C> Sx1509<I2C>
where
    I2C: WriteRead + Write,
{
    /// Create an instance.  No implicit initialization is performed.
    /// You will likely want to perform a `software_reset` as the
    /// next step.
    pub fn new(_i2c: &mut I2C, address: u8) -> Self {
        Self {
            i2c: PhantomData,
            address,
        }
    }

    /// Take ownership of the bus and return an `Owned` instance.
    /// This is best suited for asynchronous usage where you initiate IO
    /// and later test to see whether it is complete.  This doesn't make
    /// a lot of sense with the current implementation of the Sx1509 driver
    /// as no such workflows are supported today.
    /// You probably want the `borrow` method instead.
    pub fn take(self, i2c: I2C) -> Owned<I2C> {
        Owned {
            state: self,
            i2c: i2c,
        }
    }

    /// Borrow ownership of the bus and return a `Borrowed` instance that
    /// can be used to perform IO.
    pub fn borrow<'a>(&'a mut self, i2c: &'a mut I2C) -> Borrowed<'a, I2C> {
        Borrowed { state: self, i2c }
    }
}

impl<I2C> Owned<I2C>
where
    I2C: WriteRead + Write,
{
    /// Release ownership of the bus and decompose the `Owned` instance back
    /// into its constituent `Sx1509` and `I2C` components.
    pub fn release(self) -> (Sx1509<I2C>, I2C) {
        (self.state, self.i2c)
    }

    /// Create a `Borrowed` instance from this `Owned` instance so that you
    /// can perform IO on it.
    /// Ideally we'd impl `Deref` and `DerefMut` instead of doing this, but
    /// it seems impossible to do this and preserve appropriate lifetimes.
    pub fn borrow<'a>(&'a mut self) -> Borrowed<'a, I2C> {
        Borrowed {
            state: &mut self.state,
            i2c: &mut self.i2c,
        }
    }
}

impl<'a, I2C, E> Borrowed<'a, I2C>
where
    I2C: WriteRead<Error = E> + Write<Error = E> + 'a,
{
    /// Write `value` to `register`
    fn write(&mut self, register: Register, value: u8) -> Result<(), E> {
        self.i2c.write(self.state.address, &[register as u8, value])
    }

    /// Read a 16-bit value from the register and its successor
    pub fn read_16(&mut self, register: Register) -> Result<u16, E> {
        let mut buf = [0u8; 2];
        self.i2c
            .write_read(self.state.address, &[register as u8], &mut buf)?;
        Ok((buf[0] as u16) << 8 | buf[1] as u16)
    }

    /// Perform a software reset of the module.  This restores
    /// the device to its power-on defaults.
    pub fn software_reset(&mut self) -> Result<(), E> {
        self.write(Register::RegReset, 0x12)?;
        self.write(Register::RegReset, 0x34)
    }

    /// Set the direction for each pin in BankA.
    /// Each 1 bit will be set to output, each 0 bit will
    /// be set to input.
    pub fn set_bank_a_direction(&mut self, mask: u8) -> Result<(), E> {
        self.write(Register::RegDirA, !mask)
    }

    /// Set the direction for each pin in BankB.
    /// Each 1 bit will be set to output, each 0 bit will
    /// be set to input.
    pub fn set_bank_b_direction(&mut self, mask: u8) -> Result<(), E> {
        self.write(Register::RegDirB, !mask)
    }

    /// Set the data for each pin in BankA.
    /// Each 1 bit will be set to high, each 0 bit will be set low.
    pub fn set_bank_a_data(&mut self, mask: u8) -> Result<(), E> {
        self.write(Register::RegDataA, mask)
    }

    /// Set the data for each pin in BankB.
    /// Each 1 bit will be set to high, each 0 bit will be set low.
    pub fn set_bank_b_data(&mut self, mask: u8) -> Result<(), E> {
        self.write(Register::RegDataB, mask)
    }

    /// Get the data for each pin in BankA.
    /// Each 1 bit is set to high, each 0 bit is set low.
    pub fn get_bank_a_data(&mut self) -> Result<u8, E> {
        let mut res = [0u8; 1];
        self.i2c
            .write_read(self.state.address, &[Register::RegDataA as u8], &mut res)?;
        Ok(res[0])
    }

    /// Get the data for each pin in BankB.
    /// Each 1 bit is set to high, each 0 bit is set low.
    pub fn get_bank_b_data(&mut self) -> Result<u8, E> {
        let mut res = [0u8; 1];
        self.i2c
            .write_read(self.state.address, &[Register::RegDataB as u8], &mut res)?;
        Ok(res[0])
    }

    /// Get the data for each pin in BankB and BankA as a 16-bit value.
    /// Each 1 bit is set to high, each 0 bit is set low.
    pub fn get_bank_a_and_b_data(&mut self) -> Result<u16, E> {
        self.read_16(Register::RegDataB)
    }

    /// Set the pull-up for each pin in BankA.
    /// Each 1 bit will have pull up enabled, 0 disabled
    pub fn set_bank_a_pullup(&mut self, mask: u8) -> Result<(), E> {
        self.write(Register::RegPullUpA, mask)
    }

    /// Set the pull-up for each pin in BankB.
    /// Each 1 bit will have pull up enabled, 0 disabled
    pub fn set_bank_b_pullup(&mut self, mask: u8) -> Result<(), E> {
        self.write(Register::RegPullUpB, mask)
    }
}
