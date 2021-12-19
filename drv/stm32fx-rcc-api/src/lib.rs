// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.

//! Client API for the STM32fx RCC server.

#![no_std]

use core::cell::Cell;

use byteorder::LittleEndian;
use zerocopy::{AsBytes, U32};

use userlib::*;

enum Op {
    EnableClock = 1,
    DisableClock = 2,
    EnterReset = 3,
    LeaveReset = 4,
}

#[derive(Clone, Debug)]
pub struct Rcc(Cell<TaskId>);

impl From<TaskId> for Rcc {
    fn from(t: TaskId) -> Self {
        Self(Cell::new(t))
    }
}

#[derive(Copy, Clone, Debug)]
#[repr(u32)]
pub enum RccError {
    BadArg = 2,
}

impl From<u32> for RccError {
    fn from(x: u32) -> Self {
        match x {
            2 => RccError::BadArg,
            // Panicking here might be rude. TODO.
            _ => panic!(),
        }
    }
}

impl Rcc {
    /// Requests that the clock to a peripheral be turned on.
    ///
    /// This operation is idempotent and will be retried automatically should
    /// the RCC server crash while processing it.
    ///
    /// # Panics
    ///
    /// If the RCC server has died.
    pub fn enable_clock(&self, peripheral: Peripheral) {
        // We are unwrapping here because the RCC server should not return
        // BadArg for a valid member of the Peripheral enum.
        self.enable_clock_raw(peripheral as u32).unwrap()
    }

    pub fn enable_clock_raw(&self, index: u32) -> Result<(), RccError> {
        #[derive(AsBytes)]
        #[repr(C)]
        struct Request(U32<LittleEndian>);

        impl hl::Call for Request {
            const OP: u16 = Op::EnableClock as u16;
            type Response = ();
            type Err = RccError;
        }

        hl::send_with_retry(&self.0, &Request(U32::new(index)))
    }

    /// Requests that the clock to a peripheral be turned off.
    ///
    /// This operation is idempotent and will be retried automatically should
    /// the RCC server crash while processing it.
    ///
    /// # Panics
    ///
    /// If the RCC server has died.
    pub fn disable_clock(&self, peripheral: Peripheral) {
        // We are unwrapping here because the RCC server should not return
        // BadArg for a valid member of the Peripheral enum.
        self.disable_clock_raw(peripheral as u32).unwrap()
    }

    pub fn disable_clock_raw(&self, index: u32) -> Result<(), RccError> {
        #[derive(AsBytes)]
        #[repr(C)]
        struct Request(U32<LittleEndian>);

        impl hl::Call for Request {
            const OP: u16 = Op::DisableClock as u16;
            type Response = ();
            type Err = RccError;
        }

        hl::send_with_retry(&self.0, &Request(U32::new(index)))
    }

    /// Requests that the reset line to a peripheral be asserted.
    ///
    /// This operation is idempotent and will be retried automatically should
    /// the RCC server crash while processing it.
    ///
    /// # Panics
    ///
    /// If the RCC server has died.
    pub fn enter_reset(&self, peripheral: Peripheral) {
        // We are unwrapping here because the RCC server should not return
        // BadArg for a valid member of the Peripheral enum.
        self.enter_reset_raw(peripheral as u32).unwrap()
    }

    pub fn enter_reset_raw(&self, index: u32) -> Result<(), RccError> {
        #[derive(AsBytes)]
        #[repr(C)]
        struct Request(U32<LittleEndian>);

        impl hl::Call for Request {
            const OP: u16 = Op::EnterReset as u16;
            type Response = ();
            type Err = RccError;
        }

        hl::send_with_retry(&self.0, &Request(U32::new(index)))
    }

    /// Requests that the reset line to a peripheral be deasserted.
    ///
    /// This operation is idempotent and will be retried automatically should
    /// the RCC server crash while processing it.
    ///
    /// # Panics
    ///
    /// If the RCC server has died.
    pub fn leave_reset(&self, peripheral: Peripheral) {
        // We are unwrapping here because the RCC server should not return
        // BadArg for a valid member of the Peripheral enum.
        self.leave_reset_raw(peripheral as u32).unwrap()
    }

    pub fn leave_reset_raw(&self, index: u32) -> Result<(), RccError> {
        #[derive(AsBytes)]
        #[repr(C)]
        struct Request(U32<LittleEndian>);

        impl hl::Call for Request {
            const OP: u16 = Op::LeaveReset as u16;
            type Response = ();
            type Err = RccError;
        }

        hl::send_with_retry(&self.0, &Request(U32::new(index)))
    }
}

//
// A few macros for purposes of defining the Peripheral enum in terms that our
// driver is expecting:
//
// - AHB1ENR[31:0] are indices 31-0.
// - AHB2ENR[31:0] are indices 63-32.
// - AHB3ENR[31:0] are indices 95-64.
// - APB1ENR[31:0] are indices 127-96.
// - APB2ENR[31:0] are indices 159-128.

//
macro_rules! ahb1 {
    ($bit:literal) => {
        (0 * 32) + $bit
    };
}

macro_rules! ahb2 {
    ($bit:literal) => {
        (1 * 32) + $bit
    };
}

macro_rules! ahb3 {
    ($bit:literal) => {
        (2 * 32) + $bit
    };
}

macro_rules! apb1 {
    ($bit:literal) => {
        (3 * 32) + $bit
    };
}

macro_rules! apb2 {
    ($bit:literal) => {
        (4 * 32) + $bit
    };
}

/// Peripheral numbering.
///
/// Peripheral bit numbers per the STM32F4 documentation, starting with the
/// following sections:
///
///    STM32F4 PART    SECTION
///    F429            6.3.10
///
/// These are in the order that they appear in the documentation -- which,
/// while hopefully uniform across the STM32F4 variants, is not necessarily
/// an order that is at all sensible!  This is the union of all STM32F4
/// peripherals; not all peripherals will exist on all variants!
#[derive(Copy, Clone, Eq, PartialEq, Debug, FromPrimitive, AsBytes)]
#[repr(u32)]
pub enum Peripheral {
    UsbHsulpi = ahb1!(30),
    UsbOtg = ahb1!(29),
    EthPtp = ahb1!(28),  
    EthRx = ahb1!(27),  
    EthTx = ahb1!(26),  
    EthMac = ahb1!(25), 
    Dma2d = ahb1!(24), 
    Dma2 = ahb1!(22),
    Dma1 = ahb1!(21),
    CcmDataRam = ahb1!(20),
    BkpSram = ahb1!(19),
    Crc = ahb1!(12),     
    
    GpioK = ahb1!(10),
    GpioJ = ahb1!(9),
    GpioI = ahb1!(8),
    GpioH = ahb1!(7),
    GpioG = ahb1!(6),
    GpioF = ahb1!(5),
    GpioE = ahb1!(4),
    GpioD = ahb1!(3),
    GpioC = ahb1!(2),
    GpioB = ahb1!(1),
    GpioA = ahb1!(0),

    UsbFs = ahb2!(7),
    Rng = ahb2!(6),
    Hash = ahb2!(5),
    Crypt = ahb2!(4),
    Dcmi = ahb2!(0),

    Fmc = ahb3!(0),

    Usart8 = apb1!(31),
    Usart7 = apb1!(30),
    Dac = apb1!(29),
    Pwr = apb1!(28),
    Can2 = apb1!(26),
    Can1 = apb1!(25),
    I2c3 = apb1!(23),
    I2c2 = apb1!(22),
    I2c1 = apb1!(21),
    Usart5 = apb1!(20),
    Usart4 = apb1!(19),
    Usart3 = apb1!(18),
    Usart2 = apb1!(17),
    Spi3 = apb1!(15),
    Spi2 = apb1!(14),
    Wwdg = apb1!(11),
    Tim14 = apb1!(8),
    Tim13 = apb1!(7),
    Tim12 = apb1!(6),
    Tim7 = apb1!(5),
    Tim6 = apb1!(4),
    Tim5 = apb1!(3),
    Tim4 = apb1!(2),
    Tim3 = apb1!(1),
    Tim2 = apb1!(0),

    Ltdc = apb2!(26),
    Sai1 = apb2!(22),
    Spi6 = apb2!(21),
    Spi5 = apb2!(20),
    Tim11 = apb2!(18),
    Tim10 = apb2!(17),
    Tim9 = apb2!(16),
    Syscfg = apb2!(14),
    Spi4 = apb2!(13),
    Spi1 = apb2!(12),
    Sdio = apb2!(11),
    Adc3 = apb2!(10),
    Adc2 = apb2!(9),
    Adc1 = apb2!(8),
    Usart6 = apb2!(5),
    Usart1 = apb2!(4),
    Tim8 = apb2!(1),
    Tim1 = apb2!(0),
}
