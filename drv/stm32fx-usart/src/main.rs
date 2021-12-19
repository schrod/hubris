// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.

//! A driver for the STM32F4 U(S)ART.
//!
//! # IPC protocol
//!
//! ## `write` (1)
//!
//! Sends the contents of lease #0. Returns when completed.

#![no_std]
#![no_main]

#[cfg(feature = "f429")]
use stm32f4::stm32f429 as device;

#[cfg(feature = "f407")]
use stm32f4::stm32f407 as device;

#[cfg(feature = "stm32f3")]
use stm32f3::stm32f303 as device;

use userlib::*;
use zerocopy::AsBytes;

task_slot!(RCC, rcc_driver);
task_slot!(GPIO, gpio_driver);

#[derive(Copy, Clone, Debug, FromPrimitive)]
enum Operation {
    Write = 1,
}

#[repr(u32)]
enum ResponseCode {
    BadArg = 2,
    Busy = 3,
}

cfg_if::cfg_if! {
    if #[cfg(feature = "stm32f4")] {
        task_slot!(GPIO, gpio_driver);

        cfg_if::cfg_if! {
            if #[cfg(target_board = "nucleo-f429zi")] {
                // STM32F429 Nucleo: LEDs are on B0, B7 and B14.
                const LEDS: &[(drv_stm32fx_gpio_api::PinSet, bool)] = &[
                    (drv_stm32fx_gpio_api::Port::B.pin(0), true),
                    (drv_stm32fx_gpio_api::Port::B.pin(7), true),
                    (drv_stm32fx_gpio_api::Port::B.pin(14), true),
                ];
            }
            else if #[cfg(target_board = "stm32f4-discovery")] {
                // STM32F4 DISCOVERY kit: LEDs are on D12 and D13.
                const LEDS: &[(drv_stm32h7_gpio_api::PinSet, bool)] = &[
                    (drv_stm32h7_gpio_api::Port::D.pin(12), true),
                    (drv_stm32h7_gpio_api::Port::D.pin(13), true),
                ];
            } else {
                compile_error!("no LED mapping for unknown board");
            }
        }
    }
}

// TODO: it is super unfortunate to have to write this by hand, but deriving
// ToPrimitive makes us check at runtime whether the value fits
impl From<ResponseCode> for u32 {
    fn from(rc: ResponseCode) -> Self {
        rc as u32
    }
}

struct Transmit {
    caller: hl::Caller<()>,
    len: usize,
    pos: usize,
}

#[export_name = "main"]
fn main() -> ! {
    // Turn the actual peripheral on so that we can interact with it.
    turn_on_usart();

    // From thin air, pluck a pointer to the USART register block.
    //
    // Safety: this is needlessly unsafe in the API. The USART is essentially a
    // static, and we access it through a & reference so aliasing is not a
    // concern. Were it literally a static, we could just reference it.    
    cfg_if::cfg_if! {
        if #[cfg(target_board = "nucleo-f429zi")] {
            let usart = unsafe { &*device::USART3::ptr() };
        }
        else if #[cfg(any(target_board = "stm32f4-discovery", target_board = "stm32f3-discovery"))] {
            let usart = unsafe { &*device::USART2::ptr() };
        } else {
            compile_error!("no uart mapping for unknown board");
        }
    }

    // The UART has clock and is out of reset, but isn't actually on until we:
    usart.cr1.write(|w| w.ue().enabled());
    // Work out our baud rate divisor.
    const BAUDRATE: u32 = 115_200;

    #[cfg(feature = "stm32f3")]
    {
        const CLOCK_HZ: u32 = 8_000_000;
        usart
            .brr
            .write(|w| w.brr().bits((CLOCK_HZ / BAUDRATE) as u16));
    }

    #[cfg(feature = "f429")]
    {
        const CLOCK_HZ: u32 = 16_000_000;
        const CYCLES_PER_BIT: u32 = CLOCK_HZ / (BAUDRATE * 2); //assumes OVER8 = 0
        usart.brr.write(|w| {
            w.div_mantissa()
                .bits((CYCLES_PER_BIT >> 3) as u16)
                .div_fraction()
                .bits(CYCLES_PER_BIT as u8 & 0xF)
        });
    }

    #[cfg(feature = "f407")]
    {
        const CLOCK_HZ: u32 = 16_000_000;
        const CYCLES_PER_BIT: u32 = (CLOCK_HZ + (BAUDRATE / 2)) / BAUDRATE;
        usart.brr.write(|w| {
            w.div_mantissa()
                .bits((CYCLES_PER_BIT >> 4) as u16)
                .div_fraction()
                .bits(CYCLES_PER_BIT as u8 & 0xF)
        });
    }

    // Enable the transmitter.
    usart.cr1.modify(|_, w| w.te().enabled());

    turn_on_gpio();

    enable_uart_pins();

    // Turn on our interrupt. We haven't enabled any interrupt sources at the
    // USART side yet, so this won't trigger notifications yet.
    sys_irq_control(1, true);

    // Field messages.
    let mask = 1;
    let mut tx: Option<Transmit> = None;

    loop {
        hl::recv(
            // Buffer (none required)
            &mut [],
            // Notification mask
            mask,
            // State to pass through to whichever closure below gets run
            &mut tx,
            // Notification handler
            |txref, bits| {
                if bits & 1 != 0 {
                    // Handling an interrupt. To allow for spurious interrupts,
                    // check the individual conditions we care about, and
                    // unconditionally re-enable the IRQ at the end of the handler.

                    #[cfg(feature = "stm32f3")]
                    let txe = usart.isr.read().txe().bit();
                    #[cfg(any(feature = "f407", feature = "f429"))]
                    let txe = usart.sr.read().txe().bit();
                    if txe {
                        // TX register empty. Do we need to send something?
                        step_transmit(&usart, txref);
                    }

                    sys_irq_control(1, true);
                }
            },
            // Message handler
            |txref, op, msg| match op {
                Operation::Write => {
                    // Validate lease count and buffer sizes first.
                    let ((), caller) =
                        msg.fixed_with_leases(1).ok_or(ResponseCode::BadArg)?;

                    // Deny incoming writes if we're already running one.
                    if txref.is_some() {
                        return Err(ResponseCode::Busy);
                    }

                    let borrow = caller.borrow(0);
                    let info = borrow.info().ok_or(ResponseCode::BadArg)?;
                    // Provide feedback to callers if they fail to provide a
                    // readable lease (otherwise we'd fail accessing the borrow
                    // later, which is a defection case and we won't reply at
                    // all).
                    if !info.attributes.contains(LeaseAttributes::READ) {
                        return Err(ResponseCode::BadArg);
                    }

                    // Okay! Begin a transfer!
                    *txref = Some(Transmit {
                        caller,
                        pos: 0,
                        len: info.len,
                    });

                    // OR the TX register empty signal into the USART interrupt.
                    usart.cr1.modify(|_, w| w.txeie().enabled());

                    // We'll do the rest as interrupts arrive.
                    Ok(())
                }
            },
        );
    }
}

fn turn_on_usart() {
    let rcc_driver = RCC.get_task_id();

    const ENABLE_CLOCK: u16 = 1;
    cfg_if::cfg_if! {
        if #[cfg(target_board = "nucleo-f429zi")] {
            let pnum = 114; // see bits in AHB1ENR
        }
        else if #[cfg(any(target_board = "stm32f4-discovery", target_board = "stm32f3-discovery"))] {
            let pnum = 113; // see bits in AHBENR
        } else {
            compile_error!("no uart pnum mapping for unknown board");
        }
    }

    let (code, _) = userlib::sys_send(
        rcc_driver,
        ENABLE_CLOCK,
        pnum.as_bytes(),
        &mut [],
        &[],
    );
    assert_eq!(code, 0);

    const LEAVE_RESET: u16 = 4;
    let (code, _) = userlib::sys_send(
        rcc_driver,
        LEAVE_RESET,
        pnum.as_bytes(),
        &mut [],
        &[],
    );
    assert_eq!(code, 0);
}

fn turn_on_gpio() {
    let rcc_driver = RCC.get_task_id();

    const ENABLE_CLOCK: u16 = 1;

    cfg_if::cfg_if! {
        if #[cfg(target_board = "nucleo-f429zi")] {
            let pnum = 3; // see bits in AHB1ENR
        }
        else if #[cfg(target_board = "stm32f4-discovery")] {
            let pnum = 0; // see bits in AHBENR
        }
        else if #[cfg(target_board = "stm32f3-discovery")] {
            let pnum = 17; // see bits in AHB1ENR
        }
        else {
            compile_error!("no gpio pnum mapping for unknown board");
        }
    }


    let (code, _) = userlib::sys_send(
        rcc_driver,
        ENABLE_CLOCK,
        pnum.as_bytes(),
        &mut [],
        &[],
    );
    assert_eq!(code, 0);

    const LEAVE_RESET: u16 = 4;
    let (code, _) = userlib::sys_send(
        rcc_driver,
        LEAVE_RESET,
        pnum.as_bytes(),
        &mut [],
        &[],
    );
    assert_eq!(code, 0);
}

#[cfg(any(feature = "f407", feature = "f429"))]
fn enable_uart_pins() {
    use drv_stm32fx_gpio_api::*;

    let gpio_driver = GPIO.get_task_id();
    let gpio_driver = Gpio::from(gpio_driver);

    cfg_if::cfg_if! {
        if #[cfg(target_board = "nucleo-f429zi")] {
            gpio_driver.configure_alternate(drv_stm32fx_gpio_api::Port::D.pin(8), OutputType::OpenDrain, Speed::High, Pull::None, Alternate::AF7).unwrap();            
            gpio_driver.configure_alternate(drv_stm32fx_gpio_api::Port::D.pin(9), OutputType::OpenDrain, Speed::High, Pull::None, Alternate::AF7).unwrap();
        }
        else if #[cfg(target_board = "stm32f4-discovery")] {
            gpio_driver.configure_alternate(drv_stm32fx_gpio_api::Port::A.pin(2), OutputType::OpenDrain, Speed::High, Pull::None, Alternate::AF7).unwrap();
            gpio_driver.configure_alternate(drv_stm32fx_gpio_api::Port::A.pin(3), OutputType::OpenDrain, Speed::High, Pull::None, Alternate::AF7).unwrap();
        }
        else {
            compile_error!("no gpio pnum mapping for unknown board");
        }
    }
}

#[cfg(feature = "stm32f3")]
fn enable_uart_pins() {
    // TODO: the fact that we interact with GPIOA directly here is an expedient
    // hack, but control of the GPIOs should probably be centralized somewhere.
    let gpioa = unsafe { &*device::GPIOA::ptr() };

    // Mux the USART onto the output pins. We're using PA2/3, where USART2 is
    // selected by Alternate Function 7.
    gpioa
        .moder
        .modify(|_, w| w.moder2().alternate().moder3().alternate());
    gpioa.afrl.modify(|_, w| w.afrl2().af7().afrl3().af7());
}



fn step_transmit(
    usart: &device::usart1::RegisterBlock,
    tx: &mut Option<Transmit>,
) {
    // Clearer than just using replace:
    fn end_transmission(
        usart: &device::usart1::RegisterBlock,
        state: &mut Option<Transmit>,
    ) -> hl::Caller<()> {
        usart.cr1.modify(|_, w| w.txeie().disabled());
        core::mem::replace(state, None).unwrap().caller
    }

    let txs = if let Some(txs) = tx { txs } else { return };

    if let Some(byte) = txs.caller.borrow(0).read_at::<u8>(txs.pos) {
        // Stuff byte into transmitter.
        #[cfg(feature = "stm32f3")]
        usart.tdr.write(|w| w.tdr().bits(u16::from(byte)));
        #[cfg(any(feature = "f407", feature = "f429"))]
        usart.dr.write(|w| w.dr().bits(u16::from(byte)));

        txs.pos += 1;
        if txs.pos == txs.len {
            end_transmission(usart, tx).reply(());
        }
    } else {
        end_transmission(usart, tx).reply_fail(ResponseCode::BadArg);
    }
}
