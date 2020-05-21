#![no_std]
#![no_main]

extern crate panic_semihosting;

mod cmsis_dap_class;
mod cmsis_dap_device;
mod executor;

use cortex_m::asm::delay;
use cortex_m_rt::entry;
use embedded_hal::digital::v2::OutputPin;
use stm32f1xx_hal::usb::{Peripheral, UsbBus, UsbBusType};
use stm32f1xx_hal::{prelude::*, stm32, stm32::{interrupt, Interrupt}};
use stm32f1xx_hal::gpio::{Output, PushPull, gpioc::PC13};
use usb_device::prelude::*;
use crate::cmsis_dap_class::{CmsisDapV1, CmsisDapClass};
use cmsis_dap::{DapCommand, Command, DapResponse, ResponseStatus};
use cortex_m::peripheral::NVIC;
use crate::cmsis_dap_device::DapUsbDevice;
use crate::executor::{SharedWaker, SharedWakerVTable, block_on};
use core::convert::TryFrom;


static USB_WAKER: SharedWaker = SharedWaker::new(SharedWakerVTable {
    after_arm: Some(|| {
        unsafe {
            NVIC::unmask(Interrupt::USB_HP_CAN_TX);
            NVIC::unmask(Interrupt::USB_LP_CAN_RX0);
        }
    }),
    before_wake: None
});

#[interrupt]
fn USB_HP_CAN_TX() {
    // avoid continuously re-entering this interrupt handler
    NVIC::mask(Interrupt::USB_HP_CAN_TX);

    USB_WAKER.wake();
}

#[interrupt]
fn USB_LP_CAN_RX0() {
    // avoid continuously re-entering this interrupt handler
    NVIC::mask(Interrupt::USB_LP_CAN_RX0);

    USB_WAKER.wake();
}



enum DapError {
    Usb(UsbError)
}

impl From<UsbError> for DapError {
    fn from(e: UsbError) -> Self {
        DapError::Usb(e)
    }
}

pub trait DapImplementation {
    fn pin_io(&mut self, output_select: u8, output: u8, pin_wait_us: u32) -> u8;

    fn set_swj_clock(&mut self, frequency: u32) -> ResponseStatus;

    fn swj_sequence(&mut self, bits: &[u8], bit_count: usize) -> ResponseStatus;

    fn transfer_configure(&mut self, idle_cycles: u8, wait_retry: u16, match_retry: u16) -> ResponseStatus;

    fn set_status_led(&mut self, led_type: DapLedType, led_status: bool);
}

pub enum DapLedType {
    Connect,
    Running,
}

impl TryFrom<u8> for DapLedType {
    type Error = ();

    fn try_from(value: u8) -> Result<Self, Self::Error> {
        match value {
            0 => Ok(DapLedType::Connect),
            1 => Ok(DapLedType::Running),
            _ => Err(()),
        }
    }
}

struct DapImpl {
    led: PC13<Output<PushPull>>,
}

impl DapImplementation for DapImpl {
    fn pin_io(&mut self, output_select: u8, output: u8, _pin_wait_us: u32) -> u8 {
        output & output_select
    }

    fn set_swj_clock(&mut self, _frequency: u32) -> ResponseStatus {
        ResponseStatus::DAP_OK
    }

    fn swj_sequence(&mut self, _bits: &[u8], _bit_count: usize) -> ResponseStatus {
        ResponseStatus::DAP_OK
    }

    fn transfer_configure(&mut self, _idle_cycles: u8, _wait_retry: u16, _match_retry: u16) -> ResponseStatus {
        ResponseStatus::DAP_OK
    }

    fn set_status_led(&mut self, _led_type: DapLedType, led_status: bool) {
        if led_status {
            self.led.set_low().ok();
        } else {
            self.led.set_high().ok();
        }
    }
}


struct DapEngine<'a, I, C> {
    usb: DapUsbDevice<'a, UsbBusType, C>,
    dap: I,
}

impl<I: DapImplementation, C: CmsisDapClass<UsbBusType>> DapEngine<'_, I, C> {
    pub fn process_dap_info(&mut self, id: u8, response: &mut DapResponse) {
        match id {
            0xF0 => { // Capabilities
                response.write_byte(1); // length

                // SWD + JTAG supported
                response.write_byte(0x03);
            }
            0xFE => { // Packet Count
                response.write_byte(1); // length

                // Just a single packet
                response.write_byte(1);
            }
            0xFF => { // Packet Size
                response.write_byte(2); // length

                response.write_short(64);
            }
            _ => {}
        }
    }

    pub async fn process(&mut self) -> Result<(), DapError> {
        let mut buffer = [0; 64];
        let size = self.usb.read_packet(&mut buffer).await?;
        if let Some(mut command) = DapCommand::parse(&buffer[..size]) {
            let cmd = command.command();
            let mut buffer = [0; 64];
            let mut response = DapResponse::new(cmd, &mut buffer);

            match cmd {
                Command::DAP_Info => {
                    let id = command.read_byte();
                    self.process_dap_info(id, &mut response);
                }
                Command::DAP_HostStatus => {
                    let led_type = command.read_byte();
                    let led_status = command.read_byte();

                    if let Ok(led_type) = DapLedType::try_from(led_type) {
                        // openocd sends a bitmask in led_status instead of a single led status
                        // Don't check for the valid value
                        self.dap.set_status_led(led_type, led_status != 0);
                        response.write_byte(0x00);
                    } else {
                        response.reject();
                    }
                }
                Command::DAP_SWJ_Pins => {
                    let pin_output = command.read_byte();
                    let pin_select = command.read_byte();
                    let pin_wait = command.read_word();
                    let input = self.dap.pin_io(pin_select, pin_output, pin_wait);
                    response.write_byte(input);
                }
                Command::DAP_SWJ_Clock => {
                    let frequency = command.read_word();
                    let status = self.dap.set_swj_clock(frequency);
                    response.write_byte(status as u8);
                }
                Command::DAP_SWJ_Sequence => {
                    let bit_count = command.read_byte() as usize;
                    let byte_count = (bit_count + 7) / 8;
                    let bits = &command[..byte_count];
                    let status = self.dap.swj_sequence(bits, bit_count);
                    response.write_byte(status as u8);
                }
                Command::DAP_TransferConfigure => {
                    let idle_cycles = command.read_byte();
                    let wait_retry = command.read_short();
                    let match_retry = command.read_short();
                    let status = self.dap.transfer_configure(idle_cycles, wait_retry, match_retry);
                    response.write_byte(status as u8);
                }
                _ => response.reject(),
            }

            // Send response
            self.usb.write_packet(&response).await?;
        }
        Ok(())
    }
}


#[entry]
fn main() -> ! {
    let dp = stm32::Peripherals::take().unwrap();

    let mut flash = dp.FLASH.constrain();
    let mut rcc = dp.RCC.constrain();

    let clocks = rcc
        .cfgr
        .use_hse(8.mhz())
        .sysclk(48.mhz())
        .pclk1(24.mhz())
        .freeze(&mut flash.acr);

    assert!(clocks.usbclk_valid());

    // Configure the on-board LED (PC13, green)
    let mut gpioc = dp.GPIOC.split(&mut rcc.apb2);
    let mut led = gpioc.pc13.into_push_pull_output(&mut gpioc.crh);
    led.set_high().ok(); // Turn off

    let mut gpioa = dp.GPIOA.split(&mut rcc.apb2);

    // BluePill board has a pull-up resistor on the D+ line.
    // Pull the D+ pin down to send a RESET condition to the USB bus.
    // This forced reset is needed only for development, without it host
    // will not reset your device when you upload new firmware.
    let mut usb_dp = gpioa.pa12.into_push_pull_output(&mut gpioa.crh);
    usb_dp.set_low().ok();
    delay(clocks.sysclk().0 / 100);

    let usb = Peripheral {
        usb: dp.USB,
        pin_dm: gpioa.pa11,
        pin_dp: usb_dp.into_floating_input(&mut gpioa.crh),
    };
    let usb_bus = UsbBus::new(usb);

    let cmsis_dap = CmsisDapV1::new(&usb_bus);

    let usb_dev = UsbDeviceBuilder::new(&usb_bus, UsbVidPid(0xc251, 0xf001))
        .manufacturer("KEIL - Tools By ARM")
        .product("LPC-Link-II CMSIS-DAP")
        .serial_number("TEST")
        .build();

    // let usb_dev = UsbDeviceBuilder::new(&usb_bus, UsbVidPid(0xc251, 0xf000))
    //     .manufacturer("KEIL - Tools By ARM")
    //     .product("CMSIS-DAP v2")
    //     .serial_number("TEST")
    //     .device_class(0xff)
    //     .build();

    let dap_impl = DapImpl {
        led,
    };

    let mut dap = DapEngine {
        usb: DapUsbDevice::new(usb_dev, cmsis_dap, &USB_WAKER),
        dap: dap_impl
    };

    block_on(async {
        loop {
            dap.process().await.ok();
        }
    })
}
