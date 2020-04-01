#![no_std]
#![no_main]

extern crate panic_semihosting;

mod cmsis_dap_class;
mod cmsis_dap_device;

use cortex_m::asm::delay;
use cortex_m_rt::entry;
use embedded_hal::digital::v2::OutputPin;
use stm32f1xx_hal::usb::{Peripheral, UsbBus, UsbBusType};
use stm32f1xx_hal::{prelude::*, stm32, stm32::{interrupt, Interrupt}};
use stm32f1xx_hal::gpio::{Output, PushPull, gpioc::PC13};
use usb_device::prelude::*;
use crate::cmsis_dap_class::CmsisDapV1;
use cmsis_dap::{DapCommand, Command, DapResponse, ResponseStatus};
use core::future::Future;
use core::pin::Pin;
use core::task::{Context, Poll, Waker, RawWaker, RawWakerVTable};
use core::sync::atomic::{AtomicBool, Ordering};
use cortex_m::peripheral::NVIC;
use crate::cmsis_dap_device::DapUsbDevice;
use core::cell::Cell;
use core::marker::PhantomData;
use cortex_m::interrupt::Mutex;

pub trait CustomWaker {
    fn after_arm() {}

    fn before_wake() {}
}

pub struct SharedWaker<T> {
    inner: Mutex<Cell<Option<Waker>>>,
    _marker: PhantomData<T>,
}

impl<T> SharedWaker<T> {
    pub const fn new() -> Self {
        Self {
            inner: Mutex::new(Cell::new(None)),
            _marker: PhantomData
        }
    }
}

impl<T: CustomWaker> SharedWaker<T> {
    pub fn arm(&self, waker: Waker) {
        cortex_m::interrupt::free(|cs| {
            self.inner.borrow(cs).set(Some(waker));
        });

        T::after_arm();
    }

    pub fn wake(&self) {
        cortex_m::interrupt::free(|cs| {
            if let Some(waker) = self.inner.borrow(cs).take() {
                T::before_wake();

                waker.wake();
            }
        });
    }
}

pub struct UsbWaker;

impl CustomWaker for UsbWaker {
    fn after_arm() {
        unsafe {
            NVIC::unmask(Interrupt::USB_HP_CAN_TX);
            NVIC::unmask(Interrupt::USB_LP_CAN_RX0);
        }
    }
}

static USB_WAKER: SharedWaker<UsbWaker> = SharedWaker::new();

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
}


struct DapEngine<'a, I> {
    usb: DapUsbDevice<'a, UsbBusType, UsbWaker>,
    dap: I,
}

impl<I: DapImplementation> DapEngine<'_, I> {
    async fn reject_command(&mut self) -> Result<(), DapError> {
        self.usb.write_packet(&[Command::DAP_Invalid as u8]).await.map_err(DapError::Usb)
    }

    async fn send_response(&mut self, response: DapResponse<'_>) -> Result<(), DapError> {
        self.usb.write_packet(&response).await.map_err(DapError::Usb)
    }

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
                _ => response.reject(),
            }

            self.send_response(response).await?;
        }
        Ok(())
    }
}


pub fn block_on<T>(f: impl Future<Output = T>) -> T {
    // NOTE `*const ()` is &AtomicBool
    static VTABLE: RawWakerVTable = {
        unsafe fn clone(p: *const ()) -> RawWaker {
            RawWaker::new(p, &VTABLE)
        }
        unsafe fn wake(p: *const ()) {
            wake_by_ref(p)
        }
        unsafe fn wake_by_ref(p: *const ()) {
            (*(p as *const AtomicBool)).store(true, Ordering::Release)
        }
        unsafe fn drop(_: *const ()) {
            // no-op
        }

        RawWakerVTable::new(clone, wake, wake_by_ref, drop)
    };

    // Move the value to ensure that it is owned
    let mut f = f;
    // Shadow the original binding so that it can't be directly accessed
    // ever again.
    let mut f = unsafe {
        Pin::new_unchecked(&mut f)
    };

    let ready = AtomicBool::new(true);
    let waker = unsafe {
        Waker::from_raw(RawWaker::new(&ready as *const _ as *const _, &VTABLE))
    };

    loop {
        let mut cx = Context::from_waker(&waker);
        if let Poll::Ready(val) = f.as_mut().poll(&mut cx) {
            break val;
        }
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
