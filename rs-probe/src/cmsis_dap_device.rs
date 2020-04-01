use usb_device::bus::UsbBus;
use usb_device::device::UsbDevice;
use crate::cmsis_dap_class::CmsisDapV1;
use usb_device::UsbError;
use core::task::{Context, Poll};
use core::future::Future;
use core::pin::Pin;
use crate::{SharedWaker, CustomWaker};


pub struct DapUsbDevice<'a, B: UsbBus, W> {
    device: UsbDevice<'a, B>,
    dap: CmsisDapV1<'a, B>,
    waker: &'a SharedWaker<W>,
}

impl<'a, B: UsbBus, W: CustomWaker> DapUsbDevice<'a, B, W> {
    pub fn new(device: UsbDevice<'a, B>, dap: CmsisDapV1<'a, B>, waker: &'a SharedWaker<W>) -> Self {
        Self {
            device,
            dap,
            waker,
        }
    }

    async fn poll(&mut self) {
        self.await
    }

    pub async fn read_packet(&mut self, buffer: &mut [u8]) -> Result<usize, UsbError> {
        loop {
            match self.dap.read_packet(buffer) {
                Ok(size) => return Ok(size),
                Err(UsbError::WouldBlock) => {
                    self.poll().await;
                }
                Err(e) => return Err(e),
            }
        }
    }

    pub async fn write_packet(&mut self, data: &[u8]) -> Result<(), UsbError> {
        loop {
            match self.dap.write_packet(data) {
                Ok(()) => return Ok(()),
                Err(UsbError::WouldBlock) => {
                    self.poll().await;
                }
                Err(e) => return Err(e),
            }
        }
    }
}


impl<B: UsbBus, W: CustomWaker> Future for DapUsbDevice<'_, B, W> {
    type Output = ();

    fn poll(self: Pin<&mut Self>, cx: &mut Context<'_>) -> Poll<Self::Output> {
        let this = self.get_mut();

        if this.device.poll(&mut [&mut this.dap]) {
            Poll::Ready(())
        } else {
            this.waker.arm(cx.waker().clone());
            Poll::Pending
        }
    }
}
