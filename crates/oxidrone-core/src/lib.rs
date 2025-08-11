#![no_std]

pub mod flight_controller;

use embassy_sync::{blocking_mutex::raw::CriticalSectionRawMutex, zerocopy_channel::Receiver};
use embedded_hal::digital::OutputPin;
use heapless::Vec;
use oxidrone_hal::platform::PlatformAbstraction;
use usb_device::prelude::*;

pub struct CdcAcmDevice<const N: usize, P>
where
    P: PlatformAbstraction,
    P::USB: usb_device::bus::UsbBus + 'static,
{
    usb_dev: UsbDevice<'static, P::USB>,
    serial: usbd_serial::SerialPort<'static, P::USB>,
    receiver: Receiver<'static, CriticalSectionRawMutex, heapless::Vec<u8, N>>,
    buffer: heapless::Vec<u8, N>,
}

impl<const N: usize, P> CdcAcmDevice<N, P>
where
    P: PlatformAbstraction,
    P::USB: usb_device::bus::UsbBus,
    P::OutputPin: embedded_hal::digital::OutputPin,
{
    pub fn new(
        platform: &mut P,
        product_name: &'static str,
        serial_number: &'static str,
        receiver: Receiver<'static, CriticalSectionRawMutex, heapless::Vec<u8, N>>,
    ) -> Self {
        let usb_bus = platform.usb0().unwrap();
        let serial = usbd_serial::SerialPort::new(usb_bus);
        let usb_dev = UsbDeviceBuilder::new(usb_bus, UsbVidPid(0x16c0, 0x27dd))
            .strings(&[usb_device::device::StringDescriptors::default()
                .manufacturer("OxiDrone")
                .product(product_name)
                .serial_number(serial_number)])
            .unwrap()
            .device_class(usbd_serial::USB_CLASS_CDC)
            .build();

        let mut led = platform.secondary_led();
        if let Some(led) = &mut led {
            let _ = led.set_high();
        }

        Self {
            usb_dev,
            serial,
            receiver,
            buffer: Vec::new(),
        }
    }

    /// Services the USB CDC device from the USB FS interrupt.
    ///
    /// This function:
    /// - Advances the USB device state machine via `usb_dev.poll()`.
    /// - Attempts to transmit any pending data in the local buffer.
    /// - If the buffer is empty, pulls one message from the `receiver`
    ///   and sends it, queuing the remainder if only partially written.
    /// - Must be called frequently from the USB interrupt to keep the
    ///   device responsive and drain outgoing data.
    ///
    /// Returns once no further progress can be made in this call.
    pub fn poll(&mut self) {
        for _ in 0..3 {
            if !self.usb_dev.poll(&mut [&mut self.serial]) {
                break;
            }
        }

        if self.usb_dev.state() != UsbDeviceState::Configured || !self.serial.dtr() {
            self.buffer.clear();
            return;
        }

        if !self.buffer.is_empty() {
            match self.serial.write(self.buffer.as_slice()) {
                Ok(written) if written != 0 => {
                    let len = self.buffer.len();
                    if written < len {
                        self.buffer.as_mut_slice().copy_within(written..len, 0);
                    }
                    self.buffer.truncate(len - written);
                }
                Err(UsbError::WouldBlock) | Ok(_) => {}
                Err(_) => {
                    self.buffer.clear();
                }
            }
        } else if let Some(buffer) = self.receiver.try_receive() {
            match self.serial.write(buffer) {
                Ok(n) => {
                    let _ = self.buffer.extend_from_slice(&buffer[n..buffer.len()]);
                }
                Err(UsbError::WouldBlock) => {
                    let _ = self.buffer.extend_from_slice(buffer);
                }
                Err(_) => {
                    self.buffer.clear();
                }
            }
            self.receiver.receive_done();
        }

        // drain reads
        let mut tmp = [0u8; 64];
        loop {
            match self.serial.read(&mut tmp) {
                Ok(n) if n > 0 => continue,
                Ok(_) | Err(usb_device::UsbError::WouldBlock) => break,
                Err(_) => break,
            }
        }
    }
}
