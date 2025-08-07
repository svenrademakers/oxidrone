#![no_std]
pub mod flight_controller;
use oxidrone_hal::platform::PlatformAbstraction;
use usb_device::prelude::*;

pub struct SerialLogger<P>
where
    P: PlatformAbstraction,
    P::USB: usb_device::bus::UsbBus + 'static,
{
    usb_dev: UsbDevice<'static, P::USB>,
    serial: usbd_serial::SerialPort<'static, P::USB>,
}

impl<P> SerialLogger<P>
where
    P: PlatformAbstraction,
    P::USB: usb_device::bus::UsbBus,
{
    pub fn new(platform: &mut P) -> Self {
        let usb_bus = platform.usb0().unwrap();
        let serial = usbd_serial::SerialPort::new(usb_bus);
        let usb_dev = UsbDeviceBuilder::new(usb_bus, UsbVidPid(0x16c0, 0x27dd))
            .strings(&[usb_device::device::StringDescriptors::default()
                .manufacturer("OxiDrone")
                .product("Serial port")
                .serial_number("TEST PORT 1")])
            .unwrap()
            .device_class(usbd_serial::USB_CLASS_CDC)
            .build();
        Self { usb_dev, serial }
    }

    pub fn poll(&mut self) {
        loop {
            if !self.usb_dev.poll(&mut [&mut self.serial]) {
                return;
            }

            let mut buf = [0u8; 64];

            match self.serial.read(&mut buf) {
                Ok(count) if count > 0 => {
                    // Echo back in upper case
                    for c in buf[0..count].iter_mut() {
                        if 0x61 <= *c && *c <= 0x7a {
                            *c &= !0x20;
                        }
                    }

                    let mut write_offset = 0;
                    while write_offset < count {
                        match self.serial.write(&buf[write_offset..count]) {
                            Ok(len) if len > 0 => {
                                write_offset += len;
                            }
                            _ => {}
                        }
                    }
                }
                _ => {}
            }
        }
    }
}
