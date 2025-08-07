use crate::imu::InertialMeasurementUnit;
use usb_device::{bus::UsbBusAllocator, class_prelude::UsbBus};

pub trait PlatformAbstraction {
    type OutputPin;
    type IMU: InertialMeasurementUnit;
    type USB: UsbBus;

    fn primary_led(&mut self) -> Option<Self::OutputPin>;
    fn secondary_led(&mut self) -> Option<Self::OutputPin>;

    fn imu0(&mut self) -> Option<Self::IMU>;
    fn imu1(&mut self) -> Option<Self::IMU>;

    fn usb0(&mut self) -> Option<&'static UsbBusAllocator<Self::USB>>;
}
