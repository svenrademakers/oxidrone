use crate::imu::InertialMeasurementUnit;

pub trait PlatformAbstraction {
    type OutputPin;
    type IMU: InertialMeasurementUnit;
    fn primary_led(&mut self) -> Option<Self::OutputPin>;
    fn secondary_led(&mut self) -> Option<Self::OutputPin>;
    fn imu0(&mut self) -> Option<Result<Self::IMU, ()>>;
    fn imu1(&mut self) -> Option<Result<Self::IMU, ()>>;
}
