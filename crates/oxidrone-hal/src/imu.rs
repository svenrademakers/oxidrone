use core::mem::MaybeUninit;
use nalgebra::{Rotation3, Vector3};

/// Represents different types of IMU sensors
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum ImuType {
    Accelerometer,
    Gyroscope,
}

/// Configuration for sensor filtering and scaling
#[derive(Debug, Clone, Default)]
pub struct SensorConfig {
    pub gyro_scale: f32,
    pub accel_scale: f32,
    pub temp_sensitivity: f32,
    pub temp_zero: f32,
    pub backend_rate_hz: u16,
    pub sampling_rate_hz: u32,
    pub fast_sampling: bool,
    pub highres_sampling: bool,
}

/// Raw sensor data from FIFO
#[derive(Debug, Clone, Copy)]
pub struct RawSensorData {
    pub timestamp: u64,
    pub accel: Vector3<f32>,
    pub gyro: Vector3<f32>,
    pub temperature: f32,
}

/// Processed sensor data
#[derive(Debug, Clone, Default)]
pub struct ProcessedSensorData<D> {
    pub timestamp: u64,
    pub accel: Vector3<f32>,
    pub gyro: Vector3<f32>,
    pub temperature: f32,
    pub delta_time: D,
}

/// Result type for IMU operations
pub type ImuResult<T> = Result<T, ()>;

/// Trait for basic IMU sensor operations
pub trait ImuSensor {
    /// Initialize the sensor hardware
    fn initialize(&mut self) -> ImuResult<()>;

    /// Start sensor data acquisition
    fn start(&mut self) -> ImuResult<()>;

    /// Stop sensor data acquisition
    fn stop(&mut self) -> ImuResult<()>;

    /// Check if sensor is healthy/operational
    fn is_healthy(&self) -> bool;

    /// Get sensor configuration
    fn get_config(&self) -> &SensorConfig;

    /// Update sensor configuration
    fn set_config(&mut self, config: SensorConfig) -> ImuResult<()>;

    /// Get sensor unique identifier
    fn get_sensor_id(&self) -> u32;

    /// Get sensor type information
    fn get_sensor_type(&self) -> heapless::String<32>;
}

/// Trait for reading sensor data
pub trait SensorDataReader {
    type Duration;
    /// Read latest processed sensor data
    fn read_data(&mut self) -> ImuResult<ProcessedSensorData<Self::Duration>>;

    /// Check if new data is available
    fn data_available(&self) -> bool;

    /// Get the current sampling rate
    fn get_sampling_rate(&self) -> u32;

    /// Get error counts for diagnostics
    fn get_error_counts(&self) -> (u32, u32); // (gyro_errors, accel_errors)
}

/// Trait for FIFO operations
pub trait FifoOperations {
    /// Reset the FIFO buffer
    fn fifo_reset(&mut self) -> ImuResult<()>;

    /// Get number of samples available in FIFO
    fn fifo_count(&mut self) -> ImuResult<u16>;

    /// Read samples from FIFO
    fn read_fifo<'a>(
        &mut self,
        data: &'a mut [MaybeUninit<RawSensorData>],
    ) -> ImuResult<&'a [RawSensorData]>;

    /// Configure FIFO settings
    fn configure_fifo(
        &mut self,
        enable_gyro: bool,
        enable_accel: bool,
        enable_temp: bool,
    ) -> ImuResult<()>;
}

/// Trait for sensor calibration operations
pub trait SensorCalibration {
    /// Perform gyroscope calibration
    fn calibrate_gyro(&mut self) -> ImuResult<Vector3<f32>>;

    /// Perform accelerometer calibration
    fn calibrate_accel(&mut self) -> ImuResult<(Vector3<f32>, Vector3<f32>)>; // (offset, scale)

    /// Set gyroscope offsets
    fn set_gyro_offsets(&mut self, offsets: Vector3<f32>) -> ImuResult<()>;

    /// Set accelerometer offsets and scaling
    fn set_accel_calibration(
        &mut self,
        offsets: Vector3<f32>,
        scale: Vector3<f32>,
    ) -> ImuResult<()>;

    /// Get current calibration status
    fn is_calibrated(&self) -> bool;
}

/// Trait for advanced sensor features
pub trait AdvancedSensorFeatures {
    /// Enable/disable high-resolution sampling
    fn set_highres_sampling(&mut self, enable: bool) -> ImuResult<()>;

    /// Configure fast sampling rates
    fn set_fast_sampling(&mut self, rate_multiplier: u8) -> ImuResult<()>;

    /// Set sensor rotation/orientation
    fn set_rotation(&mut self, rotation: Rotation3<f32>) -> ImuResult<()>;

    /// Enable/disable sensor filters
    fn configure_filters(&mut self, gyro_lpf_hz: u16, accel_lpf_hz: u16) -> ImuResult<()>;

    /// Get sensor temperature
    fn get_temperature(&self) -> f32;

    /// Perform self-test
    fn self_test(&mut self) -> ImuResult<bool>;
}

/// Main trait combining all IMU functionality
pub trait InertialMeasurementUnit:
    ImuSensor + SensorDataReader + FifoOperations + SensorCalibration + AdvancedSensorFeatures
{
    /// Update sensor data (main processing loop)
    fn update(&mut self) -> ImuResult<()>;

    /// Get comprehensive sensor status
    fn get_status(&self) -> ImuStatus;
}

/// Comprehensive sensor status information
#[derive(Debug, Clone, Copy, Default)]
pub struct ImuStatus {
    pub is_healthy: bool,
    pub gyro_healthy: bool,
    pub accel_healthy: bool,
    pub temperature: f32,
    pub gyro_error_count: u32,
    pub accel_error_count: u32,
    pub sampling_rate: u32,
    pub backend_rate: u16,
    pub fast_sampling: bool,
    pub highres_sampling: bool,
    pub calibrated: bool,
}
