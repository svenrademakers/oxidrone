#![allow(unused_variables, dead_code)]
use crate::imu::{
    self, AdvancedSensorFeatures, FifoOperations, ImuResult, ImuSensor, ImuStatus,
    InertialMeasurementUnit, ProcessedSensorData, SensorCalibration, SensorDataReader,
};
use core::str::FromStr;
use nalgebra::Vector3;

#[derive(Debug, Default)]
pub struct DummyImu {
    config: imu::SensorConfig,
}

impl InertialMeasurementUnit for DummyImu {
    fn update(&mut self) -> imu::ImuResult<()> {
        Ok(())
    }

    fn get_status(&self) -> imu::ImuStatus {
        ImuStatus::default()
    }
}

impl ImuSensor for DummyImu {
    fn initialize(&mut self) -> imu::ImuResult<()> {
        Ok(())
    }

    fn start(&mut self) -> imu::ImuResult<()> {
        Ok(())
    }

    fn stop(&mut self) -> imu::ImuResult<()> {
        Ok(())
    }

    fn is_healthy(&self) -> bool {
        true
    }

    fn get_config(&self) -> &imu::SensorConfig {
        &self.config
    }

    fn set_config(&mut self, config: imu::SensorConfig) -> imu::ImuResult<()> {
        Ok(())
    }

    fn get_sensor_id(&self) -> u32 {
        0
    }

    fn get_sensor_type(&self) -> heapless::String<32> {
        heapless::String::from_str("dummy imu").unwrap()
    }
}

impl SensorDataReader for DummyImu {
    type Duration = u32;

    fn read_data(&mut self) -> imu::ImuResult<imu::ProcessedSensorData<Self::Duration>> {
        Ok(ProcessedSensorData::default())
    }

    fn data_available(&self) -> bool {
        true
    }

    fn get_sampling_rate(&self) -> u32 {
        1
    }

    fn get_error_counts(&self) -> (u32, u32) {
        (0, 0)
    }
}

impl FifoOperations for DummyImu {
    fn fifo_reset(&mut self) -> imu::ImuResult<()> {
        Ok(())
    }

    fn fifo_count(&mut self) -> imu::ImuResult<u16> {
        Ok(0)
    }

    fn read_fifo<'a>(
        &mut self,
        data: &'a mut [core::mem::MaybeUninit<imu::RawSensorData>],
    ) -> imu::ImuResult<&'a [imu::RawSensorData]> {
        Ok(&[])
    }

    fn configure_fifo(
        &mut self,
        enable_gyro: bool,
        enable_accel: bool,
        enable_temp: bool,
    ) -> imu::ImuResult<()> {
        Ok(())
    }
}

impl SensorCalibration for DummyImu {
    fn calibrate_gyro(&mut self) -> ImuResult<Vector3<f32>> {
        // Return a fixed fake gyro offset
        Ok(Vector3::new(0.01, -0.02, 0.005))
    }

    fn calibrate_accel(&mut self) -> ImuResult<(Vector3<f32>, Vector3<f32>)> {
        // Return fixed fake accel offset and scale
        Ok((
            Vector3::new(-0.1, 0.05, 0.02), // offset
            Vector3::new(1.01, 0.99, 1.02), // scale
        ))
    }

    fn set_gyro_offsets(&mut self, _offsets: Vector3<f32>) -> ImuResult<()> {
        // Accept and discard
        Ok(())
    }

    fn set_accel_calibration(
        &mut self,
        _offsets: Vector3<f32>,
        _scale: Vector3<f32>,
    ) -> ImuResult<()> {
        // Accept and discard
        Ok(())
    }

    fn is_calibrated(&self) -> bool {
        true
    }
}

impl AdvancedSensorFeatures for DummyImu {
    fn set_highres_sampling(&mut self, enable: bool) -> imu::ImuResult<()> {
        Ok(())
    }

    fn set_fast_sampling(&mut self, rate_multiplier: u8) -> imu::ImuResult<()> {
        Ok(())
    }

    fn set_rotation(&mut self, rotation: nalgebra::Rotation3<f32>) -> imu::ImuResult<()> {
        Ok(())
    }

    fn configure_filters(&mut self, gyro_lpf_hz: u16, accel_lpf_hz: u16) -> imu::ImuResult<()> {
        Ok(())
    }

    fn get_temperature(&self) -> f32 {
        0f32
    }

    fn self_test(&mut self) -> imu::ImuResult<bool> {
        Ok(true)
    }
}
