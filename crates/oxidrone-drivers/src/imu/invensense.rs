use core::ops::Sub;
use core::{mem::MaybeUninit, time::Duration};
use defmt::{Format, error, warn};
use embedded_hal::spi::{Operation, SpiDevice};
use heapless::String;
use nalgebra::Rotation3;
use nalgebra::Vector3;
use oxidrone_hal::imu::*;
use rtic_time::Monotonic;

#[allow(dead_code)]
mod registers {
    // Common registers
    pub const INV3REG_WHOAMI: u8 = 0x75;
    pub const INV3REG_FIFO_CONFIG: u8 = 0x16;
    pub const INV3REG_PWR_MGMT0: u8 = 0x4e;
    pub const INV3REG_GYRO_CONFIG0: u8 = 0x4f;
    pub const INV3REG_ACCEL_CONFIG0: u8 = 0x50;
    pub const INV3REG_GYRO_CONFIG1: u8 = 0x51;
    pub const INV3REG_GYRO_ACCEL_CONFIG0: u8 = 0x52;
    pub const INV3REG_ACCEL_CONFIG1: u8 = 0x53;
    pub const INV3REG_FIFO_CONFIG1: u8 = 0x5f;
    pub const INV3REG_FIFO_CONFIG2: u8 = 0x60;
    pub const INV3REG_FIFO_CONFIG3: u8 = 0x61;
    pub const INV3REG_SIGNAL_PATH_RESET: u8 = 0x4b;
    pub const INV3REG_INTF_CONFIG0: u8 = 0x4c;
    pub const INV3REG_FIFO_COUNTH: u8 = 0x2e;
    pub const INV3REG_FIFO_DATA: u8 = 0x30;
    pub const INV3REG_BANK_SEL: u8 = 0x76;
    pub const INV3REG_DEVICE_CONFIG: u8 = 0x11;
    pub const INV3REG_INT_STATUS: u8 = 0x2D;

    // ICM42xxx specific
    pub const INV3REG_42XXX_INTF_CONFIG1: u8 = 0x4d;
    pub const INV3REG_42XXX_INTF_CONFIG5: u8 = 0x7b;

    // WHOAMI values
    pub const INV3_ID_ICM40605: u8 = 0x33;
    pub const INV3_ID_ICM40609: u8 = 0x3b;
    pub const INV3_ID_ICM42605: u8 = 0x42;
    pub const INV3_ID_ICM42688: u8 = 0x47;
    pub const INV3_ID_IIM42652: u8 = 0x6f;
    pub const INV3_ID_IIM42653: u8 = 0x56;
    pub const INV3_ID_ICM42670: u8 = 0x67;
    pub const INV3_ID_ICM45686: u8 = 0xE9;

    // SPI read flag
    pub const BIT_READ_FLAG: u8 = 0x80;
}

#[repr(C, packed)]
#[derive(Debug, Clone, Copy)]
pub struct FifoData {
    pub header: u8,
    pub accel: [i16; 3],
    pub gyro: [i16; 3],
    pub temperature: i8,
    pub timestamp: u16,
}

#[repr(C, packed)]
#[derive(Debug, Clone, Copy)]
pub struct FifoDataHighRes {
    pub header: u8,
    pub accel: [u8; 6],
    pub gyro: [u8; 6],
    pub temperature: i16,
    pub timestamp: u16,
    pub extra_bits: [u8; 3], // ax:4, gx:4, ay:4, gy:4, az:4, gz:4
}

const FIFO_SAMPLE_SIZE: usize = core::mem::size_of::<FifoData>();
const FIFO_HIGHRES_SAMPLE_SIZE: usize = core::mem::size_of::<FifoDataHighRes>();
const FIFO_BUFFER_SAMPLES: usize = 24;
const FIFO_BUFFER_SIZE_HIGH_RES: usize = FIFO_BUFFER_SAMPLES * FIFO_HIGHRES_SAMPLE_SIZE + 1;

/// Invensense V3 IMU driver
pub struct InvensenseV3<SPI, DELAY, MONO>
where
    SPI: SpiDevice,
    DELAY: embedded_hal::delay::DelayNs,
    MONO: Monotonic,
{
    spi: SPI,
    delay: DELAY,
    sensor_type: Invensensev3Type,
    config: SensorConfig,

    // State
    is_healthy: bool,
    gyro_healthy: bool,
    accel_healthy: bool,

    // Error tracking
    gyro_error_count: u32,
    accel_error_count: u32,

    // Calibration
    gyro_offsets: Vector3<f32>,
    accel_offsets: Vector3<f32>,
    accel_scale: Vector3<f32>,
    is_calibrated: bool,

    spi_buffer: [u8; FIFO_BUFFER_SIZE_HIGH_RES],
    fifo_config1: u8,
    last_update: MONO::Instant,

    // Timing
    backend_period: Duration,

    // Temperature filtering
    temp_filtered: f32,
}

impl<SPI, DELAY, MONO> InvensenseV3<SPI, DELAY, MONO>
where
    SPI: SpiDevice,
    DELAY: embedded_hal::delay::DelayNs,
    MONO: Monotonic,
{
    pub fn new(spi: SPI, delay: DELAY) -> Self {
        let default_config = SensorConfig {
            gyro_scale: Self::gyro_scale_2000dps(),
            accel_scale: Self::accel_scale_16g(),
            temp_sensitivity: 1.0 / 2.07,
            temp_zero: 25.0,
            backend_rate_hz: 1000,
            sampling_rate_hz: 1000,
            fast_sampling: false,
            highres_sampling: false,
        };

        Self {
            spi,
            delay,
            sensor_type: Invensensev3Type::Unknown,
            config: default_config,
            is_healthy: false,
            gyro_healthy: false,
            accel_healthy: false,
            gyro_error_count: 0,
            accel_error_count: 0,
            gyro_offsets: Vector3::zeros(),
            accel_offsets: Vector3::zeros(),
            accel_scale: Vector3::new(1.0, 1.0, 1.0),
            is_calibrated: false,
            spi_buffer: [0; FIFO_BUFFER_SIZE_HIGH_RES],
            fifo_config1: 0x07,
            last_update: MONO::now(),
            backend_period: Duration::from_micros(1000), // 1kHz default
            temp_filtered: 25.0,
        }
    }

    /// Scaling constants
    const fn gyro_scale_2000dps() -> f32 {
        (core::f32::consts::PI / 180.0) / (32768.0 / 2000.0)
    }

    const fn gyro_scale_4000dps() -> f32 {
        (core::f32::consts::PI / 180.0) / (32768.0 / 4000.0)
    }

    const fn gyro_scale_highres_2000dps() -> f32 {
        (core::f32::consts::PI / 180.0) / (524288.0 / 2000.0)
    }

    const fn gyro_scale_highres_4000dps() -> f32 {
        (core::f32::consts::PI / 180.0) / (524288.0 / 4000.0)
    }

    const fn accel_scale_16g() -> f32 {
        9.80665 / (32768.0 / 16.0)
    }

    const fn accel_scale_32g() -> f32 {
        9.80665 / (32768.0 / 32.0)
    }

    const fn accel_scale_highres_16g() -> f32 {
        9.80665 / (524288.0 / 16.0)
    }

    const fn accel_scale_highres_32g() -> f32 {
        9.80665 / (524288.0 / 32.0)
    }

    fn read_register(&mut self, reg: u8) -> ImuResult<u8> {
        let tx_buf = [reg | registers::BIT_READ_FLAG, 0];
        let mut rx_buf = [0u8; 2];

        if self.spi.transfer(&mut rx_buf, &tx_buf).is_err() {
            error!("invensense read register failed");
            return Err(());
        }

        Ok(rx_buf[1])
    }

    fn write_register(&mut self, reg: u8, value: u8) -> ImuResult<()> {
        let tx_buf = [reg, value];

        if self.spi.write(&tx_buf).is_err() {
            error!("invensense write register failed");
            return Err(());
        }

        Ok(())
    }

    /// Read multiple registers
    fn read_registers(&mut self, reg: u8, data: &mut [u8]) -> ImuResult<()> {
        let cmd = [reg | registers::BIT_READ_FLAG];
        if self
            .spi
            .transaction(&mut [
                Operation::Write(&cmd),
                Operation::TransferInPlace(data), // clock in data directly into `data`
            ])
            .is_err()
        {
            error!("SPI bulk read failed");
            return Err(());
        }
        Ok(())
    }

    /// Check WHO_AM_I register and detect sensor type
    fn check_whoami(&mut self) -> ImuResult<()> {
        let whoami = self.read_register(registers::INV3REG_WHOAMI)?;

        self.sensor_type = match whoami {
            registers::INV3_ID_ICM40609 => Invensensev3Type::ICM40609,
            registers::INV3_ID_ICM42688 => Invensensev3Type::ICM42688,
            registers::INV3_ID_ICM42605 => Invensensev3Type::ICM42605,
            registers::INV3_ID_ICM40605 => Invensensev3Type::ICM40605,
            registers::INV3_ID_IIM42652 => Invensensev3Type::IIM42652,
            registers::INV3_ID_IIM42653 => Invensensev3Type::IIM42653,
            registers::INV3_ID_ICM42670 => Invensensev3Type::ICM42670,
            registers::INV3_ID_ICM45686 => Invensensev3Type::ICM45686,
            _ => {
                error!("Unknown WHOAMI: 0x{:02x}", whoami);
                return Err(());
            }
        };

        Ok(())
    }

    /// Configure sensor based on detected type
    fn configure_sensor(&mut self) -> ImuResult<()> {
        match self.sensor_type {
            Invensensev3Type::IIM42652 => {
                self.config.temp_sensitivity = 1.0 / 2.07;
            }
            Invensensev3Type::IIM42653 => {
                self.config.temp_sensitivity = 1.0 / 2.07;
                self.config.gyro_scale = Self::gyro_scale_4000dps();
                self.config.accel_scale = Self::accel_scale_32g();
            }
            Invensensev3Type::ICM42688 => {
                self.config.temp_sensitivity = 1.0 / 2.07;
            }
            Invensensev3Type::ICM42605 => {
                self.config.temp_sensitivity = 1.0 / 2.07;
            }
            Invensensev3Type::ICM40605 => {
                self.fifo_config1 = 0x0F;
                self.config.temp_sensitivity = 1.0 * 128.0 / 115.49;
            }
            Invensensev3Type::ICM42670 => {
                self.config.temp_sensitivity = 1.0 / 2.0;
            }
            Invensensev3Type::ICM45686 => {
                self.config.temp_sensitivity = 1.0 / 2.0;
                self.config.gyro_scale = Self::gyro_scale_4000dps();
                self.config.accel_scale = Self::accel_scale_32g();
            }
            Invensensev3Type::ICM40609 => {
                self.config.temp_sensitivity = 1.0 / 2.07;
                self.config.accel_scale = Self::accel_scale_32g();
            }
            Invensensev3Type::Unknown => {
                warn!("Unknown sensor. cannot configure.");
            }
        }

        // Configure high-res sampling if supported and enabled
        if self.config.highres_sampling {
            match self.sensor_type {
                Invensensev3Type::ICM42688
                | Invensensev3Type::IIM42652
                | Invensensev3Type::IIM42653 => {
                    self.fifo_config1 |= 1 << 4; // FIFO_HIRES_EN
                    self.config.gyro_scale = Self::gyro_scale_highres_2000dps();
                    self.config.accel_scale = Self::accel_scale_highres_16g();
                    self.config.temp_sensitivity = 1.0 / 132.48;
                }
                Invensensev3Type::ICM45686 => {
                    self.fifo_config1 |= 1 << 4; // FIFO_HIRES_EN
                    self.config.temp_sensitivity = 1.0 / 128.0;
                    self.config.accel_scale = Self::accel_scale_highres_32g();
                    self.config.gyro_scale = Self::gyro_scale_highres_4000dps();
                }
                _ => {
                    warn!(
                        "High-res not supported for {:x}, disable it",
                        self.sensor_type
                    );
                    self.config.highres_sampling = false;
                }
            }
        }

        Ok(())
    }

    /// Set filter and scaling configuration
    fn set_filter_and_scaling(&mut self) -> ImuResult<()> {
        let mut odr_config = 0x06u8;
        let mut backend_rate_hz = self.config.backend_rate_hz;

        // Configure fast sampling if enabled
        if self.config.fast_sampling {
            backend_rate_hz =
                self.calculate_fast_sampling_backend_rate(backend_rate_hz, 8 * backend_rate_hz);

            odr_config = match backend_rate_hz {
                2000 => 0x05, // 2KHz
                4000 => 0x04, // 4KHz
                8000 => 0x03, // 8KHz
                _ => 0x06,    // 1KHz default
            };
        }

        self.config.backend_rate_hz = backend_rate_hz;
        self.config.sampling_rate_hz = backend_rate_hz as u32;
        self.backend_period = Duration::from_micros(1_000_000u64 / backend_rate_hz as u64);

        // Disable gyro and accel first
        self.write_register(registers::INV3REG_PWR_MGMT0, 0x00)?;

        // Setup gyro and accel for backend rate
        self.write_register(registers::INV3REG_GYRO_CONFIG0, odr_config)?;
        self.write_register(registers::INV3REG_ACCEL_CONFIG0, odr_config)?;

        // Enable gyro and accel in low-noise modes
        self.write_register(registers::INV3REG_PWR_MGMT0, 0x0F)?;

        // Small delay for power up
        self.delay.delay_ms(300);

        Ok(())
    }

    /// Calculate fast sampling backend rate
    fn calculate_fast_sampling_backend_rate(&self, base_rate: u16, max_rate: u16) -> u16 {
        // This would normally depend on loop rate and fast sampling rate settings
        // For now, return the base rate
        core::cmp::min(base_rate * 2, max_rate) // Example: double the rate
    }

    /// Convert 20-bit value to float for high-res mode
    fn uint20_to_float(msb: u8, bits: u8, lsb: u8) -> f32 {
        let value20bit = ((msb as u32) << 12) | ((bits as u32) << 4) | (lsb as u32);
        let value32bit = if value20bit & 0x80000 != 0 {
            // Sign extend for negative values
            (value20bit | 0xFFF00000) as i32
        } else {
            value20bit as i32
        };
        value32bit as f32
    }

    /// Process standard resolution FIFO samples
    fn process_fifo_samples(
        &mut self,
        samples: &[FifoData],
        data: &mut [MaybeUninit<RawSensorData>],
    ) -> ImuResult<usize> {
        assert!(data.len() >= samples.len());

        for (i, sample) in samples.into_iter().enumerate() {
            if (sample.header & 0xFC) != 0x68 {
                error!("Invalid FIFO header");
                return Err(());
            }

            let mut accel = Vector3::new(
                sample.accel[0] as f32,
                sample.accel[1] as f32,
                sample.accel[2] as f32,
            );

            let mut gyro = Vector3::new(
                sample.gyro[0] as f32,
                sample.gyro[1] as f32,
                sample.gyro[2] as f32,
            );

            // Apply scaling
            accel *= self.config.accel_scale;
            gyro *= self.config.gyro_scale;

            // Apply calibration
            gyro -= self.gyro_offsets;
            accel = Vector3::new(
                (accel.x - self.accel_offsets.x) * self.accel_scale.x,
                (accel.y - self.accel_offsets.y) * self.accel_scale.y,
                (accel.z - self.accel_offsets.z) * self.accel_scale.z,
            );

            let temperature =
                sample.temperature as f32 * self.config.temp_sensitivity + self.config.temp_zero;

            data[i].write(RawSensorData {
                timestamp: sample.timestamp as u64,
                accel,
                gyro,
                temperature,
            });
        }

        Ok(samples.len())
    }

    /// Process high-resolution FIFO samples
    fn process_highres_fifo_samples(
        &mut self,
        samples: &[FifoDataHighRes],
        data: &mut [MaybeUninit<RawSensorData>],
    ) -> ImuResult<usize> {
        assert!(data.len() >= samples.len());
        for (i, sample) in samples.into_iter().enumerate() {
            if (sample.header & 0xFC) != 0x78 {
                error!("Invalid high-res FIFO header");
                return Err(());
            }

            // Extract high-resolution data
            let ax = (sample.extra_bits[0] & 0x0F) as u8;
            let gx = ((sample.extra_bits[0] & 0xF0) >> 4) as u8;
            let ay = (sample.extra_bits[1] & 0x0F) as u8;
            let gy = ((sample.extra_bits[1] & 0xF0) >> 4) as u8;
            let az = (sample.extra_bits[2] & 0x0F) as u8;
            let gz = ((sample.extra_bits[2] & 0xF0) >> 4) as u8;

            let mut accel = Vector3::new(
                Self::uint20_to_float(sample.accel[1], sample.accel[0], ax),
                Self::uint20_to_float(sample.accel[3], sample.accel[2], ay),
                Self::uint20_to_float(sample.accel[5], sample.accel[4], az),
            );

            let mut gyro = Vector3::new(
                Self::uint20_to_float(sample.gyro[1], sample.gyro[0], gx),
                Self::uint20_to_float(sample.gyro[3], sample.gyro[2], gy),
                Self::uint20_to_float(sample.gyro[5], sample.gyro[4], gz),
            );

            // Apply scaling
            accel *= self.config.accel_scale;
            gyro *= self.config.gyro_scale;

            // Apply calibration
            gyro -= self.gyro_offsets;
            accel = Vector3::new(
                (accel.x - self.accel_offsets.x) * self.accel_scale.x,
                (accel.y - self.accel_offsets.y) * self.accel_scale.y,
                (accel.z - self.accel_offsets.z) * self.accel_scale.z,
            );

            let temperature =
                sample.temperature as f32 * self.config.temp_sensitivity + self.config.temp_zero;

            data[i].write(RawSensorData {
                timestamp: sample.timestamp as u64,
                accel,
                gyro,
                temperature,
            });
        }

        Ok(samples.len())
    }
}

/// Implementation of the ImuSensor trait
impl<SPI, DELAY, MONO> ImuSensor for InvensenseV3<SPI, DELAY, MONO>
where
    SPI: SpiDevice,
    DELAY: embedded_hal::delay::DelayNs,
    MONO: Monotonic,
{
    fn initialize(&mut self) -> ImuResult<()> {
        // Check WHO_AM_I and detect sensor type
        self.check_whoami()?;

        // Configure sensor based on detected type
        self.configure_sensor()?;

        // Set up filtering and scaling
        self.set_filter_and_scaling()?;

        // Reset FIFO
        self.fifo_reset()?;

        self.is_healthy = true;
        self.gyro_healthy = true;
        self.accel_healthy = true;

        Ok(())
    }

    fn start(&mut self) -> ImuResult<()> {
        // Enable sensors
        self.write_register(registers::INV3REG_PWR_MGMT0, 0x0F)?;

        // Configure FIFO
        self.configure_fifo(true, true, true)?;

        Ok(())
    }

    fn stop(&mut self) -> ImuResult<()> {
        // Disable sensors
        self.write_register(registers::INV3REG_PWR_MGMT0, 0x00)?;
        Ok(())
    }

    fn is_healthy(&self) -> bool {
        self.is_healthy && self.gyro_healthy && self.accel_healthy
    }

    fn get_config(&self) -> &SensorConfig {
        &self.config
    }

    fn set_config(&mut self, config: SensorConfig) -> ImuResult<()> {
        self.config = config;
        self.configure_sensor()?;
        self.set_filter_and_scaling()?;
        Ok(())
    }

    fn get_sensor_id(&self) -> u32 {
        // Generate a unique ID based on sensor type
        match self.sensor_type {
            Invensensev3Type::ICM40609 => 0x40609,
            Invensensev3Type::ICM42688 => 0x42688,
            Invensensev3Type::ICM42605 => 0x42605,
            Invensensev3Type::ICM40605 => 0x40605,
            Invensensev3Type::IIM42652 => 0x42652,
            Invensensev3Type::IIM42653 => 0x42653,
            Invensensev3Type::ICM42670 => 0x42670,
            Invensensev3Type::ICM45686 => 0x45686,
            Invensensev3Type::Unknown => {
                error!("unknown sensor id");
                0
            }
        }
    }

    fn get_sensor_type(&self) -> String<32> {
        use core::fmt::Write;
        let mut sensor_type = String::<32>::new();
        write!(sensor_type, "{:?}", self.sensor_type).unwrap();
        sensor_type
    }
}

/// Implementation of the FifoOperations trait
impl<SPI, DELAY, MONO> FifoOperations for InvensenseV3<SPI, DELAY, MONO>
where
    SPI: SpiDevice,
    DELAY: embedded_hal::delay::DelayNs,
    MONO: Monotonic,
{
    fn fifo_reset(&mut self) -> ImuResult<()> {
        match self.sensor_type {
            Invensensev3Type::ICM45686 => {
                // FIFO_FLUSH for ICM45686
                self.write_register(0x20, 0x80)?; // FIFO_CONFIG2
                self.write_register(0x20, 0x00)?;
            }
            _ => {
                // Standard FIFO reset
                self.write_register(registers::INV3REG_FIFO_CONFIG, 0x80)?; // Stop-on-full
                self.write_register(registers::INV3REG_FIFO_CONFIG1, self.fifo_config1)?;
                self.write_register(registers::INV3REG_INTF_CONFIG0, 0xC0)?; // Little-endian, count in records
                self.write_register(registers::INV3REG_SIGNAL_PATH_RESET, 2)?;
            }
        }

        Ok(())
    }

    fn fifo_count(&mut self) -> ImuResult<u16> {
        let mut count_bytes = [0u8; 2];
        self.read_registers(registers::INV3REG_FIFO_COUNTH, &mut count_bytes)?;
        Ok(u16::from_be_bytes(count_bytes))
    }

    fn read_fifo<'a>(
        &mut self,
        data: &'a mut [MaybeUninit<RawSensorData>],
    ) -> ImuResult<&'a [RawSensorData]> {
        let n_samples = self.fifo_count()?;

        if n_samples == 0 {
            return Ok(&[]);
        }

        let sample_size = if self.config.highres_sampling {
            FIFO_HIGHRES_SAMPLE_SIZE
        } else {
            FIFO_SAMPLE_SIZE
        };

        let max_samples = core::cmp::min(n_samples as usize, FIFO_BUFFER_SAMPLES);
        let read_size = max_samples * sample_size;
        self.spi_buffer[..read_size].fill(0);

        if self
            .spi
            .transaction(&mut [
                Operation::Write(&[registers::INV3REG_FIFO_DATA | registers::BIT_READ_FLAG]),
                Operation::TransferInPlace(&mut self.spi_buffer), // clock in data directly into `data`
            ])
            .is_err()
        {
            error!("FIFO read failed");
            return Err(());
        }

        // Process samples
        let write_len;
        if self.config.highres_sampling {
            let samples_ptr = self.spi_buffer[..read_size].as_ptr() as *const FifoDataHighRes;
            let samples = unsafe { core::slice::from_raw_parts(samples_ptr, max_samples) };
            write_len = self.process_highres_fifo_samples(samples, data)?;
        } else {
            let samples_ptr = self.spi_buffer[..read_size].as_ptr() as *const FifoData;
            let samples = unsafe { core::slice::from_raw_parts(samples_ptr, max_samples) };
            write_len = self.process_fifo_samples(samples, data)?;
        }

        let slice = unsafe { core::mem::transmute::<_, &'a [RawSensorData]>(&data[..write_len]) };
        Ok(slice)
    }

    fn configure_fifo(
        &mut self,
        enable_gyro: bool,
        enable_accel: bool,
        enable_temp: bool,
    ) -> ImuResult<()> {
        let mut config = 0u8;

        if enable_gyro {
            config |= 0x0E; // XG_FIFO_EN | YG_FIFO_EN | ZG_FIFO_EN
        }
        if enable_accel {
            config |= 0x10; // ACCEL_FIFO_EN
        }
        if enable_temp {
            config |= 0x01; // TEMP_FIFO_EN
        }

        self.write_register(registers::INV3REG_FIFO_CONFIG2, config)?;
        Ok(())
    }
}

/// Implementation of the SensorCalibration trait
impl<SPI, DELAY, MONO> SensorCalibration for InvensenseV3<SPI, DELAY, MONO>
where
    SPI: SpiDevice,
    DELAY: embedded_hal::delay::DelayNs,
    MONO: Monotonic,
{
    fn calibrate_gyro(&mut self) -> ImuResult<Vector3<f32>> {
        const CALIBRATION_SAMPLES: usize = 1000;
        let mut gyro_sum = Vector3::zeros();

        // Collect samples for calibration
        for _ in 0..CALIBRATION_SAMPLES {
            let data = self.read_data()?;
            gyro_sum += data.gyro;
            self.delay.delay_ms(1);
        }

        // Calculate average offset
        self.gyro_offsets = gyro_sum / CALIBRATION_SAMPLES as f32;

        Ok(self.gyro_offsets)
    }

    fn calibrate_accel(&mut self) -> ImuResult<(Vector3<f32>, Vector3<f32>)> {
        // This is a simplified 1G calibration
        // For full 6-point calibration, more complex logic would be needed
        const CALIBRATION_SAMPLES: usize = 1000;
        let mut accel_sum = Vector3::zeros();

        for _ in 0..CALIBRATION_SAMPLES {
            let data = self.read_data()?;
            accel_sum += data.accel;
            self.delay.delay_ms(1);
        }

        let accel_avg = accel_sum / CALIBRATION_SAMPLES as f32;

        // Assume Z-axis is pointing down (1G)
        self.accel_offsets = Vector3::new(
            accel_avg.x,
            accel_avg.y,
            accel_avg.z - 9.80665, // Remove 1G from Z-axis
        );

        // Set scale to 1.0 for simple calibration
        self.accel_scale = Vector3::new(1.0, 1.0, 1.0);

        Ok((self.accel_offsets, self.accel_scale))
    }

    fn set_gyro_offsets(&mut self, offsets: Vector3<f32>) -> ImuResult<()> {
        self.gyro_offsets = offsets;
        Ok(())
    }

    fn set_accel_calibration(
        &mut self,
        offsets: Vector3<f32>,
        scale: Vector3<f32>,
    ) -> ImuResult<()> {
        self.accel_offsets = offsets;
        self.accel_scale = scale;
        Ok(())
    }

    fn is_calibrated(&self) -> bool {
        self.is_calibrated
    }
}

/// Implementation of the AdvancedSensorFeatures trait
impl<SPI, DELAY, MONO> AdvancedSensorFeatures for InvensenseV3<SPI, DELAY, MONO>
where
    SPI: SpiDevice,
    DELAY: embedded_hal::delay::DelayNs,
    MONO: Monotonic,
{
    fn set_highres_sampling(&mut self, enable: bool) -> ImuResult<()> {
        if self.sensor_type.high_res_supported() {
            self.config.highres_sampling = enable;
            self.configure_sensor()?;
            Ok(())
        } else {
            error!(
                "High-resolution sampling not supported for {:?}",
                &self.sensor_type
            );
            Err(())
        }
    }

    fn set_fast_sampling(&mut self, rate_multiplier: u8) -> ImuResult<()> {
        self.config.fast_sampling = rate_multiplier > 1;
        self.set_filter_and_scaling()?;
        Ok(())
    }

    fn set_rotation(&mut self, _rotation: Rotation3<f32>) -> ImuResult<()> {
        todo!()
    }

    fn configure_filters(&mut self, _gyro_lpf_hz: u16, _accel_lpf_hz: u16) -> ImuResult<()> {
        todo!()
    }

    fn get_temperature(&self) -> f32 {
        self.temp_filtered
    }

    fn self_test(&mut self) -> ImuResult<bool> {
        // Implement self-test procedure
        // This would typically involve enabling self-test mode and checking responses
        Ok(true)
    }
}

/// Implementation of the main InertialMeasurementUnit trait
impl<SPI, DELAY, MONO> InertialMeasurementUnit for InvensenseV3<SPI, DELAY, MONO>
where
    SPI: SpiDevice,
    DELAY: embedded_hal::delay::DelayNs,
    MONO: Monotonic,
{
    fn update(&mut self) -> ImuResult<()> {
        // Read and process new data if available
        if self.data_available() {
            let _data = self.read_data()?;
            // Process the data as needed
        }

        // Update health status
        self.is_healthy = self.gyro_healthy && self.accel_healthy;

        Ok(())
    }

    fn get_status(&self) -> ImuStatus {
        ImuStatus {
            is_healthy: self.is_healthy,
            gyro_healthy: self.gyro_healthy,
            accel_healthy: self.accel_healthy,
            temperature: self.temp_filtered,
            gyro_error_count: self.gyro_error_count,
            accel_error_count: self.accel_error_count,
            sampling_rate: self.config.sampling_rate_hz,
            backend_rate: self.config.backend_rate_hz,
            fast_sampling: self.config.fast_sampling,
            highres_sampling: self.config.highres_sampling,
            calibrated: self.is_calibrated,
        }
    }
}

impl<SPI, DELAY, MONO> SensorDataReader for InvensenseV3<SPI, DELAY, MONO>
where
    SPI: SpiDevice,
    DELAY: embedded_hal::delay::DelayNs,
    MONO: Monotonic,
{
    type Duration = MONO::Duration;

    fn read_data(&mut self) -> ImuResult<ProcessedSensorData<Self::Duration>> {
        let mut raw_samples = [const { MaybeUninit::uninit() }; FIFO_BUFFER_SAMPLES];
        let raw_samples = self.read_fifo(&mut raw_samples)?;

        if raw_samples.is_empty() {
            error!("no data available");
            return Err(());
        }

        // Use the latest sample
        let latest = &raw_samples[raw_samples.len() - 1];

        // Update filtered temperature
        const TEMP_FILTER_ALPHA: f32 = 0.1;
        self.temp_filtered =
            self.temp_filtered * (1.0 - TEMP_FILTER_ALPHA) + latest.temperature * TEMP_FILTER_ALPHA;

        // Calculate delta time
        let now = MONO::now();
        let delta_time = now.sub(self.last_update);
        self.last_update = now;

        Ok(ProcessedSensorData {
            timestamp: latest.timestamp,
            accel: latest.accel,
            gyro: latest.gyro,
            temperature: self.temp_filtered,
            delta_time,
        })
    }

    fn data_available(&self) -> bool {
        //TODO
        true
    }

    fn get_sampling_rate(&self) -> u32 {
        self.config.sampling_rate_hz
    }

    fn get_error_counts(&self) -> (u32, u32) {
        (self.gyro_error_count, self.accel_error_count)
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Format)]
pub enum Invensensev3Type {
    ICM40609,
    ICM42688,
    ICM42605,
    ICM40605,
    IIM42652,
    IIM42653,
    ICM42670,
    ICM45686,
    Unknown,
}

impl Invensensev3Type {
    pub fn high_res_supported(&self) -> bool {
        match self {
            Invensensev3Type::ICM42688
            | Invensensev3Type::IIM42652
            | Invensensev3Type::IIM42653
            | Invensensev3Type::ICM45686 => true,
            _ => false,
        }
    }
}
