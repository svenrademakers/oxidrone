use embedded_hal::spi::MODE_0;
use oxidrone_drivers::dummy::DummyImu;
use oxidrone_hal::platform::PlatformAbstraction;
use stm32h7xx_hal::gpio::Speed;
use stm32h7xx_hal::gpio::{EPin, GpioExt, Output, PushPull};
use stm32h7xx_hal::pac::Peripherals;
use stm32h7xx_hal::prelude::*;
use stm32h7xx_hal::stm32::TIM2;
use stm32h7xx_hal::time::MilliSeconds;
use stm32h7xx_hal::timer::{Event, Timer};

pub struct PlatformMatek {
    tim2: Option<Timer<TIM2>>,
    green_led: Option<EPin<Output<PushPull>>>,
    blue_led: Option<EPin<Output<PushPull>>>,
}

impl PlatformMatek {
    pub fn system_init(dp: Peripherals) -> Self {
        let pwr = dp.PWR.constrain();
        let pwrcfg = pwr.freeze();

        let rcc = dp.RCC.constrain();
        let ccdr = rcc
            .sys_ck(200.MHz())
            .pll1_q_ck(48.MHz())
            .freeze(pwrcfg, &dp.SYSCFG);
        let mut tim2 = dp.TIM2.timer(
            MilliSeconds::from_ticks(200).into_rate(),
            ccdr.peripheral.TIM2,
            &ccdr.clocks,
        );
        tim2.listen(Event::TimeOut);

        let gpioa = dp.GPIOA.split(ccdr.peripheral.GPIOA);
        let gpiod = dp.GPIOD.split(ccdr.peripheral.GPIOD);
        let gpioc = dp.GPIOC.split(ccdr.peripheral.GPIOC);
        let gpioe = dp.GPIOE.split(ccdr.peripheral.GPIOE);

        // imu0
        let imu0_sck = gpioa.pa5.into_alternate();
        let imu0_miso = gpioa.pa6.into_alternate();
        let imu0_mosi = gpiod.pd7.into_alternate();
        let _imu0_cs = gpioc.pc15.into_push_pull_output().speed(Speed::VeryHigh);
        let _spi1: stm32h7xx_hal::spi::Spi<_, _, u8> = dp.SPI1.spi(
            (imu0_sck, imu0_miso, imu0_mosi),
            MODE_0,
            2.MHz(),
            ccdr.peripheral.SPI1,
            &ccdr.clocks,
        );

        let green_led = Some(gpioe.pe4.into_push_pull_output().into());
        let blue_led = Some(gpioe.pe3.into_push_pull_output().into());

        PlatformMatek {
            tim2: Some(tim2),
            green_led,
            blue_led,
        }
    }

    pub fn timer2(&mut self) -> Option<Timer<TIM2>> {
        self.tim2.take()
    }
}

impl PlatformAbstraction for PlatformMatek {
    type OutputPin = EPin<Output<PushPull>>;
    type IMU = DummyImu;

    fn primary_led(&mut self) -> Option<Self::OutputPin> {
        self.green_led.take()
    }

    fn secondary_led(&mut self) -> Option<Self::OutputPin> {
        self.blue_led.take()
    }

    fn imu0(&mut self) -> Option<Result<Self::IMU, ()>> {
        Some(Ok(DummyImu::default()))
    }

    fn imu1(&mut self) -> Option<Result<Self::IMU, ()>> {
        Some(Ok(DummyImu::default()))
    }
}
