use core::mem::MaybeUninit;
use embedded_hal::spi::MODE_0;
use oxidrone_hal::dummy::DummyImu;
use oxidrone_hal::platform::PlatformAbstraction;
use static_cell::StaticCell;
use stm32h7xx_hal::gpio::Speed;
use stm32h7xx_hal::gpio::{EPin, GpioExt, Output, PushPull};
use stm32h7xx_hal::pac::Peripherals;
use stm32h7xx_hal::prelude::*;
use stm32h7xx_hal::rcc::rec::UsbClkSel;
use stm32h7xx_hal::stm32::TIM2;
use stm32h7xx_hal::time::MilliSeconds;
use stm32h7xx_hal::timer::{Event, Timer};
use stm32h7xx_hal::usb_hs::{USB2, UsbBus};
use usb_device::bus::UsbBusAllocator;

static EP_MEMORY: StaticCell<[MaybeUninit<u32>; 1024]> = StaticCell::new();
static USB_BUS: StaticCell<UsbBusAllocator<UsbBus<USB2>>> = StaticCell::new();

pub struct Platform {
    tim2: Option<Timer<TIM2>>,
    green_led: Option<EPin<Output<PushPull>>>,
    blue_led: Option<EPin<Output<PushPull>>>,
    usb: Option<&'static UsbBusAllocator<UsbBus<USB2>>>,
}

impl PlatformAbstraction for Platform {
    type OutputPin = EPin<Output<PushPull>>;
    type IMU = DummyImu;
    type USB = UsbBus<USB2>;

    fn primary_led(&mut self) -> Option<Self::OutputPin> {
        self.green_led.take()
    }

    fn secondary_led(&mut self) -> Option<Self::OutputPin> {
        self.blue_led.take()
    }

    fn imu0(&mut self) -> Option<Self::IMU> {
        return Some(DummyImu::default());
    }

    fn imu1(&mut self) -> Option<Self::IMU> {
        return Some(DummyImu::default());
    }

    fn usb0(&mut self) -> Option<&'static UsbBusAllocator<Self::USB>> {
        self.usb.take()
    }
}

impl Platform {
    pub fn system_init(dp: Peripherals) -> Self {
        let pwr = dp.PWR.constrain();
        let pwrcfg = pwr.freeze();

        let rcc = dp.RCC.constrain();
        let mut ccdr = rcc
            .sys_ck(200.MHz())
            .pll1_q_ck(48.MHz())
            .freeze(pwrcfg, &dp.SYSCFG);

        // 48MHz CLOCK
        let _ = ccdr.clocks.hsi48_ck().expect("HSI48 must run");
        ccdr.peripheral.kernel_usb_clk_mux(UsbClkSel::Hsi48);

        let gpioa = dp.GPIOA.split(ccdr.peripheral.GPIOA);
        let gpiod = dp.GPIOD.split(ccdr.peripheral.GPIOD);
        let gpioc = dp.GPIOC.split(ccdr.peripheral.GPIOC);
        let gpioe = dp.GPIOE.split(ccdr.peripheral.GPIOE);

        let green_led = Some(gpioe.pe4.into_push_pull_output().into());
        let blue_led = Some(gpioe.pe3.into_push_pull_output().into());

        let mut tim2 = dp.TIM2.timer(
            MilliSeconds::from_ticks(200).into_rate(),
            ccdr.peripheral.TIM2,
            &ccdr.clocks,
        );
        tim2.listen(Event::TimeOut);

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

        let usb_dm = gpioa.pa11.into_alternate();
        let usb_dp = gpioa.pa12.into_alternate();

        let usb_dev = USB2::new(
            dp.OTG2_HS_GLOBAL,
            dp.OTG2_HS_DEVICE,
            dp.OTG2_HS_PWRCLK,
            usb_dm,
            usb_dp,
            ccdr.peripheral.USB2OTG,
            &ccdr.clocks,
        );

        let ep_mem = EP_MEMORY.init([MaybeUninit::uninit(); 1024]);
        for slot in ep_mem.iter_mut() {
            slot.write(0u32);
        }

        let mem = unsafe { core::mem::transmute::<_, &'static mut [u32; 1024]>(ep_mem) };
        let usb = Some(USB_BUS.init(UsbBus::new(usb_dev, mem)) as &'static _);

        Platform {
            tim2: Some(tim2),
            green_led,
            blue_led,
            usb,
        }
    }

    pub fn timer2(&mut self) -> Option<Timer<TIM2>> {
        self.tim2.take()
    }
}
