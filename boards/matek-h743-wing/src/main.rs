#![no_std]
#![no_main]

use panic_probe as _;
use rtic::app;

#[app(device = stm32h7xx_hal::pac, dispatchers = [EXTI3])]
mod app {
    use oxidrone_core::flight_controller::FlightController;
    use stm32h7xx_hal::{pac, prelude::*, timer};

    #[shared]
    struct Shared {/* shared resources */}

    #[local]
    struct Local {
        flight_controller: FlightController,
        tim2: timer::Timer<pac::TIM2>,
    }

    #[init]
    fn init(cx: init::Context) -> (Shared, Local) {
        let pwr = cx.device.PWR.constrain();
        let pwrcfg = pwr.freeze();

        let rcc = cx.device.RCC.constrain();
        let ccdr = rcc
            .sys_ck(200.MHz())
            //.hclk(200.MHz())
            .freeze(pwrcfg, &cx.device.SYSCFG);

        let mut tim2 = cx
            .device
            .TIM2
            .tick_timer(400.Hz(), ccdr.peripheral.TIM2, &ccdr.clocks);
        tim2.listen(timer::Event::TimeOut);

        let flight_controller = FlightController::new();
        (
            Shared {},
            Local {
                tim2,
                flight_controller,
            },
        )
    }

    #[task(binds = TIM2, local = [tim2, flight_controller], priority = 2)]
    fn flight_controller(cx: flight_controller::Context) {
        cx.local.flight_controller.update();
    }

    #[task]
    async fn app_main(_cx: app_main::Context) -> ! {
        oxidrone_core::oxidrone_app_main().await
    }
}
