use rtic::app;

#[app(device = stm32h7xx_hal::pac, peripherals = true)]
mod app {

    use crate::bsp::Platform;
    use oxidrone_core::CdcAcmDevice;
    use oxidrone_core::flight_controller::FlightController;
    use stm32h7xx_hal::stm32::{TIM2, TIM3};
    use stm32h7xx_hal::timer::Timer;

    #[shared]
    struct Shared {/* shared resources */}

    #[local]
    struct Local {
        tim2: Timer<TIM2>,
        tim3: Timer<TIM3>,
        flight_controller: FlightController<Platform>,
        #[cfg(feature = "usb_logger")]
        usb_logger: CdcAcmDevice<{ crate::logger::LOG_BUF_SIZE }, Platform>,
    }

    #[init]
    fn init(cx: init::Context) -> (Shared, Local) {
        let mut platform = Platform::system_init(cx.device);

        #[cfg(feature = "usb_logger")]
        let usb_logger = CdcAcmDevice::new(
            &mut platform,
            "debug logging",
            "oxi1234",
            crate::logger::init_logger(),
        );

        let tim2 = platform.timer2().unwrap();
        let tim3 = platform.timer3().unwrap();
        let flight_controller = FlightController::new(platform);

        (
            Shared {},
            Local {
                tim2,
                tim3,
                flight_controller,
                #[cfg(feature = "usb_logger")]
                usb_logger,
            },
        )
    }

    //TODO:
    //* run on OTG interrupt instead of timer.
    //* this feature gate is not compiling when feature !usb_logger.
    //it seems that its only disabling the task proc-macro, leaving us with an function signature
    //that does not exist.
    #[cfg(feature = "usb_logger")]
    #[task(binds = TIM2, local = [tim2, usb_logger])]
    fn timer2_tick(ctx: timer2_tick::Context) {
        ctx.local.tim2.clear_irq();
        ctx.local.usb_logger.poll();
    }

    #[task(binds = TIM3, local = [tim3, flight_controller])]
    fn timer3_tick(ctx: timer3_tick::Context) {
        ctx.local.tim3.clear_irq();
        ctx.local.flight_controller.update();
    }
}
