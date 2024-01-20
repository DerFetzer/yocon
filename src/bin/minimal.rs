#![no_main]
#![no_std]
#![feature(type_alias_impl_trait)]

use yocon as _; // global logger + panicking-behavior + memory layout

#[rtic::app(device = stm32f4xx_hal::pac, dispatchers = [TIM2], peripherals = true)]
mod app {
    use ds18b20::Ds18b20;
    use embedded_graphics::{
        draw_target::DrawTarget,
        geometry::Point,
        pixelcolor::{Gray4, GrayColor},
        primitives::{Circle, Primitive, PrimitiveStyle},
        Drawable,
    };
    use one_wire_bus::OneWire;
    use rtic_monotonics::systick::Systick;
    use stm32f4xx_hal::{i2c::I2c, prelude::*, timer::Channel2};

    // Shared resources go here
    #[shared]
    struct Shared {
        // TODO: Add resources
    }

    // Local resources go here
    #[local]
    struct Local {
        // TODO: Add resources
    }

    #[init]
    fn init(cx: init::Context) -> (Shared, Local) {
        defmt::info!("init");

        let rcc = cx.device.RCC.constrain();
        let freq = 48.MHz();
        let clocks = rcc.cfgr.sysclk(freq).freeze();

        // Initialize the systick interrupt & obtain the token to prove that we did
        let systick_mono_token = rtic_monotonics::create_systick_token!();
        Systick::start(cx.core.SYST, freq.to_Hz(), systick_mono_token);

        let mut delay = cx.device.TIM1.delay_us(&clocks);

        let gpioa = cx.device.GPIOA.split();
        let gpiob = cx.device.GPIOB.split();
        let _gpioc = cx.device.GPIOC.split();

        // Pins
        let disp_scl = gpioa.pa8.into_open_drain_output();
        let disp_sda = gpiob.pb4.into_open_drain_output();

        let heater = gpiob.pb3.into_push_pull_output();

        let temp_one_wire_pin = gpiob.pb5.into_open_drain_output();

        // Peripherals
        // Display
        let disp_i2c = I2c::new(cx.device.I2C3, (disp_scl, disp_sda), 100.kHz(), &clocks);
        let mut disp = ssd1327_i2c::SSD1327I2C::new(disp_i2c);
        disp.init();
        disp.clear(Gray4::BLACK).unwrap();
        disp.flush().unwrap();

        Circle::new(Point::new(0, 0), 20)
            .into_styled(PrimitiveStyle::with_stroke(Gray4::WHITE, 5))
            .draw(&mut disp)
            .unwrap();
        disp.flush().unwrap();

        // Heater
        let heater_pwm_channel = Channel2::new(heater);
        let mut heater_pwm = cx
            .device
            .TIM2
            .pwm_hz(heater_pwm_channel, 1.Hz(), &clocks)
            .split();
        let max_duty = heater_pwm.get_max_duty();
        heater_pwm.set_duty(max_duty / 4);
        heater_pwm.enable();

        // Temperature sensor
        let mut temp_one_wire_bus = OneWire::new(temp_one_wire_pin).unwrap();
        let search_state = None;
        let (device_address, _state) = temp_one_wire_bus
            .device_search(search_state.as_ref(), false, &mut delay)
            .unwrap()
            .unwrap();
        if device_address.family_code() != ds18b20::FAMILY_CODE {
            defmt::panic!("No valid temperature sensor connected");
        }
        let temp_sensor = Ds18b20::new::<()>(device_address).unwrap();
        defmt::info!("Start");
        temp_sensor
            .start_temp_measurement(&mut temp_one_wire_bus, &mut delay)
            .unwrap();
        delay.delay_ms(1000);
        let sensor_data = temp_sensor
            .read_data(&mut temp_one_wire_bus, &mut delay)
            .unwrap();
        defmt::info!("Temp: {}", sensor_data.temperature);

        defmt::info!("Start");

        task1::spawn().ok();

        (
            Shared {
                // Initialization of shared resources go here
            },
            Local {
                // Initialization of local resources go here
            },
        )
    }

    // Optional idle, can be removed if not needed.
    #[idle]
    fn idle(_: idle::Context) -> ! {
        defmt::info!("idle");

        loop {
            continue;
        }
    }

    // TODO: Add tasks
    #[task(priority = 1)]
    async fn task1(_cx: task1::Context) {
        defmt::info!("Hello from task1!");
    }
}
