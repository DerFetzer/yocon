#![no_main]
#![no_std]
#![feature(type_alias_impl_trait)]

use yocon as _; // global logger + panicking-behavior + memory layout

#[rtic::app(device = stm32f4xx_hal::pac, dispatchers = [TIM2], peripherals = true)]
mod app {
    use core::fmt::Write;
    use ds18b20::{Ds18b20, SensorData};
    use embedded_graphics::{
        draw_target::DrawTarget,
        geometry::{Point, Size},
        mono_font::{ascii::FONT_6X9, MonoTextStyle},
        pixelcolor::{Gray4, GrayColor},
        primitives::Rectangle,
        Drawable,
    };
    use embedded_text::{
        alignment::HorizontalAlignment,
        style::{HeightMode, TextBoxStyleBuilder},
        TextBox,
    };
    use heapless::String;
    use one_wire_bus::OneWire;
    use pid::{ControlOutput, Pid};
    use rtic_monotonics::{systick::Systick, Monotonic};
    use ssd1327_i2c::SSD1327I2C;
    use stm32f4xx_hal::{
        gpio::{OpenDrain, Output, Pin},
        i2c::I2c,
        pac::{I2C3, TIM1, TIM2},
        prelude::*,
        timer::{Channel2, Delay, PwmChannel},
    };

    // Shared resources go here
    #[shared]
    struct Shared {
        // TODO: Add resources
    }

    // Local resources go here
    #[local]
    struct Local {
        heater_pwm: PwmChannel<TIM2, 1>,
        temp_sensor: Ds18b20,
        temp_one_wire_bus: OneWire<Pin<'B', 5, Output<OpenDrain>>>,
        temp_delay: Delay<TIM1, 1000000>,
        display: SSD1327I2C<I2c<I2C3>>,
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

        let mut temp_delay = cx.device.TIM1.delay_us(&clocks);

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
        let disp_i2c = I2c::new(cx.device.I2C3, (disp_scl, disp_sda), 400.kHz(), &clocks);
        let mut display = ssd1327_i2c::SSD1327I2C::new(disp_i2c);
        display.init();

        // Heater
        let heater_pwm_channel = Channel2::new(heater);
        let heater_pwm = cx
            .device
            .TIM2
            .pwm_hz(heater_pwm_channel, 1.Hz(), &clocks)
            .split();

        // Temperature sensor
        let mut temp_one_wire_bus = OneWire::new(temp_one_wire_pin).unwrap();
        let search_state = None;
        let (device_address, _state) = temp_one_wire_bus
            .device_search(search_state.as_ref(), false, &mut temp_delay)
            .unwrap()
            .unwrap();
        if device_address.family_code() != ds18b20::FAMILY_CODE {
            defmt::panic!("No valid temperature sensor connected");
        }
        let temp_sensor = Ds18b20::new::<()>(device_address).unwrap();

        controller::spawn().unwrap();

        defmt::info!("Start");

        (
            Shared {
                // Initialization of shared resources go here
            },
            Local {
                heater_pwm,
                temp_sensor,
                temp_one_wire_bus,
                temp_delay,
                display,
            },
        )
    }

    #[derive(Debug)]
    struct DisplayData {
        sensor_data: SensorData,
        control_output: ControlOutput<f32>,
        out_of_range: bool,
    }

    #[task(local = [display], priority = 1)]
    async fn refresh_display(cx: refresh_display::Context, data: DisplayData) {
        let display = cx.local.display;

        let character_style = MonoTextStyle::new(&FONT_6X9, Gray4::WHITE);
        let textbox_style = TextBoxStyleBuilder::new()
            .height_mode(HeightMode::FitToText)
            .alignment(HorizontalAlignment::Justified)
            .paragraph_spacing(3)
            .build();

        let bounds = Rectangle::new(Point::zero(), Size::new(128, 0));

        let mut text = String::<512>::new();
        writeln!(&mut text, "Temp: {}", data.sensor_data.temperature).unwrap();
        writeln!(&mut text, "SetTemp: {}", TEMP_SETPOINT).unwrap();
        writeln!(&mut text, "OOR: {}", data.out_of_range).unwrap();
        writeln!(&mut text, "PWM: {}", data.control_output.output).unwrap();
        writeln!(&mut text, "P: {}", data.control_output.p).unwrap();
        writeln!(&mut text, "I: {}", data.control_output.i).unwrap();
        writeln!(&mut text, "D: {}", data.control_output.d).unwrap();

        let text_box = TextBox::with_textbox_style(&text, bounds, character_style, textbox_style);

        display.clear(Gray4::BLACK).unwrap();
        text_box.draw(display).unwrap();
        display.flush().unwrap();
    }

    const TEMP_SETPOINT: f32 = 45.;
    const TEMP_SETPOINT_TOL: f32 = 15.;

    #[task(local = [heater_pwm, temp_sensor, temp_one_wire_bus, temp_delay], priority = 1)]
    async fn controller(cx: controller::Context) {
        let heater_pwm = cx.local.heater_pwm;
        let temp_sensor = cx.local.temp_sensor;
        let temp_one_wire_bus = cx.local.temp_one_wire_bus;
        let temp_delay = cx.local.temp_delay;

        let max_duty = heater_pwm.get_max_duty();
        heater_pwm.set_duty(0);
        heater_pwm.enable();

        let mut pid = Pid::new(TEMP_SETPOINT, 100.);
        pid.p(20., 100.);
        pid.i(10., 100.);

        let mut last_loop_instant = Systick::now();

        loop {
            temp_sensor
                .start_temp_measurement(temp_one_wire_bus, temp_delay)
                .unwrap();
            Systick::delay_until(last_loop_instant + 1000.millis()).await;
            last_loop_instant = Systick::now();
            let sensor_data = temp_sensor
                .read_data(temp_one_wire_bus, temp_delay)
                .unwrap();
            let control_output = pid.next_control_output(sensor_data.temperature);
            let out_of_range = sensor_data.temperature < TEMP_SETPOINT - TEMP_SETPOINT_TOL
                || sensor_data.temperature > TEMP_SETPOINT + TEMP_SETPOINT_TOL;
            if !out_of_range {
                let duty = (control_output.output.max(0.0) / 100. * max_duty as f32) as u16;
                defmt::info!("duty: {}, max_duty: {}", duty, max_duty);
                heater_pwm.set_duty(duty)
            } else {
                defmt::warn!(
                    "Current temperature is out of range! ({}+-{})",
                    TEMP_SETPOINT,
                    TEMP_SETPOINT_TOL
                );
                heater_pwm.set_duty(0);
            }
            let display_data = DisplayData {
                sensor_data,
                control_output,
                out_of_range,
            };
            defmt::info!("{:?}", defmt::Debug2Format(&display_data));
            refresh_display::spawn(display_data).unwrap();
        }
    }
}
