use thermal_plant_model::{SimulationConfig, ThermalInput, rk2_step};

fn main() {
    const DT: f32 = 0.1;
    const STEPS_PER_MINUTE: usize = 600;
    const HEATING_MINUTES: usize = 60;
    const TOTAL_MINUTES: usize = 120;

    let config = SimulationConfig::default();
    config.validate().expect("Invalid simulation configuration");

    let params = config.parameters;
    let mut state = config.initial_state;

    let mut input = ThermalInput {
        background_temp: config.background_temp,
        heater_command: 1.0,
    };

    let mut room_at_shutdown = None;

    println!("minute,heater_c,sample_c,room_c,power_percent");

    for step in 0..=TOTAL_MINUTES * STEPS_PER_MINUTE {
        if step == HEATING_MINUTES * STEPS_PER_MINUTE {
            room_at_shutdown = Some(state.room_temp);
            input.heater_command = 0.0;
        }

        if step % (10 * STEPS_PER_MINUTE) == 0 {
            println!(
                "{},{:.3},{:.3},{:.6},{:.0}",
                step / STEPS_PER_MINUTE,
                state.heater_temp,
                state.sample_temp,
                state.room_temp,
                input.heater_command * 100.0
            );
        }

        if step < TOTAL_MINUTES * STEPS_PER_MINUTE {
            state = rk2_step(state, params, input, DT);
        }
    }

    let room_at_shutdown = room_at_shutdown.expect("Missing shutdown state");

    assert!(room_at_shutdown > input.background_temp + 0.5);
    assert!(state.room_temp < room_at_shutdown);
    assert!(state.heater_temp < 25.0);
}
