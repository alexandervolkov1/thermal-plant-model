use thermal_plant_model::{ThermalInput, ThermalParams, ThermalState, rk2_step};

fn main() {
    const DT: f32 = 0.1;
    const STEPS_PER_MINUTE: usize = 600;
    const HEATING_MINUTES: usize = 60;
    const TOTAL_MINUTES: usize = 120;

    let params = ThermalParams {
        heater_capacity: 100.0,
        sample_capacity: 200.0,
        room_capacity: 300_000.0,
        coupling: 2.0,
        heater_loss: 1.0,
        sample_loss: 0.5,
        room_loss: 20.0,
        heater_radiation: 4.0e-10,
        sample_radiation: 2.0e-10,
        max_heater_power: 100.0,
    };

    let mut state = ThermalState {
        heater_temp: 20.0,
        sample_temp: 20.0,
        room_temp: 20.0,
    };

    let mut input = ThermalInput {
        background_temp: 20.0,
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
