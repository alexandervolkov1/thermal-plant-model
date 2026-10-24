use thermal_plant_model::{SimulationConfig, ThermalPlant};

fn main() {
    const STEPS_PER_MINUTE: usize = 600;
    const HEATING_MINUTES: usize = 60;
    const TOTAL_MINUTES: usize = 120;

    let config = SimulationConfig::default();

    let mut plant = ThermalPlant::new(config).expect("Invalid simulation configuration");

    plant.set_power_percent(100.0).expect("Invalid power");

    let mut room_at_shutdown = None;

    println!("minute,heater_c,sample_c,room_c,power_percent");

    for step in 0..=TOTAL_MINUTES * STEPS_PER_MINUTE {
        if step == HEATING_MINUTES * STEPS_PER_MINUTE {
            room_at_shutdown = Some(plant.temperatures().room_temp);
            plant.set_power_percent(0.0).expect("Invalid power");
        }

        if step % (10 * STEPS_PER_MINUTE) == 0 {
            let state = plant.temperatures();
            println!(
                "{},{:.3},{:.3},{:.6},{:.0}",
                step / STEPS_PER_MINUTE,
                state.heater_temp,
                state.sample_temp,
                state.room_temp,
                plant.power_percent()
            );
        }

        if step < TOTAL_MINUTES * STEPS_PER_MINUTE {
            plant.step();
        }
    }

    let room_at_shutdown = room_at_shutdown.expect("Missing shutdown state");

    let state = plant.temperatures();
    assert!(room_at_shutdown > config.background_temp + 0.5);
    assert!(state.room_temp < room_at_shutdown);
    assert!(state.heater_temp < 25.0);
}
