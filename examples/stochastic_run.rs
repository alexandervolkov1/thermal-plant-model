use thermal_plant_model::{SimulationConfig, ThermalPlant};

fn main() {
    const STEPS_PER_SECOND: usize = 10;
    const HEATING_SECONDS: usize = 3600;
    const TOTAL_SECONDS: usize = 7200;
    const LOG_EVERY_SECONDS: usize = 10;

    let seed = std::env::args()
        .nth(1)
        .map(|arg| arg.parse::<u32>().expect("Seed must be a u32"))
        .unwrap_or(12345);

    let config = SimulationConfig {
        seed,
        parameter_spread: 0.03,
        heater_power_fluctuation: 0.03,
        sensor_noise: 0.15,
        sensor_drift_amplitude: 0.25,
        ..SimulationConfig::default()
    };

    let mut plant = ThermalPlant::new(config).expect("Invalid simulation configuration");

    plant.set_power_percent(100.0).expect("Invalid power");

    println!(
        "second,power_percent,true_heater_c,measured_heater_c,\
true_sample_c,measured_sample_c,true_room_c,measured_room_c"
    );

    for step in 0..=TOTAL_SECONDS * STEPS_PER_SECOND {
        if step == HEATING_SECONDS * STEPS_PER_SECOND {
            plant.set_power_percent(0.0).expect("Invalid power");
        }

        if step % (LOG_EVERY_SECONDS * STEPS_PER_SECOND) == 0 {
            let actual = plant.temperatures();
            let measured = plant.measured_temperatures();

            println!(
                "{},{:.0},{:.6},{:.6},{:.6},{:.6},{:.6},{:.6}",
                step / STEPS_PER_SECOND,
                plant.power_percent(),
                actual.heater_temp,
                measured.heater_temp,
                actual.sample_temp,
                measured.sample_temp,
                actual.room_temp,
                measured.room_temp,
            );
        }

        if step < TOTAL_SECONDS * STEPS_PER_SECOND {
            plant.step();
        }
    }
}
