#![no_std]

#[derive(Clone, Copy)]
pub struct ThermalState {
    pub heater_temp: f32,
    pub sample_temp: f32,
    pub room_temp: f32,
}

#[derive(Clone, Copy)]
pub struct ThermalParams {
    pub heater_capacity: f32,
    pub sample_capacity: f32,
    pub room_capacity: f32,

    pub coupling: f32,

    pub heater_loss: f32,
    pub sample_loss: f32,
    pub room_loss: f32,

    pub heater_radiation: f32,
    pub sample_radiation: f32,

    pub max_heater_power: f32,
}

#[derive(Clone, Copy)]
pub struct ThermalInput {
    pub background_temp: f32,
    pub heater_command: f32,
}

#[derive(Clone, Copy)]
pub struct SimulationConfig {
    pub parameters: ThermalParams,
    pub initial_state: ThermalState,
    pub background_temp: f32,
}

impl Default for SimulationConfig {
    fn default() -> Self {
        Self {
            parameters: ThermalParams {
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
            },
            initial_state: ThermalState {
                heater_temp: 20.0,
                sample_temp: 20.0,
                room_temp: 20.0,
            },
            background_temp: 20.0,
        }
    }
}

impl SimulationConfig {
    pub fn validate(&self) -> Result<(), &'static str> {
        let p = &self.parameters;

        let capacities = [p.heater_capacity, p.sample_capacity, p.room_capacity];

        if !capacities.iter().all(|x| x.is_finite() && *x > 0.0) {
            return Err("Heat capacities must be finite and positive");
        }

        let coefficients = [
            p.coupling,
            p.heater_loss,
            p.sample_loss,
            p.room_loss,
            p.heater_radiation,
            p.sample_radiation,
        ];

        if !coefficients.iter().all(|x| x.is_finite() && *x >= 0.0) {
            return Err("Heat transfer coefficients must be finite and nonnegative");
        }

        if !p.max_heater_power.is_finite() || p.max_heater_power <= 0.0 {
            return Err("Maximum heater power must be finite and positive");
        }

        let temperatures = [
            self.initial_state.heater_temp,
            self.initial_state.sample_temp,
            self.initial_state.room_temp,
            self.background_temp,
        ];

        if !temperatures.iter().all(|t| t.is_finite() && *t > -273.15) {
            return Err("Temperatures must be finite and above absolute zero");
        }

        Ok(())
    }
}

pub const MODEL_STEP_SECONDS: f32 = 0.1;

pub struct ThermalPlant {
    parameters: ThermalParams,
    state: ThermalState,
    input: ThermalInput,
}

impl ThermalPlant {
    pub fn new(config: SimulationConfig) -> Result<Self, &'static str> {
        config.validate()?;

        Ok(Self {
            parameters: config.parameters,
            state: config.initial_state,
            input: ThermalInput {
                background_temp: config.background_temp,
                heater_command: 0.0,
            },
        })
    }

    pub fn temperatures(&self) -> ThermalState {
        self.state
    }

    pub fn power_percent(&self) -> f32 {
        self.input.heater_command * 100.0
    }

    pub fn set_power_percent(&mut self, power: f32) -> Result<(), &'static str> {
        if !power.is_finite() || !(0.0..=100.0).contains(&power) {
            return Err("Heater power must be between 0 and 100 percent");
        }

        self.input.heater_command = power / 100.0;
        Ok(())
    }

    pub fn step(&mut self) {
        self.state = rk2_step(self.state, self.parameters, self.input, MODEL_STEP_SECONDS);
    }
}

#[derive(Clone, Copy)]
pub struct ThermalDerivative {
    pub heater_temp_rate: f32,
    pub sample_temp_rate: f32,
    pub room_temp_rate: f32,
}

pub fn derivatives(
    state: ThermalState,
    params: ThermalParams,
    input: ThermalInput,
) -> ThermalDerivative {
    let heater_power = params.max_heater_power * input.heater_command;

    let heater_to_sample = params.coupling * (state.heater_temp - state.sample_temp);

    let heater_to_room = params.heater_loss * (state.heater_temp - state.room_temp);

    let sample_to_room = params.sample_loss * (state.sample_temp - state.room_temp);

    let heater_radiation =
        radiative_loss(state.heater_temp, state.room_temp, params.heater_radiation);

    let sample_radiation =
        radiative_loss(state.sample_temp, state.room_temp, params.sample_radiation);

    let room_to_background = params.room_loss * (state.room_temp - input.background_temp);

    let heater_temp_rate = (heater_power - heater_to_sample - heater_to_room - heater_radiation)
        / params.heater_capacity;

    let sample_temp_rate =
        (heater_to_sample - sample_to_room - sample_radiation) / params.sample_capacity;

    let room_temp_rate = (heater_to_room + sample_to_room + heater_radiation + sample_radiation
        - room_to_background)
        / params.room_capacity;

    ThermalDerivative {
        heater_temp_rate,
        sample_temp_rate,
        room_temp_rate,
    }
}

pub fn euler_step(
    state: ThermalState,
    params: ThermalParams,
    input: ThermalInput,
    dt: f32,
) -> ThermalState {
    let ThermalDerivative {
        heater_temp_rate,
        sample_temp_rate,
        room_temp_rate,
    } = derivatives(state, params, input);

    ThermalState {
        heater_temp: state.heater_temp + heater_temp_rate * dt,
        sample_temp: state.sample_temp + sample_temp_rate * dt,
        room_temp: state.room_temp + room_temp_rate * dt,
    }
}

pub fn rk2_step(
    state: ThermalState,
    params: ThermalParams,
    input: ThermalInput,
    dt: f32,
) -> ThermalState {
    let k1 = derivatives(state, params, input);

    let mid_state = ThermalState {
        heater_temp: state.heater_temp + k1.heater_temp_rate * dt / 2.0,
        sample_temp: state.sample_temp + k1.sample_temp_rate * dt / 2.0,
        room_temp: state.room_temp + k1.room_temp_rate * dt / 2.0,
    };

    let k2 = derivatives(mid_state, params, input);

    ThermalState {
        heater_temp: state.heater_temp + k2.heater_temp_rate * dt,
        sample_temp: state.sample_temp + k2.sample_temp_rate * dt,
        room_temp: state.room_temp + k2.room_temp_rate * dt,
    }
}

fn radiative_loss(temperature: f32, ambient_temperature: f32, coefficient: f32) -> f32 {
    let temp_k = temperature + 273.15;
    let amb_temp_k = ambient_temperature + 273.15;

    let temp_k4 = temp_k * temp_k * temp_k * temp_k;
    let amb_temp_k4 = amb_temp_k * amb_temp_k * amb_temp_k * amb_temp_k;

    coefficient * (temp_k4 - amb_temp_k4)
}

#[cfg(test)]
mod tests {
    use super::*;

    fn parameters() -> ThermalParams {
        SimulationConfig::default().parameters
    }

    #[test]
    fn equilibrium_without_heating() {
        let state = ThermalState {
            heater_temp: 20.0,
            sample_temp: 20.0,
            room_temp: 20.0,
        };

        let input = ThermalInput {
            background_temp: 20.0,
            heater_command: 0.0,
        };

        let next = rk2_step(state, parameters(), input, 0.1);

        assert_eq!(next.heater_temp, 20.0);
        assert_eq!(next.sample_temp, 20.0);
        assert_eq!(next.room_temp, 20.0);
    }

    #[test]
    fn heating_without_losses() {
        let params = ThermalParams {
            coupling: 0.0,
            heater_loss: 0.0,
            sample_loss: 0.0,
            heater_radiation: 0.0,
            sample_radiation: 0.0,
            ..parameters()
        };

        let state = ThermalState {
            heater_temp: 20.0,
            sample_temp: 20.0,
            room_temp: 20.0,
        };

        let input = ThermalInput {
            background_temp: 20.0,
            heater_command: 0.5,
        };

        let next = rk2_step(state, params, input, 0.1);

        assert!((next.heater_temp - 20.05).abs() < 1e-4);
        assert_eq!(next.sample_temp, 20.0);
    }

    #[test]
    fn internal_heat_exchange_conserves_energy() {
        let params = ThermalParams {
            heater_loss: 0.0,
            sample_loss: 0.0,
            heater_radiation: 0.0,
            sample_radiation: 0.0,
            ..parameters()
        };

        let state = ThermalState {
            heater_temp: 40.0,
            sample_temp: 20.0,
            room_temp: 20.0,
        };

        let input = ThermalInput {
            background_temp: 20.0,
            heater_command: 0.0,
        };

        let next = rk2_step(state, params, input, 0.1);

        let energy_before =
            params.heater_capacity * state.heater_temp + params.sample_capacity * state.sample_temp;

        let energy_after =
            params.heater_capacity * next.heater_temp + params.sample_capacity * next.sample_temp;

        assert!(next.heater_temp < state.heater_temp);
        assert!(next.sample_temp > state.sample_temp);
        assert!((energy_after - energy_before).abs() < 0.01);
    }
    #[test]
    fn heater_cools_after_power_off() {
        let state = ThermalState {
            heater_temp: 60.0,
            sample_temp: 30.0,
            room_temp: 20.0,
        };

        let input = ThermalInput {
            background_temp: 20.0,
            heater_command: 0.0,
        };

        let next = rk2_step(state, parameters(), input, 0.1);

        assert!(next.heater_temp < state.heater_temp);
    }

    #[test]
    fn room_warms_from_hot_heater() {
        let state = ThermalState {
            heater_temp: 60.0,
            sample_temp: 20.0,
            room_temp: 20.0,
        };

        let input = ThermalInput {
            background_temp: 20.0,
            heater_command: 0.0,
        };

        let next = rk2_step(state, parameters(), input, 0.1);

        assert!(next.room_temp > state.room_temp);
        assert!(next.heater_temp < state.heater_temp);
    }

    #[test]
    fn room_cools_to_background() {
        let state = ThermalState {
            heater_temp: 22.0,
            sample_temp: 22.0,
            room_temp: 22.0,
        };

        let input = ThermalInput {
            background_temp: 20.0,
            heater_command: 0.0,
        };

        let next = rk2_step(state, parameters(), input, 0.1);

        assert!(next.room_temp < state.room_temp);
    }

    #[test]
    fn closed_three_body_system_conserves_energy() {
        let params = ThermalParams {
            room_loss: 0.0,
            ..parameters()
        };

        let state = ThermalState {
            heater_temp: 100.0,
            sample_temp: 35.0,
            room_temp: 20.0,
        };

        let input = ThermalInput {
            background_temp: 20.0,
            heater_command: 0.0,
        };

        let rates = derivatives(state, params, input);

        let energy_rate = f64::from(params.heater_capacity) * f64::from(rates.heater_temp_rate)
            + f64::from(params.sample_capacity) * f64::from(rates.sample_temp_rate)
            + f64::from(params.room_capacity) * f64::from(rates.room_temp_rate);

        assert!(energy_rate.abs() < 1e-3);
    }

    #[test]
    fn default_configuration_is_valid() {
        let config = SimulationConfig::default();

        assert_eq!(config.validate(), Ok(()));
        assert_eq!(config.initial_state.heater_temp, 20.0);
        assert_eq!(config.initial_state.sample_temp, 20.0);
        assert_eq!(config.initial_state.room_temp, 20.0);
        assert_eq!(config.parameters.max_heater_power, 100.0);
    }

    #[test]
    fn rejects_invalid_configuration() {
        let mut config = SimulationConfig::default();

        config.parameters.room_capacity = 0.0;
        assert!(config.validate().is_err());

        config.parameters.room_capacity = 300_000.0;
        config.parameters.coupling = -1.0;
        assert!(config.validate().is_err());

        config.parameters.coupling = 2.0;
        config.background_temp = f32::NAN;
        assert!(config.validate().is_err());
    }

    #[test]
    fn plant_step_matches_rk2() {
        let config = SimulationConfig::default();

        let mut plant = ThermalPlant::new(config).unwrap();
        plant.set_power_percent(50.0).unwrap();

        let expected = rk2_step(
            config.initial_state,
            config.parameters,
            ThermalInput {
                background_temp: config.background_temp,
                heater_command: 0.5,
            },
            MODEL_STEP_SECONDS,
        );

        plant.step();

        let actual = plant.temperatures();

        assert_eq!(actual.heater_temp, expected.heater_temp);
        assert_eq!(actual.sample_temp, expected.sample_temp);
        assert_eq!(actual.room_temp, expected.room_temp);
    }

    #[test]
    fn rejects_invalid_power() {
        let mut plant = ThermalPlant::new(SimulationConfig::default()).unwrap();

        plant.set_power_percent(50.0).unwrap();

        assert_eq!(plant.power_percent(), 50.0);

        for invalid in [-1.0, 101.0, f32::NAN, f32::INFINITY] {
            assert!(plant.set_power_percent(invalid).is_err());
            assert_eq!(plant.power_percent(), 50.0);
        }
    }

    #[test]
    fn plant_rejects_invalid_configuration() {
        let mut config = SimulationConfig::default();

        config.parameters.room_capacity = -100.0;

        assert!(ThermalPlant::new(config).is_err());
    }
}
