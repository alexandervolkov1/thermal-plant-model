#![no_std]

mod random;

use random::XorShift32;

pub const MODEL_STEP_SECONDS: f32 = 0.1;
const POWER_TARGET_INTERVAL_STEPS: u16 = 100;
const POWER_SMOOTHING: f32 = 0.01;
const SENSOR_SAMPLE_INTERVAL_STEPS: u8 = 10;
const DRIFT_TARGET_INTERVAL_SAMPLES: u16 = 300;
const DRIFT_SMOOTHING: f32 = 0.002;

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
    pub seed: u32,
    pub parameter_spread: f32,
    pub heater_power_fluctuation: f32,
    pub sensor_noise: f32,
    pub sensor_drift_amplitude: f32,
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
            seed: 1,
            parameter_spread: 0.0,
            heater_power_fluctuation: 0.0,
            sensor_noise: 0.0,
            sensor_drift_amplitude: 0.0,
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

        if !(0.0..=0.10).contains(&self.parameter_spread) {
            return Err("Parameter spread must be between 0 and 0.10");
        }

        if !(0.0..=0.10).contains(&self.heater_power_fluctuation) {
            return Err("Heater power fluctuation must be between 0 and 0.10");
        }

        if !(0.0..=2.0).contains(&self.sensor_noise) {
            return Err("Sensor noise must be between 0 and 2 degrees Celsius");
        }

        if !(0.0..=1.0).contains(&self.sensor_drift_amplitude) {
            return Err("Sensor drift amplitude must be between 0 and 1 degree Celsius");
        }

        Ok(())
    }
}

fn vary_parameters(nominal: ThermalParams, seed: u32, spread: f32) -> ThermalParams {
    if spread == 0.0 {
        return nominal;
    }

    let mut rng = XorShift32::new(seed);

    let mut vary = |value: f32| value * (1.0 + spread * rng.next_signed_unit());

    ThermalParams {
        heater_capacity: vary(nominal.heater_capacity),
        sample_capacity: vary(nominal.sample_capacity),
        room_capacity: vary(nominal.room_capacity),

        coupling: vary(nominal.coupling),

        heater_loss: vary(nominal.heater_loss),
        sample_loss: vary(nominal.sample_loss),
        room_loss: vary(nominal.room_loss),

        heater_radiation: vary(nominal.heater_radiation),
        sample_radiation: vary(nominal.sample_radiation),

        max_heater_power: vary(nominal.max_heater_power),
    }
}

fn sample_temperatures(true_state: ThermalState, noise: f32, rng: &mut XorShift32) -> ThermalState {
    if noise == 0.0 {
        return true_state;
    }

    ThermalState {
        heater_temp: true_state.heater_temp + noise * rng.next_signed_unit(),
        sample_temp: true_state.sample_temp + noise * rng.next_signed_unit(),
        room_temp: true_state.room_temp + noise * rng.next_signed_unit(),
    }
}

pub struct ThermalPlant {
    parameters: ThermalParams,
    state: ThermalState,
    input: ThermalInput,
    heater_power_fluctuation: f32,
    power_rng: XorShift32,
    power_offset: f32,
    power_target: f32,
    steps_until_power_target: u16,
    sensor_noise: f32,
    sensor_rng: XorShift32,
    measured_state: ThermalState,
    steps_until_sensor_sample: u8,
    sensor_drift_amplitude: f32,
    drift_rng: XorShift32,
    drift_offset: ThermalState,
    drift_target: ThermalState,
    samples_until_drift_target: u16,
}

impl ThermalPlant {
    pub fn new(config: SimulationConfig) -> Result<Self, &'static str> {
        config.validate()?;

        let parameters = vary_parameters(config.parameters, config.seed, config.parameter_spread);

        SimulationConfig {
            parameters,
            ..config
        }
        .validate()?;

        let mut sensor_rng = XorShift32::new(config.seed ^ 0xC2B2_AE35);

        let measured_state =
            sample_temperatures(config.initial_state, config.sensor_noise, &mut sensor_rng);

        let zero_drift = ThermalState {
            heater_temp: 0.0,
            sample_temp: 0.0,
            room_temp: 0.0,
        };

        Ok(Self {
            parameters,
            state: config.initial_state,
            input: ThermalInput {
                background_temp: config.background_temp,
                heater_command: 0.0,
            },
            heater_power_fluctuation: config.heater_power_fluctuation,
            power_rng: XorShift32::new(config.seed ^ 0x9E37_79B9),
            power_offset: 0.0,
            power_target: 0.0,
            steps_until_power_target: 0,
            sensor_noise: config.sensor_noise,
            sensor_rng,
            measured_state,
            steps_until_sensor_sample: SENSOR_SAMPLE_INTERVAL_STEPS,
            sensor_drift_amplitude: config.sensor_drift_amplitude,
            drift_rng: XorShift32::new(config.seed ^ 0x85EB_CA6B),
            drift_offset: zero_drift,
            drift_target: zero_drift,
            samples_until_drift_target: 0,
        })
    }

    pub fn temperatures(&self) -> ThermalState {
        self.state
    }

    pub fn measured_temperatures(&self) -> ThermalState {
        self.measured_state
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

    fn update_sensor_drift(&mut self) {
        if self.sensor_drift_amplitude == 0.0 {
            return;
        }

        if self.samples_until_drift_target == 0 {
            let amplitude = self.sensor_drift_amplitude;

            self.drift_target = ThermalState {
                heater_temp: amplitude * self.drift_rng.next_signed_unit(),
                sample_temp: amplitude * self.drift_rng.next_signed_unit(),
                room_temp: amplitude * self.drift_rng.next_signed_unit(),
            };

            self.samples_until_drift_target = DRIFT_TARGET_INTERVAL_SAMPLES;
        }

        self.samples_until_drift_target -= 1;

        self.drift_offset.heater_temp +=
            DRIFT_SMOOTHING * (self.drift_target.heater_temp - self.drift_offset.heater_temp);

        self.drift_offset.sample_temp +=
            DRIFT_SMOOTHING * (self.drift_target.sample_temp - self.drift_offset.sample_temp);

        self.drift_offset.room_temp +=
            DRIFT_SMOOTHING * (self.drift_target.room_temp - self.drift_offset.room_temp);
    }

    pub fn step(&mut self) {
        if self.heater_power_fluctuation > 0.0 {
            if self.steps_until_power_target == 0 {
                self.power_target =
                    self.heater_power_fluctuation * self.power_rng.next_signed_unit();

                self.steps_until_power_target = POWER_TARGET_INTERVAL_STEPS;
            }

            self.steps_until_power_target -= 1;

            self.power_offset += POWER_SMOOTHING * (self.power_target - self.power_offset);
        }

        let effective_parameters = ThermalParams {
            max_heater_power: self.parameters.max_heater_power * (1.0 + self.power_offset),
            ..self.parameters
        };

        self.state = rk2_step(
            self.state,
            effective_parameters,
            self.input,
            MODEL_STEP_SECONDS,
        );

        self.steps_until_sensor_sample -= 1;

        if self.steps_until_sensor_sample == 0 {
            self.update_sensor_drift();

            let mut measured =
                sample_temperatures(self.state, self.sensor_noise, &mut self.sensor_rng);

            measured.heater_temp += self.drift_offset.heater_temp;
            measured.sample_temp += self.drift_offset.sample_temp;
            measured.room_temp += self.drift_offset.room_temp;

            self.measured_state = measured;

            self.steps_until_sensor_sample = SENSOR_SAMPLE_INTERVAL_STEPS;
        }
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

    #[test]
    fn parameter_variation_is_reproducible() {
        let config = SimulationConfig {
            seed: 12345,
            parameter_spread: 0.03,
            ..SimulationConfig::default()
        };

        let mut first = ThermalPlant::new(config).unwrap();
        let mut second = ThermalPlant::new(config).unwrap();

        first.set_power_percent(75.0).unwrap();
        second.set_power_percent(75.0).unwrap();

        for _ in 0..1000 {
            first.step();
            second.step();
        }

        let a = first.temperatures();
        let b = second.temperatures();

        assert_eq!(a.heater_temp.to_bits(), b.heater_temp.to_bits());
        assert_eq!(a.sample_temp.to_bits(), b.sample_temp.to_bits());
        assert_eq!(a.room_temp.to_bits(), b.room_temp.to_bits());
    }

    #[test]
    fn different_seeds_change_parameters() {
        let mut config = SimulationConfig {
            seed: 12345,
            parameter_spread: 0.03,
            ..SimulationConfig::default()
        };

        let first = ThermalPlant::new(config).unwrap();

        config.seed = 67890;

        let second = ThermalPlant::new(config).unwrap();

        assert_ne!(
            first.parameters.heater_capacity.to_bits(),
            second.parameters.heater_capacity.to_bits()
        );
    }

    #[test]
    fn zero_spread_preserves_nominal_parameters() {
        let config = SimulationConfig {
            seed: 98765,
            parameter_spread: 0.0,
            ..SimulationConfig::default()
        };

        let plant = ThermalPlant::new(config).unwrap();

        assert_eq!(
            plant.parameters.heater_capacity,
            config.parameters.heater_capacity
        );

        assert_eq!(
            plant.parameters.sample_capacity,
            config.parameters.sample_capacity
        );

        assert_eq!(
            plant.parameters.room_capacity,
            config.parameters.room_capacity
        );
    }

    #[test]
    fn rejects_invalid_power_fluctuation() {
        for invalid in [-0.01, 0.11, f32::NAN] {
            let config = SimulationConfig {
                heater_power_fluctuation: invalid,
                ..SimulationConfig::default()
            };

            assert!(config.validate().is_err());
        }
    }

    #[test]
    fn power_fluctuation_is_reproducible() {
        let config = SimulationConfig {
            seed: 12345,
            heater_power_fluctuation: 0.03,
            ..SimulationConfig::default()
        };

        let mut first = ThermalPlant::new(config).unwrap();
        let mut second = ThermalPlant::new(config).unwrap();

        first.set_power_percent(75.0).unwrap();
        second.set_power_percent(75.0).unwrap();

        for _ in 0..600 {
            let _ = first.temperatures();
            let _ = first.temperatures();

            first.step();
            second.step();
        }

        let a = first.temperatures();
        let b = second.temperatures();

        assert_eq!(a.heater_temp.to_bits(), b.heater_temp.to_bits());
        assert_eq!(a.sample_temp.to_bits(), b.sample_temp.to_bits());
        assert_eq!(a.room_temp.to_bits(), b.room_temp.to_bits());

        assert_eq!(first.power_percent(), 75.0);
    }

    #[test]
    fn different_seeds_change_heating() {
        let config = SimulationConfig {
            seed: 12345,
            heater_power_fluctuation: 0.03,
            ..SimulationConfig::default()
        };

        let mut first = ThermalPlant::new(config).unwrap();

        let mut second = ThermalPlant::new(SimulationConfig {
            seed: 67890,
            ..config
        })
        .unwrap();

        first.set_power_percent(100.0).unwrap();
        second.set_power_percent(100.0).unwrap();

        for _ in 0..600 {
            first.step();
            second.step();
        }

        assert_ne!(
            first.temperatures().heater_temp.to_bits(),
            second.temperatures().heater_temp.to_bits()
        );
    }

    #[test]
    fn rejects_invalid_sensor_noise() {
        for invalid in [-0.1, 2.1, f32::NAN, f32::INFINITY] {
            let config = SimulationConfig {
                sensor_noise: invalid,
                ..SimulationConfig::default()
            };

            assert!(config.validate().is_err());
        }
    }

    #[test]
    fn sensor_samples_once_per_second() {
        let mut plant = ThermalPlant::new(SimulationConfig::default()).unwrap();

        plant.set_power_percent(100.0).unwrap();

        let initial = plant.measured_temperatures().heater_temp;

        for _ in 0..9 {
            plant.step();

            assert_eq!(plant.measured_temperatures().heater_temp, initial);
        }

        plant.step();

        assert!(plant.measured_temperatures().heater_temp > initial);

        assert_eq!(
            plant.measured_temperatures().heater_temp,
            plant.temperatures().heater_temp
        );
    }

    #[test]
    fn sensor_read_count_does_not_change_results() {
        let config = SimulationConfig {
            seed: 12345,
            parameter_spread: 0.03,
            heater_power_fluctuation: 0.03,
            sensor_noise: 0.2,
            ..SimulationConfig::default()
        };

        let mut first = ThermalPlant::new(config).unwrap();
        let mut second = ThermalPlant::new(config).unwrap();

        first.set_power_percent(75.0).unwrap();
        second.set_power_percent(75.0).unwrap();

        for _ in 0..600 {
            let _ = first.measured_temperatures();
            let _ = first.measured_temperatures();
            let _ = first.measured_temperatures();

            first.step();
            second.step();
        }

        let a = first.measured_temperatures();
        let b = second.measured_temperatures();

        assert_eq!(a.heater_temp.to_bits(), b.heater_temp.to_bits());
        assert_eq!(a.sample_temp.to_bits(), b.sample_temp.to_bits());
        assert_eq!(a.room_temp.to_bits(), b.room_temp.to_bits());
    }

    #[test]
    fn sensor_noise_does_not_change_physics() {
        let config = SimulationConfig {
            seed: 98765,
            parameter_spread: 0.03,
            heater_power_fluctuation: 0.03,
            sensor_noise: 0.2,
            ..SimulationConfig::default()
        };

        let mut noisy = ThermalPlant::new(config).unwrap();

        let mut noiseless = ThermalPlant::new(SimulationConfig {
            sensor_noise: 0.0,
            ..config
        })
        .unwrap();

        noisy.set_power_percent(80.0).unwrap();
        noiseless.set_power_percent(80.0).unwrap();

        for _ in 0..600 {
            noisy.step();
            noiseless.step();
        }

        let a = noisy.temperatures();
        let b = noiseless.temperatures();

        assert_eq!(a.heater_temp.to_bits(), b.heater_temp.to_bits());
        assert_eq!(a.sample_temp.to_bits(), b.sample_temp.to_bits());
        assert_eq!(a.room_temp.to_bits(), b.room_temp.to_bits());
    }

    #[test]
    fn rejects_invalid_sensor_drift() {
        for invalid in [-0.1, 1.1, f32::NAN, f32::INFINITY] {
            let config = SimulationConfig {
                sensor_drift_amplitude: invalid,
                ..SimulationConfig::default()
            };

            assert!(config.validate().is_err());
        }
    }

    #[test]
    fn sensor_drift_is_reproducible() {
        let config = SimulationConfig {
            seed: 12345,
            sensor_noise: 0.2,
            sensor_drift_amplitude: 0.3,
            ..SimulationConfig::default()
        };

        let mut first = ThermalPlant::new(config).unwrap();
        let mut second = ThermalPlant::new(config).unwrap();

        for _ in 0..6000 {
            let _ = first.measured_temperatures();
            let _ = first.measured_temperatures();

            first.step();
            second.step();
        }

        let a = first.measured_temperatures();
        let b = second.measured_temperatures();

        assert_eq!(a.heater_temp.to_bits(), b.heater_temp.to_bits());
        assert_eq!(a.sample_temp.to_bits(), b.sample_temp.to_bits());
        assert_eq!(a.room_temp.to_bits(), b.room_temp.to_bits());
    }

    #[test]
    fn sensor_drift_does_not_change_physics() {
        let config = SimulationConfig {
            seed: 12345,
            sensor_noise: 0.0,
            sensor_drift_amplitude: 0.4,
            ..SimulationConfig::default()
        };

        let mut drifting = ThermalPlant::new(config).unwrap();

        let mut ideal = ThermalPlant::new(SimulationConfig {
            sensor_drift_amplitude: 0.0,
            ..config
        })
        .unwrap();

        drifting.set_power_percent(75.0).unwrap();
        ideal.set_power_percent(75.0).unwrap();

        for _ in 0..6000 {
            drifting.step();
            ideal.step();
        }

        let actual = drifting.temperatures();
        let ideal_actual = ideal.temperatures();
        let measured = drifting.measured_temperatures();

        assert_eq!(
            actual.heater_temp.to_bits(),
            ideal_actual.heater_temp.to_bits()
        );
        assert_eq!(
            actual.sample_temp.to_bits(),
            ideal_actual.sample_temp.to_bits()
        );
        assert_eq!(actual.room_temp.to_bits(), ideal_actual.room_temp.to_bits());

        let offsets = [
            measured.heater_temp - actual.heater_temp,
            measured.sample_temp - actual.sample_temp,
            measured.room_temp - actual.room_temp,
        ];

        assert!(offsets.iter().all(|offset| offset.abs() <= 0.4001));
        assert!(offsets.iter().any(|offset| offset.abs() > 0.001));
    }
}
