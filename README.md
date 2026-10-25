# Thermal plant model

Standalone `no_std` Rust library for a three-node thermal plant:
heater, sample and room. The library has no UART or hardware interface.

## Model

Each node has a heat capacity and a temperature in degrees Celsius.
Heater/sample coupling transfers heat between those nodes; linear loss
terms exchange heat with the room, which exchanges heat with a background.
Heater and sample also have radiation terms based on absolute temperature.
Maximum heater power converts the commanded fraction into heat input.

`ThermalPlant::step()` advances simulated time by 0.1 seconds using RK2.
The caller schedules steps; the library does not read a wall clock.
Heater power commands accept finite values from 0 through 100 percent.

Seeded pseudorandom processes provide optional parameter spread, smoothed
heater power fluctuation, sensor noise and slowly varying sensor drift.
Measured temperatures are sampled every ten model steps and held between
samples. Zero variation/noise/drift settings disable the corresponding effect.

## API

- `SimulationConfig`: nominal parameters, initial temperatures, background,
  seed and stochastic settings; `Default` supplies a usable configuration.
- `ThermalPlant`: simulation state and seeded processes.
- `ThermalState`: heater, sample and room temperatures.
- `ThermalPlant::new(config)`: validates and constructs a plant at zero power.
- `step()`: advances the model by one fixed step.
- `set_power_percent(value)`: validates and changes the heater command.
- `temperatures()`: returns the current physical state.
- `measured_temperatures()`: returns the latest sampled sensor values.

`SimulationConfig::validate()` checks finite values, positive capacities and
maximum power, nonnegative transfer coefficients, temperatures above absolute
zero, and supported variation/noise/drift ranges. Construction also validates
the realized parameters after applying the configured spread.

## Tests

With a Rust toolchain supporting edition 2024, run:

```sh
cargo test --locked
cargo fmt --all -- --check
```
