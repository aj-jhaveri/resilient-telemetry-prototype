//! Simulated vessel plant: first-order kinematics with no I/O.
//!
//! The parameters are illustrative. They do not model any real vessel.
//! Heading is measured clockwise from north, and positive rudder turns
//! clockwise.

use std::time::Duration;

use crate::telemetry::Telemetry;
use crate::{
    BATTERY_DRAIN_PERCENT_PER_S, BATTERY_MIN_PERCENT, HEADING_MAX_DEG, HEADING_MIN_DEG,
    INITIAL_BATTERY_PERCENT, INITIAL_HEADING_DEG, INITIAL_LATITUDE_DEG, INITIAL_LONGITUDE_DEG,
    INITIAL_SPEED_MPS, LATITUDE_MAX_DEG, LATITUDE_MIN_DEG, LONGITUDE_MIN_DEG, MAX_TURN_RATE_DEG_S,
    METERS_PER_DEG_LAT, PLANT_MAX_SPEED_MPS, RUDDER_MAX, RUDDER_MIN, SPEED_TIME_CONSTANT_S,
    SUPPORTED_VERSION, THROTTLE_MAX, THROTTLE_MIN,
};

const HALF_TURN_DEG: f64 = HEADING_MAX_DEG / 2.0;

/// Rudder and throttle applied to the plant. The only constructor clamps,
/// so every value is finite and inside its bounds.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Actuation {
    rudder: f64,
    throttle: f64,
}

impl Actuation {
    pub const ZERO: Self = Self {
        rudder: 0.0,
        throttle: 0.0,
    };

    /// Clamps finite inputs to their bounds. Non-finite inputs become 0.0.
    pub fn clamped(rudder: f64, throttle: f64) -> Self {
        Self {
            rudder: clamp_finite(rudder, RUDDER_MIN, RUDDER_MAX),
            throttle: clamp_finite(throttle, THROTTLE_MIN, THROTTLE_MAX),
        }
    }

    pub fn rudder(&self) -> f64 {
        self.rudder
    }

    pub fn throttle(&self) -> f64 {
        self.throttle
    }
}

fn clamp_finite(value: f64, min: f64, max: f64) -> f64 {
    if value.is_finite() {
        value.clamp(min, max)
    } else {
        0.0
    }
}

/// Wraps a finite angle into `[0, 360)`.
pub fn wrap_heading_deg(deg: f64) -> f64 {
    let wrapped = deg.rem_euclid(HEADING_MAX_DEG);
    // `rem_euclid` can round up to exactly 360.0 for tiny negative inputs.
    if wrapped >= HEADING_MAX_DEG {
        HEADING_MIN_DEG
    } else {
        wrapped
    }
}

/// Shortest signed turn from `current_deg` to `target_deg`, in `[-180, 180)`.
/// Positive means turn clockwise. Exactly opposite headings give -180.
pub fn heading_error_deg(target_deg: f64, current_deg: f64) -> f64 {
    wrap_heading_deg(target_deg - current_deg + HALF_TURN_DEG) - HALF_TURN_DEG
}

fn wrap_longitude_deg(deg: f64) -> f64 {
    wrap_heading_deg(deg - LONGITUDE_MIN_DEG) + LONGITUDE_MIN_DEG
}

/// State of the simulated vessel plant.
#[derive(Debug, Clone, PartialEq)]
pub struct PlantState {
    pub latitude_deg: f64,
    pub longitude_deg: f64,
    pub heading_deg: f64,
    pub speed_mps: f64,
    pub battery_percent: f32,
}

impl PlantState {
    pub fn initial() -> Self {
        Self {
            latitude_deg: INITIAL_LATITUDE_DEG,
            longitude_deg: INITIAL_LONGITUDE_DEG,
            heading_deg: INITIAL_HEADING_DEG,
            speed_mps: INITIAL_SPEED_MPS,
            battery_percent: INITIAL_BATTERY_PERCENT,
        }
    }

    /// Advances the plant by `dt` with `actuation` held constant.
    pub fn step(&mut self, actuation: Actuation, dt: Duration) {
        let dt_s = dt.as_secs_f64();

        // Heading: turn rate proportional to rudder, wrapped to [0, 360).
        self.heading_deg =
            wrap_heading_deg(self.heading_deg + actuation.rudder() * MAX_TURN_RATE_DEG_S * dt_s);

        // Speed: exact solution of v' = (v_target - v) / tau over this step,
        // so it never overshoots and does not depend on how dt is split.
        let target_speed = actuation.throttle() * PLANT_MAX_SPEED_MPS;
        let alpha = 1.0 - (-dt_s / SPEED_TIME_CONSTANT_S).exp();
        self.speed_mps += alpha * (target_speed - self.speed_mps);

        // Position: flat-earth dead reckoning with the updated heading and
        // speed. Not meaningful near the poles.
        let heading_rad = self.heading_deg.to_radians();
        let north_m = self.speed_mps * heading_rad.cos() * dt_s;
        let east_m = self.speed_mps * heading_rad.sin() * dt_s;
        self.latitude_deg = (self.latitude_deg + north_m / METERS_PER_DEG_LAT)
            .clamp(LATITUDE_MIN_DEG, LATITUDE_MAX_DEG);
        let meters_per_deg_lon = METERS_PER_DEG_LAT * self.latitude_deg.to_radians().cos();
        self.longitude_deg = wrap_longitude_deg(self.longitude_deg + east_m / meters_per_deg_lon);

        // Battery: drains in proportion to throttle, never below empty.
        let drain = BATTERY_DRAIN_PERCENT_PER_S * actuation.throttle() * dt_s;
        self.battery_percent = (self.battery_percent - drain as f32).max(BATTERY_MIN_PERCENT);
    }

    /// Reports the current state as telemetry.
    pub fn to_telemetry(&self, sequence: u64, sent_at_ms: u128) -> Telemetry {
        Telemetry {
            version: SUPPORTED_VERSION,
            sequence,
            sent_at_ms,
            latitude_deg: self.latitude_deg,
            longitude_deg: self.longitude_deg,
            heading_deg: self.heading_deg,
            speed_mps: self.speed_mps,
            battery_percent: self.battery_percent,
        }
    }
}

#[cfg(test)]
#[allow(clippy::unwrap_used, clippy::expect_used)]
mod tests {
    use super::*;
    use crate::SPEED_MAX_MPS;

    const EPS: f64 = 1e-9;
    const ONE_S: Duration = Duration::from_secs(1);
    const TENTH_S: Duration = Duration::from_millis(100);

    fn assert_close(actual: f64, expected: f64) {
        assert!(
            (actual - expected).abs() < EPS,
            "expected {expected}, got {actual}"
        );
    }

    fn cruising(heading_deg: f64, throttle: f64) -> (PlantState, Actuation) {
        let mut plant = PlantState::initial();
        plant.heading_deg = heading_deg;
        plant.speed_mps = throttle * PLANT_MAX_SPEED_MPS;
        (plant, Actuation::clamped(0.0, throttle))
    }

    #[test]
    fn wrap_heading_covers_full_turns_and_negatives() {
        assert_eq!(wrap_heading_deg(HEADING_MAX_DEG), HEADING_MIN_DEG);
        assert_close(wrap_heading_deg(-10.0), 350.0);
        assert_close(wrap_heading_deg(725.0), 5.0);
        for deg in [-720.5, -0.25, 0.0, 359.75, 1e6] {
            let wrapped = wrap_heading_deg(deg);
            assert!(
                (HEADING_MIN_DEG..HEADING_MAX_DEG).contains(&wrapped),
                "{deg}"
            );
        }
    }

    #[test]
    fn wrap_heading_guards_rem_euclid_rounding_to_360() {
        let tiny_negative: f64 = -1e-15;
        assert_eq!(tiny_negative.rem_euclid(HEADING_MAX_DEG), HEADING_MAX_DEG);
        assert_eq!(wrap_heading_deg(tiny_negative), HEADING_MIN_DEG);
    }

    #[test]
    fn heading_error_takes_the_shortest_turn() {
        assert_close(heading_error_deg(10.0, 350.0), 20.0);
        assert_close(heading_error_deg(350.0, 10.0), -20.0);
        assert_close(heading_error_deg(0.0, 0.0), 0.0);
        assert_close(heading_error_deg(HALF_TURN_DEG, 0.0), -HALF_TURN_DEG);
    }

    #[test]
    fn actuation_is_clamped_and_non_finite_becomes_zero() {
        let high = Actuation::clamped(RUDDER_MAX + 1.0, THROTTLE_MAX + 1.0);
        assert_eq!((high.rudder(), high.throttle()), (RUDDER_MAX, THROTTLE_MAX));
        let low = Actuation::clamped(RUDDER_MIN - 1.0, THROTTLE_MIN - 1.0);
        assert_eq!((low.rudder(), low.throttle()), (RUDDER_MIN, THROTTLE_MIN));
        for bad in [f64::NAN, f64::INFINITY, f64::NEG_INFINITY] {
            assert_eq!(Actuation::clamped(bad, bad), Actuation::ZERO);
        }
    }

    #[test]
    fn speed_follows_the_exact_first_order_response() {
        let mut plant = PlantState::initial();
        let full = Actuation::clamped(0.0, THROTTLE_MAX);
        plant.step(full, Duration::from_secs_f64(SPEED_TIME_CONSTANT_S));
        assert_close(
            plant.speed_mps,
            PLANT_MAX_SPEED_MPS * (1.0 - (-1.0_f64).exp()),
        );
    }

    #[test]
    fn speed_update_does_not_depend_on_step_size() {
        let full = Actuation::clamped(0.0, THROTTLE_MAX);
        let mut one_step = PlantState::initial();
        one_step.step(full, ONE_S);
        let mut ten_steps = PlantState::initial();
        for _ in 0..10 {
            ten_steps.step(full, TENTH_S);
        }
        assert!((one_step.speed_mps - ten_steps.speed_mps).abs() < 1e-12);
    }

    #[test]
    fn zero_actuation_coasts_down_exponentially() {
        let start_mps = 3.0;
        let mut plant = PlantState::initial();
        plant.speed_mps = start_mps;
        let mut previous = plant.speed_mps;
        for step in 1..=100 {
            plant.step(Actuation::ZERO, TENTH_S);
            assert!(plant.speed_mps <= previous && plant.speed_mps >= 0.0);
            previous = plant.speed_mps;
            let elapsed_s = TENTH_S.as_secs_f64() * f64::from(step);
            assert_close(
                plant.speed_mps,
                start_mps * (-elapsed_s / SPEED_TIME_CONSTANT_S).exp(),
            );
        }
    }

    #[test]
    fn rudder_turns_at_the_configured_rate_and_wraps() {
        let mut plant = PlantState::initial();
        plant.step(Actuation::clamped(RUDDER_MAX, 0.0), ONE_S);
        assert_close(plant.heading_deg, INITIAL_HEADING_DEG + MAX_TURN_RATE_DEG_S);

        plant.heading_deg = HEADING_MAX_DEG - MAX_TURN_RATE_DEG_S / 2.0;
        plant.step(Actuation::clamped(RUDDER_MAX, 0.0), ONE_S);
        assert_close(plant.heading_deg, MAX_TURN_RATE_DEG_S / 2.0);

        let before = plant.heading_deg;
        plant.step(Actuation::ZERO, ONE_S);
        assert_eq!(plant.heading_deg, before);
    }

    #[test]
    fn position_moves_along_the_heading() {
        let (mut north, throttle) = cruising(0.0, 0.5);
        let start = north.clone();
        north.step(throttle, ONE_S);
        assert_close(
            north.latitude_deg,
            start.latitude_deg + start.speed_mps / METERS_PER_DEG_LAT,
        );
        assert_close(north.longitude_deg, start.longitude_deg);

        let (mut east, throttle) = cruising(90.0, 0.5);
        east.step(throttle, ONE_S);
        assert_close(east.latitude_deg, start.latitude_deg);
        let meters_per_deg_lon = METERS_PER_DEG_LAT * start.latitude_deg.to_radians().cos();
        assert_close(
            east.longitude_deg,
            start.longitude_deg + start.speed_mps / meters_per_deg_lon,
        );
    }

    #[test]
    fn battery_drains_with_throttle_and_never_goes_negative() {
        let mut plant = PlantState::initial();
        plant.step(Actuation::ZERO, ONE_S);
        assert_eq!(plant.battery_percent, INITIAL_BATTERY_PERCENT);

        plant.step(Actuation::clamped(0.0, THROTTLE_MAX), ONE_S);
        let expected = INITIAL_BATTERY_PERCENT - BATTERY_DRAIN_PERCENT_PER_S as f32;
        assert!((plant.battery_percent - expected).abs() < 1e-5);

        plant.battery_percent = 0.01;
        plant.step(Actuation::clamped(0.0, THROTTLE_MAX), ONE_S * 100);
        assert_eq!(plant.battery_percent, BATTERY_MIN_PERCENT);
    }

    #[test]
    fn plant_speed_limit_fits_inside_telemetry_validation() {
        const { assert!(PLANT_MAX_SPEED_MPS <= SPEED_MAX_MPS) };
    }

    #[test]
    fn plant_always_reports_valid_telemetry() {
        let rudders = [RUDDER_MIN, -0.3, 0.0, 0.7, RUDDER_MAX];
        let throttles = [THROTTLE_MIN, 0.5, THROTTLE_MAX];
        let mut plant = PlantState::initial();
        for step in 0..10_000_u64 {
            let rudder = rudders[(step / 7) as usize % rudders.len()];
            let throttle = throttles[(step / 13) as usize % throttles.len()];
            plant.step(Actuation::clamped(rudder, throttle), TENTH_S);
            assert!(plant.speed_mps <= PLANT_MAX_SPEED_MPS);
            let telemetry = plant.to_telemetry(step + 1, 0);
            assert_eq!(telemetry.validate(), Ok(()), "step {step}: {plant:?}");
        }
    }
}
