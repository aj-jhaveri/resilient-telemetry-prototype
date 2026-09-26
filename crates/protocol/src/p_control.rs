//! Proportional control law with speed feedforward. Pure: no state, no time.
//!
//! The gains are illustrative. There is no integral or derivative term.

use thiserror::Error;

use crate::plant::{heading_error_deg, Actuation};
use crate::telemetry::Telemetry;
use crate::{
    HEADING_KP_PER_DEG, HEADING_MAX_DEG, HEADING_MIN_DEG, PLANT_MAX_SPEED_MPS, SPEED_KP_PER_MPS,
    SPEED_MIN_MPS,
};

/// Heading and speed the controller steers toward.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Setpoint {
    heading_deg: f64,
    speed_mps: f64,
}

/// Why a setpoint was refused. `NaN` fails both range checks.
#[derive(Debug, Clone, PartialEq, Error)]
pub enum SetpointError {
    #[error("target heading {0} must be in [0, 360)")]
    Heading(f64),
    #[error("target speed {0} must be between 0 and the plant maximum speed")]
    Speed(f64),
}

impl Setpoint {
    pub fn new(heading_deg: f64, speed_mps: f64) -> Result<Self, SetpointError> {
        if !(HEADING_MIN_DEG..HEADING_MAX_DEG).contains(&heading_deg) {
            return Err(SetpointError::Heading(heading_deg));
        }
        if !(SPEED_MIN_MPS..=PLANT_MAX_SPEED_MPS).contains(&speed_mps) {
            return Err(SetpointError::Speed(speed_mps));
        }
        Ok(Self {
            heading_deg,
            speed_mps,
        })
    }

    pub fn heading_deg(&self) -> f64 {
        self.heading_deg
    }

    pub fn speed_mps(&self) -> f64 {
        self.speed_mps
    }
}

/// Computes clamped rudder and throttle from one telemetry sample.
///
/// - Rudder is proportional to the shortest-turn heading error. The plant
///   integrates rudder into heading, so P alone removes the heading error.
/// - Throttle is feedforward (the throttle that holds the target speed) plus
///   a proportional term. Without the feedforward term, P control of a
///   first-order speed response settles below the target.
pub fn p_control(setpoint: Setpoint, telemetry: &Telemetry) -> Actuation {
    let rudder =
        HEADING_KP_PER_DEG * heading_error_deg(setpoint.heading_deg, telemetry.heading_deg);
    let feedforward = setpoint.speed_mps / PLANT_MAX_SPEED_MPS;
    let throttle = feedforward + SPEED_KP_PER_MPS * (setpoint.speed_mps - telemetry.speed_mps);
    Actuation::clamped(rudder, throttle)
}

#[cfg(test)]
#[allow(clippy::unwrap_used, clippy::expect_used)]
mod tests {
    use super::*;
    use crate::plant::PlantState;
    use crate::{
        DEFAULT_TARGET_HEADING_DEG, DEFAULT_TARGET_SPEED_MPS, RUDDER_MAX, RUDDER_MIN, THROTTLE_MAX,
        THROTTLE_MIN,
    };
    use std::time::Duration;

    const DT: Duration = Duration::from_millis(100);

    fn setpoint(heading_deg: f64, speed_mps: f64) -> Setpoint {
        Setpoint::new(heading_deg, speed_mps).expect("valid setpoint")
    }

    fn observed(heading_deg: f64, speed_mps: f64) -> Telemetry {
        let mut plant = PlantState::initial();
        plant.heading_deg = heading_deg;
        plant.speed_mps = speed_mps;
        plant.to_telemetry(1, 0)
    }

    /// Runs plant and controller together without sockets or sleeping.
    /// The command computed from step k's telemetry is applied at step
    /// k + 1, matching the one-step delay of the real loop.
    fn run_closed_loop(
        mut plant: PlantState,
        target: Setpoint,
        steps: u64,
    ) -> (PlantState, Vec<f64>) {
        let mut applied = Actuation::ZERO;
        let mut headings = Vec::new();
        for sequence in 1..=steps {
            plant.step(applied, DT);
            let telemetry = plant.to_telemetry(sequence, 0);
            assert_eq!(telemetry.validate(), Ok(()));
            applied = p_control(target, &telemetry);
            headings.push(plant.heading_deg);
        }
        (plant, headings)
    }

    #[test]
    fn setpoint_rejects_out_of_range_and_non_finite_values() {
        assert!(Setpoint::new(DEFAULT_TARGET_HEADING_DEG, DEFAULT_TARGET_SPEED_MPS).is_ok());
        assert!(Setpoint::new(HEADING_MIN_DEG, PLANT_MAX_SPEED_MPS).is_ok());
        for heading in [f64::NAN, HEADING_MAX_DEG, HEADING_MIN_DEG - 1.0] {
            assert!(matches!(
                Setpoint::new(heading, DEFAULT_TARGET_SPEED_MPS),
                Err(SetpointError::Heading(_))
            ));
        }
        for speed in [f64::NAN, SPEED_MIN_MPS - 0.1, PLANT_MAX_SPEED_MPS + 0.1] {
            assert!(matches!(
                Setpoint::new(DEFAULT_TARGET_HEADING_DEG, speed),
                Err(SetpointError::Speed(_))
            ));
        }
    }

    #[test]
    fn zero_error_gives_zero_rudder_and_feedforward_throttle() {
        let target = setpoint(DEFAULT_TARGET_HEADING_DEG, DEFAULT_TARGET_SPEED_MPS);
        let actuation = p_control(
            target,
            &observed(DEFAULT_TARGET_HEADING_DEG, DEFAULT_TARGET_SPEED_MPS),
        );
        assert_eq!(actuation.rudder(), 0.0);
        assert_eq!(
            actuation.throttle(),
            DEFAULT_TARGET_SPEED_MPS / PLANT_MAX_SPEED_MPS
        );
    }

    #[test]
    fn large_errors_saturate_at_the_bounds() {
        let fast_east = setpoint(90.0, PLANT_MAX_SPEED_MPS);
        let high = p_control(fast_east, &observed(0.0, 0.0));
        assert_eq!((high.rudder(), high.throttle()), (RUDDER_MAX, THROTTLE_MAX));

        let stopped_north = setpoint(0.0, SPEED_MIN_MPS);
        let low = p_control(stopped_north, &observed(90.0, PLANT_MAX_SPEED_MPS));
        assert_eq!((low.rudder(), low.throttle()), (RUDDER_MIN, THROTTLE_MIN));
    }

    #[test]
    fn rudder_sign_follows_the_shortest_turn_across_north() {
        let target = setpoint(10.0, DEFAULT_TARGET_SPEED_MPS);
        let actuation = p_control(target, &observed(350.0, DEFAULT_TARGET_SPEED_MPS));
        assert!(actuation.rudder() > 0.0, "{actuation:?}");
    }

    #[test]
    fn closed_loop_converges_to_the_setpoint() {
        let target = setpoint(DEFAULT_TARGET_HEADING_DEG, DEFAULT_TARGET_SPEED_MPS);
        let (plant, _) = run_closed_loop(PlantState::initial(), target, 600);
        assert!(
            heading_error_deg(target.heading_deg(), plant.heading_deg).abs() < 0.5,
            "{plant:?}"
        );
        assert!(
            (plant.speed_mps - target.speed_mps()).abs() < 0.01,
            "{plant:?}"
        );
    }

    #[test]
    fn closed_loop_turns_the_short_way_across_north() {
        let target = setpoint(10.0, DEFAULT_TARGET_SPEED_MPS);
        let mut start = PlantState::initial();
        start.heading_deg = 350.0;
        let (plant, headings) = run_closed_loop(start, target, 600);
        for heading in headings {
            assert!(heading >= 350.0 || heading <= 10.0 + 1e-9, "{heading}");
        }
        assert!(heading_error_deg(target.heading_deg(), plant.heading_deg).abs() < 0.5);
    }
}
