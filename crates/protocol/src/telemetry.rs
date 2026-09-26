//! Versioned JSON telemetry message and its boundary validation.

use serde::{Deserialize, Serialize};
use thiserror::Error;

use crate::{
    BATTERY_MAX_PERCENT, BATTERY_MIN_PERCENT, HEADING_MAX_DEG, HEADING_MIN_DEG,
    INITIAL_BATTERY_PERCENT, INITIAL_LATITUDE_DEG, INITIAL_LONGITUDE_DEG, LATITUDE_MAX_DEG,
    LATITUDE_MIN_DEG, LONGITUDE_MAX_DEG, LONGITUDE_MIN_DEG, SPEED_MAX_MPS, SPEED_MIN_MPS,
    SUPPORTED_VERSION,
};

// Fixed heading and speed used by `Telemetry::simulated`.
const SIMULATED_HEADING_DEG: f64 = 90.0;
const SIMULATED_SPEED_MPS: f64 = 1.5;

/// One telemetry sample as carried in a single UDP datagram.
///
/// `sent_at_ms` is the sender's Unix time in milliseconds. It is
/// informational only and is never used to judge freshness.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct Telemetry {
    pub version: u8,
    pub sequence: u64,
    pub sent_at_ms: u128,
    pub latitude_deg: f64,
    pub longitude_deg: f64,
    pub heading_deg: f64,
    pub speed_mps: f64,
    pub battery_percent: f32,
}

/// A decoded message that violates a protocol rule.
#[derive(Debug, Clone, PartialEq, Error)]
pub enum ValidationError {
    #[error("unsupported version {0}")]
    UnsupportedVersion(u8),
    #[error("field {0} is not finite")]
    NonFinite(&'static str),
    #[error("latitude_deg {0} is out of range")]
    LatitudeOutOfRange(f64),
    #[error("longitude_deg {0} is out of range")]
    LongitudeOutOfRange(f64),
    #[error("heading_deg {0} is out of range")]
    HeadingOutOfRange(f64),
    #[error("speed_mps {0} is out of range")]
    SpeedOutOfRange(f64),
    #[error("battery_percent {0} is out of range")]
    BatteryOutOfRange(f32),
}

/// Why a datagram could not be turned into valid telemetry.
#[derive(Debug, Error)]
pub enum DecodeError {
    #[error("malformed telemetry JSON: {0}")]
    Malformed(#[from] serde_json::Error),
    #[error("invalid telemetry: {0}")]
    Invalid(#[from] ValidationError),
}

impl Telemetry {
    /// Builds the fixed simulated sample sent by `sensor-node`.
    pub fn simulated(sequence: u64, sent_at_ms: u128) -> Self {
        Self {
            version: SUPPORTED_VERSION,
            sequence,
            sent_at_ms,
            latitude_deg: INITIAL_LATITUDE_DEG,
            longitude_deg: INITIAL_LONGITUDE_DEG,
            heading_deg: SIMULATED_HEADING_DEG,
            speed_mps: SIMULATED_SPEED_MPS,
            battery_percent: INITIAL_BATTERY_PERCENT,
        }
    }

    /// Serializes to the JSON wire format.
    pub fn encode(&self) -> Result<Vec<u8>, serde_json::Error> {
        serde_json::to_vec(self)
    }

    /// Parses a datagram and validates it. This is the network boundary.
    pub fn decode(bytes: &[u8]) -> Result<Self, DecodeError> {
        let telemetry: Self = serde_json::from_slice(bytes)?;
        telemetry.validate()?;
        Ok(telemetry)
    }

    /// Checks version, finiteness, and physical ranges.
    pub fn validate(&self) -> Result<(), ValidationError> {
        if self.version != SUPPORTED_VERSION {
            return Err(ValidationError::UnsupportedVersion(self.version));
        }

        let floats = [
            ("latitude_deg", self.latitude_deg),
            ("longitude_deg", self.longitude_deg),
            ("heading_deg", self.heading_deg),
            ("speed_mps", self.speed_mps),
        ];
        for (field, value) in floats {
            if !value.is_finite() {
                return Err(ValidationError::NonFinite(field));
            }
        }
        if !self.battery_percent.is_finite() {
            return Err(ValidationError::NonFinite("battery_percent"));
        }

        if !(LATITUDE_MIN_DEG..=LATITUDE_MAX_DEG).contains(&self.latitude_deg) {
            return Err(ValidationError::LatitudeOutOfRange(self.latitude_deg));
        }
        if !(LONGITUDE_MIN_DEG..=LONGITUDE_MAX_DEG).contains(&self.longitude_deg) {
            return Err(ValidationError::LongitudeOutOfRange(self.longitude_deg));
        }
        if !(HEADING_MIN_DEG..HEADING_MAX_DEG).contains(&self.heading_deg) {
            return Err(ValidationError::HeadingOutOfRange(self.heading_deg));
        }
        if !(SPEED_MIN_MPS..=SPEED_MAX_MPS).contains(&self.speed_mps) {
            return Err(ValidationError::SpeedOutOfRange(self.speed_mps));
        }
        if !(BATTERY_MIN_PERCENT..=BATTERY_MAX_PERCENT).contains(&self.battery_percent) {
            return Err(ValidationError::BatteryOutOfRange(self.battery_percent));
        }
        Ok(())
    }
}

#[cfg(test)]
#[allow(clippy::unwrap_used, clippy::expect_used)]
mod tests {
    use super::*;

    const STEP: f64 = 0.001;

    fn sample() -> Telemetry {
        Telemetry::simulated(7, 1_700_000_000_123)
    }

    fn with(edit: impl FnOnce(&mut Telemetry)) -> Telemetry {
        let mut telemetry = sample();
        edit(&mut telemetry);
        telemetry
    }

    #[test]
    fn simulated_sample_is_valid() {
        assert_eq!(sample().validate(), Ok(()));
    }

    #[test]
    fn json_round_trip_preserves_every_field() {
        let original = sample();
        let bytes = original.encode().expect("encode");
        let decoded = Telemetry::decode(&bytes).expect("decode");
        assert_eq!(decoded, original);
    }

    #[test]
    fn malformed_json_is_rejected() {
        let valid = String::from_utf8(sample().encode().expect("encode")).expect("utf8");
        let cases: Vec<(&str, String)> = vec![
            ("empty", String::new()),
            ("not json", "hello".to_owned()),
            ("truncated", valid[..valid.len() / 2].to_owned()),
            ("missing field", valid.replace(",\"speed_mps\":1.5", "")),
            (
                "wrong type",
                valid.replace("\"sequence\":7", "\"sequence\":\"7\""),
            ),
            (
                "negative sequence",
                valid.replace("\"sequence\":7", "\"sequence\":-7"),
            ),
            ("unknown field", valid.replace('}', ",\"extra\":1}")),
            (
                "NaN literal",
                valid.replace("\"heading_deg\":90.0", "\"heading_deg\":NaN"),
            ),
        ];
        for (name, input) in cases {
            assert_ne!(input, valid, "case {name} did not alter the input");
            let result = Telemetry::decode(input.as_bytes());
            assert!(
                matches!(result, Err(DecodeError::Malformed(_))),
                "case {name}: expected Malformed, got {result:?}"
            );
        }
    }

    #[test]
    fn unsupported_version_is_rejected() {
        for version in [SUPPORTED_VERSION - 1, SUPPORTED_VERSION + 1, u8::MAX] {
            assert_eq!(
                with(|t| t.version = version).validate(),
                Err(ValidationError::UnsupportedVersion(version))
            );
        }
    }

    #[test]
    fn non_finite_values_are_rejected() {
        type Setter = fn(&mut Telemetry, f64);
        let fields: [(&str, Setter); 5] = [
            ("latitude_deg", |t, v| t.latitude_deg = v),
            ("longitude_deg", |t, v| t.longitude_deg = v),
            ("heading_deg", |t, v| t.heading_deg = v),
            ("speed_mps", |t, v| t.speed_mps = v),
            ("battery_percent", |t, v| t.battery_percent = v as f32),
        ];
        for (field, set) in fields {
            for value in [f64::NAN, f64::INFINITY, f64::NEG_INFINITY] {
                assert_eq!(
                    with(|t| set(t, value)).validate(),
                    Err(ValidationError::NonFinite(field)),
                    "{field} = {value}"
                );
            }
        }
    }

    #[test]
    fn json_number_overflowing_f32_is_rejected_as_non_finite() {
        let json = String::from_utf8(sample().encode().expect("encode"))
            .expect("utf8")
            .replace("\"battery_percent\":87.5", "\"battery_percent\":1e39");
        let result = Telemetry::decode(json.as_bytes());
        assert!(
            matches!(
                result,
                Err(DecodeError::Invalid(ValidationError::NonFinite(
                    "battery_percent"
                )))
            ),
            "got {result:?}"
        );
    }

    #[test]
    fn latitude_bounds_are_inclusive() {
        for ok in [LATITUDE_MIN_DEG, LATITUDE_MAX_DEG] {
            assert_eq!(with(|t| t.latitude_deg = ok).validate(), Ok(()));
        }
        for bad in [LATITUDE_MIN_DEG - STEP, LATITUDE_MAX_DEG + STEP] {
            assert_eq!(
                with(|t| t.latitude_deg = bad).validate(),
                Err(ValidationError::LatitudeOutOfRange(bad))
            );
        }
    }

    #[test]
    fn longitude_bounds_are_inclusive() {
        for ok in [LONGITUDE_MIN_DEG, LONGITUDE_MAX_DEG] {
            assert_eq!(with(|t| t.longitude_deg = ok).validate(), Ok(()));
        }
        for bad in [LONGITUDE_MIN_DEG - STEP, LONGITUDE_MAX_DEG + STEP] {
            assert_eq!(
                with(|t| t.longitude_deg = bad).validate(),
                Err(ValidationError::LongitudeOutOfRange(bad))
            );
        }
    }

    #[test]
    fn heading_minimum_is_inclusive_and_maximum_exclusive() {
        for ok in [HEADING_MIN_DEG, HEADING_MAX_DEG - STEP] {
            assert_eq!(with(|t| t.heading_deg = ok).validate(), Ok(()));
        }
        for bad in [HEADING_MIN_DEG - STEP, HEADING_MAX_DEG] {
            assert_eq!(
                with(|t| t.heading_deg = bad).validate(),
                Err(ValidationError::HeadingOutOfRange(bad))
            );
        }
    }

    #[test]
    fn speed_bounds_are_inclusive() {
        for ok in [SPEED_MIN_MPS, SPEED_MAX_MPS] {
            assert_eq!(with(|t| t.speed_mps = ok).validate(), Ok(()));
        }
        for bad in [SPEED_MIN_MPS - STEP, SPEED_MAX_MPS + STEP] {
            assert_eq!(
                with(|t| t.speed_mps = bad).validate(),
                Err(ValidationError::SpeedOutOfRange(bad))
            );
        }
    }

    #[test]
    fn battery_bounds_are_inclusive() {
        for ok in [BATTERY_MIN_PERCENT, BATTERY_MAX_PERCENT] {
            assert_eq!(with(|t| t.battery_percent = ok).validate(), Ok(()));
        }
        let step = STEP as f32;
        for bad in [BATTERY_MIN_PERCENT - step, BATTERY_MAX_PERCENT + step] {
            assert_eq!(
                with(|t| t.battery_percent = bad).validate(),
                Err(ValidationError::BatteryOutOfRange(bad))
            );
        }
    }

    #[test]
    fn decode_reports_validation_failures_as_invalid() {
        let bytes = with(|t| t.speed_mps = -1.0).encode().expect("encode");
        assert!(matches!(
            Telemetry::decode(&bytes),
            Err(DecodeError::Invalid(ValidationError::SpeedOutOfRange(_)))
        ));
    }
}
