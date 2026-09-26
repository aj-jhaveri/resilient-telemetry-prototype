//! Command wire message and the sensor-side receiver with its watchdog.
//!
//! `Command` keeps private fields so it can only be built correctly.
//! `CommandMessage` is the separate wire type, validated at the receiving
//! boundary exactly like telemetry.

use std::time::{Duration, Instant};

use serde::{Deserialize, Serialize};
use thiserror::Error;

use crate::control::{Command, CommandMode, CommandReason};
use crate::plant::Actuation;
use crate::sequence::{SequenceError, SequenceGate};
use crate::{
    COMMAND_VERSION, RUDDER_MAX, RUDDER_MIN, SAFE_STOP_RUDDER, SAFE_STOP_THROTTLE, THROTTLE_MAX,
    THROTTLE_MIN,
};

/// One command as carried in a single UDP datagram.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct CommandMessage {
    pub version: u8,
    pub sequence: u64,
    pub mode: CommandMode,
    pub rudder: f64,
    pub throttle: f64,
    pub reason: CommandReason,
}

/// A decoded command that violates a protocol rule.
#[derive(Debug, Clone, PartialEq, Error)]
pub enum CommandValidationError {
    #[error("unsupported command version {0}")]
    UnsupportedVersion(u8),
    #[error("field {0} is not finite")]
    NonFinite(&'static str),
    #[error("rudder {0} is out of range")]
    RudderOutOfRange(f64),
    #[error("throttle {0} is out of range")]
    ThrottleOutOfRange(f64),
    #[error("mode {mode:?} does not match reason {reason}")]
    ReasonMismatch {
        mode: CommandMode,
        reason: CommandReason,
    },
    #[error("safe_stop command must carry the SafeStop rudder and throttle")]
    SafeStopNotStopped,
}

/// Why a datagram could not be turned into a valid command.
#[derive(Debug, Error)]
pub enum CommandDecodeError {
    #[error("malformed command JSON: {0}")]
    Malformed(#[from] serde_json::Error),
    #[error("invalid command: {0}")]
    Invalid(#[from] CommandValidationError),
}

/// Why the receiver ignored a command datagram.
#[derive(Debug, Error)]
pub enum CommandRejection {
    #[error(transparent)]
    Decode(#[from] CommandDecodeError),
    #[error(transparent)]
    Sequence(#[from] SequenceError),
}

impl Command {
    /// Converts to the wire message.
    pub fn to_message(&self) -> CommandMessage {
        CommandMessage {
            version: COMMAND_VERSION,
            sequence: self.sequence(),
            mode: self.mode(),
            rudder: self.rudder(),
            throttle: self.throttle(),
            reason: self.reason(),
        }
    }
}

impl CommandMessage {
    pub fn encode(&self) -> Result<Vec<u8>, serde_json::Error> {
        serde_json::to_vec(self)
    }

    /// Parses a datagram and validates it. This is the sensor-side boundary.
    pub fn decode(bytes: &[u8]) -> Result<Self, CommandDecodeError> {
        let message: Self = serde_json::from_slice(bytes)?;
        message.validate()?;
        Ok(message)
    }

    /// Checks version, finiteness, bounds, and mode/reason consistency.
    pub fn validate(&self) -> Result<(), CommandValidationError> {
        if self.version != COMMAND_VERSION {
            return Err(CommandValidationError::UnsupportedVersion(self.version));
        }
        for (field, value) in [("rudder", self.rudder), ("throttle", self.throttle)] {
            if !value.is_finite() {
                return Err(CommandValidationError::NonFinite(field));
            }
        }
        if !(RUDDER_MIN..=RUDDER_MAX).contains(&self.rudder) {
            return Err(CommandValidationError::RudderOutOfRange(self.rudder));
        }
        if !(THROTTLE_MIN..=THROTTLE_MAX).contains(&self.throttle) {
            return Err(CommandValidationError::ThrottleOutOfRange(self.throttle));
        }
        let consistent = matches!(
            (self.mode, self.reason),
            (CommandMode::Autonomous, CommandReason::ValidTelemetry)
                | (CommandMode::SafeStop, CommandReason::TelemetryTimeout)
        );
        if !consistent {
            return Err(CommandValidationError::ReasonMismatch {
                mode: self.mode,
                reason: self.reason,
            });
        }
        if self.mode == CommandMode::SafeStop
            && (self.rudder != SAFE_STOP_RUDDER || self.throttle != SAFE_STOP_THROTTLE)
        {
            return Err(CommandValidationError::SafeStopNotStopped);
        }
        Ok(())
    }

    fn actuation(&self) -> Actuation {
        Actuation::clamped(self.rudder, self.throttle)
    }
}

/// What the plant should apply at a given instant.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Applied {
    pub actuation: Actuation,
    /// True only on the call where the command watchdog first expires.
    pub watchdog_tripped: bool,
}

/// Holds the latest valid command and zeroes it when commands stop arriving.
#[derive(Debug)]
pub struct CommandReceiver {
    gate: SequenceGate,
    timeout: Duration,
    last_command_at: Option<Instant>,
    current: Actuation,
    stale: bool,
}

impl CommandReceiver {
    pub fn new(timeout: Duration) -> Self {
        Self {
            gate: SequenceGate::default(),
            timeout,
            last_command_at: None,
            current: Actuation::ZERO,
            stale: true,
        }
    }

    /// Decodes, validates, and sequence-checks one command datagram. A
    /// rejected datagram changes nothing and does not refresh the watchdog.
    pub fn handle_datagram(
        &mut self,
        bytes: &[u8],
        now: Instant,
    ) -> Result<CommandMessage, CommandRejection> {
        let message = CommandMessage::decode(bytes)?;
        self.gate.check_and_record(message.sequence)?;
        self.last_command_at = Some(now);
        self.current = message.actuation();
        self.stale = false;
        Ok(message)
    }

    /// Returns the latest command's actuation, or zero before the first
    /// command and once `now - last_command >= timeout`.
    pub fn applied(&mut self, now: Instant) -> Applied {
        let fresh = self
            .last_command_at
            .is_some_and(|at| now.saturating_duration_since(at) < self.timeout);
        if fresh {
            return Applied {
                actuation: self.current,
                watchdog_tripped: false,
            };
        }
        let watchdog_tripped = !self.stale;
        self.stale = true;
        Applied {
            actuation: Actuation::ZERO,
            watchdog_tripped,
        }
    }
}

#[cfg(test)]
#[allow(clippy::unwrap_used, clippy::expect_used)]
mod tests {
    use super::*;
    use crate::{
        Controller, ControllerConfig, PlantState, Setpoint, DEFAULT_COMMAND_TIMEOUT_MS,
        DEFAULT_RECOVERY_PACKETS, DEFAULT_TARGET_HEADING_DEG, DEFAULT_TARGET_SPEED_MPS,
        DEFAULT_TIMEOUT_MS,
    };

    const ONE_MS: Duration = Duration::from_millis(1);

    fn timeout() -> Duration {
        Duration::from_millis(DEFAULT_COMMAND_TIMEOUT_MS)
    }

    fn autonomous(sequence: u64, rudder: f64, throttle: f64) -> CommandMessage {
        CommandMessage {
            version: COMMAND_VERSION,
            sequence,
            mode: CommandMode::Autonomous,
            rudder,
            throttle,
            reason: CommandReason::ValidTelemetry,
        }
    }

    fn safe_stop(sequence: u64) -> CommandMessage {
        CommandMessage {
            version: COMMAND_VERSION,
            sequence,
            mode: CommandMode::SafeStop,
            rudder: SAFE_STOP_RUDDER,
            throttle: SAFE_STOP_THROTTLE,
            reason: CommandReason::TelemetryTimeout,
        }
    }

    fn bytes(message: &CommandMessage) -> Vec<u8> {
        message.encode().expect("encode")
    }

    #[test]
    fn command_watchdog_outlasts_the_telemetry_timeout() {
        const { assert!(DEFAULT_COMMAND_TIMEOUT_MS > DEFAULT_TIMEOUT_MS) };
    }

    #[test]
    fn controller_command_round_trips_through_the_wire_message() {
        let mut controller = Controller::new(ControllerConfig {
            timeout: Duration::from_millis(DEFAULT_TIMEOUT_MS),
            recovery_packets: DEFAULT_RECOVERY_PACKETS,
            setpoint: Setpoint::new(DEFAULT_TARGET_HEADING_DEG, DEFAULT_TARGET_SPEED_MPS)
                .expect("setpoint"),
        });
        let command = controller
            .on_telemetry(&PlantState::initial().to_telemetry(1, 0), Instant::now())
            .expect("accepted")
            .command
            .expect("autonomous command");
        let message = command.to_message();
        let decoded = CommandMessage::decode(&bytes(&message)).expect("decode");
        assert_eq!(decoded, message);
        assert_eq!(decoded.sequence, command.sequence());
        assert_eq!(decoded.rudder, command.rudder());
        assert_eq!(decoded.throttle, command.throttle());
    }

    #[test]
    fn wire_names_match_the_reason_strings() {
        for message in [autonomous(1, 0.0, 0.5), safe_stop(2)] {
            let json = String::from_utf8(bytes(&message)).expect("utf8");
            let reason = format!("\"reason\":\"{}\"", message.reason.as_str());
            assert!(json.contains(&reason), "{json}");
        }
    }

    #[test]
    fn malformed_commands_are_rejected() {
        let valid = String::from_utf8(bytes(&autonomous(1, 0.0, 0.5))).expect("utf8");
        let cases: Vec<(&str, String)> = vec![
            ("empty", String::new()),
            ("truncated", valid[..valid.len() / 2].to_owned()),
            ("unknown field", valid.replace('}', ",\"extra\":1}")),
            (
                "unknown mode",
                valid.replace("\"autonomous\"", "\"full_ahead\""),
            ),
            (
                "wrong type",
                valid.replace("\"sequence\":1", "\"sequence\":\"1\""),
            ),
        ];
        for (name, input) in cases {
            assert_ne!(input, valid, "case {name} did not alter the input");
            let result = CommandMessage::decode(input.as_bytes());
            assert!(
                matches!(result, Err(CommandDecodeError::Malformed(_))),
                "case {name}: {result:?}"
            );
        }
    }

    #[test]
    fn invalid_commands_are_rejected() {
        let mut wrong_version = autonomous(1, 0.0, 0.5);
        wrong_version.version = COMMAND_VERSION + 1;
        let mut mismatched = safe_stop(1);
        mismatched.reason = CommandReason::ValidTelemetry;
        let mut moving_stop = safe_stop(1);
        moving_stop.throttle = THROTTLE_MAX;

        let cases = [
            (
                wrong_version,
                CommandValidationError::UnsupportedVersion(COMMAND_VERSION + 1),
            ),
            (
                autonomous(1, f64::NAN, 0.5),
                CommandValidationError::NonFinite("rudder"),
            ),
            (
                autonomous(1, 0.0, f64::INFINITY),
                CommandValidationError::NonFinite("throttle"),
            ),
            (
                autonomous(1, RUDDER_MAX + 0.1, 0.5),
                CommandValidationError::RudderOutOfRange(RUDDER_MAX + 0.1),
            ),
            (
                autonomous(1, RUDDER_MIN - 0.1, 0.5),
                CommandValidationError::RudderOutOfRange(RUDDER_MIN - 0.1),
            ),
            (
                autonomous(1, 0.0, THROTTLE_MAX + 0.1),
                CommandValidationError::ThrottleOutOfRange(THROTTLE_MAX + 0.1),
            ),
            (
                autonomous(1, 0.0, THROTTLE_MIN - 0.1),
                CommandValidationError::ThrottleOutOfRange(THROTTLE_MIN - 0.1),
            ),
            (
                mismatched,
                CommandValidationError::ReasonMismatch {
                    mode: CommandMode::SafeStop,
                    reason: CommandReason::ValidTelemetry,
                },
            ),
            (moving_stop, CommandValidationError::SafeStopNotStopped),
        ];
        for (message, expected) in cases {
            assert_eq!(message.validate(), Err(expected));
        }
    }

    #[test]
    fn receiver_applies_zero_before_any_command() {
        let mut receiver = CommandReceiver::new(timeout());
        let applied = receiver.applied(Instant::now());
        assert_eq!(applied.actuation, Actuation::ZERO);
        assert!(!applied.watchdog_tripped);
    }

    #[test]
    fn receiver_applies_valid_command_until_the_watchdog_expires_once() {
        let t0 = Instant::now();
        let mut receiver = CommandReceiver::new(timeout());
        receiver
            .handle_datagram(&bytes(&autonomous(1, 0.5, 0.25)), t0)
            .expect("accepted");

        let fresh = receiver.applied(t0 + timeout() - ONE_MS);
        assert_eq!(fresh.actuation, Actuation::clamped(0.5, 0.25));
        assert!(!fresh.watchdog_tripped);

        let expired = receiver.applied(t0 + timeout());
        assert_eq!(expired.actuation, Actuation::ZERO);
        assert!(expired.watchdog_tripped);

        let later = receiver.applied(t0 + timeout() * 3);
        assert_eq!(later.actuation, Actuation::ZERO);
        assert!(!later.watchdog_tripped);
    }

    #[test]
    fn rejected_commands_do_not_refresh_the_watchdog() {
        let t0 = Instant::now();
        let mut receiver = CommandReceiver::new(timeout());
        receiver
            .handle_datagram(&bytes(&autonomous(2, 0.5, 0.25)), t0)
            .expect("accepted");
        let near_deadline = t0 + timeout() - ONE_MS;
        assert!(matches!(
            receiver.handle_datagram(&bytes(&autonomous(2, 0.5, 0.25)), near_deadline),
            Err(CommandRejection::Sequence(SequenceError::Duplicate(2)))
        ));
        assert!(matches!(
            receiver.handle_datagram(&bytes(&autonomous(1, 0.5, 0.25)), near_deadline),
            Err(CommandRejection::Sequence(SequenceError::OutOfOrder { .. }))
        ));
        assert!(matches!(
            receiver.handle_datagram(b"{not json", near_deadline),
            Err(CommandRejection::Decode(CommandDecodeError::Malformed(_)))
        ));
        assert!(receiver.applied(t0 + timeout()).watchdog_tripped);
    }

    #[test]
    fn new_command_after_expiry_reactivates_actuation() {
        let t0 = Instant::now();
        let mut receiver = CommandReceiver::new(timeout());
        receiver
            .handle_datagram(&bytes(&autonomous(1, 0.5, 0.25)), t0)
            .expect("accepted");
        assert!(receiver.applied(t0 + timeout()).watchdog_tripped);

        let t1 = t0 + timeout() * 2;
        receiver
            .handle_datagram(&bytes(&autonomous(2, -0.5, 0.75)), t1)
            .expect("accepted");
        assert_eq!(
            receiver.applied(t1).actuation,
            Actuation::clamped(-0.5, 0.75)
        );
    }

    #[test]
    fn safe_stop_command_zeroes_actuation_immediately() {
        let t0 = Instant::now();
        let mut receiver = CommandReceiver::new(timeout());
        receiver
            .handle_datagram(&bytes(&autonomous(1, 0.5, 0.25)), t0)
            .expect("accepted");
        receiver
            .handle_datagram(&bytes(&safe_stop(2)), t0 + ONE_MS)
            .expect("accepted");
        let applied = receiver.applied(t0 + ONE_MS);
        assert_eq!(applied.actuation, Actuation::ZERO);
        assert!(!applied.watchdog_tripped);
    }
}
