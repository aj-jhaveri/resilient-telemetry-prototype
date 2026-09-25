//! Control state machine: sequence checks, freshness, and SafeStop.
//!
//! Every method takes `now: Instant` from the caller. `control-node` passes
//! `Instant::now()`; tests pass synthetic instants so timeouts are
//! deterministic without sleeping.

use std::fmt;
use std::time::{Duration, Instant};

use thiserror::Error;

use crate::telemetry::{DecodeError, Telemetry};
use crate::{
    AUTONOMOUS_RUDDER, AUTONOMOUS_THROTTLE, RUDDER_MAX, RUDDER_MIN, SAFE_STOP_RUDDER,
    SAFE_STOP_THROTTLE, THROTTLE_MAX, THROTTLE_MIN,
};

/// Controller state.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ControlState {
    WaitingForTelemetry,
    Autonomous,
    SafeStop,
}

/// Mode carried by an emitted command.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum CommandMode {
    Autonomous,
    SafeStop,
}

/// Why a command was emitted.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum CommandReason {
    ValidTelemetry,
    TelemetryTimeout,
}

impl CommandReason {
    pub fn as_str(self) -> &'static str {
        match self {
            Self::ValidTelemetry => "valid_telemetry",
            Self::TelemetryTimeout => "telemetry_timeout",
        }
    }
}

impl fmt::Display for CommandReason {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.write_str(self.as_str())
    }
}

/// A command produced by the controller. It can only be built through the
/// two private constructors below, so its values are fixed and bounded.
#[derive(Debug, Clone, PartialEq)]
pub struct Command {
    sequence: u64,
    mode: CommandMode,
    rudder: f64,
    throttle: f64,
    reason: CommandReason,
}

impl Command {
    fn autonomous(sequence: u64) -> Self {
        Self {
            sequence,
            mode: CommandMode::Autonomous,
            rudder: AUTONOMOUS_RUDDER,
            throttle: AUTONOMOUS_THROTTLE,
            reason: CommandReason::ValidTelemetry,
        }
    }

    fn safe_stop(sequence: u64) -> Self {
        Self {
            sequence,
            mode: CommandMode::SafeStop,
            rudder: SAFE_STOP_RUDDER,
            throttle: SAFE_STOP_THROTTLE,
            reason: CommandReason::TelemetryTimeout,
        }
    }

    pub fn sequence(&self) -> u64 {
        self.sequence
    }

    pub fn mode(&self) -> CommandMode {
        self.mode
    }

    pub fn rudder(&self) -> f64 {
        self.rudder
    }

    pub fn throttle(&self) -> f64 {
        self.throttle
    }

    pub fn reason(&self) -> CommandReason {
        self.reason
    }

    /// True when rudder and throttle are finite and inside their bounds.
    pub fn is_within_bounds(&self) -> bool {
        (RUDDER_MIN..=RUDDER_MAX).contains(&self.rudder)
            && (THROTTLE_MIN..=THROTTLE_MAX).contains(&self.throttle)
    }
}

/// Why a datagram was ignored.
#[derive(Debug, Error)]
pub enum Rejection {
    #[error(transparent)]
    Decode(#[from] DecodeError),
    #[error("duplicate sequence {0}")]
    Duplicate(u64),
    #[error("out-of-order sequence {received} (last accepted {last_accepted})")]
    OutOfOrder { received: u64, last_accepted: u64 },
}

/// Tracks accepted telemetry and decides the control state.
#[derive(Debug)]
pub struct Controller {
    state: ControlState,
    timeout: Duration,
    last_sequence: Option<u64>,
    last_accepted_at: Option<Instant>,
    next_command_sequence: u64,
}

impl Controller {
    pub fn new(timeout: Duration) -> Self {
        Self {
            state: ControlState::WaitingForTelemetry,
            timeout,
            last_sequence: None,
            last_accepted_at: None,
            next_command_sequence: 1,
        }
    }

    pub fn state(&self) -> ControlState {
        self.state
    }

    pub fn last_accepted_sequence(&self) -> Option<u64> {
        self.last_sequence
    }

    /// Decodes, validates, and sequence-checks one datagram.
    pub fn handle_datagram(
        &mut self,
        bytes: &[u8],
        now: Instant,
    ) -> Result<(Telemetry, Command), Rejection> {
        let telemetry = Telemetry::decode(bytes)?;
        let command = self.accept_sequence(telemetry.sequence, now)?;
        Ok((telemetry, command))
    }

    /// Validates and sequence-checks already-parsed telemetry.
    pub fn on_telemetry(
        &mut self,
        telemetry: &Telemetry,
        now: Instant,
    ) -> Result<Command, Rejection> {
        telemetry.validate().map_err(DecodeError::from)?;
        self.accept_sequence(telemetry.sequence, now)
    }

    /// Evaluates freshness. Returns a SafeStop command only on the
    /// transition into `SafeStop`; repeated calls in `SafeStop` return `None`.
    pub fn tick(&mut self, now: Instant) -> Option<Command> {
        if self.state != ControlState::Autonomous {
            return None;
        }
        let last = self.last_accepted_at?;
        if now.saturating_duration_since(last) < self.timeout {
            return None;
        }
        self.state = ControlState::SafeStop;
        Some(Command::safe_stop(self.next_sequence()))
    }

    fn accept_sequence(&mut self, sequence: u64, now: Instant) -> Result<Command, Rejection> {
        if let Some(last_accepted) = self.last_sequence {
            if sequence == last_accepted {
                return Err(Rejection::Duplicate(sequence));
            }
            if sequence < last_accepted {
                return Err(Rejection::OutOfOrder {
                    received: sequence,
                    last_accepted,
                });
            }
        }
        self.last_sequence = Some(sequence);
        self.last_accepted_at = Some(now);
        self.state = ControlState::Autonomous;
        Ok(Command::autonomous(self.next_sequence()))
    }

    fn next_sequence(&mut self) -> u64 {
        let sequence = self.next_command_sequence;
        self.next_command_sequence = self.next_command_sequence.saturating_add(1);
        sequence
    }
}

#[cfg(test)]
#[allow(clippy::unwrap_used, clippy::expect_used)]
mod tests {
    use super::*;
    use crate::telemetry::ValidationError;
    use crate::DEFAULT_TIMEOUT_MS;

    const ONE_MS: Duration = Duration::from_millis(1);

    fn timeout() -> Duration {
        Duration::from_millis(DEFAULT_TIMEOUT_MS)
    }

    fn telemetry(sequence: u64) -> Telemetry {
        Telemetry::simulated(sequence, 0)
    }

    fn autonomous_at(sequence: u64, now: Instant) -> Controller {
        let mut controller = Controller::new(timeout());
        controller
            .on_telemetry(&telemetry(sequence), now)
            .expect("first valid telemetry is accepted");
        controller
    }

    #[test]
    fn starts_waiting_and_never_times_out_without_telemetry() {
        let t0 = Instant::now();
        let mut controller = Controller::new(timeout());
        assert_eq!(controller.state(), ControlState::WaitingForTelemetry);
        assert_eq!(controller.tick(t0 + timeout() * 10), None);
        assert_eq!(controller.state(), ControlState::WaitingForTelemetry);
    }

    #[test]
    fn first_valid_telemetry_of_any_sequence_becomes_autonomous() {
        let t0 = Instant::now();
        let mut controller = Controller::new(timeout());
        let command = controller
            .on_telemetry(&telemetry(42), t0)
            .expect("accepted");
        assert_eq!(controller.state(), ControlState::Autonomous);
        assert_eq!(controller.last_accepted_sequence(), Some(42));
        assert_eq!(command.mode(), CommandMode::Autonomous);
        assert_eq!(command.reason(), CommandReason::ValidTelemetry);
    }

    #[test]
    fn accepts_strictly_increasing_sequences_including_gaps() {
        let t0 = Instant::now();
        let mut controller = autonomous_at(1, t0);
        for sequence in [2, 5, 6] {
            assert!(controller.on_telemetry(&telemetry(sequence), t0).is_ok());
        }
        assert_eq!(controller.last_accepted_sequence(), Some(6));
    }

    #[test]
    fn rejects_duplicate_and_out_of_order_sequences() {
        let t0 = Instant::now();
        let mut controller = autonomous_at(5, t0);
        assert!(matches!(
            controller.on_telemetry(&telemetry(5), t0),
            Err(Rejection::Duplicate(5))
        ));
        assert!(matches!(
            controller.on_telemetry(&telemetry(4), t0),
            Err(Rejection::OutOfOrder {
                received: 4,
                last_accepted: 5
            })
        ));
        assert_eq!(controller.last_accepted_sequence(), Some(5));
    }

    #[test]
    fn invalid_telemetry_does_not_advance_sequence() {
        let t0 = Instant::now();
        let mut controller = Controller::new(timeout());
        let mut bad = telemetry(9);
        bad.speed_mps = -1.0;
        assert!(matches!(
            controller.on_telemetry(&bad, t0),
            Err(Rejection::Decode(DecodeError::Invalid(
                ValidationError::NegativeSpeed(_)
            )))
        ));
        assert_eq!(controller.state(), ControlState::WaitingForTelemetry);
        assert!(controller.on_telemetry(&telemetry(3), t0).is_ok());
    }

    #[test]
    fn malformed_datagram_is_rejected_without_state_change() {
        let t0 = Instant::now();
        let mut controller = Controller::new(timeout());
        assert!(matches!(
            controller.handle_datagram(b"{not json", t0),
            Err(Rejection::Decode(DecodeError::Malformed(_)))
        ));
        assert_eq!(controller.state(), ControlState::WaitingForTelemetry);
        assert_eq!(controller.last_accepted_sequence(), None);
    }

    #[test]
    fn handle_datagram_accepts_encoded_telemetry() {
        let t0 = Instant::now();
        let mut controller = Controller::new(timeout());
        let bytes = telemetry(1).encode().expect("encode");
        let (accepted, _) = controller.handle_datagram(&bytes, t0).expect("accepted");
        assert_eq!(accepted.sequence, 1);
        assert_eq!(controller.state(), ControlState::Autonomous);
    }

    #[test]
    fn timeout_enters_safe_stop_exactly_once() {
        let t0 = Instant::now();
        let mut controller = autonomous_at(1, t0);

        assert_eq!(controller.tick(t0 + timeout() - ONE_MS), None);
        assert_eq!(controller.state(), ControlState::Autonomous);

        let command = controller.tick(t0 + timeout()).expect("SafeStop command");
        assert_eq!(controller.state(), ControlState::SafeStop);
        assert_eq!(command.mode(), CommandMode::SafeStop);
        assert_eq!(command.rudder(), SAFE_STOP_RUDDER);
        assert_eq!(command.throttle(), SAFE_STOP_THROTTLE);
        assert_eq!(command.reason(), CommandReason::TelemetryTimeout);
        assert_eq!(command.reason().as_str(), "telemetry_timeout");

        for later in [timeout() + ONE_MS, timeout() * 2, timeout() * 10] {
            assert_eq!(controller.tick(t0 + later), None);
            assert_eq!(controller.state(), ControlState::SafeStop);
        }
    }

    #[test]
    fn rejected_telemetry_does_not_refresh_freshness() {
        let t0 = Instant::now();
        let mut controller = autonomous_at(1, t0);
        let near_deadline = t0 + timeout() - ONE_MS;
        assert!(controller
            .on_telemetry(&telemetry(1), near_deadline)
            .is_err());
        assert!(controller.tick(t0 + timeout()).is_some());
        assert_eq!(controller.state(), ControlState::SafeStop);
    }

    #[test]
    fn accepted_telemetry_restarts_the_freshness_window() {
        let t0 = Instant::now();
        let mut controller = autonomous_at(1, t0);
        let t1 = t0 + timeout() - ONE_MS;
        assert!(controller.on_telemetry(&telemetry(2), t1).is_ok());
        assert_eq!(controller.tick(t0 + timeout()), None);
        assert!(controller.tick(t1 + timeout()).is_some());
    }

    #[test]
    fn sender_timestamp_does_not_affect_freshness() {
        let t0 = Instant::now();
        let mut controller = Controller::new(timeout());
        for (sequence, sent_at_ms) in [(1, 0), (2, u128::MAX)] {
            let sample = Telemetry::simulated(sequence, sent_at_ms);
            assert!(controller.on_telemetry(&sample, t0).is_ok());
            assert_eq!(controller.tick(t0 + timeout() - ONE_MS), None);
            assert_eq!(controller.state(), ControlState::Autonomous);
        }
    }

    #[test]
    fn valid_telemetry_recovers_from_safe_stop() {
        let t0 = Instant::now();
        let mut controller = autonomous_at(1, t0);
        assert!(controller.tick(t0 + timeout()).is_some());
        assert_eq!(controller.state(), ControlState::SafeStop);

        let t1 = t0 + timeout() * 2;
        let command = controller
            .on_telemetry(&telemetry(2), t1)
            .expect("accepted");
        assert_eq!(controller.state(), ControlState::Autonomous);
        assert_eq!(command.mode(), CommandMode::Autonomous);

        // Freshness is measured from the new packet, so SafeStop can recur.
        assert!(controller.tick(t1 + timeout()).is_some());
    }

    #[test]
    fn every_command_is_bounded_and_sequenced() {
        let t0 = Instant::now();
        let mut controller = Controller::new(timeout());
        let mut commands = Vec::new();
        commands.push(
            controller
                .on_telemetry(&telemetry(1), t0)
                .expect("accepted"),
        );
        commands.extend(controller.tick(t0 + timeout()));
        commands.push(
            controller
                .on_telemetry(&telemetry(2), t0 + timeout() * 2)
                .expect("accepted"),
        );

        let autonomous = &commands[0];
        assert_eq!(autonomous.rudder(), AUTONOMOUS_RUDDER);
        assert_eq!(autonomous.throttle(), AUTONOMOUS_THROTTLE);
        assert_eq!(autonomous.reason().as_str(), "valid_telemetry");

        assert_eq!(commands.len(), 3);
        assert_eq!(commands[1].mode(), CommandMode::SafeStop);
        for (index, command) in commands.iter().enumerate() {
            assert!(command.is_within_bounds(), "{command:?}");
            assert_eq!(command.sequence(), index as u64 + 1);
        }
    }
}
