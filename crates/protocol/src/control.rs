//! Control state machine: sequence checks, freshness, SafeStop with recovery
//! hysteresis, and P-control commands.
//!
//! Every method takes `now: Instant` from the caller. `control-node` passes
//! `Instant::now()`; tests pass synthetic instants so timeouts are
//! deterministic without sleeping.

use std::fmt;
use std::time::{Duration, Instant};

use serde::{Deserialize, Serialize};
use thiserror::Error;

use crate::p_control::{p_control, Setpoint};
use crate::plant::Actuation;
use crate::sequence::{SequenceError, SequenceGate};
use crate::telemetry::{DecodeError, Telemetry};
use crate::{
    RUDDER_MAX, RUDDER_MIN, SAFE_STOP_RUDDER, SAFE_STOP_THROTTLE, THROTTLE_MAX, THROTTLE_MIN,
};

/// Controller state.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ControlState {
    WaitingForTelemetry,
    Autonomous,
    SafeStop,
}

/// Mode carried by an emitted command.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum CommandMode {
    Autonomous,
    SafeStop,
}

/// Why a command was emitted. The serde names match `as_str`.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
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
/// two private constructors below: `autonomous` takes an already-clamped
/// `Actuation`, and `safe_stop` uses the SafeStop constants. Every command
/// is therefore bounded.
#[derive(Debug, Clone, PartialEq)]
pub struct Command {
    sequence: u64,
    mode: CommandMode,
    rudder: f64,
    throttle: f64,
    reason: CommandReason,
}

impl Command {
    fn autonomous(sequence: u64, actuation: Actuation) -> Self {
        Self {
            sequence,
            mode: CommandMode::Autonomous,
            rudder: actuation.rudder(),
            throttle: actuation.throttle(),
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
    #[error(transparent)]
    Sequence(#[from] SequenceError),
}

/// Result of accepting one telemetry packet.
#[derive(Debug, Clone, PartialEq)]
pub struct Update {
    pub telemetry: Telemetry,
    /// SafeStop command emitted because the freshness deadline had already
    /// passed when this packet arrived, before it was accepted.
    pub timeout_command: Option<Command>,
    /// Autonomous command, or `None` while SafeStop recovery is incomplete.
    pub command: Option<Command>,
}

/// Controller settings.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct ControllerConfig {
    /// Time without accepted telemetry before SafeStop.
    pub timeout: Duration,
    /// Consecutive fresh packets needed to leave SafeStop. Values below 1
    /// behave as 1.
    pub recovery_packets: u32,
    pub setpoint: Setpoint,
}

/// Tracks accepted telemetry and decides the control state.
#[derive(Debug)]
pub struct Controller {
    state: ControlState,
    timeout: Duration,
    recovery_packets: u32,
    setpoint: Setpoint,
    gate: SequenceGate,
    last_accepted_at: Option<Instant>,
    recovery_streak: u32,
    next_command_sequence: u64,
}

impl Controller {
    pub fn new(config: ControllerConfig) -> Self {
        Self {
            state: ControlState::WaitingForTelemetry,
            timeout: config.timeout,
            recovery_packets: config.recovery_packets.max(1),
            setpoint: config.setpoint,
            gate: SequenceGate::default(),
            last_accepted_at: None,
            recovery_streak: 0,
            next_command_sequence: 1,
        }
    }

    pub fn state(&self) -> ControlState {
        self.state
    }

    pub fn last_accepted_sequence(&self) -> Option<u64> {
        self.gate.last_accepted()
    }

    /// Consecutive fresh packets counted so far while in `SafeStop`.
    pub fn recovery_streak(&self) -> u32 {
        self.recovery_streak
    }

    pub fn recovery_packets(&self) -> u32 {
        self.recovery_packets
    }

    /// Decodes, validates, and sequence-checks one datagram.
    pub fn handle_datagram(&mut self, bytes: &[u8], now: Instant) -> Result<Update, Rejection> {
        let telemetry = Telemetry::decode(bytes)?;
        self.accept(telemetry, now)
    }

    /// Validates and sequence-checks already-parsed telemetry.
    pub fn on_telemetry(
        &mut self,
        telemetry: &Telemetry,
        now: Instant,
    ) -> Result<Update, Rejection> {
        telemetry.validate().map_err(DecodeError::from)?;
        self.accept(telemetry.clone(), now)
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
        self.recovery_streak = 0;
        Some(Command::safe_stop(self.next_sequence()))
    }

    fn accept(&mut self, telemetry: Telemetry, now: Instant) -> Result<Update, Rejection> {
        self.gate.check_and_record(telemetry.sequence)?;
        // Judge freshness at the arrival instant, before this packet
        // refreshes it, so the outcome does not depend on when the caller
        // last called `tick`.
        let timeout_command = self.tick(now);
        let fresh_gap = self
            .last_accepted_at
            .is_some_and(|at| now.saturating_duration_since(at) < self.timeout);
        self.last_accepted_at = Some(now);

        let resume = match self.state {
            ControlState::WaitingForTelemetry | ControlState::Autonomous => true,
            ControlState::SafeStop => {
                // Hysteresis: leave SafeStop only after `recovery_packets`
                // consecutive packets, each within `timeout` of the last.
                self.recovery_streak = if fresh_gap {
                    self.recovery_streak.saturating_add(1)
                } else {
                    1
                };
                self.recovery_streak >= self.recovery_packets
            }
        };

        let command = if resume {
            self.state = ControlState::Autonomous;
            self.recovery_streak = 0;
            let actuation = p_control(self.setpoint, &telemetry);
            Some(Command::autonomous(self.next_sequence(), actuation))
        } else {
            None
        };

        Ok(Update {
            telemetry,
            timeout_command,
            command,
        })
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
    use crate::plant::PlantState;
    use crate::telemetry::ValidationError;
    use crate::{
        DEFAULT_POLL_INTERVAL_MS, DEFAULT_RECOVERY_PACKETS, DEFAULT_TARGET_HEADING_DEG,
        DEFAULT_TARGET_SPEED_MPS, DEFAULT_TIMEOUT_MS,
    };

    const ONE_MS: Duration = Duration::from_millis(1);

    fn timeout() -> Duration {
        Duration::from_millis(DEFAULT_TIMEOUT_MS)
    }

    fn setpoint() -> Setpoint {
        Setpoint::new(DEFAULT_TARGET_HEADING_DEG, DEFAULT_TARGET_SPEED_MPS).expect("setpoint")
    }

    fn controller_with(recovery_packets: u32) -> Controller {
        Controller::new(ControllerConfig {
            timeout: timeout(),
            recovery_packets,
            setpoint: setpoint(),
        })
    }

    fn controller() -> Controller {
        controller_with(DEFAULT_RECOVERY_PACKETS)
    }

    fn telemetry(sequence: u64) -> Telemetry {
        PlantState::initial().to_telemetry(sequence, 0)
    }

    fn autonomous_at(sequence: u64, now: Instant) -> Controller {
        let mut controller = controller();
        controller
            .on_telemetry(&telemetry(sequence), now)
            .expect("first valid telemetry is accepted");
        controller
    }

    /// Puts a controller into SafeStop at `t0 + timeout` after one packet
    /// at `t0`.
    fn safe_stopped(recovery_packets: u32, t0: Instant) -> Controller {
        let mut controller = controller_with(recovery_packets);
        controller
            .on_telemetry(&telemetry(1), t0)
            .expect("accepted");
        assert!(controller.tick(t0 + timeout()).is_some());
        assert_eq!(controller.state(), ControlState::SafeStop);
        controller
    }

    #[test]
    fn starts_waiting_and_never_times_out_without_telemetry() {
        let t0 = Instant::now();
        let mut controller = controller();
        assert_eq!(controller.state(), ControlState::WaitingForTelemetry);
        assert_eq!(controller.tick(t0 + timeout() * 10), None);
        assert_eq!(controller.state(), ControlState::WaitingForTelemetry);
    }

    #[test]
    fn first_valid_telemetry_of_any_sequence_becomes_autonomous() {
        let t0 = Instant::now();
        let mut controller = controller();
        let update = controller
            .on_telemetry(&telemetry(42), t0)
            .expect("accepted");
        assert_eq!(controller.state(), ControlState::Autonomous);
        assert_eq!(controller.last_accepted_sequence(), Some(42));
        assert_eq!(update.timeout_command, None);
        let command = update.command.expect("autonomous command");
        assert_eq!(command.mode(), CommandMode::Autonomous);
        assert_eq!(command.reason(), CommandReason::ValidTelemetry);
    }

    #[test]
    fn autonomous_command_carries_the_p_control_output() {
        let t0 = Instant::now();
        let mut controller = controller();
        let sample = telemetry(1);
        let command = controller
            .on_telemetry(&sample, t0)
            .expect("accepted")
            .command
            .expect("autonomous command");
        let expected = p_control(setpoint(), &sample);
        assert_eq!(command.rudder(), expected.rudder());
        assert_eq!(command.throttle(), expected.throttle());
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
            Err(Rejection::Sequence(SequenceError::Duplicate(5)))
        ));
        assert!(matches!(
            controller.on_telemetry(&telemetry(4), t0),
            Err(Rejection::Sequence(SequenceError::OutOfOrder {
                received: 4,
                last_accepted: 5
            }))
        ));
        assert_eq!(controller.last_accepted_sequence(), Some(5));
    }

    #[test]
    fn invalid_telemetry_does_not_advance_sequence() {
        let t0 = Instant::now();
        let mut controller = controller();
        let mut bad = telemetry(9);
        bad.speed_mps = -1.0;
        assert!(matches!(
            controller.on_telemetry(&bad, t0),
            Err(Rejection::Decode(DecodeError::Invalid(
                ValidationError::SpeedOutOfRange(_)
            )))
        ));
        assert_eq!(controller.state(), ControlState::WaitingForTelemetry);
        assert!(controller.on_telemetry(&telemetry(3), t0).is_ok());
    }

    #[test]
    fn malformed_datagram_is_rejected_without_state_change() {
        let t0 = Instant::now();
        let mut controller = controller();
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
        let mut controller = controller();
        let bytes = telemetry(1).encode().expect("encode");
        let update = controller.handle_datagram(&bytes, t0).expect("accepted");
        assert_eq!(update.telemetry.sequence, 1);
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
    fn packet_arriving_after_deadline_reports_timeout_before_acceptance() {
        let t0 = Instant::now();
        let mut controller = autonomous_at(1, t0);
        // No tick between the two packets: the gap is judged on arrival.
        let update = controller
            .on_telemetry(&telemetry(2), t0 + timeout())
            .expect("accepted");
        let safe_stop = update.timeout_command.expect("timeout detected on arrival");
        assert_eq!(safe_stop.mode(), CommandMode::SafeStop);
        // The late packet only starts the recovery streak.
        assert_eq!(update.command, None);
        assert_eq!(controller.state(), ControlState::SafeStop);
        assert_eq!(controller.recovery_streak(), 1);
    }

    #[test]
    fn packet_arriving_before_deadline_reports_no_timeout() {
        let t0 = Instant::now();
        let mut controller = autonomous_at(1, t0);
        let update = controller
            .on_telemetry(&telemetry(2), t0 + timeout() - ONE_MS)
            .expect("accepted");
        assert_eq!(update.timeout_command, None);
        assert!(update.command.is_some());
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
        let mut controller = controller();
        for (sequence, sent_at_ms) in [(1, 0), (2, u128::MAX)] {
            let sample = PlantState::initial().to_telemetry(sequence, sent_at_ms);
            assert!(controller.on_telemetry(&sample, t0).is_ok());
            assert_eq!(controller.tick(t0 + timeout() - ONE_MS), None);
            assert_eq!(controller.state(), ControlState::Autonomous);
        }
    }

    #[test]
    fn recovery_requires_consecutive_fresh_packets() {
        let t0 = Instant::now();
        let mut controller = safe_stopped(DEFAULT_RECOVERY_PACKETS, t0);
        let gap = timeout() - ONE_MS;
        let first = t0 + timeout() * 2;
        for index in 0..DEFAULT_RECOVERY_PACKETS {
            let now = first + gap * index;
            let update = controller
                .on_telemetry(&telemetry(u64::from(index) + 2), now)
                .expect("accepted");
            assert_eq!(update.timeout_command, None);
            if index + 1 < DEFAULT_RECOVERY_PACKETS {
                assert_eq!(update.command, None, "packet {index}");
                assert_eq!(controller.state(), ControlState::SafeStop);
                assert_eq!(controller.recovery_streak(), index + 1);
            } else {
                let command = update.command.expect("recovered");
                assert_eq!(command.mode(), CommandMode::Autonomous);
                assert_eq!(controller.state(), ControlState::Autonomous);
                assert_eq!(controller.recovery_streak(), 0);
            }
        }
    }

    #[test]
    fn stale_gap_resets_the_recovery_streak() {
        let t0 = Instant::now();
        let mut controller = safe_stopped(DEFAULT_RECOVERY_PACKETS, t0);
        let t1 = t0 + timeout() * 2;
        controller
            .on_telemetry(&telemetry(2), t1)
            .expect("accepted");
        controller
            .on_telemetry(&telemetry(3), t1 + timeout() - ONE_MS)
            .expect("accepted");
        assert_eq!(controller.recovery_streak(), 2);
        controller
            .on_telemetry(&telemetry(4), t1 + timeout() * 3)
            .expect("accepted");
        assert_eq!(controller.recovery_streak(), 1);
        assert_eq!(controller.state(), ControlState::SafeStop);
    }

    #[test]
    fn persistent_loss_enters_safe_stop_once_without_chattering() {
        // Accepted packets every 1000 ms against a 750 ms timeout, with the
        // node's 50 ms poll ticks in between: matches the packet-loss demo.
        let t0 = Instant::now();
        let mut controller = controller();
        let packet_gap = Duration::from_millis(1000);
        let poll = Duration::from_millis(DEFAULT_POLL_INTERVAL_MS);
        let mut commands = Vec::new();
        for index in 0..10_u32 {
            let arrival = t0 + packet_gap * index;
            let update = controller
                .on_telemetry(&telemetry(u64::from(index) + 1), arrival)
                .expect("accepted");
            commands.extend(update.timeout_command);
            commands.extend(update.command);
            for tick in 1..20 {
                commands.extend(controller.tick(arrival + poll * tick));
            }
        }
        let safe_stops = commands
            .iter()
            .filter(|c| c.mode() == CommandMode::SafeStop)
            .count();
        let autonomous = commands.len() - safe_stops;
        assert_eq!(safe_stops, 1, "{commands:?}");
        assert_eq!(autonomous, 1, "{commands:?}");
        assert_eq!(controller.state(), ControlState::SafeStop);
    }

    #[test]
    fn one_recovery_packet_recovers_immediately() {
        let t0 = Instant::now();
        let mut controller = safe_stopped(1, t0);
        let update = controller
            .on_telemetry(&telemetry(2), t0 + timeout() * 2)
            .expect("accepted");
        assert_eq!(controller.state(), ControlState::Autonomous);
        assert!(update.command.is_some());
    }

    #[test]
    fn zero_recovery_packets_behaves_as_one() {
        assert_eq!(controller_with(0).recovery_packets(), 1);
    }

    #[test]
    fn every_command_is_bounded_and_sequenced() {
        let t0 = Instant::now();
        let mut controller = controller_with(1);
        let mut commands = Vec::new();
        commands.extend(
            controller
                .on_telemetry(&telemetry(1), t0)
                .expect("accepted")
                .command,
        );
        commands.extend(controller.tick(t0 + timeout()));
        commands.extend(
            controller
                .on_telemetry(&telemetry(2), t0 + timeout() * 2)
                .expect("accepted")
                .command,
        );

        assert_eq!(commands.len(), 3);
        assert_eq!(commands[0].reason().as_str(), "valid_telemetry");
        assert_eq!(commands[1].mode(), CommandMode::SafeStop);
        for (index, command) in commands.iter().enumerate() {
            assert!(command.is_within_bounds(), "{command:?}");
            assert_eq!(command.sequence(), index as u64 + 1);
        }
    }
}
