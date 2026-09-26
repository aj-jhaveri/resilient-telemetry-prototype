//! Shared protocol and control logic for the Resilient Telemetry Prototype.
//!
//! This is a learning prototype. It is not production, real-time, or
//! safety-certified software and must not be used to control hardware.

mod command;
mod control;
mod p_control;
mod plant;
mod sequence;
mod telemetry;

pub use command::{
    Applied, CommandDecodeError, CommandMessage, CommandReceiver, CommandRejection,
    CommandValidationError,
};
pub use control::{
    Command, CommandMode, CommandReason, ControlState, Controller, ControllerConfig, Rejection,
    Update,
};
pub use p_control::{p_control, Setpoint, SetpointError};
pub use plant::{heading_error_deg, wrap_heading_deg, Actuation, PlantState};
pub use sequence::{SequenceError, SequenceGate};
pub use telemetry::{DecodeError, Telemetry, ValidationError};

/// The only telemetry message version this build accepts.
pub const SUPPORTED_VERSION: u8 = 1;

/// Inclusive latitude bounds in degrees.
pub const LATITUDE_MIN_DEG: f64 = -90.0;
pub const LATITUDE_MAX_DEG: f64 = 90.0;
/// Inclusive longitude bounds in degrees.
pub const LONGITUDE_MIN_DEG: f64 = -180.0;
pub const LONGITUDE_MAX_DEG: f64 = 180.0;
/// Heading bounds in degrees: minimum inclusive, maximum exclusive.
pub const HEADING_MIN_DEG: f64 = 0.0;
pub const HEADING_MAX_DEG: f64 = 360.0;
/// Inclusive speed bounds in metres per second.
pub const SPEED_MIN_MPS: f64 = 0.0;
pub const SPEED_MAX_MPS: f64 = 20.0;
/// Inclusive battery bounds in percent.
pub const BATTERY_MIN_PERCENT: f32 = 0.0;
pub const BATTERY_MAX_PERCENT: f32 = 100.0;

/// Inclusive rudder bounds for any emitted command.
pub const RUDDER_MIN: f64 = -1.0;
pub const RUDDER_MAX: f64 = 1.0;
/// Inclusive throttle bounds for any emitted command.
pub const THROTTLE_MIN: f64 = 0.0;
pub const THROTTLE_MAX: f64 = 1.0;
/// Fixed values used by a `SafeStop` command.
pub const SAFE_STOP_RUDDER: f64 = 0.0;
pub const SAFE_STOP_THROTTLE: f64 = 0.0;

/// Default freshness timeout used by `control-node`.
pub const DEFAULT_TIMEOUT_MS: u64 = 750;
/// Consecutive fresh packets `control-node` needs to leave SafeStop.
pub const DEFAULT_RECOVERY_PACKETS: u32 = 3;
/// The only command message version this build sends or accepts.
pub const COMMAND_VERSION: u8 = 1;
/// Default command watchdog used by `sensor-node`. It is longer than
/// `DEFAULT_TIMEOUT_MS` because commands are sent only when telemetry is
/// accepted, so healthy command gaps can approach the telemetry timeout.
pub const DEFAULT_COMMAND_TIMEOUT_MS: u64 = 1000;
/// Address `link-sim` listens on and `sensor-node` sends to by default.
pub const DEFAULT_LINK_ADDR: &str = "127.0.0.1:9100";
/// Address `control-node` binds and `link-sim` forwards to by default.
pub const DEFAULT_CONTROL_ADDR: &str = "127.0.0.1:9200";
/// Receive buffer size; large enough for any UDP payload.
pub const MAX_DATAGRAM_BYTES: usize = 65_536;

// Simulated vessel plant. Illustrative values only; they do not model any
// real vessel.

/// Plant speed approached at full throttle. Never above `SPEED_MAX_MPS`.
pub const PLANT_MAX_SPEED_MPS: f64 = 5.0;
/// Time constant of the first-order speed response.
pub const SPEED_TIME_CONSTANT_S: f64 = 2.0;
/// Heading rate at full rudder; positive rudder turns clockwise.
pub const MAX_TURN_RATE_DEG_S: f64 = 10.0;
/// Battery drain at full throttle.
pub const BATTERY_DRAIN_PERCENT_PER_S: f64 = 0.05;
/// Flat-earth conversion used for position updates.
pub const METERS_PER_DEG_LAT: f64 = 111_320.0;
/// Initial plant state: Austin-like position, stopped, pointing north.
pub const INITIAL_LATITUDE_DEG: f64 = 30.2672;
pub const INITIAL_LONGITUDE_DEG: f64 = -97.7431;
pub const INITIAL_HEADING_DEG: f64 = 0.0;
pub const INITIAL_SPEED_MPS: f64 = 0.0;
pub const INITIAL_BATTERY_PERCENT: f32 = 87.5;

// Proportional control. Illustrative gains only.

/// Rudder per degree of heading error (full rudder at 30 degrees).
pub const HEADING_KP_PER_DEG: f64 = 1.0 / 30.0;
/// Throttle per m/s of speed error, added to the feedforward term.
pub const SPEED_KP_PER_MPS: f64 = 0.5;
/// Default setpoint used by `control-node`.
pub const DEFAULT_TARGET_HEADING_DEG: f64 = 90.0;
pub const DEFAULT_TARGET_SPEED_MPS: f64 = 2.0;
