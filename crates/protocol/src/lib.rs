//! Shared protocol and control logic for the Resilient Telemetry Prototype.
//!
//! This is a learning prototype. It is not production, real-time, or
//! safety-certified software and must not be used to control hardware.

mod control;
mod telemetry;

pub use control::{Command, CommandMode, CommandReason, ControlState, Controller, Rejection};
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
/// Inclusive minimum speed in metres per second.
pub const SPEED_MIN_MPS: f64 = 0.0;
/// Inclusive battery bounds in percent.
pub const BATTERY_MIN_PERCENT: f32 = 0.0;
pub const BATTERY_MAX_PERCENT: f32 = 100.0;

/// Inclusive rudder bounds for any emitted command.
pub const RUDDER_MIN: f64 = -1.0;
pub const RUDDER_MAX: f64 = 1.0;
/// Inclusive throttle bounds for any emitted command.
pub const THROTTLE_MIN: f64 = 0.0;
pub const THROTTLE_MAX: f64 = 1.0;
/// Fixed values used by an `Autonomous` command.
pub const AUTONOMOUS_RUDDER: f64 = 0.0;
pub const AUTONOMOUS_THROTTLE: f64 = 0.25;
/// Fixed values used by a `SafeStop` command.
pub const SAFE_STOP_RUDDER: f64 = 0.0;
pub const SAFE_STOP_THROTTLE: f64 = 0.0;

/// Default freshness timeout used by `control-node`.
pub const DEFAULT_TIMEOUT_MS: u64 = 750;
/// Address `link-sim` listens on and `sensor-node` sends to by default.
pub const DEFAULT_LINK_ADDR: &str = "127.0.0.1:9100";
/// Address `control-node` binds and `link-sim` forwards to by default.
pub const DEFAULT_CONTROL_ADDR: &str = "127.0.0.1:9200";
/// Receive buffer size; large enough for any UDP payload.
pub const MAX_DATAGRAM_BYTES: usize = 65_536;
