//! Receives telemetry, validates it, tracks freshness, enters SafeStop when
//! telemetry stops arriving, and sends each command to `sensor-node`.

use std::io::ErrorKind;
use std::net::{SocketAddr, UdpSocket};
use std::process::ExitCode;
use std::time::{Duration, Instant};

use clap::Parser;
use protocol::{
    Command, CommandMode, ControlState, Controller, ControllerConfig, Setpoint,
    DEFAULT_CONTROL_ADDR, DEFAULT_POLL_INTERVAL_MS, DEFAULT_RECOVERY_PACKETS, DEFAULT_SENSOR_ADDR,
    DEFAULT_TARGET_HEADING_DEG, DEFAULT_TARGET_SPEED_MPS, DEFAULT_TIMEOUT_MS, MAX_DATAGRAM_BYTES,
};
use tracing::{error, info, warn};

/// Socket read timeout; bounds how late a freshness check can run.
const POLL_INTERVAL: Duration = Duration::from_millis(DEFAULT_POLL_INTERVAL_MS);

#[derive(Debug, Parser)]
#[command(
    about = "Validate telemetry, steer a simulated plant, SafeStop on timeout (learning prototype)"
)]
struct Args {
    /// Address to receive telemetry on.
    #[arg(long, default_value = DEFAULT_CONTROL_ADDR)]
    bind: SocketAddr,
    /// Milliseconds without accepted telemetry before SafeStop.
    #[arg(long, default_value_t = DEFAULT_TIMEOUT_MS, value_parser = clap::value_parser!(u64).range(1..))]
    timeout_ms: u64,
    /// Address of sensor-node's command socket.
    #[arg(long, default_value = DEFAULT_SENSOR_ADDR)]
    command_destination: SocketAddr,
    /// Target heading in degrees, in [0, 360).
    #[arg(long, default_value_t = DEFAULT_TARGET_HEADING_DEG)]
    target_heading_deg: f64,
    /// Target speed in m/s, from 0 to the plant maximum.
    #[arg(long, default_value_t = DEFAULT_TARGET_SPEED_MPS)]
    target_speed_mps: f64,
    /// Consecutive fresh packets needed to leave SafeStop.
    #[arg(long, default_value_t = DEFAULT_RECOVERY_PACKETS, value_parser = clap::value_parser!(u32).range(1..))]
    recovery_packets: u32,
}

fn main() -> ExitCode {
    tracing_subscriber::fmt::init();
    let args = Args::parse();
    match run(&args) {
        Ok(()) => ExitCode::SUCCESS,
        Err(err) => {
            error!(error = %err, "control-node failed");
            ExitCode::FAILURE
        }
    }
}

fn run(args: &Args) -> Result<(), Box<dyn std::error::Error>> {
    // clap accepts "NaN" for f64 flags, so the setpoint is validated here.
    let config = ControllerConfig {
        timeout: Duration::from_millis(args.timeout_ms),
        recovery_packets: args.recovery_packets,
        setpoint: Setpoint::new(args.target_heading_deg, args.target_speed_mps)?,
    };
    let socket = UdpSocket::bind(args.bind)?;
    socket.set_read_timeout(Some(POLL_INTERVAL))?;
    let mut controller = Controller::new(config);
    let mut buf = vec![0_u8; MAX_DATAGRAM_BYTES];
    info!(
        bind = %args.bind,
        command_destination = %args.command_destination,
        timeout_ms = args.timeout_ms,
        target_heading_deg = args.target_heading_deg,
        target_speed_mps = args.target_speed_mps,
        recovery_packets = args.recovery_packets,
        state = ?controller.state(),
        "control-node started"
    );

    loop {
        poll_once(&socket, &mut controller, &mut buf, args.command_destination);
    }
}

/// Waits up to `POLL_INTERVAL` for one datagram, handles it, then evaluates
/// freshness. Every command is logged and sent to `command_destination`.
/// All expected failures are logged and never stop the node.
fn poll_once(
    socket: &UdpSocket,
    controller: &mut Controller,
    buf: &mut [u8],
    command_destination: SocketAddr,
) {
    match socket.recv_from(buf) {
        Ok((len, from)) => {
            let mut before = controller.state();
            match controller.handle_datagram(&buf[..len], Instant::now()) {
                Ok(update) => {
                    if let Some(command) = &update.timeout_command {
                        log_transition(before, ControlState::SafeStop);
                        emit(socket, command_destination, command);
                        before = ControlState::SafeStop;
                    }
                    info!(
                        sequence = update.telemetry.sequence,
                        heading_deg = update.telemetry.heading_deg,
                        speed_mps = update.telemetry.speed_mps,
                        sent_at_ms = update.telemetry.sent_at_ms,
                        %from,
                        "accepted telemetry"
                    );
                    log_transition(before, controller.state());
                    match &update.command {
                        Some(command) => emit(socket, command_destination, command),
                        None => info!(
                            streak = controller.recovery_streak(),
                            needed = controller.recovery_packets(),
                            "recovery in progress; staying in SafeStop"
                        ),
                    }
                }
                Err(rejection) => {
                    warn!(%from, bytes = len, reason = %rejection, "ignored datagram")
                }
            }
        }
        Err(err) if matches!(err.kind(), ErrorKind::WouldBlock | ErrorKind::TimedOut) => {}
        Err(err) => warn!(error = %err, "receive failed"),
    }

    let before = controller.state();
    if let Some(command) = controller.tick(Instant::now()) {
        log_transition(before, controller.state());
        emit(socket, command_destination, &command);
    }
}

fn log_transition(from: ControlState, to: ControlState) {
    if from != to {
        info!(from = ?from, to = ?to, "state transition");
    }
}

/// Logs a command and sends it as a `CommandMessage` datagram.
fn emit(socket: &UdpSocket, destination: SocketAddr, command: &Command) {
    match command.mode() {
        CommandMode::Autonomous => info!(
            sequence = command.sequence(),
            mode = ?command.mode(),
            rudder = command.rudder(),
            throttle = command.throttle(),
            reason = %command.reason(),
            "command"
        ),
        CommandMode::SafeStop => warn!(
            sequence = command.sequence(),
            mode = ?command.mode(),
            rudder = command.rudder(),
            throttle = command.throttle(),
            reason = %command.reason(),
            "command"
        ),
    }
    match command.to_message().encode() {
        Ok(bytes) => {
            if let Err(err) = socket.send_to(&bytes, destination) {
                warn!(%destination, error = %err, "command send failed");
            }
        }
        Err(err) => warn!(error = %err, "command encode failed"),
    }
}

#[cfg(test)]
#[allow(clippy::unwrap_used, clippy::expect_used)]
mod tests {
    use super::*;
    use protocol::{CommandMessage, PlantState};

    fn test_controller(timeout: Duration) -> Controller {
        Controller::new(ControllerConfig {
            timeout,
            recovery_packets: DEFAULT_RECOVERY_PACKETS,
            setpoint: Setpoint::new(DEFAULT_TARGET_HEADING_DEG, DEFAULT_TARGET_SPEED_MPS)
                .expect("setpoint"),
        })
    }

    fn bound_socket() -> UdpSocket {
        let socket = UdpSocket::bind("127.0.0.1:0").expect("bind");
        socket
            .set_read_timeout(Some(POLL_INTERVAL))
            .expect("read timeout");
        socket
    }

    fn receive_command(socket: &UdpSocket) -> CommandMessage {
        let mut buf = vec![0_u8; MAX_DATAGRAM_BYTES];
        let (len, _) = socket.recv_from(&mut buf).expect("command datagram");
        CommandMessage::decode(&buf[..len]).expect("valid command")
    }

    #[test]
    fn valid_telemetry_over_udp_sends_an_autonomous_command() {
        let (node, sensor) = (bound_socket(), bound_socket());
        let node_addr = node.local_addr().expect("node addr");
        let sensor_addr = sensor.local_addr().expect("sensor addr");
        let mut controller = test_controller(Duration::from_millis(DEFAULT_TIMEOUT_MS));
        let mut buf = vec![0_u8; MAX_DATAGRAM_BYTES];

        let bytes = PlantState::initial()
            .to_telemetry(1, 0)
            .encode()
            .expect("encode");
        sensor.send_to(&bytes, node_addr).expect("send");
        poll_once(&node, &mut controller, &mut buf, sensor_addr);

        assert_eq!(controller.state(), ControlState::Autonomous);
        let command = receive_command(&sensor);
        assert_eq!(command.mode, CommandMode::Autonomous);
        assert_eq!(command.sequence, 1);
    }

    #[test]
    fn telemetry_timeout_sends_one_safe_stop_command() {
        let (node, sensor) = (bound_socket(), bound_socket());
        let node_addr = node.local_addr().expect("node addr");
        let sensor_addr = sensor.local_addr().expect("sensor addr");
        // A 1 ms timeout expires during the next 50 ms poll, so no sleeping
        // beyond the socket read timeout is needed.
        let mut controller = test_controller(Duration::from_millis(1));
        let mut buf = vec![0_u8; MAX_DATAGRAM_BYTES];

        let bytes = PlantState::initial()
            .to_telemetry(1, 0)
            .encode()
            .expect("encode");
        sensor.send_to(&bytes, node_addr).expect("send");
        poll_once(&node, &mut controller, &mut buf, sensor_addr);
        let _autonomous = receive_command(&sensor);
        poll_once(&node, &mut controller, &mut buf, sensor_addr);

        assert_eq!(controller.state(), ControlState::SafeStop);
        let safe_stop = receive_command(&sensor);
        assert_eq!(safe_stop.mode, CommandMode::SafeStop);

        poll_once(&node, &mut controller, &mut buf, sensor_addr);
        let mut probe = [0_u8; 1];
        assert!(
            sensor.recv_from(&mut probe).is_err(),
            "no repeated SafeStop"
        );
    }

    #[test]
    fn bad_datagram_and_read_timeout_leave_node_waiting() {
        let (node, sensor) = (bound_socket(), bound_socket());
        let node_addr = node.local_addr().expect("node addr");
        let sensor_addr = sensor.local_addr().expect("sensor addr");
        let mut controller = test_controller(Duration::from_millis(DEFAULT_TIMEOUT_MS));
        let mut buf = vec![0_u8; MAX_DATAGRAM_BYTES];

        sensor.send_to(b"not telemetry", node_addr).expect("send");
        poll_once(&node, &mut controller, &mut buf, sensor_addr);
        poll_once(&node, &mut controller, &mut buf, sensor_addr);

        assert_eq!(controller.state(), ControlState::WaitingForTelemetry);
        assert_eq!(controller.last_accepted_sequence(), None);
        let mut probe = [0_u8; 1];
        assert!(sensor.recv_from(&mut probe).is_err(), "no command sent");
    }
}
