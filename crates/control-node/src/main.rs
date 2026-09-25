//! Receives telemetry, validates it, tracks freshness, and enters SafeStop
//! when telemetry stops arriving.

use std::io::ErrorKind;
use std::net::{SocketAddr, UdpSocket};
use std::process::ExitCode;
use std::time::{Duration, Instant};

use clap::Parser;
use protocol::{
    Command, CommandMode, ControlState, Controller, DEFAULT_CONTROL_ADDR, DEFAULT_TIMEOUT_MS,
    MAX_DATAGRAM_BYTES,
};
use tracing::{error, info, warn};

/// Socket read timeout; bounds how late a freshness check can run.
const POLL_INTERVAL: Duration = Duration::from_millis(50);

#[derive(Debug, Parser)]
#[command(about = "Validate telemetry and enter SafeStop on timeout (learning prototype)")]
struct Args {
    /// Address to receive telemetry on.
    #[arg(long, default_value = DEFAULT_CONTROL_ADDR)]
    bind: SocketAddr,
    /// Milliseconds without accepted telemetry before SafeStop.
    #[arg(long, default_value_t = DEFAULT_TIMEOUT_MS, value_parser = clap::value_parser!(u64).range(1..))]
    timeout_ms: u64,
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

fn run(args: &Args) -> std::io::Result<()> {
    let socket = UdpSocket::bind(args.bind)?;
    socket.set_read_timeout(Some(POLL_INTERVAL))?;
    let mut controller = Controller::new(Duration::from_millis(args.timeout_ms));
    let mut buf = vec![0_u8; MAX_DATAGRAM_BYTES];
    info!(bind = %args.bind, timeout_ms = args.timeout_ms, state = ?controller.state(), "control-node started");

    loop {
        poll_once(&socket, &mut controller, &mut buf);
    }
}

/// Waits up to `POLL_INTERVAL` for one datagram, handles it, then evaluates
/// freshness. All expected failures are logged and never stop the node.
fn poll_once(socket: &UdpSocket, controller: &mut Controller, buf: &mut [u8]) {
    match socket.recv_from(buf) {
        Ok((len, from)) => {
            let before = controller.state();
            match controller.handle_datagram(&buf[..len], Instant::now()) {
                Ok((telemetry, command)) => {
                    info!(
                        sequence = telemetry.sequence,
                        sent_at_ms = telemetry.sent_at_ms,
                        %from,
                        "accepted telemetry"
                    );
                    log_transition(before, controller.state());
                    log_command(&command);
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
        log_command(&command);
    }
}

fn log_transition(from: ControlState, to: ControlState) {
    if from != to {
        info!(from = ?from, to = ?to, "state transition");
    }
}

fn log_command(command: &Command) {
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
}

#[cfg(test)]
#[allow(clippy::unwrap_used, clippy::expect_used)]
mod tests {
    use super::*;
    use protocol::Telemetry;

    fn control_socket() -> (UdpSocket, UdpSocket) {
        let receiver = UdpSocket::bind("127.0.0.1:0").expect("bind receiver");
        receiver
            .set_read_timeout(Some(POLL_INTERVAL))
            .expect("read timeout");
        let sender = UdpSocket::bind("127.0.0.1:0").expect("bind sender");
        (receiver, sender)
    }

    #[test]
    fn valid_telemetry_over_udp_becomes_autonomous() {
        let (receiver, sender) = control_socket();
        let target = receiver.local_addr().expect("local addr");
        let mut controller = Controller::new(Duration::from_millis(DEFAULT_TIMEOUT_MS));
        let mut buf = vec![0_u8; MAX_DATAGRAM_BYTES];

        let bytes = Telemetry::simulated(1, 0).encode().expect("encode");
        sender.send_to(&bytes, target).expect("send");
        poll_once(&receiver, &mut controller, &mut buf);

        assert_eq!(controller.state(), ControlState::Autonomous);
        assert_eq!(controller.last_accepted_sequence(), Some(1));
    }

    #[test]
    fn bad_datagram_and_read_timeout_leave_node_waiting() {
        let (receiver, sender) = control_socket();
        let target = receiver.local_addr().expect("local addr");
        let mut controller = Controller::new(Duration::from_millis(DEFAULT_TIMEOUT_MS));
        let mut buf = vec![0_u8; MAX_DATAGRAM_BYTES];

        sender.send_to(b"not telemetry", target).expect("send");
        poll_once(&receiver, &mut controller, &mut buf);
        poll_once(&receiver, &mut controller, &mut buf);

        assert_eq!(controller.state(), ControlState::WaitingForTelemetry);
        assert_eq!(controller.last_accepted_sequence(), None);
    }
}
