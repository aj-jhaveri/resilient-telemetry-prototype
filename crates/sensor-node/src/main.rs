//! Simulated vessel plant and sensors: steps the plant, sends its telemetry
//! over UDP, and applies commands received from `control-node`, falling back
//! to zero rudder and throttle when commands stop arriving.

use std::io::ErrorKind;
use std::net::{SocketAddr, UdpSocket};
use std::process::ExitCode;
use std::thread;
use std::time::{Duration, Instant, SystemTime, UNIX_EPOCH};

use clap::Parser;
use protocol::{
    CommandReceiver, PlantState, DEFAULT_COMMAND_TIMEOUT_MS, DEFAULT_LINK_ADDR,
    DEFAULT_SENSOR_ADDR, MAX_DATAGRAM_BYTES,
};
use tracing::{error, info, warn};

#[derive(Debug, Parser)]
#[command(about = "Simulated vessel plant: send telemetry, apply commands (learning prototype)")]
struct Args {
    /// Address to send telemetry to.
    #[arg(long, default_value = DEFAULT_LINK_ADDR)]
    destination: SocketAddr,
    /// Milliseconds between packets; also the plant's simulation step.
    #[arg(long, default_value_t = 100, value_parser = clap::value_parser!(u64).range(1..))]
    interval_ms: u64,
    /// Stop after this many packets. Runs until interrupted if omitted.
    #[arg(long)]
    count: Option<u64>,
    /// Address to send telemetry from and receive commands on.
    #[arg(long, default_value = DEFAULT_SENSOR_ADDR)]
    bind: SocketAddr,
    /// Milliseconds without a valid command before zero rudder and throttle.
    #[arg(long, default_value_t = DEFAULT_COMMAND_TIMEOUT_MS, value_parser = clap::value_parser!(u64).range(1..))]
    command_timeout_ms: u64,
}

fn main() -> ExitCode {
    tracing_subscriber::fmt::init();
    let args = Args::parse();
    match run(&args) {
        Ok(()) => ExitCode::SUCCESS,
        Err(err) => {
            error!(error = %err, "sensor-node failed");
            ExitCode::FAILURE
        }
    }
}

fn run(args: &Args) -> std::io::Result<()> {
    let socket = UdpSocket::bind(args.bind)?;
    let interval = Duration::from_millis(args.interval_ms);
    let mut plant = PlantState::initial();
    let mut receiver = CommandReceiver::new(Duration::from_millis(args.command_timeout_ms));
    let mut buf = vec![0_u8; MAX_DATAGRAM_BYTES];
    info!(
        bind = %args.bind,
        destination = %args.destination,
        interval_ms = args.interval_ms,
        command_timeout_ms = args.command_timeout_ms,
        count = ?args.count,
        "sensor-node started"
    );

    let mut next_send = Instant::now();
    for sequence in 1..=args.count.unwrap_or(u64::MAX) {
        receive_commands_until(&socket, &mut receiver, &mut buf, next_send);
        step_and_send(
            &socket,
            args.destination,
            &mut plant,
            &mut receiver,
            interval,
            sequence,
        );
        // Fixed-rate schedule; if the loop fell behind, restart from now
        // instead of sending a burst.
        next_send += interval;
        next_send = next_send.max(Instant::now());
    }
    info!("sensor-node finished");
    Ok(())
}

/// Receives and applies command datagrams until `deadline`. All expected
/// failures are logged and never stop the node.
fn receive_commands_until(
    socket: &UdpSocket,
    receiver: &mut CommandReceiver,
    buf: &mut [u8],
    deadline: Instant,
) {
    loop {
        let remaining = deadline.saturating_duration_since(Instant::now());
        // A zero read timeout is an error, so stop before reaching zero.
        if remaining.is_zero() {
            return;
        }
        if let Err(err) = socket.set_read_timeout(Some(remaining)) {
            warn!(error = %err, "set read timeout failed");
            thread::sleep(remaining);
            return;
        }
        match socket.recv_from(buf) {
            Ok((len, from)) => match receiver.handle_datagram(&buf[..len], Instant::now()) {
                Ok(message) => info!(
                    sequence = message.sequence,
                    mode = ?message.mode,
                    rudder = message.rudder,
                    throttle = message.throttle,
                    "accepted command"
                ),
                Err(rejection) => {
                    warn!(%from, bytes = len, reason = %rejection, "ignored datagram")
                }
            },
            Err(err) if matches!(err.kind(), ErrorKind::WouldBlock | ErrorKind::TimedOut) => {}
            Err(err) => {
                // Wait out the interval instead of spinning on a failing socket.
                warn!(error = %err, "receive failed");
                thread::sleep(deadline.saturating_duration_since(Instant::now()));
                return;
            }
        }
    }
}

/// Advances the plant by one step with the currently applied command and
/// sends the resulting telemetry.
fn step_and_send(
    socket: &UdpSocket,
    destination: SocketAddr,
    plant: &mut PlantState,
    receiver: &mut CommandReceiver,
    dt: Duration,
    sequence: u64,
) {
    let applied = receiver.applied(Instant::now());
    if applied.watchdog_tripped {
        warn!("command watchdog expired; applying zero rudder and throttle");
    }
    plant.step(applied.actuation, dt);
    let telemetry = plant.to_telemetry(sequence, unix_time_ms());
    match telemetry.encode() {
        Ok(bytes) => match socket.send_to(&bytes, destination) {
            Ok(_) => info!(
                sequence,
                heading_deg = plant.heading_deg,
                speed_mps = plant.speed_mps,
                battery_percent = plant.battery_percent,
                rudder = applied.actuation.rudder(),
                throttle = applied.actuation.throttle(),
                "sent telemetry"
            ),
            Err(err) => warn!(sequence, error = %err, "send failed"),
        },
        Err(err) => warn!(sequence, error = %err, "encode failed"),
    }
}

/// Current Unix time in milliseconds; informational only.
fn unix_time_ms() -> u128 {
    SystemTime::now()
        .duration_since(UNIX_EPOCH)
        .map_or(0, |elapsed| elapsed.as_millis())
}

#[cfg(test)]
#[allow(clippy::unwrap_used, clippy::expect_used)]
mod tests {
    use super::*;
    use protocol::{
        Actuation, CommandMessage, CommandMode, CommandReason, Telemetry, COMMAND_VERSION,
        THROTTLE_MAX,
    };

    const SHORT_WAIT: Duration = Duration::from_millis(100);

    fn local_socket() -> UdpSocket {
        let socket = UdpSocket::bind("127.0.0.1:0").expect("bind");
        socket.set_read_timeout(Some(SHORT_WAIT)).expect("timeout");
        socket
    }

    #[test]
    fn command_datagram_is_applied_by_the_receiver() {
        let (sensor, controller) = (local_socket(), local_socket());
        let mut receiver = CommandReceiver::new(Duration::from_millis(DEFAULT_COMMAND_TIMEOUT_MS));
        let mut buf = vec![0_u8; MAX_DATAGRAM_BYTES];
        let message = CommandMessage {
            version: COMMAND_VERSION,
            sequence: 1,
            mode: CommandMode::Autonomous,
            rudder: 0.5,
            throttle: 0.25,
            reason: CommandReason::ValidTelemetry,
        };
        let sensor_addr = sensor.local_addr().expect("addr");
        controller
            .send_to(&message.encode().expect("encode"), sensor_addr)
            .expect("send");

        receive_commands_until(
            &sensor,
            &mut receiver,
            &mut buf,
            Instant::now() + SHORT_WAIT,
        );

        let applied = receiver.applied(Instant::now());
        assert_eq!(applied.actuation, Actuation::clamped(0.5, 0.25));
    }

    #[test]
    fn step_sends_valid_telemetry_and_applies_zero_without_commands() {
        let (sensor, link) = (local_socket(), local_socket());
        let mut receiver = CommandReceiver::new(Duration::from_millis(DEFAULT_COMMAND_TIMEOUT_MS));
        let mut plant = PlantState::initial();
        let link_addr = link.local_addr().expect("addr");

        step_and_send(&sensor, link_addr, &mut plant, &mut receiver, SHORT_WAIT, 1);

        let mut buf = vec![0_u8; MAX_DATAGRAM_BYTES];
        let (len, _) = link.recv_from(&mut buf).expect("telemetry datagram");
        let telemetry = Telemetry::decode(&buf[..len]).expect("valid telemetry");
        assert_eq!(telemetry.sequence, 1);
        // No command yet, so the plant stayed stopped.
        assert_eq!(telemetry.speed_mps, PlantState::initial().speed_mps);
    }

    #[test]
    fn applied_command_moves_the_plant() {
        let (sensor, link) = (local_socket(), local_socket());
        let mut receiver = CommandReceiver::new(Duration::from_millis(DEFAULT_COMMAND_TIMEOUT_MS));
        let mut buf = vec![0_u8; MAX_DATAGRAM_BYTES];
        let message = CommandMessage {
            version: COMMAND_VERSION,
            sequence: 1,
            mode: CommandMode::Autonomous,
            rudder: 0.0,
            throttle: THROTTLE_MAX,
            reason: CommandReason::ValidTelemetry,
        };
        receiver
            .handle_datagram(&message.encode().expect("encode"), Instant::now())
            .expect("accepted");
        let mut plant = PlantState::initial();

        step_and_send(
            &sensor,
            link.local_addr().expect("addr"),
            &mut plant,
            &mut receiver,
            SHORT_WAIT,
            1,
        );

        let (len, _) = link.recv_from(&mut buf).expect("telemetry datagram");
        let telemetry = Telemetry::decode(&buf[..len]).expect("valid telemetry");
        assert!(telemetry.speed_mps > PlantState::initial().speed_mps);
    }
}
