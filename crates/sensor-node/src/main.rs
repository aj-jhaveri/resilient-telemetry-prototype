//! Sends simulated JSON telemetry over UDP.

use std::net::{SocketAddr, UdpSocket};
use std::process::ExitCode;
use std::thread;
use std::time::{Duration, SystemTime, UNIX_EPOCH};

use clap::Parser;
use protocol::{Telemetry, DEFAULT_LINK_ADDR};
use tracing::{error, info, warn};

#[derive(Debug, Parser)]
#[command(about = "Send simulated telemetry over UDP (learning prototype)")]
struct Args {
    /// Address to send telemetry to.
    #[arg(long, default_value = DEFAULT_LINK_ADDR)]
    destination: SocketAddr,
    /// Milliseconds between packets.
    #[arg(long, default_value_t = 100, value_parser = clap::value_parser!(u64).range(1..))]
    interval_ms: u64,
    /// Stop after this many packets. Runs until interrupted if omitted.
    #[arg(long)]
    count: Option<u64>,
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
    let bind_addr = if args.destination.is_ipv4() {
        "0.0.0.0:0"
    } else {
        "[::]:0"
    };
    let socket = UdpSocket::bind(bind_addr)?;
    let interval = Duration::from_millis(args.interval_ms);
    info!(destination = %args.destination, interval_ms = args.interval_ms, count = ?args.count, "sensor-node started");

    for sequence in 1..=args.count.unwrap_or(u64::MAX) {
        let telemetry = Telemetry::simulated(sequence, unix_time_ms());
        match telemetry.encode() {
            Ok(bytes) => match socket.send_to(&bytes, args.destination) {
                Ok(len) => info!(sequence, bytes = len, "sent telemetry"),
                Err(err) => warn!(sequence, error = %err, "send failed"),
            },
            Err(err) => warn!(sequence, error = %err, "encode failed"),
        }
        thread::sleep(interval);
    }
    info!("sensor-node finished");
    Ok(())
}

/// Current Unix time in milliseconds; informational only.
fn unix_time_ms() -> u128 {
    SystemTime::now()
        .duration_since(UNIX_EPOCH)
        .map_or(0, |elapsed| elapsed.as_millis())
}
