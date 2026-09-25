//! Forwards UDP datagrams unchanged, deterministically dropping every Nth
//! packet and applying a fixed delay to the rest.

use std::net::{SocketAddr, UdpSocket};
use std::process::ExitCode;
use std::thread;
use std::time::Duration;

use clap::Parser;
use protocol::{DEFAULT_CONTROL_ADDR, DEFAULT_LINK_ADDR, MAX_DATAGRAM_BYTES};
use tracing::{error, info, warn};

#[derive(Debug, Parser)]
#[command(about = "Deterministic lossy UDP forwarder (learning prototype)")]
struct Args {
    /// Address to receive datagrams on.
    #[arg(long, default_value = DEFAULT_LINK_ADDR)]
    listen: SocketAddr,
    /// Address to forward datagrams to.
    #[arg(long, default_value = DEFAULT_CONTROL_ADDR)]
    forward: SocketAddr,
    /// Drop every Nth received packet. 0 disables dropping.
    #[arg(long, default_value_t = 0)]
    drop_every: u64,
    /// Fixed delay in milliseconds before forwarding each kept packet.
    #[arg(long, default_value_t = 0)]
    delay_ms: u64,
}

fn main() -> ExitCode {
    tracing_subscriber::fmt::init();
    let args = Args::parse();
    match run(&args) {
        Ok(()) => ExitCode::SUCCESS,
        Err(err) => {
            error!(error = %err, "link-sim failed");
            ExitCode::FAILURE
        }
    }
}

fn run(args: &Args) -> std::io::Result<()> {
    let socket = UdpSocket::bind(args.listen)?;
    let delay = Duration::from_millis(args.delay_ms);
    let mut buf = vec![0_u8; MAX_DATAGRAM_BYTES];
    let mut packet_count: u64 = 0;
    info!(
        listen = %args.listen,
        forward = %args.forward,
        drop_every = args.drop_every,
        delay_ms = args.delay_ms,
        "link-sim started"
    );

    loop {
        let (len, from) = match socket.recv_from(&mut buf) {
            Ok(received) => received,
            Err(err) => {
                warn!(error = %err, "receive failed");
                continue;
            }
        };
        packet_count = packet_count.saturating_add(1);

        if should_drop(packet_count, args.drop_every) {
            info!(packet_count, bytes = len, %from, "dropped packet");
            continue;
        }
        if !delay.is_zero() {
            thread::sleep(delay);
        }
        match socket.send_to(&buf[..len], args.forward) {
            Ok(_) => info!(
                packet_count,
                bytes = len,
                delay_ms = args.delay_ms,
                "forwarded packet"
            ),
            Err(err) => warn!(packet_count, error = %err, "forward failed"),
        }
    }
}

/// Packet counts start at 1. With `drop_every = N > 0`, packets N, 2N, 3N...
/// are dropped.
fn should_drop(packet_count: u64, drop_every: u64) -> bool {
    drop_every > 0 && packet_count.is_multiple_of(drop_every)
}

#[cfg(test)]
mod tests {
    use super::should_drop;

    fn dropped(drop_every: u64, packets: u64) -> Vec<u64> {
        (1..=packets)
            .filter(|&count| should_drop(count, drop_every))
            .collect()
    }

    #[test]
    fn zero_disables_dropping() {
        assert!(dropped(0, 20).is_empty());
    }

    #[test]
    fn one_drops_everything() {
        assert_eq!(dropped(1, 5), vec![1, 2, 3, 4, 5]);
    }

    #[test]
    fn drops_every_nth_packet_deterministically() {
        assert_eq!(dropped(3, 10), vec![3, 6, 9]);
        assert_eq!(dropped(3, 10), dropped(3, 10));
    }
}
