# Resilient Telemetry Prototype

## Purpose

A small Rust learning prototype that shows how a receiver can defend itself
against an unreliable link:

```text
sensor-node → link-sim → control-node
```

`sensor-node` sends versioned JSON telemetry over UDP. `link-sim` forwards the
datagrams unchanged, but it can drop every Nth one and add a fixed delay.
`control-node` validates each message, rejects duplicate and out-of-order
sequences, and tracks freshness with its own monotonic clock. When accepted
telemetry stops arriving, it enters an explicit `SafeStop` state.

See [DESIGN.md](DESIGN.md) for the rules and how they are tested.

## Limitations

This is a learning prototype. It is not production software, vessel software,
embedded software, a navigation system, real-time software,  or safety-certified
software, and it is not suitable for controlling physical hardware. Commands are only logged. They are never sent anywhere.

## Requirements

- Linux or macOS. Networking uses only `std::net::UdpSocket`.
- Rust stable toolchain with `cargo`, `rustfmt`, and `clippy`.
- UDP ports 9100 and 9200 free on `127.0.0.1` for the demos.

## Build and test

```bash
cargo build --workspace
cargo fmt --all -- --check
cargo clippy --workspace --all-targets -- -D warnings
cargo test --workspace
```

## Run the normal demo

Run `cargo build --workspace` first. Then use three terminals from the
repository root and start them in this order:

```bash
cargo run -q -p control-node
```

```bash
cargo run -q -p link-sim
```

```bash
cargo run -q -p sensor-node -- --count 20
```

When the sensor finishes, stop `control-node` and `link-sim` with Ctrl-C.

## Run the packet-loss demo

Stop every process from the previous demo first. `control-node` remembers the
last sequence it accepted, and `link-sim` remembers its packet count.

```bash
cargo run -q -p control-node
```

```bash
cargo run -q -p link-sim -- --drop-every 2 --delay-ms 20
```

```bash
cargo run -q -p sensor-node -- --interval-ms 500 --count 10
```

## What to look for

Normal demo, in `control-node`:

- `state transition from=WaitingForTelemetry to=Autonomous` on sequence 1.
- An `accepted telemetry` line and an `Autonomous` command
  (`rudder=0.0 throttle=0.25 reason=valid_telemetry`) for each of the 20 packets.
- About 750 ms after the last packet, a single
  `state transition from=Autonomous to=SafeStop`, followed by one `SafeStop` command
  (`rudder=0.0 throttle=0.0 reason=telemetry_timeout`). No further SafeStop lines
  follow while the node stays idle.

Packet-loss demo:

- `link-sim` logs `dropped packet` for packets 2, 4, 6, 8, and 10, and
  `forwarded packet` for the others.
- `control-node` accepts sequences 1, 3, 5, 7, and 9. Each dropped packet leaves
  a gap of about 1000 ms, which exceeds the 750 ms timeout. After each
  accepted packet, the node enters `SafeStop` once, then returns to
  `Autonomous` when the next accepted packet arrives.
- The gaps in sequence numbers are accepted, because only duplicates and
  decreases are rejected.
