# Resilient Telemetry Prototype

## Purpose

A small Rust learning prototype. It shows how a controller can defend itself
against an unreliable telemetry link, and how a simulated vessel plant can
defend itself against losing commands. It is a closed loop of two simulated
onboard processes plus a fault injector:

```text
sensor-node ──telemetry──► link-sim ──telemetry──► control-node
     ▲                                                  │
     └──────────── command (direct, no faults) ─────────┘
```

- `sensor-node` runs a simulated vessel plant. It sends its heading, speed,
  position, and battery as versioned JSON telemetry over UDP. It applies the
  latest valid command, and zeroes rudder and throttle if commands stop
  arriving.
- `link-sim` is a deterministic fault injector on the telemetry path only. It
  forwards datagrams unchanged, but it can drop every Nth one and add a fixed
  delay.
- `control-node` validates telemetry, rejects duplicate and out-of-order
  sequences, and tracks freshness with its own monotonic clock. It steers
  toward a target heading and speed with proportional control. When accepted
  telemetry stops arriving, it enters `SafeStop`.

See [DESIGN.md](DESIGN.md) for the rules, the plant model, and the tests.

## Limitations

This is a learning prototype. It is not production software, vessel software,
embedded software, a navigation system, real-time software, safety-certified
software, or DoD-compliant software. It is not suitable for controlling
physical hardware. Commands only drive a simulated vessel plant inside
`sensor-node`, and its parameters are illustrative. The command path has no
injected faults. Restarting `control-node` restarts its command sequence at
1, so a running `sensor-node` rejects those commands until it is restarted
too. The demos below were verified on macOS only.

## Requirements

- Linux or macOS. Networking uses only `std::net::UdpSocket`. Only macOS has
  been tested.
- Rust stable toolchain with `cargo`, `rustfmt`, and `clippy`.
- UDP ports 9100, 9200, and 9300 free on `127.0.0.1` for the demos.

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
cargo run -q -p sensor-node -- --count 300
```

The sensor runs for about 30 seconds. When it finishes, stop `control-node`
and `link-sim` with Ctrl-C.

To see the command watchdog, run the demo again from a clean start. This time,
press Ctrl-C in the `control-node` terminal while `sensor-node` is still
running.

## Run the packet-loss demo

Stop every process from the previous demo first. Each process remembers
sequences or packet counts from its last run.

```bash
cargo run -q -p control-node
```

```bash
cargo run -q -p link-sim -- --drop-every 2 --delay-ms 20
```

```bash
cargo run -q -p sensor-node -- --interval-ms 500 --count 40
```

After about 8 seconds, stop `link-sim` with Ctrl-C and restart it without
faults in the same terminal:

```bash
cargo run -q -p link-sim
```

## What to look for

The timings below come from one macOS run of each demo.

Normal demo:

- `control-node` logs `state transition from=WaitingForTelemetry to=Autonomous`
  on sequence 1. It then logs `accepted telemetry` and an `Autonomous` command
  for each of the 300 packets.
- In `sensor-node`'s `sent telemetry` lines, sequence 1 applies zero because
  no command has arrived yet. From sequence 2, rudder and throttle are `1.0`
  because the errors are large. Speed was within 0.01 m/s of the 2.0 m/s
  target by sequence 31. Rudder stayed at `1.0` until heading passed 60
  degrees (sequence 62). Heading was within 1 degree of 90 by sequence 162.
  Throttle settles at `0.4`, the feedforward value for 2.0 m/s.
- After the last packet, `control-node` logs one
  `state transition from=Autonomous to=SafeStop` and one `SafeStop` command
  (`rudder=0.0 throttle=0.0 reason=telemetry_timeout`). This was 775 ms after
  the last accepted packet. No further SafeStop lines follow.

Watchdog variant: `sensor-node` logs
`command watchdog expired; applying zero rudder and throttle` once, 1103 ms
after the last command it accepted. The timeout is 1000 ms, but the watchdog
is checked only once per 100 ms step. Heading then freezes, and speed decays
exponentially: 1.90, 1.15, and 0.70 m/s at one-second intervals.

Packet-loss demo:

- `link-sim` logs `dropped packet` for every even packet count and
  `forwarded packet` for the rest.
- `control-node` accepts sequences 1, 3, 5, and so on, about 1000 ms apart.
  That gap exceeds the 750 ms timeout, so the node enters `SafeStop` once and
  stays there. Leaving SafeStop needs 3 consecutive packets less than 750 ms
  apart, so each later packet logs `recovery in progress` with `streak=1`. The
  node does not toggle between states.
- `sensor-node` applies the SafeStop command. Speed decays toward zero, and
  the command watchdog logs its expiry once.
- After `link-sim` restarts without faults, `control-node` logs `streak=1`,
  then `streak=2`, then `state transition from=SafeStop to=Autonomous` on the
  third consecutive packet. The plant starts moving again.
