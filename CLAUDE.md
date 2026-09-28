# CLAUDE.md — Resilient Telemetry Prototype

Rules for every Claude session in this repository. They are mandatory.

## What this is

A small Rust/Linux **learning prototype**: `sensor-node → link-sim → control-node`
over local UDP, with versioned JSON telemetry, boundary validation, sequence
checks, monotonic-clock freshness, and an explicit `SafeStop` state.

Phase 2 closes the loop: `sensor-node` runs a **simulated vessel plant** that
produces the telemetry, and `control-node` sends validated `CommandMessage`
datagrams directly back to `sensor-node`, which zeroes rudder and throttle if
commands stop arriving. Both are simulated onboard processes on one machine.

It is **not** production, vessel, embedded, navigation, real-time, or
safety-certified software, and must never be described as fit for controlling
physical hardware. Never write or imply otherwise in code, docs, commit
messages, or reports.

## Scope

- Work only inside this directory. Do not read, modify, or create anything
  outside it.
- Never make a Git commit, push, or tag unless the user explicitly asks in the
  current conversation.
- Workspace packages are exactly: `crates/protocol`, `crates/sensor-node`,
  `crates/link-sim`, `crates/control-node`. No extra packages, services, or
  binaries.
- Rust stable, synchronous `std::net::UdpSocket`. No async runtime.
- Permitted external crates only: `serde` (derive), `serde_json`, `thiserror`,
  `clap` (derive), `tracing`, `tracing-subscriber`. Adding any other crate,
  including dev-dependencies, requires asking the user first.

## Hard exclusions

No frontend/UI, web service, REST/GraphQL, dashboard; no database, Redis,
Kafka, MQTT, NATS, RabbitMQ; no Docker, Kubernetes, cloud, Terraform, CI/CD,
GitHub Actions, Nix; no hardware, serial, CAN, microcontrollers, embedded SDKs;
no ROS, Gazebo, OpenCV, TensorFlow, SLAM, sensor fusion, perception, computer
vision, path planning; no C++, Python, JavaScript, TypeScript; no
authentication, crypto, encryption, secrets; no randomness, benchmarks, or
performance claims; no screenshots, recordings, incident reports,
retrospectives, roadmaps, changelogs; no hydrodynamics, drag matrices, PID
integral terms, or other controller state beyond proportional control with
feedforward.

`link-sim` is a deterministic fault injector on the sensor-to-controller
telemetry path only. It implements only drop-every-N and fixed delay. No
jitter, random loss, duplication, corruption, reordering, or retries.
Commands travel directly from `control-node` to `sensor-node` and never pass
through `link-sim`; documentation must state that the command path has no
injected faults.

## Code quality

- No `unsafe` (workspace lint `unsafe_code = "forbid"`).
- No `unwrap()` / `expect()` in production code (workspace clippy lints deny
  them; test modules may opt out explicitly).
- Expected bad input (malformed, invalid, duplicate, out-of-order datagrams;
  socket errors) is logged and the node continues. It never panics.
- Freshness uses only the local monotonic clock (`Instant`). The sender's
  `sent_at_ms` is informational and must never affect freshness.
- Sans-I/O: all plant, control, sequence, freshness, and watchdog logic lives
  in `crates/protocol` with no sockets, sleeping, or clock reads. Time enters
  only as explicit `dt: Duration` or `now: Instant` arguments. Binaries only
  do I/O, read the clock, and log.
- Every command's rudder and throttle are clamped to their bounds by
  construction. Received `CommandMessage`s are validated at the boundary,
  exactly like telemetry.
- Single source of truth: protocol constants (versions, ranges, timeouts,
  plant parameters, gains, default addresses, buffer size) live in
  `crates/protocol`. Binaries and tests read those constants instead of
  repeating literals.

## Documentation

- Only these non-Cargo files exist: `README.md`, `DESIGN.md`, `CLAUDE.md`,
  `.gitignore`. Do not add more documents.
- README sections, in order, and only these: Title, Purpose, Limitations,
  Requirements, Build and test, Run the normal demo, Run the packet-loss demo,
  What to look for.
- DESIGN sections, in order, and only these: Scope, Data flow, Validation,
  Sequence handling, Freshness and SafeStop, Plant model and control,
  Deterministic link behavior, Tests, Limitations. Keep DESIGN.md under 1,200
  words.
- Always call the plant a "simulated vessel plant". Plant parameters and gains
  are illustrative and must never be described as modelling a real vessel.
- Every README command must work exactly as written with the implemented CLI
  arguments. When a flag or default changes, update README and DESIGN in the
  same change.
- Describe only what the code actually does. No speculative features, no
  unmeasured numbers.

## Tests

Keep the suite compact and meaningful. It must cover: JSON round-trip and
malformed JSON; every validation category including non-finite values;
sequence acceptance/rejection; state transitions; command bounds and SafeStop
invariants; a socket-level path to `Autonomous`; a deterministic timeout path
to `SafeStop`; SafeStop recovery hysteresis; plant kinematics and heading
wrapping; closed-loop convergence and SafeStop coast-down without sockets or
sleeping; `CommandMessage` validation; the sensor-side command watchdog; and a
socket-level check that a command leaves `control-node`. Assertions read
protocol constants, not duplicated literals.

## Quality gate (run all three before reporting)

```bash
cargo fmt --all -- --check
cargo clippy --workspace --all-targets -- -D warnings
cargo test --workspace
```

If one fails, make only the smallest direct fix, then rerun all three. Report
results exactly as observed. Never claim a check ran or passed without output
from this session.
