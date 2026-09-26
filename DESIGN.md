# Design

## Scope

A local learning prototype: three single-threaded processes on one machine,
connected by UDP. A proportional controller and a simulated vessel plant form
a closed loop. There is no async runtime and no hardware interface. It is not
production, real-time, or safety-certified software, and the plant's
parameters are illustrative.

`protocol` holds all the logic, with no I/O: messages, validation,
`SequenceGate`, `Controller`, the plant, the P law, and the command watchdog.
Time enters only as `dt` or `now` arguments. The binaries only do sockets,
clock reads, and logging.

## Data flow

```text
sensor-node :9300 ──telemetry──► link-sim :9100 ──telemetry──► control-node :9200
      ▲                                                            │
      └─────────────── CommandMessage (direct, no faults) ─────────┘
```

`Telemetry` fields: `version: u8`, `sequence: u64`, `sent_at_ms: u128`,
`latitude_deg`, `longitude_deg`, `heading_deg`, `speed_mps` (all `f64`), and
`battery_percent: f32`. `sensor-node` numbers packets from 1, one per
`--interval-ms`.

`CommandMessage` fields: `version: u8`, `sequence: u64`,
`mode` (`autonomous` | `safe_stop`), `rudder: f64`, `throttle: f64`, and
`reason` (`valid_telemetry` | `telemetry_timeout`). `control-node` sends one
per command. Commands bypass `link-sim`.

## Validation

`Telemetry::decode` is the telemetry boundary. It returns **Malformed** for
invalid JSON, missing or unknown fields (`deny_unknown_fields`), and wrong or
out-of-type values such as a negative `sequence`. It returns **Invalid** for
well-formed JSON that breaks a rule, checked in this order:

1. `version` must equal `SUPPORTED_VERSION` (1).
2. Every float must be finite. `1e39` overflows `f32` to infinity, so this
   check is reachable from the wire.
3. Latitude must be in `[-90, 90]` and longitude in `[-180, 180]`.
4. Heading must be in `[0, 360)`.
5. Speed must be in `[0, 20]` m/s.
6. Battery must be in `[0, 100]`.

`CommandMessage::decode` is the command boundary in `sensor-node`. It checks
that `version` equals `COMMAND_VERSION` (1), that the values are finite, that
rudder is in `[-1, 1]` and throttle in `[0, 1]`, that mode matches reason, and
that a `safe_stop` carries zero rudder and throttle. All limits are constants
in `crates/protocol/src/lib.rs`.

## Sequence handling

`SequenceGate` accepts the first sequence it sees, then only strictly greater
ones. Gaps are allowed. An equal sequence is `Duplicate`, and a smaller one is
`OutOfOrder`. Telemetry in `control-node` and commands in `sensor-node` each
have their own gate. A rejected datagram changes nothing: not the state, the
stored sequence, or any freshness timestamp. The node logs it and continues.

Restarting either sender resets its sequence to 1. The running receiver then
rejects that sender until the receiver is restarted too. There are no
session IDs.

## Freshness and SafeStop

`Controller` states: `WaitingForTelemetry` (initial), `Autonomous`,
`SafeStop`.

- An accepted packet records the local `Instant` it arrived. From
  `WaitingForTelemetry` or `Autonomous`, it moves to `Autonomous` and emits a
  command computed by the P law.
- `tick(now)` moves from `Autonomous` to `SafeStop` once
  `now - last_accepted >= timeout` (750 ms by default). On that transition
  only, it emits a SafeStop command: rudder `0.0`, throttle `0.0`, reason
  `telemetry_timeout`. Later ticks emit nothing.
- An accepted packet is first checked against the deadline at its arrival
  instant, so a late packet reports the timeout even without a `tick`.
- **Recovery hysteresis:** leaving `SafeStop` needs `--recovery-packets`
  (default 3) consecutive accepted packets, each less than `timeout` after the
  previous one. A longer gap resets the streak to 1. No command is emitted
  until the streak completes. So with packets 1000 ms apart, the node enters
  `SafeStop` once and stays there instead of toggling. A value of 1
  reproduces Phase 1.
- `WaitingForTelemetry` never times out, and `sent_at_ms` is never read.

`control-node` polls with a 50 ms read timeout, so it detects SafeStop up to
about 50 ms after the deadline, plus scheduling delay. `Command` fields are
private. Its only constructors take a clamped `Actuation` or the SafeStop
constants, so every command is bounded.

**Sensor-side watchdog:** `CommandReceiver` applies the latest valid command.
It returns zero rudder and throttle before the first command, and once
`--command-timeout-ms` (default 1000) passes without one. It reports the
expiry once. The default is longer than the 750 ms telemetry timeout, because
commands are sent only when telemetry is accepted. `sensor-node` checks the
watchdog once per step, so the zero takes effect up to one interval late.

## Plant model and control

The simulated vessel plant (`PlantState::step`, fixed `dt` = `--interval-ms`)
works as follows. Heading is measured clockwise from north, and positive
rudder turns clockwise.

- `heading ← wrap(heading + rudder · 10 °/s · dt)`, where `wrap` maps into
  `[0, 360)`. It guards the case where `rem_euclid` rounds a tiny negative
  input up to exactly 360.
- `speed ← speed + α · (throttle · 5 m/s − speed)`, with
  `α = 1 − e^(−dt/2 s)`. This is the exact solution of a first-order lag, so it
  never overshoots and does not depend on step size.
- Position uses flat-earth dead reckoning (`111 320` m per degree of
  latitude), which is not meaningful near the poles. Battery drains at
  0.05 %/s times throttle.

The P law (`p_control`) is pure:

- `rudder = (1/30) · heading_error`, where the heading error is the shortest
  turn, in `[-180, 180)`.
- `throttle = target/5 + 0.5 · (target − speed)`.
- Both outputs are clamped to their bounds.

Heading needs only P, because the plant integrates rudder. Throttle adds
feedforward, because P alone would settle below the target: at 5/3.5 ≈ 1.43 m/s
instead of 2.0. The closed-loop convergence test fails if the feedforward
term is removed. There is no integral
or derivative term. `Setpoint` rejects non-finite or out-of-range targets,
including `NaN`, which clap accepts for `f64` flags.

## Deterministic link behavior

`link-sim` counts received datagrams from 1. With `--drop-every N` (N > 0), it
drops each packet where `count % N == 0`. `N = 0` disables dropping. Kept
packets are forwarded byte-for-byte after sleeping `--delay-ms`. The sleep
blocks the single receive loop, so the delay should stay below the sensor
interval. There is no randomness, jitter, duplication, corruption, or
reordering.

## Tests

`cargo test --workspace` runs these tests. The pure-logic tests use a fixed
`dt` and synthetic `Instant`s, with no sockets or sleeping.

- **Messages:** round-trips, malformed input, each rule at its boundaries,
  non-finite values, and command mode/reason consistency.
- **Controller:** sequence rules, the timeout at exactly the deadline, a
  single SafeStop, late-arrival timeouts, the recovery streak and its reset,
  no toggling under 1000 ms gaps, and commands equal to the P law's output.
- **Plant and P law:** heading wrapping, clamping, the exact and
  step-independent speed response, coast-down, valid telemetry over 10,000
  steps, and closed-loop convergence (including across north) with a one-step
  command delay.
- **Watchdog:** expiry exactly at the timeout, reported once; rejected
  datagrams do not refresh it.
- **Sockets:** `control-node` sends Autonomous and single SafeStop commands.
  `sensor-node` applies a received command and sends valid telemetry.
  `link-sim` drop patterns for N = 0, 1, and 3.

## Limitations

- No automated test runs all three processes together. The README demos cover
  that path by hand, verified on macOS only.
- Timing depends on OS scheduling, with no real-time guarantees.
- There is no authentication or integrity protection. Any local process can
  send datagrams to any node.
- The command path has no injected faults.
- Restarting a sender requires restarting its receiver (see Sequence
  handling).
- Only message version 1 is supported, with no negotiation.
