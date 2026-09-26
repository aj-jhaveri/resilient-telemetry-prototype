# Design

## Scope

A local learning prototype with three synchronous processes connected by UDP.
It covers message versioning, boundary validation, sequence checks, freshness
based on a local monotonic clock, and an explicit safe state. It has no async
runtime, no command transport, and no hardware interface. It is not
production, real-time, or safety-certified software.

`protocol` holds the telemetry type, validation, the `Controller` state
machine, and shared constants. The three binaries are thin wrappers around it.

## Data flow

```text
sensor-node --UDP--> link-sim (127.0.0.1:9100) --UDP--> control-node (127.0.0.1:9200)
```

Each datagram is one JSON `Telemetry` object with these fields: `version: u8`,
`sequence: u64`, `sent_at_ms: u128`, `latitude_deg`, `longitude_deg`,
`heading_deg`, `speed_mps` (all `f64`), and `battery_percent: f32`.
`sensor-node` starts at sequence 1 and increments by one per packet. It sends
fixed, valid, Austin-like values and the current Unix-millisecond time.

## Validation

`Telemetry::decode` is the boundary. It parses, validates, and returns one of
two error kinds:

- **Malformed**: invalid JSON, missing fields, wrong types, unknown fields
  (`deny_unknown_fields`), or out-of-type values such as a negative `sequence`.
- **Invalid**: well-formed JSON that breaks a rule. The checks run in this
  order:
  1. `version` must equal `SUPPORTED_VERSION` (1).
  2. Every float must be finite. JSON cannot encode `NaN`, but `1e39`
     overflows `f32` to infinity, so this check is reachable from the wire.
  3. Latitude must be in `[-90, 90]` and longitude in `[-180, 180]`.
  4. Heading must be in `[0, 360)`.
  5. Speed must be in `[0, 20]` m/s.
  6. Battery must be in `[0, 100]`.

The limits are constants in `crates/protocol/src/lib.rs`.

## Sequence handling

- The first valid packet is accepted, whatever its sequence.
- Each later packet needs a strictly greater sequence. Gaps are allowed.
- An equal sequence is rejected as `Duplicate`. A smaller one is rejected as
  `OutOfOrder`. `SequenceGate` holds this rule.
- A rejected packet changes neither the state, the stored sequence, nor the
  freshness timestamp. `control-node` logs it and continues.

Restarting `sensor-node` resets its sequence to 1. The running `control-node`
rejects those packets until it is restarted too.

## Freshness and SafeStop

The states are `WaitingForTelemetry` (initial), `Autonomous`, and `SafeStop`.

- An accepted packet records the local `Instant` it arrived, moves the state to
  `Autonomous`, and emits an `Autonomous` command: rudder `0.0`, throttle
  `0.25`, reason `valid_telemetry`.
- `tick(now)` moves the state from `Autonomous` to `SafeStop` once
  `now - last_accepted >= timeout` (750 ms by default). On that transition
  only, it emits one command: rudder `0.0`, throttle `0.0`, reason
  `telemetry_timeout`. Later ticks in `SafeStop` emit nothing.
- `WaitingForTelemetry` never times out.
- A new accepted packet returns the state from `SafeStop` to `Autonomous`.
- The controller never reads `sent_at_ms`. Clocks on different hosts are not
  comparable, and the local clock is monotonic.

`Command` fields are private, and the only constructors are the two fixed
variants above. Every command therefore stays within rudder `[-1, 1]` and
throttle `[0, 1]`. Command sequences come from their own counter, which starts
at 1.

`control-node` uses a 50 ms socket read timeout and calls `tick` after every
receive attempt. SafeStop is detected up to about 50 ms after the deadline,
plus scheduling delay. An accepted packet is also checked against the
deadline at its arrival instant, before it refreshes freshness. If the
deadline has passed, the SafeStop command is emitted first. The controller
takes `now: Instant` as a parameter, so tests can pass synthetic instants.

## Deterministic link behavior

`link-sim` counts received datagrams from 1. With `--drop-every N` (N > 0), it
drops each packet where `count % N == 0`. `N = 0` disables dropping. Kept
packets are forwarded byte-for-byte after sleeping for `--delay-ms`. The sleep
runs on the single receive loop, so the delay should stay below the sensor
interval. There is no randomness, jitter, duplication, corruption, or
reordering.

## Tests

`cargo test --workspace` covers:

- **Telemetry:** round-trip, the malformed cases above, every rule at its
  boundaries, non-finite values in each float field, and `f32` overflow from
  JSON.
- **Controller:** the initial state, sequence acceptance and rejection,
  rejected input leaving the state unchanged, a timeout exactly at the
  deadline and not before, SafeStop emitted once, recovery, independence from
  `sent_at_ms`, and command bounds and sequencing. All timing uses synthetic
  instants.
- **`control-node`:** a loopback UDP datagram through `poll_once` reaches
  `Autonomous`. A bad datagram and a read timeout leave the node waiting.
- **`link-sim`:** drop patterns for N = 0, 1, and 3.

## Limitations

- No automated test runs all three processes together. The README demos cover
  that path by hand.
- Timing depends on OS scheduling, with no real-time guarantees.
- There is no authentication or integrity protection. Any local process can
  send datagrams.
- Commands are only logged, never transmitted or applied.
- Only one message version is supported, with no negotiation.
