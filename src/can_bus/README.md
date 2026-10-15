# can_bus

`can_bus_node` is the sole physical device owner. It has no motor protocol
dependency. Run it in a bus namespace, for example `/can/main`; all endpoint
names below are relative to that namespace.

Launch only the generic owner, without any motor or simulated device nodes:

```bash
ros2 launch can_bus can_bus.launch.py \
  namespace:=can/main backend:=serial port:=/dev/ttyACM0
```

Launch arguments are `namespace` (default `can/main`), `backend` (default
`sim`), `port` (default empty), `can_fd` (`true`/`false`, default `false`),
and `can_bitrate` (positive integer, default `1000000`). Select an explicit
device/interface for hardware backends. The launch file changes no physical
interface settings.

The executable can also be run directly; this example uses no hardware:

```bash
ros2 run can_bus can_bus_node --ros-args -r __ns:=/can/main -p backend:=sim
```

Parameters: `backend` (`sim`, `serial`, `socketcan`), `port` (serial device or
SocketCAN interface), `can_fd` (default false), `can_bitrate` (informational;
configure the physical interface separately), `io_rate` (default 1000 Hz),
and `client_timeout` (default 0.5 seconds; registrations explicitly select
their lease). SocketCAN supports standard/extended classical and native FD
frames, including RTR on classical CAN and BRS on FD. FD wire lengths are
0–8, 12, 16, 20, 24, 32, 48, and 64. The legacy USB adapter only supports
channel-zero standard classical frames: eight bytes, or four bytes padded
to eight. Unsupported flags or lengths are rejected, never silently truncated.
SocketCAN ownership uses a process-lifetime abstract UNIX socket lock;
serial ownership uses exclusive device access. No automatic reconnect exists.

## Interface

* `register_client` (`RegisterClient`): submit `client_name`, `filters_rx`,
  `filters_tx`, opaque `stop_frames`, `lease_duration` (0.3–1 seconds),
  `rate_hz`, `max_tx_hz` (actual transmit ceiling), and optional `fault_group`.
  The response contains `success`, `token`, `epoch`, `rx_topic`, `message`.
  Tokens are epoch-scoped routing capabilities, not a network authentication
  boundary; secure the ROS domain separately.
* `FrameFilter` matches `can_id & mask`, exact `extended`/`channel`, and
  masked payload `prefix`/`prefix_mask`. Prefix arrays must have equal lengths.
  All transmit filters are exclusive across clients. A fresh, explicitly
  relaunched client can replace its own revoked registration by name, but
  cannot revive the old token. Receive filters may overlap for observers.
* Publish `TxFrame` on `tx`: `token`, `epoch`, strictly increasing `sequence`,
  `valid_for` (positive, at most 250 ms), `priority`, `active`, `sent_steady`,
  and `frame`. Stamp `frame.header.stamp` at publication with ROS time:
  unstamped, future, or expired frames are rejected before entering queues.
  `active=true` is required for operations that can activate or move a device;
  `sent_steady` is the sender's monotonic nanoseconds. Read-only queries,
  parameter operations and stops set `active=false`.
* Priority 0 is normal; 1 must exactly equal a registered stop; 2 requires
  bus maintenance ownership. Stops bypass rate limiting, clear old normal
  work, and are never synthesized by interpreting a device opcode.
* Publish `Heartbeat` on `heartbeat` every 50 ms: `token`, `epoch`, separate
  monotonic `sequence`, `quiet`, `fault`, `sent_steady`. Quiet means physically
  disabled, not merely that the sender has no queued command. Activity
  atomically clears quiet. After activity, even a newer quiet heartbeat
  cannot restore quiet until an exact registered stop has been dispatched
  and the heartbeat's timestamp is subsequent to that dispatch.
* `maintenance` (`Maintenance`): `token`, `epoch`, `acquire`, `duration`
  (at most one second). Acquisition requires all live clients fresh and quiet.
  Renew before expiry; release explicitly. Active transmissions are forbidden
  even by the maintenance owner. The motor bridge renews a 600 ms lease,
  releases after 100 ms parameter inactivity, and rejects enable/motion
  during acquisition or ownership.
* `rx` broadcasts received frames; each registration also receives only its
  matched frames on the returned `rx_topic`. `Frame` carries `header`,
  `can_id`, `channel`, `extended`, `rtr`, `error`, `fd`, `brs`, `length`,
  bounded `data` (64 bytes), and acquisition `received_steady` nanoseconds.
  `data.size()` is authoritative; nonzero `length` must agree.
* `status` is reliable/transient-local, depth one, published every 50 ms.
It includes `epoch`, `ready`, `endpoint_ready`, `backend`, `error`, counters, queue/client
  counts, `maintenance_owner` (token), `all_quiet`, latched `group_faults`,
  and `client_errors` (`token: reason` for rejected activity). All other
  traffic is reliable with finite history.
  Parallel arrays `client_tokens`, `heartbeat_sequences`, `client_faults`,
  `client_rx_ready`
  acknowledge each client's most recently processed heartbeat. The bridge
  uses that acknowledgment before allowing activity after an ungrouped
  explicit fault reset, avoiding a cross-topic heartbeat/TX race.

The monotonic timestamps used for heartbeat/activity and acquisition age
assume clients and the bus owner share the same host monotonic clock.
Cross-host clients require a clock-domain bridge rather than comparing
unrelated monotonic clocks.

## Failure and boundedness

There are at most 128 registrations per epoch, 32 TX/RX filters per client,
16 stop frames, 32 pending TX frames per client, and 64-message DDS histories.
Normal work coalesces to the latest frame per matching route and CAN ID.
Each client can transmit at most its registered `max_tx_hz`; emergency stops
are exempt. TTL is measured from publication, including DDS and local queues.
The I/O timer has bounded physical reads.

Lease expiry clears work, revokes the token, and sends the opaque stops once.
An explicitly reported ungrouped fault sends stops and rejects activity but
keeps the token, RX, stop transmission, and read-only polling alive; an
explicit host reset can clear that client's fault flag without registering
again. A nonempty fault group latches and stops/revokes all its
members independently of motor processes. A fresh same-name registration can
replace revoked ownership but cannot clear a latched fault group, and the
motor bridge still requires disabled startup confirmation and explicit enable.
Maintenance expiry similarly revokes its owner. Hardware
failure latches `ready=false` and attempts all stops; failed hardware cannot
guarantee stop delivery, so devices still need their own watchdogs.
Distributed faults deliberately require bus/client relaunch; there is no
automatic group reset, replacement registration, reconnect, or resume.
Repeated stale traffic cannot revive a token.

## Simulation and library

The `sim` backend publishes accepted traffic on `wire_tx` and accepts physical
receive traffic only from `wire_rx`. It never echoes TX as feedback and knows
no motor opcodes. Attach independent simulated device processes, loopback
responders, or other device emulators to these endpoints.
`ready` still reports an initialized empty simulation bus as ready;
`endpoint_ready` additionally requires a matched `wire_tx` subscriber and
`wire_rx` publisher. Until that bidirectional endpoint is connected, queued
TX is held while TTL and heartbeat leases continue to advance. Motor bridge
readiness requires endpoint readiness and matched private RX, TX and
heartbeat channels, including broker acknowledgment that its private RX
publisher has matched the client. Normal traffic for an unmatched private
RX channel is held independently of other clients; exact stops still bypass
that hold. Maintenance acquisition likewise waits for the client's receive
path to match. No fixed startup sleep is used. Loss of an
established virtual endpoint latches the motor bridge instead of freezing
its pending operations silently.
The DM bridge additionally proves an actual status round trip with
read-only `0xCC` polls before reporting ready. DDS graph discovery alone
does not prove the first response will meet an operation deadline. Parameter
transactions start only after that probe succeeds; no parameter request or
write is automatically retried, and normal operation deadlines are unchanged.

`can_bus::can_bus_transport` exports generic `Frame`, `Transport`, physical
factories, legacy framing/parser, and the ROS-independent leased `Router`.
The old motor serial APIs are conversion-only compatibility wrappers over
this library. Production motor processes use the asynchronous `CanTransport`
ROS bridge, which owns an isolated executor, not a device descriptor.
The motor runtime must check `Transport::activity_available()` before changing
its phase for an enable request. Register-operation confirmation precedes the
bridge's asynchronous maintenance release; enable during that short interval
must be rejected as a precondition, not converted into a transport fault.
`CanTransport::activity_available()` requires release completion and an
unlocked bus status, while `ready()` remains true throughout normal maintenance.
The motor bridge treats read-only status polls as replaceable observations:
polls suppressed by another client's maintenance, acquisition, or release are
dropped and subsequent polling resumes normally. Poll expiry is not a
transport failure; motion expiry and parameter-operation deadlines retain
their fail-safe behavior.
At transaction stage zero, the runtime must call
`Transport::prepare_maintenance()` before starting the operation's response
deadline or transmitting its first frame. The bridge starts asynchronous
acquisition and returns false until its reservation is acknowledged; false
leaves the transaction queued without consuming its response timeout.
`maintenance_available()` is only an admission/status snapshot and is not
proof of ownership. Parameter commands and motor clear/zero commands use
the acknowledged maintenance lease and FIFO priority-two transmission.
`maintenance_blocked()` reports an authoritative non-quiet status, not
another quiet maintenance owner or the client's own asynchronous
acquisition/release. Quiet clients arbitrate through reservation acquisition;
explicit queued operations can reject when activity blocks maintenance,
while startup may wait. An acknowledged local reservation takes
precedence over older cached bus status, including for query transmission.
