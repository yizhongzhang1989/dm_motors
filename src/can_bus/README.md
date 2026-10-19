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
These parameters are startup-only/read-only: changing a ROS parameter cannot
silently leave the actual device or polling timer at its old setting.
SocketCAN ownership uses a process-lifetime abstract UNIX socket lock;
serial ownership uses exclusive device access. No automatic reconnect exists.

## Real-time data path

Accepted TX requests wake transmission immediately in the subscription callback;
they do not wait for the `io_rate` timer. Client rate ceilings, coalescing, TTLs,
RX-readiness and maintenance ownership still apply. The periodic timer remains
responsible for physical RX polling, watchdog/lease checks and rate-limited
pending traffic; lowering it therefore still increases hardware receive latency.
Fault heartbeats also trigger an immediate drain so registered STOP frames do
not wait for the next periodic tick. Pending priority STOP frames precede other
clients' normal frames within a drain batch.

Private filtered RX is forwarded directly to device clients before optional
public observation. Each received frame is observed exactly once on public `rx`,
including when there are no registered clients; observer counts must not scale
with the number of registrations. Public `rx` and `tx_observed` publication runs on a separate
bounded worker queue, so slow public recorders do not directly block the physical
I/O callback. Its 8192-entry backlog discards the oldest observation on overflow,
logs a warning, and reports `observer_dropped` / `observer_queue_depth` in status.
These counters describe public observation only, not motor feedback decimation.
Hardware receive timestamps are retained rather than replaced after routing.

Serial parsing scans each input batch once and compacts its remainder once,
preserving partial-frame resynchronization, adapter errors and all decoded frames.
Normal scheduling no longer matches each sent frame against every client's STOP
catalog; ownership-local matching is used on the ordinary path.

This is not a hard-real-time guarantee. Status publication, registration and
maintenance still share the broker executor with data callbacks, and private
reliable RX delivery can still experience middleware backpressure. Avoid dynamic
registration/configuration during latency-critical motion. No OS scheduler
priority, physical baud rate or CAN bitrate is changed by these optimizations.

Sourcing this workspace selects the installed [Fast DDS UDPv4 profile](config/fastdds_udp.xml)
when Fast DDS is the selected/default RMW and no explicit DDS profile is set.
It disables the built-in SHM transport, not DDS reliability or the physical CAN backend.
Explicit `FASTRTPS_DEFAULT_PROFILES_FILE` / `FASTDDS_DEFAULT_PROFILES_FILE` values
and other `RMW_IMPLEMENTATION` choices are preserved. All participating processes
must load the workspace environment; already running processes do not inherit a new profile.

With Humble / Fast DDS 2.6.12, repeated abrupt motor-process exits exposed intermittent
peer communication timeouts under default SHM; the unchanged failure-isolation test
passed 20 consecutive runs with UDPv4. The
[Fast DDS SHM documentation](https://fast-rtps.docs.eprosima.com/en/2.6.x/fastdds/transport/shared_memory/shared_memory.html)
describes crashed-port recovery with a default 1000 ms health-check timeout.
This deployment default avoids that recovery path instead of relaxing application
watchdogs or deleting shared-memory objects belonging to other processes.
Custom middleware profiles need their own process-failure and latency qualification.

## Interface

* `register_client` (`RegisterClient`): submit `client_name`, `filters_rx`,
  `filters_tx`, opaque `stop_frames`, optional `poll_frames`, `lease_duration` (0.3–1 seconds),
  `rate_hz`, `max_tx_hz` (actual transmit ceiling), and optional `fault_group`.
  The response contains `success`, `token`, `epoch`, `rx_topic`, `message`.
  Tokens are epoch-scoped routing capabilities, not a network authentication
  boundary; secure the ROS domain separately.
  Registration initially reserves routes for at most two seconds. Normal TX
  is held (with its original TTL still advancing) and maintenance is unavailable
  until the first fresh heartbeat activates the requested 0.3–1 second lease.
  Exact STOPs remain available before activation. Subsequent heartbeat loss
  uses the requested lease duration, not the startup reservation budget.
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
  An exact registered `poll_frames` match with priority 0 and `active=false`
  may pass another client's maintenance lock. Clients certify these templates
  as read-only; the broker remains protocol-neutral. Unregistered frames,
  active traffic and priority-two parameter traffic do not get this exception.
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
  Revoked leases are reported explicitly in `client_errors`, so their clients
  fail immediately rather than waiting for an unrelated startup probe timeout.

The monotonic timestamps used for heartbeat/activity and acquisition age
assume clients and the bus owner share the same host monotonic clock.
Cross-host clients require a clock-domain bridge rather than comparing
unrelated monotonic clocks.

## Failure and boundedness

There are at most 128 registrations per epoch, 32 TX/RX filters per client,
16 stop frames, 16 read-only poll templates, 32 pending TX frames per client,
and 64-message DDS histories.
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
The bridge waits for TX/heartbeat topic discovery before requesting registration,
and for a broker heartbeat acknowledgment before probing the device. Graph
matching alone does not prove data delivery. The broker starts the bounded
registration window after preparing private RX, so entity setup does not consume
the client's activation budget.
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
The motor bridge registers only its exact read-only `0xCC` status request as
a maintenance-safe poll. Other axes continue receiving real feedback while a
client performs a long parameter batch, including while their own maintenance
request is waiting. The runtime still suppresses its own status queries during
an ambiguous parameter transaction; parameter exclusivity is unchanged.
Expired polls are replaceable observations rather than transport failures;
motion expiry and parameter-operation deadlines retain their fail-safe behavior.
`RegisterClient` now includes `poll_frames`; rebuild and restart the broker and
all CAN clients together after this interface change.
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

The motor bridge applies its configured `can_fd` framing to every runtime frame,
including queries, parameter requests and STOPs. The transmitted STOP format must
match its registered opaque stop frames exactly. FD is not supported by the legacy
serial backend; native SocketCAN FD framing does not configure physical bit timing
or imply that a particular motor/adapter combination has been hardware-certified.
Motor feedback age retains host acquisition time through the receive queue; it is
not refreshed when a control cycle processes the frame.
