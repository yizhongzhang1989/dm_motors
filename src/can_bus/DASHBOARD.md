# CAN Dashboard (included in can_bus)

Independent ROS 2 generic CAN HTTP monitor. No motor dependencies, opcodes,
enable helpers, probing, replay, maintenance acquisition, or hardware setup.
The dashboard does not launch the bus or any device. Monitoring registers no
client and cannot transmit.

```bash
ros2 launch can_bus dashboard.launch.py \
  bus_namespace:=/can/main dashboard_port:=config
```

Executable: `ros2 run can_bus dashboard.py`. Launch arguments:
`bus_namespace` (absolute, default `/can/main`), `dashboard_host` (default `0.0.0.0`),
`dashboard_port` (empty by default: no dashboard). `config` selects `parameters.dashboard_port`
from `packages.can_bus.launches["dashboard.launch.py"]` in
[the common configuration](../../config/dm_motors.yaml); a numeric override must be in
1..65535. The shared file also provides host/bus defaults; `config_file` and
`COMMON_CONFIG` select an alternative file. Launch maps these to the HTTP server's
`host`/`port` ROS parameters. Backend/FD capability, hardware port and bitrate are read from
actual bus status; debugging remains disabled until that status is fresh.

The default listener accepts connections on all local IPv4 interfaces. From
another computer on the trusted LAN, open `http://<server-lan-ip>:8080/`, not
`localhost`, `127.0.0.1` or `0.0.0.0`. Set `dashboard_host:=127.0.0.1` for local-only
access, or bind a specific local address. Existing processes must be restarted
to change their listener. No firewall, routing or public port forwarding is
configured automatically.

## Console layout and interaction

The Chinese-language console follows the same visual layout as the motor
workspaces, without adding motor dependencies or interpreting device opcodes.
The same 84px fixed single-row header places bus identity/environment/connection
on the left, large authoritative RX/TX counts and rates, rejection count and
registered clients in the middle, and release/cancel on the right. It stays
visible and updates while scrolling or switching tabs. On narrow screens only
the counter strip scrolls horizontally; identity and release stay in place.
Full namespace and counter values remain available in tooltips.
Four keyboard-accessible tabs separate the ID aggregate, recent frames,
diagnostics and manual debugging. Endpoint, queues, observer drops and status
age join client health and the complete raw snapshot in diagnostics. Fault and
observation-gap warnings remain explicit; TX still means driver-write observation,
not device execution.

ID and direction filters apply to both traffic views. Invalid filter input is
reported beside the field, not as a connection failure. Pause freezes only the
tables; status counters and the independent 250ms browser heartbeat remain live.
CSV exports exactly the displayed, filtered frames (at most 200, newest first);
JSON exports the public status snapshot with an export timestamp. No private
session identifier is exported.

Debugging uses direct claim/send button actions with fresh bus/endpoint
capabilities; there is no extra authorization checkbox or confirmation dialog.
Once claimed, route fields are locked; sends use the returned route rather than
an editable draft. Release stays available in the header, including during a
pending operation. A pending claim can also be cancelled from the header.
Releasing/cancelling, hiding/leaving the page, heartbeat
failure or loss of monitoring stops renewal and requests release. Returning or
recovering the connection never reclaims automatically; another explicit claim
is sufficient without a separate authorization step. A late claim response
after cancellation is released, not restored. Unsupported flag drafts are cleared
with an explicit notice when backend capabilities change.

The non-active option maps to `readonly:true` / `active=false`; it still permits
raw frame transmission and is not a transmit-disabled monitoring mode. Submitted,
driver-written and device-executed are intentionally not presented as equivalent.

## Monitoring and bounds

`status` shows backend, actual broker readiness/endpoint readiness, epoch,
errors, authoritative RX/TX/rejection counters, queue depth, client count,
maintenance owner name, client names/heartbeat acknowledgments/faults/receive
readiness, quiet state and group faults. Capability tokens and
token-bearing client diagnostic prefixes are never exposed over HTTP.

Status also exposes `observer_dropped` and `observer_queue_depth`. Public RX/TX
observation is asynchronous and bounded; a nonzero drop counter means the raw
observer trace is incomplete, not that private motor feedback was downsampled.

`rx` means acquired receive traffic, including error frames where the backend
provides them. `tx_observed` means a successful driver write (or simulation
wire publication), **not CAN ACK or device execution**. A submitted debug frame
is tracked separately; an HTTP send response is not a broker acceptance
acknowledgment. Error-frame observations are distinct from broker errors;
hardware errors may instead latch `ready=false` before an RX frame is emitted.
Actual status counters remain authoritative if monitoring loses DDS samples.

Frame history: 1000 entries; per-ID/direction recent table: 512 keys. Rates use
counter differences over completed measurement windows of at least one second,
with zero during initial warm-up. Slower polling lengthens the measurement window;
rates are not capped by retained frame history. Only constant-size rate state is
kept per direction/key, rather than one timestamp per observed frame. Old keys
evict least recently observed. DDS histories are bounded (64, status 1).
HTTP permits at most 16 concurrent handlers, 16 pending connections, two-second
socket read timeouts and 16 KiB JSON bodies. Browser monitoring polls sequentially
at up to 10 Hz, with two-second request deadlines; heartbeat requests run
independently and never overlap each other. Paused tables do not fetch frames.
There is no unbounded SSE or replay queue.

## Stable HTTP contract

IDs and bytes are JSON integers (not hex strings), boolean flags are strict
booleans. Public `status.epoch` and session-response `epoch` are **decimal
strings**, preserving exact uint64 values beyond JavaScript's safe-integer
range. Internal broker authentication and ROS messages still use integer
epochs; clients never need to submit an epoch through HTTP.
All mutation bodies are JSON objects; `Content-Type: application/json`
is required. Nonfinite numbers, invalid byte arrays, unsupported flags/lengths,
oversized bodies and cross-site requests are rejected. Errors are
`{"error":"description"}` with HTTP 400/403/409/413/415/429/503/504.

* `GET /api/state`: `bus_namespace`, `connected`, `status`, `status_age`,
  `capabilities`, `monitor` (totals, rates, IDs, sequence, observed error frames),
  `debug` (public owner/route/remaining duration, never session ID or bus token),
  `notice`, `release_pending`, `tx_observed_available`.
* `GET /api/frames?after=0&limit=200&can_id=0x600&direction=rx`:
  `frames`, `latest`, `oldest`. Optional ID accepts decimal or hex; optional
  direction `rx`, `written`, `submitted`; limit 0..1000. `after` is the global
  dashboard sequence. Sequence gaps/oldest reveal history eviction.
* `POST /api/session`: exclusive exact route registration (no wildcard ID).
  Example **simulation only**, not a meaningful device stop:

  ```json
  {
    "confirm": true, "owner": "SIM operator", "duration": 30,
    "readonly": false,
    "route": {"can_id": 1536, "channel": 0, "extended": false, "prefix": []},
    "stop_frames": [{"can_id": 1536, "channel": 0, "extended": false,
                     "data": [0,0,0,0,0,0,0,0]}]
  }
  ```

  Response: `session` (browser-only random identifier), `epoch`, `duration`,
  `readonly`, `route`. Broker capability remains private. Duration 1..300s,
  required owner 1..64 chars. Active is the default and requires 1..16 opaque
  device-safe stop frames matching the same exact ID/channel/extended/prefix.
  `readonly:true` permits no stops, but retains exclusive routing and does not
  prove the supplied payload is safe. The broker refuses existing motor-route
  overlap; the dashboard never bypasses it.
* `POST /api/heartbeat {"session":"..."}`: browser activity, expires after 1s.
  ROS heartbeats are independently published every 50ms while this browser
  session and its absolute operator duration remain live.
* `POST /api/send {"session":"...","confirm":true,"frame":{...}}`: exactly one
  frame, at most 20Hz, no automatic repeat; response `submitted`, `sequence`,
  `written:false`, `note`. Uses the claimed route, monotonic TX sequence, bus
  epoch, ROS publication stamp, monotonic activity stamp and 200ms TTL.
  `active=true` unless the claim explicitly selected read-only.
* `POST /api/release {"session":"..."}`: `ok`, `immediate:false`,
  `lease_expiry_seconds:0.3` after heartbeat activation, or `2.0` while the
  initial registration is still unconfirmed. Broker has no unregister API; local release stops
  renewal, then broker expiry clears work and dispatches opaque stops once.
  Reclaim waits for expiry. Process death follows the same expiry mechanism.
  The current broker retains revoked route reservations until same-name
  registration replacement (or a new epoch), rather than making them available
  to arbitrary clients. This dashboard uses the stable broker client name
  `can_bus_dashboard`, so later claims/restarts replace its own expired
  registration without bypassing other clients' routes. Only one live dashboard
  debug registration per bus is supported.

Frame shape: `can_id`, `channel` (currently only 0), `extended`, `data`, optional
`rtr`, `fd`, `brs` (flags default false). Error frames cannot be submitted.
Classical 0..8 bytes; FD 0..8/12/16/20/24/32/48/64, no RTR, BRS only on enabled
FD. Serial supports standard channel-0 classical 4/8-byte frames only: four
bytes are explicitly zero-padded to eight by the legacy adapter, not here.

## Security and safety

Starts monitor-only and never registers or sends automatically. Explicit claim
and send clicks submit directly; the UI includes `confirm:true` to preserve the
existing HTTP contract, which still requires this field for API clients.
No additional UI authorization checkbox or modal is required.
The claim is not a physical safety interlock; opaque
stops cannot be verified without device knowledge and failed hardware cannot
guarantee their delivery. Devices need their own watchdogs.

No login: use localhost or a **trusted network only**. Origin and Fetch-Site
must be same-origin. Host is restricted to the configured address, localhost
aliases, the machine's hostname/short name/FQDN, its short-name `.local` form,
or the accepted socket's actual local destination address, with the listening
port. Hostname comparison is case-insensitive and accepts a DNS trailing dot.
Machine names come from the server's own identity; client-supplied names are
never resolved to decide whether to trust them. This permits IP and hostname
access with a wildcard bind without trusting arbitrary domains. The client
must resolve the chosen hostname to this server, and an already running
dashboard must be restarted to load a code update. Rejections include the
received Host to make unsupported aliases or wrong ports visible.
This does not authenticate clients; secure network access and the ROS domain.
Browser session IDs are not broker capability tokens.

## Validation

The dashboard tests now run as part of `can_bus`:

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
colcon build --packages-select can_bus
source install/setup.bash
colcon test --packages-select can_bus --event-handlers console_direct+
colcon test-result --test-result-base build/can_bus
```

Unit tests check finite strict frame validation, FD/classical/serial bounds,
exact prefix matching, confirmation and bounded histories. Integration uses an
isolated ROS domain, `backend=sim`, independent opaque loopback dummy device,
HTTP monitoring through a second local address with wildcard binding,
system hostnames/FQDN/mDNS names (including case and trailing-dot variants),
same-origin hostname session requests, all three static assets, forged Host/port and
cross-origin rejection, one active `0x600` frame, observed write/RX, conflicting
existing `0x1` motor-style route rejection, privacy, release stops, read-only
reclaim and browser expiry. It never opens a physical transport.

Browser regression uses the real HTML/CSS/DOM with isolated mocked APIs and no
ROS or hardware connection. From the workspace root:

```bash
python3 -m http.server 18991 --bind 127.0.0.1 --directory src
```

Open <http://127.0.0.1:18991/dm_motor_control/test/browser.html> in Chromium.
The [shared test entry](../dm_motor_control/test/browser.html) runs both
independent dashboard suites. The [CAN suite](test/test_dashboard.js) covers
monitoring, exact epochs, filters, bounded tables and exports, all claim/send
controls, malformed input, route conflicts, slow polling with independent
heartbeats, release and late-response races, visibility/disconnection, backend
flag changes, keyboard navigation and all four views at 320 through 1440px.
These browser tests are separate from colcon and reload their source files on
every run. Do not inject them into a live hardware dashboard.
