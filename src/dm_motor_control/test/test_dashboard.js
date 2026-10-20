"use strict";

// Run in a browser with dashboard HTML, JS and CSS; all API traffic is mocked.
async function runDashboardTests(html, source, css) {
  if (!css) throw new Error("CSS is required to validate rotary wheel geometry.");
  const iframe = document.createElement("iframe");
  iframe.srcdoc = html.replace(/<script\b[^>]*>[\s\S]*?<\/script>/gi, "")
    .replace(/<link\b[^>]*>/gi, "");
  const loaded = new Promise((resolve) => iframe.addEventListener("load", resolve, {once: true}));
  document.body.appendChild(iframe);
  await loaded;
  const win = iframe.contentWindow;
  const doc = win.document;
  Object.defineProperty(doc, "hidden", {configurable: true, value: false});
  const style = doc.createElement("style");
  style.textContent = css;
  doc.head.append(style);
  iframe.style.width = "1440px";
  iframe.style.height = "1000px";
  const fields = ["position", "velocity", "kp", "kd", "torque"];
  const zero = () => Object.fromEntries(fields.map((field) => [field, 0]));
  const snapshot = {
    motor_id: 7, connected: true, has_control: false, armed: false, target: zero(),
    control_generation: 0, target_sequence: 0,
    capabilities: {ready: true, name: "motor_7", modes: [1, 2, 3, 4],
      profile: "DM", backend: "sim", registers: [
        {address: 10, name: "test_float", integer: false, unit: "rad", writable: true, raw_write_allowed: true},
        {address: 11, name: "test_integer", integer: true, unit: "", writable: false, raw_write_allowed: false}]},
    state_time: 1, state: {
      motor_id: 7, master_id: 0, phase: 0, status: 0, simulated: true, mode: 1, ready: true,
      activity_available: true, maintenance_available: true,
      position: 0.123456789, velocity: 0, torque: 0, mos_temperature: 25,
      absolute_position: 0.123456789, absolute_position_valid: true, position_wraps: 75,
      native_position: 0.123456789, direction: 1, software_offset: 0,
      position_min: -12.5, position_max: 12.5, lease_remaining: 1,
      pmax: 12.5, vmax: 30, tmax: 10,
      rotor_temperature: 26, feedback_fresh: true, feedback_count: 1,
      feedback_age: 0, period_ms: 5, max_period_ms: 5, deadline_misses: 0,
      control_rate: 200, actual_control_rate: 199.8, control_cycle_count: 100,
      position_limit: 12.5, velocity_limit: 30, kp_limit: 500, kd_limit: 1, torque_limit: 10,
    },
  };
  const setMode = mode => {
    const limit = mode === 1 ? 12.5 : 3.4028234663852886e38;
    Object.assign(snapshot.state, {mode, position_limit: limit, position_min: -limit, position_max: limit});
  };
  const requests = [];
  const errors = [];
  let nextPoll = null;
  let rejectTarget = false;
  let rejectRead = false;
  let rejectOperation = false;
  let rejectZeroRequest = false;
  let failState = false;
  let stream = null;
  let registerAddress = 10;
  const downloads = [];
  const blobs = [];
  let holdOperation = false;
  let releaseOperation = null;
  let failStop = false;
  let holdTarget = false;
  let releaseTarget = null;
  let holdSession = false;
  let releaseSession = null;
  const intervals = [];
  const resizeObservers = [];
  win.addEventListener("error", (event) => errors.push(event.message));
  win.addEventListener("unhandledrejection", (event) => errors.push(String(event.reason)));
  win.setInterval = (callback, delay) => { intervals.push({callback, delay}); return intervals.length; };
  win.setTimeout = (callback) => { nextPoll = callback; return 1; };
  win.clearTimeout = () => { nextPoll = null; };
  win.requestAnimationFrame = () => 1;
  // Native resize delivery pauses when the integrated browser is hidden.
  win.ResizeObserver = class {
    constructor(callback) { this.callback = callback; this.targets = new Set(); resizeObservers.push(this); }
    observe(element) {
      if (!(element instanceof win.Element)) throw new Error("Invalid resize-observer target");
      this.targets.add(element);
    }
    disconnect() { this.targets.clear(); }
  };
  win.EventSource = class {
    constructor() { stream = this; }
    close() {}
    addEventListener(name, callback) { this[name] = callback; }
  };
  win.URL.createObjectURL = blob => { blobs.push(blob); return "blob:dashboard-test"; };
  win.URL.revokeObjectURL = () => {};
  win.HTMLAnchorElement.prototype.click = function () { downloads.push(this.download); };
  win.fetch = async (url, options) => {
    const path = url.split("/").pop();
    requests.push({path, body: options.body && JSON.parse(options.body), headers: options.headers});
    let status = 200;
    let result = {message: "Accepted"};
    if (path === "state" && failState) throw new Error("Test connection lost");
    if (path === "stop" && failStop) throw new Error("Stop network unavailable");
    if (path === "session") {
      snapshot.has_control = true;
      result = {session: "test-session", message: "Claimed"};
    } else if (path === "enable") {
      if (!snapshot.state.activity_available) {
        status = 409; result = {error: "CAN maintenance is pending"};
      } else {
        snapshot.armed = true;
        snapshot.state.phase = 2;
        snapshot.target = zero();
      }
    } else if (path === "control-rate") {
      snapshot.state.control_rate = JSON.parse(options.body).control_rate;
      snapshot.state.actual_control_rate = snapshot.state.control_rate - 0.2;
    } else if (path === "target") {
      const generation = Number(options.headers["X-Control-Generation"]);
      const sequence = Number(options.headers["X-Target-Sequence"]);
      if (generation !== snapshot.control_generation || sequence <= snapshot.target_sequence) {
        status = 409; result = {error: "Stale target"};
      } else if (rejectTarget) { status = 400; result = {error: "Target rejected"}; }
      else {
        snapshot.target = JSON.parse(options.body);
        snapshot.target_sequence = sequence;
        result = {message: "Accepted", target: snapshot.target,
          control_generation: generation, target_sequence: sequence};
      }
    } else if (path === "registers") {
      registerAddress = JSON.parse(options.body).addresses[0];
      result = {operation_ids: [123], message: "Read queued"};
    } else if (path === "123") {
      result = {success: !rejectRead, state: rejectRead ? 2 : 1, address: registerAddress,
        raw_value: registerAddress === 10 ? 1065353216 : 7, message: rejectRead ? "Entire bus must be disabled" : "Confirmed"};
    } else if (path === "register-write") {
      registerAddress = JSON.parse(options.body).address;
      result = {operation_ids: [125], message: "Write queued"};
    } else if (path === "125") {
      result = {success: true, state: 1, address: registerAddress, raw_value: 1073741824, message: "Confirmed"};
    } else if (path === "task") {
      snapshot.control_generation += 1;
      snapshot.target_sequence = 0;
      snapshot.task = {state: "running", message: "Driver accepted task",
        feedback: {progress: 0.5, elapsed: 2.5, position_error: 0.01}};
      snapshot.state.task_active = true;
      result = {message: "Task submitted"};
    } else if (path === "operation" && rejectZeroRequest) {
      status = 409; result = {error: "Zero requires disabled motor feedback"};
    } else if (path === "operation") {
      snapshot.control_generation += 1;
      snapshot.target_sequence = 0;
      const operation = JSON.parse(options.body);
      if (!rejectOperation && operation.operation === "mode") setMode(operation.mode);
      if (!rejectOperation && operation.operation === "clear") snapshot.state.status = 0;
      result = {operation_id: operation.operation === "software_zero" ? 0 : 124,
        message: "Request sent, not yet confirmed. Check state.phase/status."};
    } else if (path === "reset") {
      snapshot.state.phase = 0;
      snapshot.state.host_fault = false;
    } else if (path === "124") {
      result = {success: !rejectOperation, state: rejectOperation ? 2 : 1,
        message: rejectOperation ? "Maintenance rejected" : "Confirmed"};
    } else if (path === "stop" || path === "release") {
      snapshot.control_generation += 1;
      snapshot.target_sequence = 0;
      snapshot.armed = false;
      snapshot.state.phase = 0;
      snapshot.target = zero();
      snapshot.state.task_active = false;
      if (snapshot.task) snapshot.task.state = "stopped";
      if (path === "release") snapshot.has_control = false;
    }
    const response = {ok: status === 200, status, json: async () => path === "state"
      ? JSON.parse(JSON.stringify(snapshot)) : result};
    if (path === "target" && holdTarget) {
      return await new Promise(resolve => { releaseTarget = () => resolve(response); });
    }
    if (path === "session" && holdSession) {
      return await new Promise(resolve => { releaseSession = () => resolve(response); });
    }
    if (["124", "125"].includes(path) && holdOperation) {
      return await new Promise(resolve => { releaseOperation = () => resolve(response); });
    }
    return response;
  };
  let checks = 0;
  const assert = (condition, message) => {
    if (!condition) throw new Error(message);
    checks += 1;
  };
  const el = (id) => doc.getElementById(id);
  const settle = async () => {
    for (let i = 0; i < 4; i += 1) {
      // Yield a task without hidden-tab timer throttling delaying the mock harness.
      await new Promise(resolve => {
        const channel = new MessageChannel();
        channel.port1.onmessage = () => {
          channel.port1.close(); channel.port2.close(); resolve();
        };
        channel.port2.postMessage(null);
      });
    }
  };
  const poll = async () => {
    const callback = nextPoll;
    nextPoll = null;
    assert(Boolean(callback), "State polling remains scheduled");
    await callback();
    await settle();
  };
  const layout = async () => {
    for (const observer of resizeObservers) if (observer.targets.size) observer.callback();
    await settle();
  };
  const singleRow = label => {
    const bar = el("status-dock").getBoundingClientRect();
    const identity = doc.querySelector(".identity").getBoundingClientRect();
    const readings = el("status-panel").getBoundingClientRect();
    const actions = doc.querySelector(".header-actions").getBoundingClientRect();
    const middle = bar.top + bar.height / 2;
    assert(identity.right <= readings.left && readings.right <= actions.left,
      `${label}: identity, readings and actions are side by side`);
    assert([identity, readings, actions].every(bounds => bounds.top <= middle && bounds.bottom >= middle),
      `${label}: all three groups share one horizontal row rather than stacked header bands`);
  };
  const edit = (id, value) => {
    el(id).value = String(value);
    el(id).dispatchEvent(new win.Event("input", {bubbles: true}));
  };
  const targetCount = () => requests.filter((request) => request.path === "target").length;
  try {
    failState = true;
    win.eval(source);
    await settle();
    assert(el("connection").textContent === "重连中" && !el("stop").disabled && el("enable").disabled,
      "Initial connection failure keeps STOP available and schedules automatic observation retry");
    failState = false;
    await poll();
    await layout();
    assert(!requests.some(request => ["session", "enable", "target"].includes(request.path)),
      "Opening the page only observes; no session, enable or target is sent");
    assert(["connect", "claim", "release", "disconnect", "quick-stop"].every(id => !el(id)),
      "Connection/session controls and duplicate STOP are removed");
    assert(!el("target-position").disabled, "Owner can prepare disabled-motor drafts");
    assert(!doc.querySelector("dialog"), "Dashboard commands have no second-confirmation dialog");
    snapshot.state.activity_available = false;
    await poll();
    assert(!el("enable").disabled, "A bus-availability snapshot does not preemptively block enable requests");
    el("enable").click(); await settle();
    assert(el("notice").textContent.includes("CAN maintenance is pending") && !snapshot.armed,
      "The actual backend rejection is shown without treating the request as enabled");
    snapshot.state.activity_available = true;
    await poll();
    const ids = [...doc.querySelectorAll("[id]")].map(element => element.id);
    assert(new Set(ids).size === ids.length, "Every dashboard HTML ID is unique");
    assert(!el("workspace").hidden && el("monitor-panel").hidden && el("maintenance-panel").hidden,
      "Primary controls and plots are visible by default, not buried under diagnostics");
    const primaryTop = el("workspace").getBoundingClientRect().top;
    assert(primaryTop < 450, `Main workspace starts within 450px of the viewport top (actual ${primaryTop})`);
    assert(el("chart").getBoundingClientRect().bottom <= win.innerHeight,
      "All three chart traces fit in the initial desktop viewport");
    assert(el("enable").getBoundingClientRect().bottom < 700,
      "Enable and STOP are reachable without scrolling the desktop");
    for (const id of ["device-zero", "software-zero"]) {
      const button = el(id), bounds = button.getBoundingClientRect();
      assert(button.closest("#control-panel") && !button.closest("details"),
        `${id} is directly accessible in the main motor controls`);
      assert(!button.disabled && bounds.height > 0 && bounds.bottom < 700,
        `${id} is ready and visible without scrolling or opening maintenance`);
      assert(button.getAttribute("aria-describedby") === "zero-operation-status" && button.title,
        `${id} explains its coordinate semantics and references inline feedback`);
    }
    assert(el("zero-operation-status")?.closest("#control-panel"),
      "Zero-operation results are shown next to the main controls");
    assert(doc.title.includes("#7") && doc.title.includes("仿真"), "Browser tab identifies the motor and environment");
    const dock = el("status-dock");
    const dockHeight = dock.getBoundingClientRect().height;
    let maxDockHeight = dockHeight;
    assert(dockHeight >= 80 && dockHeight <= 85,
      `The entire fixed header stays at the original 84px height (actual ${dockHeight})`);
    singleRow("Desktop header");
    const readoutFont = parseFloat(win.getComputedStyle(el("position")).fontSize);
    assert(readoutFont >= 26, "The single-row layout uses larger, readable desktop values");
    assert(doc.querySelector(".header-actions").contains(el("connection")),
      "Online status is visible without a connection menu");
    assert(doc.querySelector(".power-actions").contains(el("enable"))
      && doc.querySelector(".power-actions").contains(el("stop"))
      && !el("control-panel").contains(el("enable")),
      "Enable and disable are grouped exclusively in the top-right header");
    assert(el("status-panel").tabIndex === 0, "The narrow-screen telemetry scroller is keyboard focusable");
    assert(el("phase").textContent === "SIM · DISABLED" && el("phase").title.includes("SIMULATION"),
      "The compact phase label retains its complete environment and phase in the tooltip");
    assert(el("temperature").title.includes("MOS 25") && el("temperature").title.includes("转子 26"),
      "Compact dual temperatures retain their exact meaning");
    for (const id of ["position-turns", "position-degrees", "compact-position", "absolute-valid", "control-owner"]) {
      assert(el("monitor-panel").contains(el(id)) && !dock.contains(el(id)),
        `${id} remains available in full monitoring without increasing the fixed header`);
    }
    assert(doc.querySelector('meta[name="viewport"]').content.includes("interactive-widget=resizes-content"),
      "The mobile keyboard can resize the viewport instead of covering controls below the pinned status");
    assert(resizeObservers.some(observer =>
      [dock, el("plot-panel"), doc.querySelector("footer")].every(element => observer.targets.has(element))),
      "Layout observes the actual status, chart and footer dimensions");
    assert(dock.contains(el("status-panel")), "Live readings belong to the pinned top panel");
    assert(el("status-panel").getBoundingClientRect().bottom < el("health-banner").getBoundingClientRect().top,
      "Real-time status precedes scrolling notices and workspace content");
    const statusTop = el("status-panel").getBoundingClientRect().top;
    const beforeLayoutTargets = targetCount();
    win.scrollTo(0, 600);
    await settle();
    assert(win.scrollY > 0, "Pinning is tested during actual document scrolling");
    assert(Math.abs(dock.getBoundingClientRect().top) < 1
      && Math.abs(el("status-panel").getBoundingClientRect().top - statusTop) < 1,
      "The status panel stays at the same viewport position while content scrolls");
    assert(win.getComputedStyle(el("plot-panel")).position === "sticky"
      && el("plot-panel").getBoundingClientRect().top >= dock.getBoundingClientRect().bottom + 12,
      "A fitting desktop chart sticks below the complete status panel, not behind it");
    const positionBeforeScroll = snapshot.state.position;
    snapshot.state.position = 1.25;
    await poll();
    assert(el("position").textContent === "1.2500" && Math.abs(dock.getBoundingClientRect().top) < 1,
      "Pinned readings continue updating while the operator is scrolled down");
    snapshot.state.position = 1e20;
    await poll();
    iframe.style.width = "390px";
    await layout();
    assert(dock.getBoundingClientRect().height <= 85 && el("position").scrollWidth > el("position").clientWidth,
      "Very long readings truncate horizontally instead of making the fixed header taller");
    assert(el("position").title === snapshot.state.position.toFixed(4),
      "A truncated position is available at full displayed precision in its tooltip");
    iframe.style.width = "1440px";
    snapshot.state.position = positionBeforeScroll;
    await poll();
    await layout();
    const originalDockHeight = dock.getBoundingClientRect().height;
    const extraStatus = doc.createElement("div");
    extraStatus.style.height = "48px";
    el("status-panel").append(extraStatus);
    await layout();
    assert(dock.getBoundingClientRect().height > originalDockHeight
      && parseFloat(doc.documentElement.style.getPropertyValue("--status-dock-height"))
        === Math.ceil(dock.getBoundingClientRect().height),
      "Sticky offsets automatically follow status-panel height changes");
    assert(el("plot-panel").getBoundingClientRect().top >= dock.getBoundingClientRect().bottom + 12,
      "Expanding the pinned status does not cover the chart");
    extraStatus.style.height = "400px";
    await layout();
    assert(win.getComputedStyle(el("plot-panel")).position === "static",
      "An oversized header leaves the chart in document flow rather than trapping it below the viewport");
    extraStatus.remove();
    await layout();
    el("target-torque").scrollIntoView({block: "start"});
    await settle();
    assert(el("target-torque").getBoundingClientRect().top >= dock.getBoundingClientRect().bottom,
      "Scroll-into-view respects the pinned status area");
    win.scrollTo(0, 0);
    await settle();
    assert(targetCount() === beforeLayoutTargets, "Scrolling and layout updates do not send motor targets");
    for (const label of doc.querySelectorAll("label[for]")) {
      assert(Boolean(el(label.htmlFor)), `Label ${label.htmlFor} targets an input`);
    }
    el("tab-control").dispatchEvent(new win.KeyboardEvent("keydown", {key: "ArrowRight", bubbles: true}));
    assert(!el("task-panel").hidden && doc.activeElement === el("tab-task"), "Arrow keys switch and focus workspace tabs");
    el("tab-task").dispatchEvent(new win.KeyboardEvent("keydown", {key: "End", bubbles: true}));
    assert(!el("events-panel").hidden, "End opens the final tab");
    el("tab-events").dispatchEvent(new win.KeyboardEvent("keydown", {key: "Home", bubbles: true}));
    assert(!el("workspace").hidden, "Home returns to the main workspace");
    assert(el("mode-select").closest("#control-panel") && !el("set-mode"),
      "Drive mode selection is in motor controls without a separate submission button");
    assert(el("mode-operation-status").closest("#control-panel"), "Mode confirmation is shown beside motor controls");
    assert(el("target-velocity_limit").disabled && el("target-current_ratio").disabled
      && !el("target-velocity_limit").parentElement.hidden, "MIT-inapplicable parameters remain visible but disabled");
    assert(el("telemetry").textContent.includes("连续位置有效"), "Continuous position validity shown");
    assert(el("telemetry").textContent.includes("回绕 75 次"), "Signed wrap count shown");
    assert(el("position-turns").textContent !== "--", "Continuous turns shown");
    assert(el("position-degrees").textContent.endsWith("°"), "Angle in degrees shown");
    assert(el("compact-position").textContent === "0.123457", "Raw compact position is distinct");
    el("tab-monitor").click();
    assert(el("position-turns").getBoundingClientRect().height > 0
      && el("control-owner").getBoundingClientRect().height > 0, "Coordinate and control details are visible in full monitoring");
    el("tab-control").click();
    assert(el("position-data").textContent.includes("原生连续位置"), "Coordinate diagnostics exposed");
    assert(el("timing-data").textContent.includes("P50"), "Timing percentile diagnostics exposed");
    assert(el("live-targets").checked, "Event-driven manual control is enabled by default");
    assert(el("control-rate-status").textContent.includes("199.8"), "Actual frequency is distinct from configured frequency");
    el("control-rate").closest("details").open = true;
    const rateRequests = requests.filter(r => r.path === "control-rate").length;
    for (const value of ["", "19", "1001"]) {
      el("control-rate").value = value;
      el("control-rate").dispatchEvent(new win.Event("change"));
      await settle();
      assert(requests.filter(r => r.path === "control-rate").length === rateRequests
        && el("notice").textContent.includes("控制频率"), `Invalid rate '${value}' is rejected before HTTP`);
    }
    el("control-rate").value = "100";
    el("control-rate").dispatchEvent(new win.Event("change"));
    await settle();
    await poll();
    assert(requests.some(r => r.path === "control-rate" && r.body.control_rate === 100),
      "Runtime frequency change is submitted immediately");
    assert(el("control-rate-status").textContent.includes("99.8"), "Measured frequency updates from driver state");
    el("control-rate").closest("details").open = false;
    assert(!intervals.some(i => i.delay === 50), "No periodic target sender remains");
    assert(intervals.filter(i => i.delay === 20).length === 1, "Chart has one independent 50 Hz render timer");
    assert(Number(el("target-position").value) === snapshot.state.position, "Position defaults to current continuous feedback");
    assert(el("position-scroll").getAttribute("role") === "spinbutton", "Position uses an accessible rotary wheel, not a scrollbar");
    assert(!doc.querySelector(".position-scroll-track"), "No scrollbar track or sliding thumb remains");
    for (const field of fields.filter(field => field !== "position")) {
      assert(el(`slider-${field}`).min === el(`target-${field}`).min, `${field} minimum matches`);
      assert(el(`slider-${field}`).max === el(`target-${field}`).max, `${field} maximum matches`);
    }
    assert(el("target-kd").max === "1", "Kd numeric maximum follows application cap");
    assert(el("slider-kd").max === "1", "Kd slider maximum follows application cap");
    const gainDraftRequests = targetCount();
    for (const [value, code, decoded] of [[0.01, 0, "0.000000"], [0.1221, 0, "0.000000"],
      [0.13, 1, "0.122100"], [1, 8, "0.976801"]]) {
      edit("target-kp", value);
      assert(!el("encoding-kp").hidden && el("encoding-kp").textContent.includes(`码 ${code}/4095`)
        && el("encoding-kp").textContent.includes(decoded), `Kp ${value} shows its actual 12-bit encoding`);
      assert(el("encoding-kp").classList.contains("warning") === (code === 0),
        "A positive Kp that disappears in encoding is explicitly highlighted");
      assert(Number(el("target-kp").value) === value, "The encoding hint never raises or rewrites a gain");
    }
    edit("target-kd", 0.2);
    assert(el("encoding-kd").textContent.includes("0.199023") && el("encoding-kd").textContent.includes("码 163/4095"),
      "Kd uses the protocol 0..5 scale, not its application cap");
    edit("target-kd", 0.0001);
    assert(el("encoding-kd").classList.contains("warning") && el("encoding-kd").textContent.includes("速度阻尼"),
      "Sub-resolution damping is not mistaken for a nonzero gain");
    edit("target-kp", "");
    assert(el("encoding-kp").textContent.includes("待输入有效数值")
      && !el("encoding-kp").classList.contains("warning"), "An empty gain is not silently interpreted as a zero-gain command");
    snapshot.state.kp_limit = 50;
    snapshot.state.kd_limit = 0.5;
    await poll();
    edit("target-kp", 1);
    edit("target-kd", 0.2);
    assert(el("encoding-kp").textContent.includes("码 8/4095")
      && el("encoding-kd").textContent.includes("码 163/4095"), "Changing application limits never rescales MIT gains");
    snapshot.target.kp = 0.01; snapshot.target.kd = 0.2;
    await poll();
    assert(el("applied-kp").textContent.includes("服务目标：0.01")
      && el("applied-kp").textContent.includes("0.000000"), "The service's unquantized Kp is distinguished from its predicted wire value");
    snapshot.target = zero();
    snapshot.state.kp_limit = 500;
    snapshot.state.kd_limit = 1;
    edit("target-kp", 0); edit("target-kd", 0);
    await poll();
    assert(targetCount() === gainDraftRequests, "Encoding previews neither enable nor send a target");
    snapshot.state.absolute_position_valid = false;
    await poll();
    assert(el("enable").disabled && el("fill-position").disabled, "Invalid absolute position cannot arm or fill");
    assert(el("telemetry").textContent.includes("连续位置无效"), "Invalid continuous position is explicit");
    snapshot.state.absolute_position_valid = true;
    await poll();
    edit("target-position", 0.314159265);
    assert(Number(el("position-scroll").getAttribute("aria-valuenow")) === 0.314159265, "Numeric precision preserved");
    const scroll = el("position-scroll");
    const captures = new Set();
    scroll.setPointerCapture = id => captures.add(id);
    scroll.hasPointerCapture = id => captures.has(id);
    scroll.releasePointerCapture = id => captures.delete(id);
    const wheelPointer = (type, angle, id = 1, radiusScale = 0.42) => {
      const bounds = scroll.getBoundingClientRect();
      const radius = Math.min(bounds.width, bounds.height) * radiusScale;
      scroll.dispatchEvent(new win.PointerEvent(type, {
        pointerId: id, pointerType: "mouse", button: 0,
        buttons: type === "pointerup" ? 0 : 1, bubbles: true, cancelable: true,
        clientX: bounds.left + bounds.width / 2 + Math.cos(angle) * radius,
        clientY: bounds.top + bounds.height / 2 + Math.sin(angle) * radius,
      }));
    };
    const rotateWheel = direction => {
      wheelPointer("pointerdown", -Math.PI / 2);
      for (let step = 1; step <= 24; step += 1) {
        wheelPointer("pointermove", -Math.PI / 2 + direction * step * Math.PI / 12);
      }
      const value = Number(el("target-position").value);
      const orientation = el("position-wheel-rotor").style.transform;
      wheelPointer("pointerup", -Math.PI / 2 + direction * 2 * Math.PI);
      assert(Math.abs(Number(el("target-position").value) - value) < 1e-9,
        "Releasing the wheel preserves the accumulated target");
      assert(el("position-wheel-rotor").style.transform === orientation, "Wheel orientation does not spring back on release");
    };
    assert(Math.abs(scroll.getBoundingClientRect().width - scroll.getBoundingClientRect().height) < 1,
      "Rotary wheel is circular, not a linear slider");
    assert(win.getComputedStyle(scroll).overflowX !== "scroll", "Wheel does not use native scrollbar overflow");
    const disabledPosition = Number(el("target-position").value);
    wheelPointer("pointerdown", -Math.PI / 2);
    assert(scroll.classList.contains("dragging") && captures.has(1), "Wheel captures active pointer for continuous rotation");
    wheelPointer("pointermove", 0);
    wheelPointer("pointerup", 0);
    assert(Math.abs(Number(el("target-position").value) - disabledPosition - Math.PI / 2) < 1e-9,
      "Quarter wheel turn continuously adds a quarter motor turn");
    assert(!captures.size && !scroll.classList.contains("dragging"), "Pointer released cleanly without resetting target");
    const beforeBranch = Number(el("target-position").value);
    wheelPointer("pointerdown", 170 * Math.PI / 180);
    wheelPointer("pointermove", -170 * Math.PI / 180);
    wheelPointer("pointerup", -170 * Math.PI / 180);
    assert(Math.abs(Number(el("target-position").value) - beforeBranch - 20 * Math.PI / 180) < 1e-9,
      "Pointer angle branch crossing adds 20 degrees, not a 340-degree jump");
    const beforeCenter = Number(el("target-position").value);
    wheelPointer("pointerdown", 0, 1, 0);
    wheelPointer("pointermove", Math.PI / 2);
    wheelPointer("pointerup", Math.PI / 2);
    assert(Number(el("target-position").value) === beforeCenter, "Center click does not create an ambiguous angular jump");
    wheelPointer("pointerdown", 0);
    wheelPointer("pointermove", Math.PI / 2, 2);
    assert(Number(el("target-position").value) === beforeCenter, "A different pointer cannot steal the active rotation");
    scroll.dispatchEvent(new win.PointerEvent("pointercancel", {pointerId: 1}));
    wheelPointer("pointermove", Math.PI / 2);
    assert(!captures.size && Number(el("target-position").value) === beforeCenter,
      "Cancelled pointer gesture cannot continue adjusting targets");
    scroll.dispatchEvent(new win.KeyboardEvent("keydown", {key: "ArrowRight", bubbles: true}));
    assert(targetCount() === 0, "Disabled wheel rotation and keyboard changes never send targets");
    const beforeBounds = Number(el("target-position").value);
    edit("target-position", 12.49);
    wheelPointer("pointerdown", 0);
    wheelPointer("pointermove", 1);
    assert(Number(el("target-position").value) === 12.5, "MIT pointer motion stops exactly at the upper bound");
    const upperOrientation = el("position-wheel-rotor").style.transform;
    wheelPointer("pointermove", 2);
    assert(el("position-wheel-rotor").style.transform === upperOrientation, "Pointer overshoot cannot rotate the ring beyond its upper bound");
    wheelPointer("pointermove", 1.9);
    assert(Math.abs(Number(el("target-position").value) - 12.4) < 1e-9,
      "Reversing a pointer immediately leaves the limit without unwinding overshoot");
    wheelPointer("pointerup", 1.9);
    edit("target-position", -12.49);
    scroll.dispatchEvent(new win.WheelEvent("wheel", {deltaY: -1000, bubbles: true, cancelable: true}));
    assert(Number(el("target-position").value) === -12.5, "MIT mouse wheel stops exactly at the lower bound");
    const lowerOrientation = el("position-wheel-rotor").style.transform;
    scroll.dispatchEvent(new win.KeyboardEvent("keydown", {key: "ArrowLeft", bubbles: true}));
    assert(Number(el("target-position").value) === -12.5
      && el("position-wheel-rotor").style.transform === lowerOrientation, "Outward keys cannot rotate beyond the lower bound");
    scroll.dispatchEvent(new win.KeyboardEvent("keydown", {key: "ArrowRight", bubbles: true}));
    assert(Math.abs(Number(el("target-position").value) + 12.49) < 1e-9, "An inward key immediately leaves the lower bound");
    assert(scroll.getAttribute("aria-valuemin") === "-12.5" && scroll.getAttribute("aria-valuemax") === "12.5",
      "Accessible wheel bounds match the MIT command range");
    edit("target-position", 13);
    assert(Number(scroll.getAttribute("aria-valuenow")) === 12.5 && el("target-position").value === "13",
      "Invalid numeric drafts remain unchanged while the wheel itself stops at its bound");
    scroll.dispatchEvent(new win.KeyboardEvent("keydown", {key: "ArrowRight", bubbles: true}));
    assert(el("target-position").value === "13", "Outward input at the displayed bound does not silently apply an invalid draft");
    assert(targetCount() === 0, "Unarmed boundary handling never sends motion targets");
    edit("target-position", beforeBounds);
    edit("slider-kd", 0.25);
    assert(Number(el("target-kd").value) === 0.25, "Slider updates numeric draft");
    assert(targetCount() === 0, "Draft edits never send targets");
    assert(el("draft-status").textContent.includes("2 项"), "Dirty fields counted");
    el("fill-position").click();
    assert(Number(el("target-position").value) === snapshot.state.position, "Fill uses fresh position");
    assert(Number(el("target-kd").value) === 0.25, "Fill preserves other fields");
    assert(targetCount() === 0, "Fill never sends");
    el("restore-target").click();
    assert(fields.every((field) => Number(el(`target-${field}`).value) === 0), "Restore uses service target");
    el("live-targets").click();
    edit("target-torque", 0.1);
    el("enable").click();
    await settle();
    await poll();
    assert(fields.filter(field => field !== "position").every(field => Number(el(`target-${field}`).value) === 0),
      "Enable clears non-position drafts");
    assert(Number(el("target-position").value) === snapshot.state.position, "Enable starts position at current feedback, not zero");
    const enabledZeroRequests = requests.filter(r => r.path === "operation").length;
    assert(el("device-zero").disabled && el("software-zero").disabled,
      "Both zero controls preserve the driver's disabled-state requirement");
    el("device-zero").click(); el("software-zero").click(); await settle();
    assert(requests.filter(r => r.path === "operation").length === enabledZeroRequests && snapshot.armed,
      "Unavailable zero controls neither submit operations nor automatically stop the motor");
    edit("target-kd", 0.2);
    edit("target-torque", 0.1);
    el("apply").click();
    await settle();
    assert(targetCount() === 1, "Explicit application sends once");
    assert(Object.keys(requests.find((r) => r.path === "target").body).sort().join() === fields.slice().sort().join(),
      "Application contains exactly all five MIT fields");
    await poll();
    assert(el("applied-torque").textContent === "服务目标：0.1", "Applied target follows server state");
    assert(el("draft-status").textContent.includes("草稿与服务目标一致"), "Accepted draft no longer dirty");
    edit("target-kd", 1.01);
    el("apply").click();
    await settle();
    assert(targetCount() === 1, "Kd above application maximum is not sent");
    assert(el("target-kd").value === "1.01", "Invalid Kd is not silently clamped");
    edit("target-kd", 0.2);
    edit("target-torque", 11);
    el("apply").click();
    await settle();
    assert(targetCount() === 1, "Out-of-range target not sent");
    assert(el("target-torque").value === "11", "Invalid numeric input is not silently clamped");
    assert(el("slider-torque").value === "10", "Only slider presentation is bounded");
    edit("target-torque", "");
    el("apply").click();
    await settle();
    assert(targetCount() === 1, "Empty target not sent");
    edit("target-torque", 0.2);
    rejectTarget = true;
    el("apply").click();
    await settle();
    assert(el("notice").textContent === "Target rejected", "API rejection surfaced");
    assert(el("applied-torque").textContent === "服务目标：0.1", "Rejection does not change applied target");
    rejectTarget = false;
    el("zero").click();
    await settle();
    await poll();
    assert(fields.every((field) => snapshot.target[field] === 0), "Zero applies all five zeros");
    edit("target-position", 1);
    el("stop").click();
    await settle();
    await poll();
    assert(fields.filter(field => field !== "position").every(field => Number(el(`target-${field}`).value) === 0),
      "STOP clears non-position drafts");
    assert(Number(el("target-position").value) === snapshot.state.position, "STOP restores current position baseline");
    assert(el("mode-select").options.length === 4, "Advertised modes are listed");
    el("mode-select").value = "2";
    el("mode-select").dispatchEvent(new win.Event("change"));
    assert(requests.some(request => request.path === "operation" && request.body.mode === 2),
      "Mode selection immediately submits the switch without a confirmation");
    assert(!doc.querySelector("dialog"), "Mode switching has no confirmation dialog");
    assert(el("enable").disabled && el("target-position").disabled && el("target-kp").disabled,
      "Pending mode selection cannot enable or edit targets before driver confirmation");
    assert(el("mode-status").textContent.includes("等待驱动确认"), "Pending driver confirmation is explicit");
    await settle();
    await poll();
    assert(snapshot.state.mode === 2 && snapshot.state.phase === 0 && !snapshot.armed,
      "Explicit mode operation stays disabled and unarmed");
    assert(el("mode-operation-status").textContent.includes("#124")
      && el("mode-operation-status").textContent.includes("已确认"), "Mode operation confirmation is polled");
    assert(!el("target-position").disabled && !el("target-velocity_limit").disabled
      && el("target-kp").disabled && el("target-kd").disabled, "Confirmed position mode immediately enables only its parameters");
    const switched = requests.filter(request => request.path === "operation").length;
    el("mode-select").dispatchEvent(new win.Event("change"));
    await settle();
    assert(requests.filter(request => request.path === "operation").length === switched,
      "Selecting the already active mode does not repeat configuration");
    el("tab-maintenance").click();
    el("read-parameter").click();
    await settle();
    assert(el("parameter-value").textContent.includes("test_float: 1 rad"), "Async parameter read decodes float32");
    const stopsBeforeRead = requests.filter((request) => ["stop", "release"].includes(request.path)).length;
    snapshot.state.maintenance_available = false;
    await poll();
    assert(!el("read-parameter").disabled && !el("mode-select").disabled && !el("save-parameters").disabled,
      "Maintenance availability snapshots do not preemptively block parameter or configuration requests");
    rejectRead = true;
    el("read-parameter").click();
    await settle();
    assert(el("parameter-value").textContent.includes("已拒绝")
      && el("notice").textContent.includes("Entire bus must be disabled"), "Queued read rejection is explicit");
    assert(requests.filter((request) => ["stop", "release"].includes(request.path)).length === stopsBeforeRead,
      "Read rejection does not trigger STOP or release");
    rejectRead = false;
    snapshot.state.maintenance_available = true;
    await poll();
    assert(el("parameter-results").children.length === 1, "Confirmed reads retained in parameter table");
    edit("parameter-input", 2);
    holdOperation = true;
    el("write-parameter").click();
    await settle();
    assert(requests.filter(r => r.path === "register-write").length === 1
      && requests.some(r => r.path === "register-write" && r.body.value === 2),
      "A single parameter-write click submits immediately without a confirmation step");
    assert(el("write-parameter").disabled && !el("stop").disabled,
      "Only duplicate writes are blocked while device readback is pending; STOP remains available");
    const pendingOperationStops = requests.filter(r => r.path === "stop").length;
    el("stop").click();
    await settle();
    assert(requests.filter(r => r.path === "stop").length === pendingOperationStops + 1,
      "Header STOP submits immediately while a write is awaiting device readback");
    doc.dispatchEvent(new win.KeyboardEvent("keydown", {key: "Escape", bubbles: true}));
    await settle();
    assert(requests.filter(r => r.path === "stop").length === pendingOperationStops + 2, "Escape explicitly requests disable");
    holdOperation = false;
    releaseOperation();
    await settle();
    assert(el("parameter-value").textContent.includes("test_float: 2 rad"), "Independent write readback decoded");
    el("parameter-select").value = "11";
    el("parameter-select").dispatchEvent(new win.Event("change"));
    assert(el("write-parameter").disabled && el("parameter-input").disabled, "Read-only parameter cannot be written");
    el("parameter-select").value = "10";
    el("parameter-select").dispatchEvent(new win.Event("change"));
    el("read-all-parameters").click();
    await settle();
    assert(el("parameter-results").children.length === 2, "Read-all confirms both advertised parameters");
    el("export-parameters").click();
    assert(downloads.includes("motor-parameters.json"), "Confirmed parameters can be exported");
    for (const [id, operation] of [["device-zero", "device_zero"], ["software-zero", "software_zero"],
      ["save-parameters", "save"]]) {
      const isZero = operation !== "save";
      el(isZero ? "tab-control" : "tab-maintenance").click();
      const output = el(isZero ? "zero-operation-status" : "operation-status");
      const before = requests.filter(r => r.path === "operation").length;
      const motionBefore = requests.filter(r => ["target", "enable", "stop"].includes(r.path)).length;
      el(id).click(); await settle();
      assert(requests.filter(r => r.path === "operation").length === before + 1
        && requests.filter(r => r.path === "operation").at(-1).body.operation === operation,
        `${operation} submits exactly once on the initial click`);
      assert(output.textContent.includes(operation === "software_zero" ? "无异步操作 ID" : "已确认")
        && snapshot.state.phase === 0, `${operation} displays its actual completion contract without enabling`);
      assert(output.getBoundingClientRect().height > 0, `${operation} reports its result in the visible workspace`);
      assert(requests.filter(r => ["target", "enable", "stop"].includes(r.path)).length === motionBefore,
        `${operation} does not send a zero target, enable, or implicit STOP`);
      if (operation === "software_zero") assert(!output.textContent.includes("已确认")
        && !requests.some(r => r.path === "0"), "A zero operation ID is not falsely confirmed or polled");
    }
    el("tab-control").click();
    const zeroRequests = requests.filter(r => r.path === "operation").length;
    holdOperation = true;
    el("device-zero").click(); await settle();
    assert(el("zero-operation-status").textContent.includes("提交中")
      && el("device-zero").disabled && el("software-zero").disabled && !el("stop").disabled,
      "A pending zero operation has visible progress and prevents duplicate submissions without blocking STOP");
    el("device-zero").click(); el("software-zero").click(); await settle();
    assert(requests.filter(r => r.path === "operation").length === zeroRequests + 1,
      "Repeated zero clicks while awaiting confirmation do not send additional operations");
    const beforeZeroPosition = snapshot.state.position;
    snapshot.state.position = 0;
    holdOperation = false;
    releaseOperation(); await settle();
    assert(el("zero-operation-status").textContent.includes("已确认") && el("position").textContent === "0.0000",
      "Confirmed zero operations refresh the displayed position from the driver state");
    snapshot.state.position = beforeZeroPosition;
    await poll();
    rejectOperation = true;
    el("device-zero").click(); await settle();
    assert(el("zero-operation-status").textContent.includes("已拒绝")
      && el("notice").textContent.includes("Maintenance rejected"),
      "A driver-rejected device-zero operation is explicit beside the button and in global feedback");
    rejectOperation = false;
    rejectZeroRequest = true;
    el("software-zero").click(); await settle();
    assert(el("zero-operation-status").textContent.includes("Zero requires disabled motor feedback")
      && !el("zero-operation-status").textContent.includes("已确认")
      && el("notice").textContent.includes("Zero requires disabled motor feedback"),
      "An HTTP-rejected zero request replaces any earlier success with its actual failure");
    rejectZeroRequest = false;
    el("software-zero").click(); await settle();
    assert(el("zero-operation-status").textContent.includes("无异步操作 ID") && !snapshot.armed,
      "A corrected explicit zero click can succeed without reconnecting or enabling");
    el("tab-maintenance").click();
    snapshot.state.phase = 4; snapshot.state.status = 8; snapshot.state.device_fault = true;
    await poll();
    el("clear-fault").click(); await settle();
    assert(requests.some(r => r.path === "operation" && r.body.operation === "clear"),
      "Device fault clearing is exposed independently of host reset");
    snapshot.state.device_fault = false;
    await poll();
    el("tab-control").click();
    assert(!el("reset").disabled, "Host reset becomes available for a cleared device fault");
    el("reset").click(); await settle(); await poll();
    assert(requests.some(r => r.path === "reset") && snapshot.state.phase === 0, "Host reset confirms disabled state");
    rejectOperation = true;
    el("mode-select").value = "3";
    el("mode-select").dispatchEvent(new win.Event("change"));
    await settle();
    assert(snapshot.state.mode === 2 && el("mode-operation-status").textContent.includes("已拒绝"),
      "Rejected queued mode operation is never labelled applied");
    assert(el("mode-select").value === "2" && !el("target-position").disabled && el("target-kp").disabled,
      "Failed switching restores actual mode controls");
    rejectOperation = false;
    for (const [mode, expected, values] of [
      [2, ["position", "velocity_limit"], {position: 0.2, velocity_limit: 1}],
      [3, ["velocity"], {velocity: -0.5}],
      [4, ["position", "velocity_limit", "current_ratio"], {position: 0.3, velocity_limit: 1, current_ratio: 0.5}],
    ]) {
      setMode(mode);
      snapshot.target = {...zero(), velocity_limit: 0, current_ratio: 0};
      await poll();
      const before = targetCount();
      assert(fields.concat(["velocity_limit", "current_ratio"]).every((field) =>
        !el(`target-${field}`).parentElement.hidden
        && el(`target-${field}`).disabled === !expected.includes(field)), `Mode ${mode} enables only its applicable fields`);
      assert(fields.concat(["velocity_limit", "current_ratio"]).filter(field => field !== "position").every(field =>
        el(`slider-${field}`).disabled === !expected.includes(field)), `Mode ${mode} also gates matching sliders`);
      assert((scroll.getAttribute("aria-disabled") === "true") === !expected.includes("position"),
        `Mode ${mode} gates the position wheel`);
      assert(el("encoding-kp").hidden && el("encoding-kd").hidden
        && !el("applied-kp").textContent.includes("MIT"), "Non-MIT modes do not display misleading gain predictions");
      if (mode === 3) assert(el("target-velocity").getBoundingClientRect().top < scroll.getBoundingClientRect().top,
        "Velocity mode prioritizes its usable input ahead of the disabled position wheel");
      el("enable").click();
      await settle();
      await poll();
      for (const [field, value] of Object.entries(values)) edit(`target-${field}`, value);
      assert(targetCount() === before, `Mode ${mode} editing does not send`);
      el("apply").click();
      await settle();
      assert(targetCount() === before + 1, `Mode ${mode} explicit target is sent`);
      const applied = requests.filter((request) => request.path === "target").at(-1).body;
      assert(Object.keys(applied).sort().join() === expected.slice().sort().join(),
        `Mode ${mode} sends only its typed fields`);
      assert(Object.entries(values).every(([field, value]) => applied[field] === value),
        `Mode ${mode} preserves target signs and values`);
      if (mode === 4) {
        edit("target-current_ratio", 1.01);
        el("apply").click();
        await settle();
        assert(targetCount() === before + 1, "Current ratio greater than 1 is rejected locally");
      }
      el("stop").click();
      await settle();
      await poll();
    }
    for (const mode of [2, 4]) {
      setMode(mode);
      snapshot.target = {...zero(), velocity_limit: 0.2, current_ratio: mode === 4 ? 0.1 : 0};
      await poll();
      const start = Number(el("target-position").value);
      const before = targetCount();
      for (let i = 0; i < 8; i += 1) {
        rotateWheel(1);
      }
      assert(Math.abs(Number(el("target-position").value) - start - 16 * Math.PI) < 1e-8,
        `Mode ${mode} eight clockwise revolutions accurately accumulate eight turns`);
      for (let i = 0; i < 16; i += 1) {
        rotateWheel(-1);
      }
      assert(Math.abs(Number(el("target-position").value) - start + 16 * Math.PI) < 1e-8,
        `Mode ${mode} counterclockwise revolutions continue through zero to negative multi-turn targets`);
      const beforeWheel = Number(el("target-position").value);
      for (let i = 0; i < 32; i += 1) {
        scroll.dispatchEvent(new win.WheelEvent("wheel", {deltaY: 1000, bubbles: true, cancelable: true}));
      }
      assert(Math.abs(Number(el("target-position").value) - beforeWheel - 32) < 1e-8,
        `Mode ${mode} wheel has no track endpoint and accumulates every delta`);
      assert(targetCount() === before, "Unarmed infinite scrolling never sends a motion target");
      assert(el("position-scroll-min").textContent.includes("下限") &&
        el("position-scroll-max").textContent.includes("上限"), "Wheel displays the current mode limits");
      assert(Number(scroll.getAttribute("aria-valuemax")) > 1e30, "Float position modes do not inherit the MIT two-turn bound");
    }
    setMode(2);
    snapshot.state.position_min = -1;
    snapshot.state.position_max = 1;
    snapshot.target = {...zero(), velocity_limit: 0.2, current_ratio: 0};
    await poll();
    el("enable").click();
    await settle();
    await poll();
    el("apply").click();
    await settle();
    const beforeLimit = targetCount();
    scroll.dispatchEvent(new win.WheelEvent("wheel", {deltaY: 10000, bubbles: true, cancelable: true}));
    await settle();
    assert(Number(el("target-position").value) === 1, "Wheel stops at an explicit mechanical limit");
    assert(targetCount() === beforeLimit && el("position-scroll-status").textContent.includes("已到上限"),
      "Draft-only boundary handling does not send a target");
    el("stop").click();
    await settle();
    await poll();
    setMode(1);
    snapshot.target = zero();
    await poll();
    el("enable").click();
    await settle();
    await poll();
    edit("target-kp", 5);
    edit("target-kd", 0.3);
    el("apply").click();
    await settle();
    await poll();
    el("live-targets").click();
    await settle();
    const beforeGainEdit = targetCount();
    edit("target-kp", 9);
    await settle();
    assert(targetCount() === beforeGainEdit + 1 && snapshot.target.kp === 9,
      "Enabled Kp edits reach the backend without Apply or a timer");
    const tinyGainTargets = targetCount();
    edit("target-kp", 0.01);
    await settle();
    assert(targetCount() === tinyGainTargets + 1 && snapshot.target.kp === 0.01
      && el("encoding-kp").classList.contains("warning"), "A tiny gain is still submitted unchanged, with a nonblocking encoding warning");
    assert(el("position-scroll-status").textContent.includes("Kp 编码为 0"),
      "Position control explains a zero encoded gain even when the requested Kp is positive");
    edit("target-kp", 9);
    await settle();
    assert(!el("encoding-kp").classList.contains("warning"), "The quantization warning clears after an explicit representable edit");
    const restorePosition = Number(el("target-position").value);
    edit("target-position", 12.49);
    await settle();
    scroll.dispatchEvent(new win.WheelEvent("wheel", {deltaY: 1000, bubbles: true, cancelable: true}));
    await settle();
    assert(snapshot.target.position === 12.5, "Live wheel sends the boundary target rather than dropping an overshoot");
    const atUpperLimit = targetCount();
    scroll.dispatchEvent(new win.WheelEvent("wheel", {deltaY: 1000, bubbles: true, cancelable: true}));
    scroll.dispatchEvent(new win.KeyboardEvent("keydown", {key: "ArrowRight", bubbles: true}));
    await settle();
    assert(targetCount() === atUpperLimit, "Repeated outward input sends no duplicate boundary targets");
    edit("target-position", restorePosition);
    await settle();
    const beforeScroll = targetCount();
    const movePosition = key => scroll.dispatchEvent(new win.KeyboardEvent(
      "keydown", {key, bubbles: true, cancelable: true}));
    holdTarget = true;
    movePosition("ArrowRight");
    for (let i = 0; i < 10; i += 1) movePosition("ArrowRight");
    const finalDragPosition = Number(el("target-position").value);
    assert(targetCount() === beforeScroll + 1, "Live position updates allow only one in-flight request");
    const firstScrollTarget = requests.filter(r => r.path === "target").at(-1);
    assert(firstScrollTarget.body.kp === 9 && firstScrollTarget.body.kd === 0.3,
      "Live scrolling carries the latest edited control parameters");
    assert(el("target-kp").value === "9", "Latest gain input is preserved while scrolling");
    holdTarget = false;
    releaseTarget();
    await settle();
    await settle();
    assert(targetCount() === beforeScroll + 2, "Rapid scroll events are coalesced instead of queued individually");
    assert(requests.filter(r => r.path === "target").at(-1).body.position === finalDragPosition,
      "Final wheel position is eventually delivered");
    const currentTarget = snapshot.target, currentSequence = snapshot.target_sequence;
    snapshot.target = {...snapshot.target, position: -1};
    snapshot.target_sequence -= 1;
    await poll();
    assert(!el("applied-position").textContent.endsWith("：-1"), "Older state polling cannot roll back an accepted target");
    snapshot.target = currentTarget;
    snapshot.target_sequence = currentSequence;
    rejectTarget = true;
    movePosition("ArrowRight");
    await settle();
    assert(el("notice").textContent.includes("Target rejected"), "Rejected live targets show the backend reason");
    const rejectedCount = targetCount();
    await settle();
    assert(targetCount() === rejectedCount, "Rejected targets are not automatically retried");
    rejectTarget = false;
    movePosition("ArrowRight");
    await settle();
    assert(targetCount() === rejectedCount + 1 && snapshot.target.position === Number(el("target-position").value),
      "The next explicit live edit submits directly without an extra Apply step");
    assert(el("notice").textContent === "Accepted", "Successful correction clears the obsolete rejection notice");
    rejectTarget = true;
    holdTarget = true;
    movePosition("ArrowRight");
    rejectTarget = false;
    movePosition("ArrowRight");
    const correctedPosition = Number(el("target-position").value);
    holdTarget = false;
    releaseTarget();
    await settle();
    assert(snapshot.target.position === correctedPosition,
      "A newer explicit correction is not discarded when an earlier in-flight target is rejected");
    await poll();
    holdTarget = true;
    wheelPointer("pointerdown", -Math.PI / 2);
    wheelPointer("pointermove", -Math.PI / 2 + 0.01);
    wheelPointer("pointermove", -Math.PI / 2 + 0.02);
    const beforeStop = targetCount();
    el("stop").click();
    await settle();
    assert(!captures.size && !scroll.classList.contains("dragging"), "STOP releases a captured rotary gesture");
    await poll();
    holdTarget = false;
    releaseTarget();
    await settle();
    await settle();
    assert(targetCount() === beforeStop && !snapshot.armed, "STOP discards pending scroll targets");
    el("enable").click();
    await settle();
    await poll();
    assert(targetCount() === beforeStop, "Re-enable does not replay an earlier drag");
    wheelPointer("pointermove", -Math.PI / 2 + 0.03);
    assert(targetCount() === beforeStop, "Old pointer movement cannot restart a gesture after re-enable");
    const beforeCentering = Number(el("target-position").value);
    movePosition("End");
    movePosition("Home");
    assert(Number(el("target-position").value) === beforeCentering, "Home/End do not reset or recenter the wheel target");
    el("stop").click();
    await settle();
    await poll();
    el("enable").click();
    await settle();
    await poll();
    el("tab-task").click();
    el("task-kind").value = "sine";
    el("task-kind").dispatchEvent(new win.Event("change"));
    el("task-frequency").value = "-1";
    el("task-kind").value = "torque";
    el("task-kind").dispatchEvent(new win.Event("change"));
    assert(el("task-frequency").disabled && el("task-form").checkValidity(),
      "Invalid hidden fields from another task kind do not block native form submission");
    const preparationTargets = targetCount();
    el("prepare-task").click();
    assert(!el("live-targets").checked && !el("workspace").hidden && targetCount() === preparationTargets,
      "Task preparation switches to drafts without transmitting, enabling or stopping");
    edit("target-torque", 0.02);
    const beforeTaskTargets = targetCount();
    el("tab-task").click();
    el("start-task").click();
    await settle();
    await poll();
    const task = requests.find(r => r.path === "task").body;
    assert(task.kind === "torque" && task.target.torque === 0.02, "Task uses explicit current target draft");
    assert(task.amplitude === 0 && task.frequency === 0 && task.ramp_time > 0, "Only applicable task fields enabled");
    assert(!el("apply").disabled && !el("cancel-task").disabled, "Task can be replaced by manual output or cancelled");
    assert(el("position-scroll").getAttribute("aria-disabled") === "false", "Tasks do not own the manual input path");
    assert(el("task-progress").value === 0.5, "Task progress displayed");
    assert(fields.every(field => !el(`target-${field}`).disabled), "Running tasks do not exclude manual command editing");
    el("tab-control").click();
    assert(!el("task-shortcut").hidden, "An active task remains visible outside its tab");
    el("task-shortcut").click();
    assert(!el("task-panel").hidden, "Task shortcut exposes progress and cancellation");
    assert(targetCount() === beforeTaskTargets, "Submitting task does not stream manual target");
    el("cancel-task").click();
    await settle();
    await poll();
    assert(snapshot.state.phase === 0 && !snapshot.armed, "Task cancellation disables rather than restoring output");
    assert(targetCount() === beforeTaskTargets, "Task stop never restores old manual target");
    for (const kind of ["move", "hold", "velocity", "sine", "ramp", "trajectory"]) {
      el("tab-control").click();
      el("enable").click(); await settle(); await poll();
      edit("target-position", snapshot.state.position);
      edit("target-kp", kind === "velocity" ? 0 : 1);
      edit("target-kd", kind === "velocity" ? 0.1 : 0);
      edit("target-velocity", kind === "velocity" ? 0.05 : 0);
      el("tab-task").click();
      el("task-kind").value = kind;
      el("task-kind").dispatchEvent(new win.Event("change"));
      if (kind === "sine") el("task-frequency").value = "0.2";
      if (kind === "trajectory") el("task-points").value = `0, ${snapshot.state.position}\n3, 0.2`;
      const before = requests.filter(r => r.path === "task").length;
      const targets = targetCount();
      el("start-task").click(); await settle(); await poll();
      const request = requests.filter(r => r.path === "task").at(-1);
      assert(requests.filter(r => r.path === "task").length === before + 1 && request.body.kind === kind,
        `${kind} is reachable through the visible task form`);
      assert(targetCount() === targets, `${kind} submission never sends a manual endpoint first`);
      if (kind === "trajectory") assert(request.body.times.join() === "0,3" && request.body.positions.length === 2,
        "Trajectory text produces the expected ordered arrays");
      if (kind === "sine") assert(request.body.frequency === 0.2 && request.body.amplitude === 0.1,
        "Sine-specific fields are included");
      el("cancel-task").click(); await settle(); await poll();
      assert(snapshot.state.phase === 0 && !snapshot.armed, `${kind} cancellation remains disabled`);
    }
    snapshot.task.state = "succeeded";
    snapshot.task.feedback.progress = 0.98;
    await poll();
    assert(el("task-progress").value === 1 && el("task-feedback").textContent.includes("100.0%"),
      "A confirmed successful result completes progress even if the last feedback preceded completion");
    el("tab-control").click();
    el("enable").click(); await settle(); await poll();
    el("tab-task").click();
    const invalidTimingTasks = requests.filter(r => r.path === "task").length;
    el("task-duration").value = "0.6";
    el("start-task").click(); await settle();
    assert(requests.filter(r => r.path === "task").length === invalidTimingTasks
      && el("notice").textContent.includes("稳定时间"),
      "Invalid task timing is reported before submission");
    el("task-duration").value = "5";
    el("task-kind").value = "torque";
    el("task-kind").dispatchEvent(new win.Event("change"));
    el("task-ramp_time").value = "3";
    el("start-task").click(); await settle();
    assert(requests.filter(r => r.path === "task").length === invalidTimingTasks
      && el("notice").textContent.includes("斜坡时间"),
      "Ramp time is limited to half the duration before submission");
    el("task-ramp_time").value = "1";
    el("task-kind").value = "trajectory";
    el("task-kind").dispatchEvent(new win.Event("change"));
    el("task-points").value = "not,a,trajectory";
    const invalidTasks = requests.filter(r => r.path === "task").length;
    el("start-task").click(); await settle();
    assert(requests.filter(r => r.path === "task").length === invalidTasks && el("notice").textContent.includes("轨迹"),
      "Invalid trajectory input is rejected with globally visible feedback");
    el("stop").click(); await settle(); await poll();
    el("tab-control").click();
    assert(el("event-log").children.length > 0, "Operator events retained");
    el("pause-chart").click();
    assert(el("pause-chart").getAttribute("aria-pressed") === "true", "Chart pause is explicit");
    el("chart-targets").checked = false;
    stream.onmessage({data: JSON.stringify({sequence: 1, time: 1, feedback_count: 1,
      position: 1885, absolute_position: 1886, native_position: 10, velocity: 6.28, torque: 0.2,
      target_position: 1885.1, target_velocity: 7, target_torque: 0.25})});
    await poll();
    const axisLabels = [];
    const context = el("chart").getContext("2d");
    const fillText = context.fillText;
    context.fillText = function (text, ...args) { axisLabels.push(text); return fillText.call(this, text, ...args); };
    el("chart-window").dispatchEvent(new win.Event("change"));
    assert(axisLabels.some(text => text.includes("1884.995") && text.includes("1885.005")),
      "Large absolute position uses local plot range rather than flattening motion against zero");
    const dashCalls = [], setLineDash = context.setLineDash;
    context.setLineDash = function (segments) { dashCalls.push([...segments]); return setLineDash.call(this, segments); };
    el("chart-position").value = "native_position";
    el("chart-targets").click();
    assert(axisLabels.some(text => text.includes("native_position") && text.includes("9.995") && text.includes("10.005")),
      "Compact position source draws its own values and range");
    assert(dashCalls.filter(segments => segments.length).length === 2,
      "Joint-position targets are not overlaid on compact coordinates");
    axisLabels.length = dashCalls.length = 0;
    el("chart-position").value = "absolute_position";
    el("chart-position").dispatchEvent(new win.Event("change"));
    assert(axisLabels.some(text => text.includes("absolute_position") && text.includes("1885.995") && text.includes("1886.005")),
      "Native continuous position source is distinct from joint position");
    axisLabels.length = dashCalls.length = 0;
    el("chart-position").value = "position";
    el("chart-position").dispatchEvent(new win.Event("change"));
    assert(dashCalls.filter(segments => segments.length).length === 3
      && axisLabels.some(text => text.includes("1885.110")), "Target overlays use dashed traces and participate in axis scaling");
    dashCalls.length = 0;
    el("chart-targets").click();
    assert(dashCalls.length === 0, "Unchecking targets removes all dashed overlays");
    el("chart-targets").click();
    context.setLineDash = setLineDash;
    context.fillText = fillText;
    el("export-csv").click();
    assert(downloads.includes("motor-feedback.csv"), "Captured feedback can be exported");
    const csv = await blobs.at(-1).text();
    assert(csv.includes("absolute_position") && csv.includes("1885"), "CSV retains continuous and compact coordinates");
    const beforeBurstRows = csv.trim().split("\n").length;
    for (let i = 2; i <= 101; ++i) {
      stream.onmessage({data: JSON.stringify({sequence: i, time: 1 + (i - 1) / 500,
        feedback_count: i, position: i, absolute_position: i, native_position: i,
        velocity: i % 2 ? 1 : -1, torque: 0.2, source_skipped_samples: 0})});
    }
    const recordedPoints = [];
    const lineTo = context.lineTo;
    context.lineTo = function (x, y) { recordedPoints.push([x, y]); return lineTo.call(this, x, y); };
    el("chart-window").dispatchEvent(new win.Event("change"));
    context.lineTo = lineTo;
    assert(recordedPoints.length >= 300, "Drawing visits all 101 feedback points across three traces, including same-pixel samples");
    el("export-csv").click();
    const burstCsv = await blobs.at(-1).text();
    assert(burstCsv.trim().split("\n").length === beforeBurstRows + 100,
      "A 500 Hz burst retains every sample between chart redraws, including while paused");
    const feedbackCounts = burstCsv.trim().split("\n").slice(-100).map(row => Number(row.split(",")[3]));
    assert(feedbackCounts.every((count, index) => count === index + 2), "CSV retains every ordered feedback count without decimation");
    for (let i = 102; i <= 30001; ++i) {
      stream.onmessage({data: JSON.stringify({sequence: i, time: 1 + (i - 1) / 500,
        feedback_count: i, position: i, absolute_position: i, native_position: i,
        velocity: 0, torque: 0, source_skipped_samples: 0})});
    }
    el("export-csv").click();
    assert((await blobs.at(-1).text()).trim().split("\n").length === 30002,
      "A complete 60-second 500 Hz window exceeds the old 16384-point cap without losing samples");
    const windowPoints = [];
    for (const seconds of ["5", "15", "60"]) {
      let points = 0;
      context.lineTo = function (...args) { points += 1; return lineTo.apply(this, args); };
      el("chart-window").value = seconds;
      el("chart-window").dispatchEvent(new win.Event("change"));
      windowPoints.push(points);
    }
    context.lineTo = lineTo;
    assert(windowPoints[0] > 0 && windowPoints[0] < windowPoints[1] && windowPoints[1] < windowPoints[2],
      "5/15/60 second windows draw progressively larger portions of the captured history");
    el("chart-window").value = "15";
    el("chart-window").dispatchEvent(new win.Event("change"));
    let redraws = 0;
    const clearRect = context.clearRect;
    context.clearRect = function (...args) { redraws += 1; return clearRect.apply(this, args); };
    el("pause-chart").click();
    assert(redraws === 0, "Resuming does not redraw once per already collected sample");
    intervals.find(interval => interval.delay === 20).callback();
    assert(redraws === 1, "One 50 Hz timer tick redraws the accumulated full-rate feedback exactly once");
    context.clearRect = clearRect;
    el("pause-chart").click();
    el("export-state").click();
    assert(downloads.includes("motor-state.json"), "Complete state snapshot can be exported");
    assert(!(await blobs.at(-1).text()).includes("test-session"), "State export never includes browser session token");
    const beforeClear = targetCount();
    el("clear-chart").click();
    assert(el("export-csv").disabled && targetCount() === beforeClear, "Clearing plot never changes control");
    snapshot.state.position = 13;
    await poll();
    el("fill-position").click();
    assert(Number(el("target-position").value) > Number(el("target-position").max), "Out-of-range current position is not silently wrapped or clamped");
    assert(el("position-scroll").getAttribute("aria-disabled") === "false", "Out-of-range draft remains adjustable without fixed UI endpoints");
    assert(el("position-scroll-status").textContent.includes("MIT"), "MIT encoding limit is explicit rather than silently wrapped");
    assert(el("notice").textContent.includes("超出"), "Out-of-range feedback explained");
    snapshot.state.feedback_fresh = false;
    await poll();
    assert(el("fill-position").disabled && el("position-scroll").getAttribute("aria-disabled") === "true", "Stale feedback disables edits");
    assert(el("device-zero").disabled && el("software-zero").disabled, "Zero operations require fresh feedback");
    assert(el("applied-torque").textContent.includes("过期"), "Stale target is labelled");
    assert(el("apply").disabled && el("control-rate").disabled && el("status-panel").classList.contains("stale"),
      "Stale feedback disables explicit target/rate submission and marks the summary");
    assert(el("phase").textContent === "SIM · 过期" && el("phase").title.includes("状态过期")
      && doc.querySelector(".position-strip").classList.contains("stale"),
      "The compact badge and relocated details keep stale-state information");
    snapshot.state.feedback_fresh = true;
    snapshot.has_control = false;
    await poll();
    assert(!el("target-position").disabled && !el("restore-target").disabled,
      "Fresh feedback allows draft editing without a manual connection/session button");
    const lazyClaims = requests.filter(r => r.path === "session").length;
    const lazyTargets = targetCount();
    edit("target-kp", 2);
    await poll();
    assert(requests.filter(r => r.path === "session").length === lazyClaims && targetCount() === lazyTargets,
      "Session expiry never restores old output or creates a session just for monitoring");
    edit("target-kp", 3);
    snapshot.armed = true;
    snapshot.state.phase = 2;
    snapshot.state.task_active = true;
    snapshot.task = {state: "running", message: "Task in progress", feedback: {progress: 0.5, elapsed: 1, position_error: 0}};
    await poll();
    failState = true;
    await poll();
    assert(el("applied-position").textContent.includes("未连接"), "Disconnected target marked unavailable");
    assert(fields.every((field) => Number(el(`target-${field}`).value) === 0), "Disconnect clears drafts");
    const disconnectedStops = requests.filter(r => r.path === "stop").length;
    const disconnectedClaims = requests.filter(r => r.path === "session").length;
    assert(!el("stop").disabled, "STOP remains actionable without a connected control session");
    assert(el("device-zero").disabled && el("software-zero").disabled, "Disconnected zero controls do not submit operations");
    assert(!el("cancel-task").disabled, "A last-known active task can still be stopped without reconnecting");
    el("cancel-task").click(); await settle();
    failStop = true;
    el("stop").click(); await settle();
    assert(el("notice").textContent.includes("失能未确认") && el("notice").classList.contains("error"),
      "An unreachable STOP is explicitly reported, never presented as confirmed");
    failStop = false;
    doc.dispatchEvent(new win.KeyboardEvent("keydown", {key: "Escape", bubbles: true}));
    await settle();
    assert(requests.filter(r => r.path === "stop").length === disconnectedStops + 3
      && requests.filter(r => r.path === "session").length === disconnectedClaims,
      "Disconnected task cancellation, STOP and Esc submit directly without connecting or acquiring ownership first");
    failState = false;
    await poll();
    assert(!snapshot.has_control && !snapshot.armed && el("connection").textContent === "在线",
      "Automatic reconnection restores monitoring, not command sessions or enable");
    assert(requests.filter(r => r.path === "session").length === disconnectedClaims,
      "Reconnect never creates a keepalive session");
    const enablesBeforeStop = requests.filter(r => r.path === "enable").length;
    holdSession = true;
    el("enable").click(); await settle();
    assert(typeof releaseSession === "function", "First operation waits for its lazy session");
    el("stop").click(); await settle();
    holdSession = false;
    releaseSession(); await settle(); await poll();
    assert(requests.filter(r => r.path === "enable").length === enablesBeforeStop && !snapshot.armed,
      "STOP cancels an enable waiting for a session; a late response never enables the motor");
    el("enable").click(); await settle(); await poll();
    assert(snapshot.armed && snapshot.has_control, "Explicit enable lazily creates its operation session");
    const monitoringStream = stream;
    const sessionsBeforeHide = requests.filter(r => r.path === "session").length;
    Object.defineProperty(doc, "hidden", {configurable: true, value: true});
    doc.dispatchEvent(new win.Event("visibilitychange")); await settle();
    assert(!snapshot.has_control && !snapshot.armed && el("enable").disabled,
      "Hiding a motor page releases control and prevents further commands");
    Object.defineProperty(doc, "hidden", {configurable: true, value: false});
    doc.dispatchEvent(new win.Event("visibilitychange")); await settle();
    await poll();
    assert(requests.filter(r => r.path === "session").length === sessionsBeforeHide,
      "Returning to a motor page does not reacquire a session or resume motion");
    assert(stream === monitoringStream && !el("status-panel").classList.contains("stale"),
      "Hiding and returning keep monitoring connected");
    assert(!el("enable").disabled, "Explicit enable is available again without a connection button");
    el("enable").click(); await settle(); await poll();
    win.dispatchEvent(new win.Event("pagehide")); await settle();
    assert(!snapshot.has_control && !snapshot.armed, "Leaving a motor page submits release without restoring targets");
    el("tab-events").click();
    assert(Number(el("event-count").textContent) === el("event-log").children.length, "Event counter matches the bounded log");
    el("clear-events").click();
    assert(el("event-count").textContent === "0" && !el("event-log").children.length, "Clear events resets the count and empty state");
    el("tab-control").click();
    if (css) {
      iframe.style.width = "1440px";
      iframe.style.height = "1000px";
      await settle();
      assert(win.innerWidth >= 1400, "Desktop layout tested in a real wide iframe viewport");
      assert(win.getComputedStyle(el("workspace")).gridTemplateColumns.split(" ").length === 2,
        "Desktop uses side-by-side plot and manual controls");
      assert(doc.documentElement.scrollWidth <= win.innerWidth, "Desktop has no page-level horizontal overflow");
      iframe.style.width = "390px";
      await settle();
      assert(win.innerWidth <= 400, "Mobile layout tested in a real narrow iframe viewport");
      assert(win.getComputedStyle(el("workspace")).gridTemplateColumns.split(" ").length === 1,
        "Mobile stacks plot and controls");
      assert(doc.documentElement.scrollWidth <= win.innerWidth, "Mobile has no page-level horizontal overflow");
      assert(el("control-panel").getBoundingClientRect().top < el("plot-panel").getBoundingClientRect().top,
        "Mobile prioritizes controls before the large chart");
      const connectionText = el("connection").textContent, phaseText = el("phase").textContent;
      el("connection").textContent = "反馈离线";
      for (const width of [1366, 1024, 900, 768, 640, 550, 390, 320]) {
        iframe.style.width = `${width}px`;
        await layout();
        el("tab-control").click();
        for (const id of ["device-zero", "software-zero"]) {
          const button = el(id), bounds = button.getBoundingClientRect();
          assert(bounds.height > 0 && bounds.left >= 0 && bounds.right <= win.innerWidth
            && button.scrollWidth <= button.clientWidth + 1,
            `${id} stays readable and within the main control panel at ${width}px`);
        }
        win.scrollTo(0, 600);
        await settle();
        assert(win.scrollY > 0 && Math.abs(dock.getBoundingClientRect().top) < 1,
          `Status stays pinned when scrolling the ${width}px layout`);
        const height = dock.getBoundingClientRect().height;
        maxDockHeight = Math.max(maxDockHeight, height);
        assert(height >= 80 && height <= 85, `The ${width}px layout keeps the entire fixed header within 84px (actual ${height})`);
        singleRow(`${width}px header`);
        assert(parseFloat(win.getComputedStyle(el("position")).fontSize) >= 18,
          `Readouts do not shrink to tiny text at ${width}px`);
        for (const phase of ["DISABLED", "ENABLING", "ENABLED", "STOPPING", "FAULT", "UNKNOWN"]) {
          el("phase").textContent = `SIM · ${phase}`;
          assert(el("phase").scrollWidth <= el("phase").clientWidth + 1,
            `${phase} is readable without truncation at ${width}px`);
        }
        el("phase").textContent = phaseText;
        for (const id of ["position", "velocity", "torque", "temperature", "enable", "stop"]) {
          const bounds = el(id).getBoundingClientRect();
          assert(bounds.height > 0 && bounds.top >= 0 && bounds.bottom <= dock.getBoundingClientRect().bottom,
            `${id} remains visible in the pinned ${width}px panel`);
          assert(el(id).scrollWidth <= el(id).clientWidth + 1, `${id} is readable at ${width}px`);
        }
        const readings = el("status-panel");
        const area = readings.getBoundingClientRect();
        const fixedIdentity = doc.querySelector(".identity").getBoundingClientRect().left;
        const fixedStop = el("stop").getBoundingClientRect().right;
        readings.scrollLeft = 0;
        if (width >= 640) assert(readings.scrollWidth <= readings.clientWidth + 1,
          `All four values fit without horizontal scrolling at ${width}px`);
        if (readings.scrollWidth > readings.clientWidth + 1) {
          assert(win.getComputedStyle(readings).overflowX === "auto",
            "Narrow screens scroll only the readings, not the whole fixed header");
          assert(el("position").getBoundingClientRect().left >= area.left
            && el("position").getBoundingClientRect().right <= area.right + 1,
            "The first readout is fully reachable at the start of the telemetry strip");
          readings.scrollLeft = readings.scrollWidth;
          assert(readings.scrollLeft > 0 && el("temperature").getBoundingClientRect().left >= area.left - 1
            && el("temperature").getBoundingClientRect().right <= area.right + 1,
            "The last readout is fully reachable at the end of the telemetry strip");
          assert(doc.querySelector(".identity").getBoundingClientRect().left === fixedIdentity
            && el("stop").getBoundingClientRect().right === fixedStop,
            "Identity and STOP do not move when scrolling the readings");
          readings.scrollLeft = 0;
        }
        assert(el("connection").getBoundingClientRect().height > 0,
          "Online status remains visible above the power controls");
        assert(el("enable").getBoundingClientRect().right <= el("stop").getBoundingClientRect().left,
          "Enable and disable remain adjacent and ordered in the top-right");
        for (const tab of doc.querySelectorAll("[data-panel]")) {
          tab.click();
          assert(doc.querySelectorAll(".view-panel:not([hidden])").length === 1,
            `Only the selected workspace is visible at ${width}px`);
          assert(doc.documentElement.scrollWidth <= win.innerWidth + 1, `${tab.id} fits the ${width}px viewport`);
          assert(el("notice").getBoundingClientRect().height > 0, "Operation feedback remains visible in every workspace");
          assert(Math.abs(dock.getBoundingClientRect().top) < 1
            && el("status-panel").getBoundingClientRect().height > 0, "Status remains pinned in every workspace");
        }
        win.scrollTo(0, 0);
      }
      el("connection").textContent = connectionText;
      for (const [width, height] of [[1440, 640], [844, 390], [390, 400], [800, 240]]) {
        iframe.style.width = `${width}px`;
        iframe.style.height = `${height}px`;
        el("tab-control").click();
        await layout();
        win.scrollTo(0, 600);
        await settle();
        const pinned = dock.getBoundingClientRect();
        maxDockHeight = Math.max(maxDockHeight, pinned.height);
        assert(win.scrollY > 0 && Math.abs(pinned.top) < 1, `Status is pinned at ${width}x${height}`);
        assert(pinned.height >= 80 && pinned.height <= 85,
          `Short viewports keep the same compact 84px header (actual ${pinned.height})`);
        singleRow(`${width}x${height} header`);
        assert(pinned.bottom <= height - 80, `The ${width}x${height} status leaves usable space for controls`);
        assert(win.getComputedStyle(el("plot-panel")).position === "static",
          "Short viewports can scroll all chart content instead of clipping a sticky plot");
        for (const id of ["position", "velocity", "torque", "temperature", "phase", "enable", "stop"]) {
          const bounds = el(id).getBoundingClientRect();
          assert(bounds.height > 0 && bounds.top >= 0 && bounds.bottom <= pinned.bottom,
            `${id} is visible in the compact top panel`);
        }
        assert(doc.documentElement.scrollWidth <= win.innerWidth + 1,
          `Compact status has no horizontal overflow at ${width}x${height}`);
        win.scrollTo(0, 0);
      }
    }
    assert(errors.length === 0, `No browser errors: ${errors.join("; ")}`);
    return {checks, targetRequests: targetCount(), primaryTop, dockHeight, maxDockHeight, readoutFont, errors};
  } finally {
    iframe.remove();
  }
}
