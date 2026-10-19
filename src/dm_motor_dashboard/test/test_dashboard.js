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
  let failState = false;
  let stream = null;
  let registerAddress = 10;
  const downloads = [];
  const blobs = [];
  let acceptConfirmations = true;
  let holdTarget = false;
  let releaseTarget = null;
  const intervals = [];
  win.addEventListener("error", (event) => errors.push(event.message));
  win.addEventListener("unhandledrejection", (event) => errors.push(String(event.reason)));
  win.setInterval = (callback, delay) => { intervals.push({callback, delay}); return intervals.length; };
  win.setTimeout = (callback) => { nextPoll = callback; return 1; };
  win.clearTimeout = () => { nextPoll = null; };
  win.requestAnimationFrame = () => 1;
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
    if (path === "session") {
      snapshot.has_control = true;
      result = {session: "test-session", message: "Claimed"};
    } else if (path === "enable") {
      snapshot.armed = true;
      snapshot.state.phase = 2;
      snapshot.target = zero();
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
    } else if (path === "operation") {
      snapshot.control_generation += 1;
      snapshot.target_sequence = 0;
      if (!rejectOperation) setMode(JSON.parse(options.body).mode);
      result = {operation_id: 124, message: "Mode queued"};
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
      if (acceptConfirmations && doc.getElementById("confirmation").open) {
        doc.getElementById("confirmation-accept").click();
      }
    }
  };
  const poll = async () => {
    const callback = nextPoll;
    nextPoll = null;
    assert(Boolean(callback), "State polling remains scheduled");
    await callback();
    await settle();
  };
  const edit = (id, value) => {
    el(id).value = String(value);
    el(id).dispatchEvent(new win.Event("input", {bubbles: true}));
  };
  const targetCount = () => requests.filter((request) => request.path === "target").length;
  try {
    win.eval(source);
    await settle();
    // Integrated browsers may hide the iframe until the container is foregrounded.
    if (el("connection").textContent === "未连接") {
      Object.defineProperty(doc, "hidden", {configurable: true, value: false});
      el("connect").click();
      await settle();
    }
    assert(!el("target-position").disabled, "Owner can prepare disabled-motor drafts");
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
    assert(el("position-data").textContent.includes("原生连续位置"), "Coordinate diagnostics exposed");
    assert(el("timing-data").textContent.includes("P50"), "Timing percentile diagnostics exposed");
    assert(el("live-targets").checked, "Event-driven manual control is enabled by default");
    assert(el("control-rate-status").textContent.includes("199.8"), "Actual frequency is distinct from configured frequency");
    el("control-rate").value = "100";
    el("control-rate").dispatchEvent(new win.Event("change"));
    await settle();
    await poll();
    assert(requests.some(r => r.path === "control-rate" && r.body.control_rate === 100),
      "Runtime frequency change is submitted immediately");
    assert(el("control-rate-status").textContent.includes("99.8"), "Measured frequency updates from driver state");
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
    acceptConfirmations = false;
    el("mode-select").value = "2";
    el("mode-select").dispatchEvent(new win.Event("change"));
    assert(requests.some(request => request.path === "operation" && request.body.mode === 2),
      "Mode selection immediately submits the switch without a confirmation");
    assert(!el("confirmation").open, "Mode switching never opens the confirmation dialog");
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
    acceptConfirmations = true;
    el("read-parameter").click();
    await settle();
    assert(el("parameter-value").textContent.includes("test_float: 1 rad"), "Async parameter read decodes float32");
    const stopsBeforeRead = requests.filter((request) => ["stop", "release"].includes(request.path)).length;
    rejectRead = true;
    el("read-parameter").click();
    await settle();
    assert(el("parameter-value").textContent.includes("已拒绝")
      && el("notice").textContent.includes("Entire bus must be disabled"), "Queued read rejection is explicit");
    assert(requests.filter((request) => ["stop", "release"].includes(request.path)).length === stopsBeforeRead,
      "Read rejection does not trigger STOP or release");
    rejectRead = false;
    assert(el("parameter-results").children.length === 1, "Confirmed reads retained in parameter table");
    edit("parameter-input", 2);
    acceptConfirmations = false;
    el("write-parameter").click();
    await settle();
    assert(el("confirmation").open, "Parameter writes require an explicit nonblocking confirmation");
    await poll();
    assert(!requests.some(r => r.path === "register-write"), "Pending confirmation sends no write and polling continues");
    el("confirmation-cancel").click();
    await settle();
    const dialogStops = requests.filter(r => r.path === "stop").length;
    el("write-parameter").click();
    await settle();
    el("confirmation-stop").click();
    await settle();
    assert(!el("confirmation").open, "STOP dismisses a pending confirmation without executing it");
    assert(requests.filter(r => r.path === "stop").length === dialogStops + 1,
      "Dialog STOP remains available while ordinary controls are busy and inert");
    doc.dispatchEvent(new win.KeyboardEvent("keydown", {key: "Escape", bubbles: true}));
    await settle();
    assert(requests.filter(r => r.path === "stop").length === dialogStops + 2, "Escape explicitly requests disable");
    acceptConfirmations = true;
    el("write-parameter").click();
    await settle();
    assert(requests.some(r => r.path === "register-write" && r.body.value === 2), "Explicit numeric write is submitted");
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
    assert(el("position-scroll-status").textContent.includes("暂停"), "Rejected live target explicitly pauses live scrolling");
    const rejectedCount = targetCount();
    movePosition("ArrowRight");
    await settle();
    assert(targetCount() === rejectedCount, "Rejected live controls do not keep retrying automatically");
    rejectTarget = false;
    el("restore-target").click();
    el("apply").click();
    await settle();
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
    el("task-kind").value = "torque";
    el("task-kind").dispatchEvent(new win.Event("change"));
    el("live-targets").click();
    edit("target-torque", 0.02);
    const beforeTaskTargets = targetCount();
    el("start-task").click();
    await settle();
    await poll();
    const task = requests.find(r => r.path === "task").body;
    assert(task.kind === "torque" && task.target.torque === 0.02, "Task uses explicit current target draft");
    assert(task.amplitude === 0 && task.frequency === 0 && task.ramp_time > 0, "Only applicable task fields enabled");
    assert(el("apply").disabled && !el("cancel-task").disabled, "Task owns output and can be cancelled");
    assert(el("position-scroll").getAttribute("aria-disabled") === "true", "Tasks disable live position scrolling");
    assert(el("task-progress").value === 0.5, "Task progress displayed");
    assert(targetCount() === beforeTaskTargets, "Submitting task does not stream manual target");
    el("cancel-task").click();
    await settle();
    await poll();
    assert(snapshot.state.phase === 0 && !snapshot.armed, "Task cancellation disables rather than restoring output");
    assert(targetCount() === beforeTaskTargets, "Task stop never restores old manual target");
    assert(el("event-log").children.length > 0, "Operator events retained");
    el("pause-chart").click();
    assert(el("pause-chart").getAttribute("aria-pressed") === "true", "Chart pause is explicit");
    stream.onmessage({data: JSON.stringify({sequence: 1, time: 1, feedback_count: 1,
      position: 1885, absolute_position: 1885, native_position: 10, velocity: 6.28, torque: 0.2})});
    await poll();
    const axisLabels = [];
    const context = el("chart").getContext("2d");
    const fillText = context.fillText;
    context.fillText = function (text, ...args) { axisLabels.push(text); return fillText.call(this, text, ...args); };
    el("chart-window").dispatchEvent(new win.Event("change"));
    assert(axisLabels.some(text => text.includes("1884.995") && text.includes("1885.005")),
      "Large absolute position uses local plot range rather than flattening motion against zero");
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
    assert(el("applied-torque").textContent.includes("过期"), "Stale target is labelled");
    snapshot.state.feedback_fresh = true;
    snapshot.has_control = false;
    await poll();
    assert(el("target-position").disabled && el("restore-target").disabled, "Read-only client cannot edit");
    el("claim").click();
    await settle();
    edit("target-kp", 2);
    snapshot.has_control = false;
    await poll();
    assert(el("target-kp").value === "0", "Lost ownership clears draft");
    el("claim").click();
    await settle();
    edit("target-kp", 3);
    failState = true;
    await poll();
    assert(el("applied-position").textContent.includes("未连接"), "Disconnected target marked unavailable");
    assert(fields.every((field) => Number(el(`target-${field}`).value) === 0), "Disconnect clears drafts");
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
    }
    assert(errors.length === 0, `No browser errors: ${errors.join("; ")}`);
    return {checks, targetRequests: targetCount(), errors};
  } finally {
    iframe.remove();
  }
}
