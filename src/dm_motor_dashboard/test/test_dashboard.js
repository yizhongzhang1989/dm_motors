"use strict";

// Run in a browser with the dashboard HTML and JS source; all API traffic is mocked.
async function runDashboardTests(html, source) {
  const iframe = document.createElement("iframe");
  iframe.srcdoc = html.replace(/<script\b[^>]*>[\s\S]*?<\/script>/gi, "")
    .replace(/<link\b[^>]*>/gi, "");
  const loaded = new Promise((resolve) => iframe.addEventListener("load", resolve, {once: true}));
  document.body.appendChild(iframe);
  await loaded;
  const win = iframe.contentWindow;
  const doc = win.document;
  const fields = ["position", "velocity", "kp", "kd", "torque"];
  const zero = () => Object.fromEntries(fields.map((field) => [field, 0]));
  const snapshot = {
    motor_id: 7, connected: true, has_control: false, armed: false, target: zero(),
    capabilities: {ready: true, name: "motor_7", modes: [1, 2, 3, 4],
      profile: "DM", backend: "sim", registers: [
        {address: 10, name: "test_float", integer: false, unit: "rad"},
        {address: 11, name: "test_integer", integer: true, unit: ""}]},
    state_time: 1, state: {
      motor_id: 7, master_id: 0, phase: 0, status: 0, simulated: true, mode: 1, ready: true,
      activity_available: true, maintenance_available: true,
      position: 0.123456789, velocity: 0, torque: 0, mos_temperature: 25,
      rotor_temperature: 26, feedback_fresh: true, feedback_count: 1,
      feedback_age: 0, period_ms: 5, max_period_ms: 5, deadline_misses: 0,
      position_limit: 12.5, velocity_limit: 30, kp_limit: 500, kd_limit: 5, torque_limit: 10,
    },
  };
  const requests = [];
  const errors = [];
  let nextPoll = null;
  let rejectTarget = false;
  let rejectRead = false;
  let rejectOperation = false;
  let failState = false;
  win.addEventListener("error", (event) => errors.push(event.message));
  win.addEventListener("unhandledrejection", (event) => errors.push(String(event.reason)));
  win.setInterval = () => 1;
  win.setTimeout = (callback) => { nextPoll = callback; return 1; };
  win.clearTimeout = () => { nextPoll = null; };
  win.requestAnimationFrame = () => 1;
  win.EventSource = class { close() {} };
  win.confirm = () => true;
  win.fetch = async (url, options) => {
    const path = url.split("/").pop();
    requests.push({path, body: options.body && JSON.parse(options.body)});
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
    } else if (path === "target") {
      if (rejectTarget) { status = 400; result = {error: "Target rejected"}; }
      else snapshot.target = JSON.parse(options.body);
    } else if (path === "registers") {
      result = {operation_ids: [123], message: "Read queued"};
    } else if (path === "123") {
      result = {success: !rejectRead, state: rejectRead ? 2 : 1, address: 10,
        raw_value: 1065353216, message: rejectRead ? "Entire bus must be disabled" : "Confirmed"};
    } else if (path === "operation") {
      if (!rejectOperation) snapshot.state.mode = JSON.parse(options.body).mode;
      result = {operation_id: 124, message: "Mode queued"};
    } else if (path === "124") {
      result = {success: !rejectOperation, state: rejectOperation ? 2 : 1,
        message: rejectOperation ? "Maintenance rejected" : "Confirmed"};
    } else if (path === "stop" || path === "release") {
      snapshot.armed = false;
      snapshot.state.phase = 0;
      snapshot.target = zero();
      if (path === "release") snapshot.has_control = false;
    }
    return {ok: status === 200, status, json: async () => path === "state"
      ? JSON.parse(JSON.stringify(snapshot)) : result};
  };
  let checks = 0;
  const assert = (condition, message) => {
    if (!condition) throw new Error(message);
    checks += 1;
  };
  const el = (id) => doc.getElementById(id);
  const settle = async () => { for (let i = 0; i < 4; i += 1) await new Promise((r) => setTimeout(r, 0)); };
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
    for (const field of fields) {
      assert(el(`slider-${field}`).min === el(`target-${field}`).min, `${field} minimum matches`);
      assert(el(`slider-${field}`).max === el(`target-${field}`).max, `${field} maximum matches`);
    }
    edit("target-position", 0.314159265);
    assert(Number(el("slider-position").value) === 0.314159265, "Numeric precision preserved");
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
    edit("target-torque", 0.1);
    el("enable").click();
    await settle();
    await poll();
    assert(fields.every((field) => Number(el(`target-${field}`).value) === 0), "Enable clears drafts");
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
    assert(fields.every((field) => Number(el(`target-${field}`).value) === 0), "STOP clears drafts");
    assert(el("mode-select").options.length === 4, "Advertised modes are listed");
    el("mode-select").value = "2";
    el("mode-select").dispatchEvent(new win.Event("change"));
    assert(!requests.some((request) => request.path === "operation"), "Mode selection is only a draft");
    el("set-mode").click();
    await settle();
    await poll();
    assert(snapshot.state.mode === 2 && snapshot.state.phase === 0 && !snapshot.armed,
      "Explicit mode operation stays disabled and unarmed");
    assert(el("operation-status").textContent.includes("#124")
      && el("operation-status").textContent.includes("已确认"), "Mode operation confirmation is polled");
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
    rejectOperation = true;
    el("mode-select").value = "3";
    el("set-mode").click();
    await settle();
    assert(snapshot.state.mode === 2 && el("operation-status").textContent.includes("已拒绝"),
      "Rejected queued mode operation is never labelled applied");
    rejectOperation = false;
    for (const [mode, expected, values] of [
      [2, ["position", "velocity_limit"], {position: 0.2, velocity_limit: 1}],
      [3, ["velocity"], {velocity: -0.5}],
      [4, ["position", "velocity_limit", "current_ratio"], {position: 0.3, velocity_limit: 1, current_ratio: 0.5}],
    ]) {
      snapshot.state.mode = mode;
      snapshot.target = {...zero(), velocity_limit: 0, current_ratio: 0};
      await poll();
      const before = targetCount();
      assert(fields.concat(["velocity_limit", "current_ratio"]).every((field) =>
        el(`target-${field}`).parentElement.hidden === !expected.includes(field)), `Mode ${mode} fields are selected`);
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
    snapshot.state.mode = 1;
    snapshot.target = zero();
    await poll();
    snapshot.state.position = 13;
    await poll();
    el("fill-position").click();
    assert(el("target-position").value === "0", "Out-of-range feedback cannot overwrite draft");
    assert(el("notice").textContent.includes("超出"), "Out-of-range feedback explained");
    snapshot.state.feedback_fresh = false;
    await poll();
    assert(el("fill-position").disabled && el("slider-position").disabled, "Stale feedback disables edits");
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
    assert(errors.length === 0, `No browser errors: ${errors.join("; ")}`);
    return {checks, targetRequests: targetCount(), errors};
  } finally {
    iframe.remove();
  }
}
