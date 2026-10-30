"use strict";

const assert = require("assert").strict;
const view = require("../web/view_model.js");
assert.equal(view.stepIndex({phase: "failed", stage: "stopping"}), 3);
assert.equal(view.stepIndex({phase: "completed", stage: "completed"}), 5);
assert.equal(view.stepIndex({phase: "failed", stage: "failed"}), -1);
assert.equal(view.stepIndex({phase: "failed", stage: "failed", events: [{stage:"stopping"},{stage:"fitting"},{stage:"failed"}]}), 4);
assert.equal(view.stepIndex({phase: "failed", stage: "failed", failure_stage:"running",
  events: [{stage:"stopping"},{stage:"saving_failure"},{stage:"failed"}]}), 2);
assert.equal(view.stepIndex({experiment:"precision",phase:"running",precision:{phase:"sample",validation:true}}),3);
assert.equal(view.stepIndex({experiment:"precision",phase:"running",precision:{phase:"static_ramp"}}),1);
assert.equal(view.stepIndex({experiment:"precision",phase:"completed_unqualified",precision:{phase:"done"}}),5);
assert(view.completed.has("completed_unqualified") && view.terminal.has("completed_unqualified"));
assert.equal(view.number(NaN), "—");
assert.equal(view.bounds([{points: []}]), null);
const extent = view.bounds([{points: [[0, -0.2], [1, 0.3], [NaN, 0]]}]);
assert.equal(extent.x0, 0);
assert.equal(extent.x1, 1);
assert(extent.y0 < -0.2 && extent.y1 > 0.3);
assert.deepEqual(view.mergeRows([[4, 0.4]], [[0, 0], [4, 0.4]]), [[0, 0], [4, 0.4]]);
assert.deepEqual(view.mergeRows([[4, 0.4]], [[0, 0]], true), [[0, 0]]);
const unchanged = [[0, 0], [1, 0.1]];
assert.equal(view.mergeRows(unchanged, []), unchanged);
assert.deepEqual(view.mergeRows(unchanged, [[2, 0.2]]), [[0, 0], [1, 0.1], [2, 0.2]]);
const history = Array.from({length: 1000000}, (_, index) => [index, index * 0.005, Math.sin(index / 100), 0]);
history[999123][2] = 100;
history[998456][3] = -200;
const sourceLength = history.length;
const recent = view.windowRows(history, {seconds: 60, maxPoints: 400, valueIndices: [2, 3]});
assert(recent.rows.length <= 400 && recent.inWindow <= 12001);
assert(recent.rows.includes(history[999123]) && recent.rows.includes(history[998456]));
assert(recent.rows.every(row => row[1] >= recent.start && row[1] <= recent.end));
assert.equal(history.length, sourceLength);
const overview = view.windowRows(history, {seconds: 0, maxPoints: 400, valueIndices: [2, 3]});
assert(overview.sampled && overview.rows.length <= 400);
assert(overview.rows.includes(history[0]) && overview.rows.includes(history[history.length - 1]));
assert(overview.rows.includes(history[999123]) && overview.rows.includes(history[998456]));
const replay = view.windowRows(history, {seconds: 30, end: 100, maxPoints: 400});
assert(replay.rows.every(row => row[1] >= 70 && row[1] <= 100));
const performance = require("perf_hooks").performance;
const start = performance.now();
for (let index = 0; index < 100; index++) view.windowRows(history, {seconds: 60, maxPoints: 400, valueIndices: [2, 3]});
assert(performance.now() - start < 2500, "recent-window selection must not scan the entire history");
for (const count of [1, 2, 3, 4]) {
  const budget = Math.floor(view.maxChartPoints / count);
  assert(view.limitRows(overview.rows, budget, [2, 3]).length * count <= view.maxChartPoints);
}
const elements = Object.fromEntries(view.editable.map(key => [key, {value: "1", checked: false}]));
elements.experiment.value = "calibrate";
elements.model.value = "stribeck";
elements.calibration_file.value = "";
elements.fit_bias.checked = true;
assert.equal(view.readValues(elements).fit_bias, true);
assert.equal(view.readValues(elements).known_bias, 1);
assert.equal(view.readValues(elements).model, "stribeck");
elements.duration.value = "";
assert.throws(() => view.readValues(elements), /duration/);
elements.duration.value = "Infinity";
assert.throws(() => view.readValues(elements), /duration/);
assert(!view.editable.includes("allow_hardware"));
assert(!view.editable.includes("max_torque"));
assert.deepEqual(view.constraintRows(null), []);
const constraintState = {
  experiment: "compensate", calibration_configuration: {max_speed: 1, duration: 20, compensation_fraction: 0.8},
  configuration: {max_speed: null, duration: null, compensation_fraction: 0.9, drag_stop_policy: "operator"},
  driver_constraint_observation: {
    state_limits: {torque_limit: 0.8, tmax: 10, host_protection_enabled: true},
    last_preflight_configuration: {measured_torque_limit: 1, measured_velocity_limit: 13},
  },
};
const constraintRows = view.constraintRows(constraintState);
assert.deepEqual(constraintRows.find(row => row[0].endsWith("max_speed")).slice(1, 3), ["1", "未设置（null）"]);
assert.equal(constraintRows.find(row => row[0] === "驱动状态 · torque_limit")[1], "0.8");
assert.match(constraintRows.find(row => row[0] === "驱动状态 · tmax")[2], /不是持续额定能力/);
assert.match(constraintRows.find(row => row[0] === "驱动能力 · measured_velocity_limit")[2], /可能过期/);
assert.match(view.constraintRows({configuration: {}}).flat().join(" "), /尚未取得/);
console.log("Dashboard helper assertions passed");

const fs = require("fs"), path = require("path"), vm = require("vm");
const markup = fs.readFileSync(path.join(__dirname, "../web/index.html"), "utf8");
const browserScript = fs.readFileSync(path.join(__dirname, "../web/dashboard.js"), "utf8");
class Element {
  constructor() {
    this.children = []; this.listeners = {}; this.hidden = false; this.disabled = false;
    this.className = ""; this.style = {setProperty(name, value) { this[name] = value; }};
    this.dataset = {}; this.attributes = {}; this._text = ""; this._value = "";
    this.classList = {
      toggle: (name, enabled) => {
        const names = new Set(this.className.split(/\s+/).filter(Boolean));
        if (enabled) names.add(name); else names.delete(name);
        this.className = [...names].join(" ");
      },
    };
  }
  get textContent() { return this._text + this.children.map(node => node.textContent).join(""); }
  set textContent(value) { this._text = String(value); this.children = []; }
  get value() { return this._value; }
  set value(value) { this._value = String(value); }
  get options() { return this.children; }
  replaceChildren(...children) { this._text = ""; this.children = children; }
  append(...children) { this.children.push(...children); }
  add(option) { this.append(option); }
  setAttribute(name, value) { this.attributes[name] = value; }
  addEventListener(name, callback) { (this.listeners[name] ||= []).push(callback); }
  dispatchEvent(event) { for (const callback of this.listeners[event.type] || []) callback(event); }
  reportValidity() { return true; }
  showModal() { this.open = true; }
  close() { this.open = false; }
  getBoundingClientRect() { return {width: 1000, height: 160, top: 220}; }
}
class OptionElement extends Element {
  constructor(label, value) { super(); this.textContent = label; this.value = value; }
}
function dashboardState(experiment = "fourier") {
  const configuration = {experiment, model: "auto", calibration_file: "", duration: 20, amplitude: 0.4,
    collect_warmup: true, collect_static: true, collect_training: true, collect_validation: true,
    frequency: 0.1, kp: 2, kd: 0.5, fit_bias: true, known_bias: 0, fit_gravity: true, fit_stiffness: false,
    compensation_fraction: 0.8, motor_namespace: "/current_motor", allow_hardware: false,
    max_torque: 0.4, max_speed: 7.2, max_travel: 64, max_temperature: 48, rate: 200};
  return {configuration, experiment, controller_id: "current-controller", generation: 1, phase: "idle", stage: "idle",
    can_start: true, commands_ready: true, stop_confirmed: true, run_id: "", progress: 0, elapsed: 0, samples: 0,
    events: [], error: "", output_directory: "", report: null,
    motor: {motor_id: 2, simulated: true, position: 1.2345, velocity: 0.0123, torque: 0.0456,
      rotor_temperature: 23, mos_temperature: 24},
    precision_settings: {speeds: [0.05, 0.1, Math.PI * 2], repetitions: 3, warmup_repetitions: 0, time_budget: 6000,
      rmse_limit: 0.003, max_error: 0.005, repeatability_limit: 0.003, ci95_limit: 0.005},
    fourier_settings: {frequency: 0.1, harmonics: 5, feedback_rate: 1000,
      slow_training_cycles: 6, training_cycles: 8, slow_validation_cycles: 4, validation_cycles: 4},
    precision: {phase: "baseline", warmup: {completed_cells: 0, required_cells: 6}, static_trial: 0,
      static_trials: 112, completed_cells: 0, cells: 18, target_speed: 0, repetition: 0,
      sample_revolutions: 0, required_revolutions: 1, observations: []}};
}
async function browserHarness(state, historical = null, rawRuns = [], stageSchema = null) {
  const nodes = new Map([...markup.matchAll(/<[^>]*\bid="([^"]+)"[^>]*>/g)].map(match => [match[1], new Element()]));
  nodes.get("steps").append(...Array.from({length: 6}, () => new Element()));
  const selectors = Object.fromEntries([".metrics", ".observations", ".topbar", ".workspace", ".observations .legend .amber"]
    .map(name => [name, new Element()]));
  const document = new Element();
  document.getElementById = id => { assert(nodes.has(id), `Missing dashboard element ${id}`); return nodes.get(id); };
  document.querySelector = selector => { assert(selector in selectors); return selectors[selector]; };
  document.createElement = () => new Element();
  document.createTextNode = value => { const node = new Element(); node.textContent = value; return node; };
  document.documentElement = new Element();
  const requests = [], timers = [];
  const snapshot = {state, connected: true, services_ready: true, trace: [], display_fields: ["time", "position"],
    stream_id: "browser-test", display_count: 0, trace_complete: true, capabilities: {
      reanalysis: true, drag_control: true, drag_manual_motion: true, drag_manual_stop: true,
      drag_operator_speed: true, fault_reset: true, stage_parameters: Boolean(stageSchema)}};
  const context = {
    CalibrationView: view, document, window: {devicePixelRatio: 1, scrollY: 0}, Option: OptionElement,
    Event: class { constructor(type) { this.type = type; } preventDefault() {} },
    AbortController, URLSearchParams, console,
    setTimeout(callback, delay) { if (delay === 200) timers.push(callback); return 1; },
    clearTimeout() {}, requestAnimationFrame() {},
    ResizeObserver: class { constructor(callback) { this.callback = callback; } observe() { this.callback(); } },
    async fetch(url, options) {
      requests.push({url, method: options.method, body: options.body ? JSON.parse(options.body) : null});
      let payload;
      if (url.startsWith("/api/state?")) payload = snapshot;
      else if (url === "/api/runs") payload = {runs: [...rawRuns, ...(historical ? [{id: historical.id, experiment: historical.report.experiment, status: historical.report.status}] : [])]};
      else if (url === "/api/reanalysis") payload = {status: "idle"};
      else if (url === "/api/stage-parameters" && stageSchema) payload = stageSchema;
      else if (url === "/api/reanalyze") payload = {status: "running", run: "derived", sources: JSON.parse(options.body).runs};
      else if (url.startsWith("/api/run?") && historical) payload = historical;
      else if (url === "/api/stop") payload = {message: "requested"};
      else if (url === "/api/start" || url === "/api/configure" || url === "/api/drag/start" ||
        url === "/api/fault/reset") payload = {generation: state.generation + 1};
      else throw new Error(`Unexpected dashboard request ${url}`);
      return {ok: true, async json() { return payload; }};
    },
  };
  const flush = async () => { await new Promise(setImmediate); await new Promise(setImmediate); };
  vm.runInNewContext(browserScript, context, {filename: "dashboard.js"});
  await flush();
  return {nodes, selectors, document, snapshot, requests, flush,
    async poll() { assert(timers.length); await timers.shift()(); await flush(); },
    event(id, type) { nodes.get(id).dispatchEvent(new context.Event(type)); }};
}
async function browserAssertions() {
  const errors = [], capture = error => errors.push(error);
  process.on("unhandledRejection", capture);
  try {
    for (const experiment of Object.keys(view.experiments)) {
      const app = await browserHarness(dashboardState(experiment));
      assert.equal(app.nodes.get("connection").textContent, "实时连接",
        `${experiment} must render without a scope error: ${app.nodes.get("connection-notice").textContent}; ${errors.map(String).join("; ")}`);
      assert.equal(app.nodes.get("position-value").textContent, "1.2345");
      assert.equal(app.nodes.get("start").disabled, false);
      assert.equal(app.nodes.get("fourier-panel").hidden, experiment !== "fourier");
      assert.equal(app.nodes.get("precision-panel").hidden, experiment !== "precision");
      assert(app.requests.every(request => request.method === "GET"), "Opening the page must never configure/start/stop");
      assert.equal(app.document.documentElement.style["--topbar-height"], "160px");
      assert.equal(app.document.documentElement.style["--workspace-offset"], "220px");
      if (experiment === "fourier") {
        assert.equal(app.nodes.get("calibration_file").required, false);
        assert.equal(app.nodes.get("calibration-choice").hidden, true);
        assert(!app.nodes.has("legacy-excitation") && !app.nodes.has("tracking-gains") && !app.nodes.has("advanced"));
        assert.match(app.nodes.get("calibration-summary").textContent, /计算与质量验收/);
      }
    }
    const state = dashboardState();
    Object.assign(state, {phase: "running", stage: "fourier_periodic", can_start: false, stop_confirmed: false, run_id: "current-run"});
    state.precision.phase = "periodic";
    state.precision.fourier = {reference_started: 1, role: 1, completed_cycles: 4, required_cycles: 28, samples: 1234};
    const historical = {id: "historical-run", trace: [], artifacts: [], raw_samples: 42, fit: null,
      report: {experiment: "fourier", status: "completed_unqualified", identity: {namespace: "/historical_motor"},
        events: [], fourier: {measured_cycles: 99, required_cycles: 99, quality_issues: ["historical quality"],
          metrics: {training_rmse: 0.01, validation_rmse: 0.02, maximum_validation_error: 0.03, condition: 4},
          candidate_model: {parameters: {coulomb: 0.777}, units: {coulomb: "Nm"}, warnings: ["historical model warning"]}}}};
    const app = await browserHarness(state, historical);
    app.nodes.get("history").value = historical.id; app.event("history", "change"); await app.flush();
    assert.match(app.nodes.get("fourier-current").textContent, /4\/28/);
    assert.doesNotMatch(app.nodes.get("fourier-current").textContent, /99/);
    assert.match(app.nodes.get("fourier-parameters").textContent, /0.7770000/);
    assert.match(app.nodes.get("fourier-notes").textContent, /historical model warning/);
    assert.match(app.nodes.get("motor-name").textContent, /current_motor/);
    assert.equal(app.nodes.get("position-value").textContent, "1.2345");
    assert.equal(app.nodes.get("editable").disabled, true);
    assert.equal(app.nodes.get("start").disabled, true);
    assert.match(app.nodes.get("view-notice").textContent, /历史/);
    assert(app.requests.every(request => request.method === "GET"), "History selection is read-only");
    app.document.dispatchEvent({type: "keydown", key: "Escape", preventDefault() {}});
    await app.flush();
    assert.deepEqual(app.requests.filter(r => r.method === "POST"), [{url: "/api/stop", method: "POST", body: {}}]);
    assert.equal(app.nodes.get("disabled-state").textContent, "实验运行中", "STOP acceptance is not disable confirmation");
    app.snapshot.connected = false; await app.poll();
    assert.match(app.selectors[".metrics"].className, /stale/);
    assert.match(app.nodes.get("controller-stage").textContent, /上次状态/);
    assert.equal(app.nodes.get("editable").disabled, true);
    assert.equal(app.nodes.get("start").disabled, true);
    assert.equal(app.nodes.get("disabled-state").textContent, "失联 · 停机状态未知");
    app.snapshot.state = null; await app.poll();
    assert.equal(app.nodes.get("controller-stage").textContent, "等待标定器状态");
    const precise = dashboardState("precision");
    Object.assign(precise, {phase: "running", stage: "precision_sample", can_start: false, stop_confirmed: false});
    Object.assign(precise.precision, {phase: "sample", completed_cells: 3, cells: 110, target_speed: 0.1, repetition: 2});
    const olderPrecision = {id: "older-precision", trace: [], artifacts: [], fit: null, report: {
      experiment: "precision", status: "completed_unqualified", events: [], precision: {
        qualified: false, measured_cells: 99, required_cells: 99, levels: [], observations: [], validation: [],
        settings: {...precise.precision_settings, speeds: [0.2], time_budget: 900}, quality_issues: ["older precision warning"],
      }}};
    const precisionApp = await browserHarness(precise, olderPrecision);
    precisionApp.nodes.get("history").value = olderPrecision.id;
    precisionApp.event("history", "change"); await precisionApp.flush();
    assert.match(precisionApp.nodes.get("precision-current").textContent, /速度档 3\/110/);
    assert.match(precisionApp.nodes.get("precision-budget").textContent, /3 个主档/);
    assert.match(precisionApp.nodes.get("precision-issues").textContent, /older precision warning/);
    assert.equal(precisionApp.nodes.get("fourier-protocol-details").hidden, true);
    assert.equal(precisionApp.nodes.get("editable").disabled, true);
    const completed = dashboardState();
    Object.assign(completed, {phase: "completed_unqualified", stage: "completed_unqualified", report: historical.report});
    const completeApp = await browserHarness(completed);
    assert.match(completeApp.nodes.get("fourier-current").textContent, /99\/99/);
    assert.match(completeApp.nodes.get("fourier-parameters").textContent, /0.7770000/);
    const idle = await browserHarness(dashboardState());
    for (const name of view.selectable) assert.equal(idle.nodes.get(`collect_${name}`).checked, true);
    idle.nodes.get("collect_static").checked = false;
    idle.event("collect_static", "input");
    assert.equal(idle.nodes.get("step-state-static").textContent, "跳过");
    idle.event("settings", "submit"); await idle.flush();
    const startRequest = idle.requests.find(r => r.method === "POST");
    assert.equal(startRequest.url, "/api/start");
    assert.equal(startRequest.body.controller_id, "current-controller");
    assert.equal(startRequest.body.settings.experiment, "fourier");
    assert.equal(startRequest.body.settings.calibration_file, "");
    assert.equal(startRequest.body.settings.collect_static, false);
    assert.equal(idle.nodes.get("start").disabled, true, "Duplicate starts must wait for the next controller generation");
    const rawRuns = ["old", "new"].map(id => ({
      id, raw_recording: true, experiment: "precision", status: "completed_unqualified", samples: 100,
      identity: {namespace: "/current_motor"},
    }));
    const analysisApp = await browserHarness(dashboardState("precision"), null, rawRuns);
    assert.equal(analysisApp.nodes.get("reanalyze").disabled, true);
    for (const label of analysisApp.nodes.get("reanalysis-sources").children) {
      label.children[0].checked = true;
      label.children[0].dispatchEvent({type: "change"});
    }
    assert.equal(analysisApp.nodes.get("reanalyze").disabled, false);
    analysisApp.event("reanalyze", "click"); await analysisApp.flush();
    assert.deepEqual(analysisApp.requests.filter(r => r.method === "POST"),
      [{url: "/api/reanalyze", method: "POST", body: {runs: ["new", "old"]}}]);
    assert.equal(analysisApp.nodes.get("reanalyze").disabled, true);
    assert.match(analysisApp.nodes.get("reanalysis-status").textContent, /正在核验/);
    const oldState = dashboardState();
    for (const name of view.selectable) delete oldState.configuration[`collect_${name}`];
    const oldApp = await browserHarness(oldState);
    assert.equal(oldApp.nodes.get("collect_static").disabled, true);
    assert.match(oldApp.nodes.get("stage-selection-note").textContent, /尚不支持/);
    const progressState = dashboardState("fourier");
    Object.assign(progressState, {phase: "running", stage: "fourier_periodic"});
    Object.assign(progressState.precision, {phase: "periodic", training_completed: 4, training_required: 18,
      validation_completed: 0, validation_required: 10, fourier: {cycle: 4, role: 1}});
    const progressRows = view.stageRows(progressState, progressState.configuration);
    assert.match(progressRows.find(row => row.id === "training").detail, /4\/18.*22%/);
    assert.equal(progressRows.find(row => row.id === "training").status, "执行中");
    assert.equal(progressRows.find(row => row.id === "validation").status, "等待");
    Object.assign(progressState, {phase: "failed", stage: "failed", failure_stage: "running"});
    const interrupted = view.stageRows(progressState, progressState.configuration);
    assert.equal(interrupted.find(row => row.id === "training").status, "中断");
    assert.equal(interrupted.find(row => row.id === "validation").status, "未执行");
    assert.equal(interrupted.find(row => row.id === "analysis").status, "未执行");
    const schema = {
      groups: Object.fromEntries(["checking", "warmup", "baseline", "static", "training", "validation", "stopping", "saving", "analysis"]
        .map(id => [id, {title: id, note: "固定保护不可放宽", fields: id === "baseline" ? ["precision_baseline_time"] : []}])),
      fields: {precision_baseline_time: {label: "零偏时间", type: "number", min: 0.1, max: 300}},
      defaults: {precision_baseline_time: 10},
      test_profiles: {fourier: {precision_baseline_time: 115, protocol_mode: "workflow_test"}},
    };
    const dialogState = dashboardState();
    Object.assign(dialogState.configuration, {precision_baseline_time: 10, precision_baseline_settle_time: 5, protocol_mode: "full"});
    const dialogApp = await browserHarness(dialogState, null, [], schema);
    for (const id of Object.keys(schema.groups)) {
      dialogApp.event(`parameters-${id}`, "click");
      assert.equal(dialogApp.nodes.get("stage-parameter-dialog").open, true);
      dialogApp.event("stage-parameter-cancel", "click");
      assert.equal(dialogApp.nodes.get("stage-parameter-dialog").open, false);
    }
    assert(dialogApp.requests.every(request => request.method === "GET"), "Opening parameter dialogs is read-only");
    dialogApp.event("parameters-baseline", "click");
    dialogApp.nodes.get("stage-parameter-fields").children[0].children[1].value = "120";
    dialogApp.event("stage-parameter-form", "submit");
    assert.equal(dialogApp.nodes.get("stage-parameter-dialog").open, false);
    assert(dialogApp.requests.every(request => request.method === "GET"), "Saving a dialog only changes the draft");
    dialogApp.event("apply", "click"); await dialogApp.flush();
    assert.equal(dialogApp.requests.find(request => request.method === "POST").body.settings.precision_baseline_time, 120);
    const presetApp = await browserHarness(dialogState, null, [], schema);
    presetApp.event("stage-profile-test", "click");
    assert.match(presetApp.nodes.get("protocol-mode").textContent, /流程测试/);
    presetApp.event("stage-profile-full", "click");
    assert.match(presetApp.nodes.get("protocol-mode").textContent, /完整标定/);
    Object.assign(presetApp.snapshot.state, {phase: "running", stop_confirmed: false});
    await presetApp.poll();
    presetApp.event("parameters-baseline", "click");
    assert.equal(presetApp.nodes.get("stage-parameter-save").disabled, true);
    assert.equal(presetApp.nodes.get("stage-parameter-fields").children[0].children[1].disabled, true);
    const arraySpec = {label: "速度档", type: "array", min: 0.01, max: 7};
    assert.deepEqual(view.parseStageValue("0.3,1,6.283", arraySpec), [0.3, 1, 6.283]);
    for (const text of ["1,,3", "[1,null,3]", "[1,true,3]", "NaN,1,3"])
      assert.throws(() => view.parseStageValue(text, arraySpec));
    assert.throws(() => view.parseStageValue("3.5", {label: "次数", type: "integer", min: 3, max: 8}));
    assert.equal(view.parseStageValue(false, {label: "偏置", type: "boolean"}), false);
    assert.throws(() => view.parseStageValue("false", {label: "偏置", type: "boolean"}));
    assert.equal(view.parseStageValue("auto", {label: "模型", type: "choice", choices: ["auto"]}), "auto");
    const dragState = dashboardState("fourier");
    dragState.calibration_configuration = {...dragState.configuration};
    const dragModel = {id: "model-run", eligible: true, model: "coulomb_viscous",
      calibration_file: "/models/calibration.json", domain: {velocity_min: -0.32, velocity_max: 0.32},
      quality: {rmse_limit: 0.02}, model_warnings: ["low speed only"]};
    const dragRecord = {id: "drag-run", trace: [], artifacts: [], fit: null,
      report: {experiment: "compensate", status: "completed", events: []}};
    const dragApp = await browserHarness(dragState, dragRecord, [dragModel]);
    dragApp.nodes.get("amplitude").value = "0.123"; dragApp.event("amplitude", "input");
    dragApp.nodes.get("drag-calibration-file").value = dragModel.calibration_file;
    assert(!dragApp.nodes.has("drag-duration") && !dragApp.nodes.has("drag-remaining"));
    dragApp.nodes.get("drag-fraction").value = "0.4";
    dragApp.event("drag-calibration-file", "input");
    assert.match(dragApp.nodes.get("drag-model-note").textContent, /0.320/);
    dragApp.event("drag-settings", "submit"); await dragApp.flush();
    assert.match(dragApp.nodes.get("drag-status").textContent, /等待/);
    assert.doesNotMatch(dragApp.nodes.get("drag-status").textContent, /^就绪/);
    assert.deepEqual(dragApp.requests.filter(r => r.method === "POST").map(r => r.url), ["/api/drag/start"]);
    assert.deepEqual(dragApp.requests.find(r => r.url === "/api/drag/start").body.settings,
      {calibration_file: dragModel.calibration_file, compensation_fraction: 0.4});
    Object.assign(dragState, {experiment: "compensate", generation: 2, phase: "running", stage: "assistance",
      stop_confirmed: false, can_start: false, run_id: "drag-run", leg_elapsed: 603.2,
      target: {feedforward: 0.04}, configuration: {...dragState.configuration, experiment: "compensate",
        duration: null, drag_stop_policy: "operator", compensation_fraction: 0.4}});
    await dragApp.poll();
    assert.equal(dragApp.nodes.get("experiment").value, "fourier");
    assert.equal(dragApp.nodes.get("amplitude").value, "0.123", "Drag must preserve an unapplied calibration draft");
    assert.equal(dragApp.nodes.get("compensation_fraction").value, "0.8");
    assert.equal(dragApp.nodes.get("drag-elapsed").textContent, "603.2");
    assert.match(dragApp.nodes.get("drag-status").textContent, /无定时结束，请手动停止/);
    assert.equal(dragApp.nodes.get("drag-output").textContent, "0.0400");
    const configRows = dragApp.nodes.get("constraint-rows").textContent;
    assert.match(configRows, /未设置（null）/);
    assert.match(configRows, /duration/);
    dragApp.nodes.get("constraint-filter").value = "duration";
    dragApp.event("constraint-filter", "input");
    assert.match(dragApp.nodes.get("constraint-rows").textContent, /duration/);
    assert.doesNotMatch(dragApp.nodes.get("constraint-rows").textContent, /compensation_fraction/);
    const posts = dragApp.requests.filter(r => r.method === "POST").length;
    dragApp.snapshot.connected = false;
    await dragApp.poll();
    assert.match(dragApp.nodes.get("constraint-status").textContent, /连接中断/);
    assert.equal(dragApp.requests.filter(r => r.method === "POST").length, posts);
    dragApp.snapshot.connected = true;
    await dragApp.poll();
    Object.assign(dragState, {recording_status: {
      recording_complete: false, finalized: false, error: "",
      streams: {"samples.csv": {enqueued: 100, written: 90, synced: 80, queued: 10, closed: false, pending_file: "samples.pending.csv"}},
      usage: {counts: {warmup: 100, training: 0}, descriptions: {warmup: "预运行，不进入拟合", training: "训练候选"}},
    }});
    await dragApp.poll();
    assert.match(dragApp.nodes.get("recording-status").textContent, /记录进行中/);
    assert.match(dragApp.nodes.get("recording-streams").textContent, /samples.pending.csv/);
    assert.match(dragApp.nodes.get("recording-usage").textContent, /预运行，不进入拟合100/);
    assert.match(dragApp.nodes.get("recording-selection").textContent, /尚无最终窗口索引/);
    assert.equal(dragApp.nodes.get("start").disabled, true);
    assert.equal(dragApp.nodes.get("drag-start").disabled, true);
    dragApp.event("drag-stop", "click"); await dragApp.flush();
    assert.equal(dragApp.nodes.get("disabled-state").textContent, "实验运行中");
    Object.assign(dragState, {phase: "completed", stage: "completed", stop_confirmed: true, can_start: true});
    await dragApp.poll();
    assert.equal(dragApp.nodes.get("drag-output").textContent, "—", "A cached target is not a live output after disable");
    assert.equal(dragApp.nodes.get("amplitude").value, "0.123");
    assert.equal(dragApp.nodes.get("start").disabled, false);
    dragApp.event("settings", "submit"); await dragApp.flush();
    const calibrationStart = dragApp.requests.find(r => r.url === "/api/start").body.settings;
    assert.equal(calibrationStart.experiment, "fourier");
    assert.equal(calibrationStart.amplitude, 0.123);
    assert.equal(calibrationStart.duration, 20);
    assert.equal(calibrationStart.compensation_fraction, 0.8);
    const modelSchema = {
      ...schema,
      groups: {...schema.groups, analysis: {title: "辨识", note: "模型与可辨识性",
        fourier: ["model", "fit_bias", "known_bias", "fit_gravity", "fit_stiffness"]}},
      fields: {...schema.fields,
        model: {label: "模型", type: "choice", choices: ["auto", "asymmetric_coulomb_viscous"]},
        fit_bias: {label: "拟合偏置", type: "boolean"}, fit_gravity: {label: "重力", type: "boolean"},
        fit_stiffness: {label: "刚度", type: "boolean"},
        known_bias: {label: "独立偏置", type: "number", min: -10, max: 10}},
    };
    const modelApp = await browserHarness(dashboardState(), null, [], modelSchema);
    modelApp.event("parameters-analysis", "click");
    const modelInputs = Object.fromEntries(modelApp.nodes.get("stage-parameter-fields").children
      .flatMap(label => label.children).filter(node => node.name).map(node => [node.name, node]));
    assert.equal(modelInputs.known_bias.disabled, true);
    modelInputs.model.value = "asymmetric_coulomb_viscous";
    modelApp.event("stage-parameter-form", "submit");
    assert.match(modelApp.nodes.get("stage-parameter-error").textContent, /关闭偏置拟合/);
    assert.equal(modelApp.nodes.get("model").value, "auto", "Invalid dialog changes must not leak into the calibration draft");
    modelInputs.fit_bias.checked = false;
    modelInputs.fit_bias.dispatchEvent({type: "input"});
    assert.equal(modelInputs.known_bias.disabled, false);
    modelInputs.known_bias.value = "0.01";
    modelApp.event("stage-parameter-form", "submit");
    assert.equal(modelApp.nodes.get("model").value, "asymmetric_coulomb_viscous");
    assert.equal(modelApp.nodes.get("fit_bias").checked, false);
    assert.equal(modelApp.nodes.get("known_bias").value, "0.01");
    assert(modelApp.requests.every(request => request.method === "GET"));
    const constraintSchema = {...schema, groups: {...schema.groups,
      constraints: {title: "应用层约束", fields: ["max_torque", "compensation_slew"], note: "保留后端校验"}},
      fields: {...schema.fields,
        max_torque: {label: "输出力矩预算 / Nm", type: "number", min: 0, max: 0.8},
        compensation_slew: {label: "力矩变化率 / Nm/s", type: "number", min: 0, max: 5}}};
    const editableConstraintState = dashboardState("calibrate");
    editableConstraintState.configuration.compensation_slew = 5;
    const constraintApp = await browserHarness(editableConstraintState, null, [], constraintSchema);
    assert.equal(constraintApp.nodes.get("constraint-edit").disabled, false);
    constraintApp.event("constraint-edit", "click");
    const constraintInputs = Object.fromEntries(constraintApp.nodes.get("stage-parameter-fields").children
      .flatMap(label => label.children).filter(node => node.name).map(node => [node.name, node]));
    assert.equal(constraintInputs.max_torque.value, "0.4");
    constraintInputs.max_torque.value = "0.9";
    constraintApp.event("stage-parameter-form", "submit");
    assert.equal(constraintApp.nodes.get("stage-parameter-error").hidden, false);
    constraintInputs.max_torque.value = "0.6";
    constraintInputs.compensation_slew.value = "2";
    constraintApp.event("stage-parameter-form", "submit");
    assert(constraintApp.requests.every(request => request.method === "GET"), "Editing only changes the draft");
    constraintApp.event("apply", "click"); await constraintApp.flush();
    const constraintRequest = constraintApp.requests.find(request => request.url === "/api/configure");
    assert.equal(constraintRequest.body.settings.max_torque, 0.6);
    assert.equal(constraintRequest.body.settings.compensation_slew, 2);
    assert(!constraintApp.requests.some(request => request.url === "/api/start" || request.url === "/api/drag/start"));
    Object.assign(editableConstraintState, {phase: "running", stop_confirmed: false,
      generation: editableConstraintState.generation + 1});
    await constraintApp.poll();
    assert.equal(constraintApp.nodes.get("constraint-edit").disabled, true);
    const hardwareState = dashboardState();
    hardwareState.motor.simulated = false;
    hardwareState.configuration.allow_hardware = true;
    const hardwareApp = await browserHarness(hardwareState, null, [dragModel]);
    hardwareApp.nodes.get("drag-calibration-file").value = dragModel.calibration_file;
    hardwareApp.nodes.get("drag-fraction").value = "0.5";
    hardwareApp.event("drag-calibration-file", "input");
    assert.equal(hardwareApp.nodes.get("drag-start").disabled, true, "Real drag requires its own explicit confirmation");
    hardwareApp.event("drag-settings", "submit"); await hardwareApp.flush();
    assert(hardwareApp.requests.every(request => request.method === "GET"));
    assert.match(hardwareApp.nodes.get("drag-action").textContent, /确认/);
    const limitedState = dashboardState();
    Object.assign(limitedState.configuration, {max_speed: 1, max_travel: 0.7});
    limitedState.identity = {namespace: "/current_motor", motor_id: 2};
    limitedState.driver_motion_limits = {enabled: true, position_min: -350, position_max: 350,
      speed_stop: 13, verified: true};
    const lastFailure = {id: "last-failed-drag", experiment: "compensate", status: "failed",
      failure_reason: "Speed limit reached", identity: limitedState.identity,
      stop_limits: {max_speed: 1, max_travel: 0.7}};
    const limited = await browserHarness(limitedState, null, [dragModel, lastFailure]);
    await limited.poll();
    assert(!limited.nodes.has("drag-max-speed") && !limited.nodes.has("drag-max-travel"));
    assert.match(limited.nodes.get("drag-motion-note").textContent, /不再设置额外软件门限/);
    assert.match(limited.nodes.get("drag-limits").textContent, /13.000 rad\/s/);
    assert.match(limited.nodes.get("drag-limits").textContent, /独立拖动不启用/);
    assert.equal(limited.nodes.get("drag-last-stop").hidden, false, "Saved failure must remain visible after controller restart");
    assert.match(limited.nodes.get("drag-last-stop").textContent, /速度保护.*速度 1 rad\/s/);
    limited.nodes.get("drag-calibration-file").value = dragModel.calibration_file;
    limited.nodes.get("drag-fraction").value = "0.5";
    limited.event("drag-settings", "submit"); await limited.flush();
    assert.deepEqual(limited.requests.find(r => r.url === "/api/drag/start").body.settings,
      {calibration_file: dragModel.calibration_file, compensation_fraction: 0.5});
    assert.equal(limited.nodes.get("max_speed"), undefined, "General calibration safety limits are not editable");
    const unsafeDriverState = dashboardState();
    unsafeDriverState.driver_motion_limits = {enabled: false, position_min: -350, position_max: 350,
      speed_stop: 13, verified: true};
    const unsafeDriverApp = await browserHarness(unsafeDriverState, null, [dragModel]);
    unsafeDriverApp.nodes.get("drag-calibration-file").value = dragModel.calibration_file;
    unsafeDriverApp.event("drag-calibration-file", "input");
    assert.equal(unsafeDriverApp.nodes.get("drag-start").disabled, true);
    assert.match(unsafeDriverApp.nodes.get("drag-status").textContent, /驱动的速度/);
    const resetState = dashboardState();
    Object.assign(resetState, {can_start: false, can_reset_fault: true, phase: "failed"});
    Object.assign(resetState.motor, {simulated: false, host_fault: true, phase: 4, status: 0});
    const resetApp = await browserHarness(resetState);
    assert.equal(resetApp.nodes.get("fault-reset").disabled, true);
    resetApp.nodes.get("fault-reset-confirm").checked = true;
    resetApp.event("fault-reset-confirm", "change");
    assert.equal(resetApp.nodes.get("fault-reset").disabled, false);
    resetApp.nodes.get("amplitude").value = "0.123"; resetApp.event("amplitude", "input");
    resetApp.event("fault-reset", "click"); await resetApp.flush();
    assert.deepEqual(resetApp.requests.filter(r => r.method === "POST").map(r => r.url), ["/api/fault/reset"]);
    assert.equal(resetApp.requests.find(r => r.url === "/api/fault/reset").body.confirm_hardware, true);
    Object.assign(resetState, {phase: "resetting", can_reset_fault: false, generation: resetState.generation + 1});
    await resetApp.poll();
    assert.equal(resetApp.nodes.get("fault-reset").disabled, true);
    assert.equal(resetApp.nodes.get("start").disabled, true);
    Object.assign(resetState, {phase: "idle", can_start: true});
    resetState.motor.host_fault = false;
    await resetApp.poll();
    assert.equal(resetApp.nodes.get("amplitude").value, "0.123");
    assert.equal(resetApp.nodes.get("fault-reset-confirm").checked, false);
    assert.equal(resetApp.requests.filter(r => r.url === "/api/start" || r.url === "/api/drag/start").length, 0);
    assert.match(view.reason("Speed limit reached: |velocity|=1.23321 rad/s >= 1 rad/s"), /速度保护.*1.23321/);
    assert.deepEqual(errors, [], errors.map(error => error.stack).join("\n"));
    console.log("Dashboard controller and live/history isolation assertions passed");
  } finally { process.removeListener("unhandledRejection", capture); }
}
browserAssertions().catch(error => { console.error(error); process.exitCode = 1; });
