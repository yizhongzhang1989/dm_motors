"use strict";

const assert = require("assert").strict;
const view = require("../web/view_model.js");
assert(view.completed.has("completed_unqualified") && view.terminal.has("completed_unqualified"));
assert.equal(view.number(NaN), "—");
{
  const report = {experiment: "speed_sweep", excluded_diagnostic_trials: [
    {trial: "setup", had_enable_request: false, commands: 0, startup_only: false, stop_confirmed: true, errors: ["setup error"]},
    {trial: "startup", had_enable_request: true, commands: 8, startup_only: true, stop_confirmed: true, errors: ["deadline"]},
  ]};
  const rows = view.speedSweepDiagnostics(report);
  assert.deepEqual(rows[0], ["setup", "未请求", "0 条目标", "已确认", "setup error"]);
  assert.deepEqual(rows[1], ["startup", "已请求", "8 条 baseline 起点保持目标", "已确认", "deadline"]);
  assert.equal(report.excluded_diagnostic_trials[1].had_enable_request, true);
  assert.deepEqual(view.speedSweepDiagnostics({experiment: "calibrate"}), []);
}
{
  const detail = {id: "20261106T120000-01234567", raw_samples: 50,
    display_fields: ["time", "position", "velocity", "torque"], trace: [[0, 0, 0, 0, 0], [1, 5, 1, 13.2, .3]],
    fit: {kind: "sliding_friction", parameters: {coulomb: .1, viscous: .02, bias: 0}},
    report: {experiment: "speed_sweep", status: "completed_unqualified", stop_confirmed: true,
      identity: {namespace: "/motor_3", motor_id: 3}, configuration: {topology: "separated"},
      speed_sweep: {qualified: false, prediction_qualified: true, acquisition_quality_passed: true,
        coverage: {complete_trials: 11, expected_trials: 11}}}};
  const restored = view.experimentState(detail);
  assert.equal(restored.experiment, "speed_sweep");
  assert.equal(restored.elapsed, 5);
  assert.equal(restored.fit.kind, "sliding_friction");
  assert(!("inertia" in restored.fit.parameters));
  const steps = view.stageRows(restored, {}).filter(item => item.visible);
  assert(steps.some(item => item.id === "validation" && item.label === "整轮独立验证"));
  assert(!steps.some(item => item.id === "static" || item.id === "warmup"));
  assert.equal(steps.find(item => item.id === "analysis").status, "预测通过");
  assert.equal(view.latestExperiment([{id: detail.id, same_motor: true, identity: detail.report.identity}], null, "/motor_3"), detail.id);
  assert(view.experiments.speed_sweep.includes("高速"));
}
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
elements.quick_excitation.value = "fixed";
elements.fit_bias.checked = true;
assert.equal(view.readValues(elements).fit_bias, true);
assert.equal(view.readValues(elements).known_bias, 1);
assert.equal(view.readValues(elements).model, "stribeck");
elements.quick_excitation.value = "auto";
for (const experiment of ["validate", "compensate", "breakaway", "precision", "fourier", "calibrate"]) {
  elements.experiment.value = experiment;
  assert.equal(view.readValues(elements).quick_excitation, "auto", "Other tasks retain the chosen quick mode");
}
elements.quick_excitation.value = "unknown";
assert.throws(() => view.readValues(elements), /quick_excitation/);
elements.quick_excitation.value = "fixed";
elements.duration.value = "";
assert.throws(() => view.readValues(elements), /duration/);
elements.duration.value = "Infinity";
assert.throws(() => view.readValues(elements), /duration/);
assert(!view.editable.includes("allow_hardware"));
assert(!view.editable.includes("max_torque"));
assert.deepEqual(view.quickTiming({duration: 40}), {training: 40, validation: 20});
assert.deepEqual(view.quickTiming({duration: 8}), {training: 8, validation: 16});
assert.deepEqual(view.quickTiming({duration: 80}), {training: 80, validation: 24});
assert.equal(view.quickTiming({duration: null}), null);
const sharedBands = {
  negative: {velocity_min: -.512479, velocity_max: -.286676, windows: {training: 25, external_validation: 23}},
  positive: {velocity_min: .271294, velocity_max: .524166, windows: {training: 30, external_validation: 22}},
};
const bandsBefore = JSON.stringify(sharedBands);
assert(!view.speedInSharedRanges(.1, sharedBands));
assert(!view.speedInSharedRanges(0, sharedBands));
assert(view.speedInSharedRanges(.3, sharedBands) && view.speedInSharedRanges(-.3, sharedBands));
assert(!view.speedInSharedRanges(.3, {}));
assert.match(view.sharedRangeText(null), /未记录/);
assert.deepEqual(view.sharedModelCurves([[[-.5, -.1], [-.3, -.1], [-.01, -.1], [.01, .1], [.3, .1], [.5, .1]]], sharedBands),
  [[[-.5, -.1], [-.3, -.1]], [[.3, .1], [.5, .1]]]);
assert.deepEqual(view.sharedModelCurves([[[-.5, -.1], [.5, .1]]], {}), []);
assert.equal(JSON.stringify(sharedBands), bandsBefore);
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
  const configuration = {experiment, model: "auto", calibration_file: "", quick_excitation: "fixed", duration: 20, amplitude: 0.4,
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
function quickProfile(state) {
  const c = state?.calibration_configuration || state?.configuration || dashboardState().configuration;
  return {id: "quick_mechanical", protocol: "quick_independent_v1", title: "快速机械标定",
    settings: {experiment: "calibrate", protocol_mode: "full", duration: 40, model: "auto",
      amplitude: c.amplitude, frequency: c.frequency, kp: c.kp, kd: c.kd,
      fit_bias: c.fit_bias, known_bias: c.known_bias, fit_gravity: c.fit_gravity, fit_stiffness: c.fit_stiffness},
    training_seconds: 40, validation_seconds: 20};
}
async function browserHarness(state, historical = null, rawRuns = [], stageSchema = undefined) {
  if (stageSchema === undefined) stageSchema = {groups: {}, fields: {}, defaults: {}};
  if (stageSchema && !Object.hasOwn(stageSchema, "quick_profile")) stageSchema = {...stageSchema, quick_profile: quickProfile(state)};
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
      reanalysis: true, manual_parameter_save: true, drag_control: true, drag_manual_motion: true, drag_manual_stop: true,
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
      else if (url.startsWith("/api/run?")) {
        const id = new URLSearchParams(url.split("?")[1]).get("run");
        const item = rawRuns.find(run => run.id === id);
        payload = historical?.id === id ? historical : {id, trace: [], display_fields: ["time", "position"], artifacts: [],
          report: {experiment: item?.experiment || "fourier", status: item?.status || "completed", events: [],
            error: item?.error || item?.failure_reason || "", configuration: {}}};
      }
      else if (url === "/api/stop") payload = {message: "requested"};
      else if (url === "/api/parameters/save")
        payload = {generation: state.generation + 1, saved: {system_parameters_file: "/test/system.yaml"}};
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
      assert.equal(app.nodes.get("experiment").value, "calibrate", "A new draft always uses the supported quick workflow");
      assert.equal(app.nodes.get("duration").value, "40");
      assert.match(app.nodes.get("draft-plan").textContent, /40 秒训练 \+ 20 秒独立验证/);
      assert(!app.nodes.has("fourier-panel"), "Candidate and final model share one result panel");
      assert.equal(app.nodes.get("precision-panel").hidden, experiment !== "precision");
      assert(app.requests.every(request => request.method === "GET"), "Opening the page must never configure/start/stop");
      assert.equal(app.document.documentElement.style["--topbar-height"], "160px");
      assert.equal(app.document.documentElement.style["--workspace-offset"], "220px");
      if (experiment === "fourier") {
        assert.equal(app.nodes.get("calibration_file").required, false);
        assert.equal(app.nodes.get("calibration-choice").hidden, true);
        assert(!app.nodes.has("legacy-excitation") && !app.nodes.has("tracking-gains") && !app.nodes.has("advanced"));
        assert.match(app.nodes.get("calibration-summary").textContent, /模型计算/);
      }
    }
    const saveState = dashboardState();
    saveState.system_parameters_file = "/test/system.yaml";
    saveState.identity = {namespace: "/current_motor", simulated: true};
    const saveRun = {id: "20261030T040000-12345678", eligible: true, automatic_qualified: false, calibration_file: "/test/report.json",
      model: "coulomb_viscous", save_test_run: "20261030T041000-87654321"};
    const saveApp = await browserHarness(saveState, null, [saveRun]);
    assert.match(saveApp.nodes.get("drag-calibration-file").textContent, /自动指标未达标/);
    assert.match(saveApp.nodes.get("parameter-save-model").textContent, /自动指标未达标/);
    saveApp.nodes.get("parameter-save-model").value = saveRun.id;
    saveApp.event("parameter-save-model", "change");
    assert(saveApp.nodes.get("parameter-save-submit").disabled);
    saveApp.nodes.get("parameter-save-confirm").checked = true;
    saveApp.event("parameter-save-confirm", "change");
    assert(!saveApp.nodes.get("parameter-save-submit").disabled);
    assert(saveApp.requests.every(request => request.method === "GET"));
    saveApp.event("parameter-save-form", "submit"); await saveApp.flush();
    const saveRequest = saveApp.requests.find(request => request.url === "/api/parameters/save");
    assert.deepEqual(saveRequest.body, {run: saveRun.id, test_run: saveRun.save_test_run, confirm: true,
      controller_id: saveState.controller_id, generation: saveState.generation});
    assert.match(saveApp.nodes.get("parameter-save-action").textContent, /已保存/);
    assert(!saveApp.nodes.get("parameter-save-confirm").checked);
    assert(saveApp.nodes.get("parameter-save-submit").disabled);
    assert(markup.indexOf('id="parameter-save-panel"') > markup.indexOf('id="drag-panel"'));
    const state = dashboardState();
    Object.assign(state, {phase: "running", stage: "fourier_periodic", can_start: false, stop_confirmed: false, run_id: "current-run"});
    state.precision.phase = "periodic";
    state.precision.fourier = {reference_started: 1, role: 1, completed_cycles: 4, required_cycles: 28, samples: 1234};
    const historical = {id: "historical-run", trace: [], artifacts: [], raw_samples: 42, fit: null,
      report: {experiment: "fourier", status: "completed_unqualified", identity: {namespace: "/historical_motor"},
        events: [], fourier: {measured_cycles: 99, required_cycles: 99, quality_issues: ["historical quality"],
          metrics: {training_rmse: 0.01, validation_rmse: 0.02, maximum_validation_error: 0.03, condition: 4},
          candidate_model: {parameters: {coulomb: 0.777}, units: {coulomb: "Nm"}, warnings: ["historical model warning"]}}}};
    historical.report.configuration = {...state.configuration, amplitude: 0.9};
    historical.report.dashboard_state = {precision: {phase:"done", training_completed:99, training_required:99}};
    historical.fit = {...historical.report.fourier.candidate_model, model: "coulomb_viscous", diagnostics: {}};
    const app = await browserHarness(state, historical);
    app.nodes.get("history").value = historical.id; app.event("history", "change"); await app.flush();
    assert.match(app.nodes.get("step-detail-training").textContent, /99\/99/);
    assert.equal(app.nodes.get("amplitude").value, "0.9");
    assert.match(app.nodes.get("parameter-table").textContent, /0.777000/);
    assert.match(app.nodes.get("fit-warnings").textContent, /historical model warning/);
    assert.match(app.nodes.get("motor-name").textContent, /current_motor/);
    assert.equal(app.nodes.get("position-value").textContent, "1.2345");
    assert.equal(app.nodes.get("editable").disabled, true);
    assert.equal(app.nodes.get("start").disabled, true);
    assert.match(app.nodes.get("view-notice").textContent, /已恢复实验记录/);
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
    assert.equal(precisionApp.nodes.get("experiment-status").textContent, "有质量提示");
    assert.equal(precisionApp.nodes.get("progress").attributes["aria-valuenow"], "100");
    assert.match(precisionApp.nodes.get("precision-issues").textContent, /older precision warning/);
    assert.equal(precisionApp.nodes.get("editable").disabled, true);
    const completed = dashboardState();
    Object.assign(completed, {phase: "completed_unqualified", stage: "completed_unqualified", report: historical.report});
    completed.run_id = historical.id;
    const completeApp = await browserHarness(completed, historical);
    assert.match(completeApp.nodes.get("step-detail-training").textContent, /99\/99/);
    assert.match(completeApp.nodes.get("parameter-table").textContent, /0.777000/);
    // A fresh tab always restores the latest matching experiment, even without a controller.
    const latest = structuredClone(historical);
    latest.id = "20261030T090000-00000001";
    latest.report.identity = {namespace: "/current_motor", simulated: true};
    latest.report.configuration.amplitude = 0.456;
    latest.report.dashboard_state = {elapsed: 123.4, progress: 1,
      precision: {phase:"done", training_completed:12, training_required:12, validation_completed:4, validation_required:4}};
    latest.report.samples = 24680;
    latest.display_fields = ["time", "position", "velocity", "torque"];
    latest.trace = [[0,0,1,2,3],[1,123.4,4,5,6]];
    const older = {id:"20261029T090000-00000001", experiment:"fourier", status:"completed"};
    const foreign = {id:"20261031T090000-00000001", same_motor:false, identity:{namespace:"/other"}};
    assert.equal(view.latestExperiment([foreign, older, {...latest.report,id:latest.id}], dashboardState()), latest.id);
    assert.equal(view.latestExperiment([older], {...dashboardState(),run_id:latest.id,phase:"running"}), "");
    const restored = await browserHarness(dashboardState(), latest, [foreign, older]);
    assert.equal(restored.nodes.get("history").value, latest.id);
    assert.equal(restored.nodes.get("amplitude").value, "0.456");
    assert.equal(restored.nodes.get("progress-text").textContent, "123.4 s");
    assert.match(restored.nodes.get("step-detail-training").textContent, /12\/12/);
    assert.match(restored.nodes.get("sample-count").textContent, /24,680/);
    assert.equal(restored.nodes.get("chart-window").value, "0");
    assert(restored.nodes.get("start").disabled);
    restored.nodes.get("history").value = older.id; restored.event("history", "change"); await restored.flush();
    assert.equal(restored.nodes.get("history").value, older.id);
    await restored.poll(); assert.equal(restored.nodes.get("history").value, older.id);
    const reopened = await browserHarness(null, latest, [foreign, older]);
    assert.equal(reopened.nodes.get("history").value, latest.id);
    assert.match(reopened.nodes.get("parameter-table").textContent, /0.777000/);
    assert.equal(reopened.nodes.get("progress-text").textContent, "123.4 s");
    assert(reopened.nodes.get("start").disabled && reopened.nodes.get("drag-start").disabled);
    assert.equal(reopened.nodes.get("position-value").textContent, "—");
    assert(reopened.requests.every(request => request.method === "GET"));
    restored.event("live-view", "click"); await restored.flush();
    assert.equal(restored.nodes.get("history").value, latest.id);
    restored.event("new-experiment", "click");
    assert.equal(restored.nodes.get("amplitude").value, "0.4", "New experiment uses controller settings, never archived settings");
    assert(!restored.nodes.get("start").disabled);
    assert(restored.requests.every(request => request.method === "GET"));
    assert.match(restored.nodes.get("parameter-table").textContent, /0.777000/, "Preparing an experiment keeps the previous result visible");
    const idle = await browserHarness(dashboardState());
    for (const name of view.selectable) assert.equal(idle.nodes.get(`collect_${name}`).checked, ["training", "validation"].includes(name));
    idle.nodes.get("collect_static").checked = false;
    idle.event("collect_static", "input");
    assert(idle.nodes.get("step-static").hidden, "Quick sliding calibration does not claim static friction acquisition");
    idle.event("settings", "submit"); await idle.flush();
    const startRequest = idle.requests.find(r => r.method === "POST");
    assert.equal(startRequest.url, "/api/start");
    assert.equal(startRequest.body.controller_id, "current-controller");
    assert.equal(startRequest.body.settings.experiment, "calibrate");
    assert.equal(startRequest.body.settings.duration, 40);
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
    dialogApp.nodes.get("experiment").value = "fourier"; dialogApp.event("experiment", "change");
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
    presetApp.nodes.get("experiment").value = "fourier"; presetApp.event("experiment", "change");
    presetApp.event("stage-profile-test", "click");
    assert.match(presetApp.nodes.get("protocol-mode").textContent, /流程诊断/);
    presetApp.event("stage-profile-full", "click");
    assert.match(presetApp.nodes.get("protocol-mode").textContent, /完整采集方案/);
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
    dragApp.event("new-experiment", "click");
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
    assert.equal(dragApp.nodes.get("experiment").value, "calibrate");
    assert.equal(dragApp.nodes.get("amplitude").value, "0.123", "Drag must preserve an unapplied calibration draft");
    assert.equal(dragApp.nodes.get("compensation_fraction").value, "0.8");
    assert.equal(dragApp.nodes.get("drag-elapsed").textContent, "603.2");
    assert.match(dragApp.nodes.get("drag-status").textContent, /无定时结束，请手动停止/);
    assert.equal(dragApp.nodes.get("drag-output").textContent, "0.0400");
    assert.match(dragApp.nodes.get("drag-model-note").textContent, /未记录可用共有范围/);
    assert.match(dragApp.nodes.get("drag-domain-alert").textContent, /未记录分方向共有滑动范围/);
    dragModel.shared_ranges = sharedBands;
    dragModel.domain = {velocity_min: -.512479, velocity_max: .524166};
    dragState.motor.velocity = .1;
    await dragApp.poll();
    assert(!dragApp.nodes.get("drag-domain-alert").hidden, "The low-speed hole is inside the model envelope but lacks shared validation");
    assert.match(dragApp.nodes.get("drag-domain-alert").textContent, /未获独立覆盖/);
    assert.match(dragApp.nodes.get("drag-model-note").textContent, /0.271.*训练 30 \/ 独立验证 22 窗/);
    dragState.motor.velocity = .3;
    await dragApp.poll();
    assert(dragApp.nodes.get("drag-domain-alert").hidden);
    dragState.motor.velocity = -.3;
    await dragApp.poll();
    assert(dragApp.nodes.get("drag-domain-alert").hidden);
    assert.match(dragApp.nodes.get("experiment-configuration").textContent, /durationnull/);
    assert.match(dragApp.nodes.get("experiment-configuration").textContent, /compensation_fraction0.4/);
    const posts = dragApp.requests.filter(r => r.method === "POST").length;
    dragApp.snapshot.connected = false;
    await dragApp.poll();
    assert.match(dragApp.nodes.get("connection-notice").textContent, /未连接/);
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
    assert.equal(calibrationStart.experiment, "calibrate");
    assert.equal(calibrationStart.amplitude, 0.123);
    assert.equal(calibrationStart.duration, 40);
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
    modelApp.nodes.get("experiment").value = "fourier"; modelApp.event("experiment", "change");
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
    hardwareApp.event("new-experiment", "click");
    assert(hardwareApp.nodes.get("start").disabled, "Real calibration uses the existing explicit hardware confirmation");
    hardwareApp.event("settings", "submit"); await hardwareApp.flush();
    assert(hardwareApp.requests.every(request => request.method === "GET"));
    hardwareApp.nodes.get("confirm_hardware").checked = true; hardwareApp.event("confirm_hardware", "change");
    assert(!hardwareApp.nodes.get("start").disabled);
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
    const oldNode = await browserHarness(dashboardState("compensate"), null, [], {groups: {}, fields: {}, quick_profile: null});
    assert.equal(oldNode.nodes.get("experiment").value, "calibrate");
    assert(oldNode.nodes.get("start").disabled);
    assert.match(oldNode.nodes.get("draft-plan").textContent, /尚未提供独立验证/);
    oldNode.event("settings", "submit"); await oldNode.flush();
    assert(oldNode.requests.every(r => r.method === "GET"), "An old node cannot execute a falsely labelled independent workflow");
    const seeded = dashboardState("compensate");
    seeded.configuration.duration = null;
    seeded.configuration.amplitude = 9;
    seeded.calibration_configuration = {...dashboardState().configuration, collect_validation: false};
    const canonical = quickProfile(dashboardState()); canonical.settings.amplitude = 0.55;
    canonical.available = true;
    canonical.selection = {status: "validated_local", source_report: "/fixture/report.json", source_sha256: "a".repeat(64)};
    const fresh = await browserHarness(seeded, null, [], {groups: {}, fields: {}, quick_profile: canonical});
    assert.equal(fresh.nodes.get("duration").value, "40");
    assert.equal(fresh.nodes.get("amplitude").value, "0.55");
    assert(fresh.nodes.get("collect_validation").checked);
    assert.equal(fresh.nodes.get("model").value, "auto");
    assert.match(fresh.nodes.get("draft-scope").textContent, /本机已验证方案/);
    fresh.nodes.get("amplitude").value = "0.56"; fresh.event("amplitude", "input");
    assert.match(fresh.nodes.get("draft-scope").textContent, /参数已修改，尚未验证/);
    assert.doesNotMatch(fresh.nodes.get("draft-scope").textContent, /本机已验证/);
    fresh.event("quick-profile", "click");
    assert.equal(fresh.nodes.get("amplitude").value, "0.55");
    assert.match(fresh.nodes.get("draft-scope").textContent, /本机已验证方案/);
    seeded.quick_profile = {...canonical, settings: {...canonical.settings, amplitude: 0.44},
      selection: {...canonical.selection, source_sha256: "b".repeat(64)}};
    await fresh.poll();
    assert.equal(fresh.nodes.get("amplitude").value, "0.44", "An untouched draft follows a fresh local profile");
    fresh.nodes.get("amplitude").value = "0.12"; fresh.event("amplitude", "input");
    seeded.quick_profile = {...canonical, settings: {...canonical.settings, amplitude: 0.43}};
    await fresh.poll();
    assert.equal(fresh.nodes.get("amplitude").value, "0.12", "A changed backend profile cannot overwrite user edits");
    assert.match(fresh.nodes.get("draft-scope").textContent, /参数已修改/);
    assert(!fresh.nodes.get("step-validation").hidden);
    assert(fresh.requests.every(r => r.method === "GET"));
    seeded.quick_profile = {...canonical, available: false,
      selection: {status: "unavailable", reason: "Excitation must fit inside travel limits"}};
    await fresh.poll();
    assert(fresh.nodes.get("start").disabled && fresh.nodes.get("quick-profile").disabled);
    assert.match(fresh.nodes.get("draft-plan").textContent, /快速方案暂不可用.*行程范围/);
    fresh.event("settings", "submit"); await fresh.flush();
    assert(fresh.requests.every(r => r.method === "GET"), "An unavailable profile cannot submit START");
    const candidate = await browserHarness(dashboardState(), null, [], {groups: {}, fields: {}, quick_profile: {
      ...canonical, selection: {status: "startup_candidate", reason: "No explicit local validated recipe"}}});
    assert.match(candidate.nodes.get("draft-scope").textContent, /未验证启动候选/);
    assert.doesNotMatch(candidate.nodes.get("draft-scope").textContent, /本机已验证方案/);
    const quickResult = {id: "20261104T160000-01234567", trace: [], display_fields: ["time", "position"], artifacts: [],
      fit: {model: "coulomb_viscous", parameters: {coulomb: 0.12}, units: {coulomb: "Nm"}, warnings: [],
        diagnostics: {validation_rmse: 9.876543}},
      report: {experiment: "calibrate", status: "completed", identity: {namespace: "/current_motor"},
        configuration: {...canonical.settings, motor_namespace: "/current_motor"}, events: [], stop_confirmed: true,
        quick: {protocol: view.quickProtocol, qualified: true, quality_issues: [],
          training: {duration: 39.99, raw_samples: 8000},
          external_validation: {duration: 19.99, raw_samples: 4000, rmse: 0.012345, maximum_error: 0.033333, windows: 101},
          coverage: {validated_velocity_min: -0.3, validated_velocity_max: 0.31}}}};
    const scored = await browserHarness(dashboardState(), quickResult);
    assert.match(scored.nodes.get("fit-quality").textContent, /独立验证 RMS \/ Nm0.012345/);
    assert.match(scored.nodes.get("fit-quality").textContent, /0.033333/);
    assert.doesNotMatch(scored.nodes.get("fit-quality").textContent, /9.876543/);
    assert.match(scored.nodes.get("fit-scope").textContent, /静摩擦、全速与热参数/);
    assert.match(scored.nodes.get("fit-quality").textContent, /负向共有滑动.*未记录可用共有范围/);
    assert.match(scored.nodes.get("fit-scope").textContent, /无法确认缺项速度的独立覆盖/);
    const coverageHistory = structuredClone(quickResult);
    coverageHistory.report.quick.coverage.directions = sharedBands;
    const coverageBefore = JSON.stringify(coverageHistory);
    const covered = await browserHarness(dashboardState(), coverageHistory);
    assert.match(covered.nodes.get("fit-quality").textContent, /负向共有滑动.*-0.512.*-0.287.*训练 25 \/ 独立验证 23 窗/);
    assert.match(covered.nodes.get("fit-quality").textContent, /正向共有滑动.*0.271.*0.524.*训练 30 \/ 独立验证 22 窗/);
    assert.doesNotMatch(covered.nodes.get("fit-quality").textContent, /验证速度域/);
    assert.equal(JSON.stringify(coverageHistory), coverageBefore);
    assert(covered.requests.every(request => request.method === "GET"));
    assert.doesNotMatch(scored.nodes.get("fit-scope").textContent, /低残差不代表参数准确/);
    const diagnosticResult = JSON.parse(JSON.stringify(quickResult));
    diagnosticResult.report.quick.parameter_identifiability = {schema: "quick_parameter_diagnostics_v1"};
    const diagnosticView = await browserHarness(dashboardState(), diagnosticResult);
    assert.match(diagnosticView.nodes.get("fit-scope").textContent, /惯量与各摩擦系数仍需独立验证；低残差不代表参数准确/);
    assert(!scored.nodes.get("step-validation").hidden);
    assert.match(scored.nodes.get("step-detail-validation").textContent, /4,?000 条样本/);
    scored.event("new-experiment", "click");
    assert.equal(scored.nodes.get("experiment-tools").open, false);
    assert.equal(scored.nodes.get("experiment").value, "calibrate");
    assert.match(scored.nodes.get("fit-quality").textContent, /0.012345/, "Preparing the next draft keeps the saved result intact");
    assert(scored.requests.every(r => r.method === "GET"));
    const sourceFit = {kind: "mechanical", model: "coulomb_viscous", parameters: {inertia: 0.003},
      units: {inertia: "kg*m^2"}, warnings: [], diagnostics: {samples: 61, validation_rmse: 0.006254,
        model_selection_rmse: 0.017, independent_validation: 1, external_validation_windows: 45}};
    const validationHistory = {id: "20261104T170000-12345678", trace: [], display_fields: [], artifacts: [], fit: sourceFit,
      report: {experiment: "validate", status: "completed", identity: {namespace: "/current_motor"},
        configuration: dashboardState().configuration, source_fit: sourceFit, events: [], stop_confirmed: true}};
    const sourceScored = await browserHarness(dashboardState(), validationHistory);
    assert.match(sourceScored.nodes.get("fit-quality").textContent, /独立验证窗口45/);
    assert.match(sourceScored.nodes.get("fit-quality").textContent, /独立验证 RMS \/ Nm0.006254/);
    assert.doesNotMatch(sourceScored.nodes.get("fit-quality").textContent, /留出|窗口61|0.017/);
    const missingWindows = JSON.parse(JSON.stringify(validationHistory));
    delete missingWindows.fit.diagnostics.external_validation_windows;
    const missingScored = await browserHarness(dashboardState(), missingWindows);
    assert.match(missingScored.nodes.get("fit-quality").textContent, /独立验证窗口—/);
    assert.doesNotMatch(missingScored.nodes.get("fit-quality").textContent, /窗口61/);
    const legacyHistory = JSON.parse(JSON.stringify(validationHistory));
    delete legacyHistory.fit.diagnostics.independent_validation;
    const legacyScored = await browserHarness(dashboardState(), legacyHistory);
    assert.match(legacyScored.nodes.get("fit-quality").textContent, /拟合窗口61/);
    assert.match(legacyScored.nodes.get("fit-quality").textContent, /留出 RMS \/ Nm0.006254/);
    assert.doesNotMatch(legacyScored.nodes.get("fit-quality").textContent, /独立验证窗口/);
    assert.match(markup, /<th>拟合估计<\/th>/);
    assert(sourceScored.requests.every(r => r.method === "GET"));
    const advanced = await browserHarness(dashboardState());
    advanced.nodes.get("experiment").value = "fourier"; advanced.event("experiment", "change");
    assert.match(advanced.nodes.get("protocol-mode").textContent, /跳过部分阶段/);
    assert.doesNotMatch(advanced.nodes.get("protocol-mode").textContent, /完整/);
    assert.match(view.reason("Speed limit reached: |velocity|=1.23321 rad/s >= 1 rad/s"), /速度保护.*1.23321/);
    {
    const automaticState = dashboardState("calibrate");
    automaticState.configuration.quick_excitation = "auto";
    const automatic = await browserHarness(automaticState, null, [], {groups: {}, fields: {}, quick_profile: canonical});
    assert.equal(automatic.nodes.get("quick_excitation").value, "auto", "The launch default seeds the first draft");
    assert.match(automatic.nodes.get("draft-plan").textContent, /2 × 10 秒预探测.*40 秒训练.*20 秒独立验证/);
    assert.doesNotMatch(automatic.nodes.get("draft-scope").textContent, /本机已验证方案/);
    assert(!automatic.nodes.get("step-pilot").hidden);
    assert(automatic.nodes.get("parameters-training").disabled);
    assert(automatic.requests.every(r => r.method === "GET"));
    automatic.nodes.get("experiment").value = "validate"; automatic.event("experiment", "change");
    assert(automatic.nodes.get("quick-excitation-choice").hidden);
    assert(automatic.nodes.get("step-pilot").hidden);
    assert.equal(automatic.nodes.get("quick_excitation").value, "auto");
    automatic.event("settings", "submit"); await automatic.flush();
    assert.equal(automatic.requests.find(r => r.url === "/api/start").body.settings.quick_excitation, "auto");
    for (const choice of ["auto", "fixed"]) {
      const state = dashboardState("compensate");
      state.calibration_configuration = {...state.configuration, quick_excitation: choice};
      const preference = await browserHarness(state);
      assert.equal(preference.nodes.get("quick_excitation").value, choice, "A draft after dragging uses calibration settings");
      for (const experiment of ["validate", "compensate", "breakaway", "precision", "fourier", "calibrate"]) {
        preference.nodes.get("experiment").value = experiment; preference.event("experiment", "change");
        assert.equal(preference.nodes.get("quick_excitation").value, choice);
      }
      assert(preference.requests.every(r => r.method === "GET"), "Changing task or mode does not request motion");
    }
    const oldState = dashboardState("calibrate"); delete oldState.configuration.quick_excitation;
    const oldQuick = await browserHarness(oldState);
    assert.equal(oldQuick.nodes.get("quick_excitation").value, "fixed");
    assert(oldQuick.nodes.get("quick-excitation-auto").disabled);
    oldQuick.event("settings", "submit"); await oldQuick.flush();
    assert(!Object.hasOwn(oldQuick.requests.find(r => r.url === "/api/start").body.settings, "quick_excitation"));
    const adaptiveHistory = JSON.parse(JSON.stringify(quickResult));
    adaptiveHistory.report.configuration.quick_excitation = "auto";
    adaptiveHistory.report.requested_configuration = {kp: .3, quick_excitation: "auto"};
    adaptiveHistory.report.samples = 12000; adaptiveHistory.report.recorded_samples = 14100;
    adaptiveHistory.report.quick_adaptation = {status: "selected", selected_candidate: "C1", attempts: [{candidate_id:"C1"}],
      candidates: {C1: {formal_settings: {amplitude:.45,frequency:.11,kp:4,kd:.5}}}};
    const selected = await browserHarness(dashboardState(), adaptiveHistory);
    assert.match(selected.nodes.get("quick-adaptation-summary").textContent, /自动选择 C1.*Kp 4.00 \/ Kd 0.50.*正式 12000 \/ 全部 14100/);
    assert(!selected.nodes.get("requested-configuration-details").hidden);
    assert.match(selected.nodes.get("requested-configuration").textContent, /kp0.3/);
    const restored = view.experimentState(adaptiveHistory);
    assert.equal(restored.quick_adaptation.selected_candidate, "C1");
    assert.equal(restored.requested_configuration.kp, .3);
    const legacy = view.experimentState(quickResult);
    assert.equal(legacy.configuration.quick_excitation, "fixed");
    assert.equal(legacy.quick_adaptation, null);
    for (const stage of ["quick_pilot", "quick_pilot_return"]) {
      const state = {...restored, phase:"running", stage};
      const steps = view.stageRows(state, state.configuration);
      assert.equal(steps.find(s=>s.id==="pilot").kind, "current");
      assert.notEqual(steps.find(s=>s.id==="training").kind, "current");
    }
    const aborted = {...restored, phase:"failed",stage:"failed",failure_stage:"quick_pilot",
      quick_adaptation:{status:"aborted",selected_candidate:null,attempts:[],abort_reasons:["guard limit"]}};
    assert.equal(view.stageRows(aborted, aborted.configuration).find(s=>s.id==="pilot").kind,"failed");
    assert.match(view.quickAdaptation(aborted).text,/尚未选定正式方案.*guard limit/);
    // Actual v2 archive shape: failure_stage retained "returning", while the
    // collector was still returning from C2 and had recorded no formal samples.
    const rejectedHistory = {id: "20261104T142031-9fa469a9", trace: [], display_fields: [], artifacts: [],
      report: {experiment: "calibrate", status: "failed", qualified: false, acquisition_complete: false,
        identity: {namespace: "/current_motor"}, configuration: {...canonical.settings, quick_excitation: "auto"},
        samples: 0, recorded_samples: 10933, pilot_samples: 10933, stop_confirmed: true,
        failure_stage: "returning", error: "Automatic quick excitation rejected: Both permitted pilots failed observed tracking/sliding/budget screening",
        quick_adaptation: {protocol: "quick_probe_ladder_v2_tracking_first", status: "rejected",
          selected_candidate: null, attempts: [{candidate_id: "C1"}, {candidate_id: "C2"}],
          collector: {stage: "pilot_return"}, abort_reasons: ["Both permitted pilots failed observed tracking/sliding/budget screening"]}}};
    const originalRejection = JSON.stringify(rejectedHistory);
    const rejectedView = await browserHarness(dashboardState(), rejectedHistory);
    assert.equal(rejectedView.nodes.get("current-step").textContent, "自动选择激励 · 预探测未通过");
    assert.equal(rejectedView.nodes.get("protocol-mode").textContent, "自动选择激励 · 尚无正式采集样本");
    assert.equal(rejectedView.nodes.get("step-pilot").className, "failed");
    for (const stage of ["training", "validation"]) {
      assert.equal(rejectedView.nodes.get(`step-state-${stage}`).textContent, "未执行");
      assert.match(rejectedView.nodes.get(`step-detail-${stage}`).textContent, /正式采集未开始/);
    }
    assert.equal(rejectedView.nodes.get("step-state-stopping").textContent, "已确认");
    assert.match(rejectedView.nodes.get("result-error").textContent, /Automatic quick excitation rejected/);
    assert.equal(JSON.stringify(rejectedHistory), originalRejection, "Display recovery must not rewrite archived failure evidence");
    assert(rejectedView.requests.every(r => r.method === "GET"));
    const failureState = view.experimentState(rejectedHistory);
    const rowFor = (state, id) => view.stageRows(state, state.configuration).find(row => row.id === id);
    assert.equal(rowFor({...failureState, failure_stage: "checking"}, "checking").kind, "failed");
    const pilotAborted = {...failureState, failure_stage: "running",
      quick_adaptation: {status: "aborted", selected_candidate: null, collector: {stage: "pilot"}}};
    assert.equal(rowFor(pilotAborted, "pilot").status, "预探测已中断");
    const noSampleFormal = {...failureState, failure_stage: "running",
      quick_adaptation: {status: "selected", selected_candidate: "C1", collector: {stage: "formal"}}};
    assert.equal(rowFor(noSampleFormal, "training").status, "未取得样本");
    assert.equal(rowFor(noSampleFormal, "pilot").kind, "done");
    const partialFormal = {...noSampleFormal, samples: 12, report: {...failureState.report, samples: 12}};
    assert.equal(rowFor(partialFormal, "training").status, "中断");
    assert.equal(rowFor({...partialFormal, failure_stage: "quick_validation"}, "validation").status, "中断");
    const fixedFailure = {...failureState, configuration: {...failureState.configuration, quick_excitation: "fixed"},
      quick_adaptation: null, report: {...failureState.report, quick_adaptation: null}};
    assert.equal(rowFor(fixedFailure, "training").status, "中断");
    }
    const lowArchive = {id: "low-speed-archive", trace: [], display_fields: [], artifacts: ["low_speed.json"],
      report: {experiment: "low_speed", status: "completed_unqualified", qualified: false,
        identity: {namespace: "/current_motor"}, configuration: {experiment: "low_speed"},
        samples: 1200, stop_confirmed: true,
        low_speed_progress: {phase: "done", completed_cells: 10, required_cells: 10, cell: 9},
        low_speed: {acquisition_quality_passed: false, quality_issues: ["Three independent repetitions required"],
          pairs: [{role: "training", speed: .05, repetition: 0, odd_resistance: .123,
            common_core_odd_resistance: .122, quality_passed: true}], cells: []}}};
    const lowBefore = JSON.stringify(lowArchive);
    const lowView = await browserHarness(dashboardState(), lowArchive);
    assert.equal(lowView.nodes.get("low-speed-panel").hidden, false);
    assert.equal(lowView.nodes.get("model-panel").hidden, true);
    assert.match(lowView.nodes.get("low-speed-pairs").textContent, /0\.123000/);
    assert.match(lowView.nodes.get("low-speed-summary").textContent, /未发布补偿模型/);
    assert.match(lowView.nodes.get("low-speed-issues").textContent, /Three independent repetitions/);
    assert.equal(view.experimentState(lowArchive).low_speed.completed_cells, 10);
    assert.equal(JSON.stringify(lowArchive), lowBefore, "Low-speed restoration preserves the recorded diagnostic");
    assert.equal(lowView.nodes.get("low-speed-estimator-comparison").hidden, true,
      "Legacy reports must not invent time-weighted values");
    assert(lowView.requests.every(r => r.method === "GET"));
    const comparedArchive = JSON.parse(lowBefore);
    comparedArchive.report.low_speed.pairs[0].time_odd_reported_torque = .11;
    comparedArchive.report.low_speed.pairs[0].angle_minus_time_odd_reported_torque = .013;
    const comparedBefore = JSON.stringify(comparedArchive);
    const comparedView = await browserHarness(dashboardState(), comparedArchive);
    assert.equal(comparedView.nodes.get("low-speed-estimator-comparison").hidden, false);
    assert.match(comparedView.nodes.get("low-speed-estimator-rows").textContent, /0\.110000/);
    assert.match(comparedView.nodes.get("low-speed-estimator-rows").textContent, /0\.013000/);
    assert.match(comparedView.nodes.get("low-speed-pairs").textContent, /0\.123000/);
    assert.equal(JSON.stringify(comparedArchive), comparedBefore);
    assert(comparedView.requests.every(r => r.method === "GET"));
    assert.deepEqual(errors, [], errors.map(error => error.stack).join("\n"));
    console.log("Dashboard controller and live/history isolation assertions passed");
  } finally { process.removeListener("unhandledRejection", capture); }
}
browserAssertions().catch(error => { console.error(error); process.exitCode = 1; });
