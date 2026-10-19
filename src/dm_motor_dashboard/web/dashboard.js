"use strict";

const fields = ["position", "velocity", "kp", "kd", "torque", "velocity_limit", "current_ratio"];
const modeFields = {1: ["position", "velocity", "kp", "kd", "torque"],
  2: ["position", "velocity_limit"], 3: ["velocity"],
  4: ["position", "velocity_limit", "current_ratio"]};
const modeNames = {1: "MIT", 2: "位置 + 速度上限", 3: "速度", 4: "位置 + 速度上限 + 电流比例"};
const activeFields = () => modeFields[data?.state?.mode || 1] || [];
const phases = ["DISABLED", "ENABLING", "ENABLED", "STOPPING", "FAULT"];
const $ = (id) => document.getElementById(id);
let session = "";
let connected = false;
let busy = false;
let data = null;
let polling = false;
let heartbeatPending = false;
const history = new Array(65536);
let historyStart = 0;
let historyCount = 0;
let latestSample = null;
let feedbackStream = null;
let receivedSamples = 0;
let renderedFrames = 0;
let skippedSamples = 0;
let statsStarted = performance.now();
let lastFrame = null;
let feedbackRate = 0;
let pollTimer = null;
let connectionVersion = 0;
let connectionAbort = new AbortController();
let initialConnectPending = true;
let capabilitiesKey = "";
let chartPaused = false;
let pendingConfirmation = null;
let positionFollowsCurrent = true;
let positionPointer = null;
let pendingTarget = null;
let positionSending = false;
let ratePending = false;
let positionUpdateEpoch = 0;
let positionLiveBlocked = false;
let targetSequence = 0;
const positionRadiansPerPixel = 0.001;
const float32PositionMaximum = 3.4028234663852886e38;
const parameterValues = new Map();
const taskStates = {idle: "无任务", submitting: "等待接受", running: "执行中", stopped: "已停止",
  succeeded: "成功", failed: "失败", rejected: "已拒绝", unknown: "结果未知"};
const taskBusy = () => ["submitting", "running"].includes(data?.task?.state) || Boolean(data?.state?.task_active);
const numeric = (value, digits = 4) => Number.isFinite(value) ? value.toFixed(digits) : "--";

function saveFile(name, content, type) {
  const url = URL.createObjectURL(new Blob([content], {type}));
  const link = document.createElement("a");
  link.href = url; link.download = name; link.click();
  queueMicrotask(() => URL.revokeObjectURL(url));
}

function finishConfirmation(accepted) {
  if (!pendingConfirmation) return;
  const resolve = pendingConfirmation;
  pendingConfirmation = null;
  $("confirmation").close();
  resolve(accepted);
}

function confirmAction(message) {
  if (pendingConfirmation) throw new ApiError(409, "已有操作正在等待确认。");
  $("confirmation-message").textContent = message;
  $("confirmation").showModal();
  return new Promise(resolve => { pendingConfirmation = resolve; });
}

class ApiError extends Error {
  constructor(status, message) { super(message); this.status = status; }
}

function headers() {
  return {"X-Control-Session": session, "Content-Type": "application/json"};
}

async function api(path, body) {
  const version = connectionVersion;
  const requestHeaders = headers();
  let generation, sequence;
  if (path === "target") {
    generation = data?.control_generation;
    if (!Number.isSafeInteger(generation)) throw new ApiError(409, "等待驱动控制代次，暂不能发送目标。");
    targetSequence = Math.max(targetSequence, data.target_sequence || 0) + 1;
    sequence = targetSequence;
    requestHeaders["X-Control-Generation"] = String(generation);
    requestHeaders["X-Target-Sequence"] = String(sequence);
  }
  const response = await fetch(`/api/${path}`, {
    method: body === undefined ? "GET" : "POST",
    headers: requestHeaders,
    ...(body === undefined ? {} : {body: JSON.stringify(body)}),
    signal: AbortSignal.any([connectionAbort.signal, AbortSignal.timeout(2000)]),
    cache: "no-store",
  });
  const result = await response.json();
  if (!response.ok) throw new ApiError(response.status, result.error || `HTTP ${response.status}`);
  if (path === "target" && version === connectionVersion && generation === data?.control_generation
      && sequence >= (data.target_sequence || 0)) {
    data.target = {...data.target, ...body};
    data.target_sequence = sequence;
  }
  return result;
}

function notice(message) {
  if ($("notice").textContent === message) return;
  $("notice").textContent = message;
  const entry = document.createElement("li");
  const timestamp = document.createElement("time");
  timestamp.dateTime = new Date().toISOString();
  timestamp.textContent = new Date().toLocaleTimeString();
  entry.append(timestamp, document.createTextNode(message));
  if (/error|failed|rejected|unknown|故障|失败|拒绝|未知|中断/i.test(message)) entry.className = "error";
  $("event-log").prepend(entry);
  while ($("event-log").children.length > 100) $("event-log").lastElementChild.remove();
}

function syncSlider(field) {
  if (field === "position") { syncPositionScroll(); return; }
  const input = $(`target-${field}`);
  const slider = $(`slider-${field}`);
  const value = Number(input.value);
  if (input.value.trim() && Number.isFinite(value)) {
    slider.value = String(Math.min(Number(input.max), Math.max(Number(input.min), value)));
  }
}

function currentPosition() {
  if (!connected || !data?.connected || !data.state?.feedback_fresh
      || data.state.absolute_position_valid === false) return NaN;
  return latestSample && latestSample.feedback_count >= data.state.feedback_count
    && performance.now() - latestSample.arrived < 500
    ? latestSample.position : data.state.position;
}

function positionBounds() {
  const input = $("target-position");
  return {minimum: Number(input.min), maximum: Number(input.max)};
}

function syncPositionScroll() {
  const input = $("target-position"), wheel = $("position-scroll");
  const entered = Number(input.value);
  if (!input.value.trim() || !Number.isFinite(entered)) return;
  const {minimum, maximum} = positionBounds();
  const value = Math.max(minimum, Math.min(maximum, entered));
  wheel.setAttribute("aria-valuenow", String(value));
  wheel.setAttribute("aria-valuetext", `目标 ${numeric(value, 6)} rad；连续旋转滚轮`);
  $("position-scroll-value").textContent = numeric(value, 6);
  $("position-wheel-turns").textContent = `${numeric(value / (2 * Math.PI), 4)} 圈`;
  $("position-wheel-rotor").style.transform = `rotate(${value % (2 * Math.PI)}rad)`;
}

function canEditPosition() {
  const value = Number($("target-position").value);
  return connected && Boolean(session) && data?.has_control && data.connected
    && data.state?.feedback_fresh && data.state.absolute_position_valid !== false
    && !busy && !taskBusy() && activeFields().includes("position")
    && Number($("mode-select").value) === data.state.mode
    && $("target-position").value.trim() && Number.isFinite(value);
}

function canSendPosition() {
  const value = Number($("target-position").value);
  const {minimum, maximum} = positionBounds();
  return canEditPosition() && data.armed && data.state.phase === 2 && data.state.ready
    && value >= minimum && value <= maximum
    && $("live-targets").checked
    && !positionLiveBlocked && !pendingConfirmation && Number.isSafeInteger(data.control_generation);
}

function canSendTarget() {
  return connected && session && data?.has_control && data.connected && data.armed
    && data.state?.phase === 2 && data.state.ready && data.state.feedback_fresh
    && data.state.absolute_position_valid !== false && !busy && !taskBusy()
    && $("live-targets").checked && !positionLiveBlocked && !pendingConfirmation
    && Number($("mode-select").value) === data.state.mode
    && Number.isSafeInteger(data.control_generation);
}

function cancelPositionUpdates() {
  endPositionGesture();
  pendingTarget = null;
  positionUpdateEpoch += 1;
}

function renderPositionScroll() {
  const enabled = Boolean(canEditPosition());
  if (!enabled && positionPointer) cancelPositionUpdates();
  $("position-scroll").setAttribute("aria-disabled", String(!enabled));
  $("position-scroll").tabIndex = enabled ? 0 : -1;
  const {minimum, maximum} = positionBounds();
  const boundary = value => Math.abs(value) >= 1e6 ? value.toExponential(3) : numeric(value, 4);
  $("position-scroll-min").textContent = `↶ 下限 ${boundary(minimum)} rad`;
  $("position-scroll-max").textContent = `上限 ${boundary(maximum)} rad ↷`;
  $("position-scroll").setAttribute("aria-valuemin", String(minimum));
  $("position-scroll").setAttribute("aria-valuemax", String(maximum));
  const value = Number($("target-position").value);
  let message = "未使能：滚动只修改位置草稿，不发送目标。";
  if (!connected || !data?.connected || !data.state?.feedback_fresh) message = "等待新鲜位置反馈。";
  else if (data.state.absolute_position_valid === false) message = "连续位置无效，禁止滚动控制。";
  else if (taskBusy()) message = "任务执行中，位置滚动控制已禁用。";
  else if (!data.has_control) message = "只读监控；获取控制权后才能调整位置。";
  else if (!Number.isFinite(value)) message = "位置草稿必须是有限数值。";
  else if (value < minimum || value > maximum) message = data.state.mode === 1
    ? "目标草稿超出 MIT 位置编码量程，滚轮停在边界，未发送；多圈位置控制请在失能后显式选择位置速度或力位模式。"
    : "目标草稿超出已配置行程限位或 float32 表示范围，滚轮停在边界，未发送。";
  else if (!$("live-targets").checked) message = "实时手动目标已关闭：只编辑草稿，需显式应用或提交任务。";
  else if (positionLiveBlocked && data.armed) message = "实时滚动已暂停；修正参数并显式应用目标后恢复。";
  else if (data.armed && !Number.isSafeInteger(data.control_generation)) message = "等待控制状态同步，暂不实时发送位置。";
  else if (canSendPosition()) {
    message = "参数变化立即提交 · 下一控制周期采用最新目标 · 松手保持，不周期重发。";
    if (data.state.mode === 1 && data.target.kp <= 0) message += " 当前已应用 Kp=0，不产生位置控制。";
    if ([2, 4].includes(data.state.mode) && data.target.velocity_limit <= 0) message += " 当前已应用速度上限为 0。";
  }
  if (enabled && value === minimum) message += " 已到下限，继续减小不再旋转；可反向调节。";
  if (enabled && value === maximum) message += " 已到上限，继续增大不再旋转；可反向调节。";
  $("position-scroll-status").textContent = message;
}

async function flushPositionTarget() {
  if (pendingTarget === null || positionSending) return;
  if (!canSendTarget()) { pendingTarget = null; return; }
  const target = pendingTarget;
  pendingTarget = null;
  const epoch = positionUpdateEpoch, version = connectionVersion;
  positionSending = true;
  try {
    await api("target", target);
  } catch (error) {
    if (epoch === positionUpdateEpoch && version === connectionVersion) {
      cancelPositionUpdates();
      positionLiveBlocked = true;
      if (!(error instanceof ApiError)) connectionLost(error);
      else notice(`位置更新被拒绝：${error.message}；已暂停实时滚动，请修正后显式应用目标。`);
    }
  } finally {
    positionSending = false;
    renderDraft();
    renderPositionScroll();
    if (pendingTarget !== null) void flushPositionTarget();
  }
}

function queuePositionTarget() {
  if (!canSendTarget()) return;
  try {
    pendingTarget = draftTarget();
  } catch (error) {
    if (!(error instanceof ApiError)) throw error;
    pendingTarget = null;
    notice(`${error.message} 未发送，电机保持最后接受的目标。`);
    return;
  }
  void flushPositionTarget();
}

function renderDraft() {
  const known = connected && data?.connected && data?.state?.feedback_fresh && data?.target;
  let changed = 0;
  for (const field of activeFields()) {
    const input = $(`target-${field}`);
    const differs = Boolean(known) && (!input.value.trim() || Number(input.value) !== data.target[field]);
    if (differs) changed += 1;
    $(`applied-${field}`).textContent = known ? `服务目标：${data.target[field]}` : "服务目标：--（未连接 / 过期）";
    $(`applied-${field}`).className = `applied${differs ? " edited" : ""}`;
  }
  $("draft-status").className = `subtle${changed ? " edited" : ""}`;
  $("draft-status").textContent = !known ? "等待新鲜状态；服务目标不是电机实测值。"
    : `${changed ? `${changed} 项草稿与服务目标不同，尚未应用。` : "草稿与服务目标一致。"}`
      + (data.armed && data.state?.phase === 2 ? " 服务目标不是实测反馈。" : " 当前未确认使能，不能发送目标。");
}

function resetInputs(followPosition = true) {
  cancelPositionUpdates();
  positionFollowsCurrent = followPosition;
  for (const field of fields) {
    $(`target-${field}`).value = "0";
    syncSlider(field);
  }
  const position = currentPosition();
  if (followPosition && Number.isFinite(position)) {
    $("target-position").value = String(position);
    syncPositionScroll();
  }
  renderDraft();
}

function relinquish() {
  positionLiveBlocked = true;
  finishConfirmation(false);
  if (feedbackStream) { feedbackStream.close(); feedbackStream = null; }
  $("plot-stats").textContent = "推流已断开 · 绘图暂停";
  if (session) {
    fetch("/api/release", {method: "POST", headers: headers(), body: "{}", keepalive: true})
      .then((response) => {
        if (!response.ok) console.warn("Release not confirmed; browser lease watchdog remains active.");
      })
      .catch((error) => console.warn("Release could not reach server; browser lease watchdog remains active.", error));
  }
  connectionVersion += 1;
  connectionAbort.abort();
  connectionAbort = new AbortController();
  session = "";
  connected = false;
  clearTimeout(pollTimer);
  resetInputs();
  render();
}

function connectionLost(error) {
  relinquish();
  notice(`连接中断：${error.message}。目标心跳已停止，不会自动恢复使能；请重新连接。`);
}

function render() {
  const state = data?.state;
  renderCapabilities();
  const pendingMode = Boolean(state && $("mode-select").value
    && Number($("mode-select").value) !== state.mode);
  const owns = connected && Boolean(session) && data?.has_control;
  $("connection").textContent = !connected ? "未连接" : owns ? "已连接 · 拥有控制权" : "已连接 · 只读";
  $("connection").className = `badge${connected ? " ok" : ""}`;
  $("connect").disabled = connected || busy;
  $("disconnect").disabled = !connected;
  $("claim").disabled = !connected || owns || busy;
  $("enable").disabled = !owns || busy || !data.connected || !state?.feedback_fresh
    || !state.ready || state.absolute_position_valid === false
    || !state.activity_available || state.phase !== 0;
  $("apply").disabled = !owns || busy || !data.connected || !data.armed || state?.phase !== 2;
  if (pendingMode) $("apply").disabled = true;
  if (taskBusy()) $("apply").disabled = true;
  $("zero").disabled = $("apply").disabled;
  $("stop").disabled = !connected;
  $("quick-stop").disabled = !connected;
  $("reset").disabled = !owns || busy || !data.connected || !state?.feedback_fresh
    || state.absolute_position_valid === false || state.phase !== 4 || state.status !== 0;
  const editable = owns && !busy && data.connected && state?.feedback_fresh
    && state.absolute_position_valid !== false;
  $("fill-position").disabled = !editable || pendingMode || !activeFields().includes("position");
  $("restore-target").disabled = !editable || pendingMode;
  $("live-targets").disabled = !owns || busy || taskBusy();
  $("control-rate").disabled = !owns || busy || ratePending || !state?.ready || taskBusy();
  if (document.activeElement !== $("control-rate") && !ratePending && Number.isFinite(state?.control_rate)) {
    $("control-rate").value = String(state.control_rate);
  }
  $("control-rate-status").textContent = `设定 ${numeric(state?.control_rate, 1)} Hz · `
    + `实测 ${numeric(state?.actual_control_rate, 1)} Hz（约 1 s 窗口） · 周期 ${numeric(state?.period_ms, 3)} ms`;
  for (const field of fields) {
    const active = activeFields().includes(field);
    $(`target-${field}`).parentElement.hidden = false;
    $(`target-${field}`).disabled = !editable || !active || pendingMode;
    $(`target-${field}`).title = active ? "" : "当前驱动模式不使用此参数。";
    if (field !== "position") $(`slider-${field}`).disabled = !editable || !active || pendingMode;
  }
  const disabledReady = editable && state?.phase === 0 && state.status === 0 && state.ready
    && state.maintenance_available && !state.task_active;
  for (const id of ["mode-select", "device-zero", "software-zero", "save-parameters"]) {
    $(id).disabled = !disabledReady;
  }
  $("clear-fault").disabled = !(disabledReady || (editable && state?.ready && !state.task_active
    && state.phase === 4 && state.status !== 1));
  $("parameter-select").disabled = !connected || !data?.capabilities?.ready;
  $("read-parameter").disabled = $("parameter-select").disabled || !state?.maintenance_available || busy;
  $("read-all-parameters").disabled = $("read-parameter").disabled;
  $("export-state").disabled = !state;
  $("export-csv").disabled = !historyCount;
  $("export-parameters").disabled = !parameterValues.size;
  if (pendingMode) $("enable").disabled = true;
  const fieldNames = {position: "位置/滚轮", velocity: "目标速度", kp: "Kp", kd: "Kd",
    torque: "前馈力矩", velocity_limit: "速度上限", current_ratio: "电流比例"};
  $("mode-status").textContent = state
    ? `实际模式：${modeNames[state.mode] || "未知"}；使用：${activeFields().map(field => fieldNames[field]).join("、")}。`
      + (pendingMode ? ` 正在切换到 ${modeNames[Number($("mode-select").value)] || "未知"}；等待驱动确认，暂停目标编辑和使能。`
        : "不适用的参数已禁用，切换模式不会自动使能。")
    : "等待驱动确认模式。";
  renderParameter(disabledReady);
  renderTask(owns);
  if (!state) { renderDraft(); renderPositionScroll(); return; }
  $("current-mode").textContent = `模式 ${state.mode || 1} · ${modeNames[state.mode || 1] || "未知"}`;
  $("driver-targets").textContent = JSON.stringify(state, null, 2);
  $("motor-id").textContent = `#${data.motor_id}`;
  updateReadings(connected && latestSample ? latestSample : state);
  $("temperature").textContent = `${state.mos_temperature} / ${state.rotor_temperature}`;
  renderMonitoring(state, owns);
  const stale = !connected || !data.connected || !state.feedback_fresh;
  $("phase").textContent = `${state.simulated ? "SIMULATION" : "HARDWARE"} · ${stale ? "状态过期 / 未确认" : phases[state.phase] || "UNKNOWN"}`;
  $("phase").className = `badge${stale || state.phase === 4 ? " error" : state.phase === 2 ? " ok" : ""}`;
  $("phase").title = `设备状态: ${state.status_text ?? state.status}; 主机故障: ${Boolean(state.host_fault)}; `
    + `电机故障: ${Boolean(state.device_fault)}; 主机额外物理保护: ${Boolean(state.host_protection_enabled)}`;
  $("telemetry").textContent = `${stale ? "上次反馈 · " : ""}CAN ${state.motor_id} / MST ${state.master_id} · 状态码 0x${state.status.toString(16).toUpperCase()} · `
    + (state.absolute_position_valid === undefined ? "" :
      `连续位置${state.absolute_position_valid ? "有效" : "无效"} · 回绕 ${state.position_wraps} 次 · `)
    + `${state.status_text ?? "设备状态待确认"} · ${state.device_fault ? "设备保护" : state.host_fault ? "主机通信/命令故障" : "无锁定故障"} · `
    + `主机额外物理保护 ${state.host_protection_enabled ? "已显式开启" : "关闭（固件保护）"} · `
    + `反馈 ${feedbackRate.toFixed(1)} 帧/s / ${state.feedback_count} 帧 · age ${(state.feedback_age * 1000).toFixed(1)} ms · `
    + `控制周期 ${state.period_ms.toFixed(3)} ms / 最大 ${state.max_period_ms.toFixed(3)} ms · 超周期 ${state.deadline_misses} 次`
    + (data.connected && state.feedback_fresh ? "" : " · 反馈过期 / 控制器断开");
  for (const field of fields) {
    let maximum = field === "current_ratio" ? 1
      : field === "velocity_limit" ? state.velocity_limit : state[`${field}_limit`];
    let minimum = ["kp", "kd", "velocity_limit", "current_ratio"].includes(field) ? 0 : -maximum;
    if (field === "position") {
      const offset = state.software_offset ?? 0;
      minimum = Math.max(offset - maximum, state.position_min ?? offset - maximum);
      maximum = Math.min(offset + maximum, state.position_max ?? offset + maximum);
    }
    if (field === "velocity_limit" && state.mode === 4) maximum = Math.min(maximum, 100);
    const input = $(`target-${field}`);
    const limitsChanged = input.min !== String(minimum) || input.max !== String(maximum);
    input.min = String(minimum);
    input.max = String(maximum);
    if (field === "position") {
      const position = currentPosition();
      if (positionFollowsCurrent && Number.isFinite(position)) input.value = String(position);
      if (limitsChanged || positionFollowsCurrent) syncPositionScroll();
    } else {
      const slider = $(`slider-${field}`);
      slider.min = input.min;
      slider.max = input.max;
      syncSlider(field);
    }
    $(`range-${field}`).textContent = !activeFields().includes(field) ? "当前驱动模式不使用此参数"
      : field === "position" &&
      minimum <= -float32PositionMaximum && maximum >= float32PositionMaximum
      ? "连续多圈目标 · 未设置机械行程限位 · 数值仍受 float32 表示范围限制"
      : `${minimum} ～ ${maximum}${field === "position" ? " rad · 驱动允许下发的范围，不是滚动条端点" : ""}`;
  }
  renderPositionScroll();
  renderDraft();
}

function renderData(id, entries) {
  const nodes = [];
  for (const [key, value] of entries) {
    const term = document.createElement("dt"), definition = document.createElement("dd");
    term.textContent = key; definition.textContent = String(value ?? "--");
    nodes.push(term, definition);
  }
  $(id).replaceChildren(...nodes);
}

function renderMonitoring(state, owns) {
  $("position-turns").textContent = numeric(state.position / (2 * Math.PI), 6);
  $("position-degrees").textContent = `${numeric(state.position * 180 / Math.PI, 3)}°`;
  $("compact-position").textContent = numeric(state.native_position, 6);
  $("absolute-valid").textContent = state.absolute_position_valid ? "连续位置有效" : "连续位置无效";
  $("absolute-valid").className = `badge ${state.absolute_position_valid ? "ok" : "error"}`;
  $("control-owner").textContent = `${owns ? "本页面持有控制权" : "只读监控"} · 租约 ${numeric(state.lease_remaining, 2)} s`;
  $("state-age").textContent = `反馈年龄 ${numeric(state.feedback_age * 1000, 1)} ms · ${state.feedback_count} 帧`;
  const disconnected = !connected || !data.connected || !state.feedback_fresh;
  const problem = state.error || (state.phase === 4 ? "驱动处于锁定故障，请检查设备状态并执行相应恢复操作。" : "")
    || (!state.absolute_position_valid ? "连续位置基准无效，禁止使能；检查诊断。" : "");
  $("health-banner").className = `health-banner ${disconnected ? "warning" : problem ? "error" : state.phase === 2 ? "warning" : "ok"}`;
  $("health-banner").textContent = disconnected ? "状态未连接或已过期：下方数值不可作为当前电机状态，禁止发送运动目标。"
    : problem || (state.phase === 2 ? "电机已使能：不要触碰转轴。STOP / Esc 请求失能，软件停止不能替代硬件急停。"
      : `${state.simulated ? "仿真设备" : "真实硬件"} · 已连接，电机未使能；需要显式使能后才能运动。`);
  renderData("position-data", [
    ["连续关节位置 / rad", numeric(state.position, 6)], ["原生连续位置 / rad", numeric(state.absolute_position, 6)],
    ["紧凑反馈 / rad", numeric(state.native_position, 6)], ["软件回绕次数", state.position_wraps],
    ["方向 / 软件偏移", `${state.direction ?? "--"} / ${numeric(state.software_offset, 6)}`],
    ["关节范围 / rad", state.position_min <= -float32PositionMaximum && state.position_max >= float32PositionMaximum
      ? "未设置机械行程限位" : `${numeric(state.position_min)} ～ ${numeric(state.position_max)}`],
    ["PMAX / VMAX / TMAX", `${numeric(state.pmax, 2)} / ${numeric(state.vmax, 2)} / ${numeric(state.tmax, 2)}`],
  ]);
  renderData("health-data", [
    ["设备 / 模式", `${state.simulated ? "SIM" : "HARDWARE"} / ${modeNames[state.mode] ?? state.mode}`],
    ["驱动阶段", phases[state.phase] ?? "未知"], ["设备状态", state.status_text ?? state.status],
    ["主机故障 / 设备故障", `${Boolean(state.host_fault)} / ${Boolean(state.device_fault)}`],
    ["控制者", state.control_owner || "无"], ["任务 / 目标活动", `${Boolean(state.task_active)} / ${Boolean(state.target_active)}`],
    ["主机额外保护", state.host_protection_enabled ? "已开启" : "关闭，固件保护"],
    ["故障组", state.fault_group || "独立轴"],
    ["Kp / Kd 上限", `${state.kp_limit} / ${state.kd_limit}`],
  ]);
  renderData("timing-data", [
    ["反馈速率 / Hz", numeric(state.feedback_rate, 1)], ["反馈年龄 / ms", numeric(state.feedback_age * 1000, 2)],
    ["当前 / 最大周期 ms", `${numeric(state.period_ms, 3)} / ${numeric(state.max_period_ms, 3)}`],
    ["周期 P50 / P95 / P99 ms", [state.period_p50_ms, state.period_p95_ms, state.period_p99_ms].map(v => numeric(v, 3)).join(" / ")],
    ["超周期 / 拒绝命令", `${state.deadline_misses} / ${state.rejected_commands ?? "--"}`],
    ["目标接收 / 应用序号", `${state.accepted_target_sequence ?? "--"} / ${state.applied_target_sequence ?? "--"}`],
    ["目标到控制周期接收延迟 / ms", state.applied_target_sequence ? numeric(state.target_accept_latency_ms, 3) : "--"],
    ["活动 / 维护通道", `${Boolean(state.activity_available)} / ${Boolean(state.maintenance_available)}`],
    ["CAN ID / MST ID", `${state.motor_id} / ${state.master_id}`],
  ]);
  const rows = ["position", "velocity", "torque", "kp", "kd", "velocity_limit", "current_ratio"].map(field => {
    const row = document.createElement("tr");
    for (const value of [field, numeric(state.requested_target?.[field]),
      state.target_active ? numeric(state.active_target?.[field]) : "未活动",
      ["position", "velocity", "torque"].includes(field) ? numeric(state[field]) : "—"]) {
      const cell = document.createElement("td"); cell.textContent = value; row.append(cell);
    }
    return row;
  });
  $("target-comparison").replaceChildren(...rows);
}

function renderTask(owns) {
  const state = data?.state, kind = $("task-kind").value;
  const hints = {
    move: "定位：位置草稿为终点，MIT 需 Kp>0；速度草稿为 0，轨迹速度由驱动生成。请为位移留足时间。",
    ramp: "位置过渡：与平滑定位相同，遵守速度、加速度和稳定时间约束；不是无限连续旋转。",
    hold: "保持：位置草稿须接近当前连续关节位置，MIT 需 Kp>0。不要用保持任务替代大位移定位。",
    velocity: "速度：仅支持 MIT/速度模式。MIT 需 Kp=0、Kd>0、前馈力矩=0；斜坡时间限制启动加速度。",
    torque: "力矩：仅支持 MIT。Kp、Kd、速度均须为 0，前馈力矩草稿为目标；必须提供非零斜坡时间。",
    sine: "正弦：位置草稿须接近当前位置；驱动按幅值、频率生成位置和速度，校验整个行程。",
    trajectory: "轨迹：时间从 0 严格递增，第一点为当前连续关节位置；相邻点使用静止到静止的五次插值。",
  };
  $("task-hint").textContent = hints[kind];
  const supported = name => state?.mode === 1 || (name === "velocity" ? state?.mode === 3
    : name !== "torque" && state?.mode !== 3);
  for (const option of $("task-kind").options) option.disabled = !supported(option.value);
  for (const label of document.querySelectorAll("[data-task-kinds]")) {
    label.hidden = !label.dataset.taskKinds.split(" ").includes(kind);
  }
  $("start-task").disabled = !owns || busy || !data?.armed || state?.phase !== 2
    || !state.feedback_fresh || !state.ready || taskBusy() || !supported(kind);
  $("cancel-task").disabled = !connected || !taskBusy();
  const task = data?.task;
  $("task-state").textContent = state?.task_active && !["submitting", "running"].includes(task?.state)
    ? "驱动任务活动（非本页面）" : taskStates[task?.state] ?? "无任务";
  $("task-state").className = `badge${task?.state === "succeeded" ? " ok" : ["failed", "unknown", "rejected"].includes(task?.state) ? " error" : ""}`;
  const progress = task?.feedback?.progress;
  $("task-progress").value = Number.isFinite(progress) ? Math.min(1, Math.max(0, progress)) : 0;
  $("task-feedback").textContent = task ? `${task.message} `
    + (task.feedback ? `进度 ${numeric(progress * 100, 1)}% · ${numeric(task.feedback.elapsed, 2)} s · 位置误差 ${numeric(task.feedback.position_error, 6)} rad` : "")
    + (task.result ? ` · 最终位置 ${numeric(task.result.final_position, 6)} rad` : "") : "尚未提交任务。";
}

function selectedParameter() {
  return data?.capabilities?.registers?.find(item => item.address === Number($("parameter-select").value));
}

function renderParameter(disabledReady) {
  const descriptor = selectedParameter();
  const allowed = Boolean(descriptor?.raw_write_allowed);
  $("write-parameter").disabled = !disabledReady || !allowed || busy;
  $("parameter-input").disabled = $("write-parameter").disabled;
  if (!descriptor) return;
  $("parameter-info").textContent = `0x${descriptor.address.toString(16)} · ${descriptor.integer ? "uint32" : "float32"} · ${descriptor.unit || "无单位"} · `
    + (allowed ? "允许原始写入；需要失能和控制权，写后独立回读，不自动保存。" : descriptor.writable ? "受保护参数：使用专用操作，不允许原始写入。" : "只读参数。")
    + (descriptor.has_minimum ? ` 下界 ${descriptor.minimum_inclusive ? "≥" : ">"} ${descriptor.minimum}。` : "")
    + (descriptor.has_maximum ? ` 上界 ${descriptor.maximum_inclusive ? "≤" : "<"} ${descriptor.maximum}。` : "");
}

function renderCapabilities() {
  const caps = data?.capabilities;
  if (!caps) return;
  $("capabilities").textContent = `${caps.ready ? "驱动就绪" : "驱动未就绪"} · ${caps.name} · ${caps.profile} / ${caps.backend}`
    + ` · 控制者 ${data.state?.control_owner || "无"} · 租约 ${(data.state?.lease_remaining || 0).toFixed(2)} s`
    + ` · ${caps.message || caps.evidence || ""}`;
  const key = JSON.stringify([caps.modes, caps.registers]);
  if (key === capabilitiesKey) return;
  capabilitiesKey = key;
  $("mode-select").replaceChildren(...caps.modes.map((mode) => {
    const option = document.createElement("option");
    option.value = String(mode);
    option.textContent = `${mode}: ${modeNames[mode] || "未知"}`;
    option.selected = mode === data.state?.mode;
    return option;
  }));
  $("parameter-select").replaceChildren(...caps.registers.map((parameter) => {
    const option = document.createElement("option");
    option.value = String(parameter.address);
    option.textContent = `0x${parameter.address.toString(16)} ${parameter.name} (${parameter.integer ? "uint32" : "float32"} ${parameter.unit}) · ${parameter.raw_write_allowed ? "可写" : parameter.writable ? "受保护" : "只读"}`;
    return option;
  }));
}

async function confirmOperation(id, output = "operation-status") {
  if (!id) return null;
  const version = connectionVersion;
  for (let attempt = 0; attempt < 50; attempt += 1) {
    if (!connected || version !== connectionVersion) throw new ApiError(409, "连接已改变，操作结果未确认。");
    const result = await api(`operations/${id}`);
    $(output).textContent = `操作 #${id}: ${["等待确认", "已确认", "已拒绝", "未知"][result.state] || "无效状态"} · ${result.message}`;
    if (result.state === 1) return result;
    if (result.state !== 0) throw new ApiError(409, $(output).textContent);
    await new Promise((resolve) => setTimeout(resolve, 100));
  }
  throw new ApiError(504, `操作 #${id} 确认超时，结果未知。不会自动重试写操作。`);
}

function updateReadings(sample) {
  for (const field of ["position", "velocity", "torque"]) {
    const value = sample[field].toFixed(4);
    if ($(field).textContent !== value) $(field).textContent = value;
  }
}

function addSample(sample) {
  if (latestSample && sample.sequence > latestSample.sequence + 1) {
    skippedSamples += sample.sequence - latestSample.sequence - 1;
  }
  sample.arrived = performance.now();
  latestSample = sample;
  if (historyCount === history.length) {
    historyStart = (historyStart + 1) % history.length;
    historyCount -= 1;
  }
  history[(historyStart + historyCount) % history.length] = sample;
  historyCount += 1;
  while (historyCount && sample.time - history[historyStart].time > 60) {
    historyStart = (historyStart + 1) % history.length;
    historyCount -= 1;
  }
  receivedSamples += 1;
}

function startStream() {
  const source = new EventSource("/api/stream");
  feedbackStream = source;
  $("plot-stats").textContent = "正在连接实时反馈流";
  source.onmessage = (event) => {
    if (source !== feedbackStream || !connected) return;
    try {
      const sample = JSON.parse(event.data);
      if (!Number.isSafeInteger(sample.sequence) || sample.sequence < 1 ||
          !["time", "position", "velocity", "torque"].every((field) => Number.isFinite(sample[field]))) {
        throw new Error("反馈推流数据格式无效");
      }
      addSample(sample);
    } catch (error) { connectionLost(error); }
  };
  source.onerror = () => {
    if (source === feedbackStream && connected) {
      connectionLost(new Error("反馈推流中断，控制会话已停止"));
    }
  };
  source.addEventListener("gap", event => {
    if (source !== feedbackStream || !connected) return;
    try {
      const gap = JSON.parse(event.data);
      if (!Number.isSafeInteger(gap.missing) || gap.missing < 1) throw new Error("反馈缺失通知格式无效");
      notice(`反馈连接积压超出缓存，缺失 ${gap.missing} 个采样点；图表不插值补造数据。`);
    } catch (error) { connectionLost(error); }
  });
}

function draw(frameTime = performance.now()) {
  const canvas = $("chart");
  const rect = canvas.getBoundingClientRect();
  const ratio = devicePixelRatio || 1;
  const width = Math.round(rect.width * ratio);
  const height = Math.round(rect.height * ratio);
  if (canvas.width !== width || canvas.height !== height) {
    canvas.width = width;
    canvas.height = height;
  }
  const ctx = canvas.getContext("2d");
  ctx.setTransform(ratio, 0, 0, ratio, 0, 0);
  ctx.clearRect(0, 0, rect.width, rect.height);
  const now = latestSample ? latestSample.time + Math.max(0, frameTime - latestSample.arrived) / 1000 : 0;
  const windowSeconds = Number($("chart-window").value);
  while (historyCount && now - history[historyStart].time > 60) {
    historyStart = (historyStart + 1) % history.length;
    historyCount -= 1;
  }
  const colors = ["#63b8ff", "#7ade98", "#ffcf70"];
  const labels = ["位置 / rad", "速度 / rad/s", "力矩 / Nm"];
  ["position", "velocity", "torque"].forEach((field, index) => {
    const height = rect.height / 3;
    const top = index * height;
    const sampleField = field === "position" ? $("chart-position").value : field;
    const showTarget = $("chart-targets").checked && (field !== "position" || sampleField === "position");
    const targetField = `target_${field}`;
    let low = Infinity, high = -Infinity;
    for (let i = 0; i < historyCount; i += 1) {
      const point = history[(historyStart + i) % history.length];
      if (now - point.time > windowSeconds) continue;
      for (const value of [point[sampleField], ...(showTarget ? [point[targetField]] : [])]) {
        if (Number.isFinite(value)) { low = Math.min(low, value); high = Math.max(high, value); }
      }
    }
    if (!Number.isFinite(low)) { low = -0.01; high = 0.01; }
    const padding = Math.max((high - low) * 0.1, 0.005);
    low -= padding; high += padding;
    const y = value => top + 24 + (high - value) / (high - low) * Math.max(1, height - 38);
    ctx.strokeStyle = "#28384e";
    ctx.beginPath();
    ctx.moveTo(0, top + height / 2);
    ctx.lineTo(rect.width, top + height / 2);
    ctx.stroke();
    ctx.fillStyle = colors[index];
    ctx.font = "12px system-ui";
    ctx.fillText(`${labels[index]}${field === "position" ? ` · ${sampleField}` : ""}   ${low.toFixed(3)} ～ ${high.toFixed(3)}`, 4, top + 14);
    ctx.strokeStyle = colors[index];
    ctx.lineWidth = 1.8;
    ctx.beginPath();
    let started = false, previous = null;
    for (let i = 0; i < historyCount; i += 1) {
      const point = history[(historyStart + i) % history.length];
      if (now - point.time > windowSeconds || !Number.isFinite(point[sampleField])) continue;
      const x = rect.width * (1 - (now - point.time) / windowSeconds);
      if (previous && (point.sequence !== previous.sequence + 1 ||
        point.feedback_count !== previous.feedback_count + 1)) started = false;
      if (started) ctx.lineTo(x, y(point[sampleField]));
      else { ctx.moveTo(x, y(point[sampleField])); started = true; }
      previous = point;
    }
    ctx.stroke();
    if (showTarget) {
      ctx.setLineDash([5, 4]);
      ctx.strokeStyle = "#d3dce9";
      ctx.beginPath();
      let started = false;
      for (let i = 0; i < historyCount; i += 1) {
        const point = history[(historyStart + i) % history.length];
        if (now - point.time > windowSeconds) continue;
        if (!Number.isFinite(point[targetField])) { started = false; continue; }
        const x = rect.width * (1 - (now - point.time) / windowSeconds);
        if (started) ctx.lineTo(x, y(point[targetField]));
        else { ctx.moveTo(x, y(point[targetField])); started = true; }
      }
      ctx.stroke();
      ctx.setLineDash([]);
    }
  });
}

function animate(time) {
  if (connected && latestSample && feedbackStream && !document.hidden) {
    if (!chartPaused) { draw(time); renderedFrames += 1; }
    updateReadings(latestSample);
    const elapsed = time - statsStarted;
    if (elapsed >= 1000) {
      $("plot-stats").textContent = `推流 ${(receivedSamples * 1000 / elapsed).toFixed(1)} Hz · `
        + `${chartPaused ? "绘图已暂停，采集继续" : `绘图 ${(renderedFrames * 1000 / elapsed).toFixed(1)} / 50 FPS`} · `
        + `推流缺失 ${skippedSamples} 点 · ROS 缺失 ${latestSample.source_skipped_samples || 0} 点 · 缓存 ${historyCount} 帧`;
      $("export-csv").disabled = !historyCount;
      receivedSamples = 0;
      renderedFrames = 0;
      statsStarted = time;
    }
  }
}

async function poll() {
  if (!connected || polling) return;
  polling = true;
  const version = connectionVersion;
  try {
    const snapshot = await api("state");
    if (version !== connectionVersion) return;
    if (Number.isSafeInteger(data?.control_generation) && Number.isSafeInteger(snapshot.control_generation)) {
      if (snapshot.control_generation < data.control_generation) return;
      if (snapshot.control_generation === data.control_generation &&
          snapshot.target_sequence < data.target_sequence) {
        snapshot.target = data.target;
        snapshot.target_sequence = data.target_sequence;
      }
    }
    const previousError = data?.state?.error;
    const previouslyArmed = data?.armed;
    const previouslyOwned = data?.has_control;
    const previousMode = data?.state?.mode;
    const previousNotice = data?.notice;
    data = snapshot;
    if ((previouslyArmed && !data.armed) || (previouslyOwned && !data.has_control)
        || (previousMode !== undefined && previousMode !== data.state?.mode)) {
      resetInputs(!(previouslyOwned && !data.has_control));
    }
    if (previousMode !== undefined && previousMode !== data.state?.mode) $("mode-select").value = String(data.state.mode);
    if (session && !data.has_control) {
      finishConfirmation(false);
      session = "";
      notice("控制会话已结束，目标已清空。需重新获取控制权并显式使能。");
    }
    if (data.state) {
      const time = performance.now();
      const sampleTime = data.state_time * 1000;
      if (lastFrame && sampleTime - lastFrame.time >= 500) {
        feedbackRate = Math.max(0, (data.state.feedback_count - lastFrame.count) * 1000 / (sampleTime - lastFrame.time));
        lastFrame = {time: sampleTime, count: data.state.feedback_count};
      } else if (!lastFrame) lastFrame = {time: sampleTime, count: data.state.feedback_count};
      if (data.state.error) notice(data.state.error);
      else if (previousError) notice("故障已清除，电机保持失能；需要重新显式使能。");
      else if (data.notice && previousNotice !== data.notice) notice(data.notice);
      if (!data.connected) notice("控制器反馈已断开；不能使能或应用目标。");
    }
    render();
  } catch (error) {
    if (version === connectionVersion) connectionLost(error);
  } finally {
    polling = false;
    if (connected) pollTimer = setTimeout(poll, 100);
  }
}

async function action(callback) {
  cancelPositionUpdates();
  busy = true;
  render();
  try { await callback(); }
  catch (error) {
    if (connected && !(error instanceof ApiError)) connectionLost(error);
    else notice(error.message);
  }
  finally { busy = false; render(); }
}

async function claim() {
  const result = await api("session", {});
  session = result.session;
  data.has_control = true;
  data.armed = false;
  positionLiveBlocked = true;
  positionFollowsCurrent = true;
  notice(result.message);
}

async function connect() {
  initialConnectPending = false;
  data = await api("state");
  if (document.hidden) {
    notice("页面已隐藏，未获取控制权；返回后请点击连接。");
    return;
  }
  connected = true;
  historyStart = 0;
  historyCount = 0;
  latestSample = null;
  receivedSamples = 0;
  renderedFrames = 0;
  skippedSamples = 0;
  statsStarted = performance.now();
  lastFrame = null;
  try { await claim(); }
  catch (error) {
    if (error.status !== 409) { connected = false; throw error; }
    notice(error.message);
  }
  if (document.hidden) {
    relinquish();
    notice("页面已隐藏，控制权已释放；返回后请重新连接。");
    return;
  }
  startStream();
  await poll();
}

function initialConnect() {
  if (initialConnectPending && !document.hidden) action(connect);
}

$("connect").addEventListener("click", () => action(connect));
$("claim").addEventListener("click", () => action(claim));
$("disconnect").addEventListener("click", () => { relinquish(); notice("已断开。失能请求已提交，最终结果以电机反馈为准。"); });
$("enable").addEventListener("click", () => action(async () => {
  resetInputs();
  const epoch = positionUpdateEpoch;
  const result = await api("enable", {});
  if (epoch === positionUpdateEpoch) positionLiveBlocked = false;
  notice(result.message);
  await confirmOperation(result.operation_id);
}));
$("stop").addEventListener("click", async () => {
  positionLiveBlocked = true;
  finishConfirmation(false);
  resetInputs();
  try { notice((await api("stop", {})).message); }
  catch (error) { notice(`失能未确认：${error.message}。必要时使用硬件断电。`); }
});
$("reset").addEventListener("click", () => action(async () => {
  const result = await api("reset", {});
  notice(result.message);
  await confirmOperation(result.operation_id);
}));
for (const [id, operation] of [["clear-fault", "clear"],
  ["device-zero", "device_zero"], ["software-zero", "software_zero"], ["save-parameters", "save"]]) {
  $(id).addEventListener("click", () => action(async () => {
    if (!await confirmAction(`确认${$(id).textContent}？电机必须保持失能，不会自动使能。`)) return;
    const result = await api("operation", {operation});
    notice(result.message);
    await confirmOperation(result.operation_id);
  }));
}
$("mode-select").addEventListener("change", () => {
  const selector = $("mode-select");
  if (selector.disabled || !data?.state || Number(selector.value) === data.state.mode) return;
  const selectedMode = Number(selector.value);
  void action(async () => {
    try {
      const result = await api("operation", {operation: "mode", mode: selectedMode});
      notice(result.message);
      await confirmOperation(result.operation_id, "mode-operation-status");
      await poll();
    } catch (error) {
      if (data?.state) selector.value = String(data.state.mode);
      throw error;
    }
  });
});
async function displayParameter(descriptor, response) {
  const address = descriptor.address;
  for (const id of response.operation_ids) {
    const result = await confirmOperation(id, "parameter-value");
    if (result.address !== address) throw new ApiError(409, "参数地址与请求不符。");
    const bytes = new DataView(new ArrayBuffer(4));
    bytes.setUint32(0, result.raw_value, true);
    const value = descriptor.integer ? result.raw_value : bytes.getFloat32(0, true);
    if (!Number.isFinite(value)) throw new ApiError(409, "设备返回非有限参数值，未保存结果。");
    const record = {address, name: descriptor.name, value, unit: descriptor.unit,
      raw_value: result.raw_value, confirmed_at: new Date().toISOString()};
    parameterValues.set(address, record);
    $("parameter-value").textContent = `${descriptor.name}: ${value} ${descriptor.unit} · raw 0x${result.raw_value.toString(16)} · 已确认 #${id}`;
    const rows = [...parameterValues.values()].sort((a, b) => a.address - b.address).map(record => {
      const row = document.createElement("tr");
      for (const value of [`0x${record.address.toString(16)}`, record.name, String(record.value),
        record.unit, new Date(record.confirmed_at).toLocaleTimeString()]) {
        const cell = document.createElement("td"); cell.textContent = value; row.append(cell);
      }
      return row;
    });
    $("parameter-results").replaceChildren(...rows);
  }
}

async function readParameter(descriptor) {
  if (!descriptor) throw new ApiError(400, "请选择已公布的参数。");
  await displayParameter(descriptor, await api("registers", {addresses: [descriptor.address]}));
}

$("read-parameter").addEventListener("click", () => action(() => readParameter(selectedParameter())));
$("read-all-parameters").addEventListener("click", () => action(async () => {
  const version = connectionVersion;
  for (const descriptor of data.capabilities.registers) {
    if (!connected || version !== connectionVersion) throw new ApiError(409, "连接改变，已停止参数读取。");
    await readParameter(descriptor);
  }
  notice(`已确认读取 ${data.capabilities.registers.length} 项参数；结果带时间戳，不是持续实时值。`);
}));
$("parameter-select").addEventListener("change", () => render());
$("write-parameter").addEventListener("click", () => action(async () => {
  const descriptor = selectedParameter(), input = $("parameter-input");
  if (!descriptor?.raw_write_allowed || !input.value.trim()) throw new ApiError(400, "请选择可写参数并填写数值。");
  const value = Number(input.value);
  if (!Number.isFinite(value) || (descriptor.integer && (!Number.isInteger(value) || value < 0 || value > 0xFFFFFFFF))) {
    throw new ApiError(400, "参数类型或数值无效。");
  }
  for (const side of ["minimum", "maximum"]) {
    if (!descriptor[`has_${side}`]) continue;
    const boundary = descriptor[side], inclusive = descriptor[`${side}_inclusive`];
    if (side === "minimum" ? value < boundary || (!inclusive && value === boundary)
      : value > boundary || (!inclusive && value === boundary)) throw new ApiError(400, "参数超出驱动公布的范围。");
  }
  if (!await confirmAction(`确认将 ${descriptor.name} 写为 ${value} ${descriptor.unit}？\n只修改 RAM，不自动保存 Flash；提交后需等待独立回读确认。`)) return;
  await displayParameter(descriptor, await api("register-write", {address: descriptor.address, value}));
  notice("写入及独立回读已确认；未保存 Flash，电机保持失能。");
}));

function draftTarget() {
  const target = {};
  for (const field of activeFields()) {
    const input = $(`target-${field}`);
    if (!input.value.trim() || !input.checkValidity()) throw new ApiError(400, `${field} 输入无效或超出协议范围。`);
    target[field] = Number(input.value);
    if (!Number.isFinite(target[field])) throw new ApiError(400, `${field} 必须是有限数值。`);
    if (target[field] < Number(input.min) || target[field] > Number(input.max)) {
      throw new ApiError(400, `${field} 超出驱动公布的范围。`);
    }
  }
  return target;
}

$("task-kind").addEventListener("change", () => renderTask(connected && Boolean(session) && data?.has_control));
$("task-form").addEventListener("submit", event => {
  event.preventDefault();
  action(async () => {
    const kind = $("task-kind").value;
    const request = {kind, target: draftTarget(), times: [], positions: []};
    for (const field of ["duration", "max_velocity", "max_acceleration", "tolerance",
      "settling_time", "ramp_time", "amplitude", "frequency"]) {
      const input = $(`task-${field}`);
      const applicable = !input.closest("[data-task-kinds]") || !input.closest("[data-task-kinds]").hidden;
      if (applicable && (!input.value.trim() || !input.checkValidity())) throw new ApiError(400, `任务 ${field} 输入无效。`);
      request[field] = applicable ? Number(input.value) : 0;
      if (!Number.isFinite(request[field])) throw new ApiError(400, `任务 ${field} 必须是有限数值。`);
    }
    if (kind === "trajectory") {
      const lines = $("task-points").value.trim().split(/\r?\n/);
      if (lines.length < 2 || lines.length > 256) throw new ApiError(400, "轨迹需要 2～256 个点。");
      for (const line of lines) {
        const parts = line.split(",").map(part => part.trim());
        if (parts.length !== 2 || parts.some(part => !part || !Number.isFinite(Number(part)))) throw new ApiError(400, "每行轨迹必须是两个有限数字：时间, 位置。");
        request.times.push(Number(parts[0])); request.positions.push(Number(parts[1]));
      }
    }
    if (!await confirmAction(`确认执行 ${kind.toUpperCase()}，时长 ${request.duration} s？\n使用当前目标草稿；结束后失能，不恢复手动目标。`)) return;
    notice((await api("task", request)).message);
    data.task = {state: "submitting", message: "等待驱动接受任务。"};
  });
});
$("cancel-task").addEventListener("click", () => $("stop").click());
$("quick-stop").addEventListener("click", () => $("stop").click());
document.addEventListener("keydown", event => {
  if (event.key === "Escape" && connected && !event.repeat) { event.preventDefault(); $("stop").click(); }
});
$("pause-chart").addEventListener("click", () => {
  chartPaused = !chartPaused;
  $("pause-chart").textContent = chartPaused ? "恢复绘图" : "暂停绘图";
  $("pause-chart").setAttribute("aria-pressed", String(chartPaused));
});
$("clear-chart").addEventListener("click", () => {
  historyStart = historyCount = 0;
  $("export-csv").disabled = true;
  draw();
  notice("曲线缓存已清空；未改变电机控制或反馈采集。");
});
$("chart-window").addEventListener("change", () => draw());
$("chart-position").addEventListener("change", () => draw());
$("chart-targets").addEventListener("change", () => draw());
$("export-state").addEventListener("click", () => saveFile("motor-state.json",
  JSON.stringify({exported_at: new Date().toISOString(), ...data}, null, 2), "application/json"));
$("export-parameters").addEventListener("click", () => saveFile("motor-parameters.json",
  JSON.stringify({motor_id: data.motor_id, name: data.capabilities.name,
    profile: data.capabilities.profile, exported_at: new Date().toISOString(),
    registers: [...parameterValues.values()]}, null, 2), "application/json"));
$("export-csv").addEventListener("click", () => {
  const columns = ["motor_id", "sequence", "time", "feedback_count", "source_skipped_samples", "position", "absolute_position", "native_position",
    "absolute_position_valid", "position_wraps", "velocity", "torque", "mos_temperature",
    "rotor_temperature", "target_position", "target_velocity", "target_torque",
    "applied_target_sequence", "target_accept_latency_ms"];
  const lines = [columns.join(",")];
  for (let i = 0; i < historyCount; i += 1) {
    const point = history[(historyStart + i) % history.length];
    lines.push(columns.map(field => field === "motor_id" ? data.motor_id : point[field] ?? "").join(","));
  }
  saveFile("motor-feedback.csv", lines.join("\n"), "text/csv;charset=utf-8");
});
$("clear-events").addEventListener("click", () => $("event-log").replaceChildren());
$("confirmation-accept").addEventListener("click", () => finishConfirmation(true));
$("confirmation-cancel").addEventListener("click", () => finishConfirmation(false));
$("confirmation-stop").addEventListener("click", () => $("stop").click());
$("confirmation").addEventListener("cancel", event => {
  event.preventDefault();
  finishConfirmation(false);
});
for (const field of fields) {
  $(`target-${field}`).addEventListener("input", () => {
    if (field === "position") {
      endPositionGesture();
      positionFollowsCurrent = false;
    }
    syncSlider(field);
    renderDraft();
    renderPositionScroll();
    queuePositionTarget();
  });
  if (field === "position") continue;
  $(`slider-${field}`).addEventListener("input", () => {
    $(`target-${field}`).value = $(`slider-${field}`).value;
    renderDraft();
    queuePositionTarget();
  });
}
const positionScroll = $("position-scroll");
function adjustRelativePosition(delta) {
  if (!canEditPosition()) return;
  if (Math.abs(delta) < 1e-12) return;
  const {minimum, maximum} = positionBounds();
  if (!Number.isFinite(minimum) || !Number.isFinite(maximum) || minimum > maximum) {
    notice("位置限位无效，未调整滚轮或发送目标。");
    return;
  }
  const current = Math.max(minimum, Math.min(maximum, Number($("target-position").value)));
  const value = Math.max(minimum, Math.min(maximum, current + delta));
  if (value === current && ((current === minimum && delta < 0) || (current === maximum && delta > 0))) {
    renderPositionScroll();
    return;
  }
  if (!Number.isFinite(value) || (delta !== 0 && value === current)) {
    notice("已达到数值表示或微调精度边界，未改变目标；这不是无限精度的位置计数器。");
    syncPositionScroll();
    return;
  }
  positionFollowsCurrent = false;
  $("target-position").value = String(value);
  syncPositionScroll();
  renderDraft();
  renderPositionScroll();
  queuePositionTarget();
}

function positionPointerAngle(event, bounds = positionPointer?.bounds || positionScroll.getBoundingClientRect()) {
  const x = event.clientX - bounds.left - bounds.width / 2;
  const y = event.clientY - bounds.top - bounds.height / 2;
  if (Math.hypot(x, y) < Math.min(bounds.width, bounds.height) / 8) return null;
  return Math.atan2(y, x);
}

function endPositionGesture() {
  const pointer = positionPointer;
  positionPointer = null;
  const wheel = $("position-scroll");
  wheel.classList.remove("dragging");
  if (pointer && wheel.hasPointerCapture(pointer.id)) wheel.releasePointerCapture(pointer.id);
}

function movePositionGesture(event) {
  if (!positionPointer || event.pointerId !== positionPointer.id) return;
  if (!canEditPosition()) { cancelPositionUpdates(); return; }
  const angle = positionPointerAngle(event);
  const previous = positionPointer.angle;
  positionPointer.angle = angle;
  if (angle === null || previous === null) return;
  const delta = angle - previous;
  adjustRelativePosition(Math.atan2(Math.sin(delta), Math.cos(delta)));
}

positionScroll.addEventListener("pointerdown", event => {
  if (!canEditPosition() || positionPointer || event.button !== 0) return;
  const bounds = positionScroll.getBoundingClientRect();
  const angle = positionPointerAngle(event, bounds);
  if (angle === null) return;
  event.preventDefault();
  positionFollowsCurrent = false;
  positionPointer = {id: event.pointerId, angle, bounds};
  positionScroll.focus({preventScroll: true});
  positionScroll.setPointerCapture(event.pointerId);
  positionScroll.classList.add("dragging");
});
positionScroll.addEventListener("pointermove", movePositionGesture);
positionScroll.addEventListener("pointerup", event => {
  if (!positionPointer || event.pointerId !== positionPointer.id) return;
  movePositionGesture(event);
  endPositionGesture();
});
positionScroll.addEventListener("pointercancel", event => {
  if (positionPointer?.id === event.pointerId) cancelPositionUpdates();
});
positionScroll.addEventListener("lostpointercapture", event => {
  if (positionPointer?.id === event.pointerId && !positionScroll.hasPointerCapture(event.pointerId)) {
    endPositionGesture();
  }
});
positionScroll.addEventListener("blur", endPositionGesture);
positionScroll.addEventListener("wheel", event => {
  if (!canEditPosition() || event.ctrlKey) return;
  const delta = event.deltaX || event.deltaY;
  if (!delta) return;
  event.preventDefault();
  const scale = event.deltaMode === 1 ? 16 : event.deltaMode === 2 ? positionScroll.clientWidth : 1;
  adjustRelativePosition(delta * scale * positionRadiansPerPixel);
}, {passive: false});
positionScroll.addEventListener("keydown", event => {
  if (!canEditPosition()) return;
  const directions = {ArrowLeft: -1, ArrowDown: -1, ArrowRight: 1, ArrowUp: 1};
  if (!(event.key in directions) && !["Home", "End"].includes(event.key)) return;
  event.preventDefault();
  if (["Home", "End"].includes(event.key)) {
    return;
  }
  adjustRelativePosition(directions[event.key] * (event.shiftKey ? 0.1 : 0.01));
});
$("control-rate").addEventListener("change", async () => {
  const rate = Number($("control-rate").value);
  if (!$("control-rate").value.trim() || !Number.isFinite(rate) || rate < 20 || rate > 1000) {
    notice("控制频率必须是 20–1000 Hz 的有限数值，未提交。");
    return;
  }
  ratePending = true;
  render();
  try { notice((await api("control-rate", {control_rate: rate})).message); }
  catch (error) {
    if (!(error instanceof ApiError)) connectionLost(error);
    else notice(error.message);
  } finally { ratePending = false; render(); }
});
$("live-targets").addEventListener("change", () => {
  cancelPositionUpdates();
  notice($("live-targets").checked
    ? "实时手动目标已开启：后续有效修改立即提交，未回放已有草稿。"
    : "实时手动目标已关闭：后续仅编辑草稿，当前电机保持最后接受的目标；需要失能请按 STOP。");
  render();
});
$("fill-position").addEventListener("click", () => {
  if ($("fill-position").disabled) return;
  const input = $("target-position");
  const position = currentPosition();
  if (!Number.isFinite(position) || position < Number(input.min) || position > Number(input.max)) {
    notice("当前位置超出允许目标范围或无效，未修改草稿。");
    return;
  }
  cancelPositionUpdates();
  positionFollowsCurrent = false;
  input.value = String(position);
  syncSlider("position");
  renderDraft();
  notice("已填入当前位置，仅修改位置草稿；未发送、未归零、未设置增益。");
});
$("restore-target").addEventListener("click", () => {
  if ($("restore-target").disabled) return;
  cancelPositionUpdates();
  positionFollowsCurrent = false;
  for (const field of activeFields()) {
    $(`target-${field}`).value = String(data.target[field]);
    syncSlider(field);
  }
  renderDraft();
  notice("草稿已恢复为服务目标，未发送新命令。");
});
$("target-form").addEventListener("submit", (event) => {
  event.preventDefault();
  action(async () => {
    const target = draftTarget();
    const epoch = positionUpdateEpoch;
    positionFollowsCurrent = false;
    notice((await api("target", target)).message);
    if (epoch === positionUpdateEpoch) positionLiveBlocked = false;
  });
});
$("zero").addEventListener("click", () => action(async () => {
  if (!await confirmAction("确认应用零目标？\n位置模式可能向原零点运动；MIT 会清零 Kp/Kd/前馈力矩，不是锁轴或抱闸。")) return;
  resetInputs(false);
  const epoch = positionUpdateEpoch;
  notice((await api("target", Object.fromEntries(activeFields().map((field) => [field, 0])))).message
    + " 零目标不等于锁轴，外部负载可能移动。");
  if (epoch === positionUpdateEpoch) positionLiveBlocked = false;
}));

setInterval(async () => {
  if (!connected || !session || heartbeatPending) return;
  heartbeatPending = true;
  const version = connectionVersion;
  try { await api("heartbeat", {}); }
  catch (error) { if (version === connectionVersion) connectionLost(error); }
  finally { heartbeatPending = false; }
}, 150);
document.addEventListener("visibilitychange", () => {
  if (document.hidden && connected) { relinquish(); notice("页面已隐藏，控制权已释放；返回后需重新连接。"); }
  else if (!document.hidden) initialConnect();
});
window.addEventListener("pagehide", relinquish);
window.addEventListener("resize", () => draw());
$("endpoint").textContent = `服务地址：${location.origin} · 浏览器心跳 150 ms · 无登录鉴权，仅用于可信隔离网络`;
render();
draw();
setInterval(() => animate(performance.now()), 20);
initialConnect();
