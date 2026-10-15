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
const history = new Array(16384);
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

class ApiError extends Error {
  constructor(status, message) { super(message); this.status = status; }
}

function headers() {
  return {"X-Control-Session": session, "Content-Type": "application/json"};
}

async function api(path, body) {
  const response = await fetch(`/api/${path}`, {
    method: body === undefined ? "GET" : "POST",
    headers: headers(),
    ...(body === undefined ? {} : {body: JSON.stringify(body)}),
    signal: AbortSignal.any([connectionAbort.signal, AbortSignal.timeout(2000)]),
    cache: "no-store",
  });
  const result = await response.json();
  if (!response.ok) throw new ApiError(response.status, result.error || `HTTP ${response.status}`);
  return result;
}

function notice(message) { $("notice").textContent = message; }

function syncSlider(field) {
  const input = $(`target-${field}`);
  const slider = $(`slider-${field}`);
  const value = Number(input.value);
  if (input.value.trim() && Number.isFinite(value)) {
    slider.value = String(Math.min(Number(input.max), Math.max(Number(input.min), value)));
  }
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

function resetInputs() {
  for (const field of fields) {
    $(`target-${field}`).value = "0";
    syncSlider(field);
  }
  renderDraft();
}

function relinquish() {
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
  const owns = connected && Boolean(session) && data?.has_control;
  $("connection").textContent = !connected ? "未连接" : owns ? "已连接 · 拥有控制权" : "已连接 · 只读";
  $("connection").className = `badge${connected ? " ok" : ""}`;
  $("connect").disabled = connected || busy;
  $("disconnect").disabled = !connected;
  $("claim").disabled = !connected || owns || busy;
  $("enable").disabled = !owns || busy || !data.connected || !state?.feedback_fresh
    || !state.ready || !state.activity_available || state.phase !== 0;
  $("apply").disabled = !owns || busy || !data.connected || !data.armed || state?.phase !== 2;
  $("zero").disabled = $("apply").disabled;
  $("stop").disabled = !connected;
  $("reset").disabled = !owns || busy || !data.connected || !state?.feedback_fresh || state.phase !== 4 || state.status !== 0;
  const editable = owns && !busy && data.connected && state?.feedback_fresh;
  $("fill-position").disabled = !editable || !activeFields().includes("position");
  $("restore-target").disabled = !editable;
  for (const field of fields) {
    const active = activeFields().includes(field);
    $(`target-${field}`).parentElement.hidden = !active;
    $(`target-${field}`).disabled = !editable || !active;
    $(`slider-${field}`).disabled = !editable || !active;
  }
  const disabledReady = editable && state?.phase === 0 && state.status === 0 && state.ready
    && state.maintenance_available && !state.task_active;
  for (const id of ["mode-select", "set-mode", "device-zero", "software-zero", "save-parameters"]) {
    $(id).disabled = !disabledReady;
  }
  $("clear-fault").disabled = !(disabledReady || (editable && state?.ready && !state.task_active
    && state.phase === 4 && state.status !== 1));
  $("parameter-select").disabled = !connected || !data?.capabilities?.ready;
  $("read-parameter").disabled = $("parameter-select").disabled || !state?.maintenance_available || busy;
  renderCapabilities();
  renderDraft();
  if (!state) return;
  $("current-mode").textContent = `模式 ${state.mode || 1} · ${modeNames[state.mode || 1] || "未知"}`;
  $("driver-targets").textContent = JSON.stringify({
    target_active: state.target_active, requested: state.requested_target, active: state.active_target}, null, 2);
  $("motor-id").textContent = `#${data.motor_id}`;
  updateReadings(connected && latestSample ? latestSample : state);
  $("temperature").textContent = `${state.mos_temperature} / ${state.rotor_temperature}`;
  const stale = !connected || !data.connected || !state.feedback_fresh;
  $("phase").textContent = `${state.simulated ? "SIMULATION" : "HARDWARE"} · ${stale ? "状态过期 / 未确认" : phases[state.phase] || "UNKNOWN"}`;
  $("phase").className = `badge${stale || state.phase === 4 ? " error" : state.phase === 2 ? " ok" : ""}`;
  $("phase").title = `设备状态: ${state.status_text ?? state.status}; 主机故障: ${Boolean(state.host_fault)}; `
    + `电机故障: ${Boolean(state.device_fault)}; 主机额外物理保护: ${Boolean(state.host_protection_enabled)}`;
  $("telemetry").textContent = `${stale ? "上次反馈 · " : ""}CAN ${state.motor_id} / MST ${state.master_id} · 状态码 0x${state.status.toString(16).toUpperCase()} · `
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
    input.min = String(minimum);
    input.max = String(maximum);
    const slider = $(`slider-${field}`);
    slider.min = input.min;
    slider.max = input.max;
    syncSlider(field);
    $(`range-${field}`).textContent = `${minimum} ～ ${maximum}`;
  }
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
    option.textContent = `0x${parameter.address.toString(16)} ${parameter.name} (${parameter.integer ? "uint32" : "float32"} ${parameter.unit})`;
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
  while (historyCount && now - history[historyStart].time > 15) {
    historyStart = (historyStart + 1) % history.length;
    historyCount -= 1;
  }
  const colors = ["#63b8ff", "#7ade98", "#ffcf70"];
  const labels = ["位置 / rad", "速度 / rad/s", "力矩 / Nm"];
  ["position", "velocity", "torque"].forEach((field, index) => {
    const height = rect.height / 3;
    const top = index * height;
    let scale = 0.01;
    for (let i = 0; i < historyCount; i += 1) {
      scale = Math.max(scale, Math.abs(history[(historyStart + i) % history.length][field]));
    }
    ctx.strokeStyle = "#28384e";
    ctx.beginPath();
    ctx.moveTo(0, top + height / 2);
    ctx.lineTo(rect.width, top + height / 2);
    ctx.stroke();
    ctx.fillStyle = colors[index];
    ctx.font = "12px system-ui";
    ctx.fillText(`${labels[index]}   ±${scale.toFixed(3)}`, 4, top + 14);
    ctx.strokeStyle = colors[index];
    ctx.lineWidth = 1.8;
    ctx.beginPath();
    let pixel = -1, minimum = Infinity, maximum = -Infinity, started = false;
    const y = (value) => top + height / 2 - value / scale * height * 0.32;
    const flush = () => {
      if (pixel < 0) return;
      if (!started) { ctx.moveTo(pixel, y(minimum)); started = true; }
      else ctx.lineTo(pixel, y(minimum));
      if (maximum !== minimum) ctx.lineTo(pixel, y(maximum));
    };
    // Preserve min/max peaks within each screen pixel instead of drawing every stored point.
    for (let i = 0; i < historyCount; i += 1) {
      const point = history[(historyStart + i) % history.length];
      const x = Math.min(Math.ceil(rect.width) - 1, Math.floor(rect.width * (1 - (now - point.time) / 15)));
      if (x !== pixel) {
        flush();
        pixel = x;
        minimum = point[field];
        maximum = point[field];
      } else {
        minimum = Math.min(minimum, point[field]);
        maximum = Math.max(maximum, point[field]);
      }
    }
    flush();
    ctx.stroke();
  });
}

function animate(time) {
  if (connected && latestSample && feedbackStream && !document.hidden) {
    draw(time);
    updateReadings(latestSample);
    renderedFrames += 1;
    const elapsed = time - statsStarted;
    if (elapsed >= 1000) {
      $("plot-stats").textContent = `推流 ${(receivedSamples * 1000 / elapsed).toFixed(1)} Hz · `
        + `绘图 ${(renderedFrames * 1000 / elapsed).toFixed(1)} FPS · 跳过 ${skippedSamples} 个推流样本`;
      receivedSamples = 0;
      renderedFrames = 0;
      statsStarted = time;
    }
  }
  requestAnimationFrame(animate);
}

async function poll() {
  if (!connected || polling) return;
  polling = true;
  const version = connectionVersion;
  try {
    const snapshot = await api("state");
    if (version !== connectionVersion) return;
    const previousError = data?.state?.error;
    const previouslyArmed = data?.armed;
    const previouslyOwned = data?.has_control;
    const previousMode = data?.state?.mode;
    data = snapshot;
    if ((previouslyArmed && !data.armed) || (previouslyOwned && !data.has_control)
        || (previousMode !== undefined && previousMode !== data.state?.mode)) resetInputs();
    if (session && !data.has_control) {
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
  const result = await api("enable", {});
  notice(result.message);
  await confirmOperation(result.operation_id);
}));
$("stop").addEventListener("click", async () => {
  resetInputs();
  try { notice((await api("stop", {})).message); }
  catch (error) { notice(`失能未确认：${error.message}。必要时使用硬件断电。`); }
});
$("reset").addEventListener("click", () => action(async () => {
  const result = await api("reset", {});
  notice(result.message);
  await confirmOperation(result.operation_id);
}));
for (const [id, operation] of [["set-mode", "mode"], ["clear-fault", "clear"],
  ["device-zero", "device_zero"], ["software-zero", "software_zero"], ["save-parameters", "save"]]) {
  $(id).addEventListener("click", () => action(async () => {
    if (!confirm(`确认${$(id).textContent}？电机必须保持失能，不会自动使能。`)) return;
    const result = await api("operation", {operation,
      ...(operation === "mode" ? {mode: Number($("mode-select").value)} : {})});
    notice(result.message);
    await confirmOperation(result.operation_id);
  }));
}
$("read-parameter").addEventListener("click", () => action(async () => {
  const address = Number($("parameter-select").value);
  const descriptor = data.capabilities.registers.find((item) => item.address === address);
  if (!descriptor) throw new ApiError(400, "请选择已公布的参数。");
  const response = await api("registers", {addresses: [address]});
  for (const id of response.operation_ids) {
    const result = await confirmOperation(id, "parameter-value");
    if (result.address !== address) throw new ApiError(409, "参数地址与请求不符。");
    const bytes = new DataView(new ArrayBuffer(4));
    bytes.setUint32(0, result.raw_value, true);
    const value = descriptor.integer ? result.raw_value : bytes.getFloat32(0, true);
    $("parameter-value").textContent = `${descriptor.name}: ${value} ${descriptor.unit} · raw 0x${result.raw_value.toString(16)} · 已确认 #${id}`;
  }
}));
for (const field of fields) {
  $(`target-${field}`).addEventListener("input", () => { syncSlider(field); renderDraft(); });
  $(`slider-${field}`).addEventListener("input", () => {
    $(`target-${field}`).value = $(`slider-${field}`).value;
    renderDraft();
  });
}
$("fill-position").addEventListener("click", () => {
  if ($("fill-position").disabled) return;
  const input = $("target-position");
  const position = latestSample && performance.now() - latestSample.arrived < 500
    ? latestSample.position : data.state.position;
  if (!Number.isFinite(position) || position < Number(input.min) || position > Number(input.max)) {
    notice("当前位置超出允许目标范围或无效，未修改草稿。");
    return;
  }
  input.value = String(position);
  syncSlider("position");
  renderDraft();
  notice("已填入当前位置，仅修改位置草稿；未发送、未归零、未设置增益。");
});
$("restore-target").addEventListener("click", () => {
  if ($("restore-target").disabled) return;
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
    const target = {};
    for (const field of activeFields()) {
      const input = $(`target-${field}`);
      if (!input.value.trim() || !input.checkValidity()) throw new ApiError(400, `${field} 输入无效或超出协议范围。`);
      target[field] = Number(input.value);
      if (!Number.isFinite(target[field])) throw new ApiError(400, `${field} 必须是有限数值。`);
    }
    notice((await api("target", target)).message);
  });
});
$("zero").addEventListener("click", () => action(async () => {
  resetInputs();
  notice((await api("target", Object.fromEntries(activeFields().map((field) => [field, 0])))).message
    + " 零目标不等于锁轴，外部负载可能移动。");
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
requestAnimationFrame(animate);
initialConnect();
