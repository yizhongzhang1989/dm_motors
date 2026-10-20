"use strict";

const el = id => document.getElementById(id);
const directions = {rx: "RX · 接收", written: "TX · 写出", submitted: "提交"};
let session = null;
let state = null;
let busy = false;
let claimPending = false;
let releasing = false;
let heartbeatPending = false;
let generation = 0;
let paused = false;
let frameHistory = [];
let idHistory = [];
let visibleFrames = [];
let selectedPanel = "overview-panel";
const count = value => Number.isFinite(value) ? value.toLocaleString("zh-CN") : "--";
const rate = value => Number.isFinite(value) ? value.toFixed(1) : "--";
const ready = () => Boolean(state?.connected && state.status?.ready
  && state.status.endpoint_ready && state.capabilities?.known);

function selectPanel(id, focus = false) {
  selectedPanel = id;
  for (const tab of document.querySelectorAll("[data-panel]")) {
    const selected = tab.dataset.panel === id;
    tab.setAttribute("aria-selected", String(selected));
    tab.tabIndex = selected ? 0 : -1;
    el(tab.dataset.panel).hidden = !selected;
    if (selected && focus) tab.focus();
  }
  el("traffic-tools").hidden = !["overview-panel", "frames-panel"].includes(id);
  renderTables();
}

const tabs = [...document.querySelectorAll("[data-panel]")];
for (const [index, tab] of tabs.entries()) {
  tab.addEventListener("click", () => selectPanel(tab.dataset.panel));
  tab.addEventListener("keydown", event => {
    const offset = {ArrowRight: 1, ArrowLeft: -1}[event.key];
    const next = event.key === "Home" ? 0 : event.key === "End" ? tabs.length - 1
      : offset === undefined ? null : (index + offset + tabs.length) % tabs.length;
    if (next === null) return;
    event.preventDefault();
    selectPanel(tabs[next].dataset.panel, true);
  });
}
document.querySelector(".skip-link").addEventListener("click", () => selectPanel("overview-panel"));

function bytes(value) {
  const text = value.trim();
  if (!text) return [];
  return text.split(/\s+/).map(word => {
    if (!/^[0-9a-fA-F]{2}$/.test(word)) throw new Error("每字节必须为两位十六进制数字，并以空格分隔。");
    return parseInt(word, 16);
  });
}

function number(value) {
  if (!/^(0x[0-9a-fA-F]+|[0-9]+)$/.test(value.trim())) throw new Error("请输入十进制或 0x 开头的十六进制整数。");
  const result = Number(value);
  if (!Number.isSafeInteger(result) || result > 0x1fffffff) throw new Error("CAN ID 必须在 0～0x1FFFFFFF 范围内。");
  return result;
}

function flags(frame) {
  return ["extended", "rtr", "fd", "brs", "error"].filter(key => frame[key]).map(key => key.toUpperCase()).join(" ") || "STD";
}
function hex(data) { return data.map(byte => byte.toString(16).padStart(2, "0")).join(" "); }
function row(values) {
  const tr = document.createElement("tr");
  for (const value of values) {
    const td = document.createElement("td");
    td.textContent = String(value);
    tr.append(td);
  }
  return tr;
}
function emptyRow(columns, text) {
  const tr = row([text]);
  tr.firstChild.colSpan = columns;
  tr.firstChild.className = "empty";
  return tr;
}
function frameRow(frame, values, directionColumn) {
  const tr = row(values);
  const badge = document.createElement("span");
  badge.textContent = directions[frame.direction] || frame.direction;
  badge.className = `direction ${frame.direction}`;
  tr.children[directionColumn].replaceChildren(badge);
  tr.lastChild.className = "hex-data";
  tr.classList.toggle("frame-error", Boolean(frame.error));
  return tr;
}

async function api(path, body, keepalive = false) {
  const response = await fetch(path, {
    cache: "no-store", signal: AbortSignal.timeout(2000), keepalive,
    ...(body === undefined ? {} : {
      method: "POST", headers: {"Content-Type": "application/json"}, body: JSON.stringify(body),
    }),
  });
  const result = await response.json();
  if (!response.ok) throw new Error(result.error || `HTTP ${response.status}`);
  return result;
}

function message(text, error = false) {
  el("result").parentElement.hidden = false;
  el("result").textContent = text;
  el("result").classList.toggle("error", error);
}

function controls() {
  const available = ready() && !document.hidden;
  el("debug").disabled = !available;
  el("claim-fields").disabled = Boolean(session || state?.debug) || busy || releasing || Boolean(state?.release_pending);
  el("claim").disabled = !available || el("claim-fields").disabled;
  el("release").disabled = !session && !claimPending;
  el("release").textContent = claimPending ? "取消路由申请" : "释放调试租约";
  el("send-fields").disabled = !session || busy;
  el("send").disabled = !available || !session || busy;
  el("extended").disabled = state?.status?.backend === "serial";
  el("fd").disabled = !state?.capabilities?.can_fd || state?.status?.backend === "serial" || el("rtr").checked;
  el("brs").disabled = !el("fd").checked || el("fd").disabled;
  el("rtr").disabled = state?.status?.backend === "serial" || el("fd").checked;
  el("debug-guidance").textContent = !ready() ? "总线状态过期或未就绪，无法申请 / 发送；已有租约仍可释放。"
    : releasing || state?.release_pending ? "正在等待旧租约过期；不会自动重新申请。"
    : claimPending ? "路由申请处理中，可在页头取消申请。"
    : session ? "路由已锁定；可提交单帧或随时释放租约。"
    : state?.debug ? "已有独占调试会话，本页面仅监控；不能抢占或发送。"
    : "填写路由与设备停止帧，点击“申请独占路由”直接提交，无需额外勾选或确认。";
  el("frame-help").textContent = state?.status?.backend === "serial"
    ? "旧串口适配器：仅标准经典帧、通道 0、4 / 8 字节；4 字节由适配器补零。"
    : `经典帧 0～8 字节${state?.capabilities?.can_fd ? "；FD 支持 0～8 / 12 / 16 / 20 / 24 / 32 / 48 / 64 字节" : "；当前后端不支持 FD"}。BRS 仅用于 FD，RTR 仅用于经典帧。`;
  el("claimed-route").textContent = session
    ? `固定路由 0x${session.route.can_id.toString(16).toUpperCase()} · 通道 ${session.route.channel} · ${session.route.extended ? "扩展帧" : "标准帧"} · ${session.readonly ? "active=false（仍可发送）" : "活动租约"}`
    : "申请成功后显示固定路由；修改路由需要先释放。";
}

async function releaseSession(reason) {
  generation += 1;
  const previous = session;
  const pending = claimPending;
  session = null;
  claimPending = false;
  if (!previous) {
    if (pending) message(`${reason} 路由申请已取消；迟到结果会释放，不会自动发送。`);
    controls();
    return;
  }
  releasing = true;
  controls();
  message(`${reason} 已停止续约，正在请求释放。`);
  try {
    const result = await api("/api/release", {session: previous.session}, true);
    message(`${reason} 释放非即时生效；等待总线租约过期（约 ${result.lease_expiry_seconds} s）。`);
  } catch (error) {
    message(`${reason} 释放未确认：${error.message} 已停止续约，请等待看门狗过期；软件无法保证设备停止。`, true);
  } finally {
    releasing = false;
    controls();
  }
}

async function action(fn) {
  busy = true;
  controls();
  const version = generation;
  try {
    const result = await fn();
    if (version === generation) message(result);
  } catch (error) {
    message(error.message, true);
  } finally {
    busy = false;
    claimPending = false;
    controls();
  }
}

el("release").addEventListener("click", () => {
  void releaseSession(claimPending ? "已取消路由申请。" : "已主动释放调试会话。");
});
el("claim-form").addEventListener("submit", event => {
  event.preventDefault();
  if (el("claim").disabled) return;
  claimPending = true;
  void action(async () => {
    const version = generation;
    const stops = el("stops").value.trim();
    let stopFrames;
    try { stopFrames = JSON.parse(stops || (el("readonly").checked ? "[]" : "")); }
    catch (error) {
      if (!(error instanceof SyntaxError)) throw error;
      throw new Error("停止帧必须是有效的 JSON 数组；活动租约必须提供设备安全停止帧。");
    }
    if (!Array.isArray(stopFrames)) throw new Error("停止帧必须是 JSON 数组。");
    const result = await api("/api/session", {
      confirm: true, owner: el("owner").value.trim(), duration: Number(el("duration").value),
      readonly: el("readonly").checked,
      route: {can_id: number(el("canid").value), channel: number(el("channel").value),
        extended: el("extended").checked, prefix: bytes(el("prefix").value)},
      stop_frames: stopFrames,
    });
    if (version !== generation || document.hidden) {
      await api("/api/release", {session: result.session}, true);
      throw new Error("申请已取消或页面状态已改变，迟到的路由申请已释放；不会恢复调试。");
    }
    session = result;
    return `已申请独占路由 0x${result.route.can_id.toString(16).toUpperCase()}；尚未发送数据。`;
  });
});
el("send-form").addEventListener("submit", event => {
  event.preventDefault();
  if (el("send").disabled) return;
  void action(async () => {
    const result = await api("/api/send", {
      session: session.session, confirm: true,
      frame: {can_id: session.route.can_id, channel: session.route.channel,
        extended: session.route.extended, rtr: el("rtr").checked, fd: el("fd").checked,
        brs: el("brs").checked, data: bytes(el("data").value)},
    });
    return `已提交单帧 #${result.sequence}，未确认驱动写出或设备执行。${result.note || ""}`;
  });
});
for (const id of ["fd", "rtr"]) el(id).addEventListener("change", () => {
  if (!el("fd").checked) el("brs").checked = false;
  controls();
});

function renderState() {
  const status = state?.status;
  const unsupported = status?.backend === "serial" ? ["extended", "fd", "brs", "rtr"]
    : state?.capabilities?.can_fd ? [] : ["fd", "brs"];
  const cleared = unsupported.filter(id => el(id).checked);
  for (const id of cleared) el(id).checked = false;
  if (cleared.length) {
    message("后端能力已改变：已清除不支持的帧标记，请重新核对路由和数据。", true);
    if (session) void releaseSession("后端能力已改变，调试会话已结束。");
  }
  const fresh = Boolean(state?.connected);
  const available = ready();
  document.querySelector(".status-panel").classList.toggle("stale", !fresh);
  el("connection").textContent = !fresh ? "未连接" : available ? "已连接" : "未就绪";
  el("connection").title = !fresh ? "状态过期 / 未连接" : available ? "已连接 · 总线与端点就绪" : "总线 / 端点或能力尚未就绪";
  el("connection").className = `badge ${available ? "ok" : "error"}`;
  el("bus-namespace").textContent = state?.bus_namespace || "等待连接";
  el("bus-namespace").title = el("bus-namespace").textContent;
  document.title = `CAN 总线 · ${state?.bus_namespace || "等待连接"}`;
  el("backend-label").textContent = status ? (status.backend === "sim" ? "仿真" : "硬件") : "待发现";
  el("backend-label").title = status ? `${status.backend === "sim" ? "SIMULATION" : "HARDWARE"} · ${status.backend}` : "等待后端能力";
  for (const [id, value] of [["rx-count", status?.rx_count], ["tx-count", status?.tx_count],
    ["rejected-count", status?.rejected], ["clients-count", status?.clients]]) {
    el(id).textContent = count(value);
    el(id).title = el(id).textContent;
  }
  el("rejected-count").classList.toggle("warning", Boolean(status?.rejected));
  el("rx-rate").textContent = `${rate(state?.monitor?.rates?.rx)} 帧/s`;
  el("tx-rate").textContent = `${rate(state?.monitor?.rates?.written)} 帧/s`;
  el("rx-rate").title = el("rx-rate").textContent;
  el("tx-rate").title = `${el("tx-rate").textContent} · 驱动写出不等于 CAN ACK 或设备执行`;
  el("quiet-state").textContent = status ? (status.all_quiet ? "全部 quiet" : "有活动客户端") : "等待状态";
  el("quiet-state").title = status ? (status.all_quiet ? "所有客户端 quiet" : "存在活动 / 未 quiet 客户端") : "等待客户端状态";
  el("bus-endpoint").textContent = status ? `${status.port || status.backend} · ${count(status.can_bitrate)} bit/s` : "等待接口信息";
  el("queue-status").textContent = `发送队列 ${count(status?.queue_depth)}`;
  el("observer-status").textContent = `观察链路丢弃 ${count(status?.observer_dropped)} · 队列 ${count(status?.observer_queue_depth)}`;
  el("status-age").textContent = `状态年龄 ${Number.isFinite(state?.status_age) ? (state.status_age * 1000).toFixed(0) : "--"} ms`;
  const faults = status?.error || status?.client_errors?.join("；") || status?.group_faults?.join("；");
  el("health-banner").className = `health-banner ${!fresh ? "warning" : !ready() || faults ? "error" : status.observer_dropped || !state.tx_observed_available ? "warning" : "ok"}`;
  el("health-banner").textContent = !fresh ? "总线状态已过期或尚未连接：显示的是上次数据，调试发送已禁用。"
    : faults ? `总线诊断：${faults}`
    : !ready() ? "总线 / 端点尚未就绪，不能申请路由或发送。"
    : status.observer_dropped ? `观察链路已丢弃 ${count(status.observer_dropped)} 帧，列表不是完整总线记录；总量以总线计数为准。`
    : !state.tx_observed_available ? "尚无驱动写出观察源；提交帧不能作为已写出的证明。"
    : `${status.backend === "sim" ? "仿真总线" : "真实总线"} · 连接就绪。监控不发送帧；驱动写出不等于 CAN ACK 或设备执行。`;
  el("notice").textContent = state?.notice || "";
  el("notice").hidden = !state?.notice;
  el("session").textContent = state?.debug
    ? `${state.debug.owner} · 剩余 ${rate(state.debug.remaining)} s · ${state.debug.readonly ? "active=false" : "活动租约"}${session ? " · 本页面" : " · 非本页面"}`
    : releasing || state?.release_pending ? "等待租约过期" : session ? "已申请 · 等待状态确认" : "无调试会话";
  el("export-state").disabled = !status;
  const entries = [
    ["后端 / 命名空间", `${status?.backend || "--"} / ${state?.bus_namespace || "--"}`],
    ["总线 / 端点就绪", `${Boolean(status?.ready)} / ${Boolean(status?.endpoint_ready)}`],
    ["Epoch（精确整数）", status?.epoch ?? "--"], ["支持 CAN FD", Boolean(state?.capabilities?.can_fd)],
    ["维护所有者", status?.maintenance_owner || "无"], ["释放等待", Boolean(state?.release_pending)],
    ["观察错误帧", count(state?.monitor?.error_frames)], ["可观察驱动写出", Boolean(state?.tx_observed_available)],
    ["客户端错误", status?.client_errors?.join("；") || "无"], ["故障组", status?.group_faults?.join("；") || "无"],
  ];
  el("status").replaceChildren(...entries.flatMap(([key, value]) => {
    const term = document.createElement("dt"), definition = document.createElement("dd");
    term.textContent = key; definition.textContent = String(value);
    return [term, definition];
  }));
  const clients = (status?.client_details || []).map(client => row([
    client.name, client.heartbeat_sequence, client.rx_ready ? "就绪" : "未就绪", client.fault ? "故障" : "正常",
  ]));
  el("clients").replaceChildren(...(clients.length ? clients : [emptyRow(4, "暂无注册客户端；监控本身不占用路由。")]));
  el("raw-status").textContent = JSON.stringify(state, null, 2);
  controls();
}

function renderTables() {
  let filter;
  try {
    filter = el("filter").value.trim() ? number(el("filter").value) : null;
    el("filter-error").hidden = true;
    el("filter").setAttribute("aria-invalid", "false");
  } catch (error) {
    el("filter-error").textContent = error.message;
    el("filter-error").hidden = false;
    el("filter").setAttribute("aria-invalid", "true");
    el("ids").replaceChildren(emptyRow(7, "请修正 CAN ID 筛选；总线连接与心跳不受影响。"));
    el("frames").replaceChildren(emptyRow(6, "请修正 CAN ID 筛选；总线连接与心跳不受影响。"));
    el("export-frames").disabled = true;
    visibleFrames = [];
    return;
  }
  const direction = el("direction").value;
  const matches = frame => (filter === null || frame.can_id === filter) && (!direction || frame.direction === direction);
  const ids = idHistory.filter(item => matches(item.last)).sort((a, b) =>
    a.last.can_id - b.last.can_id || a.last.direction.localeCompare(b.last.direction));
  const matchingFrames = frameHistory.filter(matches);
  visibleFrames = matchingFrames.slice(-200).reverse();
  el("id-count").textContent = `${ids.length} 个 ID / 方向组合${paused ? " · 列表已暂停" : ""}`;
  el("frame-count").textContent = `${visibleFrames.length} / ${matchingFrames.length} 条匹配${paused ? " · 列表已暂停" : ""}`;
  el("export-frames").disabled = !visibleFrames.length;
  const empty = filter !== null || direction ? "没有匹配的帧。可清除筛选查看全部数据。" : "等待 CAN 帧。监控不会主动发送探测帧。";
  if (selectedPanel === "overview-panel") {
    el("ids").replaceChildren(...(ids.length ? ids.map(item => {
      const frame = item.last;
      return frameRow(frame, [frame.direction, `0x${frame.can_id.toString(16).toUpperCase()}`,
        frame.channel, flags(frame), count(item.count), rate(item.rate), hex(frame.data)], 0);
    }) : [emptyRow(7, empty)]));
  }
  if (selectedPanel === "frames-panel") {
    el("frames").replaceChildren(...(visibleFrames.length ? visibleFrames.map(frame =>
      frameRow(frame, [frame.sequence, frame.direction, `0x${frame.can_id.toString(16).toUpperCase()}`,
        frame.channel, flags(frame), hex(frame.data)], 1)) : [emptyRow(6, empty)]));
  }
}

for (const id of ["filter", "direction"]) el(id).addEventListener(id === "filter" ? "input" : "change", renderTables);
el("reset-filter").addEventListener("click", () => {
  el("filter").value = ""; el("direction").value = ""; renderTables();
});
el("pause-frames").addEventListener("click", () => {
  paused = !paused;
  el("pause-frames").textContent = paused ? "恢复列表" : "暂停列表";
  el("pause-frames").setAttribute("aria-pressed", String(paused));
  renderTables();
  message(paused ? "列表已暂停；总线监控、状态计数与调试心跳继续。这不是停止设备。"
    : "列表已恢复，将显示下一次收到的数据，不重放控制帧。");
});
function saveFile(name, content, type) {
  const url = URL.createObjectURL(new Blob([content], {type}));
  const link = document.createElement("a");
  link.href = url; link.download = name; link.click();
  queueMicrotask(() => URL.revokeObjectURL(url));
}
el("export-frames").addEventListener("click", () => {
  const lines = ["sequence,direction,can_id,channel,flags,data"];
  for (const frame of visibleFrames) lines.push([
    frame.sequence, frame.direction, `0x${frame.can_id.toString(16)}`, frame.channel, flags(frame), hex(frame.data),
  ].join(","));
  saveFile("can-frames.csv", lines.join("\n"), "text/csv;charset=utf-8");
});
el("export-state").addEventListener("click", () => saveFile("can-state.json",
  JSON.stringify({exported_at: new Date().toISOString(), ...state}, null, 2), "application/json"));

async function poll() {
  try {
    state = await api("/api/state");
    if (!ready() && session) await releaseSession("总线状态过期或未就绪。");
    renderState();
    if (!paused) {
      const frames = await api("/api/frames?limit=1000");
      if (!paused) {
        frameHistory = frames.frames;
        idHistory = state.monitor.ids;
        renderTables();
      }
    }
  } catch (error) {
    if (state) state = {...state, connected: false};
    await releaseSession("监控连接中断。");
    renderState();
    message(`连接异常：${error.message} 调试不自动恢复。`, true);
  } finally {
    setTimeout(poll, 100);
  }
}
setInterval(async () => {
  if (!session || heartbeatPending || document.hidden) return;
  heartbeatPending = true;
  const current = session;
  try { await api("/api/heartbeat", {session: current.session}); }
  catch (error) {
    if (session === current) {
      await releaseSession(`调试心跳中断：${error.message}。`);
    }
  } finally { heartbeatPending = false; }
}, 250);
document.addEventListener("visibilitychange", () => {
  if (document.hidden) {
    void releaseSession("页面已隐藏；返回后需重新申请路由。");
  }
  controls();
});
window.addEventListener("pagehide", () => {
  void releaseSession("已离开页面。");
});
controls();
renderTables();
void poll();
