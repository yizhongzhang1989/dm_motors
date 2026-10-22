"use strict";
(() => {
  const $ = id => document.getElementById(id);
  const numbers = ["stiffness", "coupling_damping", "local_damping", "max_torque", "max_feedback_torque",
    "max_travel", "warning_temperature", "max_temperature", "feedback_age", "pair_skew", "feedback_stop", "recovery_time"];
  const phases = ["待机 / 已停止", "获取会话与核验", "中性使能中", "双边耦合中", "停止中 / 等待确认", "故障停机"];
  const qualities = ["正常配对", "降级 / 零增益", "恢复爬升"];
  let session = "", data = null, dirty = false, initialized = false, busy = false;
  let stopping = false;
  let disconnected = true, heartbeatPending = false, generation = 0, lastSeen = 0, history = [], originKey = "";
  const format = (value, digits = 4) => Number.isFinite(value) ? value.toFixed(digits) : "—";
  function notice(text, error = false) {
    $("notice").textContent = text; $("notice").classList.toggle("error", error);
  }
  async function api(path, body) {
    const response = await fetch("/api/" + path, {
      cache: "no-store", signal: AbortSignal.timeout(4000),
      method: body === undefined ? "GET" : "POST",
      headers: {"X-Control-Session": session, "Content-Type": "application/json"},
      ...(body === undefined ? {} : {body: JSON.stringify(body)}),
    });
    const value = await response.json();
    if (!response.ok) throw new Error(value.error || `HTTP ${response.status}`);
    return value;
  }
  function release(token) {
    if (!token) return;
    fetch("/api/release", {method: "POST", keepalive: true,
      headers: {"Content-Type": "application/json", "X-Control-Session": token}, body: "{}"})
      .catch(error => notice(`释放未确认，操作心跳将到期：${error.message}`, true));
  }
  function abandon(message) {
    generation++;
    const old = session; session = ""; release(old);
    notice(message, true); controls();
  }
  function controls() {
    const state = data?.state;
    const motorsReady = state && [state.motor_a, state.motor_b].every(name => {
      const motor = data.motors[name];
      return motor?.connected && motor.ready && motor.feedback_fresh;
    });
    const idle = !disconnected && data?.connected && data.configuration_fresh && motorsReady
      && state?.stop_confirmed && state.commands_ready && state.paired && state.pair_age <= data.configuration.feedback_age
      && state.pair_skew <= data.configuration.pair_skew
      && [0, 5].includes(state.phase);
    $("start").disabled = !idle || !initialized || busy || stopping || document.hidden;
    $("edit-fields").disabled = !idle || busy || stopping || document.hidden;
    $("limit-fields").disabled = !$("motion_limits_enabled").checked;
    $("stop").disabled = false;
    $("dirty").textContent = dirty ? "有未应用修改；开始时提交全部参数" : "参数来自控制器，运行期间不可修改";
    $("availability").textContent = !data?.connected || disconnected ? "等待双边控制器连接。" :
      idle ? "可编辑后直接开始。不会自动使能或恢复旧目标。" : "等待两台新鲜失能反馈，或等待停止确认。";
    $("availability").classList.toggle("blocked", !idle);
  }
  function readSettings() {
    const value = {direction: Number($("direction").value), motion_limits_enabled: $("motion_limits_enabled").checked};
    for (const name of numbers) {
      if (!$(`${name}`).value.trim() || !Number.isFinite(Number($(name).value))) throw new Error(`${name} 必须是有限数字`);
      value[name] = Number($(name).value);
    }
    if (value.coupling_damping + value.local_damping > 1) throw new Error("Dc+B 必须不超过驱动 Kd 上限 1");
    return value;
  }
  function motor(role, name) {
    const value = data.motors[name], fresh = data.connected && value?.connected && value.feedback_fresh;
    $(role + "-name").textContent = name || "—";
    $(role + "-status").textContent = !fresh ? "反馈不可用" : `${value.simulated ? "SIM" : "HW"} · ${value.status === 0 ? "失能" : value.status === 1 ? "使能" : "故障"}`;
    $(role + "-status").className = `badge${fresh && value.status === 1 ? " active" : !fresh || value.error ? " fault" : ""}`;
    for (const field of ["position", "velocity", "torque"]) $(role + "-" + field).textContent = fresh ? format(value[field]) : "—";
    $(role + "-temperature").textContent = fresh ? `${value.mos_temperature} / ${value.rotor_temperature} °C` : "—";
    const warm = fresh && Math.max(value.mos_temperature, value.rotor_temperature) >= data.configuration?.warning_temperature;
    $(role + "-temperature").classList.toggle("warning", Boolean(warm));
    $(role + "-temperature").title = warm ? "温度达到预警阈值；继续观察反馈和停机保护" : "";
    $(role + "-age").textContent = fresh ? format(value.feedback_age * 1000, 1) : "—";
    $(role + "-rate").textContent = fresh ? format(value.feedback_rate, 1) : "—";
  }
  function plot(id, key, colors) {
    const canvas = $(id), rect = canvas.getBoundingClientRect(), ratio = devicePixelRatio || 1;
    if (!rect.width) return;
    canvas.width = Math.round(rect.width * ratio); canvas.height = 190 * ratio;
    const ctx = canvas.getContext("2d"); ctx.scale(ratio, ratio);
    const values = history.flatMap(row => row[key]).filter(Number.isFinite);
    let low = values.length ? Math.min(...values) : -0.1, high = values.length ? Math.max(...values) : 0.1;
    const margin = Math.max(0.01, (high - low) * 0.15); low -= margin; high += margin;
    const left = 58, right = rect.width - 8;
    ctx.font = "10px system-ui";
    for (let i = 0; i < 4; i++) {
      const y = 15 + i * 50;
      ctx.strokeStyle = "#30435b"; ctx.beginPath(); ctx.moveTo(left, y); ctx.lineTo(right, y); ctx.stroke();
      ctx.fillStyle = "#9eb1c8"; ctx.fillText(format(high - (high - low) * i / 3, 2), 2, y + 4);
    }
    if (history.length < 2) return;
    const start = history[0].time, span = Math.max(1, history.at(-1).time - start);
    colors.forEach((color, series) => {
      ctx.strokeStyle = color; ctx.lineWidth = 1.7; ctx.setLineDash(key === "torque" && series >= 2 ? [5, 4] : []);
      ctx.beginPath();
      history.forEach((row, index) => {
        const x = left + (right - left) * (row.time - start) / span;
        const y = 15 + 150 * (high - row[key][series]) / (high - low);
        if (index) ctx.lineTo(x, y); else ctx.moveTo(x, y);
      });
      ctx.stroke();
    });
  }
  function draw() {
    plot("position-chart", "position", ["#51d3d3", "#b298ff", "#f6bf65"]);
    plot("torque-chart", "torque", ["#51d3d3", "#b298ff", "#51d3d3", "#b298ff"]);
  }
  function render(snapshot) {
    data = snapshot; disconnected = !data.connected;
    if (session && !data.has_control) {session = ""; notice("操作会话已结束；只恢复监控，不自动开始。", true);}
    const config = data.configuration, state = data.state;
    if (config && (!dirty || !initialized)) {
      for (const name of numbers) $(name).value = String(config[name]);
      $("direction").value = String(config.direction);
      $("motion_limits_enabled").checked = config.motion_limits_enabled;
      initialized = true;
    }
    $("phase").textContent = data.connected && state ? phases[state.phase] : "连接中断";
    $("phase").className = `badge${state?.phase === 5 || !data.connected ? " fault" : state?.phase === 3 ? " active" : ""}`;
    $("quality").textContent = data.connected && state?.phase === 3 ? qualities[state.quality] : "—";
    $("quality").className = `badge${state?.quality ? " degraded" : ""}`;
    if (state) {
      motor("a", state.motor_a); motor("b", state.motor_b);
      $("position-error").textContent = state.paired ? format(state.position_error) : "—";
      $("relative-velocity").textContent = state.paired ? format(state.relative_velocity) : "—";
      $("gain-scale").textContent = format(state.gain_scale, 3);
      $("gains").textContent = `${format(state.decoded_kp, 3)} / ${format(state.decoded_kd, 3)}`;
      $("pair-timing").textContent = state.paired ? `${format(state.pair_age * 1000, 2)} / ${format(state.pair_skew * 1000, 2)}${state.pair_waiting ? " · 等待新配对" : ""}` : "—";
      $("recovery").textContent = `${format(state.recovery_scale * 100, 0)}% / ${state.recovery_pairs}`;
      $("pair-count").textContent = `${state.pair_count} / ${state.degradation_count}`;
      $("command-count").textContent = String(state.command_count);
      $("a-estimate").textContent = format(state.estimated_torque_a);
      $("b-estimate").textContent = format(state.estimated_torque_b);
      $("controller-message").textContent = state.message;
      $("controller-message").classList.toggle("error", state.phase === 5);
      $("targets").textContent = JSON.stringify({A: state.target_a, B: state.target_b}, null, 2);
      const key = `${state.motor_a}:${state.motor_b}:${state.origin_a}:${state.origin_b}:${state.direction}`;
      if (key !== originKey) {history = []; originKey = key;}
      const a = data.motors[state.motor_a], b = data.motors[state.motor_b];
      if (data.connected && a?.connected && b?.connected) {
        history.push({time: Date.now(), position: [a.position - state.origin_a, state.direction * (b.position - state.origin_b), state.position_error],
          torque: [a.torque, b.torque, state.estimated_torque_a, state.estimated_torque_b]});
        while (history.length > 150 || (history.length && Date.now() - history[0].time > 15000)) history.shift();
      }
    }
    $("events").replaceChildren(...data.events.slice(-20).reverse().map(event => {
      const item = document.createElement("li"); item.textContent = `${new Date(event.time * 1000).toLocaleTimeString()} ${event.message}`; return item;
    }));
    $("rate").textContent = config ? `控制器 ${config.rate} Hz · 网页约 5 Hz，非逐帧记录` : "等待配置";
    controls(); draw();
  }
  async function poll() {
    const token = session, version = generation;
    try {
      const snapshot = await api("state");
      if (token === session && version === generation) {lastSeen = Date.now(); render(snapshot);}
    } catch (error) {
      disconnected = true; abandon(`监控连接失败：${error.message}，请确认设备停止状态`);
      if (data) render({...data, connected: false});
    } finally {setTimeout(poll, 200);}
  }
  $("settings").addEventListener("submit", async event => {
    event.preventDefault();
    if ($("start").disabled || !$("settings").reportValidity()) return;
    const version = generation;
    busy = true; controls();
    try {
      const settings = readSettings();
      if (!session) {
        const claimed = await api("session", {});
        if (version !== generation || document.hidden || disconnected) {release(claimed.session); throw new Error("开始已取消");}
        session = claimed.session;
      }
      if (version !== generation || document.hidden || disconnected) throw new Error("开始已取消");
      const response = await api("start", {settings, confirm_hardware: Object.values(data.motors).some(motor => !motor.simulated)});
      if (version === generation) {dirty = false; notice(response.message);}
    } catch (error) {notice(error.message, true);}
    finally {busy = false; controls();}
  });
  async function stop() {
    generation++; session = ""; stopping = true; controls();
    try {notice((await api("stop", {})).message);}
    catch (error) {notice(`STOP 未确认：${error.message}；必要时使用物理急停`, true);}
    finally {stopping = false; controls();}
  }
  $("stop").addEventListener("click", stop);
  for (const name of ["direction", "motion_limits_enabled", ...numbers]) $(name).addEventListener("input", () => {dirty = true; controls();});
  document.addEventListener("keydown", event => {
    if (event.key === "Escape" || (event.code === "Space" && !["INPUT", "SELECT", "BUTTON", "TEXTAREA"].includes(event.target.tagName))) {
      event.preventDefault(); stop();
    }
  });
  document.addEventListener("visibilitychange", () => {if (document.hidden) abandon("页面隐藏；已请求释放并停止"); else controls();});
  window.addEventListener("pagehide", () => abandon("页面退出；已请求停止"));
  window.addEventListener("resize", draw);
  setInterval(async () => {
    if (lastSeen && Date.now() - lastSeen > 1000 && !disconnected) {
      disconnected = true; abandon("监控超时；已释放控制");
      if (data) render({...data, connected: false});
    }
    if (!session || document.hidden || heartbeatPending) return;
    heartbeatPending = true;
    try {await api("heartbeat", {});}
    catch (error) {abandon(`网页心跳失败：${error.message}`);}
    finally {heartbeatPending = false;}
  }, 250);
  controls(); poll();
})();
