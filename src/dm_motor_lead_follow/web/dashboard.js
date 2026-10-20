"use strict";
(() => {
  const $ = id => document.getElementById(id);
  const numbers = ["kp", "kd", "target_speed", "max_error", "max_travel", "max_speed", "max_torque", "max_temperature"];
  const fields = ["swap_roles", "direction", "motion_limits_enabled", ...numbers];
  const phases = ["待机 / 已停止", "获取控制租约", "中性目标使能中", "正在跟随", "停止中 · 等待确认", "故障停机"];
  let session = "", data = null, dirty = false, initialized = false, busy = false;
  let heartbeatPending = false, lastSeen = 0, disconnected = true, history = [], mapping = "";
  let startGeneration = 0;

  function notice(text, error = false) {
    $("notice").textContent = text;
    $("notice").classList.toggle("error", error);
  }
  async function api(path, body) {
    const controller = new AbortController();
    const timer = setTimeout(() => controller.abort(), 4000);
    try {
      const headers = {"X-Control-Session": session};
      const options = {headers, signal: controller.signal, cache: "no-store"};
      if (body !== undefined) {
        options.method = "POST";
        headers["Content-Type"] = "application/json";
        options.body = JSON.stringify(body);
      }
      const response = await fetch("/api/" + path, options);
      const result = await response.json();
      if (!response.ok) throw new Error(result.error || `HTTP ${response.status}`);
      return result;
    } finally {
      clearTimeout(timer);
    }
  }
  function abandon(message) {
    startGeneration += 1;
    if (session) {
      const token = session;
      session = "";
      fetch("/api/release", {method: "POST", headers: {"Content-Type": "application/json", "X-Control-Session": token},
        body: "{}", keepalive: true}).catch(error => notice(`停止请求未送达，心跳保护将接管：${error.message}`, true));
    }
    notice(message, true);
    controls();
  }
  const format = (value, digits = 4) => Number.isFinite(value) ? value.toFixed(digits) : "—";
  function controls() {
    const online = !disconnected && data?.connected;
    const state = data?.state;
    const motorsReady = state && [state.leader, state.follower].every(name => {
      const motor = data?.motors?.[name];
      return motor?.connected && motor.ready && motor.feedback_fresh;
    });
    const idle = online && data?.configuration_fresh && motorsReady
      && state?.stop_confirmed && [0, 5].includes(state.phase);
    $("edit-fields").disabled = !idle || busy || document.hidden;
    $("limit-fields").disabled = !$("motion_limits_enabled").checked;
    $("start").disabled = !idle || busy || !initialized || document.hidden;
    $("stop").disabled = false;
    $("dirty").textContent = dirty ? "有未应用修改；开始时会应用当前全部配置" : "配置来自控制器；运行期间不可修改";
    let availability;
    if (!online) availability = "等待主从控制器连接，当前禁止开始。";
    else if (!data.configuration_fresh) availability = "等待控制器配置读取完成。";
    else if (state && ![0, 5].includes(state.phase)) availability = "跟随正在运行或停止中；确认两台失能后才能修改配置。";
    else if (!idle) availability = "等待两台电机初始化完成、反馈新鲜且确认失能。";
    else availability = "可直接编辑参数并点击右上角开始，配置会一并提交。单机面板和 Topic 不被排斥，请自行协调发送源。";
    $("availability").textContent = availability;
    $("availability").classList.toggle("blocked", !idle);
  }
  function readSettings() {
    const settings = {swap_roles: $("swap_roles").value === "true", direction: Number($("direction").value),
      motion_limits_enabled: $("motion_limits_enabled").checked};
    for (const name of numbers) {
      if ($(name).value.trim() === "") throw new Error(`${name} 不能为空`);
      const value = Number($(name).value);
      if (!Number.isFinite(value)) throw new Error(`${name} 必须是有限数字`);
      settings[name] = value;
    }
    return settings;
  }
  function showMotor(role, name) {
    const motor = data.motors[name];
    const fresh = !disconnected && data.connected && motor?.connected && motor.feedback_fresh;
    $(role + "-name").textContent = name || "—";
    $(role + "-status").textContent = !fresh ? "反馈不可用" : !motor.ready ? "初始化中"
      : motor.status === 0 ? "失能" : motor.status === 1 ? "使能" : `状态 ${motor.status}`;
    $(role + "-status").className = "badge" + (fresh && motor.status === 1 ? " active" : !fresh || motor.error ? " fault" : "");
    for (const field of ["position", "velocity", "torque"]) $(role + "-" + field).textContent = fresh ? format(motor[field]) : "—";
    $(role + "-temperature").textContent = fresh ? `${motor.mos_temperature} / ${motor.rotor_temperature} °C` : "—";
    $(role + "-rate").textContent = fresh ? format(motor.feedback_rate, 1) : "—";
    $(role + "-age").textContent = fresh ? format(motor.feedback_age * 1000, 1) : "—";
    $(role + "-owner").textContent = fresh ? motor.active_source || "无" : "—";
  }
  function draw() {
    const canvas = $("chart"), rect = canvas.getBoundingClientRect(), ratio = window.devicePixelRatio || 1;
    if (rect.width <= 0) return;
    canvas.width = Math.round(rect.width * ratio);
    canvas.height = Math.round(240 * ratio);
    const ctx = canvas.getContext("2d");
    ctx.scale(ratio, ratio);
    const width = rect.width, height = 240, left = 62, right = width - 12;
    const values = history.flatMap(row => row.values).filter(Number.isFinite);
    let low = values.length ? Math.min(...values) : -0.1, high = values.length ? Math.max(...values) : 0.1;
    const margin = Math.max(0.005, (high - low) * 0.15);
    low -= margin; high += margin;
    ctx.font = "10px system-ui";
    for (let index = 0; index <= 4; ++index) {
      const y = 16 + index * 48;
      ctx.strokeStyle = "#29364c"; ctx.beginPath(); ctx.moveTo(left, y); ctx.lineTo(right, y); ctx.stroke();
      ctx.fillStyle = "#9dadc6"; ctx.fillText((high - index * (high - low) / 4).toFixed(3), 4, y + 4);
    }
    if (history.length < 2) {
      ctx.fillStyle = "#9dadc6"; ctx.fillText("等待位置数据", left + 12, 100); return;
    }
    const start = history[0].time, span = Math.max(1, history.at(-1).time - start);
    ["#51d3d3", "#b298ff", "#f6bf65"].forEach((color, series) => {
      ctx.strokeStyle = color; ctx.lineWidth = 1.7; ctx.beginPath();
      history.forEach((row, index) => {
        const x = left + (row.time - start) / span * (right - left);
        const y = 16 + (high - row.values[series]) / (high - low) * 192;
        if (index === 0) ctx.moveTo(x, y); else ctx.lineTo(x, y);
      });
      ctx.stroke();
    });
    ctx.fillStyle = "#9dadc6"; ctx.fillText(`最近 ${(span / 1000).toFixed(1)} 秒 · 网页采样，非逐帧记录`, left, 232);
  }
  function render(snapshot) {
    const previousPhase = data?.state?.phase;
    data = snapshot;
    disconnected = !data.connected;
    const state = data.state;
    if (session && !data.has_control) {
      session = "";
      notice("网页操作会话已结束；监控继续，不会自动重新开始", true);
    }
    const configuration = data.configuration;
    if (configuration && (!dirty || !initialized)) {
      for (const name of fields) {
        if (name === "motion_limits_enabled") $(name).checked = configuration[name];
        else $(name).value = String(configuration[name]);
      }
      $("swap_roles").options[0].textContent = `${configuration.leader} → ${configuration.follower}`;
      $("swap_roles").options[1].textContent = `${configuration.follower} → ${configuration.leader}`;
      initialized = true;
    }
    $("controller").textContent = data.controller;
    $("controller-message").textContent = state?.message || "等待控制器反馈";
    $("phase").textContent = disconnected ? "控制器离线" : phases[state?.phase] || "未知状态";
    $("phase").className = "badge" + (disconnected || state?.phase === 5 ? " fault" : state?.phase === 3 ? " active" : "");
    const motors = Object.values(data.motors);
    $("environment").textContent = !motors.length ? "等待设备" : motors.every(m => m.simulated) ? "仿真" : "真机";
    $("environment").className = "badge" + (motors.some(m => !m.simulated) ? " fault" : "");
    $("rate").textContent = configuration ? `控制设定 ${configuration.rate} Hz · 面板 5 Hz` : "—";
    if (state) {
      showMotor("leader", state.leader); showMotor("follower", state.follower);
      const mappingNow = `${state.leader}:${state.follower}:${state.direction}:${state.leader_origin}:${state.follower_origin}`;
      if (mappingNow !== mapping) {history = []; mapping = mappingNow;}
      const current = !disconnected && state.command_count > 0
        && data.motors[state.follower]?.connected && data.motors[state.follower]?.feedback_fresh;
      $("ideal").textContent = current ? format(state.ideal_position) : "—";
      $("target").textContent = current ? format(state.target_position) : "—";
      $("error").textContent = current ? format(state.following_error) : "—";
      $("origins").textContent = `启动偏置：主 ${format(state.leader_origin)} rad / 从 ${format(state.follower_origin)} rad · ${state.direction === 1 ? "同向" : "反向"} · ${state.operator_guarded ? "网页心跳保护生效" : "未由网页启动"}`;
      if (current && [state.ideal_position, data.motors[state.follower].position, state.target_position].every(Number.isFinite)) {
        history.push({time: Date.now(), values: [state.ideal_position, data.motors[state.follower].position, state.target_position]});
        if (history.length > 200) history.shift();
      }
      if ([4, 5].includes(state.phase)) notice(state.message, true);
      else if (!disconnected && state.phase === 0 && previousPhase === 4) notice("两台已确认失能，可重新配置或开始。");
      else if (!disconnected && state.phase === 3 && previousPhase !== 3) notice("主从跟随已确认。请保持控制页面可见；可随时点击停止。");
    }
    $("events").replaceChildren(...[...data.events].reverse().map(event => {
      const item = document.createElement("li"), time = document.createElement("time");
      time.textContent = new Date(event.time * 1000).toLocaleTimeString();
      item.append(time, document.createTextNode(event.message)); return item;
    }));
    $("connection").textContent = disconnected ? "状态过期，禁止开始" : "状态实时更新 · 浏览器心跳 250 ms";
    if (disconnected && session) abandon("控制器状态断开，已释放网页控制");
    controls(); draw();
  }
  async function poll() {
    const token = session;
    try {
      const snapshot = await api("state");
      if (token === session) {
        lastSeen = Date.now();
        const first = !data;
        render(snapshot);
        if (first && snapshot.connected) notice("监控已连接。编辑参数后点击右上角开始即可；不会自动使能。");
      }
    } catch (error) {
      disconnected = true;
      abandon(`连接失败：${error.message}；请确认设备最终停止状态`);
      if (data) render({...data, connected: false});
    } finally {
      setTimeout(poll, 200);
    }
  }
  async function action(callback) {
    busy = true; controls();
    try {await callback();}
    catch (error) {notice(error.message, true);}
    finally {busy = false; controls();}
  }
  $("settings").addEventListener("submit", event => {
    event.preventDefault();
    if ($("start").disabled || !$("settings").reportValidity()) return;
    action(async () => {
      const settings = readSettings();
      const hardware = Object.values(data.motors).some(motor => !motor.simulated);
      const generation = startGeneration;
      if (!session) {
        const result = await api("session", {});
        if (generation !== startGeneration || document.hidden || disconnected) {
          fetch("/api/release", {method: "POST", headers: {"Content-Type": "application/json", "X-Control-Session": result.session},
            body: "{}", keepalive: true}).catch(error => notice(`后台会话释放未确认：${error.message}`, true));
          throw new Error("开始已取消：发生停止、离开页面或连接变化。");
        }
        session = result.session;
        data.has_control = true;
      }
      if (generation !== startGeneration || disconnected || document.hidden) {
        throw new Error("开始已取消：发生停止、离开页面或连接变化。");
      }
      const result = await api("start", {settings, confirm_hardware: hardware});
      dirty = false; notice(result.message);
    });
  });
  async function stop() {
    startGeneration += 1;
    try {const result = await api("stop", {}); notice(result.message);}
    catch (error) {notice(`STOP 未确认：${error.message}；必要时物理断电`, true);}
  }
  $("stop").addEventListener("click", stop);
  for (const name of fields) $(name).addEventListener("input", () => {dirty = true; controls();});
  document.addEventListener("keydown", event => {
    if (event.code === "Escape" || (event.code === "Space" && !["INPUT", "SELECT", "TEXTAREA", "BUTTON"].includes(event.target.tagName))) {
      event.preventDefault(); stop();
    }
  });
  document.addEventListener("visibilitychange", () => {
    if (document.hidden) abandon("控制页面已隐藏，已请求释放并停止");
    else controls();
  });
  window.addEventListener("pagehide", () => abandon("控制页面已关闭，已请求停止"));
  window.addEventListener("resize", draw);
  setInterval(async () => {
    if (lastSeen && Date.now() - lastSeen > 1000 && !disconnected) {
      disconnected = true; abandon("状态更新超时，已释放控制");
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
