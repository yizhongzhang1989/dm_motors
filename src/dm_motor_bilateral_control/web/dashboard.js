"use strict";
(() => {
  const $ = id => document.getElementById(id);
  const numbers = ["stiffness", "coupling_damping", "local_damping", "max_torque", "max_feedback_torque",
    "max_travel", "warning_temperature", "max_temperature", "feedback_age", "pair_skew", "feedback_stop", "recovery_time",
    "friction_compensation_fraction", "friction_torque_limit"];
  const phases = ["待机 / 已停止", "获取会话与核验", "中性使能中", "双边耦合中", "停止中 / 等待确认", "故障停机"];
  const qualities = ["正常配对", "降级 / 零增益", "恢复爬升"];
  let session = "", data = null, dirty = false, initialized = false, busy = false;
  let stopping = false;
  let disconnected = true, heartbeatPending = false, generation = 0, lastSeen = 0, history = [], originKey = "";
  let historyGap = true;
  const format = (value, digits = 4) => Number.isFinite(value) ? value.toFixed(digits) : "—";
  function nativeGains(config, mode) {
    if (!["native", "shared_reference", "differential_torque"].includes(mode)) throw new Error("控制模式尚未读取");
    const multiplier = mode === "native" ? 1 : 2;
    return {kp: mode === "differential_torque" ? 0 : multiplier * config.stiffness,
      kd: multiplier * config.coupling_damping + config.local_damping};
  }
  function gainCapability() {
    const state = data?.state, mode = state?.control_mode;
    if (!data?.connected || !["native", "shared_reference", "differential_torque"].includes(mode)) return null;
    const motors = [state.motor_a, state.motor_b].map(name => data.motors[name]);
    if (!motors.every(motor => motor?.connected && motor.ready && motor.feedback_fresh &&
      [motor.kp_limit, motor.kd_limit].every(value => Number.isFinite(value) && value > 0))) return null;
    return {mode, multiplier: mode === "native" ? 1 : 2,
      kp: Math.min(500, ...motors.map(motor => motor.kp_limit)),
      kd: Math.min(1, ...motors.map(motor => motor.kd_limit))};
  }
  function stiffnessLimit(capability) {
    return capability.mode === "differential_torque" ? 500 : capability.kp / capability.multiplier;
  }
  function velocityBandwidthStatus(state, connected) {
    if (!connected) return {text: "速度带宽状态过期", warning: false};
    const required = state?.required_velocity_bandwidth;
    const expected = Math.fround(required);
    if (!Number.isFinite(required) || required < 0 || required >= 500 ||
      (required > 0 && !(expected > 0 && expected < 500))) {
      return {text: "速度带宽状态未读取", warning: false};
    }
    if (required === 0) return {text: "速度带宽未设置核验要求", warning: false};
    const label = `速度带宽 ${required} Hz`;
    const verified = state.velocity_bandwidth_verified === true &&
      state.motor_a_velocity_bandwidth === expected && state.motor_b_velocity_bandwidth === expected;
    if (verified && [1, 2, 3].includes(state.phase)) return {text: `${label} 已核验`, warning: false};
    return state.phase === 3 ? {text: `${label} 核验不通过`, warning: true} :
      {text: `${label} · 开始前核验`, warning: false};
  }
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
    const capability = gainCapability();
    const shared = state?.control_mode === "shared_reference";
    const differential = state?.control_mode === "differential_torque";
    const unilateral = data?.configuration_fresh && data.configuration?.coupling_mode === "a_leads_b";
    const identities = state ? [state.motor_a, state.motor_b].map(name => data.motors[name]?.simulated) : [];
    const simulatedPair = identities.length === 2 && identities.every(value => value === true);
    const realPair = identities.length === 2 && identities.every(value => value === false);
    const hardwareTrial = differential && realPair && data?.configuration_fresh &&
      data.configuration?.allow_differential_hardware === true;
    $("control-mode").textContent = !data?.connected ? "等待模式" : shared ? "共享参考" :
      differential ? (unilateral ? "单向 A → B · 差模力矩" : hardwareTrial ? "差模力矩 · 真机试验模式" : "差模力矩") :
      state?.control_mode === "native" ? "相对参考" : "模式未读取";
    const bandwidth = velocityBandwidthStatus(state, !disconnected && data?.connected);
    $("velocity-bandwidth").textContent = bandwidth.text;
    $("velocity-bandwidth").classList.toggle("warning", bandwidth.warning);
    $("gain-help").textContent = !capability ? "等待读取控制模式和两轴增益上限。" : differential ?
      "差模力矩：Kc 控制相对弹性（主机弹簧）；原生 Kp=0，Kd=2Dc+B。" : shared ?
      "共享参考：原生 Kp=2Kc，Kd=2Dc+B；共同位置自由，不锁定中心。" :
      "相对参考：原生 Kp=Kc，Kd=Dc+B。Dc 抑制相对运动，B 增加每轴拖动阻尼。";
    if (capability) {
      $("stiffness").max = String(stiffnessLimit(capability));
      const local = Number($("local_damping").value);
      $("coupling_damping").max = String(Math.max(0, (capability.kd - (Number.isFinite(local) ? local : 0)) / capability.multiplier));
      $("local_damping").max = String(capability.kd);
    }
    const motorsReady = state && [state.motor_a, state.motor_b].every(name => {
      const motor = data.motors[name];
      return motor?.connected && motor.ready && motor.feedback_fresh;
    });
    const idle = !disconnected && data?.connected && data.configuration_fresh && motorsReady && capability
      && state?.stop_confirmed && [0, 5].includes(state.phase);
    const paired = idle && state.commands_ready && state.paired && state.pair_age <= data.configuration.feedback_age
      && state.pair_skew <= data.configuration.pair_skew;
    const modeReady = !differential || simulatedPair || hardwareTrial;
    const modelsReady = state?.compensation_ready_a && state?.compensation_ready_b;
    const compensationReady = !$("friction_compensation_enabled").checked || modelsReady;
    const locked = !initialized || busy || stopping || document.hidden;
    $("start").disabled = !paired || !compensationReady || !modeReady || locked;
    $("apply").disabled = !idle || locked;
    $("edit-fields").disabled = !idle || locked;
    $("limit-fields").disabled = !$("motion_limits_enabled").checked;
    $("stop").disabled = false;
    $("start").textContent = hardwareTrial ? "开始 · 真机试验" :
      identities.some(value => value === false) ? "开始 · 真机" : "开始";
    $("dirty").textContent = dirty ? "有未应用修改；可单独应用，或开始时一并提交" : "参数已同步；运行期间不可修改";
    $("availability").textContent = !data?.connected || disconnected ? "等待双边控制器连接。" :
      !capability ? "等待读取控制模式和两轴增益上限。" :
      !idle ? "等待两台新鲜失能反馈，或等待停止确认。" :
      !modeReady ? "差模力矩需要两台仿真电机，或启动时已授权的两台真机；不支持混合连接。" :
      !paired ? "可应用设置；开始还需命令通道就绪及新鲜配对。" :
      !compensationReady ? "可应用设置；启用补偿时需两轴模型均可用。" :
      hardwareTrial ? "真机试验模式已由启动配置授权；可应用设置，或点击开始耦合。" :
      "可应用设置，或点击开始耦合。";
    $("availability").classList.toggle("blocked", !paired || !compensationReady || !modeReady);
    $("compensation-summary").textContent = !data?.connected ? "等待模型状态。" :
      !$("friction_compensation_enabled").checked ? "摩擦模型补偿关闭。" :
      !modelsReady ? "模型未就绪，请查看两轴说明；应用设置不会使能。" :
      Number($("friction_compensation_fraction").value) === 0 ? "补偿比例为 0，不施加摩擦前馈。" :
      state.compensation_active ? "两轴补偿运行中；模型摩擦上限始终生效。" : "两轴模型已就绪，开始后随耦合输出。";
  }
  function readSettings() {
    const value = {direction: Number($("direction").value), motion_limits_enabled: $("motion_limits_enabled").checked,
      friction_compensation_enabled: $("friction_compensation_enabled").checked};
    for (const name of numbers) {
      if (!$(`${name}`).value.trim() || !Number.isFinite(Number($(name).value))) throw new Error(`${name} 必须是有限数字`);
      value[name] = Number($(name).value);
    }
    const capability = gainCapability();
    if (!capability) throw new Error("控制模式或驱动增益上限尚未读取");
    const native = nativeGains(value, capability.mode);
    if (value.stiffness > stiffnessLimit(capability)) throw new Error(`Kc 必须在 0..${stiffnessLimit(capability)} 范围内`);
    if ([value.stiffness, value.coupling_damping, value.local_damping].some(gain => gain < 0) ||
      native.kp > capability.kp || native.kd > capability.kd) {
      throw new Error(`映射后的原生 Kp/Kd 必须不超过两轴上限 ${capability.kp} / ${capability.kd}`);
    }
    return value;
  }
  function motor(role, name) {
    const value = data.motors[name], fresh = data.connected && value?.connected && value.feedback_fresh;
    const faults = value ? [value.host_fault ? "主机故障" : "", value.device_fault ? "设备故障" : ""].filter(Boolean) : [];
    if (value?.error && !faults.length) faults.push("驱动异常");
    const status = faults.length ? faults.join(" / ") : value?.status === 0 ? "失能" : value?.status === 1 ? "使能" : "设备故障";
    $(role + "-name").textContent = name || "—";
    $(role + "-status").textContent = !fresh ? "反馈不可用" : `${value.simulated ? "SIM" : "HW"} · ${status}`;
    $(role + "-status").className = `badge${!fresh || faults.length || value.status > 1 ? " fault" : value.status === 1 ? " active" : ""}`;
    $(role + "-fault").hidden = !faults.length && !value?.error;
    $(role + "-fault").textContent = `${!fresh ? "最近状态：" : ""}${faults.join(" / ")}${value?.error ? `：${value.error}` : ""}`;
    for (const field of ["position", "velocity", "torque"]) $(role + "-" + field).textContent = fresh ? format(value[field]) : "—";
    $(role + "-temperature").textContent = fresh ? `${value.mos_temperature} / ${value.rotor_temperature} °C` : "—";
    const warm = fresh && Math.max(value.mos_temperature, value.rotor_temperature) >= data.configuration?.warning_temperature;
    $(role + "-temperature").classList.toggle("warning", Boolean(warm));
    $(role + "-temperature").title = warm ? "温度达到预警阈值；继续观察反馈和停机保护" : "";
    $(role + "-age").textContent = fresh ? format(value.feedback_age * 1000, 1) : "—";
    $(role + "-rate").textContent = fresh ? format(value.feedback_rate, 1) : "—";
    $(role + "-demand").textContent = fresh ? format(data.state[`raw_torque_${role}`]) : "—";
    $(role + "-estimate").textContent = fresh ? format(data.state[`estimated_torque_${role}`]) : "—";
    $(role + "-friction").textContent = fresh ? format(data.state[`friction_torque_${role}`]) : "—";
    $(role + "-quantization").textContent = fresh ? format(data.state[`quantization_torque_${role}`]) : "—";
    $(role + "-total-feedforward").textContent = fresh ? format(data.state[`target_${role}`]?.torque) : "—";
  }
  function outputStatus(state, config) {
    const connected = data.connected;
    const configured = connected && data.configuration_fresh && config;
    $("gains").textContent = connected ? `${format(state.decoded_kp, 3)} / ${format(state.decoded_kd, 3)}` : "—";
    const modeKnown = ["native", "shared_reference", "differential_torque"].includes(state.control_mode);
    const requested = configured && modeKnown ? nativeGains(config, state.control_mode) : null;
    $("requested-gains").textContent = requested ?
      `有效 Kc/Dc ${format(config.stiffness, 3)} / ${format(config.coupling_damping, 3)}；原生目标 ${format(requested.kp, 3)} / ${format(requested.kd, 3)}` : "配置或模式未读取";
    $("gain-scale").textContent = connected ? `${format(state.gain_scale * 100, 1)}%` : "—";
    $("budget-mode").textContent = !configured ? "未读取" : config.max_torque === 0 ?
      "不额外限矩" : `${format(config.max_torque, 3)} Nm`;
    const recovering = Number.isFinite(state.recovery_scale) && state.recovery_scale < 0.999;
    const budgetScale = state.recovery_scale > 0 ? state.gain_scale / state.recovery_scale : null;
    const limited = state.application_torque_limited === true && Number.isFinite(budgetScale) && budgetScale < 0.999;
    const saturated = ["a", "b"].filter(role => state[`protocol_saturated_${role}`] === true);
    const differential = state.control_mode === "differential_torque";
    const couplingLimited = differential && saturated.length > 0;
    $("gain-reason").textContent = !connected ? "状态过期" : state.phase !== 3 ? "当前未输出耦合请求" :
      state.recovery_scale <= 0 ? "反馈降级，耦合输出暂停" : limited ?
      `应用预算缩放至 ${format(budgetScale * 100, 1)}%${recovering ? ` · 恢复 ${format(state.recovery_scale * 100, 1)}%` : ""}` :
      recovering ? `反馈恢复至 ${format(state.recovery_scale * 100, 1)}%` : couplingLimited ?
      "原生增益未缩小；耦合需求受协议上限限制" : "未缩小设定增益";
    $("gain-reason").classList.toggle("warning", connected && state.phase === 3 && (limited || couplingLimited || recovering));
    const saturationKnown = ["a", "b"].every(role => typeof state[`protocol_saturated_${role}`] === "boolean");
    $("protocol-status").textContent = !connected ? "状态过期" : state.phase !== 3 ? "当前未输出" :
      !saturationKnown ? "协议需求状态未读取" : saturated.length ?
      `${saturated.map(role => role.toUpperCase()).join(" / ")} ${differential ? "耦合需求受协议上限限制" : "需求超协议量程"}` :
      differential ? "耦合需求未触发协议上限" : "需求未超协议量程";
    $("protocol-status").className = `badge${connected && state.phase === 3 && saturated.length ? " degraded" : ""}`;
  }
  function driverLimits(role, state, motor) {
    const connected = data.connected && motor?.connected;
    const readable = connected && motor.ready;
    const configuration = state[`driver_configuration_${role}`];
    const known = data.connected && state[`driver_configuration_known_${role}`] === true && configuration;
    const missing = data.connected ? "未读取" : "状态过期";
    const valid = value => Number.isFinite(value) && value >= 0;
    const protocol = readable && valid(motor.tmax) && motor.tmax > 0 ? motor.tmax : null;
    const feedforward = readable && valid(motor.torque_limit) ? motor.torque_limit || protocol : null;
    $(role + "-protocol-limit").textContent = protocol === null ? missing : `±${format(protocol, 3)}`;
    $(role + "-feedforward-limit").textContent = feedforward === null ? missing : `±${format(feedforward, 3)}`;
    $(role + "-gain-limits").textContent = readable && valid(motor.kp_limit) && valid(motor.kd_limit) ?
      `${format(motor.kp_limit, 3)} / ${format(motor.kd_limit, 3)}` : missing;
    $(role + "-driver-status").textContent = !known ? missing :
      configuration.host_protection_enabled ? "主机保护开启" : "主机保护关闭";
    const threshold = value => !known || !valid(value) ? missing : value === 0 ? "未设阈值" :
      `${format(value, 3)}${configuration.host_protection_enabled ? "" : "（未启用）"}`;
    $(role + "-estimated-stop").textContent = threshold(configuration?.estimated_torque_limit);
    $(role + "-measured-stop").textContent = threshold(configuration?.measured_torque_limit);
    $(role + "-control-rate").textContent = known && valid(configuration.control_rate) ? format(configuration.control_rate, 1) : missing;
    $(role + "-watchdog").textContent = known && valid(configuration.watchdog_timeout) ? format(configuration.watchdog_timeout * 1000, 1) : missing;
    const saturated = state[`protocol_saturated_${role}`];
    $(role + "-output-status").textContent = !connected ? "输出状态过期。" : state.phase !== 3 ? "当前无耦合输出。" :
      typeof saturated !== "boolean" ? "协议需求状态未读取。" : state.control_mode === "differential_torque" ?
      saturated ? "耦合需求受前馈或协议上限限制；请结合反馈力矩判断实际输出。" : "耦合需求未触发前馈或协议上限。" : saturated ?
      `估算需求超出${protocol === null ? "" : ` ±${format(protocol, 3)} Nm `}协议量程；请结合反馈力矩判断实际输出。` : "估算需求未超出协议力矩量程。";
    $(role + "-output-status").classList.toggle("warning", connected && state.phase === 3 && saturated === true);
  }
  function compensation(role, state) {
    const ready = state[`compensation_ready_${role}`], clamped = state[`compensation_clamped_${role}`];
    const active = data.connected && state.compensation_active && ready;
    const required = data.configuration?.friction_compensation_enabled;
    const badge = $(role + "-compensation-status");
    badge.textContent = !data.connected ? "状态过期" : !ready ? required ? "模型不可用" : "未配置" :
      active ? clamped ? "补偿中 · 已限制" : "补偿中" : "模型就绪";
    badge.className = `badge${!data.connected || (!ready && required) ? " fault" : active && clamped ? " degraded" : active ? " active" : ""}`;
    const model = state[`compensation_model_${role}`];
    $(role + "-model").textContent = ({coulomb_viscous: "库仑 + 粘性", stribeck: "Stribeck"})[model] || model || "—";
    $(role + "-domain").textContent = ready ?
      `${format(state[`compensation_velocity_min_${role}`], 3)} ～ ${format(state[`compensation_velocity_max_${role}`], 3)}` : "—";
    $(role + "-source").textContent = state[`compensation_source_${role}`] || "未匹配模型来源";
    const message = state[`compensation_message_${role}`] || "";
    $(role + "-compensation-message").textContent = ready ?
      active && clamped ? "当前速度超出标定范围，已按边界速度计算补偿。" : "模型已按电机身份匹配；补偿范围限于标定速度域。" : message || "等待匹配模型。";
    $(role + "-compensation-message").title = message;
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
      let connected = false;
      history.forEach(row => {
        if (!Number.isFinite(row[key][series])) {connected = false; return;}
        const x = left + (right - left) * (row.time - start) / span;
        const y = 15 + 150 * (high - row[key][series]) / (high - low);
        if (connected && !row.gap) ctx.lineTo(x, y); else ctx.moveTo(x, y);
        connected = true;
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
      $("friction_compensation_enabled").checked = config.friction_compensation_enabled;
      initialized = true;
    }
    $("phase").textContent = data.connected && state ? phases[state.phase] : "连接中断";
    $("phase").className = `badge${state?.phase === 5 || !data.connected ? " fault" : state?.phase === 3 ? " active" : ""}`;
    $("quality").textContent = data.connected && state?.phase === 3 ? qualities[state.quality] : "—";
    $("quality").className = `badge${state?.quality ? " degraded" : ""}`;
    if (state) {
      const baselineReady = [1, 2, 3].includes(state.phase) || state.command_count > 0 || state.pair_count > 0;
      motor("a", state.motor_a); motor("b", state.motor_b);
      outputStatus(state, config);
      driverLimits("a", state, data.motors[state.motor_a]); driverLimits("b", state, data.motors[state.motor_b]);
      compensation("a", state); compensation("b", state);
      $("system-parameters-file").textContent = state.system_parameters_file || "—";
      $("position-baseline").hidden = baselineReady;
      $("position-error").textContent = !data.connected ? "—" : !baselineReady ? "待建立启动基准" :
        state.paired ? format(state.position_error) : "—";
      $("relative-velocity").textContent = data.connected && state.paired ? format(state.relative_velocity) : "—";
      $("pair-timing").textContent = state.paired ? `${format(state.pair_age * 1000, 2)} / ${format(state.pair_skew * 1000, 2)}${state.pair_waiting ? " · 等待新配对" : ""}` : "—";
      $("recovery").textContent = `${format(state.recovery_scale * 100, 0)}% / ${state.recovery_pairs}`;
      $("pair-count").textContent = `${state.pair_count} / ${state.degradation_count}`;
      $("command-count").textContent = String(state.command_count);
      $("controller-message").textContent = state.message;
      $("controller-message").classList.toggle("error", state.phase === 5);
      $("targets").textContent = JSON.stringify({A: state.target_a, B: state.target_b}, null, 2);
      const key = `${state.motor_a}:${state.motor_b}:${state.origin_a}:${state.origin_b}:${state.direction}:${state.control_mode}:${baselineReady}`;
      if (key !== originKey) {history = []; originKey = key; historyGap = true;}
      const a = data.motors[state.motor_a], b = data.motors[state.motor_b];
      if (data.connected && a?.connected && b?.connected && a.feedback_fresh && b.feedback_fresh) {
        history.push({time: Date.now(), gap: historyGap, position: baselineReady ?
          [a.position - state.origin_a, state.direction * (b.position - state.origin_b), state.paired ? state.position_error : null] : [null, null, null],
          torque: [a.torque, b.torque, state.estimated_torque_a, state.estimated_torque_b]});
        historyGap = false;
        while (history.length > 150 || (history.length && Date.now() - history[0].time > 15000)) history.shift();
      } else historyGap = true;
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
  async function submitConfiguration(start) {
    if ($(start ? "start" : "apply").disabled || !$("settings").reportValidity()) return;
    const version = generation;
    busy = true; controls();
    try {
      const settings = readSettings();
      if (!session) {
        const claimed = await api("session", {});
        if (version !== generation || document.hidden || disconnected) {release(claimed.session); throw new Error("操作已取消");}
        session = claimed.session;
      }
      if (version !== generation || document.hidden || disconnected) throw new Error("操作已取消");
      const body = start ? {settings, confirm_hardware: Object.values(data.motors).some(motor => !motor.simulated)} : {settings};
      const response = await api(start ? "start" : "configure", body);
      if (version === generation) {dirty = false; notice(response.message);}
    } catch (error) {notice(error.message, true);}
    finally {busy = false; controls();}
  }
  $("settings").addEventListener("submit", event => {event.preventDefault(); submitConfiguration(true);});
  $("apply").addEventListener("click", () => submitConfiguration(false));
  async function stop() {
    generation++; session = ""; stopping = true; controls();
    try {notice((await api("stop", {})).message);}
    catch (error) {notice(`STOP 未确认：${error.message}；必要时使用物理急停`, true);}
    finally {stopping = false; controls();}
  }
  $("stop").addEventListener("click", stop);
  for (const name of ["direction", "motion_limits_enabled", "friction_compensation_enabled", ...numbers]) $(name).addEventListener("input", () => {dirty = true; controls();});
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
