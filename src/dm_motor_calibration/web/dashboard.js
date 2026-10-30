"use strict";

if (typeof document !== "undefined") (() => {
  const V = CalibrationView, $ = id => document.getElementById(id);
  let snapshot = null, rows = [], stream = "", cursor = -1, fields = [], dirty = false, busy = false;
  let formVersion = "", selectedRun = "", detail = null, detailKey = "", lastHistory = 0, historyBusy = false;
  let lastRenderedEvents = "", lastFit = "", lastComparison = "", lastDownloads = "", chartQueued = false;
  let connectionFailed = false, runs = [];
  let pendingVersion = null;
  let lastPrecision = "", lastFourier = "";
  let chartCursor = null, motionCache = null;
  const analysisSelection = new Set();
  let analysisJob = {status: "idle"}, lastAnalysisSources = "";
  let stageSchema = null, stageSchemaController = "", stageDraft = {}, dialogStage = "", dialogVersion = "";
  let stageInputs = {};
  let calibrationSignature = "", calibrationController = "", dragInitialized = false;
  let dragWasRunning = false;
  let dragController = "";
  let lastConstraintRows = "";
  let saveContext = "";
  const stageIds = ["checking", "warmup", "baseline", "static", "training", "validation", "stopping", "saving", "analysis"];
  const elements = Object.fromEntries(V.editable.map(name => [name, $(name)]));
  const dragElements = {model: $("drag-calibration-file"), fraction: $("drag-fraction")};
  const text = (id, value) => { if ($(id).textContent !== String(value)) $(id).textContent = String(value); };
  const version = state => `${state.controller_id}:${state.generation}`;
  const rowValue = (row, field) => row[fields.indexOf(field) + 1];
  const calibrationConfig = state => state.calibration_configuration || state.configuration;
  const qualityLabel = run => run.automatic_qualified === false ? "自动指标未达标 · 待人工评估" : "自动指标达标";

  async function api(path, body) {
    const abort = new AbortController(), timer = setTimeout(() => abort.abort(), 5000);
    try {
      const response = await fetch(path, {method: body === undefined ? "GET" : "POST", cache: "no-store",
        signal: abort.signal, headers: body === undefined ? {} : {"Content-Type": "application/json"},
        body: body === undefined ? undefined : JSON.stringify(body)});
      const data = await response.json();
      if (!response.ok) throw new Error(data.error || `HTTP ${response.status}`);
      return data;
    } finally { clearTimeout(timer); }
  }
  function notice(message, error = false) {
    text("action-notice", message);
    $("action-notice").hidden = !message;
    $("action-notice").className = `notice${error ? " error" : ""}`;
  }
  function options() {
    const current = $("calibration_file").value;
    $("calibration_file").replaceChildren(new Option("请选择匹配电机的标定结果", ""));
    for (const run of runs) {
      if (!run.calibration_file) continue;
      const option = new Option(`${run.id} · ${V.models[run.model] || run.model} · ${qualityLabel(run)}${run.eligible ? "" : " · 不匹配"}`, run.calibration_file);
      option.disabled = !run.eligible;
      $("calibration_file").add(option);
    }
    if (current && !Array.from($("calibration_file").options).some(o => o.value === current))
      $("calibration_file").add(new Option(`配置指定：${current.split("/").pop()}`, current));
    $("calibration_file").value = current;
    const dragModel = dragElements.model.value;
    dragElements.model.replaceChildren(new Option("请选择匹配当前电机的模型", ""));
    for (const run of runs.filter(run => run.eligible))
      dragElements.model.add(new Option(`${run.id} · ${V.models[run.model] || run.model} · ${qualityLabel(run)}`, run.calibration_file));
    dragElements.model.value = runs.some(run => run.eligible && run.calibration_file === dragModel) ? dragModel : "";
    if (!dragInitialized && snapshot?.state?.drag_settings) {
      const saved = snapshot.state.drag_settings;
      if (runs.some(run => run.eligible && run.calibration_file === saved.calibration_file)) {
        dragElements.model.value = saved.calibration_file;
        dragElements.fraction.value = saved.compensation_fraction;
        dragInitialized = true;
      }
    }
    const savedModel = $("parameter-save-model").value;
    $("parameter-save-model").replaceChildren(new Option("请选择已测试的标定结果", ""));
    for (const run of runs.filter(run => run.eligible))
      $("parameter-save-model").add(new Option(
        `${run.id} · ${V.models[run.model] || run.model} · ${qualityLabel(run)} · ${run.save_test_run ? "拖动已完成" : "待拖动测试"}`, run.id));
    $("parameter-save-model").value = runs.some(run => run.eligible && run.id === savedModel) ? savedModel : "";
  }
  function renderParameterSave() {
    const state = snapshot?.state, connected = snapshot?.connected && !connectionFailed;
    const run = runs.find(item => item.id === $("parameter-save-model").value && item.eligible);
    const context = JSON.stringify([state?.controller_id, state?.generation, run?.id, run?.save_test_run]);
    if (context !== saveContext || !connected || !state?.stop_confirmed) {
      $("parameter-save-confirm").checked = false;
      saveContext = context;
    }
    const locked = !connected || busy || Boolean(pendingVersion) || !snapshot?.services_ready ||
      !snapshot?.capabilities?.manual_parameter_save || !state?.system_parameters_file ||
      !state?.stop_confirmed || V.active.has(state?.phase) || state?.stage === "stopping";
    $("parameter-save-model").disabled = locked;
    $("parameter-save-confirm").disabled = locked || !run?.save_test_run;
    $("parameter-save-submit").disabled = locked || !run?.save_test_run || !$("parameter-save-confirm").checked;
    populateDL("parameter-save-details", [
      ["目标电机", state?.identity?.namespace || "—"],
      ["环境", state?.identity ? state.identity.simulated ? "仿真" : "真机" : "—"],
      ["共享文件", state?.system_parameters_file || "未配置"],
      ["对应拖动记录", run?.save_test_run || "尚未完成"],
      ["自动质量评价", run ? qualityLabel(run) : "—"],
      ["验证 RMS / 门限", run ? `${V.number(run.quality?.validation_rmse, 4)} / ${V.number(run.quality?.rmse_limit, 4)} Nm` : "—"],
      ["本控制器最近保存", state?.last_parameter_save
        ? `${state.last_parameter_save.calibration_run} · ${state.last_parameter_save.saved_at}` : "尚无"],
    ]);
    text("parameter-save-status", !connected ? "连接中断，禁止保存。"
      : !snapshot?.capabilities?.manual_parameter_save ? "请在失能后更新控制器以启用手动保存。"
      : !state?.system_parameters_file ? "未配置共享参数文件，不能保存。"
      : !state.stop_confirmed || V.active.has(state.phase) ? "请先停止并等待新鲜失能反馈。"
      : !run ? "请选择模型并查看报告；自动指标不替代用户判断，仅生成报告不会改变系统参数。"
      : !run.save_test_run ? run.save_blocked_reason || "请先完成对应模型的拖动测试。"
      : "标定与拖动测试记录已匹配；是否接受由你决定。确认后保存，保留自动指标原始结果，不改标为自动合格。");
  }
  $("parameter-save-model").addEventListener("change", () => {
    $("parameter-save-action").hidden = true; renderParameterSave();
  });
  $("parameter-save-confirm").addEventListener("change", renderParameterSave);
  $("parameter-save-form").addEventListener("submit", async event => {
    event.preventDefault();
    const state = snapshot?.state;
    const run = runs.find(item => item.id === $("parameter-save-model").value && item.eligible);
    if ($("parameter-save-submit").disabled || !state || !run?.save_test_run) return;
    busy = true; render();
    try {
      const result = await api("/api/parameters/save", {run: run.id, test_run: run.save_test_run, confirm: true,
        controller_id: state.controller_id, generation: state.generation});
      pendingVersion = {controller_id: state.controller_id, generation: result.generation, kind: "save"};
      text("parameter-save-action", `已保存 ${run.id} 至 ${result.saved.system_parameters_file}。电机保持失能。`);
      $("parameter-save-action").className = "notice";
    } catch (error) {
      text("parameter-save-action", `保存未确认：${error.message}。请检查共享参数和状态，不要当作已成功保存。`);
      $("parameter-save-action").className = "notice error";
    } finally {
      $("parameter-save-action").hidden = false;
      $("parameter-save-confirm").checked = false;
      busy = false; render();
    }
  });
  function fillForm(state) {
    const config = calibrationConfig(state);
    options();
    for (const name of V.editable) {
      if (V.bools.has(name)) elements[name].checked = config[name] ?? name.startsWith("collect_");
      else elements[name].value = config[name];
    }
    if (elements.experiment.value === "compensate" || !elements.experiment.value) elements.experiment.value = "fourier";
    dirty = false;
    stageDraft = {};
    formVersion = version(state);
    calibrationSignature = JSON.stringify(config); calibrationController = state.controller_id;
    hints();
  }
  function hints() {
    const precision = elements.experiment.value === "precision", fourier = elements.experiment.value === "fourier";
    $("calibration-choice").hidden = ["calibrate", "precision", "fourier"].includes(elements.experiment.value);
    $("calibration_file").required = !["calibrate", "precision", "fourier"].includes(elements.experiment.value);
    $("precision-info").hidden = !precision;
    $("fourier-info").hidden = !fourier;
    text("calibration-summary", precision ? "双向分档表 · 各步骤分别配置参数。"
      : ["fourier", "calibrate"].includes(elements.experiment.value)
        ? `摩擦模型：${V.models[elements.model.value] || elements.model.value} · 在“计算与质量验收 → 参数”中调整。`
        : "此任务使用已有模型；轨迹 / 时长在对应采集步骤的参数中调整。");
  }
  function changed() { dirty = true; hints(); render(); }
  for (const element of Object.values(elements)) element.addEventListener("input", changed);
  elements.experiment.addEventListener("change", () => {
    if (!["calibrate", "precision", "fourier"].includes(elements.experiment.value) && !elements.calibration_file.value) {
      const recent = runs.find(run => run.eligible);
      if (recent) elements.calibration_file.value = recent.calibration_file;
    }
    changed();
  });
  function fillStagePreset(values) {
    for (const [name, value] of Object.entries(values)) if (stageInputs[name]) stageInputs[name].value = String(value);
    updateStageEstimate();
  }
  $("validation-preset").addEventListener("click", () => {
    const hardware = snapshot?.state?.motor?.simulated === false;
    const values = hardware ? {duration:20, amplitude:0.45, frequency:0.1, kp:0.3, kd:0.5, compensation_fraction:0.9}
      : {kp:0.3, kd:0.18};
    fillStagePreset(values);
  });
  $("identification-preset").addEventListener("click", () => {
    const hardware = snapshot?.state?.motor?.simulated === false;
    const values = hardware ? {duration:40, amplitude:0.55, frequency:0.06, kp:1.2, kd:0.5}
      : {duration:40, amplitude:0.9, frequency:0.08, kp:2, kd:0.5};
    fillStagePreset(values);
  });
  $("reload-settings").addEventListener("click", () => { if (snapshot?.state) { fillForm(snapshot.state); render(); } });

  function draftConfiguration() { return {...calibrationConfig(snapshot.state), ...V.readValues(elements), ...stageDraft}; }
  function readStageInput(name, input) {
    const spec = stageSchema.fields[name];
    return V.parseStageValue(spec.type === "boolean" ? input.checked : input.value, spec);
  }
  function stageDialogError(message) {
    text("stage-parameter-error", message); $("stage-parameter-error").hidden = !message;
  }
  function updateStageEstimate() {
    try {
      const config = draftConfiguration();
      for (const [name, input] of Object.entries(stageInputs)) config[name] = readStageInput(name, input);
      if (stageInputs.known_bias && stageInputs.fit_bias)
        stageInputs.known_bias.disabled = $("stage-parameter-save").disabled || stageInputs.fit_bias.checked;
      text("stage-parameter-estimate", dialogStage === "constraints"
        ? "每项允许范围以当前控制器提供的预算为准；后端会原子校验全部关联条件，任一失败则整批不应用。"
        : V.stageEstimate(dialogStage, config, elements.experiment.value));
      stageDialogError("");
    } catch (error) { stageDialogError(error.message); }
  }
  function openStageParameters(id) {
    if (!stageSchema || !snapshot?.state) { notice("阶段参数接口尚未就绪，请在失能后更新服务。", true); return; }
    const group = stageSchema.groups[id], config = draftConfiguration(), experiment = elements.experiment.value;
    const prepared = ["precision", "fourier"].includes(experiment);
    const names = [...new Set([...((prepared || id === "constraints") ? group.fields || [] : []), ...(group[experiment] || [])])];
    dialogStage = id; dialogVersion = version(snapshot.state); stageInputs = {};
    text("stage-parameter-title", `${group.title} · 参数`);
    text("stage-parameter-note", !prepared && ["training", "validation", "static"].includes(id)
      ? experiment === "breakaway" ? "正反向各两次起动试验，时长为总预算；固定起动判据与力矩保护不变。"
        : experiment === "validate" ? "基线与补偿使用同一轨迹及增益；时长为每段时长，另计回位。手动拖动在独立测试模块启动。"
        : "快速机械辨识使用这些激励与增益；这不是精密全速扫描。开始前仍需通过既有力矩、速度和行程检查。"
      : group.note);
    $("identification-preset").hidden = experiment !== "calibrate" || id !== "training";
    $("validation-preset").hidden = experiment !== "validate" || !["training", "validation"].includes(id);
    $("stage-parameter-fields").replaceChildren();
    for (const name of names) {
      const spec = stageSchema.fields[name], label = document.createElement("label");
      const input = document.createElement(spec.type === "choice" ? "select" : "input");
      input.id = `stage-field-${name}`; input.name = name;
      if (spec.type === "boolean") {
        input.type = "checkbox"; input.checked = config[name]; label.className = "check";
        label.append(input, document.createTextNode(spec.label));
      } else {
        label.append(document.createTextNode(spec.label), input);
        input.required = true;
        if (spec.type === "choice") for (const choice of spec.choices) input.add(new Option(V.models[choice] || choice, choice));
        else input.type = spec.type === "array" ? "text" : "number";
        input.value = spec.type === "array" ? JSON.stringify(config[name]) : String(config[name]);
        if (spec.type === "array") label.className = "array-parameter";
        else if (spec.type !== "choice") {
          input.min = String(spec.min); input.max = String(spec.max); input.step = spec.type === "integer" ? "1" : "any";
        }
      }
      input.addEventListener("input", updateStageEstimate);
      $("stage-parameter-fields").append(label); stageInputs[name] = input;
    }
    const prefix = !prepared ? "" : id === "warmup" ? "precision_warmup_" :
      experiment === "fourier" && ["training", "validation"].includes(id) ? "fourier_" : "precision_";
    populateDL("stage-parameter-limits", [
      ["力矩预算", `${config[`${prefix}max_torque`] ?? config.max_torque} Nm`],
      ["行程预算", `±${config[`${prefix}max_travel`] ?? config.max_travel} rad`],
      ["速度停止", `${config[`${prefix}max_speed`] ?? config.precision_max_speed} rad/s`],
      ["温度停止", `${prepared ? config.precision_max_temperature : config.max_temperature} °C`],
      ["反馈新鲜度上限", `${prepared ? config.precision_feedback_timeout : config.feedback_timeout} s`],
      ["原始记录目录", config.output_directory],
      ["验证 RMS 门限", `${experiment === "fourier" ? config.fourier_rmse_limit : prepared ? config.precision_rmse_limit : config.max_rmse} Nm`],
    ]);
    stageDialogError(""); updateStageEstimate(); render();
    $("stage-parameter-dialog").showModal();
  }
  for (const id of stageIds) $(`parameters-${id}`).addEventListener("click", () => openStageParameters(id));
  $("constraint-edit").addEventListener("click", () => {
    if (!$("constraint-edit").disabled) openStageParameters("constraints");
  });
  for (const id of ["stage-parameter-close", "stage-parameter-cancel"])
    $(id).addEventListener("click", () => $("stage-parameter-dialog").close());
  $("stage-parameter-form").addEventListener("submit", event => {
    event.preventDefault();
    if ($("stage-parameter-save").disabled || !$("stage-parameter-form").reportValidity()) return;
    if (dialogVersion !== version(snapshot.state)) { stageDialogError("节点配置已变化，请关闭后重新打开参数窗口。"); return; }
    try {
      const values = {};
      for (const [name, input] of Object.entries(stageInputs)) values[name] = readStageInput(name, input);
      if (values.model?.startsWith("asymmetric") && (values.fit_bias ?? elements.fit_bias.checked))
        throw new Error("非对称模型必须关闭偏置拟合，并提供独立测得的偏置。");
      for (const [name, value] of Object.entries(values)) {
        if (elements[name]) {
          if (V.bools.has(name)) elements[name].checked = value;
          else elements[name].value = value;
        } else stageDraft[name] = value;
      }
      changed(); $("stage-parameter-dialog").close();
      notice("阶段参数已保存到标定草稿，尚未发送给节点；应用标定设置或开始标定时统一校验并提交。");
    } catch (error) { stageDialogError(error.message); }
  });
  function applyStageProfile(test) {
    if (!stageSchema || !snapshot?.state) return;
    const values = test ? stageSchema.test_profiles[elements.experiment.value] :
      {...stageSchema.defaults, protocol_mode: "full", ...Object.fromEntries(V.selectable.map(name => [`collect_${name}`, true]))};
    if (!values) { notice("此测试预设仅适用于精密稳速与傅里叶任务。", true); return; }
    for (const [name, value] of Object.entries(values)) {
      if (elements[name]) elements[name].checked = value;
      else stageDraft[name] = value;
    }
    changed();
    notice(test ? "已填入约 2 分钟 / 采集阶段的流程测试参数，尚未运动。静摩擦与定位时间由反馈决定；结果仅供流程诊断。"
      : "已恢复节点启动时的完整标定参数，尚未提交；固定保护与验收门限未改变。");
  }
  $("stage-profile-full").addEventListener("click", () => applyStageProfile(false));
  $("stage-profile-test").addEventListener("click", () => applyStageProfile(true));

  async function control(start) {
    const state = snapshot?.state;
    if (!state || !snapshot.connected || connectionFailed || busy) return;
    if (dirty && formVersion !== version(state)) { notice("节点配置已变化，请先恢复节点配置后重新编辑。", true); return; }
    if (!$("settings").reportValidity()) return;
    busy = true; render();
    try {
      const body = {settings: {...V.readValues(elements), ...stageDraft}, controller_id: state.controller_id, generation: state.generation};
      for (const name of V.selectable) {
        const key = `collect_${name}`;
        if (!(key in state.configuration)) delete body.settings[key];
      }
      if (start) body.confirm_hardware = $("confirm_hardware").checked;
      const response = await api(start ? "/api/start" : "/api/configure", body);
      notice(start ? "开始请求已接受，等待实际使能与采样；不是立即完成。"
        : "设置已应用到当前节点，未修改磁盘配置；电机保持失能。");
      dirty = false;
      stageDraft = {};
      pendingVersion = {controller_id: state.controller_id, generation: response.generation};
      if (start) { selectedRun = ""; detail = null; detailKey = ""; $("history").value = ""; }
    } catch (error) { notice(`操作未成功：${error.message}`, true); }
    finally { busy = false; render(); }
  }
  $("settings").addEventListener("submit", event => { event.preventDefault(); control(true); });
  $("apply").addEventListener("click", () => control(false));
  async function stop() {
    try {
      await api("/api/stop", {});
      const message = "已请求停止当前电机，正在等待新的失能反馈；以“已确认失能”为准。";
      notice(message); dragNotice(message);
    } catch (error) {
      const message = `停止请求未确认：${error.message}。必要时使用独立物理停止措施。`;
      notice(message, true); dragNotice(message, true);
    }
  }
  $("stop").addEventListener("click", stop);
  $("drag-stop").addEventListener("click", stop);
  document.addEventListener("keydown", event => { if (event.key === "Escape") { event.preventDefault(); stop(); } });

  function dragNotice(message, error = false) {
    text("drag-action", message); $("drag-action").hidden = !message;
    $("drag-action").className = `notice${error ? " error" : ""}`;
  }
  function renderDrag() {
    const state = snapshot?.state, connected = Boolean(snapshot?.connected && !connectionFailed);
    $("constraint-edit").disabled = !connected || !state?.stop_confirmed || V.active.has(state?.phase) ||
      busy || Boolean(pendingVersion) || !snapshot?.services_ready || !stageSchema?.groups?.constraints;
    const supported = Boolean(snapshot?.capabilities?.drag_control && snapshot?.capabilities?.drag_manual_motion &&
      snapshot?.capabilities?.drag_manual_stop && snapshot?.capabilities?.drag_operator_speed);
    const running = Boolean(state && (V.active.has(state.phase) || state.stage === "stopping"));
    const dragRun = state?.experiment === "compensate";
    const locked = !connected || !supported || running || busy || Boolean(pendingVersion) || !state?.stop_confirmed;
    $("drag-editable").disabled = locked;
    $("drag-hardware-confirmation").hidden = !state?.motor || state.motor.simulated;
    const model = runs.find(run => run.eligible && run.calibration_file === dragElements.model.value);
    const config = state ? calibrationConfig(state) : {};
    const driver = state?.driver_motion_limits;
    if (state && dragController !== state.controller_id) {
      dragController = state.controller_id;
      $("drag-confirm-hardware").checked = false;
      $("fault-reset-confirm").checked = false;
    }
    const driverBlocked = driver && (!driver.enabled || !V.finite(driver.position_min) ||
      !V.finite(driver.position_max) || driver.verified && !(driver.speed_stop > 0));
    text("drag-motion-note", "拖动速度和行程由操作者控制，不再设置额外软件门限；独立拖动也不使用驱动的实测速度停机门限。绝对位置、力矩、温度和失联保护仍有效。");
    if (!state?.motor?.host_fault) $("fault-reset-confirm").checked = false;
    const resetHardware = state?.motor?.simulated === false;
    $("fault-reset-hardware-confirmation").hidden = !resetHardware || !state?.motor?.host_fault;
    $("fault-reset").disabled = !connected || !snapshot?.capabilities?.fault_reset || !state?.can_reset_fault ||
      busy || Boolean(pendingVersion) || (resetHardware && !$("fault-reset-confirm").checked);
    text("fault-reset-status", state?.phase === "resetting" ? "正在复位；等待新的失能、无故障反馈，不自动启动。"
      : state?.motor?.host_fault ? state?.can_reset_fault
        ? "主机故障已锁存。停止不能清除故障；确认静止后可手动复位，随后需重新明确启动。"
        : `暂不能复位：${V.reason(state?.fault_reset_blocked_reason || "请先停止并检查反馈")}`
      : "无待复位的主机故障；复位不会使能电机，也不清除固件故障。");
    const hardwareBlocked = state?.motor?.simulated === false && !config.allow_hardware;
    if (dragWasRunning && !(dragRun && running)) $("drag-confirm-hardware").checked = false;
    dragWasRunning = dragRun && running;
    $("drag-start").disabled = locked || driverBlocked || !model || !snapshot?.services_ready || !state?.commands_ready ||
      state?.motor_ready === false || state?.motor?.host_fault || state?.motor?.device_fault || hardwareBlocked ||
      (state?.motor?.simulated === false && !$("drag-confirm-hardware").checked);
    text("drag-model-note", model
      ? `${qualityLabel(model)}。辨识速度包络 ${V.number(model.domain?.velocity_min, 3)}～${V.number(model.domain?.velocity_max, 3)} rad/s；` +
        `验证 RMS ${V.number(model.quality?.validation_rmse, 4)} / 参考门限 ${V.number(model.quality?.rmse_limit, 4)} Nm。` +
        (model.quality_issues?.length ? ` 有 ${model.quality_issues.length} 项质量提示，请查看原报告后决定是否测试。` : "")
      : runs.some(run => run.eligible) ? "请选择模型并查看报告；自动未达标的候选也可由用户测试和评估。"
        : "当前没有身份匹配、参数完整有效的摩擦模型，不能开始补偿拖动。");
    const outside = model && state?.motor && (state.motor.velocity < model.domain?.velocity_min ||
      state.motor.velocity > model.domain?.velocity_max);
    dragElements.model.title = model ? `${model.id} · ${V.models[model.model] || model.model}` : "请选择模型";
    $("drag-domain-alert").hidden = !(connected && outside && dragRun && running);
    text("drag-domain-note", model
      ? `${outside ? "当前速度已超出辨识包络。 " : ""}静止 / 死区内不主动补偿；域外模型查询会夹断，不代表域外精度已验证。` +
        ` 标定温度 ${V.number(model.domain?.temperature_min, 0)}～${V.number(model.domain?.temperature_max, 0)} °C；` +
        `模型报告另有 ${model.model_warnings?.length || 0} 项提示。`
      : "");
    $("drag-model-report").hidden = !model;
    if (model) $("drag-model-report").href = `/api/report?${new URLSearchParams({run: model.id})}`;
    text("drag-status", !supported ? "当前服务尚未支持手动控制拖动与停止，请在确认失能后更新并刷新页面。"
      : !connected ? "连接中断：不能确认当前输出；实验可能仍在执行，请请求停止并确认失能。"
      : busy || pendingVersion ? "正在处理控制请求，等待新的状态反馈；请求接受不代表已使能或已失能。"
      : running && !dragRun ? "标定 / 验证占用当前电机，拖动测试已锁定。"
      : dragRun && running ? `拖动测试：${V.stages[state.stage] || state.stage}。${state.configuration.drag_stop_policy === "operator" ? "无定时结束，请手动停止；" : ""}参数已锁定，停止始终可用。`
      : !state?.stop_confirmed ? "失能尚未确认，禁止开始。"
      : !snapshot.services_ready || !state.commands_ready ? "控制服务或电机命令通道尚未就绪，不能开始拖动。"
      : hardwareBlocked ? "真机门禁未解锁，不能开始拖动。"
      : driverBlocked ? "驱动的速度 / 绝对位置保护未正确配置，不能开始拖动。"
      : state.motor_ready === false || state.motor?.host_fault || state.motor?.device_fault
        ? `电机未就绪或存在故障：${V.reason(state.motor?.error || state.blocked_reason || "请检查反馈与驱动故障状态")}`
      : dragRun && V.terminal.has(state.phase)
        ? `${state.phase === "failed" ? "拖动测试已中断" : "拖动测试已结束"} · 已确认失能。${V.reason(state.error)}`
      : model && state.motor?.simulated === false && !$("drag-confirm-hardware").checked
        ? "模型已选择；请确认手动操作条件与物理停止措施，再明确启动。"
        : "就绪：选择模型并明确启动后才能手动拖动，不执行预设运动轨迹。");
    const recent = state?.report?.experiment === "compensate" ? state.report :
      state?.identity ? runs.find(run => run.experiment === "compensate" && run.identity &&
        Object.entries(state.identity).every(([key, value]) => run.identity[key] === value)) : null;
    const lastError = recent?.error || recent?.failure_reason || (dragRun ? state?.error : "") || "";
    const lastLimits = recent?.configuration || recent?.stop_limits;
    $("drag-last-stop").hidden = !lastError;
    $("drag-last-stop").className = `notice${lastError.startsWith("Experiment interrupted by operator stop") ? "" : " error"}`;
    text("drag-last-stop", lastError ? `最近一次拖动中止：${V.reason(lastError)}` +
      (lastLimits && V.finite(lastLimits.max_speed)
        ? ` 历史门限：速度 ${lastLimits.max_speed} rad/s，行程 ±${lastLimits.max_travel} rad。` +
          (/^(Speed|Travel) limit reached/.test(lastError) ? " 当前手动拖动已不再使用这两项额外软件门限。" : "") : "") : "");
    text("drag-elapsed", V.number(connected && dragRun ? state.leg_elapsed : null, 1));
    text("drag-velocity", V.number(connected ? state?.motor?.velocity : null));
    text("drag-output", V.number(connected && dragRun && state.phase === "running" ? state.target?.feedforward : null));
    if (state) populateDL("drag-limits", [
      ["补偿力矩上限", `${config.max_torque} Nm`], ["实测力矩停止", `>${V.number(config.max_torque * 1.25, 3)} Nm`],
      ["拖动速度 / 相对行程", "操作者控制，无额外软件门限"],
      ["驱动速度门限", supported ? `独立拖动不启用；自动任务${driver?.verified ? `保留 ${V.number(driver.speed_stop, 3)} rad/s` : "保留原配置"}`
        : driver?.verified ? `${V.number(driver.speed_stop, 3)} rad/s` : "开始前回读确认"],
      ["驱动绝对位置边界", driver ? `${V.number(driver.position_min, 3)}～${V.number(driver.position_max, 3)} rad` : "等待反馈"],
      ["温度停止", `${config.max_temperature} °C`], ["反馈超时", `${config.feedback_timeout} s`],
      ["速度死区", `${config.compensation_deadband} rad/s`], ["补偿变化率", `${config.compensation_slew} Nm/s`],
    ]);
  }
  for (const element of Object.values(dragElements)) element.addEventListener("input", () => {
    dragInitialized = true; $("drag-confirm-hardware").checked = false; dragNotice(""); renderDrag();
  });
  $("drag-confirm-hardware").addEventListener("change", renderDrag);
  $("fault-reset-confirm").addEventListener("change", renderDrag);
  $("fault-reset").addEventListener("click", async () => {
    const state = snapshot?.state;
    if ($("fault-reset").disabled || !state) return;
    busy = true; render();
    try {
      const result = await api("/api/fault/reset", {settings: {}, controller_id: state.controller_id,
        generation: state.generation, confirm_hardware: $("fault-reset-confirm").checked});
      pendingVersion = {controller_id: state.controller_id, generation: result.generation, kind: "reset"};
      dragNotice("复位请求已接受；等待新的失能、无故障反馈。不会自动启动。");
    } catch (error) { dragNotice(`复位未成功：${error.message}`, true); }
    finally { busy = false; render(); }
  });
  $("drag-settings").addEventListener("submit", async event => {
    event.preventDefault();
    const state = snapshot?.state;
    if (!state || !snapshot.connected || connectionFailed || !snapshot.capabilities?.drag_control || !snapshot.capabilities?.drag_manual_motion ||
        !snapshot.capabilities?.drag_manual_stop || !snapshot.capabilities?.drag_operator_speed ||
        busy || pendingVersion || V.active.has(state.phase) || state.stage === "stopping" || !state.stop_confirmed) {
      dragNotice("当前电机尚未就绪或正在执行任务，未发送拖动请求。", true); return;
    }
    if (!$("drag-settings").reportValidity()) return;
    try {
      const settings = V.readDragValues(dragElements);
      if (!runs.some(run => run.eligible && run.calibration_file === settings.calibration_file))
        throw new Error("所选模型已不可用或与当前电机不匹配");
      if (!state.motor.simulated && !$("drag-confirm-hardware").checked)
        throw new Error("真机拖动前必须确认手动操作条件与物理停止措施");
      busy = true; render();
      const response = await api("/api/drag/start", {
        settings, controller_id: state.controller_id, generation: state.generation,
        confirm_hardware: $("drag-confirm-hardware").checked,
      });
      dragInitialized = true;
      pendingVersion = {controller_id: state.controller_id, generation: response.generation, kind: "drag"};
      selectedRun = ""; detail = null; detailKey = ""; $("history").value = "";
      dragNotice("拖动请求已接受，等待实际使能；标定表单和节点标定参数未改变。");
    } catch (error) { dragNotice(`拖动未成功：${error.message}`, true); }
    finally { busy = false; render(); }
  });

  async function history() {
    if (historyBusy || !snapshot?.state) return;
    historyBusy = true; lastHistory = Date.now();
    try {
      const result = await api("/api/runs");
      runs = result.runs;
      $("history").replaceChildren(new Option("当前实时过程", ""));
      for (const run of runs) {
        const label = `${run.id} · ${V.experiments[run.experiment] || "待保存"} · ${V.stages[run.status] || run.status}`;
        $("history").add(new Option(label + (run.error ? " · 有诊断" : ""), run.id));
      }
      $("history").value = selectedRun;
      options();
      renderReanalysis();
      if (snapshot.capabilities?.reanalysis) {
        analysisJob = await api("/api/reanalysis");
        renderReanalysis();
      }
    } catch (error) { notice(`历史记录读取失败：${error.message}`, true); }
    finally { historyBusy = false; }
  }
  async function loadRun(id, explicit = false) {
    const key = `${id}:${explicit ? "history" : "live"}`;
    if (detailKey === key) return;
    detailKey = key;
    try {
      const result = await api(`/api/run?run=${encodeURIComponent(id)}`);
      if (detailKey !== key) return;
      detail = result;
      lastFit = lastComparison = lastDownloads = "";
      if (!explicit) history();
      render(); queueCharts();
    } catch (error) {
      if (detailKey !== key) return;
      detail = null; detailKey = "";
      notice(`实验记录读取失败：${error.message}`, true);
    }
  }
  $("history").addEventListener("change", () => {
    chartCursor = null; motionCache = null;
    selectedRun = $("history").value;
    if (selectedRun === snapshot?.state?.run_id && V.active.has(snapshot.state.phase)) {
      selectedRun = ""; $("history").value = "";
      notice("该实验仍在执行，已切回完整实时过程；结束后可查看历史记录。");
    }
    detail = null; detailKey = ""; lastFit = lastComparison = lastDownloads = "";
    if (selectedRun) loadRun(selectedRun, true);
    render(); queueCharts();
  });
  $("live-view").addEventListener("click", () => { $("history").value = ""; $("history").dispatchEvent(new Event("change")); });
  $("chart-window").addEventListener("change", () => { chartCursor = null; motionCache = null; queueCharts(); });
  $("chart-position").addEventListener("input", () => { chartCursor = Number($("chart-position").value); queueCharts(); });
  $("chart-follow").addEventListener("click", () => { chartCursor = null; queueCharts(); });

  function renderReanalysis() {
    const supported = snapshot?.capabilities?.reanalysis;
    const sources = runs.filter(run => run.raw_recording && ["precision", "fourier"].includes(run.experiment));
    const key = JSON.stringify(sources.map(run => [run.id, run.status, run.samples]));
    if (key !== lastAnalysisSources) {
      lastAnalysisSources = key;
      const available = new Set(sources.map(run => run.id));
      for (const id of analysisSelection) if (!available.has(id)) analysisSelection.delete(id);
      $("reanalysis-sources").replaceChildren();
      for (const run of sources) {
        const label = document.createElement("label"), input = document.createElement("input");
        input.type = "checkbox"; input.checked = analysisSelection.has(run.id);
        input.addEventListener("change", () => {
          if (input.checked) analysisSelection.add(run.id); else analysisSelection.delete(run.id);
          renderReanalysis();
        });
        label.append(input, document.createTextNode(
          `${run.id} · ${run.identity?.namespace || "身份见记录"} · ${V.experiments[run.experiment]} · ${run.samples?.toLocaleString() || 0} 样本`));
        $("reanalysis-sources").append(label);
      }
    }
    const state = snapshot?.state, running = analysisJob.status === "running";
    $("reanalyze").disabled = !supported || !snapshot.connected || !state?.stop_confirmed ||
      V.active.has(state?.phase) || running || !analysisSelection.size || analysisSelection.size > 16;
    text("reanalysis-status", !supported ? "当前 Dashboard 服务尚未加载离线重算接口，需要在确认失能后重启该服务。"
      : running ? `正在核验并重算 ${analysisJob.sources?.length || analysisSelection.size} 份原始记录；不操作电机。`
      : analysisJob.status === "failed" ? `${analysisJob.error}；原始记录未修改。`
      : analysisJob.status === "ready" ? `重算结果已保存：${analysisJob.samples.toLocaleString()} 条源样本 · ${analysisJob.qualified ? "质量验收通过" : "未达到质量要求，保留诊断"}。`
      : `已选择 ${analysisSelection.size}/16 份记录。允许单份重新计算；多份必须同轴、同方法且方案兼容。请先确认当前电机失能。`);
    $("reanalysis-status").className = `notice${analysisJob.status === "failed" ? " error" : ""}`;
    $("reanalysis-result").hidden = analysisJob.status !== "ready";
  }
  $("reanalyze").addEventListener("click", async () => {
    $("reanalyze").disabled = true;
    try {
      analysisJob = await api("/api/reanalyze", {runs: Array.from(analysisSelection).sort()});
      renderReanalysis();
    } catch (error) { notice(`重算请求未成功：${error.message}`, true); renderReanalysis(); }
  });
  $("reanalysis-result").addEventListener("click", event => {
    event.preventDefault();
    if (analysisJob.status !== "ready") return;
    selectedRun = analysisJob.run; detail = null; detailKey = ""; chartCursor = null; motionCache = null;
    loadRun(selectedRun, true); history();
  });

  function viewed() {
    const state = snapshot?.state;
    const historyView = selectedRun !== "";
    const details = detail && (historyView ? detail.id === selectedRun : detail.id === state?.run_id) ? detail : null;
    return {state, details, trace: historyView ? details?.trace || [] : rows,
      fit: details?.fit || (!historyView ? state?.fit : null),
      report: details?.report || (!historyView ? state?.report : null)};
  }
  function renderRecording(view, state) {
    const report = view.report;
    const recording = selectedRun ? report?.recording_status : state.recording_status || report?.recording_status;
    const usage = recording?.usage || view.details?.data_usage;
    const connected = snapshot?.connected && !connectionFailed;
    const prefix = selectedRun ? `历史 ${selectedRun} · ` : !connected ? "连接中断，以下为旧状态 · " : "";
    text("recording-status", prefix + (!recording
      ? "暂无记录进度；旧记录或非精密任务未提供这些计数，不代表原始数据不存在。"
      : recording.error ? `记录异常：${recording.error}`
      : recording.recording_complete ? "原始记录已关闭并完成清单提交；不代表模型质量合格。"
      : "记录进行中，预运行数据同样保存；正式文件和完整性清单尚未提交。"));
    text("recording-path", selectedRun ? `查看历史原始记录：${selectedRun}` : state.output_directory || "尚未开始采集");
    $("recording-streams").replaceChildren();
    for (const [name, stream] of Object.entries(recording?.streams || {})) {
      const row = document.createElement("tr");
      for (const value of [name + (stream.error ? `：${stream.error}` : stream.closed ? "（已关闭）" : ` → ${stream.pending_file}`),
        stream.enqueued, stream.written, stream.synced, stream.queued,
        V.finite(stream.synced_bytes) ? V.number(stream.synced_bytes / 1048576, 2) : "—"]) {
        const cell = document.createElement("td"); cell.textContent = value; row.append(cell);
      }
      $("recording-streams").append(row);
    }
    populateDL("recording-usage", usage ? Object.entries(usage.counts).map(([name, count]) =>
      [usage.descriptions[name] || name, `${count.toLocaleString()} 行`])
      : [["候选归属", "尚无统计；旧记录可打开完整 HTML 报告，从原始标签重新统计"]]);
    const selection = report?.fourier?.data_selection;
    text("recording-selection", selection
      ? `实际窗口：参数优化 ${selection.training.fit_windows}；模型选择留出 ${selection.training.model_selection_windows}；` +
        `外部独立验证 ${selection.validation.external_validation_windows}。完整周期 / 相位箱索引见 fourier.json 的 data_selection。`
      : "尚无最终窗口索引。只有完成分析后才可报告实际进入拟合的窗口；原始阶段标签不是最终接纳结果。");
  }
  function populateDL(id, entries) {
    $(id).replaceChildren();
    for (const [name, value] of entries) {
      const term = document.createElement("dt"), description = document.createElement("dd");
      term.textContent = name; description.textContent = value; $(id).append(term, description);
    }
  }
  function renderConstraints() {
    const state = snapshot?.state, connected = Boolean(snapshot?.connected && !connectionFailed);
    text("constraint-status", !state ? "尚未收到配置；无法确认任何保护值。"
      : !connected ? "连接中断：以下是最后收到的配置，不代表当前电机状态。"
      : `当前轴 ${state.identity?.namespace || state.configuration.motor_namespace} · ${V.experiments[state.experiment] || state.experiment}。总览不自动写入；应用约束可在失能后明确提交。`);
    const fault = state?.motor?.device_fault ? `设备固件故障：${state.motor.error || "未提供详细错误"}`
      : state?.motor?.host_fault ? `驱动主机故障：${state.motor.error || "未提供详细错误"}`
      : state?.error ? `控制器报告（当前驱动未报告锁存故障）：${state.error}`
      : state ? "当前快照未报告故障；这不代表已完成硬件安全认证。" : "尚无故障状态。";
    text("constraint-fault", fault);
    const limits = state?.active_limits;
    populateDL("constraint-active", limits ? [
      ["输出力矩预算", `${V.number(limits.max_torque)} Nm`],
      ["应用实测力矩停止", V.finite(limits.max_torque) ? `>${V.number(limits.max_torque * 1.25)} Nm（输出预算 × 1.25）` : "未知"],
      ["应用速度停止", limits.max_speed === null ? "本任务不启用" : `${V.number(limits.max_speed)} rad/s`],
      ["应用相对行程", limits.max_travel === null ? "本任务不启用" : `±${V.number(limits.max_travel)} rad`],
      ["应用温度停止", `${V.number(limits.max_temperature)} °C`],
      ["反馈超时", `${V.number(limits.feedback_timeout)} s`],
    ] : [["保护值", "当前服务未提供实际阶段保护值，不使用通用配置推测"]]);
    const rows = V.constraintRows(state, stageSchema);
    const query = $("constraint-filter").value.trim().toLowerCase();
    const filtered = rows.filter(row => row.join(" ").toLowerCase().includes(query));
    const key = JSON.stringify(filtered);
    if (key !== lastConstraintRows) {
      lastConstraintRows = key;
      $("constraint-rows").replaceChildren();
      for (const values of filtered) {
        const row = document.createElement("tr");
        for (const value of values) {
          const cell = document.createElement("td");
          cell.textContent = value; row.append(cell);
        }
        $("constraint-rows").append(row);
      }
    }
    text("constraint-count", `显示 ${filtered.length} / ${rows.length} 项。硬编码保护及固件内部阈值不冒充可编辑参数；未公开的固件阈值无法由本页读取。`);
  }
  $("constraint-filter").addEventListener("input", renderConstraints);
  function renderFit(fit, report) {
    const key = JSON.stringify([fit, report?.breakaway_fit, report?.status]);
    if (key === lastFit) return;
    lastFit = key;
    const displayed = report?.breakaway_fit || fit;
    text("fit-model", displayed ? V.models[displayed.model] || displayed.model
      : report?.status === "completed_unqualified" ? "采集完成 · 候选结果需验证" : "等待拟合");
    $("parameter-table").replaceChildren(); $("fit-warnings").replaceChildren(); $("fit-quality").replaceChildren();
    $("fit-empty").hidden = Boolean(displayed);
    text("fit-empty", report?.status === "completed_unqualified"
      ? "数据与候选结果已保存；质量指标有偏差，未发布合格补偿文件。"
      : "不会把未完成的拟合或默认值显示成标定结果。");
    if (!displayed) return;
    const labels = {inertia: "惯量 J", bias: "负载 / 零偏", coulomb: "库仑 Fc", viscous: "黏性 B",
      static: "低速极限 Fs", stribeck_velocity: "特征速度 vs", stiffness: "刚度",
      gravity_amplitude: "重力幅值", gravity_offset: "重力相位", gravity_sin: "重力 sin 分量", gravity_cos: "重力 cos 分量",
      breakaway: "起动力矩", last_stationary: "静止末点", first_moving: "初动采样"};
    for (const [name, value] of Object.entries(displayed.parameters)) {
      const row = document.createElement("tr");
      const base = name.replace(/_(positive|negative)$/, "");
      const direction = name.endsWith("_positive") ? " · 正向" : name.endsWith("_negative") ? " · 反向" : "";
      const unmeasured = name === "inertia" && displayed.diagnostics.fit_inertia === 0;
      const label = displayed.model === "tabulated" && base === "static" ? "角度平均起动阈值" :
        name === "bias" && displayed.diagnostics.bias_is_sensor_offset ? "失能力矩零偏" : labels[base] || base;
      for (const content of [label + direction, unmeasured ? "此模式不辨识" : V.number(value, 6), displayed.units[name]]) {
        const cell = document.createElement("td"); cell.textContent = content; row.append(cell);
      }
      $("parameter-table").append(row);
    }
    const d = displayed.diagnostics;
    populateDL("fit-quality", displayed.model === "tabulated"
      ? [["速度档位 / 方向", `${d.speed_bins} × 2`], ["两向实测上限 / rad/s", V.number(d.validated_max_speed, 5)],
        ["独立验证 RMS / Nm", V.number(d.validation_rmse, 6)], ["最大验证误差 / Nm", V.number(d.max_bin_error, 6)],
        ["最大重复标准差 / Nm", V.number(d.repeatability_max, 6)], ["最大 95%界限 / Nm", V.number(d.ci95_max, 6)]]
      : displayed.kind === "breakaway"
      ? [["起动次数", d.trials], ["重复性 RMS / Nm", V.number(d.repeatability_rmse, 6)]]
      : [["独立窗口", d.samples], ["留出 RMSE / Nm", V.number(d.validation_rmse, 6)],
        ["归一化条件数", V.number(d.condition, 2)], ["残差门限 / Nm", V.number(d.rmse_limit, 5)]]);
    for (const warning of displayed.warnings || []) {
      const item = document.createElement("li"); item.textContent = warning; $("fit-warnings").append(item);
    }
  }
  function renderComparison(report) {
    const comparison = report?.comparison, key = JSON.stringify(comparison || null);
    if (key === lastComparison) return;
    lastComparison = key;
    $("comparison").replaceChildren(); $("comparison-empty").hidden = Boolean(comparison);
    if (!comparison) return;
    const rows = [["指标", "无补偿", "有补偿", "改善"],
      ...[["速度 RMSE / rad/s", "velocity_rmse"], ["位置 RMSE / rad", "position_rmse"], ["反馈控制力矩 RMS / Nm", "feedback_effort_rms"]].map(([name, key]) =>
        [name, V.number(comparison.baseline[key], 6), V.number(comparison.compensated[key], 6),
          V.number(comparison[`${key}_reduction`] * 100, 1) + "%"])];
    rows.forEach((values, index) => {
      const row = document.createElement("div"); row.className = `comparison-row${index ? "" : " comparison-head"}`;
      values.forEach((value, column) => {
        const cell = document.createElement("span"); cell.textContent = value;
        if (column === 3 && index) cell.className = `reduction${String(value).startsWith("-") ? " negative" : ""}`;
        row.append(cell);
      });
      $("comparison").append(row);
    });
  }
  function renderLiveProgress(state) {
    const precise = state.experiment === "precision", fourier = state.experiment === "fourier";
    $("precision-current").hidden = !precise;
    $("fourier-current").hidden = $("fourier-protocol-details").hidden = !fourier;
    const progress = state.precision;
    if (fourier) {
      const final = V.terminal.has(state.phase) ? state.report?.fourier : null;
      const cycle = progress?.fourier;
      const settings = final?.settings || cycle?.settings || state.fourier_settings;
      const role = {0:"训练轨迹过渡（不辨识）",1:"训练周期",2:"验证轨迹过渡（不辨识）",3:"独立验证周期"};
      text("fourier-current", final
        ? `傅里叶采集完成：${final.measured_cycles}/${final.required_cycles} 个周期；${final.qualified ? "达到记录目标" : "仍有质量提示，候选模型不可冒充合格结果"}。`
        : cycle?.reference_started
          ? `${role[cycle.role] || "回零 / 保存"} · ${cycle.completed_cycles}/${cycle.required_cycles} 周期 · 实测样本 ${cycle.samples}`
          : `${V.stages[state.stage] || state.stage} · 预运行 ${progress?.warmup?.completed_cells ?? 0}/${progress?.warmup?.required_cells ?? "—"} · 静摩擦 ${progress?.static_trial ?? 0}/${progress?.static_trials ?? "—"}`);
      populateDL("fourier-protocol", [
        ["基频 / 谐波数", settings ? `${settings.frequency} Hz / ${settings.harmonics}` : "—"],
        ["傅里叶反馈 / 主机控制", settings ? `${settings.feedback_rate} / ${state.configuration.rate} Hz` : "—"],
        ["训练慢速 / 常速周期", settings ? `${settings.slow_training_cycles} / ${settings.training_cycles}` : "—"],
        ["验证慢速 / 常速周期", settings ? `${settings.slow_validation_cycles} / ${settings.validation_cycles}` : "—"]]);
    }
    const final = V.terminal.has(state.phase) ? state.report?.precision : null;
    const settings = state.precision_settings;
    if (!settings?.speeds?.length) {
      text("precision-budget", "等待同步当前节点的精密采集方案。");
      if (precise) text("precision-current", "精密方案配置缺失，无法展示当前采集进度。");
      return;
    }
    const minimum = final?.minimum_duration || progress?.minimum_duration || state.precision_minimum_duration;
    const resume = state.resume;
    const resumeNotice = resume
      ? `断点续采：复用静摩擦及 ${resume.completed_cells} 个已完成速度单元；先复测失能零偏并回原基准，暂停时间不计入采集时限。 `
      : "";
    const warmup = final?.warmup || progress?.warmup;
    const warmNotice = settings.warmup_repetitions
      ? settings.warmup_profile === "rotation_sine"
        ? `预运行：三档速度，每方向每轮总计 ${settings.warmup_turns_per_direction} 圈；再做 ${settings.warmup_sine_cycles} 次正弦往复，` +
          `总摆幅 ${V.number(2*settings.warmup_sine_amplitude,3)} rad，${settings.warmup_sine_start_frequency}→${settings.warmup_sine_end_frequency} Hz。 `
        : `正式测量前预运行 ${settings.warmup_repetitions} 轮：全部主速度档 × 正反向整圈，回零后重新测失能零偏。 ` : "";
    const budget = resumeNotice + warmNotice + `${minimum ? `完整方案最低采集时间约 ${Math.ceil(minimum / 60)} 分钟，` : "正在同步方案时长，"} ` +
      `累计采集时限 ${Math.round(settings.time_budget / 60)} 分钟；${settings.speeds.length} 个主档 × 正反向 × ${settings.repetitions} 次，另有独立中间档。`;
    text("precision-budget", budget);
    if (!precise) return;
    text("precision-current", final ? final.qualified !== false
      ? "逐档、静摩擦及独立验证均通过，已保存可用于补偿的双向模型。"
      : `计划采集已完成：${final.measured_cells}/${final.required_cells} 个完整转圈单元；质量偏差见结果区，全部原始数据已保存。`
      : progress && warmup?.enabled && (warmup.active || progress.phase === "await_warmup_enable")
        ? warmup.sine && (progress.phase.startsWith("warmup_sine") || progress.phase === "warmup_final_home")
          ? `${V.stages["precision_" + progress.phase] || progress.phase} · 已测正弦往复 ${warmup.sine.completed_cycles}/${warmup.sine.required_cycles} 次 · ` +
            `参考频率 ${V.number(warmup.sine.current_reference_frequency,3)} Hz · 正式速度单元尚未计入`
          : `${V.stages["precision_" + progress.phase] || progress.phase} · 预运行 ${warmup.completed_cells}/${warmup.required_cells} 个单元 · ` +
            `当前 ${V.number(progress.target_speed, 5)} rad/s · 已转 ${V.number(progress.sample_revolutions, 2)}/${progress.required_revolutions} 圈 · 正式速度单元尚未计入`
      : progress ? `${progress.baseline_settling ? "失能后稳定等待（尚未统计零偏）"
        : V.stages["precision_" + progress.phase] || progress.phase} · ` +
        `静摩擦${progress.static_conditioning ? "预处理" : "测量"} ${progress.static_trial}/${progress.static_trials} · 速度档 ${progress.completed_cells}/${progress.cells} · ` +
        `当前 ${V.number(progress.target_speed, 5)} rad/s / 第 ${progress.repetition} 次` +
        `${progress.validation ? "（独立中间档）" : ""} · 已采 ${V.number(progress.sample_revolutions, 2)}/${progress.required_revolutions} 圈`
        : budget);
  }
  function renderFourier(view, state) {
    const shown = selectedRun ? view.report?.experiment === "fourier" : state.experiment === "fourier";
    $("fourier-panel").hidden = !shown;
    if (!shown) return;
    const final = view.report?.fourier, candidate = final?.candidate_model;
    const key = JSON.stringify([selectedRun || state.run_id, view.report?.status, final?.metrics, candidate, final?.quality_issues]);
    if (key === lastFourier) return;
    lastFourier = key;
    const metrics = final?.metrics || {};
    populateDL("fourier-quality", [
      ["训练 RMS / Nm", V.number(metrics.training_rmse,6)], ["独立验证 RMS / Nm", V.number(metrics.validation_rmse,6)],
      ["最大窗口误差 / Nm", V.number(metrics.maximum_validation_error,6)], ["标准化条件数", V.number(metrics.condition,3)]]);
    $("fourier-parameters").replaceChildren();
    if (candidate) for (const [name, value] of Object.entries(candidate.parameters)) {
      const tr = document.createElement("tr");
      for (const field of [name, V.number(value,7), candidate.units[name]]) {
        const td = document.createElement("td"); td.textContent = field; tr.append(td);
      }
      $("fourier-parameters").append(tr);
    }
    text("fourier-notes", final
      ? [...(final.quality_issues || []), ...(candidate?.warnings || [])].join("；")
      : "周期平均用于诊断；非线性模型在独立实测周期的积分窗口上拟合。结束后会恢复驱动原采样频率，完整结果见 HTML 报告。");
  }
  function renderPrecision(view, state) {
    const precise = selectedRun ? view.report?.experiment === "precision" : state.experiment === "precision";
    $("precision-panel").hidden = !precise;
    if (!precise) return;
    const final = view.report?.precision;
    const progress = selectedRun ? view.report?.precision_progress : state.precision;
    const recordedSettings = view.report?.configuration ? Object.fromEntries(
      Object.entries(view.report.configuration).filter(([name]) => name.startsWith("precision_"))
        .map(([name, value]) => [name.slice(10), value])) : null;
    const settings = final?.settings || (selectedRun ? recordedSettings : state.precision_settings);
    if (!settings?.speeds?.length) {
      text("precision-quality", "精密方案配置缺失，无法展示验收表。"); return;
    }
    const staticData = final?.static || progress?.static || progress?.static_diagnostics;
    const issues = final?.quality_issues || [...(progress?.collection_notes || []),
      ...(staticData?.qualified === false ? staticData.quality_errors || [] : [])];
    const key = JSON.stringify([view.details?.id, view.report?.status, final?.metrics,
      progress?.completed_cells, staticData?.by_angle, issues]);
    if (key === lastPrecision) return;
    lastPrecision = key;
    const observations = final?.observations || progress?.observations || [];
    const levels = final?.levels || settings.speeds.flatMap(speed => [1, -1].map(direction => {
      const group = observations.filter(row => !row.validation && row.speed === speed && row.direction === direction);
      const mean = name => group.length ? group.reduce((sum, row) => sum + row[name], 0) / group.length : null;
      return {target_speed: speed * direction, actual_speed: mean("measured_speed"),
        mean_torque: mean("torque"), repetitions: group.length};
    }));
    $("speed-bins").replaceChildren();
    function addRow(values, parent) {
      const row = document.createElement("tr");
      for (const value of values) { const cell = document.createElement("td"); cell.textContent = value; row.append(cell); }
      parent.append(row);
    }
    for (const row of levels) addRow([V.number(row.target_speed, 5),
      `${row.repetitions}/${row.required_repetitions ?? settings.repetitions}`, V.number(row.actual_speed, 5), V.number(row.mean_torque, 6),
      V.number(row.repeatability, 6), V.number(row.ci95, 6), V.number(row.validation_error, 6)], $("speed-bins"));
    for (const row of final?.validation || []) if (row.validation) addRow([
      `${V.number(row.direction * row.speed, 5)} · 验证`, "独立", V.number(row.measured_speed, 5),
      V.number(row.torque, 6), "—", "—", V.number(row.error, 6)], $("speed-bins"));
    $("static-bins").replaceChildren();
    for (const row of staticData?.by_angle || []) addRow([
      V.number(row.angle, 4), row.direction > 0 ? "正向" : "反向", V.number(row.torque, 6),
      V.number(row.repeatability, 6), V.number(row.ci95, 6)], $("static-bins"));
    $("precision-issues").replaceChildren();
    for (const message of issues.slice(-12)) {
      const item = document.createElement("li"); item.textContent = message; $("precision-issues").append(item);
    }
    text("precision-quality", (staticData?.qualified === false
      ? "静摩擦质量目标未达到：保留结果并继续速度采集，不将这些数值伪装成已验证参数。 " : "") +
      `质量参考（不阻断采集）：验证 RMS ≤ ${settings.rmse_limit} Nm；每档最大误差 ≤ ${settings.max_error} Nm；` +
      `重复标准差 ≤ ${settings.repeatability_limit} Nm；95%统计界限加量化误差 ≤ ${settings.ci95_limit} Nm。` +
      (issues.length ? ` 已记录 ${issues.length} 项提示，完整明细见报告。` : "") +
      "这些指标不认证固件力矩标尺；温度与未测外部负载必须受控。");
  }
  function render() {
    const state = snapshot?.state, connected = Boolean(snapshot?.connected && !connectionFailed);
    renderParameterSave();
    text("connection", connected ? "实时连接" : "连接中断 / 等待");
    $("connection").className = `badge ${connected ? "good" : "bad"}`;
    document.querySelector(".metrics").classList.toggle("stale", !connected || !state?.motor);
    if (!state) {
      $("start").disabled = $("apply").disabled = $("editable").disabled = true;
      for (const name of V.selectable) elements[`collect_${name}`].disabled = true;
      for (const id of stageIds) $(`parameters-${id}`).disabled = true;
      $("stage-profile-full").disabled = $("stage-profile-test").disabled = true;
      text("controller-stage", "等待标定器状态");
      text("disabled-state", "失能状态未知");
      $("disabled-state").className = "badge bad";
      text("stage-label", "等待标定器状态");
      $("precision-current").hidden = $("fourier-current").hidden = $("fourier-protocol-details").hidden = true;
      renderDrag();
      renderConstraints();
      return;
    }
    if (pendingVersion && (state.controller_id !== pendingVersion.controller_id || state.generation >= pendingVersion.generation)) {
      const kind = pendingVersion.kind;
      pendingVersion = null;
      if (kind !== "drag" && kind !== "reset") formVersion = "";
    }
    const running = V.active.has(state.phase) || state.stage === "stopping";
    const config = calibrationConfig(state);
    if (!busy && !pendingVersion && calibrationController === state.controller_id &&
        calibrationSignature === JSON.stringify(config) && formVersion)
      formVersion = version(state);
    if (!dirty && !busy && !pendingVersion && formVersion !== version(state)) fillForm(state);
    $("editable").disabled = !connected || running || busy || Boolean(pendingVersion);
    const stageSupported = V.selectable.every(name => typeof state.configuration[`collect_${name}`] === "boolean");
    const prepared = ["precision", "fourier"].includes(elements.experiment.value);
    const parameterLocked = $("editable").disabled || !state.stop_confirmed || !stageSchema;
    $("stage-profile-full").disabled = $("stage-profile-test").disabled = parameterLocked || !prepared;
    $("stage-parameter-save").disabled = parameterLocked || !Object.keys(stageInputs).length;
    for (const input of Object.values(stageInputs)) input.disabled = parameterLocked;
    if (stageInputs.known_bias && stageInputs.fit_bias)
      stageInputs.known_bias.disabled = parameterLocked || stageInputs.fit_bias.checked;
    $("identification-preset").disabled = $("validation-preset").disabled = parameterLocked;
    for (const id of stageIds) $(`parameters-${id}`).disabled = !stageSchema;
    const testMode = (stageDraft.protocol_mode || config.protocol_mode) === "workflow_test";
    text("protocol-mode", testMode ? "流程测试配置 · 目标约 2 分钟 / 采集阶段 · 不发布合格补偿模型" : "完整标定配置 · 以实测覆盖与原验收门限为准");
    $("protocol-mode").className = `notice${testMode ? " test-profile" : ""}`;
    for (const name of V.selectable) {
      elements[`collect_${name}`].disabled = $("editable").disabled || !prepared || !stageSupported;
      if (!prepared) elements[`collect_${name}`].checked = true;
    }
    const selection = Object.fromEntries(V.selectable.map(name => [`collect_${name}`, elements[`collect_${name}`].checked]));
    const dragContext = state.experiment === "compensate";
    const planning = dragContext || !running && (dirty || state.phase === "idle");
    const stepState = planning ? {...state, experiment: elements.experiment.value} : state;
    const executedSelection = !planning ? state.precision?.acquisition_stages || selection : selection;
    for (const step of V.stageRows(stepState, executedSelection, planning)) {
      $(`step-${step.id}`).hidden = !step.visible;
      $(`step-${step.id}`).className = step.kind;
      text(`step-name-${step.id}`, step.label);
      text(`step-state-${step.id}`, step.status);
      text(`step-detail-${step.id}`, step.detail);
    }
    text("stage-selection-note", !prepared
      ? "此任务按完整流程执行；可选阶段适用于精密稳速与傅里叶采集。"
      : !stageSupported ? "当前标定节点尚不支持阶段选择。需在确认失能后重启新版节点；此时仍按节点原流程执行。"
      : "勾选影响下一次采集；未勾选只跳过，不自动复用历史数据。安全检查、零偏、失能确认和保存始终执行。");
    const conflict = dirty && formVersion !== version(state);
    $("start").disabled = !connected || busy || Boolean(pendingVersion) || !snapshot.services_ready || !state.can_start || !state.commands_ready || conflict;
    $("apply").disabled = !connected || running || busy || Boolean(pendingVersion) || !snapshot.services_ready || !state.stop_confirmed || conflict;
    text("dirty", pendingVersion ? "请求已接受，等待节点发布新的配置 / 阶段。" : conflict ? "节点配置已变化。请恢复节点配置后重新编辑。" : dirty ? "表单已修改，尚未应用；开始时会应用。" : "设置与节点一致。运行中锁定，刷新页面不会启动。");
    const motor = state.motor;
    text("source", motor ? `${connected ? "" : "上次："}${motor.simulated ? "物理仿真" : "真实电机"}` : "来源未知");
    $("source").className = `badge ${connected && motor?.simulated ? "good" : "warn"}`;
    text("motor-name", `${state.configuration.motor_namespace} · 电机 ${motor?.motor_id ?? "—"}`);
    text("controller-stage", (connected ? "" : "上次状态 · ") +
      `${V.experiments[state.experiment] || state.experiment} · ${V.stages[state.phase] || state.phase}`);
    $("controller-stage").title = $("controller-stage").textContent;
    $("controller-stage").className = `badge phase-badge ${!connected || state.phase === "failed" ? "bad"
      : state.phase === "completed" ? "good" : "warn"}`;
    $("hardware-confirmation").hidden = !motor || motor.simulated;
    text("hardware-confirmation-text", ["precision", "fourier"].includes(elements.experiment.value)
      ? "已确认空载、可多圈连续旋转、无绕线/限位风险及物理停止措施"
      : "已确认真机固定、可用行程及物理停止措施");
    text("connection-notice", !connected ? "标定器状态陈旧 / 连接中断。保留已收到的曲线，不会自动重启；实验可能仍在执行。"
      : snapshot.error || (running ? "实验正在执行，设置与重复开始已锁定；停止按钮仍可使用。"
        : state.can_start ? "已就绪。开始是明确的运动请求；打开、刷新和查看历史都不会使能。" : V.reason(state.blocked_reason)));
    text("stage-label", dragContext ? running ? "拖动测试占用当前电机 · 标定已锁定" : "标定计划已保留 · 等待开始"
      : (connected ? "" : "上次状态 · ") + (V.stages[state.stage] || state.stage));
    text("disabled-state", !connected ? "失联 · 停机状态未知" : state.stop_confirmed ? "已确认失能" : state.phase === "running" ? "实验运行中" : "失能待确认");
    $("disabled-state").className = `badge ${!connected ? "bad" : state.stop_confirmed ? "good" : state.stage === "stopping" ? "bad" : "warn"}`;
    const progress = dragContext ? 0 : Math.round(100 * (V.completed.has(state.phase) ? 1 : state.progress));
    $("progress-fill").style.width = `${progress}%`;
    $("progress").className = `progress-track${dragContext ? "" : state.phase === "failed" ? " failed"
      : state.phase === "completed_unqualified" ? " warning" : state.phase === "completed" ? " completed" : ""}`;
    $("progress").setAttribute("aria-valuenow", String(progress));
    text("progress-text", dragContext ? "拖动进度见下方独立测试模块" : `${V.stages[state.stage] || state.stage} · ${V.number(state.elapsed, 1)} s`);
    text("sample-count", dragContext ? "" : `${state.samples.toLocaleString()} 条原始样本 · ${snapshot.display_count} 个显示点`);
    text("stage-detail", state.stage === "saving_failure" ? "正在保存中断或失败现场；文件写入完成后才发布终态，不把失败数据作为可用标定。"
      : state.phase === "completed_unqualified" ? "全部计划档位已执行，数据和质量诊断已保存；这不是控制失败，也不代表拟合已经达到精度目标。"
      : state.stage === "stopping" ? "已请求停止输出，等待新鲜的设备失能反馈；未确认时不能视为已停止。"
      : state.experiment === "breakaway" ? "正反向各两次：同向接近基准、精确回位、撤除预载静置，再逐步增矩；需持续同向速度和足够位移才确认起动。"
      : state.experiment === "validate" ? "两段使用相同参考轨迹；中间回位并确认静止，结束后比较误差和控制努力。"
      : state.experiment === "compensate" ? "仅根据实测速度提供有界摩擦助力，不施加位置伺服、不猜测静止起步方向；到时或停止后失能。"
      : state.stage === "fitting" ? "已确认电机失能，正在辨识 / 校验并持久化；此时不发送运动目标。"
      : state.experiment === "precision" ? "采集与质量评价分开：精度偏差、温漂和未稳速数据照常记录，不阻断后续档位；硬件故障和安全保护仍会停止。"
      : state.experiment === "fourier" ? "先预运行、测零偏和独立静摩擦，再执行傅里叶训练与验证周期；原始反馈完整记录，模型是否接受由用户查看报告和拖动测试后决定。"
      : "纯力矩驱动多频轨迹；判据未通过时保留现场数据，不发布可用标定。");
    text("position-value", V.number(motor?.position)); text("velocity-value", V.number(motor?.velocity));
    text("torque-value", V.number(motor?.torque)); text("temperature-value", motor ? `${motor.rotor_temperature} / ${motor.mos_temperature}` : "—");
    const limits = state.active_limits || (state.experiment === "precision" ? state.precision_settings : state.configuration);
    const manualMotion = state.experiment === "compensate" && state.drag_motion_policy === "driver_only";
    populateDL("limits", [["力矩预算", `${limits.max_torque} Nm`], ["行程", manualMotion ? "操作者控制，保留驱动保护" : `±${limits.max_travel} rad`],
      ["速度停止", manualMotion ? "使用驱动固有保护" : `${limits.max_speed} rad/s`], ["温度", `${limits.max_temperature} °C`],
      ["控制频率", `${state.configuration.rate} Hz`],
      ["轨迹闭环", state.trajectory_control === "friction_only" ? "摩擦补偿纯力矩 · 无位置/速度保持" : state.trajectory_control === "fourier_cascaded_velocity" ? "傅里叶位置外环 + 限速的电机侧速度环" : state.trajectory_control === "precision_velocity_pi" ? "速度 PI + 电机侧阻尼" : state.trajectory_control === "mit_pd" ? "电机侧 MIT-PD" : state.trajectory_control === "host_torque" ? "主机纯力矩" : "待开始时选择"],
      ["真机门禁", state.configuration.allow_hardware ? "已由配置解锁" : "禁止真机运动"]]);
    renderLiveProgress(state);
    $("precision-current").hidden = $("fourier-current").hidden = true;
    renderReanalysis();
    renderDrag();
    renderConstraints();
    const view = viewed();
    renderRecording(view, state);
    renderPrecision(view, state);
    renderFourier(view, state);
    $("comparison-panel").hidden = ["precision", "fourier"].includes(selectedRun ? view.report?.experiment : state.experiment);
    const precisionView = selectedRun ? view.report?.experiment === "precision" : state.experiment === "precision";
    const amberLegend = document.querySelector(".observations .legend .amber");
    if (amberLegend) amberLegend.textContent = precisionView ? "速度积分 / 力矩激励" : "摩擦前馈";
    text("view-notice", selectedRun ? `正在查看历史 ${selectedRun} · ${view.report?.identity?.namespace || "轴身份见报告"}；顶栏状态、左侧设置 / 进度及启停始终针对当前电机。${view.details?.pending ? "此记录尚未完成写入。" : ""}`
      : `${snapshot.trace_complete ? "已收到完整显示记录" : "正在补齐早期过程记录"} · 显示约 ${V.number(1/(state.display_period || 0.1),1)} Hz，完整原始数据另存 CSV。`);
    renderFit(view.fit, view.report); renderComparison(view.report);
    const events = selectedRun ? view.report?.events || [] : state.events;
    const eventSummary = selectedRun ? view.report : state;
    text("events-note", eventSummary?.events_truncated
      ? `当前仅显示最近 ${events.length} / ${eventSummary.event_count} 条事件；新格式完整事件记录在 events.csv。` : "");
    const eventKey = JSON.stringify(events);
    if (eventKey !== lastRenderedEvents) {
      lastRenderedEvents = eventKey; $("events").replaceChildren();
      for (const event of events) {
        const item = document.createElement("li"), time = document.createElement("time");
        time.textContent = `${V.number(event.elapsed, 1)} s`;
        item.append(time, document.createTextNode((V.stages[event.stage] || event.stage) + (event.error ? ` · ${event.error}` : "")));
        $("events").append(item);
      }
    }
    text("run-id", selectedRun || state.run_id || "尚无实验");
    const error = selectedRun ? view.report?.error || "" : state.error;
    const failureStage = selectedRun ? view.report?.failure_stage : state.failure_stage;
    $("result-error").hidden = !error;
    text("result-error", (error && failureStage ? `${V.stages[failureStage] || failureStage}：` : "") + V.reason(error));
    text("output-path", selectedRun ? `历史原始样本：${view.details?.raw_samples ?? "—"}` : state.output_directory);
    const reportRun = selectedRun || state.run_id;
    const reportState = selectedRun ? view.report?.status : state.phase;
    const reportAvailable = reportRun && ["completed", "completed_unqualified", "qualified", "failed", "incomplete", "recorded"].includes(reportState);
    $("html-report-link").hidden = $("html-report-hint").hidden = !reportAvailable;
    if (reportAvailable) $("html-report-link").href = `/api/report?${new URLSearchParams({run: reportRun})}`;
    const downloadsKey = JSON.stringify([view.details?.id, view.details?.artifacts]);
    if (downloadsKey !== lastDownloads) {
      lastDownloads = downloadsKey; $("downloads").replaceChildren();
      for (const filename of view.details?.artifacts || []) {
        const link = document.createElement("a");
        link.href = `/api/artifact?${new URLSearchParams({run: view.details.id, file: filename})}`;
        link.download = `${view.details.id}-${filename}`; link.textContent = `下载 ${filename}`; $("downloads").append(link);
      }
    }
  }

  function chart(id, series, axis, scatter = false) {
    const limit = Math.max(2, Math.floor(V.maxChartPoints / Math.max(1, series.length)));
    series = series.map(item => ({...item, points: V.limitRows(item.points, limit, scatter ? [0, 1, 2] : [1])}));
    const canvas = $(id), rect = canvas.getBoundingClientRect(), ratio = Math.min(window.devicePixelRatio || 1, 2);
    if (!rect.width || !rect.height) return;
    canvas.width = Math.round(rect.width * ratio); canvas.height = Math.round(rect.height * ratio);
    const ctx = canvas.getContext("2d"); ctx.scale(ratio, ratio);
    const width = rect.width, height = rect.height, area = {l: 51, r: width - 15, t: 16, b: height - 29};
    ctx.clearRect(0, 0, width, height); ctx.font = "10px system-ui";
    const domain = V.bounds(series);
    canvas.dataset.points = String(series.reduce((sum, item) => sum + item.points.length, 0));
    if (!domain) { ctx.fillStyle = "#8ea9bf"; ctx.textAlign = "center"; ctx.fillText("等待真实采样数据", width / 2, height / 2); return; }
    const x = v => area.l + (v - domain.x0) / (domain.x1 - domain.x0) * (area.r - area.l);
    const y = v => area.b - (v - domain.y0) / (domain.y1 - domain.y0) * (area.b - area.t);
    ctx.strokeStyle = "#294158"; ctx.lineWidth = 1;
    for (let i = 0; i <= 4; i++) {
      const yy = area.t + (area.b - area.t) * i / 4;
      ctx.beginPath(); ctx.moveTo(area.l, yy); ctx.lineTo(area.r, yy); ctx.stroke();
      ctx.textAlign = "right"; ctx.fillStyle = "#8ea9bf";
      ctx.fillText(V.number(domain.y1 - (domain.y1 - domain.y0) * i / 4, 3), area.l - 7, yy + 3);
      const xx = area.l + (area.r - area.l) * i / 4;
      ctx.textAlign = "center"; ctx.fillText(V.number(domain.x0 + (domain.x1 - domain.x0) * i / 4, 1), xx, height - 12);
    }
    ctx.textAlign = "right"; ctx.fillText(axis, area.r, 10);
    ctx.save(); ctx.beginPath(); ctx.rect(area.l, area.t, area.r - area.l, area.b - area.t); ctx.clip();
    for (const item of series) {
      ctx.strokeStyle = ctx.fillStyle = item.color; ctx.lineWidth = item.width || 1.5;
      ctx.setLineDash(item.dash ? [5, 4] : []);
      if (scatter && !item.line) {
        const step = Math.max(1, Math.ceil(item.points.length / 3000));
        ctx.globalAlpha = 0.6;
        for (let i = 0; i < item.points.length; i += step) {
          const p = item.points[i]; if (!V.finite(p[0]) || !V.finite(p[1])) continue;
          if (item.errorbars && V.finite(p[2])) {
            ctx.beginPath(); ctx.moveTo(x(p[0]), y(p[1] - p[2])); ctx.lineTo(x(p[0]), y(p[1] + p[2]));
            ctx.moveTo(x(p[0]) - 3, y(p[1] - p[2])); ctx.lineTo(x(p[0]) + 3, y(p[1] - p[2]));
            ctx.moveTo(x(p[0]) - 3, y(p[1] + p[2])); ctx.lineTo(x(p[0]) + 3, y(p[1] + p[2])); ctx.stroke();
          }
          ctx.beginPath(); ctx.arc(x(p[0]), y(p[1]), 1.8, 0, Math.PI * 2); ctx.fill();
        }
        ctx.globalAlpha = 1;
      } else {
        ctx.beginPath(); let previous = null;
        for (const p of item.points) {
          if (!V.finite(p[0]) || !V.finite(p[1])) { previous = null; continue; }
          if (previous === null || (!scatter && p[0] - previous > (item.maxGap || 0.5))) ctx.moveTo(x(p[0]), y(p[1]));
          else ctx.lineTo(x(p[0]), y(p[1]));
          previous = p[0];
        }
        ctx.stroke();
      }
    }
    ctx.restore();
  }
  function drawCharts() {
    chartQueued = false;
    const view = viewed(), trace = view.trace;
    const timeIndex = fields.indexOf("time") + 1, seconds = Number($("chart-window").value);
    const columns = ["position", "target_position", "velocity", "target_velocity", "torque", "command_torque", "feedforward"];
    const dimensions = ["position-chart", "velocity-chart", "torque-chart"].map(id => {
      const rect = $(id).getBoundingClientRect(); return `${rect.width}:${rect.height}`;
    }).join(",") + `:${window.devicePixelRatio}`;
    const fieldsKey = fields.join(",");
    if (!motionCache || motionCache.trace !== trace || motionCache.seconds !== seconds ||
        motionCache.cursor !== chartCursor || motionCache.fields !== fieldsKey || motionCache.dimensions !== dimensions) {
      const selection = V.windowRows(trace, {seconds, end: chartCursor, maxPoints: Math.floor(V.maxChartPoints / 3),
        timeIndex, valueIndices: columns.map(name => fields.indexOf(name) + 1)});
      motionCache = {trace, seconds, cursor: chartCursor, fields: fieldsKey, dimensions, selection};
      const interval = trace.length > 1 ? (trace[trace.length - 1][timeIndex] - trace[0][timeIndex]) / (trace.length - 1) : 0.1;
      const line = (field, color, dash = false) => ({
        points: selection.rows.map(row => [row[timeIndex], rowValue(row, field)]), color, dash,
        maxGap: selection.sampled ? Infinity : Math.max(0.5, interval * 3)});
      chart("position-chart", [line("position", V.colors.measured), line("target_position", V.colors.target, true)], "s");
      chart("velocity-chart", [line("velocity", V.colors.measured), line("target_velocity", V.colors.target, true)], "s");
      chart("torque-chart", [line("torque", V.colors.measured), line("command_torque", V.colors.target, true),
        line("feedforward", V.colors.feedforward)], "s");
      text("chart-window-info", selection.rows.length
        ? `显示 ${V.number(selection.start, 1)}–${V.number(selection.end, 1)} s · 窗口内 ${selection.inWindow} 个显示样本，保峰值绘制 ${selection.rows.length} 个时刻；每图最多 ${V.maxChartPoints} 点。原始 CSV 不删减。`
        : "等待采样。实时图默认显示最近 60 秒，每图最多 1200 点；原始 CSV 不删减。");
    }
    $("chart-position").disabled = !trace.length;
    if (trace.length) {
      $("chart-position").min = String(trace[0][timeIndex]);
      $("chart-position").max = String(trace[trace.length - 1][timeIndex]);
      $("chart-position").value = String(chartCursor === null ? trace[trace.length - 1][timeIndex] : chartCursor);
    }
    $("chart-follow").disabled = chartCursor === null;
    const plot = view.details?.plot;
    let points, curves = [];
    if (plot) {
      points = plot.points;
      curves = plot.curves || [];
      text("friction-caption", plot.kind === "precision_steady"
        ? "各速度档的实测整圈功积分均值；误差棒为重复统计界限。没有合格模型时仍显示测量点，不把候选结果当作合格补偿。"
        : plot.kind === "corrected_windows"
        ? `校正窗口：已扣除惯量、重力、刚度和偏置。${plot.window_count} 个独立积分窗；模型曲线为稳态关系。`
        : "显示所用的已标定摩擦模型；当前实验不重新辨识摩擦。");
      if (view.details.plot_error) text("friction-caption", view.details.plot_error);
    } else if (!selectedRun && snapshot?.state?.experiment === "precision") {
      points = (snapshot.state.precision?.observations || []).map(row => [row.measured_speed, row.torque]);
      text("friction-caption", "精密采集中：显示已完成整圈的实测档位均值。精度偏差只作记录，继续后续采集；全部采集后再判定模型质量。");
    } else {
      points = motionCache.selection.rows.map(row => [rowValue(row, "velocity"), rowValue(row, "torque")]);
      text("friction-caption", "当前显示窗口的原始速度—力矩点：尚未扣除惯量 / 负载，不等于纯摩擦；完整原始数据仍保存在 CSV。");
    }
    chart("friction-chart", [
      {points: points.filter(p => p[0] >= 0), color: V.colors.measured, errorbars: plot?.kind === "precision_steady"},
      {points: points.filter(p => p[0] < 0), color: V.colors.target, errorbars: plot?.kind === "precision_steady"},
      ...curves.map(points => ({points, color: V.colors.fit, line: true, width: 2.2})),
    ], "rad/s", true);
  }
  function queueCharts() {
    if (!chartQueued) { chartQueued = true; requestAnimationFrame(drawCharts); }
  }
  new ResizeObserver(queueCharts).observe(document.querySelector(".observations"));
  const topbar = document.querySelector(".topbar");
  const controlsLayout = new ResizeObserver(() => {
    document.documentElement.style.setProperty("--topbar-height", `${Math.ceil(topbar.getBoundingClientRect().height)}px`);
    const offset = document.querySelector(".workspace").getBoundingClientRect().top + window.scrollY;
    document.documentElement.style.setProperty("--workspace-offset", `${Math.ceil(offset)}px`);
  });
  for (const element of [topbar, $("connection-notice"), $("action-notice")]) controlsLayout.observe(element);

  async function poll() {
    try {
      const data = await api(`/api/state?${new URLSearchParams({after: cursor, stream})}`);
      if (!Array.isArray(data.trace) || !Array.isArray(data.display_fields)) throw new Error("无效的显示数据格式");
      const reset = stream !== data.stream_id;
      if (reset) {
        rows = []; cursor = -1;
        if (!selectedRun) { detail = null; detailKey = ""; chartCursor = null; motionCache = null; }
      }
      rows = V.mergeRows(rows, data.trace, reset);
      stream = data.stream_id; fields = data.display_fields;
      cursor = rows.length ? rows[rows.length - 1][0] : -1;
      snapshot = data; connectionFailed = false;
      if (data.capabilities?.stage_parameters && (!stageSchema || stageSchemaController !== data.state?.controller_id)) {
        stageSchema = await api("/api/stage-parameters");
        stageSchemaController = data.state.controller_id;
      }
      render(); queueCharts();
      if (Date.now() - lastHistory > 5000) history();
      const state = data.state;
      if (!selectedRun && state?.run_id && V.terminal.has(state.phase) && state.stop_confirmed)
        loadRun(state.run_id);
    } catch (error) {
      connectionFailed = true;
      render();
      text("connection-notice", `连接中断：${error.message}。保留曲线，不会自动开始；停止请求也需实际送达并确认。`);
    } finally { setTimeout(poll, 200); }
  }
  poll();
})();
