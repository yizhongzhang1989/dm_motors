"use strict";

if (typeof document !== "undefined") (() => {
  const V = CalibrationView, $ = id => document.getElementById(id);
  let snapshot = null, rows = [], stream = "", cursor = -1, fields = [], dirty = false, busy = false;
  let formVersion = "", selectedRun = "", detail = null, detailKey = "", lastHistory = 0, historyBusy = false;
  let lastRenderedEvents = "", lastFit = "", lastComparison = "", lastDownloads = "", lastConfiguration = "", chartQueued = false;
  let connectionFailed = false, runs = [], followLatest = true, draftMode = false, historyLoaded = false;
  let formContext = "";
  let pendingVersion = null;
  let lastPrecision = "";
  let chartCursor = null, motionCache = null;
  const analysisSelection = new Set();
  let analysisJob = {status: "idle"}, lastAnalysisSources = "";
  let stageSchema = null, stageSchemaController = "", stageDraft = {}, dialogStage = "", dialogVersion = "";
  let stageInputs = {};
  let calibrationSignature = "", calibrationController = "", dragInitialized = false;
  let quickDraftSeed = null, quickDraftEdited = false;
  let dragWasRunning = false;
  let dragController = "";
  let saveContext = "";
  const stageIds = ["checking", "warmup", "baseline", "static", "training", "validation", "stopping", "saving", "analysis"];
  const elements = Object.fromEntries(V.editable.map(name => [name, $(name)]));
  const dragElements = {model: $("drag-calibration-file"), fraction: $("drag-fraction")};
  const text = (id, value) => { if ($(id).textContent !== String(value)) $(id).textContent = String(value); };
  const version = state => `${state.controller_id}:${state.generation}`;
  const rowValue = (row, field, columns) => columns.includes(field) ? row[columns.indexOf(field) + 1] : null;
  const calibrationConfig = state => state.calibration_configuration || state.configuration;
  const qualityLabel = run => run.automatic_qualified === false ? "自动指标未达标 · 待人工评估" : "自动指标达标";

  async function api(path, body) {
    const abort = new AbortController(), timer = setTimeout(() => abort.abort(), path.startsWith("/api/run?") ? 60000 : 5000);
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
      ["对应拖动记录", run?.save_test_run || "尚未完成"],
      ["自动质量评价", run ? qualityLabel(run) : "—"],
      ["验证 RMS / 门限", run ? `${V.number(run.quality?.validation_rmse, 4)} / ${V.number(run.quality?.rmse_limit, 4)} Nm` : "—"],
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
  function applyDraftValues(values) {
    for (const [name, value] of Object.entries(values)) {
      if (elements[name]) {
        if (V.bools.has(name)) elements[name].checked = value;
        else elements[name].value = value;
      } else stageDraft[name] = value;
    }
  }
  function fillForm(state, quick = false) {
    const config = calibrationConfig(state);
    options();
    for (const name of V.editable) {
      if (V.bools.has(name)) elements[name].checked = config[name] ?? name.startsWith("collect_");
      else elements[name].value = config[name] ?? (name === "quick_excitation" ? "fixed" : "");
    }
    if (elements.experiment.value === "compensate" || !elements.experiment.value) elements.experiment.value = "calibrate";
    dirty = false;
    stageDraft = {};
    formVersion = version(state);
    calibrationSignature = JSON.stringify(config); calibrationController = state.controller_id;
    quickDraftSeed = null; quickDraftEdited = false;
    if (quick) {
      const profile = V.quickProfile(stageSchema);
      const settings = Object.fromEntries(Object.entries(profile?.settings || {}).filter(([name]) =>
        V.editable.includes(name) || name === "protocol_mode"));
      applyDraftValues({...settings, experiment: "calibrate", calibration_file: "",
        collect_warmup: false, collect_static: false, collect_training: true, collect_validation: true});
      quickDraftSeed = quickProfileKey(profile);
      dirty = true;
    }
    hints();
  }
  function hints() {
    const precision = elements.experiment.value === "precision";
    $("quick-excitation-choice").hidden = !draftMode || elements.experiment.value !== "calibrate";
    $("calibration-choice").hidden = ["calibrate", "precision", "fourier", "low_speed"].includes(elements.experiment.value);
    $("calibration_file").required = !["calibrate", "precision", "fourier", "low_speed"].includes(elements.experiment.value);
    text("calibration-summary", precision ? "双向分档表 · 各步骤分别配置参数。"
      : elements.experiment.value === "low_speed" ? "正反向固定区间采样，不需要旧摩擦模型；速度档和重复数使用节点配置。"
      : ["fourier", "calibrate"].includes(elements.experiment.value)
        ? `摩擦模型：${V.models[elements.model.value] || elements.model.value} · 在“模型计算 → 参数”中调整。`
        : "此任务使用已有模型；轨迹 / 时长在对应采集步骤的参数中调整。");
  }
  function quickProfileKey(profile) {
    return profile ? JSON.stringify([snapshot?.state?.controller_id, profile.settings, profile.selection]) : null;
  }
  function quickProfileReason(profile) {
    const reason = profile?.selection?.reason || "";
    const hints = [
      ["No explicit local", "尚未配置匹配的验证来源"], ["Complete motor identity", "等待新鲜电机身份确认"],
      ["another host", "方案属于其他主机"], ["identity/coordinates/ranges", "电机身份、坐标或范围不匹配"],
      ["hash changed", "来源报告已变化"], ["context changed", "当前参数与验证来源不一致"],
      ["deferred", "运行结束后更新方案来源"], ["travel limits", "激励超出当前行程范围"],
      ["Reference speed bound", "激励超出当前速度范围"], ["No such file", "来源报告不可读"],
    ];
    return hints.find(([key]) => reason.includes(key))?.[1] || V.reason(reason);
  }
  function quickDraftSource(profile) {
    if (quickDraftEdited) return "自定义草稿 · 参数已修改，尚未验证。";
    if (!profile || profile.available === false) return "快速方案暂不可用。";
    if (quickDraftSeed !== quickProfileKey(profile)) return "未验证草稿 · 方案来源已变化，请重新载入快速预设。";
    if (profile.selection?.status === "validated_local") return "本机已验证方案 · 本轮仍须独立验证。";
    const reason = quickProfileReason(profile);
    return `未验证启动候选${reason ? `：${reason}` : ""}。`;
  }
  function changed() { dirty = true; quickDraftEdited = true; hints(); render(); }
  for (const element of Object.values(elements)) element.addEventListener("input", changed);
  elements.experiment.addEventListener("change", () => {
    if (!["calibrate", "precision", "fourier", "low_speed"].includes(elements.experiment.value) && !elements.calibration_file.value) {
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

  function draftConfiguration() {
    return draftMode ? {...calibrationConfig(snapshot.state), ...V.readValues(elements), ...stageDraft}
      : {...(viewed().state?.configuration || {})};
  }
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
        : V.stageEstimate(dialogStage, config, elements.experiment.value,
          draftMode ? Boolean(V.quickProfile(stageSchema)) : viewed().report?.quick?.protocol === V.quickProtocol));
      stageDialogError("");
    } catch (error) { stageDialogError(error.message); }
  }
  function openStageParameters(id) {
    if (!stageSchema || !(draftMode ? snapshot?.state : viewed().state)) { notice("阶段参数接口尚未就绪，请在失能后更新服务。", true); return; }
    const group = stageSchema.groups[id], config = draftConfiguration(), experiment = elements.experiment.value;
    const prepared = ["precision", "fourier"].includes(experiment);
    const names = [...new Set([...((prepared || id === "constraints") ? group.fields || [] : []), ...(group[experiment] || [])])];
    dialogStage = id; dialogVersion = snapshot?.state ? version(snapshot.state) : ""; stageInputs = {};
    text("stage-parameter-title", `${group.title} · 参数`);
    text("stage-parameter-note", !prepared && ["training", "validation", "static"].includes(id)
      ? experiment === "breakaway" ? "正反向各两次起动试验，时长为总预算；固定起动判据与力矩保护不变。"
        : experiment === "validate" ? "基线与补偿使用同一轨迹及增益；时长为每段时长，另计回位。手动拖动在独立测试模块启动。"
        : (draftMode ? Boolean(V.quickProfile(stageSchema)) : viewed().report?.quick?.protocol === V.quickProtocol)
          ? "快速机械标定先训练、后用不同轨迹独立验证；训练时长决定验证时长。仅覆盖本次实测速度域，保护边界不变。"
          : "历史机械辨识参数；按该实验原采集与验收语义展示，不补造独立验证。"
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
    applyDraftValues(values);
    changed();
    notice(test ? "已填入约 2 分钟 / 采集阶段的流程测试参数，尚未运动。静摩擦与定位时间由反馈决定；结果仅供流程诊断。"
      : "已恢复节点启动时的完整标定参数，尚未提交；固定保护与验收门限未改变。");
  }
  $("stage-profile-full").addEventListener("click", () => applyStageProfile(false));
  $("stage-profile-test").addEventListener("click", () => applyStageProfile(true));
  $("quick-profile").addEventListener("click", () => {
    if ($("quick-profile").disabled) return;
    fillForm(snapshot.state, true); $("confirm_hardware").checked = false; render();
  });

  async function control(start) {
    const state = snapshot?.state;
    if (!draftMode || !state || !snapshot.connected || connectionFailed || busy || pendingVersion ||
        (start ? $("start").disabled : $("apply").disabled)) return;
    if (dirty && formVersion !== version(state)) { notice("节点配置已变化，请先恢复节点配置后重新编辑。", true); return; }
    if (!$("settings").reportValidity()) return;
    busy = true; render();
    try {
      const body = {settings: {...V.readValues(elements), ...stageDraft}, controller_id: state.controller_id, generation: state.generation};
      for (const name of V.selectable) {
        const key = `collect_${name}`;
        if (!(key in state.configuration)) delete body.settings[key];
      }
      if (!("quick_excitation" in state.configuration)) delete body.settings.quick_excitation;
      if (start) body.confirm_hardware = $("confirm_hardware").checked;
      const response = await api(start ? "/api/start" : "/api/configure", body);
      notice(start ? "开始请求已接受，等待实际使能与采样；不是立即完成。"
        : "设置已应用到当前节点，未修改磁盘配置；电机保持失能。");
      dirty = false;
      stageDraft = {};
      pendingVersion = {controller_id: state.controller_id, generation: response.generation};
      if (start) { draftMode = false; followLatest = true; selectRun(""); }
    } catch (error) { notice(`操作未成功：${error.message}`, true); }
    finally { busy = false; render(); }
  }
  $("settings").addEventListener("submit", event => { event.preventDefault(); control(true); });
  $("confirm_hardware").addEventListener("change", render);
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
    $("constraint-edit").disabled = !draftMode || !connected || !state?.stop_confirmed || V.active.has(state?.phase) ||
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
    $("fault-reset-panel").hidden = !state?.motor?.host_fault && state?.phase !== "resetting";
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
    const shared = V.sharedRanges(model?.shared_ranges);
    text("drag-model-note", model
      ? `${qualityLabel(model)}。辨识速度包络 ${V.number(model.domain?.velocity_min, 3)}～${V.number(model.domain?.velocity_max, 3)} rad/s；` +
        `共有滑动范围（rad/s）：负向 ${V.sharedRangeText(shared.negative)}；正向 ${V.sharedRangeText(shared.positive)}。` +
        `验证 RMS ${V.number(model.quality?.validation_rmse, 4)} / 参考门限 ${V.number(model.quality?.rmse_limit, 4)} Nm。` +
        (model.quality_issues?.length ? ` 有 ${model.quality_issues.length} 项质量提示，请查看原报告后决定是否测试。` : "")
      : runs.some(run => run.eligible) ? "请选择模型并查看报告；自动未达标的候选也可由用户测试和评估。"
        : "当前没有身份匹配、参数完整有效的摩擦模型，不能开始补偿拖动。");
    const outside = model && state?.motor && (state.motor.velocity < model.domain?.velocity_min ||
      state.motor.velocity > model.domain?.velocity_max);
    const uncovered = model && V.finite(state?.motor?.velocity) && !V.speedInSharedRanges(state.motor.velocity, shared);
    const coverageNotice = !Object.keys(shared).length ? "未记录分方向共有滑动范围，无法确认当前速度的独立覆盖。"
      : uncovered ? "当前速度未落在训练与独立验证的共有滑动范围，未获独立覆盖。" : "";
    dragElements.model.title = model ? `${model.id} · ${V.models[model.model] || model.model}` : "请选择模型";
    $("drag-domain-alert").hidden = !(model && connected && (outside || coverageNotice) && dragRun && running);
    text("drag-domain-alert", `${outside ? "当前速度已超出模型速度包络。" : ""}${coverageNotice}`);
    text("drag-domain-note", model
      ? `${outside ? "当前速度已超出辨识包络。 " : ""}静止 / 死区内不主动补偿；域外模型查询会夹断，不代表域外精度已验证。` +
        " 共有范围按离散滑动窗口均速统计，包络内和零附近也可能缺少独立覆盖。" +
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
      followLatest = true; selectRun("");
      $("drag-details").open = true;
      dragNotice("拖动请求已接受，等待实际使能；标定表单和节点标定参数未改变。");
    } catch (error) { dragNotice(`拖动未成功：${error.message}`, true); }
    finally { busy = false; render(); }
  });

  function selectRun(id) {
    if (selectedRun === id && (detailKey || !id)) return;
    $("stage-parameter-dialog").close();
    selectedRun = id;
    if (id) draftMode = false;
    detail = null; detailKey = ""; formContext = "";
    chartCursor = null; motionCache = null;
    $("history").value = id;
    $("chart-window").value = id ? "0" : "60";
    if (id) loadRun(id);
    render(); queueCharts();
  }
  function selectLatest() {
    const id = V.latestExperiment(runs, snapshot?.state, snapshot?.report_motor);
    selectRun(id);
    if (!id && !snapshot?.state?.run_id && !runs.length && !dirty && !draftMode && snapshot?.state) {
      draftMode = true; fillForm(snapshot.state, true);
    }
  }
  async function history() {
    if (historyBusy) return;
    historyBusy = true; lastHistory = Date.now();
    try {
      const result = await api("/api/runs");
      runs = result.runs; historyLoaded = true;
      $("history").replaceChildren(new Option("当前实验", ""));
      for (const run of runs) {
        const label = `${run.id} · ${V.experiments[run.experiment] || "待保存"} · ${V.stages[run.status] || run.status}`;
        $("history").add(new Option(label, run.id));
      }
      if (followLatest && !draftMode && !pendingVersion) selectLatest();
      if (selectedRun && detail?.pending) { detailKey = ""; loadRun(selectedRun); }
      $("history").value = selectedRun;
      options(); render(); queueCharts();
      if (snapshot?.capabilities?.reanalysis) {
        analysisJob = await api("/api/reanalysis"); renderReanalysis();
      }
    } catch (error) { notice(`实验列表读取失败：${error.message}`, true); }
    finally { historyBusy = false; }
  }
  async function loadRun(id) {
    const key = id;
    if (detailKey === key) return;
    detailKey = key;
    try {
      const result = await api(`/api/run?run=${encodeURIComponent(id)}`);
      if (detailKey !== key) return;
      detail = result;
      lastFit = lastComparison = lastDownloads = lastPrecision = "";
      formContext = "";
      render(); queueCharts();
    } catch (error) {
      if (detailKey !== key) return;
      detail = null; detailKey = "";
      notice(`实验记录读取失败：${error.message}`, true);
      render(); queueCharts();
    }
  }
  $("history").addEventListener("change", () => {
    followLatest = false; draftMode = false;
    let id = $("history").value;
    if (id === snapshot?.state?.run_id && V.active.has(snapshot.state.phase)) id = "";
    selectRun(id); render();
  });
  $("live-view").addEventListener("click", () => {
    followLatest = true; draftMode = false; formContext = ""; selectLatest(); history(); render();
  });
  $("new-experiment").addEventListener("click", () => {
    if ($("new-experiment").disabled || !snapshot?.state) return;
    $("stage-parameter-dialog").close();
    draftMode = true; followLatest = false;
    fillForm(snapshot.state, true);
    $("experiment-tools").open = false;
    $("confirm_hardware").checked = false;
    render();
  });
  $("test-viewed-model").addEventListener("click", () => {
    const run = runs.find(item => item.id === viewed().state?.run_id && item.eligible);
    if (!run) return;
    dragElements.model.value = run.calibration_file; dragInitialized = true;
    $("parameter-save-model").value = run.id;
    $("drag-confirm-hardware").checked = false;
    $("drag-details").open = true;
    renderDrag(); renderParameterSave();
    $("drag-panel").scrollIntoView?.({behavior: "smooth", block: "nearest"});
  });
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
    $("reanalyze").disabled = !supported || !snapshot?.connected || !state?.stop_confirmed ||
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
    followLatest = false; draftMode = false; selectRun(analysisJob.run); history();
  });

  function viewed() {
    const live = snapshot?.state;
    const details = detail && detail.id === (selectedRun || live?.run_id) ? detail : null;
    const archived = Boolean(selectedRun || details);
    const state = archived ? V.experimentState(details) : live;
    return {state, details, archived, trace: archived ? details?.trace || [] : rows,
      fields: archived ? details?.display_fields || [] : fields,
      fit: details?.fit || (!archived ? live?.fit : null),
      report: details?.report || (!archived ? live?.report : null)};
  }
  function populateDL(id, entries) {
    $(id).replaceChildren();
    for (const [name, value] of entries) {
      const term = document.createElement("dt"), description = document.createElement("dd");
      term.textContent = name; description.textContent = value; $(id).append(term, description);
    }
  }
  function renderFit(fit, report) {
    const quick = report?.quick?.protocol === V.quickProtocol ? report.quick : null;
    const sweep = report?.experiment === "speed_sweep" ? report.speed_sweep : null;
    const issues = quick?.quality_issues || report?.[report.experiment]?.quality_issues || [];
    const diagnostics = V.speedSweepDiagnostics(report);
    const key = JSON.stringify([fit, report?.breakaway_fit, report?.status, issues, quick, sweep, diagnostics]);
    if (key === lastFit) return;
    lastFit = key;
    const displayed = report?.breakaway_fit || fit || quick?.candidate_model || sweep?.candidate_model;
    text("fit-model", displayed ? (V.models[displayed.model] || displayed.model) + (report?.status === "completed_unqualified" ? " · 候选" : "")
      : report?.status === "completed_unqualified" ? "采集完成 · 候选结果需验证" : "等待拟合");
    $("parameter-table").replaceChildren(); $("fit-warnings").replaceChildren(); $("fit-quality").replaceChildren();
    $("fit-empty").hidden = Boolean(displayed);
    text("fit-empty", report?.status === "completed_unqualified"
      ? "数据与候选结果已保存；质量指标有偏差，未发布合格补偿文件。"
      : "不会把未完成的拟合或默认值显示成标定结果。");
    const warnings = [...new Set([...issues, ...(displayed?.warnings || []), ...(sweep?.limitations || [])])];
    $("quality-details").hidden = !warnings.length;
    text("quality-summary", `质量说明 · ${warnings.length} 项`);
    for (const warning of warnings) {
      const item = document.createElement("li"); item.textContent = warning; $("fit-warnings").append(item);
    }
    $("fit-scope").hidden = !(quick || sweep);
    $("speed-sweep-details").hidden = !sweep;
    $("speed-sweep-cells").replaceChildren();
    $("speed-sweep-diagnostics").hidden = !diagnostics.length;
    text("speed-sweep-diagnostics-summary", `准备 / 启动诊断 · ${diagnostics.length} 项（不属于正式运动轮）`);
    $("speed-sweep-diagnostic-rows").replaceChildren();
    for (const values of diagnostics) {
      const row = document.createElement("tr");
      for (const value of values) {
        const cell = document.createElement("td"); cell.textContent = value; row.append(cell);
      }
      $("speed-sweep-diagnostic-rows").append(row);
    }
    if (sweep) {
      const validation = sweep.external_validation || {}, coverage = sweep.coverage || {};
      text("fit-model", `${displayed ? V.models[displayed.model] || displayed.model : "未得到候选"} · ${sweep.prediction_qualified ? "滑动预测通过" : "预测未通过"}`);
      populateDL("fit-quality", [
        ["独立验证 RMS / Nm", V.number(validation.rmse, 6)],
        ["独立验证最大误差 / Nm", V.number(validation.maximum_error, 6)],
        ["独立完成 / 计划轮次", `${coverage.complete_trials ?? "—"} / ${coverage.expected_trials ?? "—"}`],
        ["双向实测 ≥ 4π rad/s", coverage.highest_speed_reached_both_directions ? "通过" : "未通过"],
        ["验证速度包络 / rad/s", `${V.number(coverage.validated_velocity_min, 3)} ～ ${V.number(coverage.validated_velocity_max, 3)}`],
      ]);
      text("fit-scope", "独立轴稳速平台的滑动报告力矩候选。训练轮拟合，整轮独立验证只评分；速度包络不代表连续全域认证。惯量 J 和静摩擦未测量，生产参数未发布。重新联轴后的效果需另行验证。");
      for (const cell of sweep.cells || []) {
        const measured = (validation.errors || []).find(row => row.trial === cell.trial && row.direction === cell.direction);
        const row = document.createElement("tr");
        for (const content of [`${cell.trial} · ${cell.role === "validation" ? "独立验证" : "训练"}`,
          V.number(cell.speed * cell.direction, 3), V.number(cell.measured_speed, 4), V.number(cell.time_mean_torque, 5),
          V.number(measured?.error, 5), cell.valid ? "通过" : (cell.quality_issues || []).join("；") || "未通过"]) {
          const td = document.createElement("td"); td.textContent = content; row.append(td);
        }
        $("speed-sweep-cells").append(row);
      }
    }
    if (quick) {
      const validation = quick.external_validation || {}, coverage = quick.coverage || {};
      const shared = V.sharedRanges(coverage.directions);
      populateDL("fit-quality", [
        ["独立验证 RMS / Nm", V.number(validation.rmse, 6)],
        ["独立验证最大误差 / Nm", V.number(validation.maximum_error, 6)],
        ["独立验证窗口", validation.windows ?? "—"],
        ["负向共有滑动 / rad/s", V.sharedRangeText(shared.negative)],
        ["正向共有滑动 / rad/s", V.sharedRangeText(shared.positive)],
      ]);
      text("fit-scope", "后续不同轨迹独立评分，模型与积分窗口由训练段确定。共有范围按有效滑动窗口均速统计，不代表区间内每个速度都已验证。" +
        (Object.keys(shared).length === 2 ? "零附近及空白区域未获独立滑动覆盖。"
          : "部分或全部方向未记录可用共有范围，无法确认缺项速度的独立覆盖。") +
        "静摩擦、全速与热参数需相应高级流程。" +
        (quick.parameter_identifiability ? "惯量与各摩擦系数仍需独立验证；低残差不代表参数准确。" : ""));
    }
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
    const d = displayed.diagnostics, metrics = report?.fourier?.metrics, independent = d.independent_validation === 1;
    if (!(quick || sweep)) populateDL("fit-quality", displayed.model === "tabulated"
      ? [["速度档位 / 方向", `${d.speed_bins ?? displayed.friction_table?.length ?? "—"} × 2`], ["两向实测上限 / rad/s", V.number(d.validated_max_speed, 5)],
        ["独立验证 RMS / Nm", V.number(d.validation_rmse, 6)], ["最大验证误差 / Nm", V.number(d.max_bin_error, 6)]]
      : displayed.kind === "breakaway"
      ? [["起动次数", d.trials], ["重复性 RMS / Nm", V.number(d.repeatability_rmse, 6)]]
      : [[independent ? "独立验证窗口" : "拟合窗口", independent ? d.external_validation_windows ?? "—" : d.samples],
        [metrics || independent ? "独立验证 RMS / Nm" : "留出 RMS / Nm", V.number(metrics?.validation_rmse ?? d.validation_rmse, 6)],
        ["归一化条件数", V.number(metrics?.condition ?? d.condition, 2)], ["残差门限 / Nm", V.number(d.rmse_limit, 5)]]);
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
  function renderLowSpeed(view, state) {
    const active = state?.experiment === "low_speed";
    $("low-speed-panel").hidden = !active;
    $("model-panel").hidden = active;
    if (!active) return;
    const report = view.report?.low_speed, progress = state.low_speed || {};
    text("low-speed-progress", `${progress.completed_cells ?? 0}/${progress.required_cells ?? "—"} 段`);
    text("low-speed-summary", report
      ? `${report.acquisition_quality_passed ? "采集质量检查通过。" : "采集质量与未完成项目见下表。"}正反配对值是同位置区间、按角度加权的报告力矩半差；需结合稳速与测量通道检查，尚未发布补偿模型。`
      : "逐速、逐向采集，随后检查固定区间覆盖、稳速和重复性。每段实际采样都会保存。" );
    $("low-speed-pairs").replaceChildren();
    for (const pair of report?.pairs || []) {
      const tr = document.createElement("tr");
      for (const value of [pair.role === "validation" ? "独立验证" : "训练", V.number(pair.speed, 3),
        Number.isInteger(pair.repetition) ? pair.repetition + 1 : "—", V.number(pair.odd_resistance, 6),
        V.number(pair.common_core_odd_resistance, 6), pair.quality_passed ? "通过" : "未通过"]) {
        const td = document.createElement("td"); td.textContent = String(value); tr.append(td);
      }
      $("low-speed-pairs").append(tr);
    }
    const comparisons = (report?.pairs || []).filter(pair => Number.isFinite(pair.time_odd_reported_torque));
    $("low-speed-estimator-comparison").hidden = !comparisons.length;
    $("low-speed-estimator-rows").replaceChildren();
    for (const pair of comparisons) {
      const tr = document.createElement("tr");
      for (const value of [pair.role === "validation" ? "独立验证" : "训练", V.number(pair.speed, 3),
        Number.isInteger(pair.repetition) ? pair.repetition + 1 : "—", V.number(pair.time_odd_reported_torque, 6),
        V.number(pair.angle_minus_time_odd_reported_torque, 6)]) {
        const td = document.createElement("td"); td.textContent = String(value); tr.append(td);
      }
      $("low-speed-estimator-rows").append(tr);
    }
    $("low-speed-issues").replaceChildren();
    const issues = [...new Set([...(report?.quality_issues || []),
      ...(report?.cells || []).flatMap(cell => (cell.quality_issues || []).map(issue => `段 ${cell.index + 1}：${issue}`))])];
    const reasons = {acquisition_not_complete: "采集未完成", fewer_than_three_training_repetitions: "训练重复不足3次",
      cell_quality_failed: "部分速度段未通过质量检查", paired_quality_failed: "部分正反配对未通过",
      independent_validation_unavailable: "独立速度验证数据尚不完整", cell_not_completed: "该段未完成",
      not_settled_before_entry: "进入预定区间前尚未稳速", source_time_gap: "采集时间存在空档",
      position_ols_target_error: "实测平均速度偏离目标", position_ols_subwindow_ripple: "区间内速度起伏过大",
      command_saturation: "采集期间出现指令预算缩放", position_reversal: "区间内累计反向位置报告超过门限",
      unattributed_observations_in_interval: "区间内存在未能确认采集阶段的反馈"};
    for (const issue of issues) {
      let label = issue;
      for (const [code, meaning] of Object.entries(reasons)) label = label.replace(code, meaning);
      const li = document.createElement("li"); li.textContent = label; $("low-speed-issues").append(li);
    }
  }

  function renderPrecision(view, state) {
    const precise = state?.experiment === "precision";
    $("precision-panel").hidden = !precise;
    if (!precise) return;
    const final = view.report?.precision;
    const progress = state.precision;
    const recordedSettings = view.report?.configuration ? Object.fromEntries(
      Object.entries(view.report.configuration).filter(([name]) => name.startsWith("precision_"))
        .map(([name, value]) => [name.slice(10), value])) : null;
    const settings = final?.settings || state.precision_settings || recordedSettings || {};
    if (!settings.speeds?.length && !final?.levels?.length) {
      for (const id of ["speed-bins", "static-bins", "precision-issues"]) $(id).replaceChildren();
      lastPrecision = "";
      text("precision-quality", "该实验没有逐档测量数据。"); return;
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
      `${row.repetitions ?? "—"}/${row.required_repetitions ?? settings.repetitions ?? "—"}`, V.number(row.actual_speed, 5), V.number(row.mean_torque, 6),
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
    const live = snapshot?.state, connected = Boolean(snapshot?.connected && !connectionFailed);
    const running = Boolean(live && (V.active.has(live.phase) || live.stage === "stopping"));
    if (pendingVersion && live && (live.controller_id !== pendingVersion.controller_id || live.generation >= pendingVersion.generation)) {
      const kind = pendingVersion.kind;
      pendingVersion = null;
      if (kind !== "drag" && kind !== "reset") formVersion = "";
    }
    const view = viewed(), state = view.state;
    text("connection", connected ? "实时连接" : "未连接");
    $("connection").className = `badge ${connected ? "good" : "bad"}`;
    document.querySelector(".metrics").classList.toggle("stale", !connected || !live?.motor);
    const motor = live?.motor;
    text("source", motor ? `${connected ? "" : "上次："}${motor.simulated ? "物理仿真" : "真实电机"}` : "来源未知");
    $("source").className = `badge ${connected && motor?.simulated ? "good" : "warn"}`;
    text("motor-name", `${live?.configuration.motor_namespace || snapshot?.report_motor || "电机"}${motor ? ` · ${motor.motor_id}` : ""}`);
    text("controller-stage", live ? (connected ? "" : "上次状态 · ") +
      `${V.experiments[live.experiment] || live.experiment} · ${V.stages[live.phase] || live.phase}` : "等待标定器状态");
    $("controller-stage").title = $("controller-stage").textContent;
    $("controller-stage").className = `badge phase-badge ${!connected || live?.phase === "failed" ? "bad" : "warn"}`;
    text("disabled-state", !connected ? "失联 · 停机状态未知" : live?.stop_confirmed ? "已确认失能"
      : live?.phase === "running" ? "实验运行中" : "失能待确认");
    $("disabled-state").className = `badge ${!connected ? "bad" : live?.stop_confirmed ? "good" : "warn"}`;
    text("position-value", V.number(motor?.position)); text("velocity-value", V.number(motor?.velocity));
    text("torque-value", V.number(motor?.torque)); text("temperature-value", motor ? `${motor.rotor_temperature} / ${motor.mos_temperature}` : "—");
    const connectionMessage = !connected ? "控制器未连接，已保存的实验仍可查看。当前电机状态未知。"
      : snapshot.error || (live?.motor?.host_fault || live?.motor?.device_fault ? V.reason(motor.error || live.blocked_reason) : "");
    text("connection-notice", connectionMessage); $("connection-notice").hidden = !connectionMessage;
    $("new-experiment").disabled = !connected || running || busy || Boolean(pendingVersion) || !live?.stop_confirmed;
    $("new-experiment").hidden = draftMode;
    document.querySelector(".workspace").classList.toggle("viewing-record", !draftMode);
    text("control-title", draftMode ? "新实验" : "实验过程");
    $("calibration-summary").hidden = !draftMode;
    text("experiment-mode", draftMode ? "设置下一次实验，开始后独立保存。" : selectedRun ? "已保存的实验 · 参数只读" : "当前实验");
    if (draftMode && live) {
      const config = calibrationConfig(live);
      if (!busy && !pendingVersion && calibrationController === live.controller_id && calibrationSignature === JSON.stringify(config) && formVersion)
        formVersion = version(live);
      if (!dirty && !busy && !pendingVersion && formVersion !== version(live)) fillForm(live);
      formContext = "";
    } else if (state) {
      const context = `${state.run_id}:${JSON.stringify(state.configuration)}`;
      if (context !== formContext) {
        for (const name of V.editable) {
          if (V.bools.has(name)) elements[name].checked = state.configuration[name] ?? name.startsWith("collect_");
          else elements[name].value = state.configuration[name] ?? (name === "quick_excitation" ? "fixed" : "");
        }
        elements.experiment.value = state.experiment;
        formContext = context; hints();
      }
    } else if (!draftMode && formContext !== "empty") {
      for (const name of V.editable) if (V.bools.has(name)) elements[name].checked = false; else elements[name].value = "";
      formContext = "empty"; hints();
    }
    const conflict = draftMode && dirty && live && formVersion !== version(live);
    const locked = !draftMode || !connected || running || busy || Boolean(pendingVersion);
    $("editable").disabled = locked;
    $("advanced-editable").disabled = locked;
    $("start").disabled = locked || !snapshot?.services_ready || !live?.can_start || !live?.commands_ready || Boolean(conflict);
    if (motor?.simulated === false && !$("confirm_hardware").checked) $("start").disabled = true;
    const quick = elements.experiment.value === "calibrate", quickProfile = V.quickProfile(stageSchema);
    const automatic = quick && elements.quick_excitation.value === "auto";
    const autoSupported = Boolean(live?.configuration && "quick_excitation" in live.configuration);
    $("quick-excitation-auto").disabled = !autoSupported;
    $("low-speed-choice").disabled = !live?.low_speed_settings;
    if (draftMode && elements.experiment.value === "low_speed" && !live?.low_speed_settings) $("start").disabled = true;
    $("quick-excitation-choice").hidden = !draftMode || !quick;
    const quickUnavailable = draftMode && quick && (automatic ? !autoSupported : !quickProfile || quickProfile.available === false);
    if (quickUnavailable) $("start").disabled = true;
    $("draft-summary").hidden = !draftMode;
    const timing = quick && (quickProfile || automatic) ? V.quickTiming({duration: Number(elements.duration.value), quick_excitation: automatic ? "auto" : "fixed"}) : null;
    text("draft-method", V.experiments[elements.experiment.value] || "选择实验方法");
    $("draft-timing").hidden = !timing;
    if (timing) text("draft-timing", automatic ? "60 秒正式采集 + 预探测" : `约 ${V.number(timing.training + timing.validation, 0)} 秒`);
    text("draft-plan", quickUnavailable ? quickProfile?.selection?.reason
      ? `快速方案暂不可用：${quickProfileReason(quickProfile)}`
      : "当前节点尚未提供独立验证快速流程，请更新节点后开始。历史结果仍可查看。"
      : automatic ? "最多 2 × 10 秒预探测，选定后 40 秒训练 + 20 秒独立验证；另计启动、回位与停止确认。"
      : timing ? `${V.number(timing.training, 0)} 秒训练 + ${V.number(timing.validation, 0)} 秒独立验证；另计启动、回位与停止确认。`
        : elements.experiment.value === "low_speed"
          ? `训练速度 ${live?.low_speed_settings?.speeds?.join(" / ") || "—"} rad/s；正反向各 ${live?.low_speed_settings?.repetitions ?? "—"} 次，另有独立验证速度。总预算 ${live?.low_speed_settings?.time_budget ?? "—"} 秒，实际到达固定区间后采样。`
          : "自定义高级流程。开始前请核对采集范围、运行边界与质量要求。");
    text("draft-scope", quick ? (automatic ? "根据预探测选择有限候选方案；不放宽保护，不用最终验证调参。" : quickDraftSource(quickProfile)) + "仅覆盖本次实际速度域。静摩擦、全速与热参数需相应高级流程。"
      : "参数与阶段选择仅保存在草稿中，点击开始后才提交。");
    $("apply").disabled = locked || !snapshot?.services_ready || !live?.stop_confirmed || Boolean(conflict);
    $("reload-settings").disabled = locked;
    $("hardware-confirmation").hidden = !draftMode || !motor || motor.simulated;
    text("hardware-confirmation-text", ["precision", "fourier"].includes(elements.experiment.value)
      ? "已确认空载、可多圈旋转、无绕线风险及物理停止措施" : "已确认真机固定、可用行程及物理停止措施");
    const dirtyMessage = pendingVersion ? "请求已接受，等待控制器确认。" : conflict ? "节点配置已变化，请恢复配置后重新编辑。"
      : quickUnavailable ? "快速标定接口未就绪，开始按钮已停用。"
      : draftMode && dirty ? "设置将在开始时应用。" : draftMode && connected && !live?.can_start ? V.reason(live?.blocked_reason) : "";
    text("dirty", dirtyMessage); $("dirty").hidden = !dirtyMessage;
    const prepared = ["precision", "fourier"].includes(elements.experiment.value);
    const stageSupported = live && V.selectable.every(name => typeof live.configuration[`collect_${name}`] === "boolean");
    const parameterLocked = locked || !live?.stop_confirmed || !stageSchema;
    $("stage-profile-full").disabled = $("stage-profile-test").disabled = parameterLocked || !prepared;
    $("quick-profile").disabled = parameterLocked || !quickProfile || quickProfile.available === false;
    $("stage-parameter-save").disabled = parameterLocked || !Object.keys(stageInputs).length;
    for (const input of Object.values(stageInputs)) input.disabled = parameterLocked;
    if (stageInputs.known_bias && stageInputs.fit_bias) stageInputs.known_bias.disabled = parameterLocked || stageInputs.fit_bias.checked;
    $("identification-preset").disabled = $("validation-preset").disabled = parameterLocked;
    for (const id of stageIds) {
      $(`parameters-${id}`).disabled = !stageSchema || !state && !draftMode;
      if (id === "training" && draftMode && automatic) $(`parameters-${id}`).disabled = true;
      $(`parameters-${id}`).hidden = !draftMode && !stageSchema || elements.experiment.value === "low_speed" && id !== "constraints";
    }
    for (const name of V.selectable) elements[`collect_${name}`].disabled = locked || !prepared || !stageSupported;
    const config = draftMode && live ? {...calibrationConfig(live), ...stageDraft} : state?.configuration || {};
    const selection = Object.fromEntries(V.selectable.map(name => [`collect_${name}`, elements[`collect_${name}`].checked]));
    const omitted = ["static", "training", "validation"].filter(id => selection[`collect_${id}`] === false);
    text("protocol-mode", quick ? (draftMode ? quickProfile ? "快速机械标定 · 训练与独立验证" : "快速流程未就绪"
        : automatic && state?.samples === 0 ? "自动选择激励 · 尚无正式采集样本"
          : automatic || state?.report?.quick ? "快速机械标定 · 训练与独立验证" : "历史机械辨识 · 保留原采集与验收语义")
      : config.protocol_mode === "workflow_test" ? "流程诊断预设 · 不作为合格标定"
        : prepared && omitted.length ? "自定义采集方案 · 已跳过部分阶段"
          : prepared ? "完整采集方案 · 结果仍须质量验收" : "独立高级任务");
    const stepState = draftMode && live ? {...live, experiment: elements.experiment.value,
      configuration: {...config, duration: Number(elements.duration.value), quick_excitation: automatic ? "auto" : "fixed"},
      quick_adaptation: null,
      quick_calibration_plan: quickProfile ? {protocol: quickProfile.protocol} : null} : state;
    const steps = V.stageRows(stepState || {experiment:"",phase:"idle",stage:""}, selection, draftMode || !state);
    for (const step of steps) {
      $(`step-${step.id}`).hidden = !step.visible;
      $(`step-${step.id}`).className = step.kind;
      text(`step-name-${step.id}`, step.label); text(`step-state-${step.id}`, step.status); text(`step-detail-${step.id}`, step.detail);
    }
    const progress = draftMode || !state ? 0 : Math.round(100 * (V.completed.has(state.phase) ? 1 : state.progress || 0));
    text("stage-label", draftMode ? "实验步骤" : "实验进度");
    text("progress-text", draftMode ? "待开始" : state ? `${V.number(state.elapsed, 1)} s` : "—");
    $("progress-fill").style.width = `${progress}%`; $("progress").setAttribute("aria-valuenow", String(progress));
    $("progress").className = `progress-track ${draftMode ? "" : state?.phase === "failed" ? "failed" : state?.phase === "completed_unqualified" ? "warning" : V.completed.has(state?.phase) ? "completed" : ""}`;
    text("sample-count", !draftMode && state ? `${(state.samples || 0).toLocaleString()} 条原始样本` : "");
    const currentStep = steps.find(step => step.visible && ["current", "failed"].includes(step.kind));
    text("current-step", draftMode ? (timing ? `${automatic ? "预探测 → " : ""}训练采集 → 独立验证 → 停止确认 → 保存结果` : "阶段明细见高级设置。")
      : currentStep ? `${currentStep.label} · ${currentStep.status}`
        : state ? V.stages[state.phase] || state.phase : "等待实验记录");
    renderParameterSave(); renderDrag(); renderReanalysis();
    renderExperiment(view);
  }
  function renderExperiment(view) {
    const state = view.state, report = view.report, id = selectedRun || state?.run_id;
    text("experiment-title", state ? V.experiments[state.experiment] || state.experiment : selectedRun ? "正在读取实验" : "尚无实验");
    const status = {completed:"已完成",completed_unqualified:"有质量提示",qualified:"已完成",failed:"已中断",incomplete:"记录不完整",recorded:"待计算"};
    if (state?.experiment === "low_speed") status.completed_unqualified = "诊断已保存";
    if (state?.experiment === "speed_sweep") {
      status.completed = "滑动预测通过"; status.completed_unqualified = "候选与质量说明已保存";
    }
    text("experiment-status", state ? status[state.phase] || V.stages[state.stage] || state.phase : "等待记录");
    $("experiment-status").className = `badge ${state?.phase === "failed" ? "bad" : ["completed", "qualified"].includes(state?.phase) ? "good" : "warn"}`;
    text("view-notice", id ? `${id}${view.archived ? " · 已恢复实验记录" : " · 实时记录"}${view.details?.pending ? " · 文件尚未完成写入" : ""}`
      : historyLoaded ? "选择“新实验”开始。每次实验独立保存。" : "正在读取最近一次实验…");
    populateDL("experiment-facts", state ? [["电机", state.identity?.namespace || state.configuration.motor_namespace || "—"],
      ["采集时长", `${V.number(state.elapsed,1)} s`], ["原始样本", (state.samples || 0).toLocaleString()],
      ["环境", typeof state.identity?.simulated === "boolean" ? state.identity.simulated ? "仿真" : "真机" : "—"]] : []);
    const adaptation = V.quickAdaptation(state);
    $("quick-adaptation-summary").hidden = !adaptation;
    text("quick-adaptation-summary", adaptation ? adaptation.text + (Number.isFinite(state.recorded_samples)
      ? ` 正式 ${state.samples || 0} / 全部 ${state.recorded_samples} 条记录。` : "") : "");
    const error = [state?.error, view.details?.model_error, view.details?.trace_error].filter(Boolean).join("；");
    text("result-error", (error && state?.failure_stage ? `${V.stages[state.failure_stage] || state.failure_stage}：` : "") + V.reason(error));
    $("result-error").hidden = !error;
    const reportAvailable = id && V.terminal.has(state?.phase);
    $("html-report-link").hidden = !reportAvailable;
    if (reportAvailable) $("html-report-link").href = `/api/report?${new URLSearchParams({run: id})}`;
    $("test-viewed-model").hidden = !runs.some(run => run.id === id && run.eligible);
    renderFit(view.fit, report); renderComparison(report); renderPrecision(view, state); renderLowSpeed(view, state);
    $("comparison-panel").hidden = !report?.comparison;
    const events = state?.events || [];
    text("events-note", report?.events_truncated ? `显示最近 ${events.length} 条事件；完整记录见 events.csv。` : "");
    const eventKey = JSON.stringify(events);
    if (eventKey !== lastRenderedEvents) {
      lastRenderedEvents = eventKey; $("events").replaceChildren();
      for (const event of events) {
        const item = document.createElement("li"), time = document.createElement("time");
        time.textContent = `${V.number(event.elapsed,1)} s`;
        item.append(time, document.createTextNode((V.stages[event.stage] || event.stage) + (event.error ? ` · ${V.reason(event.error)}` : "")));
        $("events").append(item);
      }
    }
    const configurationKey = JSON.stringify([state?.configuration, state?.requested_configuration, stageSchemaController]);
    if (configurationKey !== lastConfiguration) {
      lastConfiguration = configurationKey;
      populateDL("experiment-configuration", Object.entries(state?.configuration || {}).map(([name,value]) =>
        [stageSchema?.fields?.[name]?.label || name, typeof value === "object" ? JSON.stringify(value) : String(value)]));
      $("requested-configuration-details").hidden = !state?.requested_configuration;
      populateDL("requested-configuration", Object.entries(state?.requested_configuration || {}).map(([name,value]) =>
        [stageSchema?.fields?.[name]?.label || name, typeof value === "object" ? JSON.stringify(value) : String(value)]));
    }
    const recording = report?.recording_status || state?.recording_status;
    text("recording-status", recording?.error ? `记录异常：${recording.error}` : recording?.recording_complete ? "原始记录已保存。"
      : recording ? "记录进行中，停止后保存。" : view.details ? "文件按实验独立保存，原始数据可下载。" : "");
    const downloadsKey = JSON.stringify([view.details?.id, view.details?.artifacts]);
    if (downloadsKey !== lastDownloads) {
      lastDownloads = downloadsKey; $("downloads").replaceChildren();
      for (const filename of view.details?.artifacts || []) {
        const link = document.createElement("a");
        link.href = `/api/artifact?${new URLSearchParams({run:view.details.id,file:filename})}`;
        link.download = `${view.details.id}-${filename}`; link.textContent = filename; $("downloads").append(link);
      }
    }
  }

  function chart(id, series, axis, scatter = false, emptyLabel = "等待真实采样数据") {
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
    if (!domain) { ctx.fillStyle = "#8ea9bf"; ctx.textAlign = "center"; ctx.fillText(emptyLabel, width / 2, height / 2); return; }
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
        ctx.beginPath(); let previous = null, previousSegment = null;
        for (const p of item.points) {
          if (!V.finite(p[0]) || !V.finite(p[1])) { previous = null; continue; }
          if (previous === null || item.segmented && p[2] !== previousSegment || (!scatter && p[0] - previous > (item.maxGap || 0.5))) ctx.moveTo(x(p[0]), y(p[1]));
          else ctx.lineTo(x(p[0]), y(p[1]));
          previous = p[0]; previousSegment = p[2];
        }
        ctx.stroke();
      }
    }
    ctx.restore();
  }
  function drawCharts() {
    chartQueued = false;
    const view = viewed(), trace = view.trace, fields = view.fields;
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
        points: selection.rows.map(row => [row[timeIndex], rowValue(row, field, fields), rowValue(row, "trial_index", fields)]), color, dash,
        segmented: fields.includes("trial_index"),
        maxGap: selection.sampled ? Infinity : Math.max(0.5, interval * 3)});
      chart("position-chart", [line("position", V.colors.measured), line("target_position", V.colors.target, true)], "s");
      chart("velocity-chart", [line("velocity", V.colors.measured), line("target_velocity", V.colors.target, true)], "s");
      chart("torque-chart", [line("torque", V.colors.measured), line("command_torque", V.colors.target, true),
        line("feedforward", V.colors.feedforward)], "s");
      text("chart-window-info", selection.rows.length
        ? `记录 ${V.number(selection.start, 1)}–${V.number(selection.end, 1)} s`
        : view.archived ? "该实验没有可用运动记录" : "等待采样");
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
        ? "整圈实测均值与模型曲线；误差棒表示重复测量的统计界限。"
        : plot.kind === "corrected_windows"
        ? `已扣除惯量与负载 · ${plot.window_count} 个积分窗`
        : plot.kind === "independent_speed_sweep"
        ? "训练平台与整轮独立验证的实测速度—报告力矩；模型线仅表示滑动候选，未测区间不代表已验证。"
        : "本实验使用的摩擦模型");
      if (view.details.plot_error) text("friction-caption", view.details.plot_error);
      if (view.report?.quick) {
        const shared = V.sharedRanges(view.report.quick.coverage?.directions);
        curves = V.sharedModelCurves(curves, shared);
        text("friction-caption", $("friction-caption").textContent + (Object.keys(shared).length
          ? "；模型实线仅画在已记录的共有滑动范围内，空白区域未获独立覆盖。"
          : "；未记录分方向共有滑动范围，不绘制模型实线。"));
      }
    } else if (view.state?.experiment === "precision") {
      points = (view.state.precision?.observations || []).map(row => [row.measured_speed, row.torque]);
      text("friction-caption", "已完成速度档的实测均值");
    } else {
      points = motionCache.selection.rows.map(row => [rowValue(row, "velocity", fields), rowValue(row, "torque", fields)]);
      text("friction-caption", "原始速度—力矩观测，包含惯量与负载。");
    }
    chart("friction-chart", [
      {points: plot?.kind === "independent_speed_sweep" ? plot.training_points : points.filter(p => p[0] >= 0), color: V.colors.measured, errorbars: plot?.kind === "precision_steady"},
      {points: plot?.kind === "independent_speed_sweep" ? plot.validation_points : points.filter(p => p[0] < 0), color: V.colors.target, errorbars: plot?.kind === "precision_steady"},
      ...curves.map(points => ({points, color: V.colors.fit, line: true, width: 2.2})),
    ], "rad/s", true, view.report?.quick && !Object.keys(V.sharedRanges(view.report.quick.coverage?.directions)).length
      ? "未记录共有滑动范围" : undefined);
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
      if (data.state && data.capabilities?.stage_parameters && (!stageSchema || stageSchemaController !== data.state?.controller_id)) {
        stageSchema = await api("/api/stage-parameters");
        stageSchemaController = data.state.controller_id;
      }
      if (stageSchema && data.state && Object.hasOwn(data.state, "quick_profile")) {
        stageSchema = {...stageSchema, quick_profile: data.state.quick_profile};
        const profile = V.quickProfile(stageSchema);
        // A newly fresh binding may become available after the first page load.
        // Only an untouched quick draft follows it; never replace user edits.
        if (draftMode && !quickDraftEdited && !busy && !pendingVersion &&
            elements.experiment.value === "calibrate" && profile?.selection?.status === "validated_local" &&
            profile.available !== false && quickDraftSeed !== quickProfileKey(profile)) fillForm(data.state, true);
      }
      if (followLatest && !draftMode && !pendingVersion && historyLoaded) selectLatest();
      render(); queueCharts();
      if (Date.now() - lastHistory > 5000) history();
      if (selectedRun && !detailKey) loadRun(selectedRun);
    } catch (error) {
      connectionFailed = true;
      render();
      text("connection-notice", `控制器连接中断：${error.message}。已保存的实验仍可查看。`);
      $("connection-notice").hidden = false;
      if (Date.now() - lastHistory > 5000) history();
    } finally { setTimeout(poll, 200); }
  }
  poll();
})();
