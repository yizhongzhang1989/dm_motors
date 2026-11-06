"use strict";

const CalibrationView = (() => {
  const stages = {
    idle: "等待实验", checking: "就绪与能力检查", arming: "等待实际使能确认",
    resetting: "主机故障复位 · 等待新的失能反馈",
    acquisition: "多频激励 · 采集实际反馈", baseline: "A/B · 无补偿基线",
    quick_training: "快速标定 · 训练采集", quick_validation: "快速标定 · 独立验证",
    quick_pilot: "自动激励 · 预探测", quick_pilot_return: "自动激励 · 回位确认",
    returning: "回位并等待静止", compensated: "A/B · 加入摩擦补偿",
    assistance: "低阻力拖动助力", settling: "静摩擦试验 · 回位 / 准备",
    conditioning: "静摩擦试验 · 同向接近基准", neutral_dwell: "静摩擦试验 · 撤除预载并静置",
    ramp_1: "起动试验 1 / 4 · 正转", ramp_2: "起动试验 2 / 4 · 反转",
    ramp_3: "起动试验 3 / 4 · 正转", ramp_4: "起动试验 4 / 4 · 反转",
    stopping: "停止输出 · 等待失能反馈", fitting: "已失能 · 辨识 / 评估与保存",
    saving_failure: "保存中断 / 失败现场", running: "自动采样 / 实验执行",
    completed: "实验完成 · 结果已保存", failed: "实验失败 / 已中断",
    completed_unqualified: "采集完成 · 有质量提示",
    recorded: "原始数据已保存 · 待计算", qualified: "实验完成", incomplete: "实验记录不完整",
    precision_await_warmup_enable: "预运行 / 预热 · 等待使能",
    precision_warmup_ramp: "预运行 / 预热 · 平滑变速",
    precision_warmup_settle: "预运行 / 预热 · 稳速准备",
    precision_warmup_sample: "预运行 / 预热 · 双向实测整圈旋转",
    precision_warmup_brake: "预运行 / 预热 · 平滑制动",
    precision_warmup_home: "预运行 / 预热 · 回到控制零位",
    precision_warmup_sine_approach: "预运行 / 预热 · 定位到正弦起点",
    precision_warmup_sine: "预运行 / 预热 · 变频率正弦往复",
    precision_warmup_sine_hold: "预运行 / 预热 · 确认正弦往复完成",
    precision_warmup_final_home: "预运行 / 预热 · 正弦结束回零",
    precision_warmup_complete: "预运行完成 · 等待失能后重测零偏",
    precision_baseline: "精密标定 · 失能力矩零偏",
    precision_await_enable: "精密标定 · 等待使能",
    precision_static_approach: "静摩擦 · 同向接近各角度基准",
    precision_static_origin: "静摩擦 · 精确回到角度基准",
    precision_static_neutral: "静摩擦 · 撤预载静置",
    precision_static_ramp: "静摩擦 · 缓慢增矩与持续起动确认",
    precision_static_brake: "静摩擦 · 制动并确认静止",
    precision_review_static: "静摩擦 · 已失能，记录统计后继续扫描",
    precision_ramp: "精密速度扫描 · 平滑升速",
    precision_settle: "精密速度扫描 · 等待实测速度稳定",
    precision_sample: "精密速度扫描 · 整圈稳态采样",
    precision_brake: "精密速度扫描 · 平滑制动",
    precision_home: "精密速度扫描 · 回到标定基准",
    precision_done: "精密速度扫描完成 · 等待最终验收",
    precision_qualification: "全速度与静摩擦精度验收",
  };
  for (const [name, label] of Object.entries(stages)) {
    if (name.startsWith("precision_")) stages[name.replace("precision_", "fourier_")] = label;
  }
  stages.fourier_periodic = "傅里叶激励 · 多谐波周期运动";
  stages.fourier_home = "傅里叶激励 · 定位 / 最终回零";
  const experiments = {precision: "精密全速摩擦", fourier: "傅里叶摩擦辨识", calibrate: "快速机械标定", validate: "补偿 A/B", breakaway: "静摩擦起动", compensate: "拖动助力", low_speed: "短行程低速摩擦", speed_sweep: "独立轴高速滑动摩擦"};
  for (const [phase, label] of Object.entries({home: "定位", ramp: "速度过渡", transit: "稳速接近", sample: "固定区间采样", brake: "减速", final_home: "返回起点", done: "采集结束"}))
    stages[`low_speed_${phase}`] = `低速摩擦 · ${label}`;
  const models = {auto: "自动选择", viscous: "黏性", coulomb_viscous: "库仑 + 黏性",
    stribeck: "Stribeck", asymmetric_coulomb_viscous: "非对称库仑 + 黏性", asymmetric_stribeck: "非对称 Stribeck",
    tabulated: "精密分档表（双向）", coulomb_viscous_bias: "滑动报告力矩 · 库仑 + 黏性 + 偏置"};
  const editable = ["experiment", "model", "calibration_file", "quick_excitation", "duration", "amplitude", "frequency", "kp", "kd",
    "fit_bias", "known_bias", "fit_gravity", "fit_stiffness", "compensation_fraction",
    "collect_warmup", "collect_static", "collect_training", "collect_validation"];
  const selectable = ["warmup", "static", "training", "validation"];
  const bools = new Set(["fit_bias", "fit_gravity", "fit_stiffness", ...selectable.map(name => `collect_${name}`)]);
  const strings = new Set(["experiment", "model", "calibration_file", "quick_excitation"]);
  const active = new Set(["checking", "arming", "running", "returning", "stopping", "fitting", "resetting"]);
  const completed = new Set(["completed", "completed_unqualified", "qualified"]);
  const terminal = new Set([...completed, "failed", "incomplete", "recorded"]);
  const maxChartPoints = 1200;
  const colors = {measured: "#69d6df", target: "#b3a0ff", feedforward: "#f2bc67", fit: "#72dfa8"};
  const finite = value => typeof value === "number" && Number.isFinite(value);
  const number = (value, digits = 4) => finite(value) ? value.toFixed(digits) : "—";
  function sharedRanges(directions) {
    const result = {};
    for (const direction of ["negative", "positive"]) {
      const branch = directions?.[direction], lower = branch?.velocity_min, upper = branch?.velocity_max;
      if (!finite(lower) || !finite(upper) || !(lower < upper) ||
          !(direction === "negative" ? upper < 0 : lower > 0) ||
          !["training", "external_validation"].every(role => Number.isInteger(branch?.windows?.[role]) && branch.windows[role] > 0)) continue;
      result[direction] = branch;
    }
    return result;
  }
  function sharedRangeText(branch) {
    return branch ? `${number(branch.velocity_min, 3)} ～ ${number(branch.velocity_max, 3)}；训练 ${branch.windows.training} / 独立验证 ${branch.windows.external_validation} 窗`
      : "未记录可用共有范围";
  }
  function speedInSharedRanges(speed, ranges) {
    return finite(speed) && Object.values(sharedRanges(ranges)).some(branch => speed >= branch.velocity_min && speed <= branch.velocity_max);
  }
  function sharedModelCurves(curves, ranges) {
    return (curves || []).flatMap(points => Object.values(sharedRanges(ranges)).map(branch =>
      points.filter(point => finite(point[0]) && finite(point[1]) && point[0] >= branch.velocity_min && point[0] <= branch.velocity_max)))
      .filter(points => points.length > 1);
  }
  const quickProtocol = "quick_independent_v1";
  function speedSweepDiagnostics(report) {
    if (report?.experiment !== "speed_sweep") return [];
    return (report.excluded_diagnostic_trials || []).map(item => [
      item.trial || "未命名", item.had_enable_request === true ? "已请求" : item.had_enable_request === false ? "未请求" : "未记录",
      `${item.commands ?? "—"} 条${item.startup_only === true ? " baseline 起点保持目标" : "目标"}`,
      item.stop_confirmed === true ? "已确认" : "未确认", (item.errors || []).join("；"),
    ]);
  }
  function quickProfile(schema) {
    const profile = schema?.quick_profile;
    return profile?.id === "quick_mechanical" && profile.protocol === quickProtocol &&
      profile.settings?.experiment === "calibrate" && finite(profile.settings.duration) &&
      profile.settings.duration > 0 && finite(profile.training_seconds) && profile.training_seconds > 0 &&
      finite(profile.validation_seconds) && profile.validation_seconds > 0 ? profile : null;
  }
  function quickTiming(config) {
    if (config?.quick_excitation === "auto") return {training: 40, validation: 20};
    const training = Number(config?.duration);
    return finite(training) && training > 0
      ? {training, validation: Math.max(16, Math.min(24, training / 2))} : null;
  }
  function quickAdaptation(state) {
    if (state?.experiment !== "calibrate") return null;
    const snapshot = state.quick_adaptation || state.report?.quick_adaptation;
    const mode = state.configuration?.quick_excitation ?? "fixed";
    if (!snapshot) return mode === "auto" ? {text: "自动选择 · 等待预探测记录；尚未选定正式方案。", selected: null} : null;
    const selected = snapshot.selected_candidate, settings = snapshot.candidates?.[selected]?.formal_settings;
    const count = Array.isArray(snapshot.attempts) ? snapshot.attempts.length : 0;
    const phase = {probe_ready: "等待预探测", return_required: "等待回位确认", selected: "已选定",
      rejected: "未找到可用方案", aborted: "已中止"}[snapshot.status] || snapshot.status || "等待记录";
    const status = stages[state.stage] || phase;
    const recipe = settings ? `幅值 ${number(settings.amplitude, 3)} rad · ${number(settings.frequency, 3)} Hz · Kp ${number(settings.kp, 2)} / Kd ${number(settings.kd, 2)}` : "";
    const reason = Array.isArray(snapshot.abort_reasons) ? snapshot.abort_reasons.map(reason => String(reason)).join("；") : "";
    return {selected, settings, attempts: count,
      text: selected && settings ? `自动选择 ${selected} · ${recipe}。完成 ${count} 段预探测；正式结果仍须独立验证。`
        : `自动选择 · ${terminal.has(state.phase) ? phase : status} · 已记录 ${count}/2 段预探测。尚未选定正式方案。${reason}`};
  }
  function experimentState(detail) {
    if (!detail?.report) return null;
    const report = detail.report, saved = report.dashboard_state || {}, config = {quick_excitation: "fixed", ...report.configuration};
    const experiment = report.experiment || "", phase = report.status || "incomplete";
    const settings = prefix => Object.fromEntries(Object.entries(config)
      .filter(([name]) => name.startsWith(prefix)).map(([name, value]) => [name.slice(prefix.length), value]));
    const analysis = report[experiment] || {};
    const precision = saved.precision || report[`${experiment}_progress`] || {
      phase: terminal.has(phase) ? "done" : "", warmup: report.warmup || analysis.warmup,
      static_diagnostics: analysis.static, observations: analysis.observations || [],
      completed_cells: analysis.measured_cells, cells: analysis.required_cells,
    };
    const traceFields = detail.display_fields || [], trace = detail.trace || [];
    const timeColumn = traceFields.indexOf("time") >= 0 ? traceFields.indexOf("time") + 1
      : traceFields.indexOf("elapsed") >= 0 ? traceFields.indexOf("elapsed") + 1 : -1;
    const times = (report.events || []).map(event => event.elapsed).filter(finite);
    if (trace.length && timeColumn > 0) times.push(trace[trace.length - 1][timeColumn]);
    return {...saved, run_id: detail.id, experiment, phase, stage: phase,
      configuration: config, calibration_configuration: config, identity: report.identity,
      elapsed: finite(saved.elapsed) ? saved.elapsed : times.length ? Math.max(...times) : null,
      samples: report.samples ?? detail.raw_samples ?? 0, events: report.events || [],
      stop_confirmed: report.stop_confirmed === true, error: report.error || "",
      failure_stage: report.failure_stage, report, fit: detail.fit, precision,
      quick_calibration_plan: report.quick_plan || saved.quick_calibration_plan || null,
      quick_adaptation: report.quick_adaptation || saved.quick_adaptation || null,
      requested_configuration: report.requested_configuration || saved.requested_configuration || null,
      recorded_samples: report.recorded_samples ?? saved.recorded_samples ?? report.samples ?? 0,
      precision_settings: saved.precision_settings || report.precision?.settings || settings("precision_"),
      fourier_settings: saved.fourier_settings || report.fourier?.settings || settings("fourier_"),
      low_speed: saved.low_speed || report.low_speed_progress || null,
      low_speed_settings: saved.low_speed_settings || report.low_speed?.settings || settings("low_speed_"),
      progress: completed.has(phase) ? 1 : saved.progress ?? 0};
  }
  function latestExperiment(runs, state, motor = "") {
    if (state?.run_id && (active.has(state.phase) || state.stage === "stopping")) return "";
    const namespace = state?.identity?.namespace || state?.configuration?.motor_namespace || motor;
    const matching = runs.filter(run => run.same_motor !== false &&
      (!namespace || !run.identity?.namespace || run.identity.namespace === namespace));
    return matching.map(run => run.id).concat(state?.run_id ? [state.run_id] : []).sort().at(-1) || "";
  }
  function parseStageValue(text, spec) {
    if (spec.type === "boolean") {
      if (typeof text !== "boolean") throw new Error(`${spec.label} 需要明确的勾选值`);
      return text;
    }
    const raw = String(text).trim();
    if (!raw) throw new Error(`${spec.label} 不能为空`);
    if (spec.type === "choice") {
      if (!spec.choices.includes(raw)) throw new Error(`${spec.label} 的选项无效`);
      return raw;
    }
    let value;
    if (spec.type === "array") {
      value = raw.startsWith("[") ? JSON.parse(raw) :
        (raw.includes(",") || raw.includes("，") ? raw.split(/[,，]/) : raw.split(/\s+/)).map(token => {
          if (!token.trim()) throw new Error(`${spec.label} 存在空数组项`);
          return Number(token);
        });
      if (!Array.isArray(value) || value.length < 3 || value.length > 40)
        throw new Error(`${spec.label} 需要 3–40 个数值`);
    } else value = Number(raw);
    if ((Array.isArray(value) ? value : [value]).some(item => !finite(item) || item < spec.min || item > spec.max))
      throw new Error(`${spec.label} 必须在 ${spec.min} 到 ${spec.max} 之间`);
    if (spec.type === "integer" && !Number.isInteger(value)) throw new Error(`${spec.label} 必须为整数`);
    return value;
  }
  function stageEstimate(stage, config, experiment, quick = false) {
    if (quick && experiment === "calibrate" && ["training", "validation"].includes(stage)) {
      const timing = quickTiming(config);
      return timing ? stage === "training"
        ? `训练 ${number(timing.training, 1)} s；仅训练数据用于拟合、模型选择与积分窗口选择。`
        : `独立验证 ${number(timing.validation, 1)} s；使用后续不同轨迹，冻结模型后评分，不重新拟合。`
        : "等待有效的快速标定计划。";
    }
    if (!["precision", "fourier"].includes(experiment)) {
      return ["training", "validation", "static"].includes(stage)
        ? `有限时长 ${number(config.duration, 1)} s${experiment === "validate" ? " / 段；两段之间确认回位与静止" : ""}，实际运动仍受固定保护边界限制。`
        : "参数只作用于所选标定 / 验证任务；拖动测试在独立模块启动。";
    }
    if (stage === "baseline") return `失能采集 ${number(config.precision_baseline_settle_time + config.precision_baseline_time, 1)} s（等待 + 统计）。`;
    if (stage === "static") return `${config.precision_static_angles?.length ?? "—"} 个角度 × 双向 × ` +
      `${config.precision_static_repetitions} 次测量；另加每方向 ${config.precision_static_conditioning_repetitions} 次预处理。耗时由实测起动和回位决定。`;
    if (stage === "warmup") {
      const sine = 2 * config.precision_warmup_sine_cycles /
        (config.precision_warmup_sine_start_frequency + config.precision_warmup_sine_end_frequency);
      return `${config.precision_warmup_repetitions} 轮预运行；每方向 ${config.precision_warmup_turns_per_direction} 圈。` +
        `正弦 ${config.precision_warmup_sine_cycles} 周期约 ${number(sine, 1)} s；另计旋转、斜坡、定位和停稳。`;
    }
    if (["training", "validation"].includes(stage) && experiment === "fourier") {
      const validation = stage === "validation";
      const cycles = validation
        ? config.fourier_slow_validation_cycles + config.fourier_validation_cycles + 2 * config.fourier_validation_settle_cycles
        : config.fourier_slow_training_cycles + config.fourier_training_cycles + 2 * config.fourier_settle_cycles;
      const seconds = cycles / (config.fourier_frequency * (validation ? config.fourier_validation_frequency_ratio : 1));
      return `${cycles} 个周期（含过渡）· 计划 ${number(seconds, 1)} s；另计定位、回零与失能确认。`;
    }
    if (["training", "validation"].includes(stage) && experiment === "precision")
      return `${config.precision_speeds?.length ?? "—"} 个主速度档，训练每方向 ${config.precision_repetitions} 次；验证使用相邻主档的中间速度。必须完成实测整圈，不以等待时间替代采集。`;
    if (stage === "stopping") return "立即停止输出并等待实际失能，不为满足测试时长而延后停止。";
    if (stage === "saving") return "全过程完整记录，停止后提交哈希清单；保存耗时取决于实际数据量。";
    if (stage === "analysis") return "确认失能且原始记录提交后计算；测试模式不发布合格补偿文件。";
    return `实验总预算 ${number(config.precision_time_budget, 0)} s；安全检查按实际反馈完成，不人为延时。`;
  }
  function readDragValues(elements) {
    const calibration_file = elements.model.value;
    if (!calibration_file.trim()) throw new Error("请选择身份匹配的合格摩擦模型");
    const compensation_fraction = parseStageValue(elements.fraction.value, {label: "补偿比例", type: "number", min: 0, max: 1});
    return {calibration_file, compensation_fraction};
  }
  function stageRows(state, selections, planning = false) {
    if (state.experiment === "speed_sweep") {
      const report = state.report?.speed_sweep || {}, coverage = report.coverage || {};
      const labels = {checking: "独立轴检查", training: "双向训练轮", validation: "整轮独立验证", stopping: "停止确认", saving: "原始记录归档", analysis: "滑动预测评价"};
      return ["checking", "pilot", "warmup", "baseline", "static", "training", "validation", "stopping", "saving", "analysis"].map(id => ({
        id, visible: Object.hasOwn(labels, id), label: labels[id] || id,
        kind: id === "analysis" ? report.prediction_qualified ? "done" : "warning"
          : id === "stopping" ? state.stop_confirmed ? "done" : "warning"
            : report.acquisition_quality_passed ? "done" : "warning",
        status: id === "analysis" ? report.prediction_qualified ? "预测通过" : "查看质量说明"
          : id === "stopping" ? state.stop_confirmed ? "已确认" : "未确认" : "已归档",
        detail: id === "training" || id === "validation" ? `${coverage.complete_trials ?? "—"}/${coverage.expected_trials ?? "—"} 独立轮次；验证轮不参与拟合`
          : id === "analysis" ? "高速滑动候选；惯量和静摩擦未测量，未发布生产参数。" : "",
      }));
    }
    if (state.experiment === "low_speed") {
      const progress = state.low_speed || {}, report = state.report?.low_speed;
      const phase = planning ? "idle" : state.phase, done = completed.has(phase), failed = phase === "failed";
      const role = progress.cells?.[progress.cell]?.role;
      const current = ["checking", "arming", "idle"].includes(phase) ? "checking"
        : state.stage === "stopping" ? "stopping" : phase === "fitting" || done ? "analysis"
          : role === "validation" ? "validation" : "training";
      const visible = ["checking", "training", "validation", "stopping", "analysis"];
      const labels = {checking: "启动检查", training: "双向低速采集", validation: "独立速度采集", stopping: "停止确认", analysis: "配对质量分析"};
      return ["checking", "pilot", "warmup", "baseline", "static", "training", "validation", "stopping", "saving", "analysis"].map(id => ({
        id, visible: visible.includes(id), label: labels[id] || id,
        kind: failed && current === id ? "failed" : done ? (id === "analysis" ? "warning" : "done") : current === id && !planning ? "current" : "",
        status: planning ? "待开始" : done ? (id === "analysis" ? "诊断已保存" : "已完成") : failed && current === id ? "中止" : current === id ? "进行中" : "等待",
        detail: id === "training" || id === "validation" ? `${progress.completed_cells ?? 0}/${progress.required_cells ?? "—"} 段 · 正反方向经过同一预定区间`
          : id === "analysis" ? (report ? "局部配对阻力观测；未发布补偿模型。" : "保留完整数据，检查稳速、重复性及位置影响。") : "",
      }));
    }
    const prepared = ["precision", "fourier"].includes(state.experiment), fourier = state.experiment === "fourier";
    const quick = state.experiment === "calibrate" &&
      (state.quick_calibration_plan?.protocol === quickProtocol || state.report?.quick?.protocol === quickProtocol ||
       state.quick_adaptation || state.configuration?.quick_excitation === "auto");
    const automatic = quick && (state.configuration?.quick_excitation === "auto" || state.quick_adaptation || state.report?.quick_adaptation);
    const adaptation = state.quick_adaptation || state.report?.quick_adaptation;
    const formalSamples = state.report ? state.report.samples : state.samples;
    const preparationOnly = automatic && adaptation && formalSamples === 0 &&
      !adaptation.selected_candidate && adaptation.collector?.stage !== "formal";
    const p = planning ? null : state.precision, phase = planning ? "idle" : state.phase;
    const stage = planning ? "" : state.stage, local = p?.phase || "";
    const keys = ["checking", "pilot", "warmup", "baseline", "static", "training", "validation", "stopping", "saving", "analysis"];
    const labels = ["启动检查", "自动选择激励", "预运行", "零偏测量", "静摩擦",
      fourier ? "傅里叶训练采集" : prepared ? "双向稳速采集" : quick ? "训练采集" : experiments[state.experiment],
      "独立验证", "停止确认", "保存记录", "模型计算"];
    let current = "checking";
    if (automatic && ["quick_pilot", "quick_pilot_return"].includes(stage)) current = "pilot";
    else if (prepared && (local.includes("warmup") || stage.includes("warmup"))) current = "warmup";
    else if (prepared && ["baseline", "await_enable"].includes(local)) current = "baseline";
    else if (local.startsWith("static") || local === "review_static" || state.experiment === "breakaway") current = "static";
    else if (phase === "running" || phase === "returning" || local === "done" || prepared && p && phase === "failed")
      current = p?.validation || stage === "compensated" || stage === "quick_validation" ? "validation" : "training";
    if (phase === "failed" && stage !== "stopping") {
      const failure = state.failure_stage || "";
      if (failure.includes("qualification") || failure === "fitting") current = "analysis";
      else if (["quick_pilot", "quick_pilot_return"].includes(failure)) current = "pilot";
      // Older ROS reports retain the broad running/returning failure phase.
      // The saved collector role and zero formal samples distinguish preparation.
      else if (preparationOnly && (["running", "returning", "quick_selection"].includes(failure) ||
        !failure && ["pilot", "pilot_return"].includes(adaptation.collector?.stage))) current = "pilot";
      else if (["checking", "arming"].includes(failure)) current = "checking";
      else if (failure === "recording_setup") current = "saving";
      else if (failure === "stopping") current = "stopping";
      else if (failure === "quick_validation") current = "validation";
      else if (!prepared && ["running", "returning"].includes(failure))
        current = state.experiment === "breakaway" ? "static" : "training";
    }
    const motionIndex = keys.indexOf(current);
    if (stage === "stopping") current = "stopping";
    else if (phase === "fitting") current = "analysis";
    if (completed.has(phase)) current = "analysis";
    const done = completed.has(phase), failed = ["failed", "incomplete"].includes(phase);
    const count = (value, total, unit) => `${finite(value) ? value : "—"}/${finite(total) ? total : "—"} ${unit}` +
      (finite(value) && total > 0 ? ` · ${Math.min(100, Math.floor(100 * (value || 0) / total))}%` : "");
    return keys.map((id, index) => {
      const selected = !selectable.includes(id) || !prepared || selections[`collect_${id}`] !== false;
      const visible = id === "pilot" ? Boolean(automatic) : prepared || !["warmup", "baseline"].includes(id) &&
        (id !== "static" || state.experiment === "breakaway") &&
        (id !== "training" || state.experiment !== "breakaway") &&
        (id !== "validation" || state.experiment === "validate" || quick);
      let status = "等待", kind = "", detail = "";
      const warmupOff = id === "warmup" && !state.precision_settings?.warmup_repetitions;
      if (!selected || warmupOff) { status = "跳过"; kind = "skipped"; detail = warmupOff && selected ? "节点预运行次数配置为 0" : "不执行，也不自动复用历史数据"; }
      else if (phase !== "idle") {
        if (done) { status = "完成"; kind = "done"; }
        else if (id === current) { status = failed ? "中断" : "执行中"; kind = failed ? "failed" : "current"; }
        else if (index < motionIndex || id === "checking" && phase !== "checking") { status = "完成"; kind = "done"; }
        else if (failed) status = "未执行";
        if (stage === "stopping" && index === motionIndex && id !== "stopping") { status = "收尾中"; kind = "current"; }
        if (id === "stopping" && state.stop_confirmed && (done || failed || phase === "fitting")) { status = "已确认"; kind = "done"; }
        if (id === "saving") {
          const saved = state.report?.recording_complete || done;
          status = saved ? "已保存" : failed ? "检查报告" : state.run_id ? "持续记录" : "等待";
          kind = saved ? "done" : failed ? "warning" : state.run_id ? "current" : "";
          detail = "";
        }
        if (id === "analysis" && phase === "completed_unqualified") { status = "未达标"; kind = "warning"; }
        if (p && id === "warmup" && selected && !warmupOff) {
          const w = p.warmup;
          detail = `旋转 ${count(w?.completed_cells, w?.required_cells, "单元")}`;
          if (w?.sine) detail += `\n正弦 ${count(w.sine.completed_cycles, w.sine.required_cycles, "周期")}`;
          if (!terminal.has(phase) && current === id) detail += `\n${stages[stage] || stage} · ${number(p.target_speed, 3)} rad/s · ${number(p.sample_revolutions, 2)}/${p.required_revolutions} 圈`;
        }
        if (p && id === "baseline") {
          detail = current === id ? `${p.baseline_settling ? "失能稳定等待" : "统计零偏"} · ${number(p.phase_elapsed, 1)} s · ${p.baseline_samples ?? "—"} 样本` : "";
        }
        if (p && id === "static" && selected) {
          const completedTrials = done || motionIndex > keys.indexOf("static") ? p.static_trials : p.static_completed ?? Math.max(0, p.static_trial - 1);
          detail = finite(p.static_trials) ? count(completedTrials, p.static_trials, "次") : "";
          if (!terminal.has(phase) && current === id) detail += `\n${p.static_conditioning ? "预处理" : "测量"} · ${number(p.static_angle, 3)} rad · ${stages[stage] || stage}`;
          if (p.static_diagnostics?.qualified === false || state.report?.[state.experiment]?.static?.qualified === false) {
            status = "有质量提示"; kind = "warning";
          }
        }
        if (p && ["training", "validation"].includes(id) && selected) {
          const f = p.fourier?.settings || state.fourier_settings;
          const speedCount = state.precision_settings?.speeds?.length;
          const total = p[`${id}_required`] ?? (fourier && f
            ? id === "training" ? f.slow_training_cycles + f.training_cycles + 2 * f.settle_cycles
              : f.slow_validation_cycles + f.validation_cycles + 2 * f.validation_settle_cycles
            : speedCount ? id === "training" ? speedCount * 2 * state.precision_settings.repetitions
              : (speedCount - 1) * 2 : undefined);
          const finished = p[`${id}_completed`] ?? (done ? total : undefined);
          detail = count(finished, total, fourier ? "周期（含过渡）" : "速度单元");
          if (!terminal.has(phase) && current === id) detail += fourier
            ? `\n${{0:"训练过渡",1:"训练",2:"验证过渡",3:"验证"}[p.fourier?.role] || "定位 / 回零"} · 第 ${(p.fourier?.cycle ?? -1) + 1} 周期`
            : `\n${stages[stage] || stage} · ${number(p.target_speed, 3)} rad/s · 第 ${p.repetition} 次 · ${number(p.sample_revolutions, 2)}/${p.required_revolutions} 圈`;
        }
        if (id === "stopping" && stage === "stopping") detail = "等待停止请求之后的新鲜失能反馈；请求成功不代表已失能";
        if (id === "analysis" && phase === "completed_unqualified") detail = "质量提示见模型结果";
      }
      if (phase === "recorded" && selected && !warmupOff) {
        status = id === "analysis" ? "待计算" : id === "saving" ? "已保存"
          : id === "stopping" ? state.stop_confirmed ? "已确认" : "待确认" : "完成";
        kind = id === "analysis" ? "" : "done";
        detail = "";
      }
      if (quick && ["training", "validation"].includes(id)) {
        const measured = state.report?.quick?.[id === "training" ? "training" : "external_validation"];
        const timing = quickTiming(state.configuration);
        if (measured) detail = `${number(measured.duration, 1)} s · ${measured.raw_samples ?? "—"} 条样本`;
        else if (timing) detail = `${number(timing[id], 1)} s${id === "validation" ? " · 不参与拟合" : " · 模型选择仅使用本段"}`;
      }
      if (id === "pilot" && automatic) detail = planning
        ? "最多 2 × 10 s 预探测，每段后实测回位；不用于正式拟合或独立验证。"
        : quickAdaptation(state)?.text || "等待预探测记录。";
      if (failed && preparationOnly) {
        if (id === "pilot" && current === "pilot")
          status = adaptation.status === "rejected" ? "预探测未通过" : "预探测已中断";
        if (["training", "validation"].includes(id)) {
          status = "未执行"; kind = "";
          detail = "正式采集未开始；预探测样本不用于本阶段。";
        }
      } else if (failed && automatic && adaptation?.selected_candidate && formalSamples === 0 &&
        id === "training" && current === "training") {
        status = "未取得样本";
        detail = "已选定正式方案，但未保存正式采集样本。";
      }
      return {id, label: labels[index], selected, visible, status, kind, detail};
    });
  }
  function bounds(series) {
    const points = series.flatMap(item => item.points).filter(p => finite(p[0]) && finite(p[1]));
    if (!points.length) return null;
    let x0 = Infinity, x1 = -Infinity, y0 = Infinity, y1 = -Infinity;
    for (const [x, y] of points) { x0 = Math.min(x0, x); x1 = Math.max(x1, x); y0 = Math.min(y0, y); y1 = Math.max(y1, y); }
    const dy = Math.max(y1 - y0, Math.abs(y1) * 0.02, 0.002);
    return {x0, x1: Math.max(x1, x0 + 0.001), y0: y0 - dy * 0.12, y1: y1 + dy * 0.12};
  }
  function readValues(elements) {
    const result = {};
    for (const name of editable) {
      const element = elements[name];
      const value = bools.has(name) ? element.checked : strings.has(name) ? element.value : Number(element.value);
      if (!strings.has(name) && !bools.has(name) && (!element.value.trim() || !finite(value))) throw new Error(`${name} 需要有限数值`);
      result[name] = value;
    }
    if (!["fixed", "auto"].includes(result.quick_excitation)) throw new Error("quick_excitation 选项无效");
    return result;
  }
  function mergeRows(previous, rows, reset = false) {
    if (!reset && !rows.length) return previous;
    if (!reset && rows.length && (!previous.length || rows[0][0] > previous[previous.length - 1][0]) &&
        rows.every((row, index) => index === 0 || row[0] > rows[index - 1][0])) return previous.concat(rows);
    const merged = reset ? new Map() : new Map(previous.map(row => [row[0], row]));
    for (const row of rows) merged.set(row[0], row);
    return Array.from(merged.values()).sort((a, b) => a[0] - b[0]);
  }
  function limitRows(rows, maxPoints, valueIndices = [1]) {
    if (!Number.isInteger(maxPoints) || maxPoints < 2) throw new Error("绘图点数预算无效");
    if (rows.length <= maxPoints) return rows;
    const columns = valueIndices.length ? valueIndices : [1];
    const buckets = Math.floor((maxPoints - 2) / (2 * columns.length));
    if (buckets < 1) return [rows[0], rows[rows.length - 1]];
    const selected = new Set([0, rows.length - 1]);
    for (let bucket = 0; bucket < buckets; bucket++) {
      const start = 1 + Math.floor(bucket * (rows.length - 2) / buckets);
      const end = 1 + Math.floor((bucket + 1) * (rows.length - 2) / buckets);
      for (const column of columns) {
        let low = -1, high = -1;
        for (let index = start; index < end; index++) {
          const value = rows[index][column];
          if (!finite(value)) continue;
          if (low < 0 || value < rows[low][column]) low = index;
          if (high < 0 || value > rows[high][column]) high = index;
        }
        if (low >= 0) selected.add(low);
        if (high >= 0) selected.add(high);
      }
    }
    return Array.from(selected).sort((a, b) => a - b).map(index => rows[index]);
  }
  function windowRows(rows, {seconds = 60, end = null, maxPoints = 400, timeIndex = 1, valueIndices = [2, 3, 4]} = {}) {
    if (!finite(seconds) || seconds < 0) throw new Error("显示时间窗口无效");
    if (!rows.length) return {rows: [], total: 0, inWindow: 0, start: null, end: null, sampled: false};
    const latest = rows[rows.length - 1][timeIndex];
    const finish = end === null ? latest : Math.min(latest, Math.max(rows[0][timeIndex], end));
    if (!finite(finish)) throw new Error("回放位置无效");
    const start = seconds === 0 ? rows[0][timeIndex] : Math.max(rows[0][timeIndex], finish - seconds);
    function lowerBound(value, inclusive) {
      let low = 0, high = rows.length;
      while (low < high) {
        const middle = Math.floor((low + high) / 2);
        if (rows[middle][timeIndex] < value || (inclusive && rows[middle][timeIndex] === value)) low = middle + 1;
        else high = middle;
      }
      return low;
    }
    const visible = rows.slice(lowerBound(start, false), lowerBound(finish, true));
    const selected = limitRows(visible, maxPoints, valueIndices);
    return {rows: selected, total: rows.length, inWindow: visible.length, start, end: finish,
      sampled: selected.length < visible.length};
  }
  function reason(message) {
    if (!message) return "";
    const translations = [
      ["Speed limit reached", "速度保护触发：实测速度达到本次拖动 / 实验的停止门限。"],
      ["Travel limit reached", "行程保护触发：相对起点的位移达到本次行程门限。"],
      ["Measured torque limit reached", "实测力矩保护触发；不能把它当作人手外力测量。"],
      ["Temperature limit reached", "温度保护触发。"],
      ["Control scheduler deadline missed", "控制回调超时，已中止输出；请检查系统负载，而不是放宽运动边界。"],
      ["Drag max_speed", "拖动停止速度超过节点或驱动允许范围。"],
      ["Drag max_travel", "拖动行程超过当前位置、节点或驱动允许范围。"],
      ["Manual drag requires", "手动拖动需要有效的驱动速度和绝对位置保护，请检查驱动配置。"],
      ["Stribeck roll-off not covered", "Stribeck 低速衰减区或高速平台覆盖不足。可调整激励与阻尼，或选择更简单的模型。"],
      ["Parameters are not identifiable", "参数不可辨识：请检查独立激励和模型项之间的相关性。"],
      ["Hardware motion is locked", "真机运动门禁尚未在配置中解锁。"],
      ["Start requires a stationary shaft", "启动时轴必须静止；请等待实际使能确认后再手动拖动。"],
      ["Stale feedback", "反馈已陈旧，不能继续运动；请检查驱动及通信。"],
      ["Experiment interrupted by operator stop", "实验已由操作员停止，本次不发布标定结果。"],
      ["Precision scanning requires explicit", "精密全速扫描必须显式允许空载多圈旋转。不能沿用旧的小行程安全确认。"],
      ["Precision needs velocity capacity", "当前驱动的速度或行程范围不足，请先使用专门的精密标定保护配置。"],
      ["Driver high-speed", "驱动的高速保护配置不满足精密扫描要求，未发送新的运动请求。"],
    ];
    const match = translations.find(item => message.startsWith(item[0]));
    return match ? `${match[1]} 原始诊断：${message}` : message;
  }
  return {stages, experiments, models, editable, selectable, stageRows, parseStageValue, stageEstimate, readDragValues, experimentState, latestExperiment, quickProfile, quickTiming, quickAdaptation, quickProtocol, active, completed, terminal, maxChartPoints,
    bools, colors, finite, number, sharedRanges, sharedRangeText, speedInSharedRanges, sharedModelCurves, speedSweepDiagnostics,
    bounds, readValues, mergeRows, limitRows, windowRows, reason};
})();

if (typeof module !== "undefined" && module.exports) module.exports = CalibrationView;
