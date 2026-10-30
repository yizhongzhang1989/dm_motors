"""Editable acquisition parameters, separate from fixed protection and qualification limits."""

import math
from dataclasses import asdict

from .data import CalibrationError
from .models import MODELS
from .precision import PrecisionSettings
from .fourier_trajectory import FourierSettings


FIELDS = {
    "precision_time_budget": ("总实验时间预算 / s", float, 60, 14400),
    "precision_zero_position": ("控制零位 / rad（不写硬件零位）", float, -12.5, 12.5),
    "precision_warmup_repetitions": ("预运行轮数", int, 0, 4),
    "precision_warmup_turns_per_direction": ("每方向旋转总圈数（三档合计）", int, 3, 300),
    "precision_warmup_speeds": ("三档旋转速度 / rad/s", list, 0.05, 10),
    "precision_warmup_sine_cycles": ("正弦往复周期数", int, 0, 1000),
    "precision_warmup_sine_amplitude": ("正弦中心到峰值 / rad", float, 0.05, math.pi),
    "precision_warmup_sine_start_frequency": ("正弦起始频率 / Hz", float, 0.02, 1),
    "precision_warmup_sine_end_frequency": ("正弦结束频率 / Hz", float, 0.02, 1),
    "precision_baseline_settle_time": ("失能稳定等待 / s", float, 0, 60),
    "precision_baseline_time": ("零偏统计窗口 / s", float, 0.1, 300),
    "precision_static_angles": ("静摩擦角度 / rad（等间距覆盖一圈）", list, -math.tau, math.tau),
    "precision_static_repetitions": ("每角度、每方向测量次数", int, 3, 8),
    "precision_static_conditioning_repetitions": ("每角度、每方向预处理次数", int, 0, 8),
    "precision_static_ramp_rate": ("静摩擦增矩速率 / Nm/s", float, 0.005, 0.2),
    "precision_static_average_time": ("起动前力矩平均窗口 / s", float, 0.05, 3),
    "precision_neutral_time": ("撤预载静置时间 / s", float, 0.5, 5),
    "precision_approach": ("同向接近距离 / rad", float, 0.01, 0.3),
    "precision_speeds": ("主扫描速度 / rad/s（末档须为 2π）", list, 0.01, math.tau),
    "precision_repetitions": ("每速度、每方向重复次数", int, 3, 8),
    "precision_revolutions": ("每单元最少整圈数", int, 1, 5),
    "precision_sample_time": ("每单元最少采样时间 / s", float, 0.2, 10),
    "precision_settle_time": ("稳速确认时间 / s", float, 0.4, 5),
    "fourier_frequency": ("训练基频 / Hz", float, 0.02, 0.5),
    "fourier_harmonics": ("傅里叶谐波数", int, 3, 12),
    "fourier_amplitude": ("轨迹位置半幅上限 / rad", float, 0.1, math.pi),
    "fourier_settle_cycles": ("每种训练幅值的过渡周期", int, 1, 8),
    "fourier_training_cycles": ("常速训练周期数", int, 3, 24),
    "fourier_slow_training_cycles": ("低速训练周期数", int, 3, 16),
    "fourier_slow_scale": ("低速轨迹幅值比例", float, 0.02, 0.2),
    "fourier_validation_settle_cycles": ("每种验证幅值的过渡周期", int, 1, 8),
    "fourier_validation_cycles": ("常速验证周期数", int, 3, 12),
    "fourier_slow_validation_cycles": ("低速验证周期数", int, 3, 12),
    "fourier_validation_frequency_ratio": ("验证 / 训练频率比", float, 1.01, 2),
    "fourier_validation_amplitude_ratio": ("验证 / 训练幅值比", float, 0.5, 1),
    "fourier_feedback_rate": ("傅里叶原始反馈频率 / Hz", int, 500, 1000),
    "fourier_phase_samples": ("训练周期相位分箱数", int, 256, 4000),
    "fourier_derivative_harmonics": ("实测位置重建谐波数", int, 5, 150),
    "fourier_integration_window": ("动力学积分窗口 / s", float, 0.005, 0.2),
}
EDITABLE_STAGE_TYPES = {name: field[1] for name, field in FIELDS.items()}
CONSTRAINT_DEFAULTS = {
    "max_torque": 1.0, "max_speed": 3.0, "max_travel": 2.0, "max_temperature": 50.0,
    "feedback_timeout": 0.08, "target_validity": 0.1,
    "compensation_deadband": 0.02, "compensation_smoothing": 0.03,
    "compensation_filter": 0.01, "compensation_slew": 5.0,
    "integration_window": 0.04, "min_speed": 0.015, "max_gap": 0.025, "max_rmse": 0.02,
    "minimum_improvement": 0.15,
    "breakaway_ramp_rate": 0.05, "breakaway_speed": 0.04, "breakaway_position": 0.005,
    "breakaway_samples": 3, "breakaway_approach": 0.03, "breakaway_return_tolerance": 0.003,
    "breakaway_settle_time": 0.5,
    **{prefix + name: value for prefix, settings in
       (("precision_", PrecisionSettings()), ("fourier_", FourierSettings()))
       for name, value in asdict(settings).items()
       if type(value) in (float, int) and prefix + name not in FIELDS},
}
EDITABLE_CONSTRAINT_TYPES = {name: type(value) for name, value in CONSTRAINT_DEFAULTS.items()}
CONSTRAINT_LABELS = {
    "max_torque": "输出力矩预算 / Nm", "max_speed": "速度停止门限 / rad/s",
    "max_travel": "相对行程预算 / rad", "max_temperature": "温度停止门限 / °C",
    "feedback_timeout": "反馈新鲜度上限 / s", "target_validity": "目标有效期 / s",
    "compensation_deadband": "补偿速度死区 / rad/s", "compensation_smoothing": "补偿平滑速度 / rad/s",
    "compensation_filter": "补偿速度滤波时间 / s", "compensation_slew": "补偿力矩变化率 / Nm/s",
    "acceleration": "加速度预算 / rad/s²", "jerk": "加加速度预算 / rad/s³",
    "fourier_reference_speed": "高速参考速度上限 / rad/s",
    "fourier_minimum_peak_speed": "参考轨迹双向最低峰值 / rad/s（不是实测值）",
}


def constraint_label(name: str) -> str:
    bare = name.removeprefix("precision_").removeprefix("fourier_").removeprefix("warmup_")
    scope = ("预运行 · " if name.startswith("precision_warmup_") else
             "精密 / 傅里叶基础 · " if name.startswith("precision_") else
             "傅里叶 · " if name.startswith("fourier_") else
             "快速标定（不用于傅里叶）· " if name in ("max_speed", "max_travel") else "")
    return scope + CONSTRAINT_LABELS.get(name, CONSTRAINT_LABELS.get(bare, name))


def normalize_constraint(name: str, value):
    kind = EDITABLE_CONSTRAINT_TYPES[name]
    if (type(value) not in (int, float) or not math.isfinite(value) or value < 0 or
            kind is int and type(value) is not int):
        raise CalibrationError(f"{name} must be a finite nonnegative {kind.__name__}")
    return kind(value)

TEST_PROFILE_NOTE = "Workflow test profile: diagnostic acquisition only; not a qualified full calibration"
MODEL_FIELDS = ["model", "fit_bias", "known_bias", "fit_gravity", "fit_stiffness"]
LEGACY_FIELDS = {
    "model": {"label": "摩擦模型", "type": "choice", "choices": ["auto", *MODELS]},
    "fit_bias": {"label": "拟合恒定负载 / 力矩偏置", "type": "boolean"},
    "known_bias": {"label": "独立测得的偏置 / Nm", "type": "number", "min": -1000, "max": 1000},
    "fit_gravity": {"label": "同时辨识重力幅值与相位", "type": "boolean"},
    "fit_stiffness": {"label": "同时辨识关节刚度", "type": "boolean"},
    "duration": {"label": "任务 / 每段时长 / s", "type": "number", "min": 8, "max": 600},
    "amplitude": {"label": "位置幅值 / rad", "type": "number", "min": 0.001, "max": 1000},
    "frequency": {"label": "基频 / Hz", "type": "number", "min": 0.001, "max": 1000},
    "kp": {"label": "位置增益 Kp", "type": "number", "min": 0.001, "max": 500},
    "kd": {"label": "速度增益 Kd", "type": "number", "min": 0.001, "max": 100},
    "compensation_fraction": {"label": "A/B 补偿比例", "type": "number", "min": 0, "max": 1},
}

GROUPS = {
    "checking": {
        "title": "就绪与安全检查", "fields": ["precision_time_budget"],
        "note": "总预算不等于阶段目标时长。反馈新鲜度、真机门禁和驱动保护不可在网页放宽。"},
    "warmup": {
        "title": "预运行",
        "fields": ["precision_zero_position", "precision_warmup_repetitions", "precision_warmup_turns_per_direction",
                   "precision_warmup_speeds", "precision_warmup_sine_cycles", "precision_warmup_sine_amplitude",
                   "precision_warmup_sine_start_frequency", "precision_warmup_sine_end_frequency"],
        "note": "正弦峰峰范围为半幅的两倍。圈数和周期均由实测运动确认；参数必须满足原有行程和速度预算。"},
    "baseline": {
        "title": "失能零偏测量", "fields": ["precision_baseline_settle_time", "precision_baseline_time"],
        "note": "全程失能；稳定等待样本不进入零偏统计。统计窗口至少保留完整有效样本。"},
    "static": {
        "title": "独立静摩擦",
        "fields": ["precision_static_angles", "precision_static_repetitions", "precision_static_conditioning_repetitions",
                   "precision_static_ramp_rate", "precision_static_average_time", "precision_neutral_time", "precision_approach"],
        "breakaway": ["duration"],
        "note": "测量次数与预处理次数独立。保留双向、多角度、至少三次独立重复，不提高力矩上限。"},
    "training": {
        "title": "训练数据采集",
        "precision": ["precision_speeds", "precision_repetitions", "precision_revolutions", "precision_sample_time", "precision_settle_time"],
        "fourier": ["fourier_frequency", "fourier_harmonics", "fourier_amplitude", "fourier_settle_cycles",
                    "fourier_training_cycles", "fourier_slow_training_cycles", "fourier_slow_scale"],
        "calibrate": ["duration", "amplitude", "frequency", "kp", "kd"],
        "validate": ["duration", "amplitude", "frequency", "kp", "kd", "compensation_fraction"],
        "note": "稳速末档须保留 2π；傅里叶轨迹必须通过连续运动边界和可辨识性检查。修改基频需同步检查相位分箱。"},
    "validation": {
        "title": "独立验证",
        "precision": ["precision_revolutions", "precision_sample_time", "precision_settle_time"],
        "fourier": ["fourier_validation_settle_cycles", "fourier_validation_cycles", "fourier_slow_validation_cycles",
                    "fourier_validation_frequency_ratio", "fourier_validation_amplitude_ratio"],
        "validate": ["duration", "amplitude", "frequency", "kp", "kd", "compensation_fraction"],
        "note": "稳速验证使用主档之间的独立中间速度；这里的整圈与采样参数也用于训练。傅里叶验证轨迹保持独立。"},
    "stopping": {
        "title": "停止与失能确认", "fields": [],
        "note": "安全步骤，无人为等待时长。持续请求失能并检查新鲜设备反馈；3 秒未确认即明确报错，不能跳过或放宽。"},
    "saving": {
        "title": "原始数据保存", "fourier": ["fourier_feedback_rate"],
        "note": "完整原始数据必须保存，网页显示抽样不改变采集频率。保存目录固定为节点配置，完整清单先于最终计算提交。"},
    "analysis": {
        "title": "计算与质量验收",
        "fields": ["precision_static_average_time"],
        "fourier": [*MODEL_FIELDS, "fourier_phase_samples", "fourier_derivative_harmonics", "fourier_integration_window"],
        "calibrate": MODEL_FIELDS,
        "note": "摩擦模型、偏置、重力和刚度在此统一配置。非对称摩擦必须关闭偏置拟合并提供独立偏置；不放宽验收门限。"},
}


def validate_stage_parameter(name: str, value):
    _label, kind, low, high = FIELDS[name]
    if kind is list:
        if not isinstance(value, list) or not 3 <= len(value) <= 40:
            raise CalibrationError(f"{name} must be a numeric array with 3..40 entries")
        values = value
    else:
        values = [value]
    if any(isinstance(item, bool) or not isinstance(item, (int, float)) or
           not math.isfinite(item) or not low <= item <= high for item in values):
        raise CalibrationError(f"{name} must be finite and within [{low:g}, {high:g}]")
    if kind is int and type(value) is not int:
        raise CalibrationError(f"{name} must be an integer")
    return [float(item) for item in values] if kind is list else int(value) if kind is int else float(value)


def parameter_schema(constraint_bounds: dict | None = None) -> dict:
    constraints = {
        name: {"label": constraint_label(name), "type": "integer" if EDITABLE_CONSTRAINT_TYPES[name] is int else "number",
               "min": bounds["min"], "max": bounds["max"]}
        for name, bounds in (constraint_bounds or {}).items() if name in EDITABLE_CONSTRAINT_TYPES}
    groups = {**GROUPS}
    if constraints:
        groups["constraints"] = {
            "title": "应用层约束", "fields": list(constraints),
            "note": "只在确认失能后应用。非零/关联条件由后端再次校验；力矩与速度受驱动能力约束，其他项不超过节点启动预算。"
                    "实测力矩停止门限仍为输出预算的 1.25 倍。修改此处不写驱动配置或设备寄存器。"}
    return {"groups": groups, "fields": {**{
        name: {"label": label, "type": "array" if kind is list else "integer" if kind is int else "number",
               "min": low, "max": high}
        for name, (label, kind, low, high) in FIELDS.items()}, **LEGACY_FIELDS, **constraints}}


def workflow_test_profile(experiment: str) -> dict:
    profile = {
        "protocol_mode": "workflow_test",
        "collect_warmup": True, "collect_static": True, "collect_training": True, "collect_validation": True,
        "precision_warmup_repetitions": 1, "precision_warmup_turns_per_direction": 3,
        "precision_warmup_sine_cycles": 20,
        "precision_baseline_settle_time": 5.0, "precision_baseline_time": 115.0,
        "precision_static_repetitions": 3, "precision_static_conditioning_repetitions": 0,
        "precision_static_ramp_rate": 0.12, "precision_static_average_time": 0.4,
        "precision_neutral_time": 0.5, "precision_approach": 0.03,
    }
    if experiment == "fourier":
        profile.update(fourier_frequency=0.1, fourier_settle_cycles=1, fourier_slow_training_cycles=3,
                       fourier_training_cycles=7, fourier_validation_settle_cycles=1,
                       fourier_slow_validation_cycles=6, fourier_validation_cycles=7,
                       fourier_validation_frequency_ratio=1.25, fourier_phase_samples=2000)
    elif experiment == "precision":
        profile.update(precision_speeds=[1.5, 3.0, math.tau], precision_repetitions=3,
                       precision_revolutions=1, precision_sample_time=1.0)
    else:
        raise CalibrationError("The workflow test profile is for precision or Fourier acquisition")
    return profile
