"""Traceable acquisition routing, distinct from final model-window selection."""

import math

from .data import CalibrationError


USAGE_DESCRIPTIONS = {
    "warmup": "预运行：温度与运动覆盖诊断，不进入正式摩擦拟合",
    "baseline": "失能零偏候选：排除稳定等待段后估计零偏，不直接拟合摩擦",
    "static_conditioning": "静摩擦预处理：保留诊断，不计入正式起动阈值统计",
    "static_measurement": "静摩擦测量候选：使用起动前有效静止窗口，独立于动态拟合",
    "fourier_settling": "傅里叶过渡周期：保存但不进入动态拟合或独立验证",
    "training": "动态训练候选：还需完整周期/整圈、滑动和积分窗口筛选",
    "validation": "独立验证候选：不用于动态模型参数优化",
    "transition": "回位、升降速及其他过程：保存，不直接进入正式拟合",
    "invalid": "无效分析样本：原始记录保留，不作为有效辨识数据",
    "unclassified": "旧格式或非标定记录：缺少阶段标签，不推测用途",
}


class AcquisitionUsage:
    def __init__(self, metadata: dict):
        self.metadata = metadata
        self.rows = 0
        self.counts = dict.fromkeys(USAGE_DESCRIPTIONS, 0)
        self.segments: list[dict] = []

    def classify(self, row: dict) -> tuple[str, int]:
        def integer(name, default=-1):
            value = float(row.get(name, default))
            if not math.isfinite(value) or value != int(value):
                raise CalibrationError(f"Invalid acquisition usage tag: {name}")
            return int(value)

        if "analysis_valid" not in row:
            return "unclassified", -1
        if integer("analysis_valid") != 1:
            return "invalid", -1
        if integer("analysis_warmup", 0):
            return "warmup", -1
        if integer("analysis_baseline", 0):
            return "baseline", -1
        trial = integer("analysis_static_trial")
        if trial >= 0:
            settings = self.metadata.get("precision_settings") or {}
            conditioning = settings.get("static_conditioning_repetitions", 0)
            repetitions = settings.get("static_repetitions")
            if repetitions is None:
                return "unclassified", trial
            within = trial % (2 * (conditioning + repetitions))
            return ("static_conditioning" if within < 2 * conditioning else "static_measurement"), trial
        cycle, role = integer("analysis_fourier_cycle"), integer("analysis_fourier_role")
        if cycle >= 0:
            if role not in (0, 1, 2, 3):
                raise CalibrationError("Invalid Fourier acquisition usage role")
            return ("fourier_settling" if role in (0, 2) else "training" if role == 1 else "validation"), cycle
        cell = integer("analysis_cell")
        if cell >= 0:
            cells = self.metadata.get("cells") or []
            if cell >= len(cells):
                raise CalibrationError("Acquisition usage cell is outside the recorded plan")
            return ("validation" if cells[cell]["validation"] else "training"), cell
        return "transition", -1

    def append(self, row: dict) -> None:
        category, index = self.classify(row)
        self.rows += 1
        self.counts[category] += 1
        if self.segments and (self.segments[-1]["category"], self.segments[-1]["index"]) == (category, index):
            self.segments[-1]["last_data_row"] = self.rows
        else:
            self.segments.append({"category": category, "index": index,
                                  "first_data_row": self.rows, "last_data_row": self.rows})

    def summary(self, *, include_segments: bool = False) -> dict:
        result = {"schema": 1, "rows": self.rows, "counts": dict(self.counts),
                  "descriptions": USAGE_DESCRIPTIONS,
                  "semantics": "Acquisition routing only; candidate rows are not all fitted samples. Data rows are 1-based, excluding the CSV header."}
        if include_segments:
            result["segments"] = [dict(segment) for segment in self.segments]
        return result
