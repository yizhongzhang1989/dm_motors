"""Complete read-only calibration analysis as one portable HTML file."""

from datetime import datetime, timezone
from html import escape
import json
import math
import os
from pathlib import Path
import tempfile
from typing import Iterable, Sequence

import numpy as np

from .data import CalibrationError, atomic_json, read_json
from .dashboard_data import model_curves
from .recording import environment_versions
from .reidentify import file_sha256
from .report_data import ReportData, TERMINAL_STATES, input_path, load_report_data
from .report_plot import COLORS, MAX_PLOT_POINTS, Series, plot


REPORT_NAME = "analysis.html"
REPORT_META = "analysis.html.meta.json"
REPORT_VERSION = 3
REPORT_CSP = ("default-src 'none'; script-src 'none'; style-src 'unsafe-inline'; img-src data:; "
              "base-uri 'none'; form-action 'none'; object-src 'none'")
STATUS_NAMES = {"completed": "采集完成 · 达到记录中的资格标准", "completed_unqualified": "采集完成 · 精度目标未全部通过",
                "qualified": "离线辨识完成 · 达到记录中的拟合标准", "failed": "运行失败 / 已中断",
                "incomplete": "数据或采集计划不完整"}
CSS = """
:root{color-scheme:light;font-family:system-ui,-apple-system,"Segoe UI","Noto Sans CJK SC",sans-serif;color:#203247;background:#f2f5f8}
*{box-sizing:border-box}body{margin:0}main{max-width:1240px;margin:0 auto;padding:26px}
header,section{background:white;border:1px solid #dce4ed;border-radius:12px;padding:24px;margin-bottom:20px}
h1{font-size:29px;margin:0 0 12px}h2{font-size:22px;margin:0 0 18px}h3{font-size:17px;margin:24px 0 10px}
p,li{line-height:1.65}a{color:#1768ac}nav{display:flex;gap:12px;flex-wrap:wrap;margin-top:20px}
.badge{display:inline-block;border-radius:5px;background:#e3f4eb;color:#175c3b;padding:6px 12px;font-weight:600}
.warning{background:#fff4d7;border-left:4px solid #b77900;padding:12px 16px;color:#65470a}
.failure{background:#fbe6e5;border-left:4px solid #b93232;padding:12px 16px;color:#842626}
.muted,.unavailable{color:#607388}.cards{display:grid;grid-template-columns:repeat(auto-fit,minmax(175px,1fr));gap:12px;margin-top:18px}
.card{background:#f4f7fb;padding:16px;border-radius:8px}.card b{display:block;font-size:23px;margin-bottom:5px}
.table-wrap{overflow-x:auto}table{border-collapse:collapse;width:100%;font-size:13px;margin:10px 0}
th,td{border-bottom:1px solid #e2e8f0;padding:9px 10px;text-align:left;vertical-align:top}th{background:#edf2f7;white-space:nowrap}
td{font-variant-numeric:tabular-nums}tbody tr:nth-child(even){background:#f8fafc}
code,pre{font-family:ui-monospace,SFMono-Regular,monospace;font-size:12px}code{overflow-wrap:anywhere}
pre{white-space:pre-wrap;overflow-wrap:anywhere;background:#f3f6fa;padding:14px;border-radius:7px;max-height:560px;overflow:auto}
details{border:1px solid #dce4ed;border-radius:7px;padding:12px;margin:12px 0}summary{cursor:pointer;font-weight:600}
.chart{border:1px solid #e4eaf1;border-radius:8px;margin:18px 0;overflow:hidden}.chart svg{display:block;width:100%;height:auto}
.small{font-size:13px}footer{padding:10px 0 30px;color:#607388}
@media(max-width:600px){main{padding:10px}header,section{padding:15px}h1{font-size:23px}.chart{overflow-x:auto}.chart svg{min-width:720px}}
@media print{body{background:white}main{max-width:none;padding:0}header,section{border:0;padding:10px 0}nav{display:none}
 .chart{break-inside:avoid}pre{max-height:none}details{break-inside:avoid}h2{break-after:avoid}}
"""


def value(item) -> str:
    if item is None:
        return "未记录 / 未测得"
    if isinstance(item, bool):
        return "是" if item else "否"
    if isinstance(item, float):
        return f"{item:.9g}" if math.isfinite(item) else "非有限值"
    if isinstance(item, (dict, list, tuple)):
        return json.dumps(redact(item), ensure_ascii=False, allow_nan=False)
    return str(item)


def redact(item):
    if isinstance(item, dict):
        return {str(key): ("[已省略]" if str(key).lower() in
                          {"token", "password", "passwd", "secret", "api_key", "access_token"} else redact(entry))
                for key, entry in item.items()}
    if isinstance(item, (list, tuple)):
        return [redact(entry) for entry in item]
    return item


def table(headers: Sequence[str], rows: Iterable[Sequence]) -> str:
    heading = "".join(f"<th>{escape(header)}</th>" for header in headers)
    body = "".join("<tr>" + "".join(f"<td>{escape(value(cell))}</td>" for cell in row) + "</tr>" for row in rows)
    return f'<div class="table-wrap"><table><thead><tr>{heading}</tr></thead><tbody>{body}</tbody></table></div>'


def mapping_table(mapping: dict) -> str:
    cleaned = redact(mapping)
    if not isinstance(cleaned, dict):
        raise CalibrationError("Report field table requires an object")
    return table(("字段", "记录值"), cleaned.items())


def section(identifier: str, title: str, content: str) -> str:
    return f'<section id="{identifier}"><h2>{escape(title)}</h2>{content}</section>'


def details(title: str, content: str) -> str:
    return f"<details><summary>{escape(title)}</summary>{content}</details>"


def chart(*args, **kwargs) -> str:
    return '<div class="chart">' + plot(*args, **kwargs) + "</div>"


def finite_rows(rows: list[dict], *fields: str) -> list[dict]:
    result = []
    for row in rows:
        available = True
        for name in fields:
            entry = row.get(name)
            if entry is None:
                available = False
            elif isinstance(entry, bool) or not isinstance(entry, (int, float)) or not math.isfinite(entry):
                raise CalibrationError(f"Invalid numeric report field: {name}")
        if available:
            result.append(row)
    return result


def candidate_series(precision: dict, *, signed: bool) -> list[Series]:
    candidate = precision.get("candidate_model")
    if not isinstance(candidate, dict) or not candidate.get("friction_table"):
        return []
    matrix = np.asarray(candidate["friction_table"], dtype=float)
    if matrix.ndim != 2 or matrix.shape[1] != 4:
        raise CalibrationError("Candidate friction table must have four directional columns")
    negative_sign = -1 if signed else 1
    return [
        Series("正向候选模型", sorted((float(r[0]), float(r[1])) for r in matrix), COLORS[0], True),
        Series("反向候选模型", sorted((negative_sign * float(r[2]), negative_sign * float(r[3])) for r in matrix),
               COLORS[1], True),
    ]


def primary_friction_curve(data: ReportData) -> str:
    if data.fourier is not None:
        fourier = data.fourier
        training, validation = fourier.get("training_friction"), fourier.get("validation_friction")
        if training is None or validation is None:
            return section("friction-curve", "傅里叶摩擦曲线", '<p class="unavailable">本数据尚未支持可辨识的摩擦候选模型；不把原始总力矩伪装成摩擦。</p>')
        series = [
            Series("训练实测积分窗口", [(float(x[0]), float(x[1])) for x in training["points"]], COLORS[0], connect=False),
            Series("独立验证窗口", [(float(x[0]), float(x[1])) for x in validation["points"]], COLORS[1], connect=False),
            Series("负向候选模型", [(float(x), float(y)) for x, y in validation["curves"][0]], COLORS[2], True),
            Series("正向候选模型", [(float(x), float(y)) for x, y in validation["curves"][1]], COLORS[2], True),
        ]
        content = ('<p>点为实测积分窗口，在同一模型下扣除惯性、偏置及已辨识的重力/弹性项后得到的摩擦观测。'
                   '训练与独立验证分开显示；它们不是指令力矩，也不是未经动力学校正的速度—总力矩散点。</p>')
        content += chart("傅里叶运动辨识的摩擦曲线", "实测窗口均速 / rad/s", "摩擦补偿力矩 / Nm", series,
                         symmetric_x=True, references=((0.0, "零力矩"),))
        return section("friction-curve", "摩擦曲线（傅里叶运动）", content)
    precision = data.precision or {}
    levels = precision.get("levels", [])
    series = []
    if levels:
        for direction, label, color in ((1, "正转实测", COLORS[0]), (-1, "反转实测", COLORS[1])):
            rows = sorted((row for row in finite_rows(levels, "target_speed", "actual_speed", "mean_torque")
                           if row["target_speed"] * direction > 0), key=lambda row: row["actual_speed"])
            errors = [row["ci95"] for row in rows] if all(row.get("ci95") is not None for row in rows) else None
            series.append(Series(label, [(row["actual_speed"], row["mean_torque"]) for row in rows], color, errors=errors))
    elif precision.get("observations"):
        observations = finite_rows(precision["observations"], "direction", "measured_speed", "torque")
        for direction, label, color in ((1, "正转已测单元", COLORS[0]), (-1, "反转已测单元", COLORS[1])):
            points = sorted((row["measured_speed"], row["torque"]) for row in observations if row["direction"] == direction)
            series.append(Series(label, points, color))
    series.extend(candidate_series(precision, signed=True))
    if not any(item.points for item in series) and data.fit is not None and data.fit.kind == "mechanical":
        negative, positive = model_curves(data.fit)
        series = [
            Series("反向保存模型", [(float(x), float(y)) for x, y in negative], COLORS[1]),
            Series("正向保存模型", [(float(x), float(y)) for x, y in positive], COLORS[0]),
        ]
        caption = "这里展示保存的机械摩擦模型，不将未经惯性或负载校正的原始总力矩当作纯摩擦观测。"
    else:
        caption = ("横轴保留正反转的有符号转速，纵轴为扣除零偏后的整圈平均摩擦补偿力矩（维持转速所需的电机力矩方向）。"
                   "实线为实测均值，虚线为候选模型；误差线沿用记录中的不确定度。"
                   "正反向分别绘制，零速未测区间不补零、不跨方向连线，也不将候选模型冒充合格结果。")
    if not any(item.points for item in series):
        return ""
    content = f"<p>{escape(caption)}</p>"
    content += chart("摩擦曲线：转速—摩擦补偿力矩", "转速 / rad/s（负值反转，正值正转）",
                     "摩擦补偿力矩 / Nm", series, symmetric_x=True, references=((0.0, "零力矩"),))
    if data.static is not None:
        positive = data.static.get("positive", {}).get("torque")
        negative = data.static.get("negative", {}).get("torque")
        content += (f'<p class="small muted">静摩擦起动幅值：正向 {escape(value(positive))} Nm，'
                    f'反向 {escape(value(negative))} Nm。它们属于起动阈值，不与零速滑动曲线连线。'
                    '低速放大、摩擦幅值、重复性及完整明细见后续章节。</p>')
    return section("friction-curve", "摩擦曲线（正反转）", content)


def precision_sections(data: ReportData) -> str:
    p = data.precision
    if p is None:
        return ""
    settings = p.get("settings") or (data.metadata or {}).get("precision_settings") or {}
    metrics = p.get("metrics") or {}
    issues = p.get("quality_issues") or (data.static or {}).get("quality_errors") or []
    content = '<p>采集是否完成、统计精度是否达标是两项独立结论。以下保留原始质量目标和所有测量单元，不删除异常窗口来提高分数。</p>'
    content += table(("指标", "结果", "记录目标", "是否通过"), [
        ("训练 RMSE / Nm", metrics.get("rmse"), settings.get("rmse_limit"),
         metrics["rmse"] <= settings["rmse_limit"] if metrics.get("rmse") is not None and "rmse_limit" in settings else None),
        ("独立验证 RMSE / Nm", metrics.get("validation_rmse"), settings.get("rmse_limit"),
         metrics["validation_rmse"] <= settings["rmse_limit"] if metrics.get("validation_rmse") is not None and "rmse_limit" in settings else None),
        ("最大独立验证误差 / Nm", metrics.get("max_bin_error"), settings.get("max_error"),
         metrics["max_bin_error"] <= settings["max_error"] if metrics.get("max_bin_error") is not None and "max_error" in settings else None),
        ("完成 / 测得 / 计划单元", [p.get("completed_cells"), p.get("measured_cells"), p.get("required_cells", p.get("cells"))], None, None),
        ("速度覆盖完整", p.get("coverage_complete"), True, p.get("coverage_complete")),
        ("基线位置稳定", p.get("baseline_stationary"), True, p.get("baseline_stationary")),
        ("失能零偏 / Nm", p.get("baseline_offset"), None, None),
        ("零偏报告界限 / Nm", p.get("baseline_ci95"), settings.get("ci95_limit"), None),
        ("正式标定温度范围 / °C", p.get("temperature_range"), settings.get("temperature_span"), None),
    ])
    if issues:
        content += f'<div class="warning"><b>{len(issues)} 项质量提示</b><ul>' + "".join(
            f"<li>{escape(str(issue))}</li>" for issue in issues) + "</ul></div>"
    else:
        content += '<p class="muted">本保存结果没有提供质量提示；这不替代独立的绝对力矩标尺校验。</p>'
    result = section("quality", "2. 精度与独立验证", content)
    levels = p.get("levels", [])
    observations = p.get("observations", [])
    if levels and settings:
        good = finite_rows(levels, "repeatability", "ci95")
        ripple = finite_rows(observations, "speed", "speed_std")
        if good and "repeatability_limit" in settings and "ci95_limit" in settings:
            content = table(("诊断", "记录结果"), [
                ("最大跨轮重复 SD / Nm", max(row["repeatability"] for row in good)),
                ("重复 SD 超目标档数", sum(row["repeatability"] > settings["repeatability_limit"] for row in good)),
                ("最大报告 CI95 界限 / Nm", max(row["ci95"] for row in good)),
                ("CI 界限超目标档数", sum(row["ci95"] > settings["ci95_limit"] for row in good)),
                ("未满足稳速判据即开始采集的单元", sum(row.get("settled_before_sampling") is False for row in observations)),
                ("发生过预算缩放的单元", sum(row.get("saturated_samples", 0) > 0 for row in observations)),
            ])
            if ripple and "speed_tolerance" in settings and "relative_speed_ripple" in settings:
                count = sum(row["speed_std"] > max(settings["speed_tolerance"],
                            abs(row["speed"]) * settings["relative_speed_ripple"]) for row in ripple)
                content += f"<p>速度波动超过记录目标：{count}/{len(ripple)} 个测量单元。</p>"
            content += ("<p>速度跟踪波动、完整圈平均阻力的跨轮重复性和独立预测误差不是同一指标。"
                        "预算缩放表示控制器为遵守力矩预算调整了增益，并不等于硬件过流。</p>")
            worst = sorted(good, key=lambda row: row["ci95"], reverse=True)[:8]
            content += table(("CI 最大的速度档", "实测速度", "均矩 / Nm", "重复 SD / Nm", "CI95 / Nm"),
                             ([row.get(name) for name in ("target_speed", "actual_speed", "mean_torque", "repeatability", "ci95")]
                              for row in worst))
            result += section("quality-diagnostics", "精度瓶颈定位", content)
    if levels:
        content = '<p>以下展示扣除各段失能零偏后的整圈角功平均阻力。负向使用阻力幅值，非将原始带符号数据改写。误差线为记录中的重复性/零偏合成界限。</p>'
        series = []
        for direction, label, color in ((1, "正向实测均值", COLORS[0]), (-1, "反向实测幅值", COLORS[1])):
            rows = sorted((r for r in finite_rows(levels, "target_speed", "actual_speed", "mean_torque")
                           if r["target_speed"] * direction > 0), key=lambda r: abs(r["actual_speed"]))
            errors = [r["ci95"] for r in rows] if all(r.get("ci95") is not None for r in rows) else None
            series.append(Series(label, [(abs(r["actual_speed"]), direction * r["mean_torque"]) for r in rows],
                                 color, errors=errors))
        series.extend(candidate_series(p, signed=False))
        content += chart("全速度摩擦阻力与候选模型", "|速度| / rad/s（对数坐标）", "阻力矩 / Nm", series, logarithmic=True)
        for field, title, limit in (("repeatability", "跨轮次重复标准差", settings.get("repeatability_limit")),
                                    ("ci95", "报告不确定度界限", settings.get("ci95_limit"))):
            series = []
            for direction, label, color in ((1, "正向", COLORS[0]), (-1, "反向", COLORS[1])):
                rows = sorted((r for r in finite_rows(levels, "target_speed", "actual_speed", field)
                               if r["target_speed"] * direction > 0), key=lambda r: abs(r["actual_speed"]))
                series.append(Series(label, [(abs(r["actual_speed"]), r[field] * 1000) for r in rows], color))
            content += chart(title, "|速度| / rad/s（对数坐标）", "mNm", series, logarithmic=True,
                             references=() if limit is None else ((limit * 1000, "记录目标"),))
        content += details(f"全部速度档汇总（{len(levels)} 项）", table(
            ("目标速度", "实测均速", "有符号均矩 / Nm", "重复 SD / Nm", "CI95 / Nm", "重复数", "样本数", "圈数", "留出误差 / Nm", "预算缩放样本"),
            ([r.get(k) for k in ("target_speed", "actual_speed", "mean_torque", "repeatability", "ci95", "repetitions",
                                "samples", "turns", "validation_error", "saturated_samples")] for r in levels)))
        result += section("friction", "3. 摩擦幅值与低速细节", content)
    if observations:
        result += section("cells", "4. 全部测量单元", details(f"测量明细（{len(observations)} 项）", table(
            ("名义速度", "方向", "重复索引", "中间验证档", "实测速度", "均矩 / Nm", "速度 SD", "温度 / °C", "时长 / s", "样本", "圈数", "稳速后采集", "预算缩放样本"),
            ([r.get(k) for k in ("speed", "direction", "repetition", "validation", "measured_speed", "torque",
                                "speed_std", "temperature", "duration", "samples", "turns", "settled_before_sampling",
                                "saturated_samples")] for r in observations))))
    validation = p.get("validation", [])
    if validation:
        series = []
        for direction, label, color in ((1, "正向留出数据", COLORS[0]), (-1, "反向留出数据", COLORS[1])):
            rows = sorted((r for r in finite_rows(validation, "measured_speed", "error") if r["direction"] == direction),
                          key=lambda r: abs(r["measured_speed"]))
            series.append(Series(label, [(abs(r["measured_speed"]), direction * r["error"] * 1000) for r in rows], color))
        limit = settings.get("max_error")
        content = '<p>主档前两轮建模，最后一轮与独立中间档用于验证。图中误差按阻力幅值统一方向，表中保留原始有符号误差。</p>'
        content += chart("独立验证：预测与实测的差值", "|速度| / rad/s（对数坐标）", "预测 − 实测阻力 / mNm", series,
                         logarithmic=True, references=() if limit is None else ((-limit * 1000, "下限"), (limit * 1000, "上限")))
        content += details(f"全部独立验证记录（{len(validation)} 项）", table(
            ("名义速度", "方向", "重复索引", "中间档", "实测速率", "实测均矩 / Nm", "预测 / Nm", "误差 / Nm"),
            ([r.get(k) for k in ("speed", "direction", "repetition", "validation", "measured_speed", "torque", "prediction", "error")] for r in validation)))
        result += section("validation", "5. 留出与中间速度验证", content)
    if observations and isinstance(settings.get("repetitions"), int):
        descriptions = []
        for direction in (1, -1):
            train = [row for row in finite_rows(observations, "measured_speed", "resistance", "speed", "repetition")
                     if row.get("direction") == direction and not row.get("validation") and
                     row["repetition"] < settings["repetitions"] - 1 and row["speed"] >= 1]
            blind = [row for row in finite_rows(validation, "measured_speed", "resistance", "speed")
                     if row.get("direction") == direction and row["speed"] >= 1]
            if len(train) < 4 or not blind:
                continue
            x = np.array([abs(row["measured_speed"]) for row in train])
            y = np.array([row["resistance"] for row in train])
            matrix = np.column_stack((np.ones(len(x)), x))
            coefficients, _, rank, _ = np.linalg.lstsq(matrix, y, rcond=None)
            if rank != 2:
                raise CalibrationError("Medium/high-speed report regression has no independent velocity excitation")
            errors = [coefficients[0] + coefficients[1] * abs(row["measured_speed"]) - row["resistance"] for row in blind]
            descriptions.append((direction, float(coefficients[0]), float(coefficients[1]),
                                 float(np.sqrt(np.mean((matrix @ coefficients - y) ** 2))),
                                 float(np.sqrt(np.mean(np.square(errors)))), len(train), len(blind)))
        if descriptions:
            result += section("descriptive-fit", "中高速趋势（描述性模型，非新增合格标定）",
                "<p>仅使用名义 |v|≥1 rad/s 的前两轮主档拟合 F≈Fc+B|v|，最后一轮及中间档独立验证。"
                "该近似不替代原候选表，不外推到零速，也不将截距当作已测静摩擦或独立黏性物理参数。</p>" +
                table(("方向", "截距 Fc / Nm", "斜率 B / Nm·s/rad", "训练 RMSE / Nm", "独立验证 RMSE / Nm", "训练单元", "验证单元"), descriptions))
        changes = []
        groups: dict[tuple, list] = {}
        for row in finite_rows(observations, "speed", "direction", "repetition", "resistance", "start_time"):
            if not row.get("validation"):
                groups.setdefault((row["speed"], row["direction"]), []).append(row)
        for (speed, direction), records in groups.items():
            ordered = sorted(records, key=lambda row: row["repetition"])
            if len(ordered) > 1:
                changes.append((speed, direction, ordered[0]["resistance"], ordered[-1]["resistance"],
                                ordered[-1]["resistance"] - ordered[0]["resistance"],
                                (ordered[-1]["start_time"] - ordered[0]["start_time"]) / 60))
        if changes:
            changes.sort(key=lambda row: abs(row[4]), reverse=True)
            result += section("repeat-drift", "跨轮变化与时间相关性",
                "<p>以下为相同名义速度/方向，最早与最后一次主档平均阻力的变化。它可能混合温漂、预处理历史、传感器零偏和负载变化，"
                "不能仅凭时间相关性判定唯一原因。</p>" +
                table(("速度", "方向", "首轮阻力 / Nm", "末轮阻力 / Nm", "变化 / Nm", "时间间隔 / min"), changes))
    failures = p.get("cell_failures", [])
    if failures:
        result += section("cell-failures", "未测完整的速度单元", mapping_table(
            {str(r.get("cell", index)): r for index, r in enumerate(failures)}))
    return result


def diagnostic_section(data: ReportData) -> str:
    content = ("<p>这里只统计本段实际收到/发布的数据；续采的父段记录仍单独保留。明确失能区间不计入活动目标更新间隔。"
               "主机发布/受理不等于电机硬件执行确认。</p>")
    d = data.diagnostics
    usage = d.get("data_usage")
    if usage:
        content += "<h3>原始样本的阶段归属</h3><p>这里统计候选归属，不把采集行数冒充实际拟合样本数。预运行、过渡、预处理仍保留在原始 CSV。</p>"
        content += table(("用途", "候选原始行数"), [
            (usage["descriptions"][name], count) for name, count in usage["counts"].items()])
    selection = (data.fourier or {}).get("data_selection")
    if selection:
        content += "<h3>真正进入动态模型的积分窗口</h3>"
        content += table(("数据集", "进入参数优化", "模型选择留出", "外部独立验证", "积分窗口总数"), [
            (name, selection[name]["fit_windows"], selection[name]["model_selection_windows"],
             selection[name]["external_validation_windows"], selection[name]["selected_windows"])
            for name in ("training", "validation")])
        content += ("<p>完整窗口索引见 fourier.json / report.json 的 data_selection：source_index、cycle、"
                    "first_phase_bin、last_phase_bin、use。索引从 0 开始，窗口两端相位箱均包含；"
                    "相位箱积分会使用相邻原始反馈点。源 CSV 阶段行范围见新版 recording.json 的 data_usage.segments。</p>"
                    "<p>训练集每第五个有效窗口用于模型选择，不是最终独立验证；最终验证仅使用 role=3 的周期。"
                    "静摩擦独立验收，不混入动态摩擦参数优化。</p>")
    content += table(("本段数据", "统计值"), [
        ("对齐样本行数", d.get("sample_rows")), ("原始反馈消息", d.get("feedback_rows")),
        ("控制状态快照", d.get("state_rows")), ("已发布目标", d.get("command_rows")),
        ("标记无效分析样本", d.get("invalid_analysis_rows")), ("非递增样本时间", d.get("nonincreasing_sample_times")),
        ("反馈计数缺口", d.get("feedback_counter_gaps")), ("反馈计数重复", d.get("feedback_counter_aliases")),
        ("反馈计数回退", d.get("feedback_counter_resets")), ("新增拒绝命令", d.get("rejected_commands_delta")),
        ("驱动故障 / 错误快照", d.get("fault_state_counts")), ("记录中最终状态", d.get("last_state")),
    ])
    rows = []
    for name, label in (("sample_interval_ms", "采集间隔"), ("active_command_interval_ms", "活动目标更新间隔")):
        values = d.get(name)
        if values is not None:
            rows.append((label, *[values[key] for key in ("count", "mean", "median", "p95", "p99", "max")]))
    if rows:
        content += table(("时间统计", "间隔数量", "平均 / ms", "中位 / ms", "P95 / ms", "P99 / ms", "最大 / ms"), rows)
    if d.get("temperatures"):
        content += table(("温度通道", "最低 / °C", "平均 / °C", "最高 / °C"),
                         ((name, values["min"], values["mean"], values["max"])
                          for name, values in d["temperatures"].items() if values is not None))
    content += (f'<p>驱动软调度抖动计数增量：{escape(value(d.get("deadline_misses_delta")))}；'
                f'其 1.5/rate 参考间隔：{escape(value(d.get("driver_soft_deadline_ms")))} ms。'
                "这类计数不自动等同于故障；状态快照采用 latest-state/depth 1，不能宣称保存了源端每次发布。"
                "反馈计数缺口也不应未经分析直接归因为 CAN 丢包。</p>")
    metadata = data.metadata or {}
    if metadata:
        identity = data.identity
        velocity_resolution = 2 * identity["vmax"] / 4095 if isinstance(identity.get("vmax"), (float, int)) else None
        content += table(("测量分辨率", "记录 / 协议值"), [
            ("位置 / rad", metadata.get("position_resolution")),
            ("力矩 / Nm", metadata.get("torque_resolution")),
            ("紧凑速度反馈 / rad/s", velocity_resolution),
        ])
    content += ("<p>低速分析使用位置回归估计速度。主档只有有限次独立重复，百万条时间样本不等于百万次独立试验。"
                "报告 CI 含零偏与量化项，不能当作已完成独立绝对力矩校准；温度的整数反馈也不能证明温度完全不变。</p>")
    content += details("全部诊断字段", mapping_table(d))
    return section("diagnostics", "8. 数据与控制时序审计", content)


def fourier_section(data: ReportData) -> str:
    f = data.fourier
    if f is None:
        return ""
    settings, metrics = f.get("settings", {}), f.get("metrics", {})
    content = ("<p>有限整数谐波构成周期运动；位置、速度、加速度在周期边界连续归零。"
               "慢速与常速轨迹共同覆盖非线性摩擦区和惯性/黏性区，另用不同系数、幅度和基频的周期独立验证。</p>")
    content += table(("傅里叶辨识", "结果"), [
        ("实测 / 计划周期", [f.get("measured_cycles"), f.get("required_cycles")]),
        ("反馈采样率 / Hz", settings.get("feedback_rate")),
        ("训练基频 / Hz", settings.get("frequency")), ("谐波数", settings.get("harmonics")),
        ("训练 RMS / Nm", metrics.get("training_rmse")), ("独立验证 RMS / Nm", metrics.get("validation_rmse")),
        ("最大独立窗口误差 / Nm", metrics.get("maximum_validation_error")),
        ("标准化回归条件数", metrics.get("condition")),
        ("RMS / 最大误差目标 / Nm", [settings.get("rmse_limit"), settings.get("max_error")]),
        ("模型与独立静摩擦均合格", f.get("qualified")),
    ])
    candidate = f.get("candidate_model")
    if candidate:
        content += f'<h3>候选模型：{escape(candidate["model"])}</h3>'
        content += table(("参数", "辨识值", "单位"), ((name, value, candidate["units"][name])
                         for name, value in candidate["parameters"].items()))
        content += '<p class="muted">Stribeck 模型的 static 参数是滑动摩擦的低速极限，并不替代下面独立测得的零速起动阈值。</p>'
    if f.get("quality_issues"):
        content += '<div class="warning"><ul>' + "".join(f"<li>{escape(message)}</li>" for message in f["quality_issues"]) + "</ul></div>"
    for name, title in (("training", "训练"), ("validation", "独立验证")):
        signals = f.get(name + "_signals", {})
        if signals:
            t = signals["time"]
            content += chart(title + "周期位置：实测与参考", "分组周期相位时间 / s", "rad", [
                Series("实测周期均值", list(zip(t, signals["position"])), COLORS[0]),
                Series("参考轨迹", list(zip(t, signals["reference_position"])), COLORS[1], True)])
            if "predicted_torque" in signals:
                content += chart(title + "力矩相位诊断", "分组周期相位时间 / s", "Nm", [
                    Series("实测力矩均值", list(zip(t, signals["torque"])), COLORS[0]),
                    Series("带限位置导数模型示意", list(zip(t, signals["predicted_torque"])), COLORS[1], True)])
        content += details(title + "重复性、采样与重建诊断", mapping_table(f.get(name, {})))
    content += details("保存的傅里叶系数与连续约束上界", "<pre>" + escape(json.dumps(
        f.get("trajectories", {}), ensure_ascii=False, indent=2, allow_nan=False)) + "</pre>")
    content += ("<p>反馈高于指令更新频率；先对实测位置、速度、力矩做覆盖完整指令保持区间的时间积分平均，"
                "再逐周期拟合非线性模型，避免先平均速度再代入非线性摩擦函数的偏差。"
                "周期均值和实测位置傅里叶导数只作独立一致性/波形诊断，不代替实测速度积分。"
                "惯性、重力和摩擦被同时建模；含零速黏着或预算缩放的原始窗口不参与滑动拟合，但全部保留。"
                "上面的瞬时力矩示意包含近零速及带宽外成分；资格指标只针对有明确滑动证据的实测积分窗口。</p>")
    content += "<h3>方法依据与范围</h3><ul>"
    sources = (
        ("Swevers 等，1997：Optimal robot excitation and identification", "https://doi.org/10.1109/70.631234"),
        ("Swevers 等，2007：Dynamic Model Identification for Industrial Robots", "https://doi.org/10.1109/MCS.2007.904659"),
        ("Wu 等，2012：Closed-Loop Dynamic Parameter Identification ... Using Modified Fourier Series", "https://doi.org/10.5772/45818"),
        ("Bittencourt 与 Gunnarsson，2012：Load and Temperature Effects on Robot-Joint Friction", "https://doi.org/10.1115/1.4006589"),
        ("Ku、Li、Zhu，2026：Nonlinear Friction and Hierarchical Iterative Optimization", "https://doi.org/10.3390/machines14070783"),
    )
    content += "".join(f'<li><a href="{url}" rel="noreferrer">{escape(title)}</a></li>' for title, url in sources)
    content += ("</ul><p class=\"muted\">本实现采用周期傅里叶激励、约束/条件数设计、实测数据处理和独立验证，"
                "使用稳健积分最小二乘；不宣称复现论文的完整最大似然估计、分层机器人惯量优化或神经网络。</p>")
    return section("fourier", "傅里叶周期激励与动力学辨识", content)


def static_section(data: ReportData) -> str:
    static = data.static
    if static is None:
        return section("static", "6. 静摩擦与起动",
                       '<p class="unavailable">本记录未保存精密逐角度、逐试验静摩擦明细，不补造缺失结果。'
                       '其他通用起动辨识参数如有，见保存参数与补偿评估部分。</p>')
    content = '<p>正式试验与固定预处理分开保存。角度相对于本轮原基准；静摩擦还可能受到重力、未知外载和预载历史影响。</p>'
    content += table(("方向", "四角度平均阈值 / Nm", "最差角度重复 SD / Nm", "最差角度界限 / Nm"),
                     ((label, static.get(key, {}).get("torque"), static.get(key, {}).get("repeatability"),
                       static.get(key, {}).get("ci95")) for key, label in (("positive", "正向"), ("negative", "反向"))))
    content += '<p class="small muted">方向汇总的重复性和界限是最差角度组，不是四角度平均值的新置信区间。</p>'
    by_angle = static.get("by_angle", [])
    if by_angle:
        series = []
        for direction, label, color in ((1, "正向", COLORS[0]), (-1, "反向", COLORS[1])):
            rows = [r for r in finite_rows(by_angle, "angle", "torque") if r["direction"] == direction]
            errors = [r["ci95"] for r in rows] if all(r.get("ci95") is not None for r in rows) else None
            series.append(Series(label, [(math.degrees(r["angle"]), r["torque"]) for r in rows], color, errors=errors))
        content += chart("静摩擦的方向与角度分布", "相对角度 / °", "起动力矩幅值 / Nm", series)
        content += table(("角度 / °", "方向", "起动力矩 / Nm", "重复 SD / Nm", "CI95 / Nm", "有效重复", "计划重复"),
                         ((math.degrees(r["angle"]), *[r.get(k) for k in ("direction", "torque", "repeatability",
                                                                        "ci95", "repetitions", "required_repetitions")]) for r in by_angle))
    trials, conditioning = static.get("trials", []), static.get("conditioning", [])
    series = []
    for rows, name, dashed in ((trials, "正式", False), (conditioning, "预处理", True)):
        for direction, color in ((1, COLORS[0]), (-1, COLORS[1])):
            points = [(float(r.get("acquisition_trial", r["trial"])) + 1, r["threshold"])
                      for r in finite_rows(rows, "threshold") if r.get("direction") == direction]
            series.append(Series(name + ("正向" if direction > 0 else "反向"), sorted(points), color, dashed))
    if trials or conditioning:
        content += chart("起动阈值的逐次变化", "原始采集试验序号（含预处理）", "起动力矩幅值 / Nm", series)
    for rows, label in ((trials, "正式静摩擦"), (conditioning, "固定预处理")):
        if rows:
            content += details(f"{label}完整明细（{len(rows)} 项）", table(
                ("试验索引", "原始索引", "方向", "相对角度 / rad", "有效", "阈值 / Nm", "原始样本", "稳态平均样本", "响应 RMSE / Nm", "起动时刻", "原因"),
                ([r.get(k) for k in ("trial", "acquisition_trial", "direction", "angle", "valid", "threshold", "samples",
                                    "stationary_samples", "response_rmse", "onset_time", "reason")] for r in rows)))
    errors = static.get("quality_errors", [])
    if errors:
        content += '<div class="warning"><ul>' + "".join(f"<li>{escape(str(error))}</li>" for error in errors) + "</ul></div>"
    return section("static", "6. 静摩擦与起动", content)


def warmup_section(data: ReportData) -> str:
    warmup = data.warmup
    if warmup is None or not warmup.get("enabled"):
        return ""
    content = ("<p>预运行包含所有主速度档的正反向实际旋转。它独立于正式辨识：预运行样本不计入静摩擦、"
               "稳速训练或独立验证，回到控制零位并确认失能后才重新测量零偏。</p>")
    content += table(("预运行证据", "记录值"), [
        ("按计划完成并回零", warmup.get("complete")),
        ("已完成 / 计划速度单元", [warmup.get("completed_cells"), warmup.get("required_cells")]),
        ("阶段时长 / s", warmup.get("duration")), ("本阶段样本", warmup.get("samples")),
        ("控制零位 / rad", warmup.get("zero_position")), ("实测回零位置 / rad", warmup.get("zero_return_position")),
        ("阶段温度范围 / °C", warmup.get("temperature_range")),
    ])
    cells = warmup.get("cells", [])
    if cells:
        start = warmup.get("start_time")
        if isinstance(start, (int, float)):
            series = []
            for field, time_field, label, color in (
                ("temperature_start", "start_time", "旋转窗口起点", COLORS[0]),
                ("temperature_end", "end_time", "旋转窗口终点", COLORS[1])):
                rows = finite_rows(cells, field, time_field)
                series.append(Series(label, [(row[time_field] - start, row[field]) for row in rows], color))
            content += chart("预运行温度记录", "预运行阶段时间 / s", "°C", series)
        content += details(f"全部预运行单元（{len(cells)} 项）", table(
            ("序号", "速度目标", "方向", "轮次索引", "完整", "实测均速", "要求圈数", "实测圈数", "时长 / s", "样本", "温度范围"),
            ([row.get(key) for key in ("cell", "speed", "direction", "repetition", "complete", "mean_speed",
                                      "required_turns", "measured_turns", "duration", "samples", "temperature_range")] for row in cells)))
    sine = warmup.get("sine")
    if sine is not None:
        if not isinstance(sine, dict):
            raise CalibrationError("Sine-conditioning report must be an object")
        content += "<h3>变频率正弦往复</h3>"
        content += table(("正弦预运行", "记录值"), [
            ("实际完成 / 目标周期", [sine.get("completed_cycles"), sine.get("required_cycles")]),
            ("中心到峰值幅度 / rad", sine.get("amplitude")), ("峰峰总摆幅 / rad", sine.get("peak_to_peak")),
            ("起始 / 终止频率 / Hz", [sine.get("start_frequency"), sine.get("end_frequency")]),
            ("计划时长 / s", sine.get("planned_duration")), ("参考峰值速度 / rad/s", sine.get("peak_reference_speed")),
            ("参考加速度上界 / rad/s²", sine.get("peak_reference_acceleration")),
            ("实测最大速度 / rad/s", sine.get("peak_measured_speed")),
            ("位置跟踪 RMSE / rad", sine.get("tracking_rmse")), ("最大跟踪误差 / rad", sine.get("tracking_max_error")),
            ("计为完整往复的幅值比例", sine.get("measured_fraction_required")),
        ])
        if data.sine_trace and data.trace_fields:
            fields = {name: index for index, name in enumerate(data.trace_fields)}
            start = data.sine_trace[0][fields["time"]]
            series = [Series(name, [(row[fields["time"]] - start, row[fields[name]]) for row in data.sine_trace],
                             COLORS[index], index > 0) for index, name in enumerate(("position", "target_position"))
                      if name in fields]
            content += chart("扫频正弦实测与参考", "正弦段首条样本起算 / s", "位置 / rad", series)
        cycles = sine.get("cycles", [])
        content += details(f"实测正弦周期（{len(cycles)} 次）", table(
            ("周期", "开始时刻", "结束时刻", "时长 / s", "样本", "位置最小", "位置最大", "实测峰峰幅度", "结束参考频率"),
            ([row.get(key) for key in ("cycle", "start_time", "end_time", "duration", "samples",
                                      "position_min", "position_max", "measured_peak_to_peak", "reference_frequency")]
             for row in cycles)))
        content += "<p>周期数由实测位置交替达到正、负幅值门限确认，不把时间流逝或发送了目标当成完成运动。正弦数据不进入正式摩擦拟合。</p>"
    content += '<p class="warning">完成固定预运行只能证明运动覆盖已执行。温度反馈通常为整数，显示不变不代表已独立确认热平衡；是否减少启动影响，应比较正式测量的跨轮重复性和验证误差。</p>'
    return section("warmup", "正式标定前的双向多速度预运行", content)


def trace_section(data: ReportData) -> str:
    content = (f'<p>图表全部内嵌，单图最多 {MAX_PLOT_POINTS} 点；对时间序列采用保留极值的有界抽样。'
               '数百万原始采样行不重复嵌入 HTML，完整统计与明细表不使用图形抽样重算。原始文件仍应单独保存。</p>')
    parts = [*data.parents, data]
    for index, part in enumerate(parts, 1):
        title = f"第 {index} 段 · {part.root.name}" if len(parts) > 1 else "本轮全过程"
        body = f'<p class="small">保存状态：{escape(STATUS_NAMES[part.report["status"]])}；轨迹源样本：{escape(value(part.diagnostics.get("sample_rows")))}；'
        body += f'有界概览保留：{len(part.trace)} 行。每段时间从自己的记录起点计算，不跨暂停连线。</p>'
        if part.trace and part.trace_fields:
            fields = {name: column for column, name in enumerate(part.trace_fields)}
            for plot_title, names, ylabel in (
                ("位置与参考", ("position", "target_position"), "rad"),
                ("速度与参考", ("velocity", "target_velocity"), "rad/s"),
                ("力矩与控制估计", ("torque", "command_torque"), "Nm"),
                ("温度", ("temperature",), "°C"),
                ("电流", ("current",), "A"),
                ("电压", ("voltage",), "V"),
            ):
                selected = [name for name in names if name in fields]
                if selected:
                    series = [Series(name, [(r[fields["time"]], r[fields[name]]) for r in part.trace],
                                     COLORS[i % len(COLORS)], i > 0) for i, name in enumerate(selected)]
                    body += chart(plot_title, "本段时间 / s", ylabel, series)
        else:
            body += '<p class="unavailable">没有可用的原始时序文件；本报告不会把拟合曲线伪装成原始轨迹。</p>'
        content += details(title, body) if len(parts) > 1 else body
    return section("timeline", "7. 全过程运动与温度", content)


def generic_section(data: ReportData) -> str:
    content = ""
    if data.fit is not None:
        fit = data.fit
        content += f"<p>参数类别：{escape(fit.kind)}；模型：{escape(fit.model)}。</p>"
        content += table(("参数", "结果", "单位"), ((name, entry, fit.units.get(name)) for name, entry in fit.parameters.items()))
        content += details("拟合诊断与适用域", mapping_table({**fit.diagnostics, "domain": fit.domain}))
        if fit.warnings:
            content += "<ul>" + "".join(f"<li>{escape(warning)}</li>" for warning in fit.warnings) + "</ul>"
    if data.report.get("comparison") is not None:
        content += details("独立轨迹 / 补偿对比", "<pre>" + escape(json.dumps(
            redact(data.report["comparison"]), ensure_ascii=False, indent=2, allow_nan=False)) + "</pre>")
    if data.report.get("breakaway") is not None:
        content += details("其它起动试验结果", "<pre>" + escape(json.dumps(
            redact(data.report["breakaway"]), ensure_ascii=False, indent=2, allow_nan=False)) + "</pre>")
    return section("parameters", "保存的物理参数与补偿评估", content) if content else ""


def render_html(data: ReportData, generated: str) -> str:
    motor = data.identity.get("motor_id")
    name = f"电机 {motor}" if motor is not None else "离线数据"
    title = name + " · 标定分析报告"
    state = data.report["status"]
    p = data.precision or data.fourier or {}
    metrics = p.get("metrics") or {}
    issues = p.get("quality_issues") or []
    status_class = "failure" if state in ("failed", "incomplete") else "warning" if state == "completed_unqualified" else "badge"
    summary = f'<header><h1>{escape(title)}</h1><div class="{status_class}">{escape(STATUS_NAMES[state])}</div>'
    summary += f'<p class="muted">记录：<code>{escape(data.root.name)}</code> · 报告生成：{escape(generated)}'
    summary += ' · 此文件完全离线、只读，不连接电机，不包含 START/STOP 控制。</p>'
    cards = [
        ("本段样本", data.report.get("samples", data.diagnostics.get("sample_rows"))),
        ("累计样本", data.report.get("cumulative_samples", data.report.get("samples", data.diagnostics.get("sample_rows")))),
        ("实测 / 计划周期" if data.fourier is not None else "完整 / 计划速度单元",
         f'{value(p.get("measured_cells", p.get("completed_cells")))} / {value(p.get("required_cells", p.get("cells")))}'),
        ("验证 RMSE / Nm", metrics.get("validation_rmse")),
        ("质量提示", len(issues) if issues else "见各项诊断"),
    ]
    cards_html = '<div class="cards">' + "".join(
        f'<div class="card"><b>{escape(value(item))}</b>{escape(label)}</div>' for label, item in cards) + "</div>"
    friction = primary_friction_curve(data)
    navigation = [("friction-curve", "摩擦曲线")] if friction else []
    navigation.append(("overview", "结论与来源"))
    if data.warmup is not None and data.warmup.get("enabled"):
        navigation.append(("warmup", "预运行 / 预热"))
    if data.precision is not None:
        navigation.append(("quality", "精度"))
        if p.get("levels"):
            navigation.append(("friction", "幅值 / 低速"))
    elif data.fourier is not None:
        navigation.append(("fourier", "傅里叶辨识"))
    elif data.fit is not None or data.report.get("comparison") is not None or data.report.get("breakaway") is not None:
        navigation.append(("parameters", "参数 / 评估"))
    navigation.extend([("static", "静摩擦"), ("timeline", "全过程"), ("diagnostics", "日志审计"),
                       ("events", "完整事件"), ("configuration", "配置与复现")])
    summary += "<nav>" + "".join(f'<a href="#{identifier}">{label}</a>' for identifier, label in navigation) + "</nav></header>"
    overview = cards_html + '<p><b>采集完整、数据完整性、参数精度和绝对标尺准确度是不同结论。</b>质量未达标时不生成新的合格模型，不覆盖原始报告，也不通过删除数据提高分数。</p>'
    overview += table(("项目", "记录值"), [
        ("保存状态", state), ("实验类型", data.report.get("experiment", data.report.get("operation", "离线拟合"))),
        ("原始记录创建时间（UTC）", (data.metadata or {}).get("created_utc")),
        ("采集计划完成", data.report.get("acquisition_complete")),
        ("本次结果达到记录中的资格标准", data.report.get("qualified", state in ("completed", "qualified") and data.fit is not None)),
        ("记录时已确认失能", data.report.get("stop_confirmed")), ("运行错误", data.report.get("error") or "未记录错误"),
        ("失败阶段", data.report.get("failure_stage")), ("累计采集时间（含预运行）/ s", p.get("elapsed_time")),
        ("正式标定时间 / s", p.get("formal_elapsed_time")),
        ("本段原始输入 SHA-256 / 列定义核验", data.verified_inputs), ("生成时间（UTC）", generated),
    ])
    overview += mapping_table({key: data.identity[key] for key in
                              ("namespace", "motor_id", "simulated", "mode", "direction", "software_offset", "pmax", "vmax", "tmax", "source")
                              if key in data.identity})
    if data.report.get("error"):
        overview += f'<div class="failure">{escape(str(data.report["error"]))}</div>'
    notices = [notice for part in [*data.parents, data] for notice in part.notices]
    if notices:
        overview += '<div class="warning"><ul>' + "".join(f"<li>{escape(notice)}</li>" for notice in notices) + "</ul></div>"
    body = friction + section("overview", "1. 结论、范围与来源", overview)
    body += warmup_section(data)
    body += precision_sections(data)
    body += fourier_section(data)
    if data.precision is None and data.fourier is None:
        body += generic_section(data)
    body += static_section(data)
    body += trace_section(data)
    body += diagnostic_section(data)
    event_content = ""
    for index, part in enumerate([*data.parents, data], 1):
        event_rows = []
        epoch = (part.metadata or {}).get("acquisition_epoch")
        for event in part.events:
            timestamp = event.get("time")
            relative = timestamp - epoch if isinstance(timestamp, (int, float)) and isinstance(epoch, (int, float)) else event.get("elapsed")
            event_rows.append((relative, event.get("scope"), event.get("kind"), event.get("phase", event.get("stage")),
                               {key: item for key, item in event.items() if key not in ("time", "scope", "kind", "phase", "stage")}))
        label = f"第 {index} 段 · " if data.parents else ""
        event_content += details(
            f"{label}展开已保存事件（{len(event_rows)} 条）",
            table(("本段相对时间 / s", "来源", "类型", "阶段", "完整附加字段"), event_rows))
    body += section("events", "9. 完整事件与阶段", event_content)
    configuration = ('<p>本次为手动拖动，不设额外速度 / 相对行程门限；配置中的 null 表示未设置，'
                     '不表示关闭驱动保护。力矩、温度和反馈保护保留。</p>'
                     if (data.report.get("configuration") or {}).get("drag_motion_policy") == "driver_only" else
                     '<p>本精密流程实际使用 precision settings；完整节点参数中同名的通用 max_speed 等字段不应误认为本流程的保护值。</p>')
    configuration += details("实际傅里叶设置" if data.fourier is not None else "实际精密设置",
                             mapping_table(p.get("settings") or (data.metadata or {}).get("precision_settings") or {}))
    if (data.report.get("configuration") or {}).get("drag_stop_policy") == "operator":
        configuration += '<p>拖动由用户开启和停止，无定时结束；duration=null。正常手动停止不代表生成新的标定模型。</p>'
    if (data.report.get("configuration") or {}).get("drag_speed_policy") == "operator":
        configuration += ('<p>本次使用显式零增益拖动使能：仅当前来源的有效连续力矩指令免于驱动实测速度停机门限。'
                          '配置中记录的 speed_stop 仍是自动任务门限，不是本次拖动的停止速度；'
                          '绝对位置、力矩、温度和失联保护保持有效。</p>')
    if data.fourier is not None:
        configuration += details("预运行、零偏与静摩擦设置", mapping_table(p.get("preparation_settings") or {}))
    configuration += details("完整节点 / 导入配置", mapping_table(data.report.get("configuration") or {}))
    if p.get("baseline_segments"):
        configuration += details("分段零偏（续采不跨段混用零偏）", table(
            ("起点时刻", "终点时刻", "零偏 / Nm", "本段 CI / Nm", "位置稳定", "样本"),
            ([row.get(key) for key in ("start_time", "end_time", "offset", "ci95", "stationary", "samples")]
             for row in p["baseline_segments"])))
    if data.metadata is not None:
        configuration += details("记录清单、量化范围、时钟、环境与源码指纹", "<pre>" + escape(json.dumps(
            redact(data.metadata), ensure_ascii=False, indent=2, allow_nan=False)) + "</pre>")
    for index, parent in enumerate(data.parents, 1):
        configuration += details(f"父段 {index} 的完整报告、配置、指纹与日志统计", "<pre>" + escape(json.dumps(
            redact({"report": parent.report, "recording": parent.metadata, "diagnostics": parent.diagnostics}),
            ensure_ascii=False, indent=2, allow_nan=False)) + "</pre>")
    configuration += details("完整保存结果 JSON（保留全部数值精度）", "<pre>" + escape(json.dumps(
        redact({"report": data.report, "static": data.static}), ensure_ascii=False, indent=2, allow_nan=False)) + "</pre>")
    configuration += ('<div class="warning">傅里叶运动模式可辨识惯性与滑动摩擦；其结果仍依赖电流力矩标尺、负载模型和有效激励。'
                      '动力学预测通过不等于绝对力矩已外部校准，也不能从动态零速穿越直接断言静摩擦峰值。</div>'
                      if data.fourier is not None else
                     '<div class="warning">稳速精密流程不独立辨识惯量；inertia=0 若出现是占位，不是真实惯量为零。'
                      'R/L/Kt、独立外载、齿隙等没有独立测量时不能唯一恢复。低速速度/力矩量化、相关噪声、温漂和装夹均影响精度；'
                      '统计平均低于单包量化步长，不等于绝对力矩标尺同样准确。候选模型投入补偿前仍需独立验证。</div>')
    configuration += '<p>HTML 是生成时刻的分析快照，图表和样式均已内嵌，可复制到其它机器直接打开。原始 CSV 未嵌入；重新辨识仍需保留完整记录目录及续采 parent/。浏览器查找和打印功能可用于检索、保存本报告。</p>'
    body += section("configuration", "10. 配置、完整数值与复现说明", configuration)
    return ('<!doctype html><html lang="zh-CN"><head><meta charset="utf-8">'
            '<meta name="viewport" content="width=device-width,initial-scale=1">'
            '<meta name="referrer" content="no-referrer">'
            f'<meta http-equiv="Content-Security-Policy" content="{escape(REPORT_CSP, quote=True)}">'
            f'<title>{escape(title)}</title><style>{CSS}</style></head><body><main>{summary}{body}'
            '<footer>本报告只读。记录中的状态并不是当前实时电机状态；原始数据和原始资格判定未被修改。</footer></main></body></html>')


def report_status(directory: Path) -> dict:
    root = directory.resolve()
    output, meta_path = input_path(root, REPORT_NAME), input_path(root, REPORT_META)
    if not meta_path.is_file() or not output.is_file():
        return {"status": "missing", "error": ""}
    metadata = read_json(meta_path)
    if not isinstance(metadata, dict) or metadata.get("kind") != "motor_calibration_html":
        raise CalibrationError("Unsupported HTML report metadata")
    if metadata.get("status") != "ready":
        return {"status": metadata.get("status", "failed"), "error": metadata.get("error", "")}
    source = input_path(root, "report.json")
    if (metadata.get("version") != REPORT_VERSION or
            metadata.get("source_report_sha256") != file_sha256(source) or
            output.stat().st_size != metadata.get("html_bytes")):
        return {"status": "stale", "error": "报告输入或生成版本已改变，需要重新生成"}
    return {key: metadata[key] for key in ("status", "generated_utc", "html_bytes", "source_status")}


def generate_html_report(directory: Path, output: Path | None = None, *, overwrite: bool = False) -> dict:
    root = directory.expanduser().resolve()
    target = (output or root / REPORT_NAME).expanduser().resolve()
    if target.suffix.lower() not in (".html", ".htm") or (output is None and target.parent != root):
        raise CalibrationError("Report output must be an HTML file and must not replace an input artifact")
    if target.exists() and not overwrite:
        raise CalibrationError(f"HTML output already exists: {target}; request overwrite explicitly")
    data = load_report_data(root)
    generated = datetime.now(timezone.utc).isoformat()
    document = render_html(data, generated).encode("utf-8")
    target.parent.mkdir(parents=True, exist_ok=True)
    temporary = None
    try:
        with tempfile.NamedTemporaryFile(dir=target.parent, prefix="." + target.name + ".", suffix=".tmp", delete=False) as stream:
            temporary = Path(stream.name)
            stream.write(document)
            stream.flush()
            os.fsync(stream.fileno())
        temporary.replace(target)
        temporary = None
    finally:
        if temporary is not None:
            temporary.unlink()
    metadata = {
        "schema": 1, "kind": "motor_calibration_html", "version": REPORT_VERSION, "status": "ready",
        "generated_utc": generated, "source_status": data.report["status"],
        "source_report_sha256": file_sha256(input_path(root, "report.json")),
        "html_sha256": file_sha256(target), "html_bytes": len(document), "standalone": True,
        "identity": redact(data.identity), "source_samples": data.report.get("samples", data.diagnostics.get("sample_rows")),
        "verified_inputs": data.verified_inputs, "max_plot_points": MAX_PLOT_POINTS,
        "overview_rows": len(data.trace), "environment": environment_versions(),
        "generator_sources": {name: file_sha256(Path(__file__).with_name(name)) for name in
                              ("html_report.py", "report_plot.py", "report_data.py")},
    }
    atomic_json(target.with_name(target.name + ".meta.json"), metadata)
    return {"output": str(target), "metadata": str(target.with_name(target.name + ".meta.json")),
            "source_status": metadata["source_status"], "samples": metadata["source_samples"],
            "html_bytes": len(document), "standalone": True, "verified_inputs": data.verified_inputs}
