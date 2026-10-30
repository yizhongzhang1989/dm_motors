"""Self-contained, bounded SVG plots for offline calibration reports."""

from dataclasses import dataclass
from html import escape
import math
import re
from typing import Sequence

import numpy as np

from .data import CalibrationError


MAX_PLOT_POINTS = 1200
COLORS = ("#1768ac", "#c24e16", "#16856b", "#8b57a5")


@dataclass
class Series:
    label: str
    points: list[tuple[float, float]]
    color: str = COLORS[0]
    dashed: bool = False
    errors: list[float] | None = None
    connect: bool = True


def bounded_points(points: Sequence[tuple[float, float]], limit: int) -> list[tuple[float, float]]:
    if limit < 4:
        raise CalibrationError("Plot point limit must be at least four")
    if len(points) <= limit:
        return list(points)
    values = np.asarray(points, dtype=float)
    indices = {0, len(values) - 1}
    for bucket in np.array_split(np.arange(1, len(values) - 1), max(1, (limit - 2) // 2)):
        if len(bucket):
            indices.add(int(bucket[np.argmin(values[bucket, 1])]))
            indices.add(int(bucket[np.argmax(values[bucket, 1])]))
    return [points[index] for index in sorted(indices)]


def plot(title: str, xlabel: str, ylabel: str, series: Sequence[Series], *,
         logarithmic: bool = False, references: Sequence[tuple[float, str]] = (),
         symmetric_x: bool = False) -> str:
    """Render inline SVG; the same full-turn/statistical outputs used online remain unchanged."""
    available = [s for s in series if s.points]
    if not available:
        return f'<p class="unavailable">{escape(title)}：没有可绘制的数据。</p>'
    if len(available) > 4:
        raise CalibrationError("A report chart supports at most four readable series")
    limit = max(4, MAX_PLOT_POINTS // len(available))
    displayed = []
    for item in available:
        if not re.fullmatch(r"#[0-9a-fA-F]{6}", item.color):
            raise CalibrationError("Unsupported report plot color")
        if not all(math.isfinite(value) for point in item.points for value in point):
            raise CalibrationError(f"Nonfinite report plot data: {title}")
        if item.errors is not None:
            if (len(item.errors) != len(item.points) or
                    any(not math.isfinite(value) or value < 0 for value in item.errors)):
                raise CalibrationError(f"Invalid error bars: {title}")
            if len(item.points) > limit:
                raise CalibrationError("Statistical error-bar plots exceed the bounded report size")
        displayed.append(Series(item.label, bounded_points(item.points, limit), item.color,
                                item.dashed, item.errors, item.connect))
    points = [point for item in displayed for point in item.points]
    xmin, xmax = min(p[0] for p in points), max(p[0] for p in points)
    if symmetric_x:
        if logarithmic:
            raise CalibrationError("A signed symmetric axis cannot be logarithmic")
        bound = max(abs(xmin), abs(xmax), 1e-6)
        xmin, xmax = -bound, bound
    ys = [p[1] for p in points] + [value for value, _ in references]
    for item in displayed:
        if item.errors is not None:
            ys.extend(y + direction * error for (_, y), error in zip(item.points, item.errors) for direction in (-1, 1))
    ymin, ymax = min(0.0, min(ys)), max(ys)
    if logarithmic and xmin <= 0:
        raise CalibrationError("Logarithmic velocity plots require positive measured speeds")
    if xmin == xmax:
        if logarithmic:
            xmin, xmax = xmin / 1.1, xmax * 1.1
        else:
            delta = max(0.01, abs(xmin) * 0.05)
            xmin, xmax = xmin - delta, xmax + delta
    if ymin == ymax:
        delta = max(1e-3, abs(ymin) * 0.05)
        ymin, ymax = ymin - delta, ymax + delta
    margin = (ymax - ymin) * 0.08
    ymin, ymax = ymin - margin, ymax + margin
    width, height = 1100, 510
    left, right, top, bottom = 94, 26, 90, 77
    transform = math.log10 if logarithmic else float
    def x(value: float) -> float:
        return left + (transform(value) - transform(xmin)) / (transform(xmax) - transform(xmin)) * (width - left - right)
    def y(value: float) -> float:
        return height - bottom - (value - ymin) / (ymax - ymin) * (height - top - bottom)
    parts = [
        f'<svg xmlns="http://www.w3.org/2000/svg" viewBox="0 0 {width} {height}" role="img" '
        f'aria-label="{escape(title, quote=True)}" data-points="{len(points)}" '
        f'data-x-min="{xmin:.12g}" data-x-max="{xmax:.12g}" data-x-scale="{"log" if logarithmic else "linear"}">',
        f'<title>{escape(title)}</title><rect width="100%" height="100%" fill="white"/>',
        '<g font-family="sans-serif" fill="#243445">',
        f'<text x="{left}" y="28" font-size="19" font-weight="bold">{escape(title)}</text>',
    ]
    for value in np.linspace(ymin, ymax, 6):
        yy = y(float(value))
        parts.append(f'<line x1="{left}" y1="{yy:.2f}" x2="{width-right}" y2="{yy:.2f}" stroke="#e1e7ed"/>')
        parts.append(f'<text x="{left-10}" y="{yy+5:.2f}" text-anchor="end" font-size="13">{value:.4g}</text>')
    ticks = np.geomspace(xmin, xmax, 7) if logarithmic else np.linspace(xmin, xmax, 7)
    for value in ticks:
        xx = x(float(value))
        parts.append(f'<line x1="{xx:.2f}" y1="{top}" x2="{xx:.2f}" y2="{height-bottom}" stroke="#edf0f3"/>')
        parts.append(f'<text x="{xx:.2f}" y="{height-bottom+23}" text-anchor="middle" font-size="13">{value:.4g}</text>')
    for value, label in references:
        yy = y(value)
        parts.append(f'<line x1="{left}" y1="{yy:.2f}" x2="{width-right}" y2="{yy:.2f}" stroke="#a33838" stroke-dasharray="5 5"/>')
        parts.append(f'<text x="{width-right-4}" y="{yy-5:.2f}" text-anchor="end" fill="#a33838" font-size="12">{escape(label)}</text>')
    for index, item in enumerate(displayed):
        dash = ' stroke-dasharray="7 4"' if item.dashed else ""
        coordinates = " ".join(f"{x(xx):.2f},{y(yy):.2f}" for xx, yy in item.points)
        if item.connect:
            parts.append(f'<polyline points="{coordinates}" fill="none" stroke="{item.color}" stroke-width="2"{dash}/>')
        if item.errors is not None:
            for (xx, yy), error in zip(item.points, item.errors):
                parts.append(f'<line x1="{x(xx):.2f}" y1="{y(yy-error):.2f}" x2="{x(xx):.2f}" '
                             f'y2="{y(yy+error):.2f}" stroke="{item.color}" opacity="0.55"/>')
        if len(item.points) <= 120 or not item.connect:
            for point_index, (xx, yy) in enumerate(item.points):
                error = f", reported bound={item.errors[point_index]:.6g}" if item.errors is not None else ""
                tooltip = escape(f"{item.label}: x={xx:.9g}, y={yy:.9g}{error}")
                parts.append(f'<circle cx="{x(xx):.2f}" cy="{y(yy):.2f}" r="3" fill="{item.color}"><title>{tooltip}</title></circle>')
        lx = left + index * 245
        parts.append(f'<line x1="{lx}" y1="55" x2="{lx+25}" y2="55" stroke="{item.color}" stroke-width="3"{dash}/>')
        parts.append(f'<text x="{lx+32}" y="60" font-size="13">{escape(item.label)}</text>')
    parts.extend([
        f'<text x="{(left+width-right)/2}" y="{height-20}" text-anchor="middle" font-size="15">{escape(xlabel)}</text>',
        f'<text transform="translate(22,{(top+height-bottom)/2}) rotate(-90)" text-anchor="middle" font-size="15">{escape(ylabel)}</text>',
        '</g></svg>',
    ])
    return "".join(parts)
