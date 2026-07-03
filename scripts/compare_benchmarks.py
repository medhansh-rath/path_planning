#!/usr/bin/env python3
"""Create comparison plots from benchmark CSV output.

This script reads benchmark_results.csv and writes lightweight SVG plots plus a
compact HTML index so the benchmark can be reviewed without additional tooling.
"""

from __future__ import annotations

import argparse
import csv
from collections import defaultdict
from html import escape
from pathlib import Path
from statistics import median
from typing import Dict, Iterable, List, Tuple

METRICS = [
    ("initial_plan_ms", "Initial plan time (ms)"),
    ("total_replan_ms", "Total replanning time (ms)"),
    ("total_wall_ms", "Total wall time (ms)"),
    ("path_length", "Path length"),
    ("path_cost", "Path cost"),
    ("path_efficiency", "Path efficiency"),
    ("turns", "Path turns"),
    ("expansions", "Node expansions"),
    ("memory_high_water_kb", "Memory high water (KB)"),
]

COLORS = [
    "#0f766e",
    "#2563eb",
    "#dc2626",
    "#7c3aed",
    "#ea580c",
    "#059669",
    "#be123c",
]


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--input", required=True, help="Benchmark CSV file")
    parser.add_argument("--output", required=True, help="Directory to write SVG plots and index.html")
    return parser


def read_rows(csv_path: Path) -> List[dict]:
    with csv_path.open(newline="") as handle:
        return list(csv.DictReader(handle))


def group_data(rows: Iterable[dict]) -> Tuple[List[str], List[int], Dict[str, Dict[int, dict]]]:
    planners = sorted({row["planner"] for row in rows})
    sizes = sorted({int(row["width"]) for row in rows})
    grouped: Dict[str, Dict[int, dict]] = defaultdict(dict)
    for row in rows:
        grouped[row["planner"]][int(row["width"])] = row
    return planners, sizes, grouped


def parse_float(row: dict, key: str) -> float:
    try:
        return float(row[key])
    except (KeyError, TypeError, ValueError):
        return 0.0


def metric_series(grouped: Dict[str, Dict[int, dict]], planners: List[str], sizes: List[int], metric: str) -> Dict[str, List[float]]:
    series: Dict[str, List[float]] = {}
    for planner in planners:
        values = []
        for size in sizes:
            row = grouped.get(planner, {}).get(size)
            values.append(parse_float(row, metric) if row else 0.0)
        series[planner] = values
    return series


def svg_escape(text: str) -> str:
    return escape(text, quote=True)


def format_tick(value: float) -> str:
    if abs(value) >= 1000:
        return f"{value:,.0f}"
    if abs(value) >= 10:
        return f"{value:.1f}"
    return f"{value:.2f}"


def render_svg(metric_key: str, metric_title: str, planners: List[str], sizes: List[int], series: Dict[str, List[float]]) -> str:
    width = 1100
    height = 700
    margin_left = 90
    margin_right = 200
    margin_top = 70
    margin_bottom = 90
    plot_width = width - margin_left - margin_right
    plot_height = height - margin_top - margin_bottom

    all_values = [value for planner_values in series.values() for value in planner_values]
    max_value = max(all_values) if all_values else 1.0
    min_value = min(all_values) if all_values else 0.0
    if max_value == min_value:
        max_value += 1.0
    padded_min = min(0.0, min_value)
    padded_max = max_value * 1.08 if max_value > 0 else 1.0

    def x_for_index(index: int) -> float:
        if len(sizes) == 1:
            return margin_left + plot_width / 2
        return margin_left + (index / (len(sizes) - 1)) * plot_width

    def y_for_value(value: float) -> float:
        normalized = (value - padded_min) / (padded_max - padded_min)
        normalized = max(0.0, min(1.0, normalized))
        return margin_top + (1.0 - normalized) * plot_height

    parts = [
        f'<svg xmlns="http://www.w3.org/2000/svg" width="{width}" height="{height}" viewBox="0 0 {width} {height}">',
        '<rect width="100%" height="100%" fill="#f8fafc"/>',
        f'<text x="{margin_left}" y="34" font-size="28" font-family="Inter, Arial, sans-serif" fill="#0f172a">{svg_escape(metric_title)}</text>',
        f'<text x="{margin_left}" y="58" font-size="14" font-family="Inter, Arial, sans-serif" fill="#475569">Planners compared across grid sizes</text>',
        f'<line x1="{margin_left}" y1="{margin_top + plot_height}" x2="{margin_left + plot_width}" y2="{margin_top + plot_height}" stroke="#334155" stroke-width="1.4"/>',
        f'<line x1="{margin_left}" y1="{margin_top}" x2="{margin_left}" y2="{margin_top + plot_height}" stroke="#334155" stroke-width="1.4"/>',
    ]

    grid_lines = 5
    for i in range(grid_lines + 1):
        y = margin_top + (plot_height * i / grid_lines)
        value = padded_max - (padded_max - padded_min) * (i / grid_lines)
        parts.append(f'<line x1="{margin_left}" y1="{y:.1f}" x2="{margin_left + plot_width}" y2="{y:.1f}" stroke="#e2e8f0" stroke-width="1"/>')
        parts.append(
            f'<text x="{margin_left - 12}" y="{y + 4:.1f}" text-anchor="end" font-size="12" font-family="Inter, Arial, sans-serif" fill="#475569">{svg_escape(format_tick(value))}</text>'
        )

    for index, size in enumerate(sizes):
        x = x_for_index(index)
        parts.append(f'<line x1="{x:.1f}" y1="{margin_top + plot_height}" x2="{x:.1f}" y2="{margin_top + plot_height + 6}" stroke="#334155" stroke-width="1"/>')
        parts.append(
            f'<text x="{x:.1f}" y="{margin_top + plot_height + 26}" text-anchor="middle" font-size="12" font-family="Inter, Arial, sans-serif" fill="#475569">{size}x{size}</text>'
        )

    for planner_index, planner in enumerate(planners):
        values = series[planner]
        points = []
        color = COLORS[planner_index % len(COLORS)]
        for index, value in enumerate(values):
            x = x_for_index(index)
            y = y_for_value(value)
            points.append(f"{x:.1f},{y:.1f}")
        parts.append(
            f'<polyline fill="none" stroke="{color}" stroke-width="3" points="{" ".join(points)}"/>'
        )
        for index, value in enumerate(values):
            x = x_for_index(index)
            y = y_for_value(value)
            parts.append(f'<circle cx="{x:.1f}" cy="{y:.1f}" r="4.5" fill="{color}" stroke="#ffffff" stroke-width="1.5"/>')

    legend_x = margin_left + plot_width + 24
    legend_y = margin_top + 12
    parts.append(f'<text x="{legend_x}" y="{legend_y}" font-size="16" font-family="Inter, Arial, sans-serif" fill="#0f172a">Legend</text>')
    for planner_index, planner in enumerate(planners):
        color = COLORS[planner_index % len(COLORS)]
        y = legend_y + 24 + planner_index * 28
        parts.append(f'<rect x="{legend_x}" y="{y - 12}" width="16" height="16" rx="3" fill="{color}"/>')
        parts.append(f'<text x="{legend_x + 24}" y="{y + 1}" font-size="13" font-family="Inter, Arial, sans-serif" fill="#334155">{svg_escape(planner)}</text>')

    parts.append(f'<text x="{margin_left}" y="{height - 24}" font-size="12" font-family="Inter, Arial, sans-serif" fill="#64748b">Metric: {svg_escape(metric_key)}</text>')
    parts.append('</svg>')
    return "\n".join(parts)


def render_index(output_dir: Path, metrics: List[Tuple[str, str]]) -> str:
    links = "\n".join(
        f'<li><a href="{metric_key}.svg">{escape(metric_title)}</a></li>'
        for metric_key, metric_title in metrics
    )
    return f"""<!doctype html>
<html lang="en">
<head>
  <meta charset="utf-8" />
  <meta name="viewport" content="width=device-width, initial-scale=1" />
  <title>Path Planning Benchmark Comparison</title>
  <style>
    body {{ font-family: Inter, Arial, sans-serif; margin: 0; background: #0f172a; color: #e2e8f0; }}
    main {{ max-width: 1200px; margin: 0 auto; padding: 32px; }}
    .card {{ background: #111827; border: 1px solid #1f2937; border-radius: 18px; padding: 24px; box-shadow: 0 20px 60px rgba(15, 23, 42, 0.35); }}
    a {{ color: #38bdf8; text-decoration: none; }}
    a:hover {{ text-decoration: underline; }}
    ul {{ line-height: 1.9; }}
    code {{ background: #1e293b; padding: 0.15rem 0.35rem; border-radius: 6px; }}
  </style>
</head>
<body>
  <main>
    <div class="card">
      <h1>Path Planning Benchmark Comparison</h1>
      <p>SVG plots generated from benchmark CSV output.</p>
      <ul>
        {links}
      </ul>
    </div>
  </main>
</body>
</html>
"""


def main() -> int:
    args = build_parser().parse_args()
    input_path = Path(args.input)
    output_dir = Path(args.output)
    output_dir.mkdir(parents=True, exist_ok=True)

    rows = read_rows(input_path)
    if not rows:
        raise SystemExit(f"no rows found in {input_path}")

    planners, sizes, grouped = group_data(rows)
    if not planners or not sizes:
        raise SystemExit("benchmark CSV does not contain planner/size data")

    metrics = METRICS
    for metric_key, metric_title in metrics:
        series = metric_series(grouped, planners, sizes, metric_key)
        svg = render_svg(metric_key, metric_title, planners, sizes, series)
        (output_dir / f"{metric_key}.svg").write_text(svg, encoding="utf-8")

    index_html = render_index(output_dir, metrics)
    (output_dir / "index.html").write_text(index_html, encoding="utf-8")

    summary_lines = ["planner,size,median_total_wall_ms"]
    for planner in planners:
        wall_times = [parse_float(grouped[planner][size], "total_wall_ms") for size in sizes if size in grouped[planner]]
        if wall_times:
            summary_lines.append(f"{planner},all_sizes,{median(wall_times):.3f}")
    (output_dir / "summary.csv").write_text("\n".join(summary_lines) + "\n", encoding="utf-8")

    print(f"wrote plots to {output_dir}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
