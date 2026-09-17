"""Plot this run's Prediction.total callback wall time and circle strict-P99 frames."""
from __future__ import annotations

import csv
import json
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parent
sys.path.insert(0, str(ROOT / ".plot_deps"))

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt

SOURCE = ROOT / "timings/prediction_frames.csv"
OUTPUT = ROOT / "figures/prediction_total_execution_scatter"


def percentile(values, probability):
    ordered = sorted(values)
    position = (len(ordered) - 1) * probability
    lower = int(position)
    upper = min(lower + 1, len(ordered) - 1)
    fraction = position - lower
    return ordered[lower] * (1 - fraction) + ordered[upper] * fraction


with SOURCE.open(encoding="utf-8-sig", newline="") as handle:
    rows = [row for row in csv.DictReader(handle) if row["complete"] == "True"]

frames = [int(row["frame_index"]) for row in rows]
execution_ms = [float(row["execution_ms"]) for row in rows]
p99_ms = percentile(execution_ms, 0.99)
anomalies = [(frame, value) for frame, value in zip(frames, execution_ms) if value > p99_ms]

assert len(rows) == 1193
assert len(anomalies) == 12
assert {frame for frame, _ in anomalies} == {int(row["frame_index"]) for row in rows if row["execution_ms_p99_tail"] == "True"}

OUTPUT.mkdir(parents=True, exist_ok=True)
plot_rows = [
    {
        "frame_index": frame,
        "prediction_total_execution_ms": value,
        "p99_ms": p99_ms,
        "strict_p99_anomaly": value > p99_ms,
    }
    for frame, value in zip(frames, execution_ms)
]
with (OUTPUT / "prediction_total_execution_scatter_data.csv").open("w", encoding="utf-8-sig", newline="") as handle:
    writer = csv.DictWriter(handle, fieldnames=list(plot_rows[0]))
    writer.writeheader()
    writer.writerows(plot_rows)

plt.rcParams.update({
    "font.sans-serif": ["Microsoft YaHei", "SimHei", "DejaVu Sans"],
    "axes.unicode_minus": False,
    "font.size": 11,
})
fig, ax = plt.subplots(figsize=(14, 7), layout="constrained")
ax.scatter(frames, execution_ms, s=15, color="#4477AA", alpha=0.70, linewidths=0, label=f"全部完整帧（n={len(rows)}）", zorder=2)
ax.scatter(
    [frame for frame, _ in anomalies],
    [value for _, value in anomalies],
    s=88,
    facecolors="none",
    edgecolors="#D62728",
    linewidths=1.8,
    label=f"严格大于P99（n={len(anomalies)}）",
    zorder=4,
)
ax.axhline(p99_ms, color="#D62728", linestyle="--", linewidth=1.25, alpha=0.9, label=f"P99 = {p99_ms:.3f} ms", zorder=1)

for index, (frame, value) in enumerate(anomalies):
    y_offset = 8 if index % 2 == 0 else -15
    ax.annotate(f"F{frame}", (frame, value), xytext=(0, y_offset), textcoords="offset points", ha="center", va="bottom" if y_offset > 0 else "top", fontsize=8, color="#A61B1B")

ax.set_title("202609111649 Prediction.total 执行时间散点图", fontsize=16, fontweight="bold", pad=14)
ax.set_xlabel("Prediction帧索引")
ax.set_ylabel("proc_enter → proc_exit 墙钟时间（ms）")
ax.set_xlim(0, max(frames) + 1)
ax.set_ylim(0, max(execution_ms) * 1.14)
ax.grid(axis="y", color="#D9DEE3", linewidth=0.7, alpha=0.8)
ax.spines[["top", "right"]].set_visible(False)
ax.legend(loc="upper right", frameon=True, edgecolor="#D9DEE3")
ax.text(0.01, 0.98, "红色空心圆为严格大于全run P99的帧", transform=ax.transAxes, va="top", color="#5A6670", fontsize=9)

for suffix in ["png", "svg"]:
    fig.savefig(OUTPUT / f"prediction_total_execution_scatter.{suffix}", dpi=220 if suffix == "png" else None, bbox_inches="tight")
plt.close(fig)

validation = {
    "run_id": "202609111649",
    "metric": "Prediction.total callback wall time: proc_enter to proc_exit",
    "source": str(SOURCE),
    "complete_frames": len(rows),
    "percentile_method": "linear interpolation at (N-1)*0.99",
    "p99_ms": p99_ms,
    "strict_p99_count": len(anomalies),
    "strict_p99_frames": [frame for frame, _ in anomalies],
    "source_flag_agreement": True,
}
(OUTPUT / "validation.json").write_text(json.dumps(validation, ensure_ascii=False, indent=2), encoding="utf-8")
print(json.dumps(validation, ensure_ascii=False))
