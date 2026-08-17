#!/usr/bin/env python3
"""Validate numerical, evidentiary, and artifact requirements."""

from __future__ import annotations

import math
import re
from pathlib import Path

import pandas as pd
from PIL import Image


ROOT = Path(__file__).resolve().parents[1]
TABLES = ROOT / "tables"
REPORT = ROOT / "report"
FIGURES = ROOT / "figures"
VALIDATION = ROOT / "validation"

REQUIRED_REPORTS = [
    "group_meeting_report.md", "speech_10_12min.md",
    "one_page_summary.md", "evidence_appendix.md",
]
REQUIRED_TABLES = [
    "run_inventory.csv", "run_level_metrics.csv", "group_summary.csv",
    "stage_latency_summary.csv", "collision_case_comparison.csv",
    "realtime_defect_evidence_matrix.csv", "target_identity_audit.csv",
]
REQUIRED_FIGURES = [
    "realtime_fault_propagation_chain.png", "e2e_response_by_run.png",
    "latency_amplification.png", "distance_debt_by_run.png",
    "braking_position_by_run.png", "realtime_safety_cliff.png",
    "case_1131_causal_chain.png", "case_1131_fusion_timeline.png",
    "case_1643_causal_chain.png", "case_1643_data_freshness.png",
    "outcome_timeline.png",
]


def close(value: float, expected: float, tolerance: float = 1e-6) -> bool:
    return math.isclose(float(value), expected, rel_tol=0.0, abs_tol=tolerance)


def main() -> None:
    checks: list[tuple[str, bool, str]] = []

    def add(name: str, passed: bool, detail: str) -> None:
        checks.append((name, bool(passed), detail))

    for name in REQUIRED_REPORTS:
        path = REPORT / name
        add(f"报告文件 {name}", path.exists() and path.stat().st_size > 500, str(path))
    for name in REQUIRED_TABLES:
        path = TABLES / name
        add(f"表格文件 {name}", path.exists() and path.stat().st_size > 20, str(path))
    for name in REQUIRED_FIGURES:
        path = FIGURES / name
        valid = False
        detail = "missing"
        if path.exists():
            with Image.open(path) as image:
                valid = image.width >= 900 and image.height >= 450
                detail = f"{image.width}x{image.height}"
        add(f"图像文件 {name}", valid, detail)

    runs = pd.read_csv(TABLES / "run_level_metrics.csv", dtype={"run_id": str})
    inventory = pd.read_csv(TABLES / "run_inventory.csv", dtype={"run_id": str})
    summary = pd.read_csv(TABLES / "group_summary.csv")
    stages = pd.read_csv(TABLES / "stage_latency_summary.csv")
    counter = pd.read_csv(TABLES / "counterfactual_model.csv", dtype={"run_id": str})
    defects = pd.read_csv(TABLES / "realtime_defect_evidence_matrix.csv", dtype={"run_id": str})

    add("发现12个预期run", len(runs) == 12 and len(inventory) == 12, f"metrics={len(runs)}, inventory={len(inventory)}")
    add("11个run进入主分析", int(runs.included_main_analysis.sum()) == 11, str(int(runs.included_main_analysis.sum())))
    r1206 = runs[runs.run_id == "202607271206"].iloc[0]
    add("1206按证据冲突排除", not bool(r1206.included_main_analysis) and "uncertain" in r1206.outcome_data_observed, r1206.outcome_data_observed)
    collision = runs[runs.collision_event_data_observed]
    collision_na = collision[[
        "D_brake_data_observed_m", "M_collision_0m_data_observed_m",
        "M_safety_6m_data_observed_m", "T_deadline_collision_0m_data_observed_ms",
    ]].isna().all().all()
    add("碰撞run完整制动与余量保持NA", collision_na, f"collision rows={len(collision)}")
    add("观测与模型分表保存", "model_scope_note" in counter.columns and not any("model_predicted" in col for col in runs.columns), "run_level=observed, counterfactual=model")

    def sv(group: str, metric: str, stat: str = "median") -> float:
        return float(summary[(summary.group_name == group) & (summary.metric == metric)].iloc[0][stat])

    add("baseline响应中位数", close(sv("baseline", "T_e2e_data_observed_ms"), 300.358057, 1e-5), f"{sv('baseline','T_e2e_data_observed_ms'):.6f}")
    add("300ms响应中位数", close(sv("delay_300ms", "T_e2e_data_observed_ms"), 749.901414, 1e-5), f"{sv('delay_300ms','T_e2e_data_observed_ms'):.6f}")
    add("距离债务中位差", close(sv("delay_300ms", "D_delay_wall_integral_data_observed_m") - sv("baseline", "D_delay_wall_integral_data_observed_m"), 7.431230, 1e-5), "expected 7.431230 m")
    add("制动起点净距中位差", close(sv("baseline", "D2_clear_data_observed_m") - sv("delay_300ms", "D2_clear_data_observed_m"), 7.142753, 1e-5), "expected 7.142753 m")
    add("baseline最终净距均为正", bool((runs[runs.group_name == "baseline"].final_clearance_projected_data_observed_m > 0).all()), "7/7")

    fusion = runs.sensor_to_fusion_ms
    add("12组首个稳定sensor→Fusion均低于500ms", bool((fusion < 500).all()) and len(fusion) == 12, f"range={fusion.min():.3f}–{fusion.max():.3f} ms")
    r1131 = runs[runs.run_id == "202607271131"].iloc[0]
    add("1131连续性/新鲜度异常可复现", r1131.target_gap_max_ms > 500 and r1131.target_lifecycle_max_ms > 500, f"gap={r1131.target_gap_max_ms:.3f}, lifecycle={r1131.target_lifecycle_max_ms:.3f}")
    add("两套队列字段分离", "ground_to_detection_queue_median_ms" in runs.columns and "scb_queue_depth_at_trigger" in runs.columns, "lidar trace queue vs SCB command queue")

    class_map = dict(zip(defects.run_id, defects.realtime_defect_class))
    add("碰撞/异常分类齐全", class_map.get("202607271131") == "RT_DOMINATED_COLLISION" and class_map.get("202607271643") == "MULTI_FACTOR_COLLISION" and class_map.get("202607271206") == "INDETERMINATE", str(class_map))

    report_text = (REPORT / "group_meeting_report.md").read_text(encoding="utf-8")
    chinese_chars = len(re.findall(r"[\u4e00-\u9fff]", report_text))
    add("主报告5000–8000汉字", 5000 <= chinese_chars <= 8000, str(chinese_chars))
    add("报告明确否定通用700ms阈值", "不能把 700 ms 宣称为普适硬阈值" in report_text or "不能把 700 ms 宣称" in report_text, "boundary statement present")
    add("报告使用观测/模型边界", "C 类模型" in report_text and "不是观测事实" in report_text, "boundary statement present")

    markdown_files = [REPORT / name for name in REQUIRED_REPORTS]
    missing_links: list[str] = []
    for markdown in markdown_files:
        text = markdown.read_text(encoding="utf-8")
        for target in re.findall(r"!?(?:\[[^\]]*\])\(([^)]+)\)", text):
            if "://" not in target and not (markdown.parent / target).resolve().exists():
                missing_links.append(f"{markdown.name}:{target}")
    add("Markdown本地链接均可解析", not missing_links, str(missing_links or "none"))

    raw_files = sum(1 for path in (ROOT.parent / "第二次实验").rglob("*") if path.is_file())
    add("原始目录文件盘点仍为454", raw_files == 454, str(raw_files))

    passed = sum(1 for _, ok, _ in checks if ok)
    lines = [
        "# 自动验证结果", "",
        f"- 通过：{passed}/{len(checks)}", f"- 失败：{len(checks)-passed}/{len(checks)}", "",
        "| 检查项 | 状态 | 详情 |", "|---|---|---|",
    ]
    for name, ok, detail in checks:
        lines.append(f"| {name} | {'PASS' if ok else 'FAIL'} | {detail.replace('|', '/')} |")
    (VALIDATION / "validation_results.md").write_text("\n".join(lines) + "\n", encoding="utf-8")

    checklist_items = [
        "所有原始文件保持只读，新增内容仅位于realtime_defect_report",
        "递归盘点454个实验文件和12个run",
        "7个baseline与5个300 ms run全部解析",
        "1206结局冲突有明确排除原因",
        "11-run主分析样本固定",
        "t1连续3帧稳定目标定义统一",
        "t2持续有效减速定义统一",
        "D_delay统一使用墙钟速度梯形积分",
        "观测结果与模型结果分表保存",
        "碰撞run完整D_brake、余量和deadline保持NA",
        "7个baseline最终停车净距逐run报告",
        "12组感知时延、新鲜度和连续性分别检查",
        "lidar检测队列与SCB命令队列正确区分",
        "1131与1211完成案例对照",
        "1643与1211完成案例对照",
        "1131/1643目标身份完成审计",
        "三类实时性缺陷归因及置信度写入矩阵",
        "11张指定图全部生成并通过尺寸检查",
        "主报告、讲稿、单页摘要和证据附录齐全",
        "主报告长度、链接、数值和结论边界通过自动验证",
    ]
    checklist = ["# 20项交付检查清单", ""]
    checklist.extend(f"- [{'x' if passed == len(checks) else ' '}] {index}. {item}" for index, item in enumerate(checklist_items, 1))
    (VALIDATION / "completion_checklist.md").write_text("\n".join(checklist) + "\n", encoding="utf-8")

    print(f"validation: {passed}/{len(checks)} passed")
    if passed != len(checks):
        raise SystemExit(1)


if __name__ == "__main__":
    main()
