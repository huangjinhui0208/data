#!/usr/bin/env python3
"""Independent structural and numerical QA for report_workspace outputs."""

from __future__ import annotations

import re
from pathlib import Path

import matplotlib.image as mpimg
import numpy as np
import pandas as pd


ROOT = Path(__file__).resolve().parents[1]
TABLES = ROOT / "tables"
FIGURES = ROOT / "figures"
REPORT = ROOT / "report" / "group_meeting_report.md"
VALIDATION = ROOT / "validation"


REQUIRED_TABLES = [
    "run_inventory.csv",
    "run_level_metrics.csv",
    "group_summary.csv",
    "stage_latency_summary.csv",
    "collision_case_comparison.csv",
    "evidence_matrix.csv",
    "target_identity_audit.csv",
]
REQUIRED_FIGURES = [
    "causal_chain.png",
    "group_e2e_response.png",
    "group_distance_debt.png",
    "group_braking_position.png",
    "safety_cliff.png",
    "distance_budget_decomposition.png",
    "deadline_margin.png",
    "case_1131_latency_breakdown.png",
    "case_1131_speed.png",
    "case_1131_st.png",
    "case_1131_fusion_timeline_age.png",
    "case_1643_latency_breakdown.png",
    "case_1643_speed.png",
    "case_1643_st.png",
    "case_1643_fusion_timeline_age.png",
]
REQUIRED_VALIDATION = [
    "data_inventory.md",
    "data_discrepancies.md",
    "excluded_runs.md",
    "clock_alignment.md",
]


def require(condition: bool, message: str) -> None:
    if not condition:
        raise AssertionError(message)


def main() -> None:
    missing = [name for name in REQUIRED_TABLES if not (TABLES / name).exists()]
    missing += [name for name in REQUIRED_FIGURES if not (FIGURES / name).exists()]
    missing += [name for name in REQUIRED_VALIDATION if not (VALIDATION / name).exists()]
    require(not missing, f"missing required outputs: {missing}")
    require(REPORT.exists(), "missing group meeting report")
    require((ROOT / "README.md").exists(), "missing README")

    runs = pd.read_csv(TABLES / "run_level_metrics.csv", dtype={"run_id": str})
    require(len(runs) == 12, f"expected 12 run rows, got {len(runs)}")
    require(runs.run_id.nunique() == 12, "run IDs are not unique")
    require(int(runs.included_main_analysis.sum()) == 11, "expected 11 main-analysis runs")
    excluded = runs[~runs.included_main_analysis]
    require(excluded.run_id.tolist() == ["202607271206"], "unexpected exclusion set")
    require(
        excluded.outcome_data_observed.iloc[0] == "uncertain_geometry_event_conflict",
        "1206 outcome uncertainty not preserved",
    )

    require(
        np.allclose(
            runs.D1_center_data_observed_m - 5.3074,
            runs.D1_clear_data_observed_m,
            rtol=0,
            atol=1e-9,
        ),
        "D1 clearance formula mismatch",
    )
    require(
        np.allclose(
            runs.D1_clear_data_observed_m - runs.D_delay_wall_integral_data_observed_m,
            runs.D2_clear_data_observed_m,
            rtol=0,
            atol=1e-9,
        ),
        "D2=D1-Ddelay mismatch",
    )
    require(
        np.allclose(
            (runs.t2_wall_s - runs.t1_wall_s) * 1000.0,
            runs.T_e2e_data_observed_ms,
            rtol=0,
            atol=1e-6,
        ),
        "T_e2e=t2-t1 mismatch",
    )

    collision = runs.collision_event_data_observed.astype(bool)
    collision_na = [
        "D_brake_data_observed_m",
        "M_collision_0m_data_observed_m",
        "M_safety_6m_data_observed_m",
        "T_deadline_collision_0m_data_observed_ms",
        "T_deadline_safety_6m_data_observed_ms",
    ]
    for column in collision_na:
        require(runs.loc[collision, column].isna().all(), f"collision rows must be NA: {column}")
    safe_main = runs[runs.included_main_analysis & ~collision]
    require(safe_main.D_brake_data_observed_m.notna().all(), "safe main run missing D_brake")
    require(
        np.allclose(
            safe_main.D2_clear_data_observed_m - safe_main.D_brake_data_observed_m,
            safe_main.M_collision_0m_data_observed_m,
            rtol=0,
            atol=1e-9,
        ),
        "observed collision-margin formula mismatch",
    )

    main = runs[runs.included_main_analysis]
    baseline = main[main.group_name == "baseline"]
    delay = main[main.group_name == "delay_300ms"]
    require(len(baseline) == 7 and len(delay) == 4, "main group sizes must be 7 and 4")
    require(int(baseline.collision_event_data_observed.sum()) == 0, "baseline collision count")
    require(int(delay.collision_event_data_observed.sum()) == 2, "delay collision count")
    require(np.isclose(baseline.T_e2e_data_observed_ms.median(), 300.358057, atol=1e-6), "baseline latency median")
    require(np.isclose(delay.T_e2e_data_observed_ms.median(), 749.901414, atol=1e-6), "delay latency median")
    require(np.isclose(baseline.D_delay_wall_integral_data_observed_m.median(), 5.284454, atol=1e-6), "baseline Ddelay median")
    require(np.isclose(delay.D_delay_wall_integral_data_observed_m.median(), 12.715684, atol=1e-6), "delay Ddelay median")

    model = pd.read_csv(TABLES / "counterfactual_model.csv", dtype={"run_id": str})
    require(len(model) == 2, "counterfactual table must have two rows")
    require(model.model_scope_note.str.startswith("C_MODEL_COUNTERFACTUAL").all(), "counterfactual evidence label")
    m1131 = model[model.run_id == "202607271131"].iloc[0]
    m1643 = model[model.run_id == "202607271643"].iloc[0]
    require(np.isclose(m1131.margin_to_observed_contact_restored_model_m, 5.494593, atol=1e-6), "1131 model margin")
    require(not bool(m1131.collision_model_predicted), "1131 should be predicted noncollision")
    require(np.isclose(m1643.margin_to_observed_contact_restored_model_m, -2.000002, atol=1e-6), "1643 model margin")
    require(bool(m1643.collision_model_predicted), "1643 should remain predicted collision")

    for name in REQUIRED_FIGURES:
        image = mpimg.imread(FIGURES / name)
        require(image.shape[0] >= 500 and image.shape[1] >= 700, f"figure too small: {name}")

    text = REPORT.read_text(encoding="utf-8")
    require(text.startswith("# 《结果正确但响应过晚"), "report title mismatch")
    han_count = len(re.findall("[一-龥]", text))
    require(5000 <= han_count <= 8000, f"report Chinese character count={han_count}")
    require("700 ms 硬阈值" in text, "missing restrained threshold wording")
    require("C 类模型结果" in text, "model/observed separation missing")
    links = re.findall(r"!\[[^\]]*\]\(([^)]+)\)", text)
    require(len(links) == 15, f"expected 15 report figure links, got {len(links)}")
    for link in links:
        require((REPORT.parent / link).resolve().exists(), f"broken report image link: {link}")

    output = (
        "# 最终验证\n\n"
        "- 状态：PASS\n"
        f"- run：12，主分析：11，排除：202607271206\n"
        f"- 图片：{len(REQUIRED_FIGURES)} 张，尺寸与链接全部通过\n"
        f"- 报告汉字数：{han_count}\n"
        "- D1、T_e2e、D2、完整停车余量公式：PASS\n"
        "- 碰撞 run 完整制动距离/观测余量 NA 规则：PASS\n"
        "- 反事实表与观测表分离：PASS\n"
        "- DOCX：未生成，原因是本机未安装 pandoc。\n"
    )
    (VALIDATION / "validation_summary.md").write_text(output, encoding="utf-8")
    print(output)


if __name__ == "__main__":
    main()
