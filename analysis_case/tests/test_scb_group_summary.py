from __future__ import annotations

import sys
import tempfile
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from scb_group_summary import summarize


class ScbGroupSummaryTest(unittest.TestCase):
    def test_ten_empty_case_directories_per_group_do_not_pass(self) -> None:
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            for group in ("baseline", "e2e200", "e2e300"):
                for index in range(10):
                    case = root / f"{group}_{index:02d}"
                    case.mkdir()
                    (case / "scb_experiment.yaml").write_text(
                        f"group: {group}\nrepeat_index: {index + 1}\n",
                        encoding="utf-8",
                    )
            result = summarize(root, root / "summary")

        self.assertEqual(result["status"], "INSUFFICIENT_ANALYZABLE_RUNS")
        for group in ("baseline", "e2e200", "e2e300"):
            self.assertEqual(result["groups"][group]["run_count"], 10)
            self.assertEqual(result["groups"][group]["analyzable_run_count"], 0)
            self.assertFalse(result["groups"][group]["ready_for_comparison"])


if __name__ == "__main__":
    unittest.main()
