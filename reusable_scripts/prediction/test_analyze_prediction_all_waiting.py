import importlib.util
import unittest
from pathlib import Path


PATH = Path(__file__).with_name("analyze_prediction_all_waiting.py")
SPEC = importlib.util.spec_from_file_location("all_waiting", PATH)
MODULE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(MODULE)


class WaitingClassificationTests(unittest.TestCase):
    def test_futex_wait_private_is_wait(self):
        self.assertEqual(MODULE.futex_call("NR 98 (abc, 80, 0, 0, 0, 0)"), ("abc", "80", True))

    def test_futex_wake_private_is_not_wait(self):
        self.assertEqual(MODULE.futex_call("NR 98 (abc, 81, 1, 0, 0, 0)"), ("abc", "81", False))

    def test_yield_precedes_other_runnable_classes(self):
        row = {"episode_type": "runnable", "sched_yield_associated": True,
               "ready_basis": "switchout_R", "competitor_covered_ms": 2.0}
        self.assertEqual(MODULE.primary_cause(row)[0], "sched_yield_associated_runnable")

    def test_wakeup_delay_is_distinct_from_blocked(self):
        row = {"episode_type": "runnable", "sched_yield_associated": False,
               "ready_basis": "sched_wakeup", "competitor_covered_ms": 2.0}
        self.assertEqual(MODULE.primary_cause(row)[0], "post_wakeup_runnable")


if __name__ == "__main__":
    unittest.main()
