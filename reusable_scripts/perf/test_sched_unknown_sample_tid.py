"""Regression: perf can print sample_tid=-1 while switch payload is valid."""
import unittest
from pathlib import Path
from unittest.mock import patch
import generate_p4_perf_analysis as p

class UnknownSampleTidTest(unittest.TestCase):
    def test_negative_sample_tid_preserves_switch_in(self):
        lines = [
            'mainboard 42 [011] 1.000000: sched:sched_switch: prev_comm=mainboard prev_pid=42 prev_prio=120 prev_state=S ==> next_comm=free next_pid=9 next_prio=120',
            ':-1 -1 [011] 1.100000: sched:sched_switch: prev_comm=free prev_pid=9 prev_prio=120 prev_state=X ==> next_comm=mainboard next_pid=42 next_prio=120',
            'mainboard 42 [011] 1.110000: sched:sched_switch: prev_comm=mainboard prev_pid=42 prev_prio=120 prev_state=S ==> next_comm=swapper/11 next_pid=0 next_prio=120',
        ]
        events = [p.parse_sched_line(line) for line in lines]
        self.assertEqual(events[1]['sample_tid'], -1)
        with patch.object(p, 'iter_sched_range', return_value=iter(events)):
            sched = p.scan_scheduler(Path('.'), {42}, .99, 1.12)
        result = p.classify_target_window(42, .99, 1.11, sched)
        self.assertAlmostEqual(result['running_s'], .02)
        self.assertAlmostEqual(result['blocked_s'], .1)
        self.assertAlmostEqual(result['unknown_s'], 0)

if __name__ == '__main__':
    unittest.main()
