"""Small regression for the validation gate that caught stale-only D/917."""
import json
from pathlib import Path
import tempfile
import unittest

from joint_limit_recovery_analysis import control_continuity


class ControlContinuityTest(unittest.TestCase):
    def analyze(self, rows):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / 'control.jsonl'
            path.write_text(''.join(json.dumps({
                'source_time': t, 'data': {'valid': valid, 'reason': 'stale'}
            }) + '\n' for t, valid in rows))
            return control_continuity(path, 0., 3.)

    def test_valid_stream_with_short_invalid_interval(self):
        result = self.analyze([(0., True), (.5, False), (1., True),
                               (1.5, True), (2., True), (2.5, True), (3., True)])
        self.assertEqual(result['max_invalid_duration_s'], .5)
        self.assertEqual(result['max_sample_gap_s'], .5)
        self.assertEqual(result['valid_samples'], 6)

    def test_one_valid_sample_does_not_hide_stale_tail(self):
        result = self.analyze([(0., True), (.1, False), (1., False), (3., False)])
        self.assertAlmostEqual(result['max_invalid_duration_s'], 2.9)

    def test_missing_stream_and_missing_valid_tail_are_uncovered(self):
        self.assertEqual(self.analyze([])['max_invalid_duration_s'], 3.)
        self.assertEqual(self.analyze([(0., True)])['max_sample_gap_s'], 3.)


if __name__ == '__main__':
    unittest.main()
