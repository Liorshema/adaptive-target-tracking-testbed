"""Frame, causal timing, optical-axis and coverage checks for diagnostic tools."""
import unittest
import numpy as np
from tracking_loss_geometry import History, pose, task, interval_fraction, oracle_mean

class DiagnosticTest(unittest.TestCase):
    def test_causal_history_rejects_future_stale_and_clock_reset(self):
        rows=[{'source_time':1.,'data':1},{'source_time':2.,'data':2}]
        h=History(rows)
        self.assertIsNone(h.at(.9,.2));self.assertIsNone(h.at(1.3,.2))
        self.assertEqual(h.at(1.1,.2)['data'],1)
        with self.assertRaises(ValueError):History(rows[::-1])
    def test_optical_axis_and_distance_are_independent_of_arm_shaft_assumption(self):
        self.assertAlmostEqual(task(np.eye(4),[2,0,0],[1,0,0],1)[1],0)
        self.assertAlmostEqual(task(np.eye(4),[2,0,0],[0,0,1],1)[1],np.pi/2)
    def test_oracle_spawn_rotation_and_untouched_joint_state(self):
        spawn=pose([2,3,.25],[0,0,np.sin(np.pi/4),np.cos(np.pi/4)])
        physical=spawn@pose([1,0,0],[0,0,0,1]);mean=np.arange(13,dtype=float)
        out=oracle_mean(mean,physical,spawn, .2,.3)
        np.testing.assert_allclose(out[:5],[1,0,0,.2,.3],atol=1e-14)
        np.testing.assert_array_equal(out[5:],mean[5:]);np.testing.assert_array_equal(mean,np.arange(13))
    @unittest.expectedFailure
    def test_proposed_motion_consistent_forecast_correction(self):
        # Proposed-fix regression: current production intentionally remains
        # unchanged. This expected failure documents the mathematical defect.
        from robot_prediction.interfaces.prediction_data import TrajectoryPrediction
        from robot_prediction.fusion.weighted_correction import WeightedCorrection
        times=np.array([.1,.2,.3])
        states=np.column_stack([times,np.zeros((3,2)),np.ones(3),np.zeros((3,2))])
        correction=np.tile([0.,0.,0.,.2,0.,0.],(3,1))
        result=WeightedCorrection().apply(TrajectoryPrediction(states,times),correction)
        position_rate=np.diff(result.states[:,:3],axis=0)/np.diff(times)[:,None]
        velocity=(result.states[:-1,3:]+result.states[1:,3:])/2
        np.testing.assert_allclose(position_rate,velocity,atol=1e-12)

    def test_time_compliance_never_bridges_missing_evidence(self):
        result=interval_fraction([0,.1,.2,1.],[True,False,True,True],0,1,.15)
        self.assertAlmostEqual(result['observed_s'],.2)
        self.assertAlmostEqual(result['compliant_s'],.1)
        self.assertAlmostEqual(result['valid_time_percent'],50)
        self.assertAlmostEqual(result['coverage_percent'],20)

if __name__=='__main__':unittest.main()
