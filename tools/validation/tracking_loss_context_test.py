"""Reproduce causal context/cursor failure without altering time or freshness policy."""
from dataclasses import dataclass,replace
import unittest
import numpy as np
from robot_simulation.state.physical_context import PhysicalPositionHistory
from robot_simulation.integration.time_stepper import TimeStepper

@dataclass(frozen=True)
class Cursor:
    time: float
class Engine:
    def step(self,state,dt):return replace(state,time=state.time+dt)
class ContextTest(unittest.TestCase):
    def test_partial_candidate_is_not_committed_after_missing_frontier_context(self):
        history=PhysicalPositionHistory('world','diagnostic',.02,.1)
        history.add(np.zeros(3),0.,'world');stepper=TimeStepper(Engine(),.01,10);original=Cursor(0.)
        def refresh(state):
            if history.at(state.time) is None:raise ValueError('missing causal context')
            return state
        with self.assertRaisesRegex(ValueError,'missing causal'):
            stepper.advance(original,.06,refresh_context=refresh)
        self.assertEqual(original.time,0.)
        # New data are fresh at the live clock but cannot repair an evicted past
        # cursor. This is the documented absorbing state, not permission to use
        # a future sample or to relabel its timestamp.
        history.add(np.ones(3),1.,'world')
        self.assertIsNone(history.at(original.time))
        with self.assertRaises(ValueError):stepper.advance(original,.06,refresh_context=refresh)
    def test_history_clear_refuses_previous_epoch_and_future_context(self):
        h=PhysicalPositionHistory('world','diagnostic',.02,.1);h.add(np.zeros(3),5.,'world');h.clear()
        self.assertIsNone(h.at(0.));h.add(np.ones(3),.01,'world')
        self.assertIsNone(h.at(0.));self.assertIsNotNone(h.at(.02))
if __name__=='__main__':unittest.main()
