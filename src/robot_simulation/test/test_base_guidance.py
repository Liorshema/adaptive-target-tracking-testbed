"""Physical frame, source-time and continuous acceleration contracts."""
import numpy as np
import pytest


def schedule():
    from robot_simulation.disturbances.phase_schedule import PhaseSchedule
    return PhaseSchedule([
        dict(name='a', duration=4., transition=0., values=[1., 2., 0.]),
        dict(name='b', duration=6., transition=2., values=[3., 4., 2.]),
    ])


def test_schedule_source_time_clamps_and_resets():
    s = schedule()
    np.testing.assert_allclose(s.values(-1.), [1, 2, 0])
    np.testing.assert_allclose(s.values(4.), [1, 2, 0])
    np.testing.assert_allclose(s.values(5.), [2, 3, 1])
    np.testing.assert_allclose(s.values(6.), [3, 4, 2])
    np.testing.assert_allclose(s.values(100.), [3, 4, 2])
    np.testing.assert_allclose(s.values(0.), [1, 2, 0])
    for t in [4., 6.]:
        assert np.linalg.norm(s.values(t+1e-6)-s.values(t-1e-6)) < 1e-10


@pytest.mark.parametrize('change', [dict(duration=0),dict(transition=5),
    dict(values=[-1,2,0]),dict(values=[1,float('nan'),0]),dict(values=[True,2,0]),
    dict(values=['1',2,0]),dict(unused=1)])
def test_bad_phase_rejected(change):
    from robot_simulation.disturbances.phase_schedule import PhaseSchedule
    phase = dict(name='a', duration=4., transition=0., values=[1.,2.,0.])
    phase.update(change)
    with pytest.raises((ValueError, TypeError)):
        PhaseSchedule([phase])


def test_guidance_geometry_context_and_composition():
    from robot_simulation.disturbances.base_guidance import BaseRelativeGuidance
    from robot_simulation.state.physical_context import PhysicalBaseContext
    from robot_simulation.disturbances.composite_field import CompositeAccelerationField
    from robot_simulation.disturbances.vector_field import RestoringVectorField, VectorFieldParameters
    g = BaseRelativeGuidance(2., 3., .01, 'world', schedule())
    c = PhysicalBaseContext(np.array([5.,6.,7.]), 0., 'world', '/physical', .2)
    # rho=2,h=3 relative to moving physical origin. No duplicate damping.
    expected = [-4./np.sqrt(4+.01**2), 0., -3.]
    np.testing.assert_allclose(g.acceleration_with_context(np.array([7.,6.,10.]), 0., c),expected)
    np.testing.assert_allclose(g.acceleration_with_context(np.array([5.,6.,9.]), 0., c),0.)
    comp=CompositeAccelerationField([g,RestoringVectorField(VectorFieldParameters(np.zeros(3),np.eye(3)))])
    np.testing.assert_allclose(comp.acceleration_with_context(np.array([7.,6.,10.]),0.,c), np.array(expected)-[7,6,10])
    with pytest.raises(ValueError):
        comp.acceleration(np.ones(3),0.)
    with pytest.raises(ValueError):
        g.acceleration_with_context(np.ones(3),-.01,c)
    with pytest.raises(ValueError):
        g.acceleration_with_context(np.ones(3),.201,c)
    wrong=PhysicalBaseContext(np.zeros(3),0.,'odom','/physical',.2)
    with pytest.raises(ValueError):
        g.acceleration_with_context(np.ones(3),0.,wrong)


def test_tangential_sign_and_determinism():
    from robot_simulation.disturbances.base_guidance import BaseRelativeGuidance
    from robot_simulation.state.physical_context import PhysicalBaseContext
    g=BaseRelativeGuidance(2.,3.,.01,'world',schedule())
    c=PhysicalBaseContext(np.zeros(3),5.,'world','/physical',.2)
    a=g.acceleration_with_context(np.array([2.,0.,3.]),5.,c)
    assert a[1]>0
    np.testing.assert_array_equal(a,g.acceleration_with_context(np.array([2.,0.,3.]),5.,c))


def test_physical_history_never_uses_future_or_stale_and_clears_on_reset():
    from robot_simulation.state.physical_context import PhysicalPositionHistory
    h = PhysicalPositionHistory('world', '/physical', .2)
    h.add(np.ones(3), 1., 'world')
    h.add(np.ones(3)*2, 1.1, 'world')
    assert h.at(.9) is None
    np.testing.assert_array_equal(h.at(1.05).position_world, np.ones(3))
    assert h.at(1.301) is None
    h.add(np.ones(3)*9, 1.2, 'odom')
    np.testing.assert_array_equal(h.at(1.2).position_world, np.ones(3)*2)
    h.clear()
    assert h.at(1.1) is None


def test_engine_context_retained_and_missing_context_rejected():
    from robot_simulation.factory.simulation_factory import SimulationFactory
    from robot_simulation.state.simulation_state import SimulationState
    from robot_simulation.state.physical_context import PhysicalBaseContext
    from robot_models.target.target_state import TargetState
    from dataclasses import replace
    engine,dt=SimulationFactory.with_defaults().create(
        {'target':{'dynamics':{'type':'saturated_fight_flight','parameters':dict(
            a_max=.15,transition_length=.15,preferred_distance=1.,damping=.5,distance_epsilon=1e-6)}}},
        {'environment':{'acceleration_fields':[{'type':'base_relative_guidance','parameters':dict(
            radial_gain=2.,height_gain=3.,radial_epsilon=.01,reference_frame='world',
            schedule=[dict(name='a',duration=4.,transition=0.,values=[1.,2.,0.])])}]}},
        {'simulation':{'backend':'numerical','integration':{'type':'rk4','parameters':{'dt':.01}}}})
    state=SimulationState(TargetState(np.array([2.,0.,3.]),np.zeros(3)),np.zeros(3),0.)
    with pytest.raises(ValueError):
        engine.step(state,dt)
    state=replace(state,physical_base=PhysicalBaseContext(np.zeros(3),0.,'world','/physical',.2))
    result=engine.step(state,dt)
    assert result.physical_base is state.physical_base
    assert np.isfinite(result.target.position).all()


def test_old_pose_service_completion_cannot_commit_after_clock_reset():
    from types import SimpleNamespace
    from unittest.mock import Mock
    from robot_simulation.nodes.simulation_node import SimulationNode
    node=SimpleNamespace(_epoch=2,_state='current epoch',_pending=True)
    future=Mock()
    SimulationNode._on_pose_result(node,future,Mock(),epoch=1)
    assert node._state=='current epoch'
    assert node._pending is False
    future.result.assert_not_called()


def test_late_previous_epoch_physical_message_cannot_poison_new_history():
    from types import SimpleNamespace as NS
    from robot_simulation.nodes.simulation_node import SimulationNode
    from robot_simulation.state.physical_context import PhysicalPositionHistory
    history=PhysicalPositionHistory('world','/physical',.3)
    node=NS(_physical_history=history,get_clock=lambda:NS(now=lambda:NS(nanoseconds=1_000_000_000)),
            get_parameter=lambda _:NS(value=0.))
    message=NS(header=NS(stamp=NS(sec=25,nanosec=0),frame_id='world'),
               pose=NS(pose=NS(position=NS(x=99.,y=0.,z=0.))))
    SimulationNode._on_physical_base(node,message)
    assert history.at(25.) is None
    message.header.stamp.sec=1
    SimulationNode._on_physical_base(node,message)
    assert history.at(1.) is not None


def test_retained_history_can_catch_up_without_using_future_context():
    from robot_simulation.state.physical_context import PhysicalPositionHistory
    h=PhysicalPositionHistory('world','/physical',.2,history_duration=2.)
    for t in np.arange(1.,2.,.1):h.add(np.ones(3)*t,float(t),'world')
    np.testing.assert_allclose(h.at(1.05).position_world,1.)
    assert h.at(.9) is None
    assert h.at(2.2) is None


def test_stepper_refreshes_causal_context_each_substep():
    from robot_simulation.integration.time_stepper import TimeStepper
    from robot_simulation.state.simulation_state import SimulationState
    from robot_models.target.target_state import TargetState
    from dataclasses import replace
    from types import SimpleNamespace
    state=SimulationState(TargetState(np.ones(3),np.zeros(3)),np.zeros(3),0.)
    seen=[]
    engine=SimpleNamespace(step=lambda state,dt:replace(state,time=state.time+dt))
    def refresh(current):
        seen.append(current.time)
        return current
    result=TimeStepper(engine,.1,10).advance(state,.3,refresh_context=refresh)
    np.testing.assert_allclose(seen,[0.,.1,.2])
    assert result.time==.3


def test_delayed_causal_context_repairs_hole_without_replacing_duplicate():
    from robot_simulation.state.physical_context import PhysicalPositionHistory
    h = PhysicalPositionHistory('world', 'test', .02, .1)
    h.add(np.zeros(3), 1., 'world')
    h.add(np.ones(3), 1.08, 'world')
    h.add(np.full(3, 2.), 1.04, 'world')
    h.add(np.full(3, 99.), 1.04, 'world')
    np.testing.assert_equal(h.at(1.05).position_world, np.full(3, 2.))
    assert h.at(.99) is None
    for t in np.arange(1.1, 2., .01): h.add(np.zeros(3), float(t), 'world')
    assert h.at(1.) is None
    assert len(h._samples) <= 11
    h.clear()
    assert h.at(1.99) is None
