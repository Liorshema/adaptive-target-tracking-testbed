
"""Tests for configuration-driven control pipeline construction."""

from copy import deepcopy

import numpy as np
import pytest

from robot_models.interfaces.base_model import BaseModel
from robot_models.interfaces.manipulator_model import ManipulatorModel
from robot_models.reference.geometry.sphere_reference import SphereReference

from robot_control.adapters.planar_kinematics_adapter import (
    PlanarKinematicsAdapter,
)
from robot_control.controllers.jacobian_controller import JacobianController
from robot_control.factory.component_registry import ComponentRegistry
from robot_control.factory.control_factory import ControlFactory
from robot_control.interfaces.control_data import (
    PredictedTargetTrajectory,
    RobotState,
)
from robot_control.pipeline.control_execution_pipeline import (
    ControlExecutionPipeline,
)
from robot_control.tracking.radial_direction_policy import (
    RadialDirectionPolicy,
)


class FactoryTestBase(BaseModel):
    """Planar base with forward and yaw velocities."""

    @property
    def velocity_dimension(self):
        """Return the number of generalized base velocities."""
        return 2

    def pose_derivative(self, pose, generalized_velocity):
        """Provide a simple pose derivative for testing."""
        return np.zeros_like(pose)

    def spatial_jacobian(self, point_base):
        """Map base velocities to task-space motion."""
        jacobian = np.zeros((6, 2))
        jacobian[0, 0] = 1.0
        jacobian[5, 1] = 1.0
        return jacobian


class FactoryTestManipulator(ManipulatorModel):
    """Configurable Cartesian test manipulator."""

    def __init__(self, joint_count=2):
        """Store the configured number of joints."""
        self._joint_count = joint_count

    @property
    def dof(self):
        """Return manipulator degrees of freedom."""
        return self._joint_count

    def forward_kinematics(self, joint_positions):
        """Compute a simple tool transform."""
        transform = np.eye(4)
        transform[0, 3] = joint_positions[0]
        transform[1, 3] = joint_positions[1]
        transform[2, 3] = 0.5
        return transform

    def jacobian(self, joint_positions):
        """Map the first two joints to x and y motion."""
        jacobian = np.zeros((6, self._joint_count))
        jacobian[0, 0] = 1.0
        jacobian[1, 1] = 1.0
        return jacobian


@pytest.fixture
def registries():
    """Register concrete implementations independently of the factory."""
    result = {
        name: ComponentRegistry()
        for name in ControlFactory.REGISTRY_NAMES
    }

    result["base"].register("test_base", FactoryTestBase)
    result["manipulator"].register(
        "test_manipulator",
        FactoryTestManipulator,
    )
    result["geometry"].register("sphere", SphereReference)
    result["policy"].register("radial", RadialDirectionPolicy)
    result["controller"].register("jacobian", JacobianController)
    result["adapter"].register(
        "planar",
        PlanarKinematicsAdapter,
    )

    return result


@pytest.fixture
def config():
    """Create a complete, externally configurable test setup."""
    return {
        "robot": {
            "base": {
                "type": "test_base",
            },
            "manipulator": {
                "type": "test_manipulator",
                "parameters": {
                    "joint_count": 2,
                },
            },
        },
        "tracking": {
            "geometry": {
                "type": "sphere",
            },
            "policy": {
                "type": "radial",
                "parameters": {
                    "radius": 1.0,
                },
            },
            "pose": {
                "forward_axis_local": [0.0, 0.0, 1.0],
                "up_axis_local": [0.0, -1.0, 0.0],
                "world_up": [0.0, 0.0, 1.0],
            },
            "lookahead_time": 0.0,
        },
        "controller": {
            "type": "jacobian",
            "task_selection": [
                [1.0, 0.0, 0.0, 0.0, 0.0, 0.0],
            ],
            "parameters": {
                "gains": [[1.0]],
                "damping": 1.0,
            },
        },
        "safety": {
            "velocity": {
                "lower": [-1.0, -1.0, -1.0, -1.0],
                "upper": [1.0, 1.0, 1.0, 1.0],
            },
        },
        "adapter": {
            "type": "planar",
            "parameters": {
                "joint_count": 2,
            },
        },
    }


@pytest.fixture
def factory(registries):
    """Provide a factory with registered test implementations."""
    return ControlFactory(registries)


def test_build_complete_pipeline(factory, config):
    """Build the complete compute-only control pipeline."""
    pipeline = factory.build(config)

    assert isinstance(pipeline, ControlExecutionPipeline)


def test_computed_tracking_command(factory, config):
    """Execute the factory-built pipeline with real tracking logic."""
    pipeline = factory.build(config)

    robot_state = RobotState(
        configuration=np.zeros(5),
        velocity=np.zeros(4),
        timestamp=10.0,
        frame_id="odom",
    )

    prediction = PredictedTargetTrajectory(
        states=np.array([
            [3.0, 0.0, 0.5, 0.0, 0.0, 0.0],
            [3.0, 0.0, 0.5, 0.0, 0.0, 0.0],
        ]),
        timestamps=np.array([10.1, 10.2]),
        frame_id="odom",
    )

    result = pipeline.compute(
        robot_state=robot_state,
        prediction=prediction,
        now=10.0,
    )

    np.testing.assert_allclose(
        result.raw_output.command,
        [2.0 / 3.0, 0.0, 2.0 / 3.0, 0.0],
        atol=1e-10,
    )

    np.testing.assert_allclose(
        result.safe_output.command,
        result.raw_output.command,
        atol=1e-10,
    )


def test_velocity_limits_dimension_mismatch(factory, config):
    """Reject safety bounds incompatible with the robot model."""
    config["safety"]["velocity"]["lower"] = [-1.0] * 3
    config["safety"]["velocity"]["upper"] = [1.0] * 3

    with pytest.raises(ValueError, match="Velocity limits dimension"):
        factory.build(config)


def test_acceleration_limits_dimension_mismatch(factory, config):
    """Reject acceleration bounds of the wrong dimension."""
    config["safety"]["acceleration"] = {
        "lower": [-2.0] * 3,
        "upper": [2.0] * 3,
    }

    with pytest.raises(ValueError, match="Acceleration limits dimension"):
        factory.build(config)


def test_adapter_joint_count_mismatch(factory, config):
    """Reject different manipulator and adapter joint counts."""
    config["adapter"]["parameters"]["joint_count"] = 3

    with pytest.raises(ValueError, match="Adapter joint count"):
        factory.build(config)


def test_joint_limits_dimension_mismatch(factory, config):
    """Reject joint constraints incompatible with manipulator DOF."""
    config["safety"]["joint"] = {
        "lower": [-1.0] * 3,
        "upper": [1.0] * 3,
        "velocity_indices": [2, 3, 4],
    }

    with pytest.raises(ValueError, match="Joint limits dimension"):
        factory.build(config)


def test_unknown_controller(factory, config):
    """Reject controller names that have not been registered."""
    config["controller"]["type"] = "unknown"

    with pytest.raises(KeyError, match="Unknown component"):
        factory.build(config)


def test_missing_configuration_section(factory, config):
    """Require all essential configuration sections."""
    del config["tracking"]

    with pytest.raises(ValueError, match="tracking"):
        factory.build(config)


def test_invalid_controller_parameters(factory, config):
    """Propagate constructor validation for invalid controller gains."""
    config["controller"]["parameters"]["gains"] = [[-1.0]]

    with pytest.raises(ValueError, match="positive definite"):
        factory.build(config)


def test_configuration_is_not_modified(factory, config):
    """Do not mutate caller-owned configuration data."""
    original = deepcopy(config)

    factory.build(config)

    assert config == original


def test_factory_can_build_independent_pipelines(factory, config):
    """Create independent pipeline instances from one configuration."""
    first = factory.build(config)
    second = factory.build(config)

    assert first is not second


def test_reject_missing_registry(registries):
    """Require every component category used by the factory."""
    del registries["controller"]

    with pytest.raises(ValueError, match="controller"):
        ControlFactory(registries)


def test_reject_invalid_registry(registries):
    """Require actual ComponentRegistry instances."""
    registries["base"] = object()

    with pytest.raises(TypeError, match="ComponentRegistry"):
        ControlFactory(registries)


def test_reject_invalid_config_type(factory):
    """Require a mapping as the top-level configuration."""
    with pytest.raises(TypeError, match="mapping"):
        factory.build(None)


def test_test_only_controller_is_discovered_and_built_without_factory_branches(registries, config):
    from robot_configuration.components import schema_for
    from robot_control.interfaces.controller import Controller
    from robot_control.interfaces.control_data import ControlMode, ControlOutput

    class ExampleController(Controller):
        def __init__(self, gain, task):
            self.gain, self.task = gain, task

        def compute(self, control_input, model, model_data):
            return ControlOutput(command=np.zeros(model.generalized_velocity_dimension),
                                 mode=ControlMode.VELOCITY)

    registry = registries['controller']
    registry.register('test_only', ExampleController, defaults={'gain': 2.0},
                      schema=schema_for({'gain': 2.0}, units={'gain': 's^-1'},
                                        constraints={'gain': {'minimum': 0.0}}),
                      dependencies=('task',), capabilities={'test_only': True, 'deployable': False})
    config['controller'] = {'type': 'test_only', 'parameters': {'gain': 3.0},
                            'task_selection': [[1., 0., 0., 0., 0., 0.]]}
    pipeline = ControlFactory(registries).build(config)
    actual = pipeline._controller._tracking._controller
    assert isinstance(actual, ExampleController) and actual.gain == 3.0
    assert registry.describe('test_only')['capabilities']['test_only'] is True
    config['controller']['parameters']['gain'] = -1.0
    with pytest.raises(ValueError, match='gain'):
        ControlFactory(registries).build(config)
