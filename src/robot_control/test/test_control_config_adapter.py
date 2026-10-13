
"""Tests for ROS parameters to ControlFactory configuration mapping."""

from copy import deepcopy

import numpy as np
import pytest

from robot_control.adapters.control_config_adapter import (
    ControlConfigAdapter,
)


@pytest.fixture
def ros_parameters():
    """Provide representative ROS control parameters."""
    return {
        "joint_count": 2,
        "base_model_type": "test_base",
        "manipulator_model_type": "test_manipulator",
        "kinematics_adapter_type": "planar",
        "controller_type": "jacobian",
        "controller_damping": 1.0,
        "controller_task_axes": [0, 1, 2],
        "controller_task_gains": [1.0, 2.0, 3.0],
        "reference_geometry_type": "sphere",
        "reference_policy_type": "radial",
        "tracking_radius": 1.5,
        "tracking_lookahead_time": 0.1,
        "tracking_forward_axis": [0.0, 0.0, 1.0],
        "tracking_up_axis": [0.0, -1.0, 0.0],
        "world_up_axis": [0.0, 0.0, 1.0],
    }


@pytest.fixture
def robot_configuration():
    """Provide robot specifications independent of ROS."""
    return {
        "base": {
            "type": "test_base",
        },
        "manipulator": {
            "type": "test_manipulator",
            "parameters": {
                "joint_count": 2,
            },
        },
    }


@pytest.fixture
def safety_configuration():
    """Provide illustrative four-dimensional command limits."""
    return {
        "velocity": {
            "lower": [-1.0, -1.0, -1.0, -1.0],
            "upper": [1.0, 1.0, 1.0, 1.0],
        },
    }


def convert(ros_parameters, robot_configuration, safety_configuration):
    """Invoke the adapter using the standard test configuration."""
    return ControlConfigAdapter.convert(
        ros_parameters=ros_parameters,
        robot_configuration=robot_configuration,
        safety_configuration=safety_configuration,
    )


def test_complete_configuration(
    ros_parameters,
    robot_configuration,
    safety_configuration,
):
    """Build all configuration sections required by ControlFactory."""
    result = convert(
        ros_parameters,
        robot_configuration,
        safety_configuration,
    )

    assert set(result) == {
        "robot",
        "tracking",
        "controller",
        "safety",
        "adapter",
    }

    assert result["robot"] == robot_configuration
    assert result["safety"] == safety_configuration
    assert result["controller"]["type"] == "jacobian"
    assert result["adapter"]["type"] == "planar"
    assert result["adapter"]["parameters"]["joint_count"] == 2


def test_task_selection_matrix(
    ros_parameters,
    robot_configuration,
    safety_configuration,
):
    """Generate a task-selection matrix from configured axes."""
    ros_parameters["controller_task_axes"] = [0, 2, 5]

    result = convert(
        ros_parameters,
        robot_configuration,
        safety_configuration,
    )

    expected = np.eye(6)[[0, 2, 5]]

    np.testing.assert_array_equal(
        result["controller"]["task_selection"],
        expected,
    )


def test_diagonal_gain_matrix(
    ros_parameters,
    robot_configuration,
    safety_configuration,
):
    """Convert per-axis gains into a positive diagonal matrix."""
    result = convert(
        ros_parameters,
        robot_configuration,
        safety_configuration,
    )

    np.testing.assert_allclose(
        result["controller"]["parameters"]["gains"],
        np.diag([1.0, 2.0, 3.0]),
    )


def test_tracking_frame_axis_mapping(
    ros_parameters,
    robot_configuration,
    safety_configuration,
):
    """Map tracking-frame conventions onto generic pose parameters."""
    result = convert(
        ros_parameters,
        robot_configuration,
        safety_configuration,
    )

    pose = result["tracking"]["pose"]

    assert pose["forward_axis_local"] == [0.0, 0.0, 1.0]
    assert pose["up_axis_local"] == [0.0, -1.0, 0.0]
    assert pose["world_up"] == [0.0, 0.0, 1.0]


def test_tracking_configuration(
    ros_parameters,
    robot_configuration,
    safety_configuration,
):
    """Preserve geometry, policy, radius and prediction lookahead."""
    result = convert(
        ros_parameters,
        robot_configuration,
        safety_configuration,
    )

    tracking = result["tracking"]

    assert tracking["geometry"]["type"] == "sphere"
    assert tracking["policy"]["type"] == "radial"
    assert tracking["policy"]["parameters"]["radius"] == 1.5
    assert tracking["lookahead_time"] == 0.1


def test_input_configurations_are_not_modified(
    ros_parameters,
    robot_configuration,
    safety_configuration,
):
    """Leave all caller-owned configuration objects unchanged."""
    originals = deepcopy((
        ros_parameters,
        robot_configuration,
        safety_configuration,
    ))

    convert(
        ros_parameters,
        robot_configuration,
        safety_configuration,
    )

    assert ros_parameters == originals[0]
    assert robot_configuration == originals[1]
    assert safety_configuration == originals[2]


def test_output_does_not_share_nested_robot_or_safety_configuration(
    ros_parameters,
    robot_configuration,
    safety_configuration,
):
    """Deep-copy model and safety configuration structures."""
    result = convert(
        ros_parameters,
        robot_configuration,
        safety_configuration,
    )

    result["robot"]["manipulator"]["parameters"]["joint_count"] = 99
    result["safety"]["velocity"]["lower"][0] = -99.0

    assert robot_configuration["manipulator"]["parameters"][
        "joint_count"
    ] == 2
    assert safety_configuration["velocity"]["lower"][0] == -1.0


@pytest.mark.parametrize(
    "missing",
    [
        "joint_count",
        "controller_task_axes",
        "controller_task_gains",
        "controller_damping",
        "tracking_radius",
        "tracking_lookahead_time",
        "tracking_forward_axis",
        "tracking_up_axis",
        "world_up_axis",
    ],
)
def test_missing_required_ros_parameter(
    missing,
    ros_parameters,
    robot_configuration,
    safety_configuration,
):
    """Report missing mandatory control parameters."""
    del ros_parameters[missing]

    with pytest.raises(ValueError, match="Missing ROS control parameter"):
        convert(
            ros_parameters,
            robot_configuration,
            safety_configuration,
        )


@pytest.mark.parametrize(
    "axes",
    [
        [],
        [0, 0],
        [-1],
        [6],
        [0, 2.5],
        ["x"],
    ],
)
def test_invalid_task_axes(
    axes,
    ros_parameters,
    robot_configuration,
    safety_configuration,
):
    """Reject duplicate, out-of-range and noninteger task axes."""
    ros_parameters["controller_task_axes"] = axes

    with pytest.raises(ValueError, match="controller_task_axes"):
        convert(
            ros_parameters,
            robot_configuration,
            safety_configuration,
        )


@pytest.mark.parametrize(
    "gains",
    [
        [1.0],
        [1.0, 0.0, 2.0],
        [1.0, -1.0, 2.0],
        [1.0, np.nan, 2.0],
    ],
)
def test_invalid_task_gains(
    gains,
    ros_parameters,
    robot_configuration,
    safety_configuration,
):
    """Require positive finite gains matching the selected task."""
    ros_parameters["controller_task_gains"] = gains

    with pytest.raises(ValueError, match="controller_task_gains"):
        convert(
            ros_parameters,
            robot_configuration,
            safety_configuration,
        )


@pytest.mark.parametrize(
    "parameter, value",
    [
        ("controller_damping", 0.0),
        ("controller_damping", -1.0),
        ("tracking_radius", 0.0),
        ("tracking_radius", np.nan),
        ("tracking_lookahead_time", -0.1),
    ],
)
def test_invalid_scalar_parameters(
    parameter,
    value,
    ros_parameters,
    robot_configuration,
    safety_configuration,
):
    """Reject invalid controller and tracking scalar parameters."""
    ros_parameters[parameter] = value

    with pytest.raises(ValueError, match=parameter):
        convert(
            ros_parameters,
            robot_configuration,
            safety_configuration,
        )


def test_robot_model_type_mismatch(
    ros_parameters,
    robot_configuration,
    safety_configuration,
):
    """Detect inconsistent model selection across sources."""
    robot_configuration["base"]["type"] = "other_base"

    with pytest.raises(ValueError, match="base_model_type"):
        convert(
            ros_parameters,
            robot_configuration,
            safety_configuration,
        )


def test_manipulator_joint_count_mismatch(
    ros_parameters,
    robot_configuration,
    safety_configuration,
):
    """Reject conflicting manipulator joint counts."""
    robot_configuration["manipulator"]["parameters"][
        "joint_count"
    ] = 3

    with pytest.raises(ValueError, match="joint_count"):
        convert(
            ros_parameters,
            robot_configuration,
            safety_configuration,
        )


def test_missing_velocity_safety_configuration(
    ros_parameters,
    robot_configuration,
):
    """Prevent construction without velocity safety bounds."""
    with pytest.raises(ValueError, match="velocity"):
        convert(
            ros_parameters,
            robot_configuration,
            safety_configuration={},
        )


@pytest.mark.parametrize(
    "source",
    ["ros", "robot", "safety"],
)
def test_invalid_configuration_source(
    source,
    ros_parameters,
    robot_configuration,
    safety_configuration,
):
    """Reject invalid top-level source configuration types."""
    sources = {
        "ros": ros_parameters,
        "robot": robot_configuration,
        "safety": safety_configuration,
    }

    sources[source] = None

    with pytest.raises(TypeError, match="mapping"):
        convert(
            sources["ros"],
            sources["robot"],
            sources["safety"],
        )
