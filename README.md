# Adaptive Target Tracking Testbed

A modular ROS 2 and Gazebo testbed for developing, testing, and comparing target-tracking methods for mobile robotic systems.

The platform follows a complete closed-loop pipeline:

**Measurement → Estimation → Prediction → Reference → Control → Robot Motion**

Pure numerical components are separated from ROS deployment. Supported components
can be extended through their domain interfaces; full YAML selection and additional
robot families are not yet implemented.

## Main Components

- `robot_description` — robot geometry and physical configuration
- `robot_models` — reusable robot, camera, and whole-body models
- `robot_configuration` — shared YAML mapping and inheritance loading
- `robot_interfaces` — ROS state, trajectory, command and telemetry messages
- `robot_estimation` — robot and target state estimation
- `robot_prediction` — target-motion prediction and online adaptation
- `robot_control` — tracking and robot command generation
- `robot_simulation` — target dynamics, disturbances, and Gazebo simulation
- `robot_bringup` — system configuration and launch files

The workspace is organized into separate ROS 2 packages with clear responsibilities. 

Synthetic target measurements currently belong to `robot_simulation`. Separate
perception and experiment-management packages are planned, not implemented.
Prediction deployment currently uses IMM with residual correction; the supported
robot deployment is the planar mobile manipulator. Other robot families require
matching model, state and actuator implementations.

ROS adapters live in each Python package's `nodes/` directory. Assembly lives in
`factory/`; mathematical domain directories retain their own structure. Existing
ROS executable names are unchanged. Obsolete M06 Python aliases have been removed;
use the canonical paths documented in [the current architecture guide](docs/ARCHITECTURE.md).
`python3 tools/show_architecture.py` renders the real tree in canonical semantic order.
Generic `common/` and `types.py` dumping grounds have been removed; controllers,
prediction evaluation and Gazebo SDF construction have explicit homes.
The [M06.1 hardening report](docs/m06_1_architecture_hardening.md) records verification
and intentional internal API removals. Earlier M06 documents describe historical states.

## Key Features

The current Control Compute Only milestone, verified behavior, and local
reproduction commands are documented in [docs/control_compute_only.md](docs/control_compute_only.md).

- Modular and reusable architecture
- Configurable target motion and disturbances
- Classical and adaptive prediction methods
- Online learning and residual correction
- Replaceable estimation and control algorithms
- Inherited scenario configurations with recorded validation evidence
- Explicit model, prediction, task and safety contracts

`docs/validation/` holds reproducible validation utilities and selected historical
evidence. New raw run outputs belong in gitignored `results/`; existing local evidence
is preserved. Physical joint overspeed, skid localization and aggressive tracking
remain research limitations, not hardware-safety guarantees.

## Technology

- ROS 2
- Gazebo
- Python
- YAML
- URDF / Xacro

## Goal

The goal of the project is to provide a reusable experimental platform for evaluating different approaches to robotic target tracking while keeping the core system architecture stable.

> **Keep the testbed stable while allowing the algorithms inside it to evolve.**
