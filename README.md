# Adaptive Target Tracking Testbed

A modular ROS 2 and Gazebo testbed for developing, testing, and comparing target-tracking methods for mobile robotic systems.

The platform follows a complete closed-loop pipeline:

**Measurement → Estimation → Prediction → Reference → Control → Robot Motion**

It is designed so that robot models, estimators, predictors, adaptation methods, controllers, target dynamics, and experiment configurations can be replaced independently without changing the overall architecture.

## Main Components

- `robot_description` — robot geometry and physical configuration
- `robot_models` — reusable robot, camera, and whole-body models
- `robot_configuration` — shared YAML mapping and inheritance loading
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
ROS executable names are unchanged, and former Python paths remain temporary
compatibility aliases. See [the M06 architecture audit](docs/m06_architecture_audit.md),
[target design](docs/m06_target_architecture.md) and
[migration results](docs/m06_structural_consolidation.md).

## Key Features

The current Control Compute Only milestone, verified behavior, and local
reproduction commands are documented in [docs/control_compute_only.md](docs/control_compute_only.md).

- Modular and reusable architecture
- Configurable target motion and disturbances
- Classical and adaptive prediction methods
- Online learning and residual correction
- Replaceable estimation and control algorithms
- Reproducible experiment configurations
- Designed for easy extension to new robots and tracking problems

## Technology

- ROS 2
- Gazebo
- Python
- YAML
- URDF / Xacro

## Goal

The goal of the project is to provide a reusable experimental platform for evaluating different approaches to robotic target tracking while keeping the core system architecture stable.

> **Keep the testbed stable while allowing the algorithms inside it to evolve.**
