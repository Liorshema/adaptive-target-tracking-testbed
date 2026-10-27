# Adaptive Target Tracking Testbed

A modular ROS 2 and Gazebo testbed for developing, testing, and comparing target-tracking methods for mobile robotic systems.

The platform follows a complete closed-loop pipeline:

**Measurement → Estimation → Prediction → Reference → Control → Robot Motion**

Pure numerical components are separated from ROS deployment. Supported components
can be selected through a validated system manifest and extended through their
domain interfaces. The deployed robot family remains the planar mobile manipulator.

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
- `robot_experiments` — reproducible run orchestration, telemetry, metrics and benchmarks

The workspace is organized into separate ROS 2 packages with clear responsibilities. 

Synthetic target measurements currently belong to `robot_simulation`. Separate
perception remains a future package; experiment management is implemented in M08.
Prediction deployment supports IMM or constant velocity, with residual correction
or no adaptation. Other robot families require
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

M09 adds read-only Gazebo/RViz inspection around the same closed-loop system:

```bash
ros2 run robot_experiments run compose --case reactive_tracking_oscillatory --controller whole_body \
  --seed 42 --output results/visualization_inputs
ros2 launch robot_bringup visualization.launch.py \
  system_config:=$PWD/results/visualization_inputs/inputs/system.yaml gui:=true rviz:=true
```

Actuation remains disabled unless `enable_actuation:=true` is explicit. Use
`gui:=false rviz:=false` for headless marker/path publication. The display shows
physical and estimated histories, the current future forecast, camera optical
direction, target line of sight, configured radius and measured joint warnings.
See the [M09 guide and runtime evidence](docs/m09_robot_realism_and_visualization.md)
for YAML layers, frame conventions, GUI limitations and unchanged physics.

M08 runs experiments through the existing M07 system manifests:

```bash
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install --packages-up-to robot_experiments
source install/setup.bash
ros2 run robot_experiments run --experiment EXP-PREDICTION-COMPOSITION
```

The example explicitly enables actuation and compares real IMM/CV variants with
matched sensor seeds. Versioned results include raw timestamped telemetry,
physical tracking/prediction metrics and independent safety status. Exit code 3
means execution completed with failed or unknown acceptance; physical overspeed
remains visible. See the [M08 guide](docs/m08_experiments_and_benchmarking.md)
for compute-only operation, YAML, result contracts and reproduction.

The [experiment-management guide](docs/experiment_management.md) documents the
revisioned catalog, immutable run inputs, headless create/revise/reset/history
operations and independently owned background workers. Use
`ros2 run robot_experiments run catalog list` to discover definitions. Study and scenario selections are consolidated into `robot_experiments/config/experiments.yaml`;
old M08 entry names remain explicit imports. Revision 3 removes 42 redundant YAMLs;
see [composition workflows and direct-launch migration](docs/experiment_composition.md). Algorithm defaults now have one YAML
per package; experiment selection lives in `robot_experiments/config/`. Bringup
assembles validated selections and simulation owns reusable physics and sensing.

The [component configuration guide](docs/component_configuration.md) describes
canonical model-family YAML, owner registries, strict schemas and extension contracts.
Inspect IDs, defaults, units and deployment requirements without starting ROS nodes:

```bash
ros2 run robot_experiments run components list
ros2 run robot_experiments run components show predictor imm
```

Continuous simulation uses experiment-owned explicit composition:

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 launch robot_experiments tracking.launch.py \
  case:=reactive_tracking_oscillatory controller:=whole_body gui:=false
# Existing reactive case and WLS allocation, with physical commands enabled:
ros2 launch robot_experiments tracking.launch.py \
  case:=reactive_tracking_oscillatory controller:=whole_body \
  enable_actuation:=true gui:=false
```

Actuation is disabled by default. Select CV/IMM or DLS/WLS in a catalog variant
using the shared composition and selection deltas. Direct launch cases
previously selected by bringup `config:=config/<case>.yaml` now use experiment
`case:=<case>`; WLS allocation names use `controller:=<name>` instead of the removed
strategy files. Generic bringup accepts an explicit manifest path and contains no
experiment default. See the [ownership and migration guide](docs/experiment_management.md)
for parameter groups, supported selections, compatibility boundaries and evidence.
The earlier M07 report records its original milestone rather than current paths.

Use the [sensor navigation guide](docs/architecture/sensors.md) to find sensor
owners, select publisher/fusion settings and inspect the effective M07 inventory.

The current Control Compute Only milestone, verified behavior, and local
reproduction commands are documented in [docs/control_compute_only.md](docs/control_compute_only.md).

- Modular and reusable architecture
- Configurable target motion and disturbances
- Classical and adaptive prediction methods
- Online learning and residual correction
- Replaceable estimation and control algorithms
- Catalog-owned scenario recipes with immutable validation evidence
- Explicit model, prediction, task and safety contracts

`docs/validation/` holds permanent engineering reports. Reusable probes, analyses
and their fixtures live in [`tools/validation/`](tools/validation/README.md).
Generated captures and source snapshots live in ignored `results/validation/`;
experiment studies use `results/<study>/`. Large historical captures and copied
workspaces are preserved in verified `results/archives/*.tar.gz` archives with
member checksums and exact restore commands in adjacent JSON manifests. For example:

```bash
tar -xzf results/archives/reactive_challenge_study.tar.gz -C .
```

Restore from the workspace root before using an archived study's original paths.
Archives and local evidence do not ship with a source clone. Captured logs, source
snapshots and metadata retain their original contents and paths as historical
records; current reproduction commands use the new locations. Keep these artifacts
with the scientific results when transferring a study.

The [repository cleanup report](docs/repository_cleanup.md) records the exact
relocations, configuration aliases removed, storage savings and verification.
R1C benchmarking remains **CONDITIONAL**; research results do not certify hardware safety.

## Technology

- ROS 2
- Gazebo
- Python
- YAML
- URDF / Xacro

## Goal

The goal of the project is to provide a reusable experimental platform for evaluating different approaches to robotic target tracking while keeping the core system architecture stable.

> **Keep the testbed stable while allowing the algorithms inside it to evolve.**
