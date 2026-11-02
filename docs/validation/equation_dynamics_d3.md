# D3 — Equation-driven continuous state-space dynamics

Implemented one `equation_state_space` owner type, using the existing `Dynamics`,
RK4, SimulationEngine, Experiment Author, Catalog and materializer. New models
are data: no per-model Python class or registration is needed. Existing trusted
Fight/Flight models and the acceleration adapter remain available.

| Area | Verdict | Verified scope |
| --- | --- | --- |
| Mathematical correctness | **PASS** | Supported dimensional algebra, analytical solutions, bounded numerical domains and matched Python equivalence tests. |
| Equation/configuration validation | **PASS** | Strict owner validation of names, declarations, units, shapes, bounds, dependencies and restricted expressions. |
| Owner/Catalog integration | **PASS** | Discovery, YAML authoring, immutable revisions, preflight, atomic definition replacement, frozen replay and protected comparison hashes. |
| Existing six-state deployment compatibility | **PASS** | Actual factory/TargetState/SimulationEngine numerical path and launch/system preflight; existing lifecycle/adapter regressions. No new live Gazebo run claimed. |
| Arbitrary-state numerical support | **PASS** | Scalar, six-state and full eight-state models use unchanged RK4 equations; unsupported live adapters fail before launch. Resource bounds below apply. |

These verdicts do not requalify the unresolved Recorder reliability, physical
actuation or localization limitations from earlier milestones. No dashboard
Dynamics Editor, MPC, new estimator, process engine or results store was added.

## Canonical contract and ownership

`EquationDynamics.derivative(state, control, time, context)` evaluates

`x_dot = f(t, x, u, c; theta)`.

- `state` is one real, finite, flat numerical vector. The **ordered state list**
  defines names, shapes and contiguous slices; dictionary ordering never defines
  vector layout. Scalars occupy one entry and vectors occupy their declared size.
- Each state quantity has exactly one derivative expression of the same shape
  and unit `state_unit / s`. Equations are evaluated in state-list order.
- `control` is a flat vector ordered by the optional `control` declaration list.
  Undeclared control entries fail rather than being ignored.
- `context` supplies explicitly declared named real numerical quantities. Model
  parameters are separate declarations with fixed values and optional bounds.
- `t` is the exact caller's simulation-stage time in seconds. `pi` is a named
  dimensionless constant. Neither source timestamps nor integration stage times
  are replaced, clipped or resampled.
- Named intermediates are compiled in deterministic topological order. Missing
  symbols, cycles, reserved names and excessive dependency depth fail preparation.

`equation_units.py` owns the small dimensional algebra;
`equation_expression.py` owns the mathematical parser/compiler;
`equation_model.py` owns model declarations, ordered vectors, runtime validation
and the explicit existing Cartesian deployment capability. The parser has no
knowledge of tracking, base-state, ROS or Gazebo input names.

`SimulationFactory` registers the generic type once. Its default model is a
reusable Cartesian constant-velocity definition in the authoritative
`robot_simulation/config/dynamics.yaml`. Supplying `parameters.model` replaces
that **entire** definition. It does not merge old equations, parameters, context
or intermediates into a different model. External legacy defaults files without
the new owner group retain their existing registrations and remain usable.

The simulation registry delegates coupled/atomic parameter preparation to the
owner's optional `prepare_parameters` hook. Its `apply_overrides` method keeps
ordinary owner overrides strict while allowing complete structured definitions.
The generic type advertises `atomic_parameters: [model]`; Experiment Author
honors that when replacing an inherited definition. Trusted built-ins retain
their existing numerical parameter-override behavior.

## YAML schema

The canonical document below is a complete standalone eight-state example.
The same `model` mapping goes into a target component's `parameters.model`, or
into Author's `experiment.parameters.target_dynamics.model`.

```yaml
model:
  id: translation_rotation
  type: equation_state_space
  schema_version: 1
  frame: world
  state:
    - {name: p, shape: [3], unit: m, initial: [0.0, 0.0, 1.0]}
    - {name: v, shape: [3], unit: m/s, initial: [0.2, 0.0, 0.0]}
    - {name: theta, shape: [], unit: rad, initial: 0.3}
    - {name: omega, shape: [], unit: rad/s, initial: 0.7}
  parameters:
    gamma: {value: 0.1, shape: [], unit: 1/s, minimum: 0.0}
  equations:
    p: v
    v: "[0, 0, 0]"
    theta: omega
    omega: "-gamma * omega"
```

Flattened order is `[px, py, pz, vx, vy, vz, theta, omega]`. This model is
mathematically supported and RK4-integrable but has **no live target adapter**.
The states are retained in full; yaw and angular velocity are never discarded.

Schema rules:

| Field | Contract |
| --- | --- |
| `id` | Stable ASCII identifier, up to 64 characters; frozen as part of the definition. |
| `type`, `schema_version` | Exactly `equation_state_space`, integer `1` (not boolean). |
| `frame` | Nonempty name of one fixed coordinate frame. No implicit transform. |
| `state` | Nonempty ordered list; each entry has exactly `name`, `shape`, `unit`, `initial`. |
| `parameters` | Mapping; each entry has `value`, `shape`, `unit`, optionally `minimum`, `maximum`, `exclusiveMinimum`, `exclusiveMaximum`. Bounds are finite scalar componentwise constraints. |
| `equations` | Mapping from every state name to one expression string; no extra derivatives. |
| `control` | Optional ordered list of `name`, `shape`, `unit`, `frame` declarations. |
| `context` | Optional mapping from a symbol to `shape`, `unit`, `frame`. All declared context is required at evaluation, including unused declarations. |
| `intermediates` | Optional mapping of names to expression strings; types are inferred and checked. |
| `deployment` | Optional explicit current adapter contract, shown below. |

Shapes are `[]` for a scalar or `[n]` for a flat vector. Names are unique across
state, parameters, control, context and intermediates; functions, `t` and `pi`
are reserved. Initial values and parameters must match their exact declared
shapes and be finite real numbers. Booleans, complex data and string-to-number
coercion are rejected. Unknown fields at every owner declaration level fail.

The registry exposes structural schema metadata; dynamic symbol tables,
dimensional constraints, coupled rules and expression domains are enforced by
the owning validator, not claimed as a complete general-purpose JSON Schema.
YAML uses the existing safe loader, inheritance and recursive-alias rejection.
No executable YAML constructors or model-supplied import paths are introduced.

## Operations, units and cost limits

Supported syntax: numeric literals, declared symbols, parentheses, `+ - * / **`,
unary signs, flat vector literals `[a, b, c]`, and nonnegative constant vector
indices such as `p[0]`. Addition/subtraction require identical shapes.
Multiplication/division are componentwise for equal vectors and permit scalar
broadcasting. Use `dot` for a scalar inner product, not vector `*`.

Supported calls: `sin`, `cos`, `tanh`, `exp`, `sqrt`, `norm`, `dot`, `cross`.
Dot operands have matching vector lengths; cross operands are both 3-vectors.
Norm requires a vector. Nested arrays/matrices, slicing, dynamic indices,
attributes, imports, comprehensions, lambdas, arbitrary calls, keywords and
conditionals are unsupported and rejected.

The compiler walks an allowlisted AST and builds numerical callables. It never
uses unrestricted `eval`, `exec` or Python source compilation. Constants are
folded at preparation. Expressions are not reparsed at integration steps.

Units are products/divisions of **m, s, kg, rad, 1**, with integer exponents up
to magnitude 16: e.g. `m/s^2`, `1/s`, `s^-2`, `kg*m/s^2`, `rad/s`.
No prefixes, scale conversion, degrees, offsets, arbitrary unit names or full
symbolic physical algebra are implemented. Rad is retained as an angle
dimension; sin/cos accept rad or dimensionless arguments. Tanh/exp require
dimensionless arguments. Sqrt halves dimensions; powers scale dimensions.
Unitful powers require constant half-integer exponents; dimensionless powers
allow real scalar exponents. All exponents are bounded to `[-16, 16]`, with
runtime domain checks. Pure literal zero may adopt the required physical unit;
nonzero dimensionless values cannot. `0**0` cannot exploit zero typing to become
an arbitrary physical quantity.

This is **static dimensional validation** for the supported language, plus
runtime numerical validation. It assumes declared input units are truthful; it
cannot infer the physical units of an external sensor or trusted Python field.
Vector frames must match the model frame. A supplied runtime `frame_id` must
also match; unlabelled existing runtime context relies on the deployment's
validated frame contract. No synchronization or transforms are fabricated.

Resource limits: 2,048 characters and 256 AST nodes per expression; expression
depth 32; 4,096 compiled nodes per model; intermediate dependency depth 32;
up to 128 quantities in each declaration family; vector length up to 64; up to
1,024 flattened scalar states. These are explicit preparation/evaluation cost
bounds, not a restriction to second-order or six-state mathematical systems.

Division by constant zero and constant invalid domains fail preparation.
State-dependent zero division, invalid square roots/powers, overflow and
nonfinite results fail evaluation without clipping or substituted values.
Underflow follows normal floating-point behavior. An added final finite-result
check in existing RK4 also rejects overflow of its weighted combination when
each stage itself was finite. The RK4 formula and timestep are unchanged.
No global existence/stability proof or automatic step-size selection is claimed.

## Six-state deployment and external fields

Existing `TargetState`, measurement, prediction and Gazebo interfaces remain
six-state Cartesian position/velocity. A deployable equation definition includes:

```yaml
deployment: {adapter: cartesian_target, position: p, velocity: v}
```

This requires **exactly two ordered 3-vector states**, position in m followed by
velocity in m/s, with position derivative exactly the velocity symbol. Extra
states, reordered layouts, unsupported inputs and missing adapters fail live
factory preflight. The model frame must equal the selected tracking/reference
frame; launch, bringup and SimulationNode pass that actual frame to the factory.

The live engine supplies no control vector. The equation adapter currently
supports declared live `tracking_position_world` context with shape `[3]`, unit
`m` and the model frame. Its numerical value still comes from the existing
SimulationState/TF path; no static tracking position was substituted. Other
named numerical contexts and controls work in standalone `Dynamics`/RK4, but
require a matching live binding before deployment; preflight rejects absent
bindings. In particular, the typed `physical_base` object is not silently treated
as a flat numerical vector. Existing context-dependent environmental fields can
still consume that real stamped physical-base object using their existing
freshness/frame checks.

With this Cartesian adapter, the YAML velocity derivative is **model-local
acceleration**. The configured environment's composite acceleration is added
exactly once after evaluating it, including contextual-field dispatch at the
caller’s stage time. Non-Cartesian models cannot silently receive an acceleration
field. Generic models without this adapter evaluate exactly their declared f.
Analytical constant-velocity tests disable external fields; an enabled restoring
field intentionally changes the total motion, as for existing Python dynamics.

`initial_state` returns the validated standalone default vector. In the existing
live path, the experiment's already-authoritative `target.initial_position` and
`target.initial_velocity` are **explicit initial-condition overrides**, mapped
to the same p/v slices. Both the model's standalone defaults and the actual
deployment values are frozen. Tests verify this conversion and precedence;
no second initial-posture owner or launch initialization routine was added.

Capability inspection distinguishes constructor-validated mathematics,
`Dynamics`/RK4 numerical support with runtime domains, and actual deployment:

```python
from robot_configuration.yaml_loader import load_yaml
from robot_simulation.dynamics.equation_model import EquationDynamics
from robot_simulation.integration.rk4 import RK4Integrator

model = EquationDynamics(model=load_yaml('my_model.yaml')['model'])
print(model.capabilities)
next_state = RK4Integrator().step(model, model.initial_state, [], 0.0, 0.01, {})
model.validate_deployment('world')  # rejects the 8D example above
```

## YAML-only authoring examples

This complete spring-damper Author document uses the existing saved-revision
workflow. The environment and initial live position/velocity remain inherited
from the selected base scenario; they are part of the effective frozen evidence.
For a pure spring analytical experiment, explicitly select an environment with
no acceleration fields through the existing resources mechanism.

```yaml
schema_version: 1
experiment:
  id: EXP-CUSTOM-SPRING
  name: Declarative spring damper
  base_case: EXP-NOMINAL
  components:
    target_dynamics: equation_state_space
  parameters:
    target_dynamics:
      model:
        id: spring_damper
        type: equation_state_space
        schema_version: 1
        frame: world
        state:
          - {name: p, shape: [3], unit: m, initial: [1.0, 0.0, 0.0]}
          - {name: v, shape: [3], unit: m/s, initial: [0.0, 0.0, 0.0]}
        parameters:
          k: {value: 2.0, shape: [], unit: 1/s^2, minimum: 0.0}
          c: {value: 0.5, shape: [], unit: 1/s, minimum: 0.0}
        equations:
          p: v
          v: "-k * p - c * v"
        deployment: {adapter: cartesian_target, position: p, velocity: v}
  execution: {duration: 6.0, settling_time: 3.0, seeds: [42]}
  runtime: {enable_actuation: false, gui: false}
```

A saturated Fight/Flight standalone definition is:

```yaml
model:
  id: saturated_interaction
  type: equation_state_space
  schema_version: 1
  frame: world
  state:
    - {name: p, shape: [3], unit: m, initial: [1.0, 0.0, 1.0]}
    - {name: v, shape: [3], unit: m/s, initial: [0.0, 0.0, 0.0]}
  parameters:
    a_max: {value: 0.15, shape: [], unit: m/s^2, exclusiveMinimum: 0.0}
    transition_length: {value: 0.15, shape: [], unit: m, exclusiveMinimum: 0.0}
    preferred_distance: {value: 1.0, shape: [], unit: m, exclusiveMinimum: 0.0}
    damping: {value: 0.5, shape: [], unit: 1/s, minimum: 0.0}
    epsilon: {value: 1.0e-6, shape: [], unit: m, exclusiveMinimum: 0.0}
  context:
    tracking_position_world: {shape: [3], unit: m, frame: world}
  intermediates:
    delta: "p - tracking_position_world"
    distance: "norm(delta)"
    direction: "delta / sqrt(dot(delta, delta) + epsilon**2)"
  equations:
    p: v
    v: "a_max*tanh((distance-preferred_distance)/transition_length)*direction - damping*v"
  deployment: {adapter: cartesian_target, position: p, velocity: v}
```

This preserves smooth near-zero regularization, including exact coincidence.
Constant velocity is the shipped generic owner's default model. Circular/turning
motion uses the same p/v states with `rate` in `1/s`, a dimensionless three-vector
`axis`, and velocity equation `rate*cross(axis, v)`. The retained `models.yaml`
contains complete CV, spring, turning, eight-state and saturated definitions.
New nonlinear interactions can combine the supported declared quantities; no
Python registration is needed. Source YAML inheritance can build a complete
definition; the resolved `model` supplied to the registry must be complete.

## Frozen provenance and comparison

Full effective state declarations, order, initial defaults, equations,
intermediates, parameter values/bounds/units, frame, inputs, deployment and model
identity are frozen in the ordinary target input and resolved experiment
evidence. Existing Catalog definition/revision hashes are reused. No alternative
versioning or result storage was introduced.

Tests edit the source file after save, create a second revision with changed
stiffness, and edit owner defaults after freezing. Revision 1 and its complete
effective model remain unchanged and replay with the original equations. A
specific inherited-model regression proves that replacing spring with CV removes
the inherited spring parameters instead of preserving hidden unused quantities.

The existing comparison contract contains the complete target dynamics document.
Changed equations, parameters and identity change protected original hashes and
the invariant hashes even under an authorized **predictor** comparison. No
blanket model container is masked. Existing authorized dynamics policies for
trusted registered models remain intact. Generic definitions' dynamically named
equation/parameter leaves are not automatically exposed as maskable comparison
variables; a future editor/policy extension must explicitly authorize them.
Until then, comparing different declarative definitions is conservatively
rejected rather than treated as the same protected dynamics.

## Numerical results

Measured errors retained in `results/equation_dynamics/numerical.json`:

| Case | Independent reference | Maximum absolute error |
| --- | --- | ---: |
| 6D constant velocity, 100 × 0.01 s | Analytical position and constant velocity | 2.220446e-16 |
| 6D spring, k=2 s^-2, c=0.5 s^-1, 100 × 0.01 s | Analytical underdamped oscillator | 3.397680e-10 |
| 8D translation/rotation, gamma=0.1 s^-1 | Analytical integrated angular decay, complete 8-vector retained | 5.107026e-15 (angular pair) |
| Turning velocity, rate=0.4 s^-1, 100 × 0.01 s | Analytical integrated rotating velocity | 3.950174e-13 |
| Saturated Fight/Flight derivatives | Existing trusted Python, same context and external field; four positions including coincidence and 1e-10 m | 2.775558e-17 |
| Saturated trajectories, four initial positions, 50 × 0.002 s | Existing Python under identical RK4/context/field | 0.0 |

Additional tests change spring stiffness and verify changed motion, reverse
equation-map order, check arbitrary scalar dynamics, control/time dependence,
vectors/indexing/trigonometric operations and physical contextual fields. The
actual SimulationEngine receives the same live tracking-position value and
physical-base context for both trusted and equation-backed dynamics.

No large simulation study or new Gazebo process was launched. Numerical
integration plus genuine owner/Catalog/preflight and existing adapter/lifecycle
checks exercise the changed path without re-running unrelated recorded studies.
Real-time cost and long-horizon closed-loop behavior of user equations are not
qualified by these tests.

## Verification and evidence

Generated preparation identity: `EXP-EQUATION-DYNAMICS`, revision 1, SHA-256
`a6c3e9a0b86a9d3eca60ac2f8d8c7c6b65d0568b10c2236c20a1a3b05f5c328a`.
`results/equation_dynamics/prepared/` is configuration-only preparation, not a
scientific Run and not inserted into Run/Session history.

- **78 new tests** across the three new suites: math/security/domain checks,
  deployment/legacy configuration, Author/Catalog/freezing/comparison checks.
- Final focused math, deployment, integrator and Author suites: **106 passed**,
  137.38 s (`final_focused.txt`). This includes the inherited-model correction.
- Architecture/system preflight plus mathematical/deployment suites at that
  checkpoint: **161 passed**, 34.49 s (`final_math_architecture.txt`).
- Recorder, Runner, cleanup, coverage, configuration and installed-resource
  regression selection: **92 passed**, 39.07 s (`lifecycle.txt`).
- Earlier dynamics/controller-experiment regression: **207 passed**, with one
  expected inventory assertion requiring the new registered type. It was updated
  without weakening its extension-isolation check; the corrected acceleration
  suite is included in **111 passing** focused checks (`focused.txt`). No other
  functional regression was found.
- New production mathematical modules pass their focused flake8 check
  (`lint_new.txt`). No repository-wide style-clean claim is made.
- Three affected packages build successfully in **45.9 s**; final build evidence
  is `build_final.txt`. Colcon's existing-underlay override warning is retained.
- Installed Author and saved-revision CLI preflight both return `valid:true`
  with `EquationDynamics` selected (`cli_author.txt`, `cli_preflight.txt`).
- Installed imports and numerical evaluation work from `/tmp` after sourcing
  installation, without source `PYTHONPATH` (`installed.txt`). The installed
  generic type is discoverable and a full eight-state RK4 step remains eight-state.
- A baseline SHA-256 audit covers 1,833 existing files, including **1,091 historical
  evidence files** across D2.9/D2.9a/D2.9b. All historical files remain byte-identical.
  Only the ten intended pre-existing source/config/test files changed; no prior
  documentation or evidence was overwritten. No files were deleted or relocated.

Red-test evidence is retained for the missing engine/registration, dimensional
power behavior, legacy owner configuration, inherited parameter leakage and
RK4 nonfinite final result. The final checks pass after their focused corrections.
Intermediate failing logs remain evidence and are not presented as final results.

## Reproduction

```bash
cd /home/user/ros2_ws
source install/setup.bash
export PYTHONPATH="src/robot_simulation:src/robot_experiments:$PYTHONPATH"
build/dashboard_venv/bin/python -m pytest \
  src/robot_simulation/test/test_equation_dynamics.py \
  src/robot_simulation/test/test_equation_deployment.py \
  src/robot_experiments/test/test_equation_configuration.py \
  src/robot_simulation/test/test_integrator.py -q
build/dashboard_venv/bin/python results/equation_dynamics/numerical_audit.py
ros2 run robot_experiments run author validate \
  results/equation_dynamics/experiment.yaml \
  --catalog results/equation_dynamics/catalog.yaml
ros2 run robot_experiments run --validate-only \
  --catalog results/equation_dynamics/catalog.yaml \
  --experiment EXP-EQUATION-DYNAMICS --revision 1
colcon build --packages-select robot_simulation robot_bringup robot_experiments \
  --symlink-install
```

For a new user file, initialize a separate writable Catalog with the existing
Catalog API/CLI, then use `run author validate`, `run author save`, and normal
`run --experiment ... --catalog ...`. Saved experiments retain the existing
actuation opt-in, readiness, settling, scoring and cleanup behavior. No new
execution workflow is required.

## Exact changed files

New:

1. `src/robot_simulation/robot_simulation/dynamics/equation_units.py`
2. `src/robot_simulation/robot_simulation/dynamics/equation_expression.py`
3. `src/robot_simulation/robot_simulation/dynamics/equation_model.py`
4. `src/robot_simulation/test/test_equation_dynamics.py`
5. `src/robot_simulation/test/test_equation_deployment.py`
6. `src/robot_experiments/test/test_equation_configuration.py`
7. `docs/validation/equation_dynamics_d3.md`

Modified existing files (D3 changes only; pre-existing edits preserved):

1. `src/robot_simulation/config/dynamics.yaml` — generic type's authoritative CV default.
2. `src/robot_simulation/robot_simulation/factory/registry.py` — owner preparation/override delegation.
3. `src/robot_simulation/robot_simulation/factory/simulation_factory.py` — registration and deployment preflight.
4. `src/robot_simulation/robot_simulation/integration/rk4.py` — reject nonfinite final result; same formula.
5. `src/robot_simulation/robot_simulation/nodes/simulation_node.py` — pass actual reference frame.
6. `src/robot_simulation/launch/simulation.launch.py` — pass selected reference frame during preflight.
7. `src/robot_simulation/test/test_acceleration_model.py` — include new generic type in expected inventory.
8. `src/robot_bringup/robot_bringup/factory/system_configuration.py` — frame-aware owner preflight.
9. `src/robot_experiments/robot_experiments/configuration.py` — owner-controlled structured parameter overrides.
10. `src/robot_experiments/robot_experiments/authoring.py` — honor atomic model replacement when inheriting.

Generated plans, models, configuration-only preparation, numerical audit, logs and
preservation metadata are separate under `results/equation_dynamics/`. No commit
or push. Work stops at D3.

## Limitations and Dynamics Editor handoff

The future editor can consume the stable model schema, owner validation,
`capabilities`, immutable Author/Catalog workflow and complete frozen definitions.
It must show effective live initial conditions and external fields, distinguish
standalone versus deployable models, and validate a complete replacement before
saving. Dynamic equation/parameter comparison authorization, additional typed live
context bindings, orientation/output adapters, more units and additional safe
functions need separate explicit contracts. This task supplies none implicitly.
No symbolic stability proof, automatic regularization, solver selection, adaptive
timestep or real-time guarantee is provided.
