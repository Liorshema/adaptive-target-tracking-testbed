# S3A explicit composition implementation plan

> Execute inline with superpowers:executing-plans; independent final review.

Goal: one explicit localized manipulation-D-equivalent system using M07 schema 2,
with no legacy bringup-profile selection chain. User specification:
`60854ec8-d999-468d-a6fb-1d976df4b8be/Pasted text.txt`.

Design: version-gate a narrow compiler in bringup/factory/explicit_composition.py.
It builds the existing in-memory profile adapter and then uses the existing M07
factory/owner/cross-contract validation. The alternative of a new resolver/launch
stack is excluded by the specification; broad profile consolidation is S3B.
Schema 1's profile loading remains unchanged. Schema 2's simulation.config names
a direct simulation deployment; explicit robot/localization selectors must match
that deployment's selected canonical resources, with no silent override. Calibration
retains existing owner -> calibration -> derived wiring and explicit runtime/strategy
precedence. A legacy config:=profile override is invalid with schema 2; use a manifest
resource reference. No new restrictions on already valid schema-1 resources.

Files and steps:

- [x] Capture source/index and legacy owner/plan evidence in ignored results directory.
- [x] Add failing M07 tests for valid v2, resource origins/traversal, conflicts,
  mixed schemas, selectors, calibration precedence, invalid runtime/TF, direct and
  legacy launch action equivalence, pre-action failure and source preservation.
- [x] Add failing M08 tests for old/new seeded materialization, complete and invariant
  views, provenance differences, S1 explain and S2 assignments, invalid later variants.
- [x] Add explicit_composition.py; extend existing M07 schema/assembly conditionally.
  Keep its existing validation factories and derived wiring. Add v2-only composition
  provenance; profile_path is the manifest owning the compiled adapter (no profile file).
- [x] Add config/systems/mobile_manipulator_pose_tracking.yaml, exact flat
  config/calibration/manipulation_pose.yaml and simulation/scenarios/base_relative_manipulation.yaml.
  New experiment sensor_noise_composition.yaml inherits unchanged sensor_noise_study.yaml
  and changes only its baseline system resource. No old YAML changes.
- [x] Install nested bringup configs. Preserve actual existing simulation launch.
  M08 continues writing its existing v1 run-local profile/system adapter; verify every
  generated input and delivered node parameter. Add S1 v2 provenance only if needed.
- [x] Run red/green tests, focused suite, full functional/architecture regression;
  scope style/debt checks, normal and copied affected-package installation, installed
  old/new validate-only/explain. Preserve source fingerprints and comparison policy.
- [x] Freeze source and run one bounded existing M08 Gazebo study with new composition.
  Save activity, identities, metrics/coverage, physical traces and owned cleanup.
  Keep 150 ms observation budget; no measured-torque compliance claim.
- [x] Independently review; fix material issues. Finish closure graph/hop/resource
  counts, exact differences/files, actual verdict and reproduction/rollback commands.
  Audit old YAML, unrelated work, historical results and Git index; stop before S3B.

Review focus: inherited resource origins, calibration/topic/geometry disagreement,
manifest versus deployment duplicate selectors, raw launch versus seeded M08 inputs,
and no silent eligibility promotion after missing observations. Source owner values,
ordered lists, algorithms, physical limits, numerical tolerances and S2 entry are frozen.
