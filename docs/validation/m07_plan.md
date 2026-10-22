# M07 implementation design and verification plan

Baseline 421b773, existing feature/m07-unified-configuration branch. Preserve all
historical data. User explicitly requests autonomous implementation and no commits;
there is no separate approval or commit gate for this work.

Use a composition manifest, not copied parameter sets or a second launch stack.
Strict schema_version=1 selects the deployed mobile manipulator, robot EKF, target
Kalman filter, IMM or CV, residual/none adaptation with matching weighted-correction/
none fusion, DLS or WLS, and an existing bringup scenario profile. Robot model names
are checked against the physical adapter. CA/CT, learned models and unsupported
state dimensions fail explicitly; no hidden zero acceleration/turn-rate adapter.

Resources are relative strings or {package, path} mappings. A generic per-document
load transform preserves declaring-file-relative resource semantics through extends.
Package share lookup is injected into pure bringup configuration assembly; no ROS
imports or robotics policies enter robot_configuration. Owner files retain all
physical and algorithm settings. References are resolved before recursive merge.

1. Add failing tests for manifest loading, package/relative resources, inheritance,
   strict schemas, component identities, invalid contracts and safety precedence.
2. Add generic resource resolution and document-transform support to existing loader.
   Share existing ROS parameter-name/type contracts from owner pure modules where
   preflight needs them; retain existing defaults, equations and node declaration.
3. Extend existing prediction/estimation factories with real named selection.
   Keep factory defaults identical for legacy callers. Expose prediction config and
   selection parameters in its node, retaining selection across clock resets.
4. Share existing control configuration assembly between node and preflight via
   existing owner factory module; retain independent safety and all equations.
5. Add pure bringup manifest schema/resource/compatibility preflight and integrate
   system_config into existing closed_loop launch. Preflight invokes actual owner
   factories, validates dimensions, frames, topics, authoritative joints and safety.
   Explicit existing launch options > manifest runtime/resources > owner defaults.
   Empty launch defaults act as sentinels; legacy behavior remains identical.
6. Add nominal, inherited CV, inherited DLS and intentionally invalid examples;
   update package resources/declarations and architecture tests only for real deps.
7. Run focused and full functional tests, all-nine builds, installed non-symlink
   verification, and bounded sequential unified/legacy/alternate Gazebo runs.
   Observe actual types, finite stamped messages, safe commands, actuator feedback
   and advancing clock. Preserve physical-speed failures without changing criteria.
8. Complete docs/m07_unified_system_configuration.md, current guide/README, exact
   modified-file inventory and evidence-based verdict. Stop before M08. No commit/push.

Review focus: inherited relative-resource origin; package traversal; bool versus int;
explicit false launch overrides; unknown owner keys; inconsistent joints/frames;
predictor reset preserving CV selection; active alternate runtime identity;
installed YAML resolution; physical safety gates remain independently enforced.

## Completion record — 2026-10-16

Steps 1–8 completed in the existing working tree. Final source verification:
986 functional tests passed; 128 focused tests passed including 52 architecture
tests; both all-nine-package builds passed. Copied-install verification from /tmp
checks 7 console entries, 68 resources, real nominal/CV/DLS construction and an
informative invalid selection. The actual invalid ROS launch starts no process.
Normal installed unified nominal, legacy and CV Gazebo deployments each produced
complete closed-loop streams and actual node identities; physical-speed failures
remain recorded, without loosening any limit. Overall verdict: CONDITIONAL.

Evidence and limitations are finalized in
../m07_unified_system_configuration.md, m07_final_verification.json,
m07_runtime_evidence.json and m07_modified_files.txt. All 803 preexisting untracked
paths remain; no commit or push; no later milestone started.
