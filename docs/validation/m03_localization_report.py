"""Populate the final report from retained completed-run measurements only."""
import argparse
import json
from pathlib import Path


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--analysis', type=Path, nargs='+', required=True)
    p.add_argument('--report', type=Path, default=Path(__file__).with_name('m03_localization_improvement.md'))
    a = p.parse_args()
    extra = json.loads(a.report.with_name('m03_localization_extra_checks.json').read_text())
    bootstrap_ok = extra['degraded'].get('robust_initialization_passed', False)
    cases = {}
    for path in a.analysis:
        cases.update({k:v for k,v in json.loads(path.read_text()).items()
                      if 'excluded_incomplete_attempt' not in v})
    assert set(cases) == {f'{mode}_D{seed}' for mode in ['original','localized','disabled'] for seed in [42,917]}
    def average(case, key):
        return case['physical_0_90'][key]['mean']
    localization_ok = all(cases[f'localized_D{s}']['localization_20_90']['xy_error_m']['mean'] <= .2
        and cases[f'localized_D{s}']['localization_20_90']['absolute_yaw_deg']['mean'] <= 10 for s in [42,917])
    improvements = all(average(cases[f'localized_D{s}'], key) < average(cases[f'original_D{s}'], key)
        for s in [42,917] for key in ['physical_distance_error_m','physical_pointing_deg'])
    compliance = all(cases[f'localized_D{s}']['physical_0_90']['compliance']['valid_time_percent'] >
                     cases[f'original_D{s}']['physical_0_90']['compliance']['valid_time_percent'] for s in [42,917])
    physical = all(c['result']['physical']['accepted'] for c in cases.values())
    validity = all(cases[f'localized_D{s}']['control_0_90']['valid_sample_fraction'] >= .98
                   and cases[f'localized_D{s}']['control_0_90']['max_invalid_duration_s'] < 1 for s in [42,917])
    gates = dict(localization_mean_targets=localization_ok, both_tracking_maes_improved=improvements,
                 compliance_improved=compliance, full_control_validity=validity,
                 native_position_speed=physical, robust_initialization=bootstrap_ok)
    measurements = {'gates': gates, 'cases': cases}
    a.report.with_name('m03_localization_measurements.json').write_text(json.dumps(measurements,indent=2)+'\n')
    lines = ['## Final measured comparison', '',
        'Localization is source-aligned over 20–90 s; physical tracking and compliance use 0–90 s. Each completed launch ran 90 simulation seconds after readiness, covering the requested source interval. Compliance is time-weighted among observed eligible intervals; its observation coverage is shown explicitly. Control percentages are sampled validity, with no imputation of missing telemetry.', '',
        '| Seed / policy | XY MAE m | Yaw MAE deg | Physical distance MAE / max m | Physical pointing MAE / max deg | Compliance % (coverage %) | Valid control % | RTF |',
        '|---|---:|---:|---:|---:|---:|---:|---:|']
    for seed in [42,917]:
        for mode in ['original','localized','disabled']:
            c=cases[f'{mode}_D{seed}']; loc=c['localization_20_90']; t=c['physical_0_90'];m=c['result']['tracking']['metrics']
            lines.append(f"| {seed} / {mode} | {loc['xy_error_m']['mean']:.4f} | {loc['absolute_yaw_deg']['mean']:.3f} | "
                f"{t['physical_distance_error_m']['mean']:.4f} / {t['physical_distance_error_m']['max']:.4f} | "
                f"{t['physical_pointing_deg']['mean']:.2f} / {t['physical_pointing_deg']['max']:.2f} | "
                f"{t['compliance']['valid_time_percent']:.2f} ({t['compliance']['coverage_percent']:.2f}) | "
                f"{100*c['control_0_90']['valid_sample_fraction']:.2f} | {m['realtime_factor']['mean']:.3f} |")
    lines += ['', 'The original and sensor-disabled policies remain unobservable globally. The ablation retains the same slip Q, wheel R and holdback as the localized policy: any remaining drift is exposed rather than corrected through truth. Controller, target, predictor, adaptation and physical contracts are identical across policies. Reactive trajectories differ.', '',
        '| Localized seed | XY max m / yaw max deg | Base NEES mean | 95% ellipsoid coverage % | Marginal 95% coverage X/Y/yaw % | Camera position disagreement m / optical disagreement deg |',
        '|---|---:|---:|---:|---:|---:|']
    for seed in [42,917]:
        c=cases[f'localized_D{seed}'];l=c['localization_20_90'];t=c['physical_0_90']
        coverage='/'.join(f'{100*x:.1f}' for x in l['marginal_95percent_coverage_xyz_yaw'])
        lines.append(f"| {seed} | {l['xy_error_m']['max']:.4f} / {l['absolute_yaw_deg']['max']:.2f} | {l['nees_3d']['mean']:.3f} | "
            f"{100*l['fraction_inside_95percent_3d_ellipsoid']:.2f} | {coverage} | "
            f"{t['camera_model_position_error_m']['mean']:.4f} / {t['camera_optical_direction_difference_deg']['mean']:.2f} |")
    lines += ['', 'NEES/coverage are descriptive checks on temporally correlated samples, not independent-sample significance tests. XY is conservative; residual yaw undercoverage and the stochastic slip/bias approximation remain limitations. Base covariance stays positive and finite; it is not artificially collapsed. Detailed maxima, standard deviations, pose increments and camera disagreement for all policies are in each analysis `summary.json`.', '',
        '| Policy / seed | Controller compute mean / max ms | Longest invalid interval 0–90 s / 1.5–90 s | Observed recovery episodes | Unreleased episodes | Longest inward-demand stall s |',
        '|---|---:|---:|---:|---:|---:|']
    for key,c in sorted(cases.items()):
        m=c['result']['tracking']['metrics']['control_compute_time'];recover=c['actuation']['recoveries']
        lines.append(f"| {key} | {1000*m['mean']:.2f} / {1000*m['max']:.2f} | "
            f"{c['control_0_90']['max_invalid_duration_s']:.3f} / {c['control_1p5_90']['max_invalid_duration_s']:.3f} | "
            f"{len(recover)} | {sum(not r['released_to_active'] for r in recover)} | {c['stall']['max_contiguous_stall_s']:.3f} |")
    lines += ['', 'Control gaps and missed sampled-validity gates are retained. Initial waiting and later stale-input holds remain safe holds; they prevent an unconditional tracking acceptance. The first localized seed overlapped the independent copied build and part of functional verification. Offline analysis/regression were later paused to avoid contention in remaining physical trials. These RTF values are measured observations on WSL2, not an isolated causal benchmark of sensor overhead. Estimator CPU time and hardware latency are not measured; source processing-age counters are not compute-time measurements. Best-effort recorder gaps limit tracking/continuity coverage; native physics traces remain complete.', '',
        '| Joint | Maximum position excess across six cases rad | Maximum measured speed rad/s | Configured speed limit rad/s |',
        '|---|---:|---:|---:|']
    names = next(iter(cases.values()))['result']['physical']['joints']
    for name in names:
        joints=[c['result']['physical']['joints'][name] for c in cases.values()]
        lines.append(f"| {name} | {max(j['position_limits']['max_excess_rad'] for j in joints):.9g} | "
                     f"{max(j['peak_abs_velocity_rad_s'] for j in joints):.9g} | {joints[0]['speed_limit']['description_rad_s']:.6g} |")
    lines += ['', 'All six completed cases pass native full-step coverage and unchanged .001 rad / .001 rad/s acceptance. Tiny numerical excursions are reported, not hidden by clamping the measurements. No measured motor-torque certification or serious mechanical-fault coverage is added. The repaired DART backend and all native actuator/servo sources remain unchanged.', '',
        '### Independent verdicts', '',
        f"- **LOCALIZATION: {'PASS' if localization_ok and bootstrap_ok else 'CONDITIONAL'}** — nominal mean-error targets pass={localization_ok}; startup-outlier robustness={bootstrap_ok}. Nominal independent-source observability is demonstrated, but a plausible startup outlier can poison the initial estimate and subsequent gating. See the fault-probe timeline and proposed bootstrap closure below.",
        f"- **TRACKING: {'PASS' if improvements and compliance and validity else 'CONDITIONAL'}** — both-mean-improvement gate={improvements}, compliance-improvement gate={compliance}, full control-validity gate={validity}. Remaining physical task error, infeasible late-orbit motion and input/telemetry gaps prevent a stronger claim.",
        '- **SAFETY: CONDITIONAL** — all measured native position/speed gates pass and observed recovery release is reported; unchanged torque observability and serious-fault-injection gaps preclude real-hardware safety certification.',
        '- **ARCHITECTURE: PASS** — ordinary estimator/controller interfaces, optional source selection, typed configuration, ownership tests, normal/copied installation and runtime ROS graph verified; protected source scopes remain unchanged.', '',
        '### Evidence and anti-GIGO review', '',
        '| Fact / hypothesis | Controlled evidence | Conclusion |',
        '|---|---|---|',
        '| Wheel-only pose drifts; independent pose should constrain XY/yaw. | Original/localized/disabled, two fixed seeds, unchanged physical/controller/target inputs. | Aiding benefit is measured physically; wheel/slip uncertainty alone does not create global observability. |',
        '| Near-perfect truth could make metrics misleading. | Sensor errors, nonzero white noise and random calibration bias, dropout/delay, NIS gate, single-source ROS graph. | The production estimator consumes an explicit imperfect external-pose abstraction, not the diagnostic oracle. |',
        '| Internal task error may conceal physical camera error. | Authoritative physical FK from base truth and measured joints; aligned actual target; camera disagreement table. | Tracking improvement and remaining error are assessed from physical geometry. |',
        '| A passing suite might hide unsafe actuation. | Complete 2 ms native traces, unchanged tolerances, recovery/stall audits and unchanged backend hash. | Numerical position/speed conditions pass; unmeasured torque/fault conditions remain conditional. |', '',
        'Result sources:', '']
    lines += [f'- `{c["directory"]}`' for c in cases.values()]
    text=a.report.read_text()
    text=text.replace('Final numerical tables and verdicts are populated from retained runs after completion. ', '')
    assert '## Final measured comparison' not in text
    text=text.replace('## Reproduction\n', '\n'.join(lines)+'\n\n## Reproduction\n')
    summary = ('**Verdicts: LOCALIZATION '+('PASS' if localization_ok and bootstrap_ok else 'CONDITIONAL')+
               '; TRACKING '+('PASS' if improvements and compliance and validity else 'CONDITIONAL')+
               '; SAFETY CONDITIONAL; ARCHITECTURE PASS.**\n\n')
    text=text.replace('## Root cause, alternatives and selected architecture\n', summary+'## Root cause, alternatives and selected architecture\n')
    a.report.write_text(text)
    print(gates)


if __name__ == '__main__':
    main()
