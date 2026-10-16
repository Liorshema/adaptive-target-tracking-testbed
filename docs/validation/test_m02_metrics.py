"""Check that physical audits cannot omit startup or inactive overspeed."""
from m02_metrics import audit_joints


def test_all_physical_samples_and_exact_excess_are_audited():
    result = audit_joints([[1., [.1], [0.]], [1.002, [-.0001], [2.]],
                           [1.004, [.1], [0.]]],
                          {'joint': dict(lower=0., upper=1., velocity=1.)})
    j = result['joints']['joint']
    assert j['max_position_excess_rad'] == .0001
    assert j['peak_speed_rad_s'] == 2.
    assert j['speed_violation_samples'] == 1
    assert abs(j['speed_violation_time_estimate_s'] - .002) < 1e-12
    assert j['position_outside_acceptance_time_estimate_s'] == 0.
