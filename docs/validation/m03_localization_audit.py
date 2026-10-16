"""Independent native-pose and measured wheel-angle audit of existing captures."""

import argparse
import json
from pathlib import Path
import xml.etree.ElementTree as ET

import numpy as np
import yaml

from m03_analysis import drive_phases, interpolate, planar, wrap


def audit(path):
    """Read evidence only; no estimator or simulation feedback path exists here."""
    streams = json.loads((path / 'experiment_streams.json').read_text())
    snapshot = yaml.safe_load((path / 'configuration_snapshot.yaml').read_text())
    result = json.loads((path / 'result.json').read_text())
    native = []
    for line in (path / 'gazebo_poses.jsonl').read_text().splitlines():
        message = json.loads(line)
        stamp = message['header']['stamp']
        t = float(stamp.get('sec', 0)) + float(stamp.get('nsec', 0))*1e-9
        for pose in message['pose']:
            if pose['name'] == snapshot['robot']['entity_name']:
                native.append([t, [pose.get('position', {}).get(k, 0.) for k in 'xyz'],
                               [pose.get('orientation', {}).get(k, 0.) for k in 'xyzw']])
    tn, native_pose = planar(native)
    tp, physical = planar(streams['physical_base'])
    mask = (tp >= tn[0]) & (tp <= tn[-1])
    difference = physical[mask]-interpolate(tn, native_pose, tp[mask])
    output = {'physical_vs_native': {
        'samples': int(mask.sum()),
        'max_planar_position_error_m': float(np.linalg.norm(difference[:, :2], axis=1).max()),
        'max_yaw_error_rad': float(np.abs(wrap(difference[:, 2])).max())}}
    if streams.get('wheel_joints') and 'drive_fixture' in snapshot:
        geometry = snapshot['description']['wheels']
        radius, separation = geometry['radius'], 2*geometry['y_offset']
        sdf = ET.parse(path / 'generated_world.sdf')
        plugin = next(p for p in sdf.findall('.//plugin')
                      if 'diff-drive' in p.get('filename', ''))
        output['geometry'] = dict(
            authoritative_radius_m=radius, authoritative_separation_m=separation,
            sdf_radius_m=float(plugin.findtext('wheel_radius')),
            sdf_separation_m=float(plugin.findtext('wheel_separation')),
            wheel_order=(snapshot['robot']['differential_drive']['left_joints'] +
                         snapshot['robot']['differential_drive']['right_joints']))
        tw = np.array([s[0] for s in streams['wheel_joints']])
        angles = np.array([s[1] for s in streams['wheel_joints']])
        speeds = np.array([s[2] for s in streams['wheel_joints']])
        to, odometry = planar(streams['odometry'])
        elapsed = 0.
        phases = {}
        for segment in snapshot['drive_fixture']['segments']:
            start = result['target_initialization_time'] + elapsed
            elapsed += segment['seconds']
            end = result['target_initialization_time'] + elapsed
            mask = ((to >= start + snapshot['drive_fixture']['ramp_seconds'] + .1) &
                    (to <= end - .1) & (to >= tw[0]) & (to <= tw[-1]))
            times = to[mask]
            a = interpolate(tw, angles, times)
            da = a[-1]-a[0]
            right = len(snapshot['robot']['differential_drive']['left_joints'])
            measured_yaw = radius/separation*(da[right]-da[0])
            phases[segment['name']] = dict(
                median_wheel_speeds_rad_s=np.median(
                    speeds[(tw >= times[0]) & (tw <= times[-1])], axis=0).tolist(),
                first_wheel_pair_yaw_change_rad=float(measured_yaw),
                odom_yaw_change_rad=float(odometry[mask][-1, 2]-odometry[mask][0, 2]),
                wheel_vs_odom_yaw_increment_error_rad=float(abs(
                    measured_yaw-(odometry[mask][-1, 2]-odometry[mask][0, 2]))))
        output['wheel_phases'] = phases
    if 'drive_fixture' in snapshot:
        output['drive_phases'] = drive_phases(
            streams, snapshot, result['target_initialization_time'])
    (path / 'm03_localization_audit.json').write_text(json.dumps(output, indent=2)+'\n')
    return output


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('captures', nargs='+', type=Path)
    for capture in parser.parse_args().captures:
        print(capture, json.dumps(audit(capture)))
