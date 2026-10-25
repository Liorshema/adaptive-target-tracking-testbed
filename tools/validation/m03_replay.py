"""Compare both robot estimators on exactly the same recorded arrival stream."""

import argparse
from collections import Counter
import hashlib
import importlib.util
import json
from pathlib import Path

import numpy as np
import yaml

from robot_estimation.measurement import Measurement
from robot_estimation.state import StateEstimate
from robot_estimation.timestamp_buffer import TimestampBuffer
from robot_estimation.robot.joint_bounds import JointBoundsConditioner
from robot_estimation.robot.models.measurement_model import (
    WheelOdometryMeasurementModel, JointEncoderMeasurementModel, ImuYawRateMeasurementModel)
from robot_estimation.robot.models.process_model import RobotProcessModel
from robot_estimation.robot.robot_state_estimator import RobotStateEstimator

from m03_analysis import estimate_errors
from archived_estimation_imports import canonicalize_imports

HERE=Path(__file__).resolve().parents[2] / 'results/validation'


def archived(filename, name):
    spec=importlib.util.spec_from_file_location(name,HERE/'m03_before_source'/filename)
    module=importlib.util.module_from_spec(spec)
    source = HERE / 'm03_before_source' / filename
    exec(compile(canonicalize_imports(source.read_text()), str(source), 'exec'), module.__dict__)
    return module


def run(events, snapshot, before):
    cfg={**snapshot['estimator'], **snapshot.get('estimation_overrides', {})}
    n=len(snapshot['joint_order'])
    process_type=(archived('process_model.py','m03_old_process').RobotProcessModel
                  if before else RobotProcessModel)
    estimator_type=(archived('robot_state_estimator.py','m03_old_estimator').RobotStateEstimator
                    if before else RobotStateEstimator)
    process=process_type(n,cfg['linear_acceleration_noise_std'],
                         cfg['angular_acceleration_noise_std'],cfg['joint_acceleration_noise_std'])
    constraint=None
    if not before:
        physical=list(snapshot['description']['manipulator']['joints'].values())
        constraint=JointBoundsConditioner(np.arange(5,5+n),[j['lower'] for j in physical],
            [j['upper'] for j in physical],cfg['joint_bounds_numerical_tolerance'],cfg['max_joint_state_age'])
    arguments=dict(initial_state=StateEstimate(np.array(cfg['robot_initial_state']),
        np.diag(cfg['robot_initial_covariance_diagonal'])),process_model=process,
        wheel_measurement_model=WheelOdometryMeasurementModel(n),
        imu_measurement_model=ImuYawRateMeasurementModel(n),
        joint_measurement_model=JointEncoderMeasurementModel(n))
    estimator=estimator_type(**arguments) if before else estimator_type(**arguments,state_constraint=constraint)
    counters=Counter();last=None;last_sample=None;samples=[]
    buffer=None if before else TimestampBuffer(cfg['reorder_holdback'],cfg['reorder_max_age'],
                                               cfg['future_tolerance'],cfg['reorder_capacity'])
    def fuse(source, stamp, values):
        nonlocal last,last_sample
        if last is not None and stamp<last:
            counters['rejected_late']+=1
            return
        if last is not None and stamp>last:
            estimator.predict(stamp-last)
        if source=='wheel':
            covariance=np.diag([cfg['wheel_linear_velocity_variance'],cfg['wheel_angular_velocity_variance']])
            method=estimator.update_wheel_odometry
        elif source=='joints':
            covariance=np.diag([cfg['joint_position_variance']]*n+[cfg['joint_velocity_variance']]*n)
            method=estimator.update_joint_encoders
        else:
            covariance=np.array([[cfg['imu_yaw_rate_variance']]])
            method=estimator.update_imu
        method(Measurement(np.array(values),covariance,stamp));last=stamp
        counters[source+'_accepted']+=1
        state=estimator.state
        if last_sample is None or stamp-last_sample>=.02-1e-9:
            samples.append([stamp,state.mean.tolist(),state.covariance.ravel().tolist()])
            last_sample=stamp
    def drain(now):
        for source,stamp,values in buffer.pop_ready(now):fuse(source,stamp,values)
    tick=events[0]['now']
    for event in events:
        source,stamp,now,values=event['source'],event['stamp'],event['now'],event.get('values')
        if values is None or not np.all(np.isfinite(values)):
            counters['malformed']+=1;continue
        if before:
            if not np.isfinite(stamp) or stamp>now+cfg['future_tolerance']:
                counters['rejected_future']+=1;continue
            fuse(source,stamp,values)
        else:
            while tick<=now:
                drain(tick);tick+=1/cfg['reorder_rate_hz']
            buffer.push(source,stamp,values,now)
    if buffer is not None:drain(events[-1]['now']+cfg['reorder_holdback']+.02)
    return samples,dict(counters),None if buffer is None else buffer.diagnostics()


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__);parser.add_argument('capture',type=Path)
    args=parser.parse_args();path=args.capture
    snapshot=yaml.safe_load((path/'configuration_snapshot.yaml').read_text())
    streams=json.loads((path/'experiment_streams.json').read_text())
    events=json.loads((path/'processing.json').read_text())
    result=json.loads((path/'result.json').read_text());origin=result['target_initialization_time']
    output={'input_sha256':hashlib.sha256((path/'processing.json').read_bytes()).hexdigest()}
    for before in [True,False]:
        samples,counters,queue=run(events,snapshot,before)
        output['before' if before else 'after']=dict(counters=counters,queue=queue,
            errors=estimate_errors(samples,streams,snapshot,origin+10,origin+60))
    (HERE/'m03_replay.json').write_text(json.dumps(output,indent=2)+'\n')
    print(json.dumps(output,indent=2))
