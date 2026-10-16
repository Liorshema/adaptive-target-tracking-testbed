"""Replay recorded target motion through the production estimator/predictor.

Run after sourcing the workspace, with OPENBLAS_NUM_THREADS=1. No ROS nodes
are started. All cases reuse the same seed-42 synthetic measurement sequence
and retain the actual sensor covariance; only assumed acceleration noise varies.
"""

import argparse
import json
from pathlib import Path
from types import SimpleNamespace

import numpy as np
from scipy.spatial.transform import Rotation
import yaml

from robot_control.interfaces.control_data import PredictedTargetTrajectory
from robot_control.pipeline.tracking_control_orchestrator import TrackingControlOrchestrator
from robot_estimation.common.measurement import Measurement
from robot_estimation.common.state import StateEstimate
from robot_estimation.filters.kalman_filter import KalmanFilter
from robot_estimation.target.models.measurement_model import PositionMeasurementModel
from robot_estimation.target.models.process_model import ConstantVelocityProcessModel
from robot_estimation.target.target_state_estimator import TargetStateEstimator
from robot_prediction.configuration.prediction_config import PredictionConfig
from robot_prediction.interfaces.prediction_data import PredictionInput
from robot_prediction.factory.system_factory import PredictionSystemFactory
from robot_simulation.measurements.synthetic_measurement import SyntheticMeasurementGenerator

ROOT = Path(__file__).resolve().parents[2]


def main():
    """Run a fair covariance sweep against recorded target geometry."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--samples', type=Path,
                        default=ROOT / 'docs/validation/v2_latency_trial/samples.jsonl')
    parser.add_argument('--result', type=Path,
                        default=ROOT / 'docs/validation/v2_latency_trial/result.json')
    parser.add_argument('--output', type=Path,
                        default=ROOT / 'docs/validation/v2_target_replay.json')
    args = parser.parse_args()
    rows = [json.loads(line) for line in args.samples.read_text().splitlines()]
    rows = sorted({row['time']: row for row in rows}.values(), key=lambda row: row['time'])
    recorded_times = np.array([row['time'] for row in rows])
    recorded_positions = np.array([row['target'] for row in rows])
    recorded_tracking = np.array([row['actual_position'] for row in rows])
    trial = json.loads(args.result.read_text())
    cfg = yaml.safe_load((ROOT / 'src/robot_estimation/config/target_estimator.yaml').read_text())['estimation_node']['ros__parameters']
    sensor = yaml.safe_load((ROOT / 'src/robot_simulation/config/measurements.yaml').read_text())['measurements']['target_position']
    robot = yaml.safe_load((ROOT / 'src/robot_simulation/config/gazebo/robot.yaml').read_text())['robot']
    rotation = Rotation.from_euler('xyz', robot['pose'][3:]).as_matrix()
    offset = np.array(robot['pose'][:3])
    dt = 1.0 / sensor['rate_hz']
    times = np.arange(recorded_times[0], recorded_times[-1] + 1.0e-9, dt)
    covariance = np.array(sensor['noise']['covariance'])

    def positions(query):
        """Interpolate measured ground truth and convert odometry to world."""
        odom = np.column_stack([
            np.interp(np.atleast_1d(query), recorded_times, recorded_positions[:, axis])
            for axis in range(3)
        ])
        return odom @ rotation.T + offset

    def velocities(query):
        """Estimate GT velocity using a 0.2-second centered difference."""
        return (positions(np.asarray(query) + 0.1) - positions(np.asarray(query) - 0.1)) / 0.2

    ground_truth = positions(times)
    tracking_positions = np.column_stack([
        np.interp(times, recorded_times, recorded_tracking[:, axis])
        for axis in range(3)
    ]) @ rotation.T + offset
    H = PositionMeasurementModel(3).measurement_matrix()
    generator = SyntheticMeasurementGenerator(H, covariance, np.random.default_rng(sensor['random_seed']))
    measurements = [generator.measure(np.r_[position, np.zeros(3)], timestamp)
                    for position, timestamp in zip(ground_truth, times)]
    prediction_cfg = PredictionConfig(ROOT / 'src/robot_prediction/config/prediction.yaml')
    prediction_parameters = prediction_cfg.get('prediction')
    sampler = SimpleNamespace(_target_velocity_feedforward=True, _prediction_sampling='interpolate')
    evaluate = (times >= trial['target_initialization_time'] + trial['thresholds']['settling_time']) & (
        times <= recorded_times[-1] - 1.05)
    shifts = np.arange(-1.5, 0.401, 0.01)
    results = []

    def summarize(predicted, truth):
        error = np.linalg.norm(predicted - truth, axis=1)
        return dict(mean=float(error.mean()), rms=float(np.sqrt(np.mean(error ** 2))),
                    p95=float(np.quantile(error, 0.95)), maximum=float(error.max()))

    def angular(predicted, truth, tracking):
        a, b = predicted - tracking, truth - tracking
        cosine = np.sum(a * b, axis=1) / (np.linalg.norm(a, axis=1) * np.linalg.norm(b, axis=1))
        angle = np.degrees(np.arccos(np.clip(cosine, -1.0, 1.0)))
        return dict(mean=float(angle.mean()), p95=float(np.quantile(angle, 0.95)), maximum=float(angle.max()))

    def best_shift(states, velocity=False):
        scores = []
        for shift in shifts:
            truth = velocities(times[evaluate] + shift) if velocity else positions(times[evaluate] + shift)
            scores.append(float(np.linalg.norm(states[evaluate] - truth, axis=1).mean()))
        best = int(np.argmin(scores))
        return dict(gt_time_shift_seconds=float(shifts[best]), mean_error=float(scores[best]))

    for sigma in [0.1, 0.3, 0.5, 1.0, 2.0]:
        process = ConstantVelocityProcessModel(3, sigma)
        estimator = TargetStateEstimator(KalmanFilter(
            initial_state=StateEstimate(np.array(cfg['target_initial_state']),
                                       np.diag(cfg['target_initial_covariance_diagonal']), timestamp=None),
            state_transition=process.state_transition_matrix,
            process_covariance=process.process_covariance,
            measurement_matrix=H,
        ))
        system = PredictionSystemFactory(prediction_cfg).create()
        states, sampled = [], {0.0: [], 0.25: []}
        model_probabilities = []
        for index, (timestamp, measured) in enumerate(zip(times, measurements)):
            if index:
                estimator.predict(dt=timestamp - times[index - 1])
            estimator.update(Measurement(measured.value, covariance, timestamp))
            estimate = estimator.state
            if system.last_prediction is not None:
                system.update(estimate.mean, observation_dt=timestamp - times[index - 1],
                              observation_timestamp=timestamp)
            prediction = system.predict(PredictionInput(
                state=estimate.mean, timestamp=timestamp, dt=prediction_parameters['dt'],
                horizon=prediction_parameters['horizon'], covariance=estimate.covariance,
            ))
            contract = PredictedTargetTrajectory(prediction.states, prediction.timestamps, 'world')
            states.append(estimate.mean)
            for horizon in sampled:
                sampled[horizon].append(TrackingControlOrchestrator._select_target_state(
                    sampler, contract, timestamp + horizon))
            model_probabilities.append(system._predictor.model_probabilities)
        states = np.array(states)
        current_gt = positions(times[evaluate])
        current_velocity = velocities(times[evaluate])
        result = dict(
            target_acceleration_noise_std=sigma,
            assumed_position_variance=np.diag(covariance).tolist(),
            estimated_position_error_m=summarize(states[evaluate, :3], current_gt),
            estimated_velocity_error_mps=summarize(states[evaluate, 3:], current_velocity),
            estimated_position_best_gt_shift=best_shift(states[:, :3]),
            estimated_velocity_best_gt_shift=best_shift(states[:, 3:], velocity=True),
            mean_imm_probabilities=np.mean(np.array(model_probabilities)[evaluate], axis=0).tolist(),
            queries={},
        )
        for horizon, query_states in sampled.items():
            query_states = np.array(query_states)
            future_gt = positions(times[evaluate] + horizon)
            future_velocity = velocities(times[evaluate] + horizon)
            result['queries'][str(horizon)] = dict(
                position_error_vs_current_gt_m=summarize(query_states[evaluate, :3], current_gt),
                position_error_vs_query_gt_m=summarize(query_states[evaluate, :3], future_gt),
                direction_error_vs_current_gt_degrees=angular(query_states[evaluate, :3], current_gt, tracking_positions[evaluate]),
                direction_error_vs_query_gt_degrees=angular(query_states[evaluate, :3], future_gt, tracking_positions[evaluate]),
                velocity_error_vs_current_gt_mps=summarize(query_states[evaluate, 3:], current_velocity),
                velocity_error_vs_query_gt_mps=summarize(query_states[evaluate, 3:], future_velocity),
                position_best_gt_shift=best_shift(query_states[:, :3]),
                velocity_best_gt_shift=best_shift(query_states[:, 3:], velocity=True),
            )
        results.append(result)
        print(json.dumps(dict(sigma=sigma, position_error=result['estimated_position_error_m']['mean'],
                              velocity_error=result['estimated_velocity_error_mps']['mean'],
                              query0_error=result['queries']['0.0']['position_error_vs_current_gt_m']['mean'],
                              query025_error_now=result['queries']['0.25']['position_error_vs_current_gt_m']['mean'],
                              query025_error_future=result['queries']['0.25']['position_error_vs_query_gt_m']['mean'])), flush=True)
    noise = np.array([measurement.value for measurement in measurements]) - ground_truth
    acceleration_step = 0.2
    acceleration = (
        positions(times[evaluate] + acceleration_step)
        - 2.0 * positions(times[evaluate])
        + positions(times[evaluate] - acceleration_step)
    ) / acceleration_step ** 2
    output = dict(
        source_samples=str(args.samples), source_result=str(args.result),
        measured_sensor_covariance=covariance.tolist(), configured_sensor_standard_deviation=np.sqrt(np.diag(covariance)).tolist(),
        estimator_position_variance=cfg['target_position_variance'], sensor_seed=sensor['random_seed'],
        replay_samples=len(times), evaluation_samples=int(evaluate.sum()),
        replay_time_range=[float(times[0]), float(times[-1])], evaluation_time_range=[float(times[evaluate][0]), float(times[evaluate][-1])],
        replay_dt=dt, realized_noise_rms=np.sqrt(np.mean(noise ** 2, axis=0)).tolist(),
        recorded_acceleration_rms_per_axis=np.sqrt(np.mean(acceleration ** 2, axis=0)).tolist(),
        recorded_acceleration_norm_mean=float(np.linalg.norm(acceleration, axis=1).mean()),
        recorded_acceleration_norm_p95=float(np.quantile(np.linalg.norm(acceleration, axis=1), 0.95)),
        observed_sensor_config_matches_estimator_covariance=bool(np.allclose(np.diag(covariance),cfg['target_position_variance'])),
        methods=dict(pipeline='Production TargetStateEstimator/KalmanFilter, PredictionSystemFactory IMM+ResidualAdapter, Control interpolation sampler',
                     ground_truth='Recorded target position, resampled at configured 20 Hz; fixed robot spawn transform restores world frame',
                     velocity_truth='Centered 0.2-second finite difference of recorded/interpolated target positions',
                     query_semantics='Query 0 and .25 are measured both against current GT and future GT at the exact query stamp',
                     negative_gt_shift='Negative best GT time shift denotes lag relative to the recorded target path'),
        limitations=['Recorded GT is the latest observer target sample, not a full 100 Hz trajectory or sensor bag.',
                     'Replay begins at first recorded sample and uses a new seed-42 noise sequence; actual earlier live measurements are unavailable.',
                     'No callback latency/drops or robot reaction are reproduced; frame transform is fixed, and all noise/process inputs are identical across cases.',
                     'Q std is a process-model responsiveness assumption; the simulator sensor covariance and physical target trajectory are unchanged.'],
        results=results,
    )
    args.output.write_text(json.dumps(output, indent=2) + '\n')


if __name__ == '__main__':
    main()
