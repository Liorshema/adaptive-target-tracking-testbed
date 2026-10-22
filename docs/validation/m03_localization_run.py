"""Matched localization trials using the existing M08/M02 launch and trace lifecycle."""
import argparse
from copy import deepcopy
from pathlib import Path

import yaml
from ament_index_python.packages import get_package_share_directory
from robot_bringup.factory.system_configuration import load_system_configuration
from robot_experiments.configuration import load_experiment
from robot_experiments.reporting import create_study_directory, write_json
import m02_resolution_run as existing


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--duration', type=float, default=90.)
    parser.add_argument('--seeds', type=int, nargs='+', default=[42, 917])
    parser.add_argument('--modes', nargs='+', choices=['original', 'localized', 'disabled', 'degraded'],
                        default=['original', 'localized', 'disabled'])
    args = parser.parse_args()
    share = lambda package: Path(get_package_share_directory(package))
    spec = load_experiment(share('robot_experiments')/'config/reactive_manipulation.yaml', share)
    original = next(v for v in spec['variants'] if v['name'] == 'D')
    localized = deepcopy(original)
    localized['system_config'] = share('robot_bringup')/'config/system_manipulation_D_localized.yaml'
    localized['plan'] = load_system_configuration(localized['system_config'], share,
        overrides={'enable_actuation': 'true', 'publish_ground_truth': 'true',
                   'gui': 'false', 'visualize_tracking': 'false'})
    study = create_study_directory(args.output, 'm03_localization')
    materialize = existing.materialize_run
    recorder_type = existing.Recorder
    results = []
    for mode in args.modes:
        for seed in args.seeds:
            directory = study/f'{mode}_D{seed}'
            variant = deepcopy(original if mode == 'original' else localized)
            variant['name'] = mode
            def prepare(*positional, **keyword):
                prepared = materialize(*positional, **keyword)
                if mode in ['disabled', 'degraded']:
                    path = directory/'inputs/localization_sensor.yaml'
                    cfg = yaml.safe_load(path.read_text())
                    if mode == 'disabled':
                        cfg['localization_sensor']['enabled'] = False
                    else:
                        cfg['localization_sensor'].update(outages=[[5., 7.]], outlier_probability=.05)
                    path.write_text(yaml.safe_dump(cfg, sort_keys=False))
                    prepared['snapshot']['localization_validation_override'] = cfg
                return prepared
            class Observer(recorder_type):
                def spin_once(self, timeout=.05):
                    super().spin_once(timeout)
                    clock = self.streams.get('clock', [])
                    if not getattr(self, '_graph_saved', False) and clock and clock[-1]['source_time'] > 5.:
                        graph = {}
                        for topic, types in self._node.get_topic_names_and_types():
                            if any(s in topic for s in ['ground_truth', 'localization', 'robot_state_full']):
                                graph[topic] = {'types': types,
                                    'publishers': [i.node_name for i in self._node.get_publishers_info_by_topic(topic)],
                                    'subscribers': [i.node_name for i in self._node.get_subscriptions_info_by_topic(topic)]}
                        write_json(directory/'graph.json', graph)
                        self._graph_saved = True
            existing.materialize_run = prepare
            existing.Recorder = Observer
            result = existing.trial(spec, variant, seed, 'velocity_servo', directory,
                                    share, args.duration, 0.)
            results.append(result)
            write_json(study/'results.json', results)
            print(directory, 'error=', result.get('error'),
                  'physical=', result.get('physical', {}).get('accepted'), flush=True)
    print(study, flush=True)


if __name__ == '__main__':
    main()
