"""Verify real current/archived probe globals without starting a ROS graph."""

import argparse
from importlib import import_module
import json
import os
from pathlib import Path
import runpy
import subprocess
import sys

HERE = Path(__file__).resolve().parent


def verify(kind, archived, output):
    """Intercept only final executable dispatch; execute actual instrumentation code."""
    output.mkdir(parents=True, exist_ok=True)
    if kind == 'estimation':
        module_name = 'robot_estimation.nodes.estimation_node'
        class_name, script = 'EstimationNode', 'estimator_timing_probe.py'
        environment, baseline = 'M03_BASELINE_ESTIMATION', HERE / 'm03_before_source'
        os.environ['ESTIMATION_TIMING_LOG'] = str(output / 'processing.json')
    else:
        module_name = 'robot_simulation.nodes.velocity_actuation'
        class_name, script = 'VelocityActuationNode', 'actuation_dynamics_trace.py'
        environment, baseline = 'M02_BASELINE_ACTUATION', HERE / 'm02_before_source'
        os.environ['ACTUATION_DYNAMICS_TRACE'] = str(output / 'actuation_trace.json')
    os.environ.pop(environment, None)
    if archived:
        os.environ[environment] = str(baseline)
    namespace = {'__name__': '__main__', '__file__': str(HERE / script)}
    called = []

    def console_dispatch(path, run_name):
        assert path == 'verify-console' and run_name == '__main__'
        module = import_module(module_name)
        node = getattr(module, class_name)
        assert namespace[class_name] is node
        origin = Path(node.__init__.__code__.co_filename).resolve()
        if archived:
            assert origin.parent == baseline.resolve(), origin
        else:
            assert origin.name == module_name.split('.')[-1]+'.py' and origin.parent.name == 'nodes'
        assert node.__init__.__globals__ is module.__dict__
        wrapped = namespace['predict' if kind == 'estimation' else 'send']
        assert wrapped.__globals__ is module.__dict__
        called.append(dict(kind=kind, archived=archived, module=module_name,
                           implementation_origin=str(origin), shared_globals=True))

    runpy.run_path = console_dispatch
    sys.argv = [script, 'verify-console']
    exec(compile((HERE / script).read_text(), str(HERE / script), 'exec'), namespace)
    assert len(called) == 1
    print(json.dumps(called[0]))


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--case', choices=['estimation', 'actuation'])
    parser.add_argument('--archived', action='store_true')
    parser.add_argument('--output', type=Path, default=HERE / 'm06_instrumentation')
    args = parser.parse_args()
    if args.case:
        verify(args.case, args.archived, args.output)
    else:
        results = []
        for kind in ['estimation', 'actuation']:
            for archived in [False, True]:
                command = [sys.executable, str(Path(__file__).resolve()), '--case', kind,
                           '--output', str(args.output / (kind + ('_before' if archived else '_current')))]
                if archived:
                    command.append('--archived')
                completed = subprocess.run(command, check=True, capture_output=True, text=True)
                results.append(json.loads(completed.stdout))
        (HERE / 'm06_instrumentation.json').write_text(json.dumps(results, indent=2)+'\n')
        print(json.dumps(results, indent=2))
