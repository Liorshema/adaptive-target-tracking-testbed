#!/usr/bin/env python3
"""Render real source directories using one responsibility-based display policy."""

import argparse
from pathlib import Path

STRUCTURAL = ('nodes', 'interfaces', 'factory', 'adapters')
# Domain order follows conceptual dependencies/data flow, not alphabetical order.
DOMAINS = {
    'robot_configuration': (),
    'robot_description': (),
    'robot_interfaces': ('msg',),
    'robot_models': ('spatial', 'base', 'manipulator', 'whole_body', 'target',
                     'camera', 'reference', 'task_space', 'constraints'),
    'robot_estimation': ('filters', 'robot', 'target'),
    'robot_prediction': ('baseline', 'history', 'evaluation', 'online_adaptation',
                         'fusion', 'configuration'),
    'robot_control': ('tracking', 'controllers', 'safety', 'pipeline'),
    'robot_simulation': ('state', 'dynamics', 'disturbances', 'integration',
                         'measurements', 'engine', 'gazebo', 'actuation', 'diagnostics'),
    'robot_bringup': (),
}
IGNORED = {'build', 'install', 'log', '__pycache__', '.pytest_cache', '.ruff_cache',
           'results', '.git'}


def ordered_children(directory, package):
    """Order real entries; unknown domain directories remain visible after known ones."""
    directory = Path(directory)
    order = STRUCTURAL + DOMAINS.get(package, ())
    if (directory / 'package.xml').is_file():
        order = (package, 'config', 'launch', 'test', 'resource', 'msg', 'urdf',
                 'models', 'worlds')
    rank = {name: index for index, name in enumerate(order)}

    def key(path):
        if path.is_dir():
            return (0, rank.get(path.name, len(rank)), path.name)
        return (1, path.name == '__init__.py', path.name)

    return sorted((p for p in directory.iterdir()
                   if p.name not in IGNORED and not p.name.startswith('.')
                   and p.suffix not in {'.pyc', '.pyo'}
                   and not p.name.endswith('.egg-info')), key=key)


def render_tree(directory, package, depth=None):
    """Render an actual tree, showing directories before files and initializers last."""
    directory = Path(directory)
    lines = [directory.name + '/']

    def walk(parent, prefix, level):
        if depth is not None and level >= depth:
            return
        children = ordered_children(parent, package)
        for index, path in enumerate(children):
            last = index == len(children) - 1
            lines.append(prefix + ('└── ' if last else '├── ') + path.name +
                         ('/' if path.is_dir() else ''))
            if path.is_dir() and not path.is_symlink():
                walk(path, prefix + ('    ' if last else '│   '), level + 1)

    walk(directory, '', 0)
    return '\n'.join(lines)


def render_workspace(src, depth=None):
    """Discover ROS packages; policy order is explicit and new packages stay visible."""
    src = Path(src)
    packages = [p for p in src.iterdir() if p.is_dir() and (p / 'package.xml').is_file()]
    ranks = {name: index for index, name in enumerate(DOMAINS)}
    packages.sort(key=lambda p: (ranks.get(p.name, len(ranks)), p.name))
    return '\n\n'.join(render_tree(p, p.name, depth) for p in packages)


def main():
    """Print the whole source tree or one package's Python implementation."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--src', type=Path, default=Path(__file__).resolve().parents[1] / 'src')
    parser.add_argument('--package', choices=tuple(DOMAINS))
    parser.add_argument('--depth', type=int)
    args = parser.parse_args()
    if args.depth is not None and args.depth < 0:
        parser.error('--depth must be non-negative')
    if args.package:
        path = args.src / args.package / args.package
        if not path.is_dir():
            path = args.src / args.package
        print(render_tree(path, args.package, args.depth))
    else:
        print(render_workspace(args.src, args.depth))


if __name__ == '__main__':
    main()
