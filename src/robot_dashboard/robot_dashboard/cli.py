"""Start the optional local Dash client; Catalog remains the persistence authority."""

import argparse
import os
from pathlib import Path


def main(argv=None):
    """Serve locally, with explicit data/Catalog locations and no ROS/Gazebo launch."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        '--host', default='127.0.0.1', help='Bind address (default: localhost only)'
    )
    parser.add_argument('--port', type=int, default=8050)
    data = Path(os.environ.get('XDG_DATA_HOME', Path.home() / '.local/share'))
    parser.add_argument(
        '--data-dir',
        type=Path,
        default=data / 'adaptive_tracking',
        help='Writable metadata directory; never a package share directory',
    )
    parser.add_argument(
        '--catalog',
        type=Path,
        help='Existing writable Catalog; default: DATA_DIR/catalog.yaml',
    )
    parser.add_argument(
        '--result-root',
        type=Path,
        action='append',
        default=[],
        help='Register existing Run evidence (repeatable); new Job outputs register automatically',
    )
    parser.add_argument(
        '--managed-result-root',
        type=Path,
        action='append',
        default=None,
        help='Explicit writable deletion boundary (repeatable); default: DATA_DIR/runs. '
        'Historical --result-root paths remain read-only for deletion.',
    )
    args = parser.parse_args(argv)
    if not args.host or not 1 <= args.port <= 65535:
        parser.error('host must be nonempty and port must be in [1, 65535]')
    from robot_experiments.backend import ExperimentBackend
    from robot_experiments.catalog import CatalogConflict
    from robot_experiments.catalog import ExperimentCatalog

    catalog = ExperimentCatalog(args.catalog or args.data_dir / 'catalog.yaml')
    if not catalog.path.exists():
        if args.catalog is not None:
            parser.error('--catalog must reference an existing writable Catalog')
        try:
            catalog.initialize()
        except CatalogConflict:
            # Another client may have initialized the same Catalog under its owner lock.
            if not catalog.path.exists():
                raise
    from .app import create_app

    backend = ExperimentBackend(
        args.data_dir,
        catalog=catalog,
        result_roots=args.result_root,
        managed_result_roots=args.managed_result_root,
    )
    app = create_app(backend)
    print(f'Catalog: {catalog.path}', flush=True)
    app.run(host=args.host, port=args.port, debug=False, use_reloader=False)
    return 0
