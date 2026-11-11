"""Removed Lab preview UI must leave the underlying lifecycle owner intact."""
from pathlib import Path
from robot_dashboard.app import create_app
from robot_experiments.backend import ExperimentBackend
from robot_experiments.catalog import ExperimentCatalog


def test_lab_never_launches_or_registers_a_competing_simulation_ui(tmp_path, monkeypatch):
    """Constructing the dashboard cannot launch either preview or scored execution."""
    src = Path(__file__).parents[2]
    catalog = ExperimentCatalog(tmp_path / 'catalog.yaml', lambda package: src / package)
    catalog.initialize()
    backend = ExperimentBackend(tmp_path / 'workspace', catalog=catalog)

    def forbidden(*args, **kwargs):
        raise AssertionError('Opening the dashboard must not launch simulation')

    monkeypatch.setattr(backend, 'open_simulation', forbidden)
    monkeypatch.setattr(backend, 'start_run', forbidden)
    app = create_app(backend)
    assert app.server.test_client().get('/_dash-layout').status_code == 200
    assert not any('simulation-state' in key for key in app.callback_map)
    assert callable(backend.close_simulation)
