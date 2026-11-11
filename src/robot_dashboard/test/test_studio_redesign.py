"""The redesigned Studio replaces the technical default and unifies Lab selection."""
from pathlib import Path
from robot_experiments.backend import ExperimentBackend
from robot_experiments.catalog import ExperimentCatalog


def test_default_studio_has_compact_generic_panels_and_no_source_apply_workflow(tmp_path):
    from robot_dashboard.app import create_app
    src=Path(__file__).resolve().parents[2]
    cat=ExperimentCatalog(tmp_path/'catalog.yaml',share=lambda p:src/p);cat.initialize()
    app=create_app(ExperimentBackend(tmp_path/'data',catalog=cat))
    data=app.server.test_client().get('/_dash-layout').get_data(as_text=True)
    for text in ['State-Space Equations','Parameters','Physical Output Mapping','Mathematical Representation','Motion Preview','Validate & Save','New Model']:
        assert text in data,text
    assert 'lab-apply-dynamics' not in data
    assert 'lab-dynamics-source' not in data
    assert 'dyn-id' not in data
    assert 'Dynamics Studio' in data and 'Results & History' in data
