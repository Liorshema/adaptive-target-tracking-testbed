"""The installed entry delegates to the supported configuration workflow."""
from pathlib import Path
from robot_experiments.cli import main, component_identities

def test_cli_validation_and_invalid_path(capsys):
 root=Path(__file__).resolve().parents[2]
 share=lambda name:root/name
 assert main(['--config',str(root/'robot_experiments/config/paired_prediction.yaml'),'--validate-only'],share=share)==0
 assert 'ConstantVelocityPredictor' in capsys.readouterr().out
 assert main(['--config','missing.yaml','--validate-only'],share=share)==2

def test_actual_runtime_identity_requires_node_logs():
 expected={'predictor':'x.CV','controller':'x.WLS'}
 actual,problems=component_identities('System assembly: {"predictor":"x.CV"}',expected)
 assert problems and not actual
 actual,problems=component_identities("Prediction node started: {'predictor':'x.CV'}\nControl components: {'controller':'x.WLS'}",expected)
 assert actual==expected and not problems
