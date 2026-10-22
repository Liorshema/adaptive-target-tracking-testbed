"""Task-relative inventory; preserve the pre-existing dirty tree and evidence."""
import hashlib
import json
from pathlib import Path
import subprocess
from robot_simulation.gazebo.physics_engine import physics_engine_library


def digest(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def main():
    root = Path(__file__).resolve().parents[2]
    baseline = json.loads((root/'docs/validation/m03_localization_baseline.json').read_text())
    current = subprocess.check_output(['git','ls-files','-c','-o','--exclude-standard','-z'], cwd=root).decode().split('\0')
    current = sorted({p for p in current if p and (root/p).is_file()})
    missing = [p for p in baseline['files'] if not (root/p).is_file()]
    changed = [p for p, h in baseline['files'].items() if (root/p).is_file() and digest(root/p) != h]
    added = [p for p in current if p not in baseline['files']]
    generated = sorted(str(p.relative_to(root)) for p in (root/'docs/validation').glob('m03_localization_*')
                       if p.is_file() and str(p.relative_to(root)) not in current
                       and not p.name.startswith('m03_localization_audit.'))
    result_roots = sorted(str(p.relative_to(root)) for p in (root/'results').glob('m03_localization_*') if p.is_dir())
    protected = ['src/robot_control/', 'src/robot_prediction/', 'src/robot_description/',
        'src/robot_simulation/robot_simulation/gazebo/native/',
        'src/robot_simulation/robot_simulation/actuation/',
        'src/robot_simulation/robot_simulation/dynamics/',
        'src/robot_simulation/config/gazebo/joint_position_hold.yaml']
    unexpected = [p for p in changed if any(p.startswith(prefix) for prefix in protected)]
    head = subprocess.check_output(['git','rev-parse','HEAD'], cwd=root, text=True).strip()
    staged = subprocess.check_output(['git','diff','--cached','--name-only'],cwd=root,text=True).splitlines()
    binary = Path(physics_engine_library('dartsim_recovery'))
    evidence = {'baseline_count': len(baseline['files']), 'missing': missing,
        'modified_existing_files': changed, 'added_files': added,
        'generated_validation_files': generated, 'result_roots': result_roots,
        'protected_scope_changes': unexpected, 'head': head, 'head_unchanged': head == baseline['head'],
        'staged_files': staged, 'backend': str(binary), 'backend_sha256': digest(binary),
        'backend_byte_unchanged': digest(binary) == '59c4a3a5481bf9cad4480b9af1307d09d1444ccb1d22a0af1f6aee99e5d0fc5e'}
    (root/'docs/validation/m03_localization_preservation.json').write_text(json.dumps(evidence,indent=2)+'\n')
    (root/'docs/validation/m03_localization_modified_files.txt').write_text(
        'TASK-MODIFIED PRE-EXISTING FILES\n'+'\n'.join(changed)+'\n\nTASK-ADDED NONIGNORED FILES\n'+'\n'.join(added)+
        '\n\nGENERATED VALIDATION FILES (ignored)\n'+'\n'.join(generated)+
        '\n\nGENERATED RESULT ROOTS (ignored; retained intact)\n'+'\n'.join(result_roots)+'\n')
    assert not missing and not unexpected and not staged and evidence['head_unchanged'] and evidence['backend_byte_unchanged']
    print(len(changed), 'existing files modified;', len(added), 'additions; protected scopes/HEAD/index preserved.')


if __name__ == '__main__':
    main()
