"""Versioned portable artifacts, atomic writes and never-reused study directories."""
from datetime import datetime, timezone
import json
import os
from pathlib import Path
import tempfile


def plain(value):
    if isinstance(value,Path):return str(value)
    if isinstance(value,dict):return {str(k):plain(v) for k,v in value.items()}
    if isinstance(value,(list,tuple)):return [plain(v) for v in value]
    if hasattr(value,'tolist'):return value.tolist()
    return value


def write_json(path, value):
    """Atomically replace a run-owned artifact; reject nonfinite numbers."""
    path=Path(path)
    encoded=json.dumps(plain(value),indent=2,allow_nan=False)+'\n'
    fd, temporary=tempfile.mkstemp(prefix='.'+path.name,dir=path.parent)
    try:
        with os.fdopen(fd,'w') as output:
            output.write(encoded);output.flush();os.fsync(output.fileno())
        os.replace(temporary,path)
    finally:
        if Path(temporary).exists():Path(temporary).unlink()


def create_study_directory(base, name):
    base=Path(base);base.mkdir(parents=True,exist_ok=True)
    timestamp=datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%S_%fZ')
    return Path(tempfile.mkdtemp(prefix=f'{name}_{timestamp}_',dir=base)).resolve()
