"""Advisory locking and atomic YAML replacement shared by workspace catalogs."""

import fcntl
import os
import tempfile
from contextlib import contextmanager
from pathlib import Path

import yaml


@contextmanager
def locked_yaml(path):
    """Serialize writers using a stable sidecar inode; keep the lock file afterward."""
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.with_suffix(path.suffix + '.lock').open('a') as lock:
        fcntl.flock(lock, fcntl.LOCK_EX)
        try:
            yield
        finally:
            fcntl.flock(lock, fcntl.LOCK_UN)


def atomic_write_yaml(path, document):
    """Flush a sibling temporary file, replace atomically, then fsync its directory."""
    path = Path(path)
    encoded = yaml.safe_dump(document, sort_keys=False)
    fd, name = tempfile.mkstemp(prefix='.' + path.name, dir=path.parent)
    try:
        with os.fdopen(fd, 'w') as stream:
            stream.write(encoded)
            stream.flush()
            os.fsync(stream.fileno())
        os.replace(name, path)
        directory = os.open(path.parent, os.O_RDONLY | os.O_DIRECTORY)
        try:
            os.fsync(directory)
        finally:
            os.close(directory)
    finally:
        if Path(name).exists():
            Path(name).unlink()
