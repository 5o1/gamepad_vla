"""Finite diagnostic captures and bounded child-process error output."""
import json
import os
from pathlib import Path
import threading
import time


DIAGNOSTIC_SECONDS = 900
DIAGNOSTIC_ROWS = 120_000
DIAGNOSTIC_BYTES = 32 * 1024 * 1024


class DiagnosticFile:
    """Lazy: a disabled or expired capture never creates a file."""
    def __init__(self, path, enabled=False, *, started=None, seconds=DIAGNOSTIC_SECONDS,
                 rows=DIAGNOSTIC_ROWS, max_bytes=DIAGNOSTIC_BYTES, clock=time.monotonic):
        self.path = Path(path)
        self.clock = clock
        self.deadline = (clock() if started is None else started) + seconds
        self.enabled = enabled
        self.rows_left, self.bytes_left = rows, max_bytes
        self.file = None

    def write(self, data):
        if isinstance(data, str):
            data = data.encode()
        if not self.enabled:
            return
        if self.clock() >= self.deadline or self.rows_left <= 0 or len(data) > self.bytes_left:
            self.close()
            return
        if self.file is None:
            self.file = self.path.open('wb', buffering=0)
        self.file.write(data)
        self.rows_left -= 1
        self.bytes_left -= len(data)
        if self.rows_left == 0 or self.bytes_left == 0:
            self.close()

    def json(self, value):
        if self.enabled:
            self.write(json.dumps(value, separators=(',', ':')) + '\n')

    def close(self):
        self.enabled = False
        if self.file is not None:
            self.file.close()
            self.file = None

    def __enter__(self):
        return self

    def __exit__(self, *exc):
        self.close()


def diagnostic_file(run, name, cfg):
    return DiagnosticFile(Path(run)/name, cfg.get('diagnostics') is True,
                          started=cfg.get('diagnostics_started'))


class CappedProcessLog:
    """Drain child output continuously, retaining at most 1 MiB per launch."""
    def __init__(self, path, max_bytes=1024 * 1024):
        reader, self.writer = os.pipe()
        self.thread = threading.Thread(target=self._drain, args=(reader, Path(path), max_bytes), daemon=True)
        self.thread.start()

    @staticmethod
    def _drain(reader, path, remaining):
        with os.fdopen(reader, 'rb', buffering=0) as source, path.open('wb') as target:
            while chunk := source.read(65536):
                if remaining:
                    kept = chunk[:remaining]
                    target.write(kept)
                    target.flush()
                    remaining -= len(kept)

    def fileno(self):
        return self.writer

    def close(self):
        if self.writer is not None:
            os.close(self.writer)
            self.writer = None
        self.thread.join(timeout=2)
