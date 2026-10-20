#!/usr/bin/env python3
"""Convert one copied station episode with the isolated data-tools environment."""
import argparse
import hashlib
import json
from datetime import datetime, timezone
from pathlib import Path

import numpy as np
import pyarrow.parquet as pq


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("episode", type=Path)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    files = sorted((args.episode / "data").rglob("*.parquet"))
    tables = [pq.read_table(path) for path in files]
    import pyarrow as pa
    table = pa.concat_tables(tables)
    schema = json.loads((args.episode / "meta/workstation_schema.json").read_text())
    info = json.loads((args.episode / "meta/info.json").read_text())
    columns = table.to_pydict()
    arrays = {name: np.asarray(columns[name]) for name in
              ("timestamp", "frame_index", "episode_index", "observation.state", "action")}
    if arrays["observation.state"].shape != (table.num_rows, 54):
        raise ValueError("expected 54-dimensional measured state")
    if arrays["action"].shape != (table.num_rows, 54):
        raise ValueError("expected 54-dimensional action")
    args.output.mkdir(parents=True, exist_ok=True)
    np.savez(args.output / "states.npz", **arrays,
             names=np.asarray(schema["names"]), units=np.asarray(schema["units"]))
    metadata = dict(source_episode=str(args.episode), source_files=[str(p) for p in files],
                    parquet_schema=str(table.schema), workstation_schema=schema,
                    state_feature=info["features"]["observation.state"],
                    rows=table.num_rows,
                    exported_at=datetime.now(timezone.utc).isoformat(),
                    state_min=arrays["observation.state"].min(axis=0).tolist(),
                    state_max=arrays["observation.state"].max(axis=0).tolist())
    (args.output / "states_metadata.json").write_text(json.dumps(metadata, indent=2))
    print(json.dumps(metadata, indent=2))


if __name__ == "__main__":
    main()
