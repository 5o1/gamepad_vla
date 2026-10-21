"""Write numeric LeRobot v3.0 with PyArrow; no lerobot or simulator dependency."""
import json
from pathlib import Path
import shutil
import subprocess

import numpy as np

from gamepad_vla.workstation.episode_schema import read_episode, SIDES
from gamepad_vla.workstation.g20_mapping import G20_JOINT_NAMES


def export_episode(source, destination, *, task="reference teleoperation", video=None):
    import pyarrow as pa
    import pyarrow.parquet as pq
    meta, rows = read_episode(source)
    if not rows:
        raise ValueError("cannot export an empty episode")
    root = Path(destination)
    root.mkdir(parents=True, exist_ok=False)
    for folder in ("meta/episodes/chunk-000", "data/chunk-000"):
        (root / folder).mkdir(parents=True)
    n, fps = len(rows), int(meta["fps"])
    state = np.array([np.r_[np.asarray(r["arm_q"]).ravel(), np.asarray(r["hand_q"]).ravel()] for r in rows], np.float32)
    actions = []
    for r in rows:
        c = r["commands"]
        if any(c[s + "_arm"] is None or c[s + "_hand"] is None or
               c[s + "_hand"]["motor_positions"] is None for s in SIDES):
            raise ValueError("54D action export needs known slave arm targets and original hand motors on both sides")
        actions.append(np.concatenate([c[s + "_arm"]["positions"] for s in SIDES] +
                                      [c[s + "_hand"]["motor_positions"] for s in SIDES]))
    arrays = {"observation.state": state, "action": np.asarray(actions, np.float32),
              "observation.gamepad_signal": np.array([r["gamepad_signal"] for r in rows], np.float32),
              "observation.gamepad_mask": np.array([r["gamepad_mask"] for r in rows], bool),
              "observation.world_poses": np.array([np.r_[np.asarray(r["palm_pose_world"]).ravel(),
                                                            np.asarray(r["fingertip_pose_world"]).ravel()] for r in rows], np.float32),
              "observation.sim_time_s": np.array([r["sim_time_s"] for r in rows], np.float64),
              "observation.wall_time_ns": np.array([r["wall_time_ns"] for r in rows], np.int64),
              "timestamp": np.arange(n, dtype=np.float32) / fps,
              "frame_index": np.arange(n, dtype=np.int64), "episode_index": np.zeros(n, np.int64),
              "index": np.arange(n, dtype=np.int64), "task_index": np.zeros(n, np.int64)}
    features, columns, stats = {}, {}, {}
    state_names = [f"{s}_arm_joint_{j}" for s in SIDES for j in range(1, 8)] + [f"{s}_{j}" for s in SIDES for j in G20_JOINT_NAMES]
    action_names = state_names[:14] + [f"{s}_hand_motor_{j}" for s in SIDES for j in range(20)]
    for name, a in arrays.items():
        dtype = str(a.dtype)
        width = a.shape[1] if a.ndim > 1 else 1
        names = state_names if name == "observation.state" else action_names if name == "action" else None
        features[name] = dict(dtype=dtype, shape=[width], names=names)
        if a.dtype != bool and not np.isfinite(a).all():
            raise ValueError(f"nonfinite feature {name}")
        columns[name] = pa.array(a.tolist(), type=pa.list_(pa.from_numpy_dtype(a.dtype), width)) if a.ndim > 1 else pa.array(a)
        if a.dtype != bool:
            b = a.reshape(n, -1)
            stats[name] = dict(min=b.min(0).tolist(), max=b.max(0).tolist(), mean=b.mean(0).tolist(),
                               std=b.std(0).tolist(), count=[n])
    episode_meta = dict(episode_index=[0], tasks=[[task]], length=[n],
                        dataset_from_index=[0], dataset_to_index=[n],
                        **{"data/chunk_index": [0], "data/file_index": [0],
                           "meta/episodes/chunk_index": [0], "meta/episodes/file_index": [0]})
    if video:
        probe = json.loads(subprocess.check_output(["ffprobe", "-v", "error", "-count_frames", "-select_streams", "v:0",
                                                    "-show_streams", "-of", "json", str(video)], text=True))["streams"][0]
        num, den = map(int, probe["r_frame_rate"].split("/"))
        if int(probe["nb_read_frames"]) != n or not np.isclose(num / den, fps):
            raise ValueError("optional video must have exactly one aligned frame per row and match fps")
        key = "observation.images.operator"
        path = root / f"videos/{key}/chunk-000/file-000.mp4"
        path.parent.mkdir(parents=True)
        shutil.copyfile(video, path)
        features[key] = dict(dtype="video", shape=[probe["height"], probe["width"], 3], names=["height", "width", "channels"],
                             info={"video.height": probe["height"], "video.width": probe["width"], "video.fps": fps,
                                   "video.codec": probe["codec_name"], "video.pix_fmt": probe["pix_fmt"],
                                   "video.is_depth_map": False, "has_audio": False, "video.channels": 3})
        for k, v in {"chunk_index": 0, "file_index": 0, "from_timestamp": 0., "to_timestamp": n / fps}.items():
            episode_meta[f"videos/{key}/{k}"] = [v]
    pq.write_table(pa.table(columns), root / "data/chunk-000/file-000.parquet")
    pq.write_table(pa.table(episode_meta), root / "meta/episodes/chunk-000/file-000.parquet")
    # Preserve the pandas index semantics used by LeRobot's task lookup, without
    # requiring pandas in data-tools.
    tasks = pa.table({"task_index": [0], "__index_level_0__": [task]})
    pandas_metadata = {"index_columns": ["__index_level_0__"], "column_indexes": [],
                       "columns": [{"name": "task_index", "field_name": "task_index", "pandas_type": "int64", "numpy_type": "int64", "metadata": None},
                                   {"name": None, "field_name": "__index_level_0__", "pandas_type": "unicode", "numpy_type": "object", "metadata": None}],
                       "creator": {"library": "pyarrow", "version": pa.__version__}, "pandas_version": "2.0.0"}
    tasks = tasks.replace_schema_metadata({b"pandas": json.dumps(pandas_metadata).encode()})
    pq.write_table(tasks, root / "meta/tasks.parquet")
    info = dict(codebase_version="v3.0", robot_type="ar5_dual_l25_collection", fps=fps,
                total_episodes=1, total_frames=n, total_tasks=1, chunks_size=1000,
                data_files_size_in_mb=100, video_files_size_in_mb=200, splits={"train": "0:1"},
                data_path="data/chunk-{chunk_index:03d}/file-{file_index:03d}.parquet",
                video_path="videos/{video_key}/chunk-{chunk_index:03d}/file-{file_index:03d}.mp4", features=features)
    (root / "meta/info.json").write_text(json.dumps(info, indent=2) + "\n")
    (root / "meta/stats.json").write_text(json.dumps(stats) + "\n")
    (root / "meta/collection.json").write_text(json.dumps(dict(episode=meta,
        state_units=["rad"] * 56, action_units=["rad"] * 14 + ["motor_0_255"] * 40,
        timestamp_semantics="uniform collection grid; actual engine and Unix clocks are separate features"), indent=2) + "\n")
    # Parquet roundtrip here; native LeRobot validation is a different environment.
    loaded = pq.read_table(root / "data/chunk-000/file-000.parquet")
    for name, a in arrays.items():
        if not np.array_equal(np.asarray(loaded[name].to_pylist()), a):
            raise ValueError(f"Parquet roundtrip differs: {name}")
    return dict(rows=n, features={k: v["shape"] for k, v in features.items()}, codebase_version="v3.0")
