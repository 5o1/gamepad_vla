"""Existing v4 writer, and data-root selection from private site settings."""
import json
from pathlib import Path

from gamepad_vla.site import site_value, workspace_root
from gamepad_vla.workstation.collection import overlay_state
from gamepad_vla.workstation.collection_integrity import check_and_store
from gamepad_vla.workstation.episode_schema import EpisodeWriter
from gamepad_vla.workstation.spec import spec_to_dict


def data_directory(name):
    root = Path(site_value("station.data_root") or workspace_root() / "data").expanduser().resolve()
    path = (root / "demogen" / name).resolve()
    if not path.is_relative_to(root / "demogen") or path == root / "demogen":
        raise ValueError("output name must stay below the configured data directory's demogen/")
    return path


def write_episode(path, observations, commands, *, spec, fps, tip_offsets, metadata):
    """Serialize measured rows; targets live only in commands and metadata."""
    if not observations or len(observations) != len(commands):
        raise ValueError("episode requires equally sized observations and commands")
    meta = dict(metadata, fps=fps, scene_spec_text=json.dumps(spec_to_dict(spec), sort_keys=True),
                fingertip_offsets_m=tip_offsets, received_input_messages=0,
                receive_start_sequence=0, receive_end_sequence=0,
                recording_start_sim_time_s=observations[0].sim_time_s-1/fps,
                recording_end_sim_time_s=observations[-1].sim_time_s,
                units=dict(hand_q="rad", arm_q="rad", pose_position="m"),
                input_source="demogen_joint_targets; no received hardware messages")
    writer = EpisodeWriter(path, meta)
    try:
        for observation, command in zip(observations, commands):
            overlay = overlay_state(observation)
            if metadata.get("physics_validated") is False and metadata.get("signal_source") != "post_physics_contact":
                overlay["source"] = "synthetic_scripted_label"
            writer.append(observation, command, overlay)
    except BaseException:
        writer.close("interrupted")
        raise
    writer.close()
    integrity = check_and_store(path)
    if not integrity["passed"]:
        raise ValueError("generated episode integrity failed: " + "; ".join(integrity["errors"]))
    return integrity
