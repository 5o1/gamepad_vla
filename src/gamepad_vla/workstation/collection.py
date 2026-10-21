"""Collection session lifecycle shared by CPU simulators and operator controls."""
from dataclasses import asdict
import importlib.metadata
import json
from pathlib import Path
import sys
import time

import numpy as np

from gamepad_vla.workstation.commands import ArmCommand, HandCommand
from gamepad_vla.workstation.episode_schema import EpisodeWriter, SIDES, fingertip_offsets
from gamepad_vla.workstation.arm_mapping import VERSION as ARM_MAPPING_VERSION
from gamepad_vla.workstation.spec import spec_to_dict


def create_backend(name, spec, **kwargs):
    if name == "sapien":
        from gamepad_vla.workstation.collection_sapien import CollectionSapienBackend
        return CollectionSapienBackend(spec, **kwargs)
    if name == "mujoco":
        from gamepad_vla.workstation.collection_mujoco import CollectionMujocoBackend
        return CollectionMujocoBackend(spec, **kwargs)
    raise ValueError("collection backend must be sapien or mujoco")


def overlay_state(obs):
    return dict(layout_id="sdl3-gamepad-core-v1", device_id=1,
                values=np.asarray(obs.gamepad_signal).tolist(),
                available_mask=np.asarray(obs.gamepad_mask, bool).tolist(),
                task_mask=[False] * 32, source="post_physics_contact", sim_time_s=obs.sim_time_s)


class Session:
    def __init__(self, output, spec, backend, source, *, mapper=None, fps=30):
        self.output = Path(output)
        self.output.mkdir(parents=True, exist_ok=False)
        scene = spec_to_dict(spec)
        scene["collection_variant"] = dict(name="full" if backend.environment_collisions else "free-space",
            environment_collisions=backend.environment_collisions, robot_self_collision="backend_existing",
            definition="table, gamepad, fixtures have no collision in free-space; robot model, mount, drives unchanged")
        self.meta = dict(scene_spec_text=json.dumps(scene, sort_keys=True),
                         scene_variant=scene["collection_variant"],
                         backend_name=backend.name, backend_version=importlib.metadata.version(backend.name),
                         asset_versions={"gamepad": importlib.metadata.version("xbox-gamepad-physics"),
                                         "robot": "v4.4-AR5-0.8R-L25-right"},
                         mapping_versions={"arm": ARM_MAPPING_VERSION, "hand": "station-g20-20261014"},
                         arm_mapping=None if mapper is None else mapper.metadata(),
                         input_source=source, fps=fps, operator_notes=[],
                         right_hand_mount_status="assumed", fingertip_offsets_m=fingertip_offsets(spec),
                         fingertip_definition="distal orientation; local [0,0,max_z(collision_mesh)]",
                         units={"hand_q": "rad", "arm_q": "rad", "hand_motor": "motor_0_255",
                                "raw_master": "deg", "slave_target": "rad", "pose_position": "m",
                                "control_joints": "hinge:rad;slide:m", "wall_time_ns": "Unix epoch ns"})
        module = sys.modules[type(backend).__module__]
        self.meta["controller_placeholders"] = {kind: dict(getattr(module, kind)) for kind in ("HAND_GAINS", "ARM_GAINS")}
        self.meta["physics_timestep_s"] = (float(backend.model.opt.timestep) if backend.name == "mujoco"
                                           else float(backend.scene.get_timestep()))
        self.input_sequences = {}
        self.commands = {f"{side}_{kind}": None for side in SIDES for kind in ("hand", "arm")}
        self.writer = None
        self.last_sim_time_s = 0.
        self.episode_index = 0
        self.pending_notes = []
        self.control_log = (self.output / "events.jsonl").open("x", buffering=1)

    def control(self, event):
        action = event["action"]
        self.control_log.write(json.dumps(dict(event, wall_time_ns=time.time_ns()), ensure_ascii=False) + "\n")
        if action == "start" and self.writer is None:
            self.writer = EpisodeWriter(self.output / f"episode-{self.episode_index:06d}",
                                        dict(self.meta, recording_start_sim_time_s=self.last_sim_time_s))
            self.episode_index += 1
            for note in self.pending_notes:
                self.writer.note(note)
            self.pending_notes.clear()
        elif action == "stop" and self.writer is not None:
            self.writer.close()
            self.writer = None
        elif action == "note":
            if self.writer:
                self.writer.note(event.get("text", ""))
            else:
                self.pending_notes.append(event.get("text", ""))
        elif action not in {"start", "stop", "note"}:
            raise ValueError("unknown recording event")

    def record_inputs(self, events):
        if self.writer:
            self.writer.append_inputs(events)

    def record(self, observation, commands, *, applied_monotonic_ns=None, input_sequences=None):
        self.last_sim_time_s = observation.sim_time_s
        for c in commands:
            kind = "hand" if isinstance(c, HandCommand) else "arm"
            self.commands[c.side + "_" + kind] = asdict(c)
        self.input_sequences.update(input_sequences or {})
        overlay = overlay_state(observation)
        if self.writer:
            self.writer.append(observation, self.commands, overlay, applied_monotonic_ns=applied_monotonic_ns,
                               input_sequences=self.input_sequences)
        return dict(observation.to_dict(), commands=dict(self.commands), overlay=overlay,
                    recording=self.writer is not None, applied_monotonic_ns=applied_monotonic_ns,
                    input_sequences=dict(self.input_sequences))

    def close(self, status="complete"):
        if self.writer:
            self.writer.close(status)
            self.writer = None
        self.control_log.close()
