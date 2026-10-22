"""Collection session lifecycle shared by CPU simulators and operator controls."""
from concurrent.futures import ProcessPoolExecutor
import multiprocessing as mp
from dataclasses import asdict
import importlib.metadata
import json
from pathlib import Path
import sys
import time

import numpy as np

from gamepad_vla.workstation.collection_integrity import check_and_store, pending_integrity, read_integrity, store_integrity, episode_id
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
        gamepad_collisions = getattr(backend, "gamepad_collisions", backend.environment_collisions)
        name = ("full" if backend.environment_collisions else
                "gamepad-contact" if gamepad_collisions else "free-space")
        scene["collection_variant"] = dict(name=name,
            environment_collisions=backend.environment_collisions, gamepad_collisions=gamepad_collisions,
            robot_self_collision="backend_existing",
            definition="free-space: table, gamepad, fixtures have no collision; gamepad-contact: only the "
                       "gamepad collides with robots; robot model, mount, drives unchanged")
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
        self.recording_state = "idle"
        self.last_saved = None
        self.on_status = lambda: None
        self.save_pool = self.save_future = None
        self.control_log = (self.output / "events.jsonl").open("x", buffering=1)

    def control(self, event):
        action = event["action"]
        self.control_log.write(json.dumps(dict(event, wall_time_ns=time.time_ns()), ensure_ascii=False) + "\n")
        if action == "start" and self.writer is None and self.recording_state not in {"saving","checking"}:
            self.writer = EpisodeWriter(self.output / f"episode-{self.episode_index:06d}",
                                        dict(self.meta, recording_start_sim_time_s=self.last_sim_time_s,
                                             started_by=event.get('client','cli'), stopped_by=None,
                                             forced=bool(event.get('force',False)),
                                             recording_start_health=event.get('health_snapshot',{})))
            self.episode_index += 1
            self.recording_state = "recording"
            self.current_integrity = pending_integrity(self.writer.path,"recording")
            for note in self.pending_notes:
                self.writer.note(note)
            self.pending_notes.clear()
        elif action == "stop" and self.writer is not None:
            self.writer.meta["stopped_by"] = event.get("client", "cli")
            self.finish()
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
        self.update_save()
        self.last_sim_time_s = observation.sim_time_s
        for c in commands:
            kind = "hand" if isinstance(c, HandCommand) else "arm"
            self.commands[c.side + "_" + kind] = asdict(c)
        self.input_sequences.update(input_sequences or {})
        overlay = overlay_state(observation)
        if self.writer:
            self.writer.append(observation, self.commands, overlay, applied_monotonic_ns=applied_monotonic_ns,
                               input_sequences=self.input_sequences, arm_clamps=self.arm_clamps())
        return dict(observation.to_dict(), commands=dict(self.commands), overlay=overlay,
                    recording=self.writer is not None, arm_clamps=self.arm_clamps(), recording_status=self.status(), applied_monotonic_ns=applied_monotonic_ns,
                    input_sequences=dict(self.input_sequences))

    def arm_clamps(self):
        return {side: None if self.commands[side+'_arm'] is None else self.commands[side+'_arm'].get('clamp_flags')
                for side in SIDES}

    def status(self):
        self.update_save()
        writer = self.writer
        return dict(state=self.recording_state, integrity=getattr(self,"current_integrity",None),
                    episode=writer.path.name if writer else (self.last_saved or {}).get('episode'),
                    rows=writer.meta['rows'] if writer else (self.last_saved or {}).get('rows', 0),
                    notes=writer.meta.get('operator_notes',[]) if writer else (self.last_saved or {}).get('notes',[]),
                    duration_s=max(0., self.last_sim_time_s-writer.meta['recording_start_sim_time_s']) if writer else
                               (self.last_saved or {}).get('duration_s', 0.), last_saved=self.last_saved)

    def finish(self, status='complete'):
        writer = self.writer
        self.recording_state = 'saving'
        self.current_integrity = pending_integrity(writer.path,'saving')
        self.on_status()
        if writer.meta.get('stopped_by') is None:
            writer.meta['stopped_by'] = 'session_end'
        writer.meta['recording_end_sim_time_s'] = self.last_sim_time_s
        writer.close(status)
        self.last_saved = dict(episode=writer.path.name, episode_id=episode_id(writer.path), rows=writer.meta['rows'], notes=writer.meta.get('operator_notes',[]),
                               duration_s=self.last_sim_time_s-writer.meta['recording_start_sim_time_s'], integrity=self.current_integrity)
        self.integrity_path=writer.path
        store_integrity(writer.path,self.current_integrity)
        self.writer = None
        if getattr(self,'save_pool',None) is None:
            self.save_pool = ProcessPoolExecutor(max_workers=1,mp_context=mp.get_context('spawn'))
        self.save_future = self.save_pool.submit(check_and_store,writer.path)
        self.on_status()

    def update_save(self):
        future=getattr(self,'save_future',None)
        if future is None:return
        if future.done():
            try:result=future.result()
            except Exception as exc:result=dict(self.current_integrity,state='failed',passed=False,errors=[str(exc)])
            if result.get('episode_id') != self.current_integrity['episode_id']:
                result=dict(self.current_integrity,state='failed',passed=False,errors=['integrity episode mismatch'])
            self.save_future=None
            self.current_integrity=result
            self.last_saved=dict(self.last_saved,integrity=result)
            self.recording_state='saved' if result['state']=='passed' else 'failed'
        elif time.monotonic()-getattr(self,'last_integrity_poll',0)>=.05:
            self.last_integrity_poll=time.monotonic()
            self.current_integrity=read_integrity(self.integrity_path)
            self.last_saved=dict(self.last_saved,integrity=self.current_integrity)
            if self.current_integrity['state']=='running':self.recording_state='checking'

    def wait_saved(self):
        future=getattr(self,'save_future',None)
        if future is not None:
            try:future.result()
            except Exception:pass
            self.update_save()
        if getattr(self,'save_pool',None) is not None:
            self.save_pool.shutdown(wait=True)
            self.save_pool=None
        self.on_status()

    def close(self, status="complete"):
        if self.writer:
            self.finish(status)
        self.wait_saved()
        self.control_log.close()
