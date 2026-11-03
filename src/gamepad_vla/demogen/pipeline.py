"""Reusable orchestration, with a manifest for every attempted generation."""
from dataclasses import asdict
import hashlib
import importlib.util
import json
from pathlib import Path

from gamepad_vla.provenance import code_provenance
from gamepad_vla.site import repository_root
from gamepad_vla.workstation.spec import load_spec, spec_to_dict
from .episode import load_demonstration
from .generate import GenerateConfig, GenerationFailure, generate
from .output import write_episode
from .segments import SliceConfig, slice_stream
from .validate import ValidationConfig, initialized_spec, replay


def dump_json(path, value):
    Path(path).write_text(json.dumps(value, ensure_ascii=False, indent=2, allow_nan=False)+"\n")


def available_backend():
    for name in ("sapien", "mujoco"):
        if importlib.util.find_spec(name) is not None:
            return name
    raise GenerationFailure("no existing SAPIEN or MuJoCo environment; no dependency installed")


def build_library(episodes, side, config=None):
    streams, library = [], []
    for path in episodes:
        stream = load_demonstration(path)[side]
        streams.append(stream)
        library.extend(slice_stream(stream, config))
    return streams, library


def run(episodes, target_specs, sequence, output, *, side="left", count=1, seed=0,
        backend="auto", slicing=None, generation=None, validation=None, resource_root=None):
    if count < 1 or not target_specs:
        raise ValueError("need at least one attempt and target spec")
    slicing, generation, validation = slicing or SliceConfig(), generation or GenerateConfig(), validation or ValidationConfig()
    output = Path(output)
    output.mkdir(parents=True, exist_ok=False)
    manifest = dict(format="gamepad-vla-demogen-v1", code=code_provenance(repository_root()),
                    seed=int(seed), sequence=list(sequence), side=side,
                    configs=dict(slicing=asdict(slicing), generation=asdict(generation), validation=asdict(validation)),
                    sources=[], attempts=[], accepted=0, rejected=0,
                    data_format="collection_episode_v4", status="running")
    dump_json(output / "manifest.json", manifest)
    try:
        streams, library = build_library(episodes, side, slicing)
        for stream in streams:
            path = Path(stream.source)
            manifest["sources"].append(dict(episode=stream.source, rows=len(stream.time),
                synthetic=bool(stream.meta.get("synthetic", False)),
                scene_sha256=stream.meta["scene_spec_sha256"],
                files_sha256={n: hashlib.sha256((path / n).read_bytes()).hexdigest()
                              for n in ("meta.json", "steps.jsonl", "commands.jsonl")}))
        dump_json(output / "segments.json", [s.to_dict() for s in library])
        for attempt in range(count):
            # Round-robin placement and source selection is deliberate and stable.
            # Per-attempt RNG handles all segment and augmentation choices.
            stream = streams[attempt % len(streams)]
            spec_path = Path(target_specs[attempt % len(target_specs)])
            entry = dict(attempt=attempt, seed=int(seed+attempt), target_spec=str(spec_path),
                         status="rejected", stage="generation", reasons=[])
            try:
                spec = load_spec(spec_path)
                entry["target_scene_sha256"] = hashlib.sha256(json.dumps(spec_to_dict(spec), sort_keys=True).encode()).hexdigest()
                entry["target_spec_sha256"] = hashlib.sha256(spec_path.read_bytes()).hexdigest()
                trajectory = generate(stream, library, spec, sequence, seed=seed+attempt,
                                      config=generation, source_streams=streams)
                entry["transforms"] = trajectory.provenance
                entry["stage"] = "physics"
                selected_backend = available_backend() if backend == "auto" else backend
                result, observations, commands = replay(trajectory, backend_name=selected_backend,
                                                        config=validation, resource_root=resource_root)
                entry["validation"] = result
                entry["reasons"] = result["reasons"]
                if result["passed"]:
                    entry["stage"] = "output"
                    name = f"episode-{manifest['accepted']:06d}"
                    temporary = output / ("."+name+".pending")
                    integrity = write_episode(temporary, observations, commands,
                        spec=initialized_spec(trajectory), fps=trajectory.fps, tip_offsets=trajectory.tip_offsets,
                        metadata=dict(synthetic=trajectory.provenance["source_synthetic"], generated=True,
                                      physics_validated=True,
                                      demogen=trajectory.provenance, validation=result, code=manifest["code"],
                                      physics_timestep_s=result["physics_timestep_s"]))
                    destination = output / name
                    temporary.rename(destination)
                    # Rebind the sidecar after the atomic directory promotion.
                    from gamepad_vla.workstation.collection_integrity import check_and_store
                    integrity = check_and_store(destination)
                    entry.update(status="accepted", episode=name, integrity=integrity)
                    manifest["accepted"] += 1
            except GenerationFailure as exc:
                entry["reasons"] = [str(exc)]
                entry["details"] = exc.details
            except (ImportError, OSError, ValueError, RuntimeError, FloatingPointError) as exc:
                entry["reasons"] = [type(exc).__name__+": "+str(exc)]
            if entry["status"] != "accepted":
                manifest["rejected"] += 1
            manifest["attempts"].append(entry)
            dump_json(output / "manifest.json", manifest)
        manifest["status"] = "complete"
    except BaseException as exc:
        manifest.update(status="failed", error=type(exc).__name__+": "+str(exc))
        raise
    finally:
        dump_json(output / "manifest.json", manifest)
    return manifest
