"""CPU-only load/drive smoke; this does not implement the L1/L2 gates."""
import argparse
import json
from pathlib import Path
import time

import numpy as np

from gamepad_vla.workstation.collection_sapien import CollectionSapienBackend
from gamepad_vla.workstation.commands import ArmCommand, HandCommand
from gamepad_vla.workstation.g20_mapping import G20_JOINT_NAMES
from gamepad_vla.workstation.spec import load_spec


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--spec", default="configs/workcell/station_g20_v4_4.json")
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--no-environment-collisions", action="store_true")
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=True)
    start = time.perf_counter()
    spec = load_spec(args.spec)
    def robot_signature(backend):
        return {side: {link.name: [list(shape.get_collision_groups())
                                  for shape in link.get_collision_shapes()]
                       for link in backend.robots[side].get_links()}
                for side in ("left", "right")}
    baseline_signature = None
    if args.no_environment_collisions:
        baseline = CollectionSapienBackend(spec, work_dir=args.output / "baseline_work", render=False)
        baseline_signature = robot_signature(baseline)
        baseline.close()
        start = time.perf_counter()
    backend = CollectionSapienBackend(spec, work_dir=args.output / "work", render=False,
                                     environment_collisions=not args.no_environment_collisions)
    loaded = time.perf_counter()
    first = backend.observe()
    commands = []
    for index, side in enumerate(("left", "right")):
        hand = first.hand_q[index].copy()
        arm = first.arm_q[index].copy()
        hand[3] += .03
        arm[6] += .02
        commands.extend((HandCommand(side, dict(zip(G20_JOINT_NAMES, hand)), 0.),
                         ArmCommand(side, tuple(arm), 0.)))
    backend.apply(commands)
    contact_rows = 0
    for _ in range(50):
        backend.step(.02)
        last = backend.observe()
        contact_rows += last.robot_contact
    result = dict(
        load_wall_s=loaded - start, drive_wall_s=time.perf_counter() - loaded,
        simulation_s=last.sim_time_s, steps=50, input_dt_s=.02,
        physics_dt_s=.001, schema_valid=True,
        hand_change_rad=(last.hand_q - first.hand_q).tolist(),
        arm_change_rad=(last.arm_q - first.arm_q).tolist(),
        contact_rows=contact_rows,
        render_components=sum(type(c).__name__.startswith("Render")
                              for entity in backend.scene.get_entities()
                              for c in entity.components),
        tip_offsets_m=backend.tip_offsets,
        finite=bool(np.isfinite(last.hand_q).all() and np.isfinite(last.arm_q).all()),
        environment_collisions=backend.environment_collisions,
        robot_collision_shape_counts={s: sum(len(shapes) for shapes in links.values())
                                      for s, links in robot_signature(backend).items()},
        robot_collision_shapes_and_groups_match_baseline=(None if baseline_signature is None else
                                                          baseline_signature == robot_signature(backend)),
        fixed_environment_collision_shape_count=sum(len(c.get_collision_shapes())
                                                   for actor in backend._fixed_actors
                                                   for c in actor.components
                                                   if hasattr(c, "get_collision_shapes")),
        gamepad_collision_groups=[list(shape.get_collision_groups())
                                  for link in backend.gamepad.articulation.get_links()
                                  for shape in link.get_collision_shapes()],
    )
    (args.output / "smoke.json").write_text(json.dumps(result, indent=2) + "\n")
    print(json.dumps(result, indent=2), flush=True)
    backend.close()


if __name__ == "__main__":
    main()
