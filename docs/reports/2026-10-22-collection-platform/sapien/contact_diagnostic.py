"""Inspect the first 0.1 seconds of unchanged replay, without rendering."""
import argparse
import json
import math
from pathlib import Path

import numpy as np

from gamepad_vla.workstation.collection_sapien import CollectionSapienBackend
from gamepad_vla.workstation.collection_sources import CollectionReplay
from gamepad_vla.workstation.spec import load_spec


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--replay", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=True)
    backend = CollectionSapienBackend(load_spec("configs/workcell/station_g20_v4_4.json"),
                                     work_dir=args.output / "work", render=False)
    source = CollectionReplay(args.replay)
    pairs, intervals = {}, []
    for index in range(3):
        backend.apply(source.poll(index / 30))
        end_tick = math.floor((index + 1) / 30 / .001 + 1e-9)
        found_points, found_penetrating, found_impulse = False, False, False
        backend_detected = False
        while backend._ticks < end_tick:
            backend.step(.001)
            backend_detected |= backend._has_robot_contact()
            for contact in backend.scene.get_contacts():
                if not any(body.entity in backend._robot_entities for body in contact.bodies):
                    continue
                if not contact.points:
                    continue
                pair = " | ".join(body.entity.name for body in contact.bodies)
                values = pairs.setdefault(pair, [])
                for point in contact.points:
                    separation = float(point.separation)
                    impulse = float(np.linalg.norm(point.impulse))
                    values.append((separation, impulse))
                    found_points = True
                    found_penetrating |= separation <= 0
                    found_impulse |= impulse > 0
        intervals.append(dict(sim_time_s=backend._ticks * .001,
                              any_contact_points=found_points,
                              any_separation_nonpositive=found_penetrating,
                              any_nonzero_impulse=found_impulse,
                              backend_detected=backend_detected))
    result = dict(duration_s=backend._ticks * .001, input_frames=3,
                  old_contact_intervals=sum(r["any_contact_points"] for r in intervals),
                  nonpositive_separation_intervals=sum(r["any_separation_nonpositive"] for r in intervals),
                  nonzero_impulse_intervals=sum(r["any_nonzero_impulse"] for r in intervals),
                  backend_matches_geometric_rule=all(r["backend_detected"] == r["any_separation_nonpositive"]
                                                     for r in intervals),
                  intervals=intervals, pairs={})
    for name, values in sorted(pairs.items()):
        a = np.asarray(values)
        result["pairs"][name] = dict(points=len(a), separation_min_m=float(a[:, 0].min()),
                                     separation_max_m=float(a[:, 0].max()),
                                     nonpositive_points=int((a[:, 0] <= 0).sum()),
                                     nonzero_impulse_points=int((a[:, 1] > 0).sum()),
                                     impulse_max_Ns=float(a[:, 1].max()))
    (args.output / "contact_diagnostic.json").write_text(json.dumps(result, indent=2) + "\n")
    print(json.dumps(result, indent=2), flush=True)
    backend.close()


if __name__ == "__main__":
    main()
