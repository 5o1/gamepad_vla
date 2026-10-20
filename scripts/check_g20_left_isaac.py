"""Convert the station G20 left URDF to USD and check mimic coupling in Isaac Lab.

Only the 16 independent joints are PD-driven; the 5 mimic joints have no drive
so the PhysX mimic constraint alone must move them. This is a kinematic and
constraint check with placeholder gains, not actuator identification or a
contact result.

Usage (Isaac Lab env, free GPU):
    CUDA_VISIBLE_DEVICES=3 python scripts/check_g20_left_isaac.py \
        --urdf <.../g20_left/linkerhand_g20_left.urdf> --usd-dir <dir> --report <json>
"""

import argparse
import json
from pathlib import Path

from isaaclab.app import AppLauncher

parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
parser.add_argument("--urdf", type=Path, required=True)
parser.add_argument("--usd-dir", type=Path, required=True)
parser.add_argument("--report", type=Path, required=True)
parser.add_argument("--targets", type=int, default=20)
parser.add_argument("--settle-steps", type=int, default=240)
parser.add_argument("--seed", type=int, default=0)
AppLauncher.add_app_launcher_args(parser)
args = parser.parse_args()
args.headless = True
app = AppLauncher(args).app

import torch  # noqa: E402

import isaaclab.sim as sim_utils  # noqa: E402
from isaaclab.actuators import ImplicitActuatorCfg  # noqa: E402
from isaaclab.assets import Articulation, ArticulationCfg  # noqa: E402

from gamepad_vla.sim import g20  # noqa: E402


def main():
    table = g20.urdf_joint_table(args.urdf)
    usd_path = g20.convert_urdf(args.urdf, args.usd_dir)
    repaired = g20.repair_mimic_constraints(usd_path, table)
    independent = [name for name in g20.G20_JOINT_NAMES if name not in g20.G20_MIMIC_JOINT_NAMES]

    sim = sim_utils.SimulationContext(sim_utils.SimulationCfg(dt=1 / 240, device="cuda:0"))
    hand = Articulation(ArticulationCfg(
        prim_path="/World/Hand",
        spawn=sim_utils.UsdFileCfg(
            usd_path=str(usd_path),
            rigid_props=sim_utils.RigidBodyPropertiesCfg(disable_gravity=True),
            articulation_props=sim_utils.ArticulationRootPropertiesCfg(
                fix_root_link=True, enabled_self_collisions=False,
                solver_position_iteration_count=8, solver_velocity_iteration_count=2),
        ),
        init_state=ArticulationCfg.InitialStateCfg(joint_pos={".*": 0.0}),
        actuators={
            # Placeholder gains from the station project (stiffness 240, damping 8).
            "driven": ImplicitActuatorCfg(joint_names_expr=independent, stiffness=240.0,
                                          damping=8.0, effort_limit_sim=60.0,
                                          velocity_limit_sim=12.0),
            "mimic": ImplicitActuatorCfg(joint_names_expr=list(g20.G20_MIMIC_JOINT_NAMES),
                                         stiffness=0.0, damping=0.0),
        },
    ))
    sim.reset()
    names = hand.joint_names
    index = {name: i for i, name in enumerate(names)}
    lower = torch.tensor([table[n][0] for n in names], device=sim.device)
    upper = torch.tensor([table[n][1] for n in names], device=sim.device)
    generator = torch.Generator(device="cpu").manual_seed(args.seed)

    mimic_error = {name: 0.0 for name in g20.G20_MIMIC_JOINT_NAMES}
    tracking_error, limit_violation = 0.0, 0.0
    for _ in range(args.targets):
        fraction = torch.rand(len(names), generator=generator).to(sim.device)
        target = lower + fraction * (upper - lower)
        hand.set_joint_position_target(target.unsqueeze(0))
        for _ in range(args.settle_steps):
            hand.write_data_to_sim()
            sim.step()
            hand.update(sim.get_physics_dt())
        q = hand.data.joint_pos[0]
        for name in g20.G20_MIMIC_JOINT_NAMES:
            _, _, source, multiplier = table[name]
            mimic_error[name] = max(mimic_error[name],
                                    abs(float(q[index[name]] - multiplier * q[index[source]])))
        driven = [index[n] for n in independent]
        tracking_error = max(tracking_error, float((q[driven] - target[driven]).abs().max()))
        limit_violation = max(limit_violation, float(torch.clamp(lower - q, min=0).max()),
                              float(torch.clamp(q - upper, min=0).max()))

    report = dict(
        urdf=str(args.urdf),
        usd=str(usd_path),
        mimic_constraints_repaired=repaired,
        joint_order=names,
        targets=args.targets,
        settle_steps=args.settle_steps,
        physics_dt=1 / 240,
        max_mimic_error_rad=mimic_error,
        max_driven_tracking_error_rad=tracking_error,
        max_limit_violation_rad=limit_violation,
        scope="No contact; gravity off; placeholder PD gains (240/8); not hardware-identified.",
    )
    args.report.parent.mkdir(parents=True, exist_ok=True)
    args.report.write_text(json.dumps(report, indent=2))
    print(json.dumps(report, indent=2))


main()
app.close()
