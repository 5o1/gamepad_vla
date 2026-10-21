"""Dual-arm collection dynamics, separate from the frozen direction-key task.

Only robot position actuators receive commands. Native gamepad displacement,
contact, and signals are observed after MuJoCo integration, without projection.
"""
from __future__ import annotations

import json
from pathlib import Path
import tempfile
import time
import xml.etree.ElementTree as ET

import numpy as np

from gamepad_vla.tasks.fixed_gamepad import project_inertia_tensor, resolve_asset_files
from gamepad_vla.workstation.commands import ArmCommand, HandCommand
from gamepad_vla.workstation.contact import has_robot_contact
from gamepad_vla.workstation.episode_schema import (
    FINGERS, SIDES, Observation, fingertip_offsets, tip_pose,
)
from gamepad_vla.workstation.g20_mapping import G20_JOINT_NAMES, G20_MIMIC_JOINTS
from gamepad_vla.workstation.paths import resolve_workspace_path
from gamepad_vla.workstation.robot_description import compose_robot_urdf

# Same unmeasured drive placeholders as the existing SAPIEN workcell. These
# settings are local to collection and never alter the direction-key task.
HAND_GAINS = dict(kp=240., kv=8., force_limit=60., armature=1e-4)
ARM_GAINS = dict(kp=2e4, kv=2e3, force_limit=500., armature=1e-2)


def _robot_urdf(spec, path, side):
    """Compose live arm/hand joints and repair only invalid input inertias."""
    compose_robot_urdf(spec, path, side)
    tree = ET.parse(path)
    corrections = []
    for link in tree.getroot().findall("link"):
        element = link.find("inertial/inertia")
        if element is None:
            continue
        a = {k: float(v) for k, v in element.attrib.items()}
        tensor = np.array([[a['ixx'], a['ixy'], a['ixz']],
                           [a['ixy'], a['iyy'], a['iyz']],
                           [a['ixz'], a['iyz'], a['izz']]])
        projected = project_inertia_tensor(tensor)
        if not np.array_equal(tensor, projected):
            corrections.append(dict(link=link.get('name'), before=tensor.tolist(),
                                    after=projected.tolist()))
            for key, i, j in [('ixx',0,0), ('iyy',1,1), ('izz',2,2),
                              ('ixy',0,1), ('ixz',0,2), ('iyz',1,2)]:
                element.set(key, str(projected[i, j]))
    ext = ET.SubElement(tree.getroot(), 'mujoco')
    ET.SubElement(ext, 'compiler', discardvisual='false', fusestatic='false', strippath='false')
    tree.write(path)
    return corrections


class CollectionMujocoBackend:
    name = "mujoco"

    def __init__(self, spec, *, work_dir=None, render=False, resource_root=None,
                 environment_collisions=True):
        import mujoco as mj
        from xbox_gamepad_physics.mujoco import attach_to_spec
        from xbox_gamepad_physics.mujoco.numerics import apply_standalone_policy

        if spec.right_arm is None or spec.right_hand is None:
            raise ValueError('collection requires both arms and both hands')
        if not spec.gamepad.fixed_root:
            raise ValueError('collection v1 retains the fixed gamepad mount')
        self.mj, self.spec = mj, spec
        self.environment_collisions = bool(environment_collisions)
        self._temporary = tempfile.TemporaryDirectory(prefix='collection_mujoco_') if work_dir is None else None
        self.work_dir = Path(self._temporary.name if self._temporary else work_dir)
        self.work_dir.mkdir(parents=True, exist_ok=True)
        self._renderer = None
        self._render_enabled = render
        self.tip_offsets = fingertip_offsets(spec)
        resource_root = resolve_workspace_path(str(resource_root or
            'workspace://assets/xbox_gamepad_physics_mujoco_84f43e6/resources'))
        parent = mj.MjSpec()
        corrections = {}
        for side, own_bit, other_bit in [('left', 2, 4), ('right', 4, 2)]:
            arm = getattr(spec, side + '_arm')
            source = self.work_dir / (side + '_arm_hand.urdf')
            corrections[side] = _robot_urdf(spec, source, side)
            robot = mj.MjSpec.from_file(str(source))
            for body in robot.bodies:
                body.gravcomp = 1.
            for geom in robot.geoms:
                if geom.contype or geom.conaffinity:
                    # Disable same-side self-collision, keep opposite robots,
                    # the gamepad, and the unchanged fixed support in contact.
                    geom.contype, geom.conaffinity = own_bit, 1 | other_bit
            for joint in robot.joints:
                joint.armature = (ARM_GAINS if joint.name in arm.joint_names else HAND_GAINS)['armature']
            frame = parent.worldbody.add_frame(pos=arm.base_pose[:3], quat=arm.base_pose[3:])
            parent.attach(robot, prefix=side + '_', frame=frame)
        xml = ET.fromstring(parent.to_xml())
        equality = ET.SubElement(xml, 'equality')
        actuator = ET.SubElement(xml, 'actuator')
        self._hand_names, self._arm_names = {}, {}
        for side in SIDES:
            local = 'right_' if side == 'right' else ''
            hand_names = [side + '_' + local + n for n in G20_JOINT_NAMES]
            arm_names = [side + '_' + n for n in getattr(spec, side + '_arm').joint_names]
            self._hand_names[side], self._arm_names[side] = hand_names, arm_names
            for child, (source, multiplier) in G20_MIMIC_JOINTS[side].items():
                ET.SubElement(equality, 'joint', name=side + '_mimic_' + child,
                    joint1=side + '_' + local + child, joint2=side + '_' + local + source,
                    polycoef=f'0 {multiplier} 0 0 0')
            independent = [n for n in G20_JOINT_NAMES if n not in G20_MIMIC_JOINTS[side]]
            for name, gain in ([(side + '_' + local + n, HAND_GAINS) for n in independent] +
                               [(n, ARM_GAINS) for n in arm_names]):
                ET.SubElement(actuator, 'position', name='drive_' + name, joint=name,
                    kp=str(gain['kp']), kv=str(gain['kv']), forcelimited='true',
                    forcerange=f"{-gain['force_limit']} {gain['force_limit']}")
        parent = mj.MjSpec.from_string(ET.tostring(xml, encoding='unicode'))
        table = spec.table
        parent.worldbody.add_geom(name='table', type=mj.mjtGeom.mjGEOM_BOX,
            size=table.half_extents, pos=table.pose[:3], quat=table.pose[3:],
            contype=1, conaffinity=7, rgba=[.12,.4,.2,1])
        for fixture in spec.fixtures:
            if fixture.parent and fixture.parent != spec.world_frame:
                if fixture.collision:
                    raise ValueError('attached collision fixtures are unsupported')
                continue  # Camera shells are optional visual geometry.
            if fixture.type == 'box':
                parent.worldbody.add_geom(name=fixture.name, type=mj.mjtGeom.mjGEOM_BOX,
                    size=fixture.half_extents, pos=fixture.pose[:3], quat=fixture.pose[3:],
                    contype=1 if fixture.collision else 0,
                    conaffinity=7 if fixture.collision else 0)
            elif fixture.collision:
                raise ValueError('mesh support collision requires a dedicated conversion')
        attachment = attach_to_spec(parent, resource_root=resource_root,
            fixed_root=True, pose=spec.gamepad.root_pose, profile=spec.gamepad.profile,
            attach_visual=render, collision=spec.gamepad.collision)
        apply_standalone_policy(parent.option, attachment.asset.numerical_policy)
        parent.option.sdf_iterations = attachment.asset.sdf_policy['sdf_iterations']
        parent.option.sdf_initpoints = attachment.asset.sdf_policy['sdf_initpoints']
        if not self.environment_collisions:
            for geom in parent.geoms:
                # Robot groups and same/opposite-side policy are unchanged.
                # Disable both masks for every environment/gamepad geometry.
                if geom.contype not in (2, 4):
                    geom.contype, geom.conaffinity = 0, 0
        parent.worldbody.add_light(pos=[.5,.6,2.5], dir=[0,0,-1], diffuse=[.8,.8,.8])
        self.model = parent.compile()
        self.data = mj.MjData(self.model)
        self.gamepad = attachment.bind(self.model, self.data)
        self._hand_indices = {s: np.array([self.model.joint(n).qposadr[0]
                                          for n in self._hand_names[s]]) for s in SIDES}
        self._arm_indices = {s: np.array([self.model.joint(n).qposadr[0]
                                         for n in self._arm_names[s]]) for s in SIDES}
        self._robot_bodies = {b for b in range(self.model.nbody)
            if self.model.body(b).name.startswith(('left_', 'right_'))}
        self._palms, self._tips = {}, {}
        for side in SIDES:
            prefix = side + '_' + ('right_' if side == 'right' else '')
            self._palms[side] = self.model.body(prefix + getattr(spec, side + '_hand').root_link).id
            self._tips[side] = [self.model.body(prefix + f + '_distal').id for f in FINGERS]
        (self.work_dir / 'inertia_corrections.json').write_text(json.dumps(corrections, indent=2) + '\n')
        (self.work_dir / 'scene.xml').write_text(resolve_asset_files(parent.to_xml(), resource_root))
        self.reset()

    def reset(self):
        from xbox_gamepad_physics.backends.common.handle import ResetRequest
        self.mj.mj_resetData(self.model, self.data)
        self.gamepad.reset(ResetRequest())
        for side in SIDES:
            for indices, names, values in [
                (self._arm_indices[side], self._arm_names[side], getattr(self.spec, side + '_arm').home_q),
                (self._hand_indices[side], self._hand_names[side], getattr(self.spec, side + '_hand').home_q),
            ]:
                self.data.qpos[indices] = values  # Initial condition only.
                for name, value in zip(names, values):
                    try:
                        self.data.ctrl[self.model.actuator('drive_' + name).id] = value
                    except KeyError:
                        pass  # Mimic joints have native equality, no actuator.
        self._robot_contact = False
        self._requested_time = 0.
        self._physics_ticks = 0
        self.mj.mj_forward(self.model, self.data)

    def apply(self, commands):
        for command in commands:
            if command.side not in SIDES:
                raise ValueError('unknown robot side')
            if isinstance(command, HandCommand):
                values = np.array([command.joint_positions[n] for n in G20_JOINT_NAMES], float)
                names = self._hand_names[command.side]
                mimic = G20_MIMIC_JOINTS[command.side]
                selected = [(name, value) for n, name, value in zip(G20_JOINT_NAMES, names, values) if n not in mimic]
            elif isinstance(command, ArmCommand):
                values = np.asarray(command.positions, float)
                if values.shape != (7,):
                    raise ValueError('arm command requires seven radians')
                selected = list(zip(self._arm_names[command.side], values))
            else:
                raise TypeError('expected normalized HandCommand or ArmCommand')
            if not np.isfinite(values).all():
                raise ValueError('robot targets must be finite')
            for name, value in selected:
                joint = self.model.joint(name)
                if joint.limited[0]:
                    value = np.clip(value, *joint.range)
                self.data.ctrl[self.model.actuator('drive_' + name).id] = value

    def step(self, dt):
        dt = float(dt)
        if not np.isfinite(dt) or dt <= 0:
            raise ValueError('control interval must be finite and positive')
        # Accumulate the requested clock, rather than rounding each 30 Hz frame
        # independently. Actual state remains on the native 0.5 ms grid.
        self._requested_time += dt
        target_ticks = round(self._requested_time / self.model.opt.timestep)
        count = target_ticks - self._physics_ticks
        self._robot_contact = False
        for _ in range(count):
            self.mj.mj_step(self.model, self.data)
            if not self._robot_contact:
                self._robot_contact = has_robot_contact(
                    (float(contact.dist), any(int(self.model.geom_bodyid[g]) in self._robot_bodies
                                              for g in contact.geom))
                    for contact in self.data.contact)
        self._physics_ticks = target_ticks
        if not np.isfinite(self.data.qpos).all() or not np.isfinite(self.data.qvel).all():
            raise FloatingPointError('MuJoCo robot dynamics became non-finite')
        for warning in (self.mj.mjtWarning.mjWARN_BADQPOS, self.mj.mjtWarning.mjWARN_BADQVEL,
                        self.mj.mjtWarning.mjWARN_BADQACC):
            if self.data.warning[int(warning)].number:
                raise FloatingPointError('MuJoCo reported an invalid dynamics warning')
        self.mj.mj_forward(self.model, self.data)

    def _pose(self, body):
        pose = np.r_[self.data.xpos[body], self.data.xquat[body]].copy()
        if pose[3] < 0:
            pose[3:] *= -1
        return pose

    def observe(self):
        state = self.gamepad.gamepad_state()
        observation = Observation(float(self.data.time), time.time_ns(),
            np.array([self.data.qpos[self._hand_indices[s]] for s in SIDES]),
            np.array([self.data.qpos[self._arm_indices[s]] for s in SIDES]),
            np.array([self._pose(self._palms[s]) for s in SIDES]),
            np.array([[tip_pose(self._pose(b), self.tip_offsets[s][f])
                for f, b in zip(FINGERS, self._tips[s])] for s in SIDES]),
            np.asarray(state.values, float), np.asarray(state.available_mask, bool),
            self.gamepad.joint_positions(), self._robot_contact, self.name)
        observation.validate()
        return observation

    def render_rgb(self):
        if not self._render_enabled:
            raise RuntimeError('construct with render=True to render images')
        if self._renderer is None:
            self.model.vis.global_.offwidth = 640
            self.model.vis.global_.offheight = 480
            self._renderer = self.mj.Renderer(self.model, height=480, width=640)
            self._camera = self.mj.MjvCamera()
            self._camera.lookat[:] = np.asarray(self.spec.gamepad.root_pose[:3]) + [0,0,.08]
            self._camera.distance = 1.25
            self._camera.azimuth, self._camera.elevation = 135., -25.
        self._renderer.update_scene(self.data, camera=self._camera)
        return self._renderer.render().copy()

    def set_render_state(self, observation):
        """Synchronize measured state in the dedicated, non-stepping viewer.

        This method is deliberately restricted to render instances. Collection
        physics never calls it; its qpos writes replay measurements, not targets.
        """
        if not self._render_enabled:
            raise RuntimeError('render-state synchronization requires render=True')
        fields = {name: observation[name] for name in Observation.__dataclass_fields__}
        Observation(**fields).validate()
        for k, side in enumerate(SIDES):
            self.data.qpos[self._hand_indices[side]] = observation['hand_q'][k]
            self.data.qpos[self._arm_indices[side]] = observation['arm_q'][k]
        # Native dynamics can transiently exceed a nominal shaft limit. Render
        # that measured displacement verbatim; the explicit command-state setter
        # would reject it and must not clamp or replace the recorded motion.
        for name, value in observation['control_joint_positions'].items():
            self.gamepad.joint_qpos(name)[0] = value
        self.data.qvel[:] = 0.
        self.data.time = observation['sim_time_s']
        self.mj.mj_forward(self.model, self.data)

    def close(self):
        if self._renderer is not None:
            self._renderer.close()
            self._renderer = None
        if self._temporary is not None:
            self._temporary.cleanup()
