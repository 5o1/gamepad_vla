"""Dual-arm collection adapter, with an explicitly renderer-free CPU path.

Only drive targets are changed during replay. Initial qpos assignment is limited
to reset. This separate adapter leaves the existing direction-pad task intact.
"""
from __future__ import annotations

import math
import os
from pathlib import Path
import tempfile
import time
import xml.etree.ElementTree as ET

import numpy as np

from .commands import ArmCommand, HandCommand
from .contact import has_robot_contact
from .episode_schema import FINGERS, SIDES, Observation, fingertip_offsets, tip_pose
from .g20_mapping import G20_JOINT_NAMES, apply_mimic_joint_positions
from .robot_description import compose_robot_urdf


PHYSICS_DT = 0.001
HAND_GAINS = dict(stiffness=240., damping=8., force_limit=60., armature=1e-4)
ARM_GAINS = dict(stiffness=2e4, damping=2e3, force_limit=500., armature=1e-2)


def _physics_urdf(source, destination):
    """Strip rendering input before SAPIEN parses it; preserve all physics."""
    source, destination = Path(source), Path(destination)
    root = ET.parse(source).getroot()
    for parent in root.iter():
        for child in list(parent):
            if child.tag in {"visual", "material"}:
                parent.remove(child)
    for mesh in root.iter("mesh"):
        name = mesh.get("filename", "")
        if name and not name.startswith(("/", "package://")):
            mesh.set("filename", str((source.parent / name).resolve()))
    ET.ElementTree(root).write(destination, encoding="unicode")
    return destination


class _PhysicsLoader:
    """Public-loader adapter for asset URDFs that still contain visual tags."""
    def __init__(self, loader, directory):
        object.__setattr__(self, "_loader", loader)
        object.__setattr__(self, "_directory", directory)

    def __getattr__(self, name):
        return getattr(self._loader, name)

    def __setattr__(self, name, value):
        setattr(self._loader, name, value)

    def _path(self, path):
        return str(_physics_urdf(path, self._directory / "gamepad_physics.urdf"))

    def parse(self, path, *args, **kwargs):
        return self._loader.parse(self._path(path), *args, **kwargs)

    def load(self, path, *args, **kwargs):
        return self._loader.load(self._path(path), *args, **kwargs)

    def load_multiple(self, path, *args, **kwargs):
        return self._loader.load_multiple(self._path(path), *args, **kwargs)


class _PhysicsScene:
    def __init__(self, scene, directory):
        self._scene, self._directory = scene, directory

    def __getattr__(self, name):
        return getattr(self._scene, name)

    def create_urdf_loader(self):
        return _PhysicsLoader(self._scene.create_urdf_loader(), self._directory)


def _pose(sapien, values):
    return sapien.Pose(list(values[:3]), list(values[3:]))


def _world_pose(link):
    pose = link.get_entity_pose()
    values = np.asarray([*pose.p, *pose.q], dtype=float)
    values[3:] /= np.linalg.norm(values[3:])
    if values[3] < 0:
        values[3:] *= -1
    return values


class CollectionSapienBackend:
    """CPU PhysX collection backend; rendering is an explicit separate option."""
    name = "sapien"

    def __init__(self, spec, work_dir=None, render=False, resource_root=None,
                 environment_collisions=True, gamepad_collisions=None):
        import sapien
        from xbox_gamepad_physics.robotwin import load_into_scene

        if spec.right_arm is None or spec.right_hand is None:
            raise ValueError("collection requires both arms and both hands")
        self.spec = spec
        self.environment_collisions = bool(environment_collisions)
        # gamepad-contact: robots touch only the gamepad, so a hanging master arm
        # passes through the table instead of jamming against it.
        self.gamepad_collisions = self.environment_collisions if gamepad_collisions is None else bool(gamepad_collisions)
        self.work_dir = Path(work_dir or tempfile.mkdtemp(prefix="collection_sapien_"))
        self.work_dir.mkdir(parents=True, exist_ok=True)
        self.render_enabled = bool(render)
        systems = [sapien.physx.PhysxCpuSystem()]
        if render:
            device = os.environ.get("GAMEPAD_VLA_RENDER_DEVICE")
            systems.append(sapien.render.RenderSystem(device) if device else sapien.render.RenderSystem())
        self.scene = sapien.Scene(systems)
        self.scene.set_timestep(PHYSICS_DT)
        self.tip_offsets = fingertip_offsets(spec)
        self._fixed_actors = []
        table = self.scene.create_actor_builder()
        if self.environment_collisions:
            table.add_box_collision(half_size=list(spec.table.half_extents))
        if render:
            table.add_box_visual(half_size=list(spec.table.half_extents))
        actor = table.build_static(name="table")
        actor.set_pose(_pose(sapien, spec.table.pose))
        self._fixed_actors.append(actor)
        for fixture in spec.fixtures:
            if not fixture.collision:
                continue
            if fixture.parent not in (None, spec.world_frame):
                raise ValueError("collection collision fixture must be world-fixed")
            builder = self.scene.create_actor_builder()
            if fixture.type == "box":
                if self.environment_collisions:
                    builder.add_box_collision(half_size=list(fixture.half_extents))
                if render:
                    builder.add_box_visual(half_size=list(fixture.half_extents))
            else:
                if self.environment_collisions:
                    builder.add_nonconvex_collision_from_file(fixture.mesh)
                if render:
                    builder.add_visual_from_file(fixture.mesh)
            actor = builder.build_static(name=fixture.name)
            actor.set_pose(_pose(sapien, fixture.pose))
            self._fixed_actors.append(actor)
        self.robots, self.joints, self.links, self.joint_order = {}, {}, {}, {}
        self._robot_entities = set()
        for index, side in enumerate(SIDES):
            arm, hand = getattr(spec, side + "_arm"), getattr(spec, side + "_hand")
            urdf = compose_robot_urdf(spec, self.work_dir / (side + "_robot.urdf"), side)
            if not render:
                urdf = _physics_urdf(urdf, self.work_dir / (side + "_physics.urdf"))
            loader = self.scene.create_urdf_loader()
            loader.fix_root_link = True
            robot = loader.load(str(urdf))
            robot.set_root_pose(_pose(sapien, arm.base_pose))
            self.robots[side] = robot
            self.joints[side] = {j.name: j for j in robot.get_active_joints()}
            self.joint_order[side] = list(self.joints[side])
            self.links[side] = {link.name: link for link in robot.get_links()}
            prefix = "right_" if side == "right" else ""
            for name in arm.joint_names:
                self._configure(self.joints[side][name], ARM_GAINS)
            for name in G20_JOINT_NAMES:
                self._configure(self.joints[side][prefix + name], HAND_GAINS)
            for link in robot.get_links():
                self._robot_entities.add(link.entity)
                for shape in link.get_collision_shapes():
                    groups = list(shape.get_collision_groups())
                    groups[2] |= 0x00010000 << index
                    shape.set_collision_groups(groups)
        previous_resources = os.environ.get("XBOX_GAMEPAD_PHYSICS_RESOURCES")
        if resource_root is not None:
            os.environ["XBOX_GAMEPAD_PHYSICS_RESOURCES"] = str(resource_root)
        try:
            self.gamepad = load_into_scene(
                self.scene if render else _PhysicsScene(self.scene, self.work_dir),
                _pose(sapien, spec.gamepad.root_pose), fixed_root=spec.gamepad.fixed_root,
                profile=spec.gamepad.profile, attach_visual=bool(render),
                collision=spec.gamepad.collision)
        finally:
            if resource_root is not None:
                if previous_resources is None:
                    os.environ.pop("XBOX_GAMEPAD_PHYSICS_RESOURCES", None)
                else:
                    os.environ["XBOX_GAMEPAD_PHYSICS_RESOURCES"] = previous_resources
        self._initial_gamepad_q = dict(self.gamepad.joint_positions())
        if not self.gamepad_collisions:
            for link in self.gamepad.articulation.get_links():
                for shape in link.get_collision_shapes():
                    groups = list(shape.get_collision_groups())
                    # Zero contact membership and affinity; retain the reserved
                    # asset filtering words and all model geometry/inertias.
                    groups[0] = groups[1] = 0
                    shape.set_collision_groups(groups)
        self._camera = None
        if render:
            from .spec import matrix_to_pose
            self.scene.set_ambient_light([.5] * 3)
            self.scene.add_directional_light([0., .5, -1.], [.8] * 3, shadow=True)
            self._camera = self.scene.add_camera("collection_operator", 640, 480, math.radians(55), .01, 10.)
            target = np.asarray(spec.gamepad.root_pose[:3])
            position = target + np.asarray([.6, -.6, .55])
            forward = target - position
            forward /= np.linalg.norm(forward)
            left = np.cross([0., 0., 1.], forward)
            left /= np.linalg.norm(left)
            matrix = np.eye(4)
            matrix[:3, :3] = np.column_stack((forward, left, np.cross(forward, left)))
            matrix[:3, 3] = position
            self._camera.entity.set_pose(_pose(sapien, matrix_to_pose(matrix)))
        self.reset()

    @staticmethod
    def _configure(joint, gains):
        joint.set_drive_property(stiffness=gains["stiffness"], damping=gains["damping"],
                                 force_limit=gains["force_limit"], mode="force")
        joint.set_armature([gains["armature"]])

    def reset(self):
        self._ticks = 0
        self._requested_time = 0.
        self._robot_contact = False
        for side in SIDES:
            arm, hand = getattr(self.spec, side + "_arm"), getattr(self.spec, side + "_hand")
            prefix = "right_" if side == "right" else ""
            targets = dict(zip(arm.joint_names, arm.home_q))
            targets.update(zip((prefix + n for n in G20_JOINT_NAMES), hand.home_q))
            robot = self.robots[side]
            robot.set_qpos(np.asarray([targets[n] for n in self.joint_order[side]]))
            robot.set_qvel(np.zeros(len(self.joint_order[side])))
            for name, value in targets.items():
                self.joints[side][name].set_drive_target(float(value))
        # Compatible with the installed asset release that predates ResetRequest.
        # This is initialization/reset, never a substitute for contact dynamics.
        self.gamepad.set_joint_positions(self._initial_gamepad_q)
        self.gamepad.articulation.set_qvel(np.zeros(len(self._initial_gamepad_q)))

    def apply(self, commands):
        for command in commands:
            if command.side not in SIDES:
                raise ValueError("command side must be left or right")
            side = command.side
            if isinstance(command, HandCommand):
                values = apply_mimic_joint_positions(side, [command.joint_positions[n] for n in G20_JOINT_NAMES])
                prefix = "right_" if side == "right" else ""
                targets = zip((prefix + n for n in G20_JOINT_NAMES), values)
            elif isinstance(command, ArmCommand):
                values = np.asarray(command.positions, float)
                if values.shape != (7,):
                    raise ValueError("arm command requires seven radians")
                targets = zip(getattr(self.spec, side + "_arm").joint_names, values)
            else:
                raise TypeError("unsupported collection command")
            targets = list(targets)
            if not all(math.isfinite(float(value)) for _, value in targets):
                raise ValueError("drive targets must be finite")
            for name, value in targets:
                self.joints[side][name].set_drive_target(float(value))

    def step(self, dt):
        dt = float(dt)
        if not math.isfinite(dt) or dt <= 0:
            raise ValueError("step duration must be finite and positive")
        self._requested_time += dt
        target_tick = math.floor(self._requested_time / PHYSICS_DT + 1e-9)
        self._robot_contact = False
        while self._ticks < target_tick:
            for robot in self.robots.values():
                robot.set_qf(robot.compute_passive_force(gravity=True, coriolis_and_centrifugal=True))
            self.scene.step()
            self._ticks += 1
            self._robot_contact = self._robot_contact or self._has_robot_contact()
        if any(not np.isfinite(robot.get_qpos()).all() for robot in self.robots.values()):
            raise FloatingPointError("nonfinite SAPIEN articulation state")

    def _has_robot_contact(self):
        # PhysX reports contact-offset candidates before surfaces meet. Match
        # MuJoCo's geometric dist <= 0 convention, not merely point presence.
        return has_robot_contact(
            (float(point.separation), any(body.entity in self._robot_entities for body in contact.bodies))
            for contact in self.scene.get_contacts() for point in contact.points)

    def observe(self):
        hands, arms, palms, tips = [], [], [], []
        for side in SIDES:
            q = dict(zip(self.joint_order[side], self.robots[side].get_qpos()))
            prefix = "right_" if side == "right" else ""
            hands.append([q[prefix + n] for n in G20_JOINT_NAMES])
            arms.append([q[n] for n in getattr(self.spec, side + "_arm").joint_names])
            palms.append(_world_pose(self.links[side][prefix + getattr(self.spec, side + "_hand").root_link]))
            tips.append([tip_pose(_world_pose(self.links[side][prefix + finger + "_distal"]),
                                  self.tip_offsets[side][finger]) for finger in FINGERS])
        state = self.gamepad.gamepad_state()
        observation = Observation(
            sim_time_s=self._ticks * PHYSICS_DT, wall_time_ns=time.time_ns(),
            hand_q=np.asarray(hands), arm_q=np.asarray(arms),
            palm_pose_world=np.asarray(palms), fingertip_pose_world=np.asarray(tips),
            gamepad_signal=np.asarray(state.values, float), gamepad_mask=np.asarray(state.available_mask, bool),
            control_joint_positions=self.gamepad.joint_positions(),
            robot_contact=bool(self._robot_contact or self._has_robot_contact()), backend=self.name)
        observation.validate()
        return observation

    def render_rgb(self):
        if self._camera is None:
            raise RuntimeError("render_rgb requires an explicitly enabled renderer")
        self.scene.update_render()
        self._camera.take_picture()
        return np.rint(np.clip(self._camera.get_picture("Color")[..., :3], 0., 1.) * 255).astype(np.uint8)

    def set_render_state(self, observation_dict):
        """Copy actual completed physics state into a separate render replica.

        This method never steps dynamics. The simulation/recording backend must
        use apply/step; only its independent renderer may use this snapshot API.
        """
        if not self.render_enabled:
            raise RuntimeError("set_render_state is restricted to the render replica")
        hands = np.asarray(observation_dict["hand_q"], float)
        arms = np.asarray(observation_dict["arm_q"], float)
        if hands.shape != (2, 21) or arms.shape != (2, 7):
            raise ValueError("render state requires measured hand_q(2,21) and arm_q(2,7)")
        if not np.isfinite(hands).all() or not np.isfinite(arms).all():
            raise ValueError("render state must be finite")
        for index, side in enumerate(SIDES):
            prefix = "right_" if side == "right" else ""
            positions = dict(zip(getattr(self.spec, side + "_arm").joint_names, arms[index]))
            positions.update(zip((prefix + n for n in G20_JOINT_NAMES), hands[index]))
            self.robots[side].set_qpos(np.asarray([positions[n] for n in self.joint_order[side]]))
        controls = observation_dict["control_joint_positions"]
        if set(controls) != set(self._initial_gamepad_q):
            raise ValueError("render state must contain all actual gamepad control displacements")
        if not all(math.isfinite(float(value)) for value in controls.values()):
            raise ValueError("render control displacements must be finite")
        self.gamepad.set_joint_positions(controls)

    def close(self):
        self._camera = None
        self.gamepad = None
        self.links.clear()
        self.joints.clear()
        self.robots.clear()
        self._robot_entities.clear()
        self._fixed_actors.clear()
        self.scene = None
