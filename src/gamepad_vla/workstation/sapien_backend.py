"""SAPIEN 3.0.3 simulation backend: AR5 left arm + G20 left hand + SDF Xbox gamepad.

Poses come only from the backend-neutral ``WorkcellSpec``. The gamepad is loaded
through ``xbox_gamepad_physics.robotwin.load_into_scene`` (release v0.4.1, SDF
collision), whose ``gamepad_state()`` yields the 32-channel signal.

Simulation choices, all placeholders rather than measured values:
- The arm and hand are one articulation built from the station AR5 URDF's left
  chain and the station G20 left URDF, joined at ``spec.left_hand.mount``.
- Robot self-collision is off (as in the station Isaac project); robot links
  still collide with the gamepad and the table box.
- SAPIEN loads URDF ``<mimic>`` joints as independent joints, so mimic joints
  are driven to ``multiplier * source target`` (software coupling).
- Force-mode PD drives with armature: hand 240/8 (station placeholder gains),
  arm stiff enough to hold pose; tiny G20 links diverge without armature.
- Gravity is compensated with the articulation's passive force each substep.
"""

import os
import tempfile
from pathlib import Path

import numpy as np

from gamepad_vla.workstation.backends import Observation
from gamepad_vla.workstation.commands import ArmCommand, HandCommand
from gamepad_vla.workstation.g20_mapping import G20_JOINT_NAMES, G20_MIMIC_JOINTS
from gamepad_vla.workstation.robot_description import compose_robot_urdf
from gamepad_vla.workstation.appearance import load_appearance, match_material, match_visual_split

ROBOT_IGNORE_BIT = 0x00010000  # groups[2] bit shared by every robot shape
RIGHT_ROBOT_IGNORE_BIT = 0x00020000
PHYSICS_DT = 1 / 240
HAND_GAINS = dict(stiffness=240.0, damping=8.0, force_limit=60.0, armature=1e-4)
ARM_GAINS = dict(stiffness=2.0e4, damping=2.0e3, force_limit=500.0, armature=1e-2)


def _sapien_pose(pose):
    import sapien
    return sapien.Pose(list(pose[:3]), list(pose[3:]))


def validate_scene_injection(scene, gamepad):
    """Check runner inputs before importing SAPIEN or building any actors."""
    if (scene is None) != (gamepad is None):
        raise ValueError("scene and gamepad must be supplied together")
    # PhysX stores timestep as float32; reading back 1/240 differs by ~2.2e-10 s.
    if scene is not None and not abs(scene.get_timestep() - PHYSICS_DT) <= 1e-9:
        raise ValueError(f"injected scene timestep must equal {PHYSICS_DT}")


class SapienBackend:
    """Fixed-rate SAPIEN workcell; ``step(dt)`` runs ``dt / PHYSICS_DT`` substeps."""

    name = "sapien"

    def __init__(self, spec, *, work_dir=None, collision=None,
                 scene=None, gamepad=None, render=False):
        validate_scene_injection(scene, gamepad)
        self.spec = spec
        # SAPIEN 3.0.3's native renderer owns the per-process render engine.
        # Initialize it before materials/Scene (including headless physics).
        # Source: SAPIEN 3.0.3 src/sapien_renderer/sapien_renderer_system.cpp.
        device = os.environ.get("GAMEPAD_VLA_RENDER_DEVICE")
        self._render_device_guard = None
        if device and scene is None:
            import sapien
            sapien.render.set_global_config(max_num_materials=50000, max_num_textures=50000)
            self._render_device_guard = sapien.render.SapienRenderer(sapien.Device(device))
        if scene is None:
            from xbox_gamepad_physics.robotwin import create_scene
            scene = create_scene(timestep=PHYSICS_DT, render=render)
        self.scene = scene
        self.appearance = None
        self.material_assignments = []
        if spec.appearance:
            path = Path(spec.appearance)
            if not path.is_absolute():
                if spec.source_dir is None:
                    raise ValueError("relative appearance requires a loaded spec's source directory")
                path = spec.source_dir / path
            self.appearance = load_appearance(path)

        table = self.scene.create_actor_builder()
        table.add_box_collision(half_size=list(spec.table.half_extents))
        table.add_box_visual(half_size=list(spec.table.half_extents), material=self._material("table"))
        self.table = table.build_static(name="table")
        self.table.set_pose(_sapien_pose(spec.table.pose))
        self.fixtures = []
        for fixture in spec.fixtures:
            if fixture.parent is not None and fixture.parent != spec.world_frame:
                if fixture.type != "mesh" or fixture.collision:
                    raise ValueError("attached camera shells must be visual-only meshes")
                continue
            builder = self.scene.create_actor_builder()
            material = self._material(fixture.name, fixture.mesh or "")
            if fixture.type == "box":
                if fixture.collision:
                    builder.add_box_collision(half_size=list(fixture.half_extents))
                builder.add_box_visual(half_size=list(fixture.half_extents), material=material)
            else:
                if fixture.collision:
                    builder.add_nonconvex_collision_from_file(fixture.mesh)
                builder.add_visual_from_file(fixture.mesh, material=material)
            actor = builder.build_static(name=fixture.name)
            actor.set_pose(_sapien_pose(fixture.pose))
            self.fixtures.append(actor)

        self._work_dir = Path(work_dir) if work_dir else Path(tempfile.mkdtemp(prefix="workcell_"))
        self._work_dir.mkdir(parents=True, exist_ok=True)
        urdf = compose_robot_urdf(spec, self._work_dir / "workcell_robot.urdf")
        loader = self.scene.create_urdf_loader()
        loader.fix_root_link = True
        self.robot = loader.load(str(urdf))
        self.robot.set_root_pose(_sapien_pose(spec.left_arm.base_pose))
        for link in self.robot.get_links():
            for shape in link.get_collision_shapes():
                groups = list(shape.get_collision_groups())
                groups[2] |= ROBOT_IGNORE_BIT
                shape.set_collision_groups(groups)

        self.joints = {j.name: j for j in self.robot.get_active_joints()}
        self.joint_order = [j.name for j in self.robot.get_active_joints()]
        missing = (set(G20_JOINT_NAMES) | set(spec.left_arm.joint_names)) - set(self.joints)
        if missing:
            raise ValueError(f"robot articulation lacks joints {sorted(missing)}")
        for name in G20_JOINT_NAMES:
            self._configure(self.joints[name], HAND_GAINS)
        for name in spec.left_arm.joint_names:
            self._configure(self.joints[name], ARM_GAINS)
        self.hand_root = next(l for l in self.robot.get_links() if l.name == spec.left_hand.root_link)
        self.right_robot = None
        if spec.right_arm is not None:
            right_urdf = compose_robot_urdf(spec, self._work_dir / "workcell_right_robot.urdf", "right")
            self.right_robot = loader.load(str(right_urdf))
            self.right_robot.set_root_pose(_sapien_pose(spec.right_arm.base_pose))
            self.right_joints = {j.name: j for j in self.right_robot.get_active_joints()}
            self.right_joint_order = list(self.right_joints)
            for name in spec.right_arm.joint_names:
                self._configure(self.right_joints[name], ARM_GAINS)
            for name in G20_JOINT_NAMES:
                self._configure(self.right_joints["right_" + name], HAND_GAINS)
            # A separate ignore bit disables right self-collision while retaining
            # left-right collisions. Never reuse the left self-collision bit.
            for link in self.right_robot.get_links():
                for shape in link.get_collision_shapes():
                    groups = list(shape.get_collision_groups())
                    groups[2] |= RIGHT_ROBOT_IGNORE_BIT
                    shape.set_collision_groups(groups)
        if self.appearance:
            self._apply_robot_appearance()
        for fixture in spec.fixtures:
            if fixture.parent is None or fixture.parent == spec.world_frame:
                continue
            from sapien.render import RenderBodyComponent, RenderShapeTriangleMesh
            entity = self.camera_parent_entity(fixture.parent)
            original = entity.find_component_by_type(RenderBodyComponent)
            body = original.clone() if original is not None else RenderBodyComponent()
            shell = RenderShapeTriangleMesh(fixture.mesh, [1,1,1], self._material(fixture.name,fixture.mesh))
            shell.local_pose = _sapien_pose(fixture.pose)
            body.attach(shell)
            if original is not None:
                entity.remove_component(original)
            entity.add_component(body)

        if gamepad is None:
            from xbox_gamepad_physics.robotwin import load_into_scene

            gamepad = load_into_scene(self.scene, _sapien_pose(spec.gamepad.root_pose),
                                      profile=spec.gamepad.profile, attach_visual=render,
                                      fixed_root=spec.gamepad.fixed_root,
                                      collision=collision or spec.gamepad.collision)
        self.gamepad = gamepad
        overlap = [shape for link in self.gamepad.articulation.get_links()
                   for shape in link.get_collision_shapes()
                   if shape.get_collision_groups()[2] & (ROBOT_IGNORE_BIT | RIGHT_ROBOT_IGNORE_BIT)]
        if overlap:
            raise RuntimeError("gamepad shapes use the robot ignore bit; collisions would be lost")
        self._mimic = G20_MIMIC_JOINTS["left"]
        self.time = 0.0
        self.reset()

    def _material(self, name, mesh="", material_name=None):
        if self.appearance is None:
            return None
        entry = (next(e for e in self.appearance["materials"] if e["name"] == material_name)
                 if material_name else match_material(self.appearance, name, mesh))
        if entry is None:
            # Newly added supports have no photo-fit rule. Make them visible
            # without changing the appearance file or collision geometry.
            if name in {f.name for f in self.spec.fixtures}:
                from sapien.render import RenderMaterial
                return RenderMaterial(base_color=[.4,.4,.4,1.], metallic=0., roughness=.7)
            return None
        from sapien.render import RenderMaterial
        self.material_assignments.append(dict(link=name, mesh=mesh, material=entry["name"]))
        return RenderMaterial(base_color=entry["base_color"], metallic=entry["metallic"],
                              roughness=entry["roughness"])

    def _apply_robot_appearance(self):
        from sapien.render import RenderBodyComponent, RenderShapeTriangleMesh
        for robot in (self.robot, self.right_robot):
            if robot is None:
                continue
            for link in robot.get_links():
                body = link.entity.find_component_by_type(RenderBodyComponent)
                if body is None:
                    continue
                replacement = RenderBodyComponent()
                for shape in body.render_shapes:
                    if isinstance(shape, RenderShapeTriangleMesh):
                        split = match_visual_split(self.appearance, link.name, shape.filename)
                        if split:
                            for part in split["parts"]:
                                new_shape = RenderShapeTriangleMesh(
                                    part["mesh"], shape.scale,
                                    self._material(link.name, part["mesh"], part["material"]))
                                new_shape.local_pose = shape.local_pose
                                replacement.attach(new_shape)
                            continue
                        material = self._material(link.name, shape.filename)
                        new_shape = (RenderShapeTriangleMesh(shape.filename, shape.scale, material)
                                     if material is not None else shape.clone())
                    else:
                        # AR5's non-surface TCP marker is a box, not a mesh.
                        new_shape = shape.clone()
                    new_shape.local_pose = shape.local_pose
                    replacement.attach(new_shape)
                # URDF materials can be shared by many links. Rebuild only the
                # render component so a gray base never recolors white shells.
                link.entity.remove_component(body)
                link.entity.add_component(replacement)

    @staticmethod
    def _configure(joint, gains):
        joint.set_drive_property(stiffness=gains["stiffness"], damping=gains["damping"],
                                 force_limit=gains["force_limit"], mode="force")
        joint.set_armature([gains["armature"]])

    def _set_targets(self, targets):
        for name, value in targets.items():
            self.joints[name].set_drive_target(float(value))
            self._targets[name] = float(value)

    def reset(self):
        home = dict(zip(self.spec.left_arm.joint_names, self.spec.left_arm.home_q))
        home.update(dict(zip(G20_JOINT_NAMES, self.spec.left_hand.home_q)))
        self._targets = {}
        self.robot.set_qpos(np.array([home[name] for name in self.joint_order]))
        self.robot.set_qvel(np.zeros(len(self.joint_order)))
        self._set_targets(home)
        if self.right_robot is not None:
            home_right = dict(zip(self.spec.right_arm.joint_names, self.spec.right_arm.home_q))
            home_right.update(dict(zip(("right_" + n for n in G20_JOINT_NAMES), self.spec.right_hand.home_q)))
            self.right_robot.set_qpos(np.array([home_right[n] for n in self.right_joint_order]))
            self.right_robot.set_qvel(np.zeros(len(self.right_joint_order)))
            for name, value in home_right.items():
                self.right_joints[name].set_drive_target(float(value))
        self.time = 0.0

    def apply(self, commands):
        for command in commands:
            if command.side != "left":
                raise ValueError("the SAPIEN workcell accepts commands only for the left arm and hand")
            if isinstance(command, HandCommand):
                targets = dict(command.joint_positions)
                for mimic, (source, multiplier) in self._mimic.items():
                    targets[mimic] = multiplier * targets[source]
                self._set_targets({name: targets[name] for name in G20_JOINT_NAMES})
            elif isinstance(command, ArmCommand):
                self._set_targets(dict(zip(self.spec.left_arm.joint_names, command.positions)))

    def step(self, dt):
        for _ in range(max(1, round(dt / PHYSICS_DT))):
            self.robot.set_qf(self.robot.compute_passive_force(gravity=True, coriolis_and_centrifugal=True))
            if self.right_robot is not None:
                self.right_robot.set_qf(self.right_robot.compute_passive_force(gravity=True, coriolis_and_centrifugal=True))
            self.scene.step()
            self.time += PHYSICS_DT
        q = self.robot.get_qpos()
        if not np.all(np.isfinite(q)):
            raise FloatingPointError("SAPIEN robot state became non-finite")
        if self.right_robot is not None and not np.isfinite(self.right_robot.get_qpos()).all():
            raise FloatingPointError("SAPIEN right robot state became non-finite")

    def observe(self):
        q = dict(zip(self.joint_order, self.robot.get_qpos().tolist()))
        pose = self.hand_root.get_entity_pose()
        state = self.gamepad.gamepad_state()
        return Observation(
            stamp=self.time,
            hand_joint_positions={name: q[name] for name in G20_JOINT_NAMES},
            arm_joint_positions=tuple(q[name] for name in self.spec.left_arm.joint_names),
            hand_root_pose_world=(*pose.p.tolist(), *pose.q.tolist()),
            gamepad_signal=np.asarray(state.values, dtype=np.float32),
            gamepad_mask=np.asarray(state.available_mask, dtype=bool),
            backend=self.name,
        )

    def link_pose(self, name):
        """World pose ``[x, y, z, qw, qx, qy, qz]`` of a robot link (diagnostics, IK)."""
        links = list(self.robot.get_links())
        if self.right_robot is not None:
            links += list(self.right_robot.get_links())
        link = next(l for l in links if l.name == name)
        pose = link.get_entity_pose()
        return (*pose.p.tolist(), *pose.q.tolist())

    def camera_parent_entity(self, parent):
        owner, name = parent.split(":",1)
        side, kind = owner.split("_",1)
        robot = self.robot if side == "left" else self.right_robot
        if kind == "hand" and side == "right":
            name = "right_" + name
        return next(link.entity for link in robot.get_links() if link.name == name)

    def set_review_state(self, snapshot):
        """Set a recorded pose for static image review, without stepping physics.

        This is deliberately separate from the left-only live command API.
        Hand values must already be converted to the 21 URDF joint radians.
        """
        for side, robot in (("left", self.robot), ("right", self.right_robot)):
            if robot is None:
                continue
            arm = getattr(self.spec, side + "_arm")
            aq, hq = (np.asarray(snapshot[side + "_" + kind], float) for kind in ("arm", "hand"))
            if aq.shape != (7,) or hq.shape != (21,) or not np.isfinite(np.r_[aq,hq]).all():
                raise ValueError("review snapshot requires finite arm(7), hand(21) radians per side")
            prefix = "right_" if side == "right" else ""
            targets = dict(zip(arm.joint_names, aq))
            targets.update(dict(zip((prefix + n for n in G20_JOINT_NAMES),hq)))
            joints = {j.name:j for j in robot.get_active_joints()}
            robot.set_qpos(np.array([targets[n] for n in joints]))
            robot.set_qvel(np.zeros(len(joints)))
            for name,value in targets.items():
                joints[name].set_drive_target(float(value))

    def hold(self):
        pass

    def close(self):
        self.scene = None
