"""Fixed AR5/G20 contact scene for residual control, using the public gamepad API.

The arm joints are welded at the selected workcell posture. Hand joints remain
physical, with native equality constraints for URDF mimic coupling. Only robot
actuators are commanded; gamepad controls retain their upstream return laws.
"""
import json
from dataclasses import replace
import tempfile
import xml.etree.ElementTree as ET
from pathlib import Path

import numpy as np

from gamepad_vla.kinematics import SerialChain, rpy_to_matrix, axis_angle_to_matrix
from gamepad_vla.workstation.robot_description import compose_robot_urdf, matrix_to_rpy
from gamepad_vla.workstation.spec import load_spec, pose_to_matrix
from gamepad_vla.workstation.paths import resolve_workspace_path
from gamepad_vla.workstation.g20_mapping import G20_JOINT_NAMES, G20_MIMIC_JOINTS

THUMB_JOINTS = ('thumb_cmc_roll', 'thumb_cmc_yaw', 'thumb_cmc_pitch', 'thumb_mcp')


def load_task(path):
    path = Path(path).resolve()
    config = json.loads(path.read_text())
    config['workcell'] = str((path.parent / config['workcell']).resolve())
    config['resource_root'] = resolve_workspace_path(config['resource_root'])
    return config


def project_inertia_tensor(tensor):
    """Nearest Frobenius projection onto the positive inertia triangle cone.

    Used only for the single evidenced G20 triangle violation; non-positive
    inputs are rejected rather than inventing mass distribution.
    """
    eigenvalues, basis = np.linalg.eigh(tensor)
    if eigenvalues[0] <= 0:
        raise ValueError("non-positive input inertia")
    deficit = eigenvalues[2] - eigenvalues[0] - eigenvalues[1]
    if deficit <= 0:
        return np.asarray(tensor).copy()
    eigenvalues += (deficit / 3 + np.finfo(float).eps * eigenvalues[2]) * np.array([1.,1.,-1.])
    return (basis * eigenvalues) @ basis.T


def fixed_robot_urdf(workcell, path):
    """Fold each frozen arm angle into its joint origin without changing meshes."""
    compose_robot_urdf(workcell, path)
    tree = ET.parse(path)
    root = tree.getroot()
    for joint in root.findall('joint'):
        name = joint.get('name')
        if name not in workcell.left_arm.joint_names:
            continue
        angle = workcell.left_arm.home_q[workcell.left_arm.joint_names.index(name)]
        origin = joint.find('origin')
        if origin is None:
            origin = ET.SubElement(joint, 'origin')
        rotation = rpy_to_matrix([float(x) for x in origin.get('rpy', '0 0 0').split()])
        axis = [float(x) for x in joint.find('axis').get('xyz').split()]
        rotation = rotation @ axis_angle_to_matrix(axis, np.array([angle]))[0]
        origin.set('rpy', ' '.join(str(x) for x in matrix_to_rpy(rotation)))
        joint.set('type', 'fixed')
        for child in list(joint):
            if child.tag not in ('origin', 'parent', 'child'):
                joint.remove(child)
    corrections = []
    for link in root.findall('link'):
        element = link.find('inertial/inertia')
        if element is None:
            continue
        a = {k: float(v) for k,v in element.attrib.items()}
        tensor = np.array([[a['ixx'],a['ixy'],a['ixz']],
                           [a['ixy'],a['iyy'],a['iyz']],
                           [a['ixz'],a['iyz'],a['izz']]])
        projected = project_inertia_tensor(tensor)
        if np.array_equal(projected,tensor):
            continue
        corrections.append(dict(link=link.get('name'), before=tensor.tolist(), after=projected.tolist(),
                                method='nearest Frobenius projection onto inertia triangle halfspace'))
        for key,i,j in [('ixx',0,0),('iyy',1,1),('izz',2,2),('ixy',0,1),('ixz',0,2),('iyz',1,2)]:
            element.set(key,str(projected[i,j]))
    Path(path).with_suffix('.inertia.json').write_text(json.dumps(corrections,indent=2)+'\n')
    extension = ET.SubElement(root, 'mujoco')
    ET.SubElement(extension, 'compiler', discardvisual='false', fusestatic='false', strippath='false')
    tree.write(path)
    return path


def resolve_asset_files(xml_text, resource_root):
    """Keep composed XML reloadable after child-spec attachment loses its base path."""
    directory = Path(resource_root)
    if not (directory/'manifest.json').is_file():
        directory = directory/'mujoco/full_controls'
    root = ET.fromstring(xml_text)
    for asset in root.findall('asset/*[@file]'):
        filename = Path(asset.get('file'))
        if not filename.is_absolute():
            asset.set('file',str((directory/filename).resolve()))
    return ET.tostring(root,encoding='unicode')


class FixedGamepadPhysics:
    def __init__(self, config, *, work_dir=None):
        import mujoco as mj
        from xbox_gamepad_physics.mujoco import attach_to_spec
        from xbox_gamepad_physics.mujoco.numerics import apply_standalone_policy

        self.mj = mj
        self.config = config
        workcell = load_spec(config['workcell'])
        if 'gamepad_root_pose' in config:
            workcell = replace(workcell, gamepad=replace(workcell.gamepad, root_pose=tuple(config['gamepad_root_pose'])))
        self.workcell = workcell
        if not workcell.gamepad.fixed_root:
            raise ValueError('fixed-gamepad task requires a fixed gamepad root')
        self._temporary = tempfile.TemporaryDirectory(prefix='fixed_gamepad_') if work_dir is None else None
        self.work_dir = Path(self._temporary.name if self._temporary else work_dir)
        self.work_dir.mkdir(parents=True, exist_ok=True)
        source = fixed_robot_urdf(workcell, self.work_dir / 'fixed_arm_g20.urdf')
        robot = mj.MjSpec.from_file(str(source))
        # URDF import keeps explicit inertias. Body gravity compensation follows
        # the reference baseline; actuator gains remain unmeasured placeholders.
        for body in robot.bodies:
            body.gravcomp = 1.0
        for geom in robot.geoms:
            if geom.contype or geom.conaffinity:
                geom.contype, geom.conaffinity = 2, 1
        for joint in robot.joints:
            joint.armature = config['hand_gains']['armature']
        parent = mj.MjSpec()
        mount = parent.worldbody.add_frame(pos=workcell.left_arm.base_pose[:3],
                                          quat=workcell.left_arm.base_pose[3:])
        parent.attach(robot, prefix='robot_', frame=mount)
        # Use XML to author normal position actuators and native mimic equalities.
        xml = ET.fromstring(parent.to_xml())
        equality = ET.SubElement(xml, 'equality')
        for child, (source, multiplier) in G20_MIMIC_JOINTS['left'].items():
            ET.SubElement(equality, 'joint', name='mimic_' + child, joint1='robot_' + child,
                          joint2='robot_' + source, polycoef=f'0 {multiplier} 0 0 0')
        actuator = ET.SubElement(xml, 'actuator')
        gain = config['hand_gains']
        self.independent = tuple(n for n in G20_JOINT_NAMES if n not in G20_MIMIC_JOINTS['left'])
        for name in self.independent:
            ET.SubElement(actuator, 'position', name='drive_' + name, joint='robot_' + name,
                          kp=str(gain['kp']), kv=str(gain['kv']), forcelimited='true',
                          forcerange=f"{-gain['force_limit']} {gain['force_limit']}")
        parent = mj.MjSpec.from_string(ET.tostring(xml, encoding='unicode'))
        table = workcell.table
        parent.worldbody.add_geom(name='table', type=mj.mjtGeom.mjGEOM_BOX,
                                  size=table.half_extents, pos=table.pose[:3], quat=table.pose[3:],
                                  contype=1, conaffinity=3, rgba=[.12,.4,.2,1])
        for fixture in workcell.fixtures:
            if fixture.type == 'box' and fixture.collision:
                parent.worldbody.add_geom(name=fixture.name, type=mj.mjtGeom.mjGEOM_BOX,
                    size=fixture.half_extents, pos=fixture.pose[:3], quat=fixture.pose[3:],
                    contype=1, conaffinity=3)
        attachment = attach_to_spec(parent, resource_root=config['resource_root'],
                                    fixed_root=True, pose=workcell.gamepad.root_pose,
                                    attach_visual=False, collision='sdf')
        apply_standalone_policy(parent.option, attachment.asset.numerical_policy)
        policy = attachment.asset.sdf_policy
        parent.option.sdf_iterations = policy['sdf_iterations']
        parent.option.sdf_initpoints = policy['sdf_initpoints']
        self.model = parent.compile()
        self.data = mj.MjData(self.model)
        self.gamepad = attachment.bind(self.model, self.data)
        self.substeps = round(config['control_timestep_s'] / self.model.opt.timestep)
        if not np.isclose(self.substeps * self.model.opt.timestep, config['control_timestep_s']):
            raise ValueError('control timestep must be an integer number of physics steps')
        self.qidx = np.array([self.model.joint('robot_' + n).qposadr[0] for n in G20_JOINT_NAMES])
        self.vidx = np.array([self.model.joint('robot_' + n).dofadr[0] for n in G20_JOINT_NAMES])
        self.aidx = np.array([self.model.actuator('drive_' + n).id for n in self.independent])
        self.thumb_act = np.array([self.model.actuator('drive_' + n).id for n in THUMB_JOINTS])
        self.limits = np.array([self.model.joint('robot_' + n).range for n in THUMB_JOINTS])
        self.thumb_body = self.model.body('robot_' + config['thumb_link']).id
        self.hand_body = self.model.body('robot_' + workcell.left_hand.root_link).id
        self.gamepad_bodies = {self.model.body(n).id for n in self.gamepad.body_names} if hasattr(self.gamepad, 'body_names') else {
            i for i in range(self.model.nbody) if self.model.body(i).name.startswith('xbox_')}
        self.robot_bodies = {i for i in range(self.model.nbody) if self.model.body(i).name.startswith('robot_')}
        self.reset()
        self.ik = ThumbIK(config, workcell, self.hand_pose())
        (self.work_dir / 'ik.json').write_text(json.dumps(dict(
            commands=self.ik.commands.tolist(),point_errors_m=self.ik.errors,
            hover_to_press_distance_rad=np.linalg.norm(self.ik.commands[1:]-self.ik.commands[0],axis=1).tolist()),indent=2)+'\n')
        (self.work_dir / 'scene.xml').write_text(resolve_asset_files(parent.to_xml(),config['resource_root']))

    def reset(self):
        from xbox_gamepad_physics.backends.common.handle import ResetRequest
        self.mj.mj_resetData(self.model, self.data)
        self.gamepad.reset(ResetRequest())
        self.data.qpos[self.qidx] = self.workcell.left_hand.home_q
        for name, idx in zip(self.independent, self.aidx):
            self.data.ctrl[idx] = self.workcell.left_hand.home_q[G20_JOINT_NAMES.index(name)]
        self.mj.mj_forward(self.model, self.data)

    def hand_pose(self):
        transform = np.eye(4)
        transform[:3, :3] = self.data.xmat[self.hand_body].reshape(3,3)
        transform[:3, 3] = self.data.xpos[self.hand_body]
        return transform

    def fingertip(self):
        return self.data.xpos[self.thumb_body] + self.data.xmat[self.thumb_body].reshape(3,3) @ self.config['thumb_contact_point_m']

    def step(self, thumb_target):
        self.data.ctrl[self.thumb_act] = np.clip(thumb_target, *self.limits.T)
        maximum_force, minimum_distance, energy = 0.0, 0.0, 0.0
        peak_pair = None
        force = np.zeros(6)
        for _ in range(self.substeps):
            self.mj.mj_step(self.model, self.data)
            energy += float(np.sum(np.abs(self.data.qfrc_actuator[self.vidx] * self.data.qvel[self.vidx]))) * self.model.opt.timestep
            for contact_id in range(self.data.ncon):
                contact = self.data.contact[contact_id]
                bodies = {int(self.model.geom_bodyid[g]) for g in contact.geom}
                if bodies & self.robot_bodies and bodies & self.gamepad_bodies:
                    self.mj.mj_contactForce(self.model, self.data, contact_id, force)
                    magnitude = float(np.linalg.norm(force[:3]))
                    if magnitude > maximum_force:
                        maximum_force = magnitude
                        peak_pair = [self.model.body(i).name for i in sorted(bodies)]
                    minimum_distance = min(minimum_distance, float(contact.dist))
        if not np.isfinite(self.data.qpos).all() or not np.isfinite(self.data.qvel).all():
            raise FloatingPointError('MuJoCo state became non-finite')
        bad = [self.mj.mjtWarning.mjWARN_BADQPOS, self.mj.mjtWarning.mjWARN_BADQVEL,
               self.mj.mjtWarning.mjWARN_BADQACC]
        if any(self.data.warning[int(w)].number for w in bad):
            raise FloatingPointError('MuJoCo reported a non-finite dynamics warning')
        self.mj.mj_forward(self.model, self.data)
        return {'max_contact_force_n': maximum_force, 'min_contact_distance_m': minimum_distance,
                'energy_j': energy, 'peak_contact_pair': peak_pair}

    def close(self):
        if self._temporary:
            self._temporary.cleanup()


class ThumbIK:
    """Geometry-based base commands; goals are positions, never gamepad state writes."""
    def __init__(self, config, workcell, hand_world):
        from scipy.optimize import least_squares
        self.chain = chain = SerialChain(workcell.left_hand.urdf, config['thumb_link'])
        if tuple(chain.active) != THUMB_JOINTS:
            raise ValueError('unexpected thumb joint order')
        lower, upper = chain.active_limits
        point = np.array(config['thumb_contact_point_m'])
        normal = np.array(config['thumb_contact_normal'])
        pad = np.linalg.inv(hand_world) @ pose_to_matrix(workcell.gamepad.root_pose)
        centre = pad[:3,:3] @ config['dpad_centre_m'] + pad[:3,3]
        n = pad[:3,2]
        offsets = [np.zeros(3), pad[:3,1], -pad[:3,1], -pad[:3,0], pad[:3,0]]
        self.goal_points = np.array([centre + config['press_radius_m'] * v +
                                    (config['hover_height_m'] if i == 0 else -config['press_depth_m']) * n
                                    for i,v in enumerate(offsets)])
        rng = np.random.default_rng(config['ik_seed'])
        samples = rng.uniform(lower, upper, size=(config['ik_samples'],len(lower)))
        frames = chain.forward(samples)
        positions = frames[:,:3,:3] @ point + frames[:,:3,3]
        normals = frames[:,:3,:3] @ normal
        self.commands, self.errors = [], []
        for target in self.goal_points:
            cost = np.linalg.norm(positions - target,axis=1) + .01 * np.maximum(0, normals @ n + .866)
            candidates = np.argsort(cost)[:4]
            def residual(q):
                frame = chain.forward(q)[0]
                return np.r_[frame[:3,:3] @ point + frame[:3,3] - target,
                             .01 * max(0., (frame[:3,:3] @ normal) @ n + .866)]
            fits = [least_squares(residual, samples[i], bounds=(lower,upper)) for i in candidates]
            best = min(fits, key=lambda fit: np.linalg.norm(fit.fun))
            self.commands.append(best.x)
            self.errors.append(float(np.linalg.norm(residual(best.x)[:3])))
        self.commands = np.array(self.commands)
        if config.get('consistent_ik', False):
            from scipy.optimize import minimize
            # A shared reference chooses the same local posture family for
            # hover and press. Position and the existing normal cone remain
            # constraints, rather than being traded for shorter joint motion.
            anchor = self.commands[1:].mean(axis=0)
            def point_at(q):
                frame = chain.forward(q)[0]
                return frame[:3,:3] @ point + frame[:3,3]
            def cone(q):
                return -(chain.forward(q)[0,:3,:3] @ normal) @ n - np.cos(np.deg2rad(30))
            refined = []
            for target,initial in zip(self.goal_points,self.commands):
                fit = minimize(lambda q: float(np.sum(((q-anchor)/(upper-lower))**2)), initial,
                    bounds=list(zip(lower,upper)), method='SLSQP',
                    constraints=[{'type':'eq','fun':lambda q,target=target:point_at(q)-target},
                                 {'type':'ineq','fun':cone}], options={'ftol':1e-12,'maxiter':200})
                if not fit.success:
                    raise ValueError(f'consistent thumb posture failed: {fit.message}')
                refined.append(fit.x)
            self.commands = np.array(refined)
            self.errors = [float(np.linalg.norm(point_at(q)-target))
                           for q,target in zip(self.commands,self.goal_points)]
        self.goal_points_world = self.goal_points @ hand_world[:3,:3].T + hand_world[:3,3]
        self.normal_world = hand_world[:3,:3] @ n
