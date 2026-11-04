"""AD offline experiment. Run on station workstation Python; never initializes ROS."""
import copy
import hashlib
import importlib.util
import json
import os
from pathlib import Path
import sys
import time
import urllib.request
import xml.etree.ElementTree as ET

import numpy as np
from scipy.spatial.transform import Rotation
from rosidl_runtime_py.set_message import set_message_fields
from rosidl_runtime_py.convert import message_to_ordereddict
from manus_ros2_msgs.msg import ManusGlove

OUT = Path(__file__).resolve().parent
STATION = Path.home() / 'gamepad_vla/station_ws'
SRC = STATION / 'src/linkerhand_retarget/linkerhand_retarget'
sys.path.insert(0, str(SRC))
from linkerhand.config import HandConfig
from linkerhand.handcore import HandCore
from linkerhand_retarget.motion.manus_dex import retarget as R


def status():
    with urllib.request.urlopen('http://127.0.0.1:8780/api/v1/status', timeout=3) as response:
        state = json.load(response)
    if state.get('recording_status', {}).get('state') != 'idle':
        raise RuntimeError('Recording is not idle; abort offline computation')
    return {k: state.get(k) for k in ('session_id', 'session_state', 'recording_status', 'revision')}


def load_mapping(name):
    spec = importlib.util.spec_from_file_location(name, OUT / 'mapping_a.py')
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


class NoROS:
    """Use actual Retarget constructor without creating ROS entities."""
    def create_publisher(self, *a, **kw):
        return None
    def create_subscription(self, *a, **kw):
        return None


def skeleton(template, opposition, abduction, finger_swing):
    """Ideal open human hand in normalized palm frame; measured template bone lengths."""
    msg = ManusGlove()
    set_message_fields(msg, copy.deepcopy(template))
    msg.side = 'Left'
    points = {}
    for node in msg.raw_nodes:
        points[(node.chain_type, node.joint_type)] = np.array([getattr(node.pose.position, a) for a in 'xyz'])
    canonical = {('Hand', 'Invalid'): np.zeros(3)}
    for i, finger in enumerate(('Index', 'Middle', 'Ring', 'Pinky')):
        joints = ('MCP', 'PIP', 'IP', 'DIP', 'TIP')
        base = np.array([0., (-.026, -.009, .009, .026)[i], (.070, .076, .072, .065)[i]])
        canonical[(finger, 'MCP')] = base
        rot = Rotation.from_rotvec(np.array([1., 0., 0.]) * np.deg2rad(finger_swing)).as_matrix()
        for prev, joint in zip(joints, joints[1:]):
            length = np.linalg.norm(points[(finger, joint)] - points[(finger, prev)])
            canonical[(finger, joint)] = canonical[(finger, prev)] + rot @ np.array([0., 0., length])
    # Thumb MCP is the first raw node (CMC anchor in this controlled construction).
    base = np.array([.008, -.028, .025])
    canonical[('Thumb', 'MCP')] = base
    direction = np.array([.18, -.75, .64]); direction /= np.linalg.norm(direction)
    rot = Rotation.from_rotvec([0., 0., np.deg2rad(abduction)]).as_matrix() @ Rotation.from_rotvec([-np.deg2rad(opposition), 0., 0.]).as_matrix()
    for prev, joint in zip(('MCP', 'PIP', 'DIP'), ('PIP', 'DIP', 'TIP')):
        length = np.linalg.norm(points[('Thumb', joint)] - points[('Thumb', prev)])
        canonical[('Thumb', joint)] = canonical[('Thumb', prev)] + rot @ direction * length
    for node in msg.raw_nodes:
        p = canonical[(node.chain_type, node.joint_type)]
        for a, v in zip('xyz', p): setattr(node.pose.position, a, float(v))
        # Synthetic orientations are proper quaternions; current reference uses positions only.
        q = Rotation.from_matrix(rot if node.chain_type == 'Thumb' else np.eye(3)).as_quat()
        for a, v in zip('xyzw', q): setattr(node.pose.orientation, a, float(v))
    for ergo in msg.ergonomics:
        ergo.value = float({'ThumbMCPSpread': abduction, 'ThumbMCPStretch': opposition}.get(ergo.type, finger_swing if ergo.type.endswith('Spread') else 0.))
    msg.raw_sensor_count = 0
    msg.raw_sensor = []
    msg.raw_sensor_orientation.x = msg.raw_sensor_orientation.y = msg.raw_sensor_orientation.z = 0.
    msg.raw_sensor_orientation.w = 1.
    return msg


def mirror(msg):
    msg = copy.deepcopy(msg)
    msg.side = 'Right'
    S = np.diag([1., -1., 1.])
    for node in msg.raw_nodes:
        node.pose.position.y *= -1.
        q = np.array([getattr(node.pose.orientation, a) for a in 'xyzw'])
        q = Rotation.from_matrix(S @ Rotation.from_quat(q).as_matrix() @ S).as_quat()
        for a, v in zip('xyzw', q): setattr(node.pose.orientation, a, float(v))
    # Ergonomics are anatomical scalars (same flexion/spread amount on mirrored hands).
    return msg



def disk_only(template, roll_deg):
    """Additional FK calibration input: vary ONLY right CMC roll, then mirror human points.

    Human bone lengths here are robot lengths / scaling_factor, to isolate the
    CMC degree of freedom from anatomical mismatch in the main human sweep.
    """
    msg = ManusGlove()
    set_message_fields(msg, copy.deepcopy(template))
    root = ET.parse(SRC / 'assets/robots/hands/linker_hand/g20_right/linkerhand_g20_right.urdf').getroot()
    joints = {j.find('child').get('link'): j for j in root.findall('joint')}
    qs = {'thumb_cmc_roll': np.deg2rad(roll_deg), 'thumb_cmc_yaw': .6, 'thumb_cmc_pitch': .25, 'thumb_mcp': .15, 'thumb_ip': .15*1.03}
    cache = {}
    def T(link):
        if link in cache: return cache[link]
        j = joints.get(link)
        if j is None: cache[link] = np.eye(4); return cache[link]
        o=j.find('origin'); A=np.eye(4)
        if o is not None:
            A[:3,3]=np.fromstring(o.get('xyz','0 0 0'),sep=' ')
            A[:3,:3]=Rotation.from_euler('xyz',np.fromstring(o.get('rpy','0 0 0'),sep=' ')).as_matrix()
        A=T(j.find('parent').get('link'))@A
        if j.get('type') in ('revolute','continuous'):
            a=np.fromstring(j.find('axis').get('xyz'),sep=' '); B=np.eye(4)
            B[:3,:3]=Rotation.from_rotvec(a/np.linalg.norm(a)*qs.get(j.get('name'),0.)).as_matrix();A=A@B
        cache[link]=A;return A
    # Build right human reference first; local palm normalization is identity.
    msg.side='Right'
    for node in msg.raw_nodes:
        f,j=node.chain_type.lower(),node.joint_type
        if f=='hand': p=np.zeros(3)
        else:
            link=({'MCP':'metacarpals','PIP':'proximal','DIP':'distal','TIP':'distal'} if f=='thumb' else {'MCP':'metacarpals','PIP':'proximal','IP':'middle','DIP':'distal','TIP':'distal'})[j]
            M=T(f+'_'+link);p=M[:3,3].copy()
            if j=='TIP':
                tip=[-.004367715,0,.028888742,1.] if f=='thumb' else [-.012293405,0,.021822799,1.]
                p=(M@tip)[:3]
            if f!='thumb': p[0]=0. # Palm anchors make normalization identity, independent of thumb.
            p=p/1.5
        for a,v in zip('xyz',p):setattr(node.pose.position,a,float(v))
        node.pose.orientation.x=node.pose.orientation.y=node.pose.orientation.z=0.;node.pose.orientation.w=1.
    for e in msg.ergonomics:e.value=float(roll_deg if e.type=='ThumbMCPStretch' else (np.rad2deg(.6) if e.type=='ThumbMCPSpread' else 0.))
    msg.raw_sensor_count=0;msg.raw_sensor=[]
    # mirror() returns Right by convention; set the true side after reflection.
    left=mirror(msg);left.side='Left'
    return left


def main():
    started = time.monotonic()
    first_status = status()
    assert os.environ.get('CUDA_VISIBLE_DEVICES') == ''
    assert all(os.environ.get(n) == '1' for n in ('OMP_NUM_THREADS', 'OPENBLAS_NUM_THREADS', 'MKL_NUM_THREADS'))
    template = json.loads((OUT / 'template.json').read_text())
    a, b = load_mapping('ad_mapping_a'), load_mapping('ad_mapping_b')
    b.G20_COMMAND_LOWER['left'][5:10] = [0] * 5
    b.G20_COMMAND_UPPER['left'][5:10] = [255] * 5
    config = HandConfig(str(SRC / 'assets/robots/hands'), str(SRC))
    core = HandCore(config)
    bridge = R.Retarget(NoROS(), handcore=core, baseconfig=config.baseconfig)
    slots = R.RIGHT_SLOT_FROM_JOINT
    names = a.G20_JOINT_NAMES
    cases = [('opposition', angle, angle, 0., 0.) for angle in (-30., 0., 30., 60.)]
    cases += [('abduction', angle, 0., angle, 0.) for angle in (-30., 0., 30., 60.)]
    cases += [('disk_only', angle, angle, 0., 0.) for angle in (0., 25., 50., 75.)]
    cases += [('finger_swing', angle, 0., 0., angle) for angle in (-12., 0., 12.)]
    output = {'initial_status': first_status, 'cases': [], 'module': R.__file__, 'handcore_module': sys.modules[HandCore.__module__].__file__, 'config': config.baseconfig['manus_dex'], 'joint_names': names, 'slots': slots, 'hashes': {}}
    paths = [Path(R.__file__), Path(sys.modules[HandCore.__module__].__file__), SRC/'config/base_config.yml', SRC/'config/hand_config.yml', OUT/'mapping_a.py']
    for side in ('left', 'right'):
        paths += [Path(R.__file__).parent/'configs'/config.baseconfig['manus_dex'][side+'_config'], SRC/f'assets/robots/hands/linker_hand/g20_{side}/linkerhand_g20_{side}.urdf', Path(R.__file__).parents[2]/f'assets/robots/hands/linker_hand/g20_{side}/linkerhand_g20_{side}.urdf']
    for path in paths:
        output['hashes'][str(path).replace(str(Path.home()), '~')] = hashlib.sha256(path.read_bytes()).hexdigest()
    for category, angle, opposition, abduction, swing in cases:
        status()
        left = disk_only(template, angle) if category == 'disk_only' else skeleton(template, opposition, abduction, swing)
        right = mirror(left)
        lp = np.array([[getattr(n.pose.position, c) for c in 'xyz'] for n in left.raw_nodes])
        rp = np.array([[getattr(n.pose.position, c) for c in 'xyz'] for n in right.raw_nodes])
        assert np.max(abs(lp * [1, -1, 1] - rp)) < 1e-12
        case = {'category': category, 'angle_deg': angle, 'mirror_residual_m': float(np.max(abs(lp * [1, -1, 1] - rp))), 'hands': {}}
        for hand, msg in (('left', left), ('right', right)):
            rt = bridge.retargeting[hand]
            rt.reset()
            # Independent pose, warm up fixed target to remove optimizer history as a confound.
            history = []
            for _ in range(40):
                status()
                q = bridge._retarget_glove(msg, hand)
                history.append(np.array(q))
            assert q is not None
            ref = R.make_reference(rt, msg.raw_nodes, hand, bridge.position_scale, bridge.normalize_palm_frame)
            results = {}
            for label, mapper in (('a', a), ('b', b), ('c', b)):
                logical = q.copy()
                if label == 'c' and hand == 'left':
                    lo, hi = mapper.G20_JOINT_LIMITS[hand]['thumb_cmc_roll']
                    logical[slots['thumb_cmc_roll']] = lo + hi - logical[slots['thumb_cmc_roll']]
                motor = getattr(core, 'trans_to_motor_' + hand)(logical)
                # Fresh static command is the exact _apply_smooth first-frame behavior.
                bridge.smooth_positions[hand] = None
                motor = bridge._apply_smooth(hand, motor)
                recovered = mapper.motor_positions_to_joint_positions(hand, motor)
                ordered = [float(logical[slots[n]]) for n in names]
                results[label] = {'retarget_q': ordered, 'motor': motor, 'mapped_q': recovered, 'delta_q': (np.array(recovered)-ordered).tolist()}
            case['hands'][hand] = {'message': message_to_ordereddict(msg), 'reference': ref.tolist(), 'optimizer_q': [float(q[slots[n]]) for n in names], 'settle_max_last_step_rad': float(np.max(abs(history[-1]-history[-2]))), 'controls': results}
        output['cases'].append(case)
        print(category, angle, {h: {c: [round(v, 4) for v in case['hands'][h]['controls'][c]['mapped_q'][:2]] for c in ('a','b','c')} for h in ('left','right')}, flush=True)
    output['elapsed_s'] = time.monotonic() - started
    output['final_status'] = status()
    (OUT / 'results.json').write_text(json.dumps(output, indent=2))
    print('DONE', output['elapsed_s'], flush=True)


if __name__ == '__main__':
    main()
