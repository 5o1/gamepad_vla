"""AD figures: reuse fixture render.draw and thumb_sweep.py URDF FK; CPU only."""
import json
import signal
import hashlib
import sys
import time
import urllib.request
import xml.etree.ElementTree as ET
from pathlib import Path

import numpy as np
from scipy.spatial.transform import Rotation
import trimesh
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

OUT = Path(__file__).resolve().parent
CODE = Path.home() / 'gamepad_vla/code'
sys.path.insert(0, str(CODE / 'hardware/gamepad_fixture'))
import render as R

ASSET = Path.home() / 'gamepad_vla/assets/linkerhand_station_20261014/linkerhand_g20_l20_l25_urdf_mesh'
COLORS = {'Thumb': '#d66a20', 'Index': '#208d8a', 'Middle': '#2984b7', 'Ring': '#8968b2', 'Pinky': '#b44e7c', 'Hand': '#53636e'}


def check_idle():
    with urllib.request.urlopen('http://127.0.0.1:8780/api/v1/status', timeout=3) as response:
        state = json.load(response)
    if state.get('recording_status', {}).get('state') != 'idle':
        raise RuntimeError('Recording not idle; stop rendering')


def tf(el):
    T = np.eye(4)
    o = el.find('origin')
    if o is not None:
        T[:3, 3] = [float(v) for v in o.get('xyz', '0 0 0').split()]
        T[:3, :3] = Rotation.from_euler('xyz', [float(v) for v in o.get('rpy', '0 0 0').split()]).as_matrix()
    return T


class Hand:
    def __init__(self, side):
        self.path = ASSET / f'g20_{side}/linkerhand_g20_{side}.urdf'
        self.root = ET.parse(self.path).getroot()
        self.joints = {j.find('child').get('link'): j for j in self.root.findall('joint')}
        self.meshes = []
        for link in self.root.findall('link'):
            vis = link.find('visual')
            if vis is None or vis.find('geometry/mesh') is None: continue
            mesh = trimesh.load(self.path.parent / vis.find('geometry/mesh').get('filename'), force='mesh')
            scale = np.fromstring(vis.find('geometry/mesh').get('scale','1 1 1'), sep=' ')
            mesh.vertices *= scale
            self.meshes.append((link.get('name'), mesh, tf(vis)))

    def pose(self, q):
        cache = {}
        def T(link):
            if link in cache: return cache[link]
            joint = self.joints.get(link)
            if joint is None: cache[link] = np.eye(4); return cache[link]
            M = T(joint.find('parent').get('link')) @ tf(joint)
            if joint.get('type') in ('revolute','continuous'):
                a = np.fromstring(joint.find('axis').get('xyz'), sep=' ')
                a /= np.linalg.norm(a)
                A = np.eye(4); A[:3,:3] = Rotation.from_rotvec(a * q.get(joint.get('name'), 0.)).as_matrix()
                M = M @ A
            cache[link] = M
            return M
        items = []
        for name, mesh, visual in self.meshes:
            m = mesh.copy(); m.apply_transform(T(name) @ visual)
            finger = name.split('_')[0].title()
            items.append((name, m, COLORS.get(finger, '#a1adb5')))
        chains = {}
        for finger in ('thumb','index','middle','ring','pinky'):
            chain = ([finger+'_metacarpals_base1',finger+'_metacarpals',finger+'_proximal',finger+'_distal'] if finger=='thumb' else [finger+'_metacarpals',finger+'_proximal',finger+'_middle',finger+'_distal'])
            points = [T(link)[:3,3] for link in chain]
            # Fixed endpoint, identical to retarget asset's added tip landmark.
            tip = np.array([-.004367715,0,.028888742,1.]) if finger=='thumb' else np.array([-.012293405,0,.021822799,1.])
            points.append((T(finger+'_distal') @ tip)[:3])
            chains[finger.title()] = np.array(points)
        disc = T('thumb_metacarpals_base1')
        theta = np.linspace(0., 2*np.pi, 65)
        ring = np.c_[np.full_like(theta,.012), .014*np.sin(theta), .014*np.cos(theta), np.ones_like(theta)]
        chains['_disc_ring'] = (ring @ disc.T)[:,:3]
        chains['_disc_mark'] = np.array([(disc @ [.012,0.,0.,1.])[:3], (disc @ [.012,0.,.014,1.])[:3]])
        return items, chains


def project(points, camera):
    points = np.asarray(points)
    return (points @ camera[:3,:3].T + camera[:3,3])[:,:2]


def human_chains(hand):
    nodes = hand['message']['raw_nodes']
    p = lambda n: np.array([n['pose']['position'][a] for a in 'xyz'])
    root = next(p(n) for n in nodes if n['chain_type']=='Hand')
    return {f: np.vstack([root]+[p(n) for n in nodes if n['chain_type']==f]) for f in ('Thumb','Index','Middle','Ring','Pinky')}


def draw_chains(ax, chains, camera, neutral, mesh=False, target='Thumb'):
    for finger, pts in chains.items():
        if finger.startswith('_'): continue
        now = project(pts, camera); ghost = project(neutral[finger], camera)
        ax.plot(ghost[:,0],ghost[:,1],color='#7e8a95',lw=1.2,ls='--',alpha=.65,zorder=6)
        if not mesh:
            ax.plot(now[:,0],now[:,1],'-o',color=COLORS[finger],lw=3 if finger=='Thumb' else 2,ms=4,zorder=7)
        if finger == target or target == 'all':
            start, end = ghost[-1], now[-1]
            if np.linalg.norm(end-start)>1e-3:
                ax.annotate('',xy=end,xytext=start,arrowprops={'arrowstyle':'->','color':COLORS[finger],'lw':2.6},zorder=9)
            ax.scatter(*end,s=27,c=COLORS[finger],edgecolor='white',linewidth=.7,zorder=9)



def disc_overlay(ax, chains, zero, camera):
    ring = project(chains['_disc_ring'],camera)
    mark = project(chains['_disc_mark'],camera)
    old = project(zero['_disc_mark'],camera)
    ax.plot(ring[:,0],ring[:,1],color='#f5c23b',lw=1.1,zorder=12)
    ax.plot(mark[:,0],mark[:,1],color='#fff13b',lw=2.4,zorder=13)
    ax.scatter(*mark[1],c='#fff13b',edgecolor='#312b03',s=45,zorder=14)
    ax.scatter(*old[1],facecolor='none',edgecolor='#4c4c4c',s=35,zorder=14)
    ax.annotate('m5 disc',xy=mark[1],xytext=(mark[1,0]+.031,mark[1,1]+.012),fontsize=8,color='#553d00',arrowprops={'arrowstyle':'-','color':'#74560c'},zorder=14)


def dial(ax, value, base, side, text):
    # Front view of the physical x-axis: left and right URDF roll axes are opposite.
    ins=ax.inset_axes([.64,.69,.33,.25]);ins.set_aspect('equal');ins.axis('off')
    a=np.linspace(0,2*np.pi,100);ins.plot(np.cos(a),np.sin(a),color='#9eabb4',lw=1)
    sign=-1 if side=='left' else 1
    f=lambda q: np.array([np.sin(sign*q),np.cos(sign*q)])
    p0,p=f(base),f(value)
    ins.plot([0,p0[0]],[0,p0[1]],'--',color='#8c969e',lw=1.5)
    ins.plot([0,p[0]],[0,p[1]],color='#d66a20',lw=3)
    ins.scatter(*p,s=30,color='#f3cb28',edgecolor='#4a410d',zorder=4)
    if abs(value-base)>.01:
        t=np.linspace(sign*base,sign*value,40);xy=np.c_[.8*np.sin(t),.8*np.cos(t)]
        ins.plot(xy[:,0],xy[:,1],color='#d66a20',lw=1.6)
        ins.annotate('',xy=xy[-1],xytext=xy[-4],arrowprops={'arrowstyle':'->','color':'#d66a20','lw':1.6})
    ins.set_xlim(-1.3,1.3);ins.set_ylim(-1.2,1.45);ins.set_title(text,fontsize=8,pad=1)


def main():
    check_idle()
    def monitor(signum, frame):
        check_idle()
        signal.alarm(1)
    signal.signal(signal.SIGALRM, monitor)
    signal.alarm(1)
    category = sys.argv[1]
    label = sys.argv[2]
    data = json.loads((OUT/'results.json').read_text())
    models = {side: Hand(side) for side in ('left','right')}
    cases = [c for c in data['cases'] if c['category']==category]
    neutral = next(c for c in cases if c['angle_deg']==0.)
    (OUT/'images').mkdir(exist_ok=True)
    t0 = time.monotonic()
    for case in cases:
        check_idle()
        angle = case['angle_deg']
        fig, axes = plt.subplots(2,2,figsize=(16,14),facecolor='#f3f6f8')
        for row, side in enumerate(('left','right')):
            hand, zero = case['hands'][side], neutral['hands'][side]
            human, human_zero = human_chains(hand), human_chains(zero)
            vals, vals0 = hand['controls'][label], zero['controls'][label]
            items, chains = models[side].pose(dict(zip(data['joint_names'], vals['mapped_q'])))
            _, chains0 = models[side].pose(dict(zip(data['joint_names'], vals0['mapped_q'])))
            # Same world camera axes for input and output, fixed across all angles and hands.
            eye_offset = np.array([.40,.13,.035])
            center_h,center_r=np.array([.005,0,.12]),np.array([.015,0,.14])
            cam_h=R.lookat(center_h+eye_offset,center_h)
            cam_r=R.lookat(center_r+eye_offset,center_r)
            bounds=[-.17,.17,-.18,.18]
            ax=axes[row,0]
            ax.set_facecolor('#f3f6f8');ax.set_xlim(bounds[:2]);ax.set_ylim(bounds[3],bounds[2]);ax.set_aspect('equal');ax.axis('off')
            ax.set_title(f'{side.upper()} / SYNTHETIC MANUS',loc='left',fontsize=15,fontweight='bold')
            target='all' if category=='finger_swing' else 'Thumb'
            draw_chains(ax,human,cam_h,human_zero,target=target)
            if category in ('opposition','disk_only'):
                dial(ax,np.deg2rad(angle),0.,side,'Input CMC turn / front view')
            ax.text(.01,-.08,f'{category} = {angle:+.0f} deg\nDashed: 0 deg input; arrow: tip displacement\nPalm normal +X; across +Y; forward +Z',transform=ax.transAxes,fontsize=9,va='bottom')
            ax=axes[row,1]
            cache_dir=OUT/'render-cache';cache_dir.mkdir(exist_ok=True)
            key=hashlib.sha256((side+json.dumps(bounds)+json.dumps(vals['mapped_q'])).encode()).hexdigest()[:24]
            cached=cache_dir/(key+'.npz')
            if cached.exists():
                with np.load(cached) as f: canvas=f['canvas'];extent=f['extent']
                ax.imshow(canvas,extent=extent,interpolation='antialiased')
                ax.set_xlim(bounds[:2]);ax.set_ylim(bounds[3],bounds[2]);ax.set_aspect('equal');ax.axis('off')
                ax.set_title(f'{side.upper()} / G20 / CONTROL {label.upper()}',loc='left',fontsize=16,fontweight='bold',color='#17394a',pad=15)
            else:
                R.draw(ax,items,cam_r,title=f'{side.upper()} / G20 / CONTROL {label.upper()}',bounds=bounds,resolution=450)
                np.savez_compressed(cached,canvas=np.asarray(ax.images[0].get_array()),extent=ax.images[0].get_extent())
            draw_chains(ax,chains,cam_r,chains0,mesh=True,target=target)
            disc_overlay(ax,chains,chains0,cam_r)
            if category in ('opposition','disk_only'):
                dial(ax,vals['mapped_q'][0],vals0['mapped_q'][0],side,'m5 disc / same front view')
            motors=(5,10) if category!='finger_swing' else (6,7,8,9)
            indices=(0,1) if category!='finger_swing' else (5,9,13,17)
            text=['motor    q emit -> cmd -> q sim       delta (rad)']
            if label=='c' and side=='left':
                text.insert(0,f'Before C: raw roll = {hand["optimizer_q"][0]:+.3f}; emit = 1.4 - raw')
            for m,i in zip(motors,indices):
                text.append(f'm{m:02d}   {vals["retarget_q"][i]:+.3f} -> {vals["motor"][m]:3d} -> {vals["mapped_q"][i]:+.3f}   {vals["delta_q"][i]:+.3f}')
            ax.text(.01,-.08,'\n'.join(text),transform=ax.transAxes,fontsize=8.2,fontfamily='monospace',va='bottom',bbox={'facecolor':'white','alpha':.9,'edgecolor':'#d8e0e5'})
        descriptions={'a':'current mapping: left m5-9 255 -> 0; thumb_ergo off','b':'inverse mapping: left m5-9 0 -> 255; thumb_ergo off','c':'B + left thumb roll range reflection in retarget output'}
        fig.suptitle(f'{category.upper()} {angle:+.0f} deg | {label.upper()}: {descriptions[label]}',fontsize=15,fontweight='bold',y=.985)
        fig.text(.08,.023,'Strict mirrored human inputs. Same camera axes. Orange = thumb. Static URDF FK, no dynamics or hardware commands.',fontsize=10)
        fig.subplots_adjust(top=.93,bottom=.10,hspace=.33,wspace=.08)
        tag=f'p{int(angle):03d}' if angle>=0 else f'm{int(-angle):03d}'
        path=OUT/'images'/f'{category}_{tag}_{label}.png'
        fig.savefig(path,dpi=110,facecolor=fig.get_facecolor());plt.close(fig)
        print(path.name,round(time.monotonic()-t0,2),flush=True)
    check_idle()


if __name__=='__main__':
    try:
        main()
    finally:
        signal.alarm(0)
