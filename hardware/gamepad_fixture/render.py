#!/usr/bin/env python3
"""Deterministic software previews of the exported CAD, with source pad meshes.

Numpy triangle rasterization with a pixel depth buffer: no GPU or remote station.
Camera_3 uses its calibrated optical pose and pinhole intrinsics from the scene.
"""
from __future__ import annotations
import argparse
import json
import math
import os
from pathlib import Path

os.environ.setdefault("MPLCONFIGDIR", "/tmp/gamepad-fixture-matplotlib")
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.colors import to_rgb
from matplotlib.ticker import FuncFormatter
import numpy as np
from scipy.spatial.transform import Rotation
import trimesh
from asset_geometry import ControllerAsset, COLORS

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[1]
PALETTE = {"base_printed": "#23506b", "base_aluminum": "#9daab7", "column": "#356f89", "cradle_lower_jaw": "#1b8895",
           "slider_upper_jaw": "#dc9146", "guide_cap": "#dc9146", "M8_hand_knob": "#ad5539", "hirth_fixed": "#466b9c", "hirth_moving": "#458fa5"}


MATERIAL_PALETTE=json.loads((HERE/'materials_palette.json').read_text())

def material_color(material):
    key={'PETG':'PETG','Steel':'steel','Spring_steel':'steel','Aluminum_6061':'aluminum','TPU95A':'TPU','adhesive_silicone_sheet':'silicone'}.get(material,'EVA' if 'eva' in material.lower() else 'silicone')
    return MATERIAL_PALETTE[key]['color']

def color(name):
    return material_color('Steel' if name.startswith('hardware_') else 'adhesive_silicone_sheet' if name.startswith('pad_') else 'PETG')


def spring_mesh(x, z, y0, y1, outer_diameter, wire_diameter, coils):
    """Standard-hardware visual at installed length; no force inference."""
    angle=np.linspace(0,2*np.pi*coils,int(coils*24)+1)
    radius=(outer_diameter-wire_diameter)/2
    path=np.c_[x+radius*np.cos(angle),np.linspace(y0+wire_diameter/2,y1-wire_diameter/2,len(angle)),z+radius*np.sin(angle)]
    tangent=np.gradient(path,axis=0);tangent/=np.linalg.norm(tangent,axis=1)[:,None]
    normal=np.c_[np.cos(angle),np.zeros_like(angle),np.sin(angle)]
    binormal=np.cross(tangent,normal);binormal/=np.linalg.norm(binormal,axis=1)[:,None]
    circle=np.arange(8)*2*np.pi/8
    vertices=(path[:,None,:]+wire_diameter/2*(np.cos(circle)[None,:,None]*normal[:,None,:]+np.sin(circle)[None,:,None]*binormal[:,None,:])).reshape(-1,3)
    faces=[]
    for k in range(len(path)-1):
        for j in range(8):
            a,b=k*8+j,k*8+(j+1)%8;c,d=a+8,b+8;faces.extend([[a,b,c],[b,d,c]])
    return trimesh.Trimesh(vertices=vertices,faces=faces,process=False)


def lookat(eye, target, up=(0, 0, 1)):
    eye, target, up = np.asarray(eye, float), np.asarray(target, float), np.asarray(up, float)
    forward = (target-eye)/np.linalg.norm(target-eye)
    right = np.cross(forward, up); right /= np.linalg.norm(right)
    camera_up = np.cross(right, forward)
    rotation = np.vstack([right, -camera_up, forward])
    transform = np.eye(4); transform[:3, :3] = rotation; transform[:3, 3] = -rotation@eye
    return transform


def draw(ax, items, camera, perspective=False, intrinsics=None, title="", bounds=None, resolution=1500):
    polygons, colours, depths = [], [], []
    light = np.array([-.3, -.4, .85]); light /= np.linalg.norm(light)
    for name, mesh, rgb in items:
        triangles = mesh.triangles
        vertices = trimesh.transform_points(triangles.reshape(-1, 3), camera).reshape(-1, 3, 3)
        visible = vertices[:, :, 2].min(axis=1) > 1e-6
        vertices = vertices[visible]
        normals = mesh.face_normals[visible]
        shade = .49 + .51*np.maximum(normals@light, 0)
        base = np.array(to_rgb(rgb))
        shaded = base[None, :]*shade[:, None]
        if perspective:
            fx, fy, cx, cy = intrinsics
            projection = vertices[:, :, :2] / vertices[:, :, 2, None]
            projection[:, :, 0] = projection[:, :, 0]*fx+cx
            projection[:, :, 1] = projection[:, :, 1]*fy+cy
        else:
            projection = vertices[:, :, :2]
        polygons.extend(projection)
        colours.extend(shaded)
        depths.extend(vertices[:, :, 2].mean(axis=1))
    polygons, colours = np.asarray(polygons), np.asarray(colours)
    if bounds is None:
        lo, hi = polygons.min(axis=(0, 1)), polygons.max(axis=(0, 1))
        center = (lo+hi)/2; extent = (hi-lo)*.56
        bounds=[center[0]-extent[0],center[0]+extent[0],center[1]-extent[1],center[1]+extent[1]]
    # A real per-pixel depth buffer is essential: painter sorting exposes
    # internal CoACD triangles and incorrectly hides large coplanar CAD faces.
    width, height = bounds[1]-bounds[0], bounds[3]-bounds[2]
    factor=resolution/max(width,height);nx,ny=max(1,int(width*factor)),max(1,int(height*factor))
    canvas=np.empty((ny,nx,3),dtype=np.uint8);canvas[:]=[243,246,248]
    depthbuffer=np.full((ny,nx),np.inf)
    pixel=polygons.copy();pixel[:,:,0]=(pixel[:,:,0]-bounds[0])*factor;pixel[:,:,1]=(pixel[:,:,1]-bounds[2])*factor
    # Recompute per-vertex camera depths in the same item order.
    zvalues=[]
    for name,mesh,rgb in items:
        view=trimesh.transform_points(mesh.triangles.reshape(-1,3),camera).reshape(-1,3,3)
        zvalues.extend(view[view[:,:,2].min(axis=1)>1e-6,:,2])
    for tri,rgb,z in zip(pixel,colours,zvalues):
        low=np.maximum(np.floor(tri.min(axis=0)).astype(int),0);high=np.minimum(np.ceil(tri.max(axis=0)).astype(int),[nx-1,ny-1])
        if np.any(high<low):continue
        a,b,c=tri;den=(b[1]-c[1])*(a[0]-c[0])+(c[0]-b[0])*(a[1]-c[1])
        if abs(den)<1e-10:continue
        yy,xx=np.mgrid[low[1]:high[1]+1,low[0]:high[0]+1];xx=xx+.5;yy=yy+.5
        u=((b[1]-c[1])*(xx-c[0])+(c[0]-b[0])*(yy-c[1]))/den
        v=((c[1]-a[1])*(xx-c[0])+(a[0]-c[0])*(yy-c[1]))/den;w=1-u-v
        zz=1/(u/z[0]+v/z[1]+w/z[2]) if perspective else u*z[0]+v*z[1]+w*z[2]
        sub=depthbuffer[low[1]:high[1]+1,low[0]:high[0]+1]
        mask=(u>=-1e-7)&(v>=-1e-7)&(w>=-1e-7)&(zz<sub)
        sub[mask]=zz[mask];canvas[low[1]:high[1]+1,low[0]:high[0]+1][mask]=np.clip(rgb*255,0,255).astype(np.uint8)
    ax.imshow(canvas,extent=[bounds[0],bounds[1],bounds[3],bounds[2]],interpolation="antialiased")
    ax.set_xlim(bounds[:2]);ax.set_ylim(bounds[3],bounds[2])
    ax.set_aspect("equal"); ax.axis("off"); ax.set_title(title, loc="left", fontsize=16, fontweight="bold", color="#17394a", pad=15)


def save(fig, path, caption):
    fig.text(.04, .025, caption, fontsize=9, color="#526979")
    fig.savefig(path, dpi=175, facecolor="#f3f6f8", bbox_inches="tight", pad_inches=.22)
    plt.close(fig)


def clamp_wireframe(ax,manifest,camera,frame_to_view=None,clamp_index=None):
    """Project the exported parametric keepouts; these are not physical parts."""
    frame_to_view=np.eye(4) if frame_to_view is None else frame_to_view
    for entry in manifest.get("clamp_envelopes",[]):
        if clamp_index and entry["clamp_index"]!=clamp_index:continue
        if entry["kind"]=="spindle":continue
        lo,hi=np.asarray(entry["base_frame_bounds_mm"])
        access=entry["role"]=="operator_access"
        rgb="#B05B50" if access else "#718698"
        lines=[]
        if entry["kind"] in ("handle_rotation","operator_hand_rotation"):
            center=(lo+hi)/2;radius=(hi[0]-lo[0])/2;t=np.linspace(0,2*np.pi,49)
            for z in (lo[2],hi[2]):lines.append(np.c_[center[0]+radius*np.cos(t),center[1]+radius*np.sin(t),np.full_like(t,z)])
            for a in np.arange(4)*np.pi/2:
                lines.append(np.array([[center[0]+radius*np.cos(a),center[1]+radius*np.sin(a),z] for z in (lo[2],hi[2])]))
        else:
            import itertools
            corners=np.array(list(itertools.product(*zip(lo,hi))))
            for a in range(8):
                for b in range(a+1,8):
                    if sum(corners[a]!=corners[b])==1:lines.append(corners[[a,b]])
        for line in lines:
            points=trimesh.transform_points(trimesh.transform_points(line,frame_to_view),camera)
            ax.plot(points[:,0],points[:,1],color=rgb,lw=.55,ls="--" if access else ":",alpha=.75,zorder=6)


def world_side(ax,items_world,manifest,title="WORLD X / Z · Z LAYOUT",resolution=1700):
    """Absolute world-coordinate side projection, camera toward -world X."""
    from matplotlib import font_manager
    font_path=Path("/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc")
    font_name="DejaVu Sans"
    if font_path.exists():
        font_manager.fontManager.addfont(str(font_path))
        font_name=font_manager.FontProperties(fname=font_path).get_name()
        plt.rcParams["font.family"]=[font_name,"DejaVu Sans"]
    install=manifest["parameters"]["installation"]
    edge,table=install["table_edge_x"],install["tabletop_z"]
    camera=np.array([[1,0,0,0],[0,0,-1,0],[0,1,0,2000],[0,0,0,1]],float)
    bounds=[edge-205,edge+190,-(table+295),-(table-145)]
    draw(ax,items_world,camera,bounds=bounds,title=title,resolution=resolution)
    Fworld=np.eye(4);Fworld[:3,:3]=manifest["fixed_frame_world_axes"];Fworld[:3,3]=manifest["fixed_frame_world_origin_mm"]
    clamp_wireframe(ax,manifest,camera,Fworld,clamp_index=1)
    ax.axvline(edge,color="#8295A2",ls="--",lw=.65,zorder=5)
    ax.plot([edge-190,edge],[-table,-table],color="#8295A2",lw=1,zorder=4)
    ax.text(edge+3,-(table-113),f"桌子外沿 X={edge:g}",rotation=90,fontsize=6.4,color="#526979",va="bottom")
    ax.annotate("相机方向（−X）",xy=(edge-190,-(table+280)),xytext=(edge-108,-(table+280)),fontsize=7,color="#17394a",arrowprops={"arrowstyle":"->","color":"#526979","lw":.7},ha="center")
    ax.text(edge+34,-(table-87),"C夹可沿剩余平带自由放置\n接触深度 ≤30 mm（待测裙板）",fontsize=6.3,color="#A2574B")

    ax.axis("on");ax.set_facecolor("#f3f6f8")
    ax.set_xticks(np.arange(550,901,100));ax.set_yticks(-np.arange(650,1051,100))
    ax.yaxis.set_major_formatter(FuncFormatter(lambda value,_:f"{-value:.0f}"))
    ax.tick_params(labelsize=5.6,length=2);ax.set_xlabel("世界 X / mm",fontsize=6,fontfamily=font_name);ax.set_ylabel("世界 Z / mm",fontsize=6,fontfamily=font_name)
    for spine in ax.spines.values():spine.set_color("#CCD7DC")
    return camera

def main(out=HERE/'out_v3_1'):
    out=Path(out);dest=out/'previews';dest.mkdir(exist_ok=True)
    from matplotlib import font_manager
    f=Path('/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc')
    if f.exists():font_manager.fontManager.addfont(str(f));plt.rcParams['font.family']=[font_manager.FontProperties(fname=f).get_name(),'DejaVu Sans']
    mf=json.loads((out/'manifest.json').read_text());W=np.asarray(mf['world_from_pad'])
    meshes={p.stem:trimesh.load(p,force='mesh') for p in sorted((out/'assembly_mesh').glob('*.stl'))}
    visual=ControllerAsset().visual_meshes();metas={r['name']:r for r in mf['parts']};local=[(n,m,material_color(metas[n]['material'])) for n,m in meshes.items()]+[('controller_'+n,m,COLORS.get(n,'#67727d')) for n,m in visual.items()]
    world=[]
    for n,m,c in local:
        m=m.copy();m.apply_transform(W);world.append((n,m,c))
    target=W[:3,3]-[0,0,80]
    fig,ax=plt.subplots(figsize=(11,10),facecolor='#f3f6f8');draw(ax,world,lookat(target+[-420,-500,300],target),title='V3 / VERTICAL POST + SYMMETRIC FORK')
    save(fig,dest/'01_assembly_isometric.png','Actual CAD bodies + complete upstream visual mesh | 240 x 90 mm base | no pressure shoes')
    scene=json.loads((ROOT/mf['source_scene']).read_text());cam=next(c for c in scene['cameras'] if c['name']=='camera_3');p=cam['pose_optical'];C=np.eye(4);C[:3,:3]=Rotation.from_quat(p[4:]+p[3:4]).as_matrix();C[:3,3]=np.asarray(p[:3])*1000;k=cam['intrinsics']
    fig,ax=plt.subplots(figsize=(12.8,7.2),facecolor='#f3f6f8');draw(ax,world,np.linalg.inv(C),True,[k[s] for s in ('fx','fy','cx','cy')],title='V3 / CALIBRATED CAMERA 3 - CAD CROSS-CHECK',bounds=[0,1280,0,720],resolution=1600)
    save(fig,dest/'02_camera_3.png','Software CAD preview. Actual SAPIEN operator_view render and segmentation are in the V3 report.')
    offsets={'base_printed':[0,0,-65],'base_aluminum':[0,0,-65],'column':[-85,0,-20],'hirth_fixed':[-60,0,0],'hirth_moving':[-30,0,0], 'slider_upper_jaw':[0,45,10],'guide_cap':[0,80,0],'M8_hand_knob':[85,0,0], 'pad_back':[0,0,25],'pad_upper_jaw':[0,65,20],'pad_lower_jaw':[0,-40,20]}
    exploded=[]
    for n,m,c in local:
        m=m.copy();m.apply_translation(offsets.get(n,[0,0,0]));exploded.append((n,m,c))
    fig,ax=plt.subplots(figsize=(12,10),facecolor='#f3f6f8');draw(ax,exploded,lookat([-430,-400,260],[0,5,-40],up=(0,1,0)),title='V3 / EXPLODED ASSEMBLY')
    save(fig,dest/'03_exploded.png','M8 double shear axis; left removable Hirth pair; right sliding metal sleeve; 2 mm sheet pads')
    # Plot exact CAD sections, without triangulated mesh-fit interfaces.
    fig,axs=plt.subplots(1,3,figsize=(17,7),facecolor='#f3f6f8')
    for ax,(axis,origin,cols,title) in zip(axs,[(0,[0,0,0],[2,1],'CENTRE SECTION / Z-Y'),(1,[0,mf['nominal_span_mm']*0+mf['parameters']['measured']['top_center_y']+.8,0],[0,2],'UPPER PAD / PAIR + USB RELIEF'),(2,[0,0,-10.9],[0,1],'BACKREST / REGULAR PLANE')]):
        for n,m,c in local:
            sec=m.section(plane_origin=origin,plane_normal=np.eye(3)[axis])
            if sec:
                for loop in sec.discrete:ax.plot(loop[:,cols[0]],loop[:,cols[1]],color=c,lw=.7)
        ax.set_aspect('equal');ax.grid(alpha=.15);ax.set_title(title,fontsize=10)
    axs[0].set_xlim(-70,32);axs[0].set_ylim(-65,82);axs[1].set_xlim(-25,25);axs[1].set_ylim(-12,28);axs[2].set_xlim(-32,32);axs[2].set_ylim(5,42)
    save(fig,dest/'04_sections.png','Contact faces are planes. Openings are bridged/relieved; front lips intentionally remain at z=22..26 mm.')
    printed=[r for r in mf['parts'] if r['is_printed']]
    fig,axs=plt.subplots(math.ceil(len(printed)/4),4,figsize=(16,4*math.ceil(len(printed)/4)),facecolor='#f3f6f8')
    for ax,r in zip(axs.ravel(),printed):
        m=trimesh.load(out/'print'/(r['name']+'.stl'),force='mesh');b=m.bounds;ctr=b.mean(0);draw(ax,[(r['name'],m,material_color(r['material']))],lookat(ctr+[250,-350,300],ctr),title=r['name'],resolution=450);ax.set_title(r['name']+'\n'+' x '.join(f'{x:.1f}' for x in r['print_bounds_mm'])+' mm',fontsize=9)
    for ax in axs.ravel()[len(printed):]:ax.axis('off')
    save(fig,dest/'05_print_layout.png','Individual recommended orientations, not a shared plate packing. H2D dual-nozzle limit 300 x 320 x 325 mm.')
    fig,axs=plt.subplots(1,2,figsize=(14,7),facecolor='#f3f6f8')
    for ax,name in zip(axs,['hirth_fixed','hirth_moving']):
        m=meshes[name];ctr=m.centroid;draw(ax,[(name,m,color(name))],lookat(ctr+([-120,120,90] if name=='hirth_fixed' else [120,120,90]),ctr,up=(0,1,0)),title=name+' / 36 TEETH / 10 DEG')
    save(fig,dest/'06_hirth_detail.png','r=18..30 mm; constant 60 deg loaded flanks; fixed tip/root flats 1.1/.9 mm; .2 mm tip/root relief; no pointer or numbers.')
    fig,ax=plt.subplots(figsize=(11,9),facecolor='#f3f6f8');world_side(ax,world,mf,title='V3 / WORLD X-Z / VERTICAL POST')
    ax.text(750,-1020,f"root depth = {718.4-W[0,3]:.2f} mm\nheight = {W[2,3]-767:.2f} mm\ntilt = {mf['key_angle_deg']:.1f} deg",fontsize=8)
    save(fig,dest/'07_world_z_layout.png','Base / vertical post / cradle form the Z silhouette. Pose is derived from these dimensions, not a target root input.')
    # One shared offset chain for all three analytic contact seats.
    fig,axs=plt.subplots(1,3,figsize=(15,5),facecolor='#f3f6f8')
    for ax,c in zip(axs,mf['contact_planes']):
        for y,txt,col in [(0,'fitted shell / compressed pad face','#3c5969'),(c['rim_offset_mm']-c['fit_offset_mm'],'rigid rim','#267c92'),(c['compressed_thickness_mm'],'pad seat','#dc9146')]:ax.axhline(y,color=col,lw=3);ax.text(.05,y+.06,txt,color=col,fontsize=9)
        ax.fill_between([.2,.8],0,c['compressed_thickness_mm'],color='#bd9fc9',alpha=.6)
        ax.set_ylim(-.25,2.5);ax.set_xlim(0,1);ax.set_xticks([]);ax.set_ylabel('offset toward fixture / mm');ax.set_title(c['name']+f" / free {c['free_thickness_mm']:.2f} - compression {c['compression_mm']:.2f} = {c['compressed_thickness_mm']:.2f} mm",fontsize=9)
        ax.text(.05,2.25,'recess 1.50 mm; free protrusion 0.50 mm',fontsize=9)
    save(fig,dest/'08_pad_offsets.png','Nominal plane-chain check is distinct from artist-shell fit residual and real silicone compressibility.')
    return dest

if __name__=='__main__':
    import math
    p=argparse.ArgumentParser();p.add_argument('--out',type=Path,default=HERE/'out_v3_1');a=p.parse_args();main(a.out)
