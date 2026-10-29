#!/usr/bin/env python3
"""Deterministic software previews of the exported CAD, with source pad meshes.

Numpy triangle rasterization with a pixel depth buffer: no GPU or remote station.
Camera_3 uses its calibrated optical pose and pinhole intrinsics from the scene.
"""
from __future__ import annotations
import argparse
import json
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
           "slider_upper_jaw": "#dc9146", "guide_cap": "#dc9146", "M6_hand_knob": "#ad5539"}


def color(name):
    if name.startswith("pad_"):
        return "#5d4e68"
    if name.startswith("hardware_"):
        return "#a3b1bf"
    if name.startswith("guide_rod"):
        return "#597b89"
    return PALETTE.get(name, "#3a7187")


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
    ax.text(edge+34,-(table-87),"R50 手柄 / R75 操作手\n桌下操作，依次调整双夹",fontsize=6.3,color="#A2574B")
    ax.text(edge+54,-(table+83),"C1/C2 包络投影\n离中线 ±70 mm",fontsize=6.3,color="#526979")
    ax.axis("on");ax.set_facecolor("#f3f6f8")
    ax.set_xticks(np.arange(550,901,100));ax.set_yticks(-np.arange(650,1051,100))
    ax.yaxis.set_major_formatter(FuncFormatter(lambda value,_:f"{-value:.0f}"))
    ax.tick_params(labelsize=5.6,length=2);ax.set_xlabel("世界 X / mm",fontsize=6,fontfamily=font_name);ax.set_ylabel("世界 Z / mm",fontsize=6,fontfamily=font_name)
    for spine in ax.spines.values():spine.set_color("#CCD7DC")
    return camera


def main(out=HERE / "out"):
    out = Path(out); dest=out/"previews"; dest.mkdir(exist_ok=True)
    manifest=json.loads((out/"manifest.json").read_text())
    world=np.array(manifest["world_from_pad"])
    target=world[:3, 3]
    meshes={p.stem:trimesh.load(p, force="mesh") for p in sorted((out/"assembly_mesh").glob("*.stl"))}
    pad=ControllerAsset().meshes()
    measured=manifest["parameters"]["measured"]
    asset_scale=[measured["width"]/153.,measured["height"]/104.6,measured["depth"]/50.2]
    for mesh in pad.values():mesh.apply_scale(asset_scale)
    local=[(n,m,color(n)) for n,m in meshes.items()]+[("controller_"+n,m,COLORS.get(n,"#67727d")) for n,m in pad.items()]
    if not any("spring" in n for n in meshes):
        installation=manifest["parameters"]["installation"];hardware=manifest["parameters"]["hardware"]
        for index,x in enumerate([-installation["guide_x"],installation["guide_x"]],1):
            local.append((f"visual_standard_spring_{index}",spring_mesh(x,installation["guide_z"],installation["slider_spring_seat_y"],installation["cap_spring_seat_y"],hardware["spring_od"],hardware["spring_wire"],hardware["spring_active_coils"]+2),"#b9c4ce"))
    items=[]
    for n,m,c in local:
        transformed=m.copy();transformed.apply_transform(world);items.append((n,transformed,c))
    foot=target.copy();foot[2]-=105
    eye=foot+np.array([-400,-480,310])
    fig,ax=plt.subplots(figsize=(11,10),facecolor="#f3f6f8")
    draw(ax,items,lookat(eye,foot),title="01 / COMPLETE ASSEMBLY")
    save(fig,dest/"01_assembly_isometric.png","Xbox One S 1708 source mesh + exported fixture solids | mm | CAD preview, not a physical fit certificate")
    scene=json.loads((ROOT/manifest["source_scene"]).read_text())
    camera=next(c for c in scene["cameras"] if c["name"]=="camera_3")
    pose=camera["pose_optical"];optical=np.eye(4)
    optical[:3,:3]=Rotation.from_quat([pose[4],pose[5],pose[6],pose[3]]).as_matrix();optical[:3,3]=np.array(pose[:3])*1000
    k=camera["intrinsics"]
    fig,axes=plt.subplots(1,2,figsize=(16,8),facecolor="#f3f6f8",gridspec_kw={"width_ratios":[1.5,1]})
    camera_inverse=np.linalg.inv(optical);intrinsics=[k[s] for s in ("fx","fy","cx","cy")]
    draw(axes[0],items,camera_inverse,True,intrinsics,"02 / CAMERA 3 - FULL FRAME",[0,k["width"],0,k["height"]])
    reference_vertices=np.vstack([m.vertices for n,m,c in items if n.startswith("controller_")])
    reference_view=trimesh.transform_points(reference_vertices,camera_inverse)
    pixel_xy=reference_view[:,:2]/reference_view[:,2,None]*[k["fx"],k["fy"]]+[k["cx"],k["cy"]]
    lo,hi=pixel_xy.min(0)-40,pixel_xy.max(0)+40
    draw(axes[1],items,camera_inverse,True,intrinsics,"SAME CAMERA - DETAIL CROP",[lo[0],hi[0],lo[1],hi[1]])
    save(fig,dest/"02_camera_3.png","Recorded optical pose and pinhole intrinsics | full 1280 x 720 field at left, controller crop at right")
    # Explode by assembly role; controller remains intact and visible.
    offsets={"base_printed":[0,0,-65],"base_aluminum":[0,0,-65],"aluminum_pad_recess_tray":[0,0,-70],"pad_base_bottom":[0,0,-75],"column":[-80,0,-15],
             "cradle_lower_jaw":[0,0,0],"slider_upper_jaw":[0,45,15],"guide_cap":[0,80,-10],
             "M6_hand_knob":[85,0,0],"guide_rod_1":[-45,35,-15],"guide_rod_2":[45,35,-15],
             "pad_back":[0,0,25],"pad_upper_jaw":[0,55,35],"pad_lower_jaw":[0,-35,25]}
    exploded=[]
    for n,m,c in local:
        copy=m.copy()
        offset=offsets.get(n,[0,0,0])
        if n.startswith("controller_"):offset=[0,0,75]
        if n.startswith("visual_standard_spring_"):offset=[-45 if n.endswith("1") else 45,35,-15]
        if n.startswith("hardware_compression_spring_"):offset=[-45 if n.endswith("1") else 45,35,-15]
        if n.startswith("hardware_M6"):offset=[-110 if "bolt" in n else 110,0,-10]
        copy.apply_translation(offset);copy.apply_transform(world);exploded.append((n,copy,c))
    fig,ax=plt.subplots(figsize=(12,11),facecolor="#f3f6f8")
    draw(ax,exploded,lookat(foot+[-450,-550,350],foot),title="03 / EXPLODED ASSEMBLY")
    save(fig,dest/"03_exploded.png","Assembly-role offsets for identification; spring and clamp hardware are specified in the BOM | same controller mesh")
    # Section outlines cut the actual exported triangles. Two true cut planes.
    fig,axes=plt.subplots(1,2,figsize=(14,8),facecolor="#f3f6f8")
    pivot=np.array(manifest["pivot_pad_mm"])
    for ax,axis,origin,cols,title in [(axes[0],0,[0,0,0],[2,1],"JAW / BACKREST: x = 0 mm"),
                                      (axes[1],1,[0,pivot[1],0],[0,2],"PIVOT AXIS: y = 0 mm")]:
        normal=np.eye(3)[axis]
        for n,m,c in local:
            section=m.section(plane_origin=origin,plane_normal=normal)
            if section is None:continue
            for line in section.discrete:
                xy=line[:,cols]
                ax.fill(xy[:,0],xy[:,1],color=c,alpha=.8,linewidth=0)
                ax.plot(xy[:,0],xy[:,1],color=c,linewidth=.8)
        ax.set_aspect("equal");ax.grid(alpha=.15);ax.set_title(title,fontsize=12,fontweight="bold",color="#17394a")
        ax.set_xlabel((["x","y","z"])[cols[0]]+" / mm");ax.set_ylabel((["x","y","z"])[cols[1]]+" / mm")
    axes[0].set_xlim(-62,32);axes[0].set_ylim(-38,86)
    axes[1].set_xlim(-58,43);axes[1].set_ylim(-125,30)
    fig.tight_layout(rect=(0,.05,1,1));save(fig,dest/"04_sections.png","True triangle / plane sections | teal: cradle, amber: slider, purple: compressed sheet pads, grey: controller")
    # Rear view uses pad local x horizontally and y vertically.
    rear=lookat([0,0,-600],[0,0,-30],up=(0,1,0))
    fig,ax=plt.subplots(figsize=(11,10),facecolor="#f3f6f8")
    draw(ax,local,rear,title="05 / REAR VIEW - FREE GRIP BACKS")
    save(fig,dest/"05_back.png","Only the narrow central spine supports the shell | both handle backs remain open for ring and little fingers")
    prints=[]
    x,y,rowheight=0.,0.,0.
    labels=[]
    for part in manifest["parts"]:
        if not part["is_printed"]:continue
        name=part["name"];mesh=trimesh.load(out/"print"/(name+".stl"),force="mesh")
        size=mesh.extents
        if x+size[0]>470:x=0;y+=rowheight+35;rowheight=0
        mesh.apply_translation([x,y,0]);prints.append((name,mesh,color(name)))
        labels.append((np.array([x+size[0]/2,y-8,0]),name));x+=size[0]+28;rowheight=max(rowheight,size[1])
    # Keep the measured controller visible as a size reference on this sheet.
    for name,mesh in pad.items():
        ref=mesh.copy();ref.apply_translation([90,y+rowheight+95,35])
        prints.append(("reference_"+name,ref,COLORS.get(name,"#67727d")))
    allvertices=np.vstack([m.vertices for _,m,_ in prints]);center=(allvertices.min(axis=0)+allvertices.max(axis=0))/2
    fig,ax=plt.subplots(figsize=(14,11),facecolor="#f3f6f8")
    printcam=lookat(center+[450,-650,900],center)
    draw(ax,prints,printcam,title="06 / RECOMMENDED PRINT ORIENTATIONS")
    for index,(position,name) in enumerate(labels,1):
        point=trimesh.transform_points([position],printcam)[0]
        ax.text(point[0],point[1],f"P{index:02}",fontsize=9,color="#17394a",ha="center",fontweight="bold")
    fig.text(.04,.075,"   |   ".join(f"P{k:02} {name}" for k,(_,name) in enumerate(labels,1) if k<=5),fontsize=8,color="#526979")
    fig.text(.04,.052,"   |   ".join(f"P{k:02} {name}" for k,(_,name) in enumerate(labels,1) if k>5),fontsize=8,color="#526979")
    save(fig,dest/"06_print_layout.png","Multi-plate overview | STLs rest at z = 0 | controller is a size reference, not a print part")
    fig,ax=plt.subplots(figsize=(13,10),facecolor="#f3f6f8")
    world_side(ax,items,manifest,title="07 / V2 Z LAYOUT - WORLD X / Z",resolution=2200)
    save(fig,dest/"07_world_z_layout.png","World side projection; C-clamp geometry is a procurement keepout, operator access is dashed red; clamps are +/-70 mm off the table centreline")
    print("\n".join(str(p) for p in sorted(dest.glob("*.png"))))


if __name__=="__main__":
    parser=argparse.ArgumentParser();parser.add_argument("--out",type=Path,default=HERE/"out")
    main(parser.parse_args().out)
