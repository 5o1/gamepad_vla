#!/usr/bin/env python3
"""One-page A3 material-coloured drawing from the existing exported CAD.

Uses render.draw's CPU depth buffer, original controller FK and true sections.
No scene, geometry, station or installation parameters are changed.
"""
from __future__ import annotations
import argparse
import json
import os
from pathlib import Path

os.environ.setdefault("MPLCONFIGDIR","/tmp/gamepad-fixture-matplotlib")

import numpy as np
import trimesh
from matplotlib import font_manager
from matplotlib.path import Path as MplPath
from matplotlib.patches import Arc, PathPatch, Rectangle

from render import plt, draw, lookat, clamp_wireframe, world_side
from asset_geometry import ControllerAsset

HERE=Path(__file__).resolve().parent
INK="#183C4A"
MUTED="#526D79"
BG="#F3F6F8"


def project(points,camera):
    return trimesh.transform_points(np.atleast_2d(points),camera)[:,:2]


def dimension(ax,a,b,label,offset=(0,0),vertical=False):
    a,b=np.array(a),np.array(b);delta=np.array(offset)
    p,q=a+delta,b+delta
    for start,end in ((a,p),(b,q)):
        ax.plot([start[0],end[0]],[start[1],end[1]],color=MUTED,lw=.55,zorder=7)
    ax.annotate("",xy=p,xytext=q,arrowprops={"arrowstyle":"<->","color":INK,"lw":.85},zorder=9)
    mid=(p+q)/2
    ax.text(mid[0]-(2.4 if vertical else 0),mid[1]-(0 if vertical else 2.5),label,
            fontsize=7.6,color=INK,rotation=90 if vertical else 0,ha="center",va="center",
            bbox={"facecolor":BG,"edgecolor":"none","pad":1.2},zorder=10)


def leader(ax,point,position,text):
    ax.annotate(text,xy=point,xytext=position,fontsize=7.5,color=INK,
                ha="left",va="center",arrowprops={"arrowstyle":"-","color":MUTED,"lw":.7},
                bbox={"facecolor":BG,"edgecolor":"none","pad":1.5},zorder=10)


def section(ax,items,axis,origin,cols,limits,title):
    """Compound oriented rings preserve holes instead of filling every loop."""
    for name,mesh,color in items:
        cut=mesh.section(plane_origin=origin,plane_normal=np.eye(3)[axis])
        if cut is None:continue
        loops=[np.asarray(line)[:,cols] for line in cut.discrete if len(line)>3]
        loops.sort(key=lambda xy:abs(np.sum(xy[:-1,0]*xy[1:,1]-xy[1:,0]*xy[:-1,1])),reverse=True)
        paths=[]
        for k,poly in enumerate(loops):
            if np.linalg.norm(poly[0]-poly[-1])<1e-6:poly=poly[:-1]
            depth=sum(MplPath(other).contains_point(poly[0]) for other in loops[:k])
            signed=np.sum(poly[:,0]*np.roll(poly[:,1],-1)-np.roll(poly[:,0],-1)*poly[:,1])
            if (signed>0)!=(depth%2==0):poly=poly[::-1]
            vertices=np.vstack([poly,poly[0]])
            paths.append(MplPath(vertices,[MplPath.MOVETO]+[MplPath.LINETO]*(len(poly)-1)+[MplPath.CLOSEPOLY]))
        if paths:
            ax.add_patch(PathPatch(MplPath.make_compound_path(*paths),facecolor=color,edgecolor="#405A65",lw=.3))
    ax.set_xlim(*limits[:2]);ax.set_ylim(*limits[2:]);ax.set_aspect("equal")
    ax.set_facecolor(BG);ax.grid(alpha=.12,lw=.4);ax.tick_params(labelsize=5.5,length=2)
    ax.set_title(title,loc="left",fontsize=9.3,color=INK,fontweight="bold",pad=7)
    for spine in ax.spines.values():spine.set_color("#CED8DD")


def copper_insert(center,od,length,thread):
    mesh=trimesh.creation.annulus(r_min=thread/2,r_max=od/2,height=length,sections=24)
    mesh.apply_translation(center)
    return mesh


def main(out=HERE/"out",destination=HERE):
    out,destination=Path(out),Path(destination)
    manifest=json.loads((out/"manifest.json").read_text())
    palette=json.loads((HERE/"materials_palette.json").read_text())
    font_path=Path("/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc")
    if font_path.exists():
        font_manager.fontManager.addfont(str(font_path))
        plt.rcParams["font.family"]=[font_manager.FontProperties(fname=font_path).get_name(),"DejaVu Sans"]
    plt.rcParams.update({"axes.unicode_minus":False,"pdf.fonttype":42,"font.size":8})
    color=lambda key:palette[key]["color"]
    metadata={part["name"]:part for part in manifest["parts"]}
    local=[]
    for name,part in metadata.items():
        material=part["material"]
        key={"Steel":"steel","Spring_steel":"steel","Aluminum_6061":"aluminum","TPU95A":"TPU","adhesive_silicone_sheet":"silicone","adhesive_silicone_or_EVA_sheet":"silicone","adhesive_eva_sheet":"EVA","adhesive_EVA_sheet":"EVA","PETG":"PETG"}[material]
        local.append((name,trimesh.load(out/"assembly_mesh"/(name+".stl"),force="mesh"),color(key)))
    p=manifest["parameters"];m,i,h=[p[k] for k in ("measured","installation","hardware")]
    asset=ControllerAsset().meshes()
    for name,mesh in asset.items():
        mesh.apply_scale([m["width"]/153,m["height"]/104.6,m["depth"]/50.2])
        local.append(("reference_"+name,mesh,color("reference")))
    # Copper inserts are BOM reference envelopes, not extra printed CAD parts.
    for n,x in enumerate((-i["cap_bolt_x"],i["cap_bolt_x"]),1):
        local.append((f"bom_copper_M3_{n}",copper_insert([x,71,-27],h["cap_insert_od"],h["cap_insert_length"],3),color("copper")))
    local.append(("bom_copper_M4",copper_insert([0,i["lock_center_y"],-47],h["lock_insert_od"],h["lock_insert_length"],4),color("copper")))
    world=np.asarray(manifest["world_from_pad"])
    Fworld=np.asarray(manifest["fixed_frame_world_axes"])
    datum=np.asarray(manifest["fixed_frame_world_origin_mm"])
    Fpad=np.eye(4);Fpad[:3,:3]=Fworld.T@world[:3,:3];Fpad[:3,3]=Fworld.T@(world[:3,3]-datum)
    def in_F(items):
        result=[]
        for name,mesh,c in items:
            mesh=mesh.copy();mesh.apply_transform(Fpad);result.append((name,mesh,c))
        return result
    assembled=in_F(local)
    root=Fpad[:3,3]
    target=np.array([0,-30,125])
    iso=lookat(target+[-450,-550,300],target)
    fig=plt.figure(figsize=(420/25.4,297/25.4),facecolor=BG)
    fig.text(.035,.965,"Xbox One S 1708 · 手柄夹具设计总图",fontsize=20,color=INK,fontweight="bold")
    fig.text(.965,.969,"任务 V · v2 Z 布局  |  2026-10-30  |  A3 单页",fontsize=8.5,color=MUTED,ha="right")
    fig.text(.035,.939,"讲台式底座 / M6 端面齿锁紧转轴 / 双压簧滑动夹 / 背胶片材接触面",fontsize=10,color=MUTED)
    # The same material ID has one colour everywhere, including alternatives.
    legend_keys=["PETG","silicone","steel","copper","aluminum","TPU","PLA-CF","ASA","EVA","reference"]
    for k,key in enumerate(legend_keys):
        x=.035+k*.094
        fig.add_artist(Rectangle((x,.910),.011,.011,transform=fig.transFigure,facecolor=color(key),edgecolor="#9BABBA",lw=.4))
        fig.text(x+.014,.911,palette[key]["label"],fontsize=6.9,color=INK,va="bottom")
    ax=fig.add_axes([.035,.535,.445,.345])
    draw(ax,assembled,iso,title="",resolution=2200)
    clamp_wireframe(ax,manifest,iso)
    pad_label="TPU" if m["use_printed_tpu_pads"] else "EVA片" if "eva" in m["pad_material"].lower() else "硅胶片"
    ax.set_title(f"01  装配等轴测 · PETG + {pad_label}",loc="left",fontsize=10.5,color=INK,fontweight="bold",pad=5)
    # Explode the same exported parts; no replacement fixture drawing.
    offsets={"base_printed":[0,0,-60],"base_aluminum":[0,0,-60],"aluminum_pad_recess_tray":[0,0,-65],"pad_base_bottom":[0,0,-72],
             "column":[-75,0,-12],"slider_upper_jaw":[0,45,15],"guide_cap":[0,82,-10],"M6_hand_knob":[88,0,0],
             "guide_rod_1":[-45,35,-15],"guide_rod_2":[45,35,-15],"pad_back":[0,0,27],"pad_upper_jaw":[0,55,35],"pad_lower_jaw":[0,-35,25]}
    exploded=[]
    for name,mesh,c in local:
        offset=offsets.get(name,[0,0,0])
        if name.startswith("reference_"):offset=[0,0,75]
        elif name.startswith("hardware_compression_spring_"):offset=[-45 if name.endswith("1") else 45,35,-15]
        elif name.startswith("hardware_M6"):offset=[-110 if "bolt" in name else 110,0,-10]
        elif name.startswith("bom_copper_M3"):offset=[-55 if name.endswith("1") else 55,82,5]
        elif name=="bom_copper_M4":offset=[-48,45,-15]
        copy=mesh.copy();copy.apply_translation(offset);exploded.append((name,copy,c))
    ax=fig.add_axes([.515,.535,.45,.345])
    draw(ax,in_F(exploded),iso,title="",resolution=2200)
    ax.set_title("02  爆炸装配 · 同材料同色",loc="left",fontsize=10.5,color=INK,fontweight="bold",pad=5)
    fig.text(.52,.526,"铜套为 BOM 参考外形，不含螺纹；采购 C 型夹外框未画。",fontsize=6.7,color=MUTED)
    # Three orthographic views in the installation/table datum frame.
    front=lookat([0,-700,140],[0,-30,140])
    side=lookat([700,-35,140],[0,-35,140])
    top=lookat([0,-70,800],[0,-70,0],up=(0,1,0))
    views=[([.035,.235,.285,.255],front,[-180,180,-155,175],"03  正视 · 桌面基准"),
           ([.35,.235,.295,.255],side,None,"04  世界坐标侧视 · X / Z"),
           ([.675,.235,.29,.255],top,[-180,180,-170,160],"05  俯视 · 桌面基准")]
    axes=[]
    for rect,cam,bounds,title in views:
        ax=fig.add_axes(rect)
        if bounds is None:
            world_items=[]
            Fworld=np.eye(4);Fworld[:3,:3]=np.asarray(manifest["fixed_frame_world_axes"]);Fworld[:3,3]=datum
            for name,mesh,c in assembled:
                copy=mesh.copy();copy.apply_transform(Fworld);world_items.append((name,copy,c))
            side=world_side(ax,world_items,manifest,title="",resolution=1700)
        else:draw(ax,assembled,cam,bounds=bounds,title="",resolution=1700)
        ax.set_title(title,loc="left",fontsize=9.5,color=INK,fontweight="bold",pad=5);axes.append(ax)
    ax=axes[0]
    q=project([[-i["base_width"]/2,-i["base_depth"],0],[i["base_width"]/2,-i["base_depth"],0]],front);dimension(ax,*q,f"{i['base_width']:g} 底板宽",(0,18))
    points=np.array([[-m["upper_jaw_width"]/2,m["top_center_y"]+3,6],[m["upper_jaw_width"]/2,m["top_center_y"]+3,6]])
    points=trimesh.transform_points(points,Fpad);dimension(ax,*project(points,front),f"上爪 {m['upper_jaw_width']:g}",(0,-23))
    ax=axes[1]
    dimpoints=project([[world[0,3],world[1,3],i["tabletop_z"]],world[:3,3]],side)
    dimension(ax,*dimpoints,f"{root[2]:.0f} mm 中心高",(i["table_edge_x"]-187-world[0,3],0),True)
    q=project([[i["table_edge_x"]-i["base_depth"],i["table_center_y"],i["tabletop_z"]],[i["table_edge_x"],i["table_center_y"],i["tabletop_z"]]],side);dimension(ax,*q,f"{i['base_depth']:g} 底板深",(0,22))
    pivot=np.asarray(manifest["pivot_base_mm"])
    pivot_world=datum+np.asarray(manifest["fixed_frame_world_axes"])@pivot
    center=project(pivot_world,side)[0]
    angle=manifest["key_angle_deg"];r=39
    ax.add_patch(Arc(center,2*r,2*r,theta1=-angle,theta2=0,color=INK,lw=1,zorder=10))
    for a in (0,-np.deg2rad(angle)):
        end=center+43*np.array([np.cos(a),np.sin(a)])
        ax.plot([center[0],end[0]],[center[1],end[1]],lw=.7,color=MUTED,zorder=9)
    leader(ax,center+[r*.94,-r*.34],center+[56,-31],f"{manifest['mark_label_deg']:g}° 主刻线\n几何 {angle:.3f}°")
    contactpoint=trimesh.transform_points([[0,m["top_center_y"],0]],world)
    leader(ax,project(contactpoint,side)[0],[i["table_edge_x"]+50,-(i["tabletop_z"]+260)],f"夹持开口 {i['slider_min_span']:g}–{i['slider_max_span']:g}\n图示名义 {m['center_span']:g}")
    # Ground reference establishes height unambiguously.
    ax=axes[2]
    thickness=i["aluminum_thickness"] if i["aluminum_base"] else i["printed_base_thickness"]
    q=project([[-i["base_width"]/2,-i["base_depth"],0],[i["base_width"]/2,-i["base_depth"],0]],top)
    dimension(ax,*q,f"{i['base_width']:g} × {i['base_depth']:g} × {thickness:g} 底板",(0,18))
    for sign in (-1,1):
        x=sign*i["clamp_center_x"];y=i["clamp_center_y"]
        rectangle=project([[x-22,y-20,0],[x+22,y-20,0],[x+22,y+20,0],[x-22,y+20,0],[x-22,y-20,0]],top)
        ax.plot(rectangle[:,0],rectangle[:,1],color=INK,lw=.75,ls="--",zorder=8)
    band=project([[-i["base_width"]/2,-i["clamp_band_depth"],0],[i["base_width"]/2,-i["clamp_band_depth"],0]],top)
    ax.plot(band[:,0],band[:,1],color=INK,lw=.8,ls="--",zorder=8)
    leader(ax,project([[-i["clamp_center_x"],-26,0]],top)[0],[-142,-127],f"连续夹持带 {i['base_width']:g} × {i['clamp_band_depth']:g}\n承压厚 {thickness:g} / 整带实心修改器")
    # True section planes of the same meshes, not an illustrative redraw.
    a=fig.add_axes([.045,.055,.205,.142])
    section(a,local,0,[0,0,0],[2,1],[-56,30,-36,80],"A–A 夹爪 / 背托剖面")
    a.set_xlabel("Z / mm",fontsize=6);a.set_ylabel("Y / mm",fontsize=6)
    b=fig.add_axes([.295,.055,.205,.142])
    section(b,local,1,[0,i["pivot_pad_y"],0],[0,2],[-43,31,-121,-48],"B–B 转轴剖面")
    b.axhline(i["pivot_pad_z"],color=MUTED,lw=.6,ls="--")
    b.text(-39,-51,"M6 × 60 / 36 齿端面咬合",fontsize=6.5,color=INK)
    b.set_xlabel("X / mm",fontsize=6);b.set_ylabel("Z / mm",fontsize=6)
    # Compact fabrication data complements the external full material table.
    fig.text(.54,.190,"材料与制造要点",fontsize=10.5,color=INK,fontweight="bold")
    notes=["PETG 结构件：11 件；同色不代表合并为一个打印件。",
           f"连续 {i['base_width']:g}×{i['clamp_band_depth']:g} 夹持带 + 压力鞋：100% / ≥6 壁 / 各≥8 层。",
           "硅胶片：爪/背托 2 mm；底板/压力鞋 1 mm；槽深 = 厚度 −0.5。",
           "铝板可选：220 × 160 × 6；软垫可整套改 EVA 或独立打印 TPU。",
           "精确中心高 219.859 mm；40 刻线已按目标四元数校准。",
           "完整逐件材料与切片参数：materials.md（颜色代码见本页图例）。"]
    for k,note in enumerate(notes):fig.text(.54,.169-k*.021,note,fontsize=7.4,color=INK)
    fig.text(.035,.014,"单位 mm · 世界侧视包含桌下操作区，各视窗独立缩放；灰虚线=C夹包络，红虚线=操作空间。尺寸暂定，打印前实测。",fontsize=6.6,color=MUTED)
    fig.text(.965,.014,"CAD 几何检查通过 / 未实物试装或加载认证",fontsize=7,color=MUTED,ha="right")
    destination.mkdir(parents=True,exist_ok=True)
    fig.savefig(destination/"design_sheet.png",dpi=300,facecolor=BG)
    fig.savefig(destination/"design_sheet.pdf",dpi=300,facecolor=BG,metadata={"Title":"Xbox One S 1708 fixture — task V design sheet"})
    plt.close(fig)
    print(json.dumps({"png":str(destination/"design_sheet.png"),"pdf":str(destination/"design_sheet.pdf"),"size_mm":[420,297],"dpi":300,"material_palette":str(HERE/"materials_palette.json")},ensure_ascii=False))


if __name__=="__main__":
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--out",type=Path,default=HERE/"out")
    parser.add_argument("--destination",type=Path,default=HERE)
    args=parser.parse_args();main(args.out,args.destination)
