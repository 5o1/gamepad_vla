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

def main(out=HERE/'out',destination=HERE):
    out,destination=Path(out),Path(destination);mf=json.loads((out/'manifest.json').read_text());p=mf['parameters'];m,i,h=[p[k] for k in ('measured','installation','hardware')]
    font=Path('/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc')
    if font.exists():font_manager.fontManager.addfont(str(font));plt.rcParams['font.family']=[font_manager.FontProperties(fname=font).get_name(),'DejaVu Sans']
    plt.rcParams.update({'axes.unicode_minus':False,'pdf.fonttype':42,'font.size':8})
    from render import material_color
    local=[(r['name'],trimesh.load(out/'assembly_mesh'/(r['name']+'.stl'),force='mesh'),material_color(r['material'])) for r in mf['parts']]
    local += [('reference_'+n,v,'#d0d6dc') for n,v in ControllerAsset().visual_meshes().items()]
    W=np.asarray(mf['world_from_pad']);F=np.eye(4);F[:3,:3]=mf['fixed_frame_world_axes'];F[:3,3]=mf['fixed_frame_world_origin_mm'];Fpad=np.linalg.inv(F)@W
    def transform(items,T):
        res=[]
        for n,v,c in items:v=v.copy();v.apply_transform(T);res.append((n,v,c))
        return res
    assembled=transform(local,Fpad);target=np.array([0,-35,135]);iso=lookat(target+[-480,-520,270],target)
    fig=plt.figure(figsize=(420/25.4,297/25.4),facecolor=BG)
    fig.text(.03,.965,'Xbox One S 1708 · V3.1 竖直立柱 / 对称叉轴 / 平面夹爪',fontsize=19,color=INK,fontweight='bold')
    fig.text(.03,.935,'单位 mm  |  H2D 双喷头 300×320×325  |  PETG 主体 + 2 mm 硅胶片（TPU 可选）  |  资产推得，待卡尺与实物试验',fontsize=9,color=MUTED)
    ax=fig.add_axes([.025,.515,.35,.355]);draw(ax,assembled,iso,title='01 / 实际装配',resolution=1700)
    exploded=[]
    for n,v,c in local:
        v=v.copy();d={'column':[-65,0,-15],'hirth_fixed':[-60,0,0],'hirth_moving':[-30,0,0],'slider_upper_jaw':[0,40,15],'guide_cap':[0,75,0],'M8_hand_knob':[70,0,0],'pad_back':[0,0,25],'pad_lower_jaw':[0,-25,20],'pad_upper_jaw':[0,50,20]}.get(n,[0,0,0])
        if n.startswith('base_'):d=[0,0,-55]
        if n.startswith('reference_'):d=[0,0,25]
        v.apply_translation(d);exploded.append((n,v,c))
    ax=fig.add_axes([.385,.515,.36,.355]);draw(ax,transform(exploded,Fpad),iso,title='02 / 爆炸装配',resolution=1700)
    ax=fig.add_axes([.76,.55,.22,.33]);ax.axis('off')
    depth=i['table_edge_x']-W[0,3];height=W[2,3]-i['tabletop_z'];mech=mf['mechanics']
    notes=[('结构尺寸',f"底板 {i['base_width']:g}×{i['base_depth']:g}×{i['printed_base_thickness']:g}",f"柱身 {i['column_width']:g}×{i['column_depth']:g}，柱脚→轴 {i['column_height_to_axis']:g}",f"叉耳 X=±{i['ear_inner_x']:g}…±{i['ear_outer_x']:g}，金属 M8 双剪"),
        ('夹持与接触',f"上爪 {m['upper_jaw_width']:g} / 下爪 {m['lower_jaw_width']:g} / 背托 {m['rear_support_width']:g}",f"平行开口 {m['center_span']:.3f}，行程 66–80",'软垫自由 2.00 / 压缩后 1.85 / 槽 1.50'),
        ('装配结果',f"内距 {depth:.2f} / 高度 {height:.2f} / 倾角 {mf['key_angle_deg']:.1f}°",'位姿来自竖直柱与托架的几何尺寸',f"4 N·m 分离安全系数 {mech['pivot_separation_safety_factor']:.2f}（粗算）")]
    y=1.
    for group in notes:
        ax.text(0,y,group[0],fontweight='bold',color=INK,fontsize=10,va='top');y-=.09
        for line in group[1:]:ax.text(0,y,line,color=INK,fontsize=7.9,va='top');y-=.075
        y-=.04
    front=lookat([0,-700,140],[0,-30,140]);side=lookat([700,-30,140],[0,-30,140]);top=lookat([0,-45,800],[0,-45,0],up=(0,1,0))
    for rect,cam,title in [([.025,.255,.30,.235],front,'03 / 正视'),([.355,.255,.30,.235],side,'04 / 侧视'),([.685,.255,.29,.235],top,'05 / 俯视与可夹区')]:
        ax=fig.add_axes(rect);draw(ax,assembled,cam,title=title,resolution=1300)
        if title.startswith('03'):
            q=project([[-120,-90,0],[120,-90,0]],cam);dimension(ax,*q,'240 底板宽',(0,15))
        elif title.startswith('04'):
            q=project([[0,-30,0],[0,-30,mf['pivot_base_mm'][2]]],cam);dimension(ax,*q,f"轴高 {mf['pivot_base_mm'][2]:.2f}",(22,0),True)
        else:
            for a,b in mf['clamp_usable_segments_base_x_mm']:
                q=project([[a,-15,18],[b,-15,18]],cam);ax.plot(q[:,0],q[:,1],color='#d59736',lw=5,zorder=10)
            q=project([[-120,-30,18],[120,-30,18]],cam);ax.plot(q[:,0],q[:,1],ls='--',color=INK,lw=.7)
            ax.text(0,90,'两侧各 75 mm 可夹；不限定 C 夹中心\n接触深度 ≤ min(30, 桌边到裙板)',ha='center',fontsize=7,color=INK)
    ax=fig.add_axes([.03,.055,.22,.16]);section(ax,local,0,[0,0,0],[2,1],[-64,30,-62,80],'A-A / 平面接触与前唇')
    ax=fig.add_axes([.29,.055,.23,.16]);pv=mf['pivot_pad_mm'];section(ax,local,1,[0,pv[1],0],[0,2],[-39,53,pv[2]-36,pv[2]+36],'B-B / 双剪轴与单侧齿盘')
    ax=fig.add_axes([.56,.055,.41,.16]);ax.axis('off')
    text=['制造与装配：','齿盘 r=18–30，36齿=10°；60°恒定齿侧，齿顶/根平带1.1/0.9，根隙0.2。',
        '齿面朝上，0.08–0.12层高；先小样测扭矩。锁紧：松→拉开→转→咬合→拧紧。',
        '柱脚四M5自上而下入底板热熔铜套；无根部螺母通道、无底板斜筋、无压力鞋。',
        '夹持带整体100%实心修改器；用户自行粘底部软垫，桌板25、裙板距离30均待测。',
        '刻线仅主线与±10°，无数字/薄指针；实际倾角用可拆标记卡测量。',
        '可打印性逐件硬门与局部支撑/豁免详见检查JSON；本图不表示已实打或已做强度试验。']
    for n,line in enumerate(text):ax.text(0,1-n*.135,line,fontsize=8 if n==0 else 7.1,fontweight='bold' if n==0 else 'normal',color=INK,va='top')
    fig.text(.03,.018,'V3.1：左右拇指通道前唇退让2 mm，剩高1 mm；上爪前端外角2 mm倒角；保留承压垫全足迹。完整BOM见 materials.md；实物放行见 VALIDATION.md。',fontsize=7,color=MUTED)
    destination.mkdir(parents=True,exist_ok=True);fig.savefig(destination/'design_sheet.png',dpi=300,facecolor=BG);fig.savefig(destination/'design_sheet.pdf',dpi=300,facecolor=BG,metadata={'Title':'Xbox 1708 fixture V3.1'});plt.close(fig)
    return destination/'design_sheet.png'

if __name__=='__main__':
    a=argparse.ArgumentParser();a.add_argument('--out',type=Path,default=HERE/'out_v3_1');a.add_argument('--destination',type=Path,default=HERE);v=a.parse_args();print(main(v.out,v.destination))
