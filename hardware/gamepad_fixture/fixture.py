#!/usr/bin/env python3
"""V3 printable Xbox 1708 fixture. CAD mm; simulation m.

Integer structural dimensions determine the assembled controller pose. Contact
surfaces are analytic planes measured by contact_fit, never mesh subtraction.
"""
from __future__ import annotations
import argparse
import dataclasses
import hashlib
import json
import math
from pathlib import Path
import shutil
import numpy as np
import trimesh
from scipy.spatial.transform import Rotation
from build123d import (Axis, Box, Circle, Color, Compound, Cylinder, Face, Helix,
                      Location, Plane, Shell, Solid, Wire, export_step, export_stl,
                      extrude, sweep)
from asset_geometry import ControllerAsset, DEFAULT_URDF
from contact_fit import measure, OPENINGS

HERE=Path(__file__).resolve().parent
ROOT=HERE.parents[1]
SCENE=ROOT/'configs/workcell/placement_trials/d050-h220-t+10.json'

@dataclasses.dataclass
class ControllerMeasurements:
    width:float=153.
    height:float=104.6
    depth:float=50.2
    # None selects current asset measurement. Caliper values override directly.
    top_center_y:float|None=None
    center_span:float|None=None
    rear_plane_z:float|None=None
    rear_plane_slope:float|None=None
    upper_jaw_width:float=40.
    lower_jaw_width:float=56.
    shoulder_inner_gap:float=46.2
    rear_support_width:float=44.
    rear_support_height:float=22.
    rear_center_y:float=23.
    jaw_pad_thickness:float=2.
    back_pad_thickness:float=2.
    pad_compression:float=.15
    pad_proud_uncompressed:float=.5
    user_base_pad_thickness:float=1.
    user_base_pad_compression:float=.25
    use_printed_tpu_pads:bool=False
    pad_material:str='adhesive_silicone_sheet'
    top_lip_y_min:float=47.5
    xbox_button_y_max:float=45.7
    pair_x_min:float|None=None
    pair_x_max:float|None=None
    pair_z_min:float|None=None
    pair_z_max:float|None=None
    port_clearance:float=2.
    port_fit_allowance:float=.1
    usb_x_min:float=-4.
    usb_x_max:float=4.
    usb_z_min:float=-4.
    usb_z_max:float=6.5
    headphone_x_min:float=-21.
    headphone_x_max:float=-18.
    extension_x_min:float=-8.
    extension_x_max:float=8.
    lateral_stop_half_gap:float=32.75
    lateral_stop_y_min:float=-30.
    lateral_stop_y_max:float=-25.
    lateral_stop_z_min:float=-6.
    lateral_stop_z_max:float=-2.
    battery_seam_y_max:float=8.

@dataclasses.dataclass
class Installation:
    layout_revision:str='v3-vertical-fork-planar'
    table_edge_x:float=718.4
    table_center_y:float=625.2
    tabletop_z:float=767.
    table_thickness:float=25.
    table_edge_to_apron:float=30.
    base_width:float=240.
    base_depth:float=90.
    printed_base_thickness:float=16.
    aluminum_base:bool=False
    aluminum_thickness:float=8.
    clamp_band_depth:float=30.
    flange_width:float=90.
    flange_depth:float=50.
    flange_thickness:float=8.
    column_center_y:float=-30.
    column_width:float=36.
    column_depth:float=32.
    column_height_to_axis:float=105.
    ear_inner_x:float=21.
    ear_outer_x:float=29.
    ear_radius:float=34.
    gusset_width:float=6.
    gusset_height:float=35.
    bolt_x:float=35.
    bolt_dy:float=16.
    # Independent design choice, not a requested root-pose constraint.
    tilt_deg:float=45.
    major_mark_deg:float=40.
    support_to_axis_vertical:float=100.
    tooth_inner_radius:float=18.
    tooth_outer_radius:float=30.
    teeth:int=36
    tooth_flank_deg:float=60.
    tooth_tip_flat:float=1.1
    tooth_root_flat:float=.9
    fixed_disc_radius:float=34.
    pivot_fixed_face_x:float=-17.
    knob_radius:float=30.
    knob_thickness:float=16.
    slider_min_span:float=66.
    slider_max_span:float=80.
    slider_length:float=40.
    slider_center_y:float=12.
    slider_width:float=36.
    slider_depth:float=16.
    guide_x:float=10.
    guide_z:float=-42.
    guide_min_y:float=-23.
    guide_max_y:float=66.
    lower_guide_seat_top_y:float=-17.
    lower_seat_length:float=10.
    cap_spring_seat_y:float=62.
    slider_spring_seat_y:float=32.
    lock_center_y:float=1.
    frame_back_z:float=-30.
    frame_front_z:float=-24.
    cap_outer_half_width:float=30.
    cap_bolt_x:float=25.
    frame_top_y:float=75.
    top_pad_z_min:float=6.
    top_pad_z_max:float=17.
    lower_pad_z_min:float=-8.
    lower_pad_z_max:float=14.
    jaw_front_lip_z:float=22.
    jaw_front_end_z:float=26.
    keep_USB_cable:bool=True
    USB_cable_back_z:float=-10.
    pair_slot_extra_z:float=1.

@dataclasses.dataclass
class Hardware:
    pivot_bolt_d:float=8.
    pivot_bolt_length:float=80.
    pivot_bolt_head_height:float=5.3
    pivot_nut_af:float=13.
    pivot_nut_height:float=6.5
    pivot_washer_od:float=20.
    pivot_washer_thickness:float=1.6
    pivot_hand_torque_Nm:float=1.2
    pivot_nut_factor:float=.2
    ring_mount_radius:float=12.
    fixed_ring_bolt_length:float=14.
    moving_ring_bolt_length:float=22.
    design_pivot_torque_Nm:float=4.
    base_bolt_d:float=5.
    base_insert_pilot:float=6.4
    base_insert_od:float=7.
    base_insert_length:float=8.
    base_insert_pilot_depth:float=9.
    aluminum_tap_drill_d:float=4.2
    aluminum_tap_depth:float=6.5
    aluminum_base_bolt_length:float=12.
    cap_bolt_d:float=3.
    cap_insert_pilot:float=4.1
    cap_insert_od:float=4.6
    cap_insert_length:float=4.
    lock_insert_pilot:float=5.4
    lock_insert_od:float=6.
    lock_insert_length:float=6.
    guide_d:float=6.
    spring_od:float=11.
    spring_wire:float=1.1
    spring_free_length:float=40.
    spring_rate:float=2.
    spring_solid_height_max:float=13.
    spring_active_coils:float=7.5
    pad_friction:float=.6
    pad_effective_E_MPa:float=.6
    clamp_force_max:float=2000.
    clamp_foot_d_min:float=15.

@dataclasses.dataclass
class Printing:
    nozzle_mm:float=.4
    line_width_mm:float=.42
    build_limits_mm:tuple=(300.,320.,325.)
    hard_shell_clearance:float=.25
    clearance_hole:float=.5
    guide_diametral_clearance:float=.6
    guide_socket_clearance:float=.25
    tooth_axial_clearance:float=.2
    nut_af_clearance:float=.3
    mesh_tolerance:float=.06
    tooth_mesh_tolerance:float=.02
    material_E_MPa:float=1500.
    tooth_shear_allow_MPa:float=5.
    effective_loaded_teeth:int=4

@dataclasses.dataclass
class Parameters:
    measured:ControllerMeasurements=dataclasses.field(default_factory=ControllerMeasurements)
    installation:Installation=dataclasses.field(default_factory=Installation)
    hardware:Hardware=dataclasses.field(default_factory=Hardware)
    printing:Printing=dataclasses.field(default_factory=Printing)


def box(x0,x1,y0,y1,z0,z1):
    return Box(x1-x0,y1-y0,z1-z0).moved(Location(((x0+x1)/2,(y0+y1)/2,(z0+z1)/2)))

def cyl(radius,length,center,axis='z'):
    return Cylinder(radius,length).moved(Location(center,{'x':(0,90,0),'y':(90,0,0),'z':(0,0,0)}[axis]))

def prism(points,direction):
    return extrude(Face(Wire.make_polygon(points,close=True)),amount=float(np.linalg.norm(direction)),dir=tuple(np.asarray(direction)/np.linalg.norm(direction)))

def hex_prism(af,length,center,axis='z'):
    r=af/math.sqrt(3)
    s=prism([(r*math.cos(k*math.pi/3),r*math.sin(k*math.pi/3),-length/2) for k in range(6)],(0,0,length))
    return s.moved(Location(center,{'x':(0,90,0),'y':(90,0,0),'z':(0,0,0)}[axis]))

def mesh_solid(mesh):
    return Solid(Shell([Face(Wire.make_polygon([tuple(v) for v in tri],close=True)) for tri in mesh.triangles]))

def bounds(shape):
    b=shape.bounding_box();return [[b.min.X,b.min.Y,b.min.Z],[b.max.X,b.max.Y,b.max.Z]]

def location(tm):
    return Location(tuple(tm[:3,3]),tuple(Rotation.from_matrix(tm[:3,:3]).as_euler('xyz',degrees=True)))


_HIRTH_CACHE={}

def hirth(p,moving=False):
    """Constant-angle planar radial flanks; radial crest height grows with r.

    Flank plane x = tan(beta)*(distance from radial valley ray - root_flat/2).
    Tip/root flats 1.1/.9 mm meet the default two-line hard gate. A complementary
    moving face uses exactly the same mesh surface + .2 mm axial clearance.
    Crest bands are conical faceted surfaces, not constant-height v2 ramps.
    """
    i,t=p.installation,p.printing;k=math.tan(math.radians(i.tooth_flank_deg))
    cache_key=(moving,i.teeth,i.tooth_inner_radius,i.tooth_outer_radius,i.tooth_tip_flat,i.tooth_root_flat,i.tooth_flank_deg,i.pivot_fixed_face_x,i.fixed_disc_radius,t.tooth_axial_clearance)
    if cache_key in _HIRTH_CACHE:return _HIRTH_CACHE[cache_key].moved(Location((0,0,0)))
    pitch=2*math.pi/i.teeth;half=pitch/2
    # At each radius split at the exact root/crest flat transitions. Joining
    # corresponding points gives planar flanks; no constant-height radial ramp.
    rings=[]
    for r in (i.tooth_inner_radius,i.tooth_outer_radius):
        maximum=k*(r*math.sin(half)-i.tooth_root_flat/2-i.tooth_tip_flat/2)
        if maximum<=t.tooth_axial_clearance:
            raise ValueError('Tooth pitch consumed by flats: reduce tooth count or use a finer nozzle/flat design')
        def angle_at(height):return math.asin((height/k+i.tooth_root_flat/2)/r)
        transitions=[angle_at(0),angle_at(t.tooth_axial_clearance),angle_at(maximum),angle_at(maximum+t.tooth_axial_clearance)]
        ang=np.array([0,*transitions,half,*[pitch-a for a in transitions[::-1]],pitch])
        raw=k*np.minimum(r*np.sin(ang)-i.tooth_root_flat/2,r*np.sin(pitch-ang)-i.tooth_root_flat/2)
        h=np.clip(raw,t.tooth_axial_clearance,maximum+t.tooth_axial_clearance) if moving else np.clip(raw,0,maximum)
        rings.append((r,ang,h))
    pieces=[]
    for tooth in range(i.teeth):
        for j in range(10):
            front=[]
            for ri,ji in ((0,j),(1,j),(1,j+1),(0,j+1)):
                r,a,h=rings[ri];ang=a[ji]+tooth*pitch
                front.append((i.pivot_fixed_face_x+h[ji],r*math.cos(ang),r*math.sin(ang)))
            back=[((-9. if moving else -21.),v[1],v[2]) for v in front]
            vertices=np.array(front+back)
            faces=[[0,1,2],[0,2,3],[4,6,5],[4,7,6],[0,4,5],[0,5,1],[1,5,6],[1,6,2],[2,6,7],[2,7,3],[3,7,4],[3,4,0]]
            mesh=trimesh.Trimesh(vertices,faces,process=True);mesh.fix_normals();pieces.append(mesh_solid(mesh))
    result=pieces[0].fuse(*pieces[1:])
    # Mid-flanks contact at locked position; root/tip flats alone retain
    # .2 mm relief. Tightening cannot consume the root clearance.
    # Central solid connects all teeth and gives three M3 screw lands. The
    # mating inner annulus retains clearance; no pilot tries to centre twice.
    result+=cyl(i.fixed_disc_radius,4 if not moving else 3,(-19 if not moving else -10.5,0,0),'x')
    if not moving:result+=cyl(i.tooth_inner_radius,4,(-19,0,0),'x')
    else:result+=cyl(i.tooth_inner_radius,3,(-10.5,0,0),'x')
    _HIRTH_CACHE[cache_key]=result
    return result


def mechanics(p):
    m,i,h,t=p.measured,p.installation,p.hardware,p.printing
    preload=h.pivot_hand_torque_Nm/(h.pivot_nut_factor*h.pivot_bolt_d/1000)
    mean_r=(i.tooth_inner_radius+i.tooth_outer_radius)/2
    sep=h.design_pivot_torque_Nm*1000/mean_r/math.tan(math.radians(i.tooth_flank_deg))
    N=2*h.spring_rate*(h.spring_free_length-(i.cap_spring_seat_y-i.slider_spring_seat_y))
    cap=h.pad_friction*N*(m.upper_jaw_width+m.lower_jaw_width-8)/4000
    kt=h.pad_effective_E_MPa/(m.back_pad_thickness-m.pad_compression)*(m.rear_support_height-2)*(m.rear_support_width-4)**3/12/1000
    area=(i.tooth_outer_radius-i.tooth_inner_radius)*(2*math.pi*i.tooth_inner_radius/i.teeth-i.tooth_root_flat)
    shaft_tau=30/(2*math.pi*h.pivot_bolt_d**2/4)
    return dict(scope='Analytic assumptions, not FEA or measured polymer properties',pivot_preload_N=preload,
        pivot_design_torque_Nm=h.design_pivot_torque_Nm,pivot_tooth_separating_force_N=sep,
        pivot_separation_safety_factor=preload/sep,pivot_tooth_shear_capacity_Nm=t.effective_loaded_teeth*area*t.tooth_shear_allow_MPa*mean_r/1000,
        pivot_cases={str(M):dict(separating_N=M*1000/mean_r/math.tan(math.radians(i.tooth_flank_deg)),safety_factor=preload/(M*1000/mean_r/math.tan(math.radians(i.tooth_flank_deg)))) for M in (.3,1.,4.)},
        metal_axis_double_shear_MPa_at_30N=shaft_tau,spring_total_closing_force_N=N,
        spring_total_closing_force_N_at_min_nominal_max=[2*h.spring_rate*(h.spring_free_length-(30-(s-m.center_span))) for s in (i.slider_min_span,m.center_span,i.slider_max_span)],
        spring_solid_height_margin_mm=30-(i.slider_max_span-m.center_span)-h.spring_solid_height_max,
        anti_twist_rectangle_upper_bound_Nm=cap,anti_twist_rectangle_upper_bound_safety_factor_at_0_4Nm=cap/.4,
        anti_twist_backpad_stiffness_Nm_per_rad=kt,anti_twist_backpad_rotation_deg_at_0_4Nm=math.degrees(.4/kt),
        anti_twist_width_sweep=[dict(upper_width_mm=w,lower_width_mm=m.lower_jaw_width,capacity_Nm=h.pad_friction*N*(w+m.lower_jaw_width-8)/4000) for w in (20,36,40)],
        anti_twist_limits='Rectangle estimate omits relief loss; use checks.json exact pad integrals. Friction mu=.6, spring rate and back-pad E are hypotheses. Backpad stiffness assumes continuous preloaded contact; without preload it is an upper bound. Bench test required.',
        column_bending_MPa_at_4Nm=4000/(i.column_width*i.column_depth**2/6),
        column_deflection_mm_at_30N=30*i.column_height_to_axis**3/(3*t.material_E_MPa*i.column_width*i.column_depth**3/12),
        clamp_pressure_MPa=h.clamp_force_max/(math.pi*h.clamp_foot_d_min**2/4),
        clamp_pressure_cases=[dict(force_N=force,foot_diameter_mm=d,pressure_MPa=force/(math.pi*d*d/4)) for force in (1000.,2000.) for d in (15.,20.)],
        clamp_pressure_limit_scope='Foot diameter and actual force unmeasured. At2kN/15mm pressure11.3MPa exceeds the v2 provisional10MPa short-term assumption; no pad/shoe/spreader is modelled. Measure foot/force and creep; do not infer allowable clamp torque from this estimate.')


def build(p=None,asset=None):
    p=p or Parameters();asset=asset or ControllerAsset();m,i,h,t=p.measured,p.installation,p.hardware,p.printing
    fit=measure(asset)
    for field,side,axis in (('pair_x_min',0,0),('pair_x_max',1,0),('pair_z_min',0,2),('pair_z_max',1,2)):
        if getattr(m,field) is None:setattr(m,field,fit['obstacles_visual_bounds_mm']['pair'][side][axis])
    if m.top_center_y is None:m.top_center_y=fit['contacts']['top']['visual']['plane_offset_mm']
    if m.center_span is None:m.center_span=m.top_center_y-fit['contacts']['bottom']['visual']['plane_offset_mm']
    if m.rear_plane_z is None:m.rear_plane_z=fit['contacts']['back']['visual']['plane_offset_mm']
    if m.rear_plane_slope is None:m.rear_plane_slope=fit['contacts']['back']['visual']['unconstrained_plane_coefficients'][1]
    lower_y=m.top_center_y-m.center_span
    uw=m.upper_jaw_width/2;lw=m.lower_jaw_width/2;rw=m.rear_support_width/2
    by0=m.rear_center_y-m.rear_support_height/2;by1=m.rear_center_y+m.rear_support_height/2
    if by0<m.battery_seam_y_max+2:raise ValueError('Back contact overlaps the measured battery seam/latch exclusion; move the back contact window')
    if m.upper_jaw_width>m.shoulder_inner_gap-2:raise ValueError('Upper jaw leaves less than1mm per shoulder; update the caliper widths/contact window')
    # Structural datum: vertical post on edge band, integer base/flange/height.
    F_R=Rotation.from_euler('x',i.tilt_deg,degrees=True).as_matrix()
    Fworld=np.array([[0,1,0],[-1,0,0],[0,0,1.]])
    base_z=m.user_base_pad_thickness-m.user_base_pad_compression
    base_t=i.aluminum_thickness if i.aluminum_base else i.printed_base_thickness
    floor=base_z+base_t;axis_z=floor+i.flange_thickness+i.column_height_to_axis
    support=np.array([0.,m.rear_center_y,m.rear_plane_z])
    pivot=support-F_R.T@np.array([0,0,i.support_to_axis_vertical])
    pivot_F=np.array([0.,i.column_center_y,axis_z]);root_F=pivot_F-F_R@pivot
    W=np.eye(4);W[:3,:3]=Fworld@F_R;W[:3,3]=np.array([i.table_edge_x,i.table_center_y,i.tabletop_z])+Fworld@root_F
    Fpad=np.eye(4);Fpad[:3,:3]=F_R;Fpad[:3,3]=root_F;padF=np.linalg.inv(Fpad)
    Ppivot=Location(tuple(pivot));parts={};meta={};modifiers={};cuts=[];cut_shapes={};ribs=[];contacts=[]
    def add(name,shape,material='PETG',fixed=False,rotation=(0,0,0),note=''):
        if fixed:shape=shape.moved(location(padF))
        shape.label=name;parts[name]=shape
        meta[name]=dict(name=name,material=material,is_printed=material in ('PETG','TPU95A'),fixed_in_base_frame=fixed,
                        print_rotation_deg=list(rotation),note=note)
    def cut(name,shape,tool,label,fixed=False,allowed=None):
        removed=shape & tool;after=shape-tool
        if removed.volume>1e-6:
            if fixed:removed=removed.moved(location(padF))
            key=f'{name}_{len(cuts):03d}_{label}';cut_shapes[key]=removed
            rb=bounds(removed)
            # Independent allowed tool envelope includes exact intended extent;
            # audit tests removed material cannot escape this specified region.
            toolpad=tool.moved(location(padF)) if fixed else tool
            cuts.append(dict(part=name,name=label,before_volume_mm3=shape.volume,after_volume_mm3=after.volume,
                removed_volume_mm3=removed.volume,removed_bounds_pad_mm=rb,allowed_bounds_pad_mm=allowed or bounds(toolpad),
                removed_mesh=f'cuts/{key}.stl',forbidden_parts=[]))
        return after
    # Flat full-width edge band; no pressure shoes, spreaders or underside pocket.
    base=box(-i.base_width/2,i.base_width/2,-i.base_depth,0,base_z,floor)
    cy=i.column_center_y
    for x in (-i.bolt_x,i.bolt_x):
        for y in (cy-i.bolt_dy,cy+i.bolt_dy):
            if i.aluminum_base:
                tool=cyl(h.aluminum_tap_drill_d/2,h.aluminum_tap_depth+.2,(x,y,floor-h.aluminum_tap_depth/2+.1))
            else:tool=cyl(h.base_insert_pilot/2,h.base_insert_pilot_depth+.2,(x,y,floor-h.base_insert_pilot_depth/2+.1))
            base=cut('base_aluminum' if i.aluminum_base else 'base_printed',base,tool,'top_insert',True)
    add('base_aluminum' if i.aluminum_base else 'base_printed',base,'Aluminum_6061' if i.aluminum_base else 'PETG',True,rotation=(i.tilt_deg,0,0),note='Flat underside down. Entire 30 mm edge band uses one 100% infill modifier; no fixed clamp locations.')
    flange=box(-45,45,cy-25,cy+25,floor,floor+8)
    column=flange+box(-18,18,cy-16,cy+16,floor+7,axis_z-36)+box(-29,29,cy-16,cy+16,axis_z-65,axis_z-36)
    # Two identical ears; circular heads blend into vertical cheek stems.
    for side in (-1,1):
        x0,x1=(-29,-21) if side<0 else (21,29)
        ear=cyl(34,8,((x0+x1)/2,cy,axis_z),'x')
        ear+=box(x0,x1,cy-16,cy+16,axis_z-65,axis_z)
        column+=ear
        xg0,xg1=(-24,-18) if side<0 else (18,24)
        # yz-plane trapezoid keeps 2 mm blunt ends; gussets lie in the main
        # fore/aft bending plane, outside the upright, one on each side.
        rib=prism([(xg0,cy-24,floor+7),(xg0,cy+24,floor+7),(xg0,cy+24,floor+9),(xg0,cy+12,floor+43),(xg0,cy-12,floor+43),(xg0,cy-24,floor+9)],(6,0,0))
        column+=rib
        ribs.append(dict(part='column',side=side,plane_normal=[1,0,0],load_plane_normal=[1,0,0],bounds_base_mm=bounds(rib)))
    for x in (-35,35):
        for y in (cy-16,cy+16):column=cut('column',column,cyl(2.75,10,(x,y,floor+4)),'top_down_M5',True)
    column=cut('column',column,cyl(4.3,12,(-25,cy,axis_z),'x'),'left_axis',True)
    column=cut('column',column,cyl(6.25,12,(25,cy,axis_z),'x'),'right_sliding_sleeve',True)
    # Fixed ring screws in top/front/rear positions, without cutting the root.
    for a in (0,120,240):
        y=cy+h.ring_mount_radius*math.cos(math.radians(a+i.tilt_deg));z=axis_z+h.ring_mount_radius*math.sin(math.radians(a+i.tilt_deg))
        column=cut('column',column,cyl(1.6,10,(-25,y,z),'x'),'fixed_ring_M3',True)
        column=cut('column',column,hex_prism(5.8,3.,(-27.6,y,z),'x').rotate(Axis((-27.6,y,z),(1,0,0)),i.tilt_deg),'fixed_ring_nut',True)
    add('column',column,'PETG',True,rotation=(i.tilt_deg,0,0),note='Vertical symmetric fork. Base bolts enter from above into base heat-set M5 inserts; no nut access channels. Support horizontal axis holes.')
    # Hirth surfaces in pivot coordinates. Fixed teeth clocked with assembled
    # cradle; only one pair locates, opposite sleeve slides axially (no spring).
    for name,moving in (('hirth_fixed',False),('hirth_moving',True)):
        ring=hirth(p,moving)
        ring=cut(name,ring,cyl(4.3,25,(-17,0,0),'x'),'M8_axis')
        for a in ((60,180,300) if moving else (0,120,240)):
            y,z=h.ring_mount_radius*math.cos(math.radians(a)),h.ring_mount_radius*math.sin(math.radians(a))
            ring=cut(name,ring,cyl(1.7,25,(-17,y,z),'x'),'M3_mount')
        # Only three readable marks, no tiny numbers or protruding pointer.
        if not moving:
            for d in (-10,0,10):
                tick=box(-17.9,-16.9,30.8,34.2,-.65,.65).rotate(Axis.X,d)
                ring=cut(name,ring,tick,'index_groove')
        else:
            ring=cut(name,ring,box(-12.1,-11.1,30.8,34.2,-.65,.65),'alignment_groove')
        # Cut audit coordinates above were pivot-local; update to pad-local.
        for row in cuts:
            if row['part']==name:
                key=Path(row['removed_mesh']).stem;cut_shapes[key]=cut_shapes[key].moved(Ppivot)
                row['removed_bounds_pad_mm']=bounds(cut_shapes[key]);row['allowed_bounds_pad_mm']=(np.asarray(row['allowed_bounds_pad_mm'])+pivot).tolist()
        add(name,ring.moved(Ppivot),rotation=(0,90 if moving else -90,0),note='Toothed face UP, 0.08-0.12 mm layers. Constant 60 deg flank; fixed tip/root 1.1/.9 mm; .2 mm tip/root relief, touching loaded flanks.')
    # Fixed carrier frame and lower jaw. All three contact seats are planes.
    c=m.pad_compression;proud=m.pad_proud_uncompressed-c;tc=m.jaw_pad_thickness-c;bc=m.back_pad_thickness-c
    upper=m.top_center_y;back=m.rear_plane_z
    frame=cut('cradle_lower_jaw',box(-28,28,-31,75,-30,-24),box(-22,22,15,76,-31,-23),'frame_window')
    rear=box(-rw,rw,by0,by1,-25,back-proud)
    rear=cut('cradle_lower_jaw',rear,box(-rw+2,rw-2,by0+1,by1-1,back-bc,back+1),'back_pad_pocket')
    backpad=box(-rw+2,rw-2,by0+1,by1-1,back-bc,back)
    back_tilt=math.degrees(math.atan(m.rear_plane_slope))
    back_axis=Axis((0,m.rear_center_y,back),(1,0,0))
    rear=rear.rotate(back_axis,back_tilt);backpad=backpad.rotate(back_axis,back_tilt)
    # Contact-pocket audit follows the same rigid plane rotation.
    for row in cuts:
        if row['name']=='back_pad_pocket':
            key=Path(row['removed_mesh']).stem;cut_shapes[key]=cut_shapes[key].rotate(back_axis,back_tilt)
            row['removed_bounds_pad_mm']=bounds(cut_shapes[key]);row['allowed_bounds_pad_mm']=row['removed_bounds_pad_mm']
    lower=box(-lw,lw,lower_y-7,lower_y-proud,-30,i.jaw_front_end_z)
    lower=cut('cradle_lower_jaw',lower,box(-lw+2,lw-2,lower_y-tc,lower_y+1,i.lower_pad_z_min,i.lower_pad_z_max),'lower_pad_pocket')
    lower+=box(-lw,lw,lower_y-7,lower_y+3,i.jaw_front_lip_z,i.jaw_front_end_z)
    lowerpad=box(-lw+2,lw-2,lower_y-tc,lower_y,i.lower_pad_z_min,i.lower_pad_z_max)
    # Coarse lateral registration at the inner grip arcs, with clearance.
    # These are stops, not extra preloaded locating planes. No grip-back rib.
    for side in (-1,1):
        x0,x1=(-m.lateral_stop_half_gap,-lw) if side<0 else (lw,m.lateral_stop_half_gap)
        lower+=box(x0,x1,m.lateral_stop_y_min,m.lateral_stop_y_max,m.lateral_stop_z_min,m.lateral_stop_z_max)
    # Lower spine overlaps frame and connects all regular faces to central hub.
    cradle=frame+rear+lower+box(-18,18,-31,-17,-49,-24)
    hub=cyl(32,18,(0,*pivot[1:]),'x')
    web=box(-9,9,pivot[1]-8,-20,pivot[2],-24)
    cradle+=hub+web
    cradle=cut('cradle_lower_jaw',cradle,cyl(4.3,22,(0,*pivot[1:]),'x'),'M8_axis')
    for a in (60,180,300):
        y,z=pivot[1]+h.ring_mount_radius*math.cos(math.radians(a)),pivot[2]+h.ring_mount_radius*math.sin(math.radians(a))
        cradle=cut('cradle_lower_jaw',cradle,cyl(1.6,20,(0,y,z),'x'),'moving_ring_M3')
        cradle=cut('cradle_lower_jaw',cradle,hex_prism(5.8,3.,(7.6,y,z),'x'),'moving_ring_nut')
    for x in (-i.guide_x,i.guide_x):cradle=cut('cradle_lower_jaw',cradle,cyl((h.guide_d+t.guide_socket_clearance)/2,6.2,(x,-20,-42),'y'),'guide_socket')
    for x in (-i.cap_bolt_x,i.cap_bolt_x):cradle=cut('cradle_lower_jaw',cradle,cyl(h.cap_insert_pilot/2,6.2,(x,71,-27),'z'),'cap_insert')
    min_stop=-8+(i.slider_min_span-m.center_span)
    for x in (-10,10):
        collar=cut('cradle_lower_jaw',cyl(4.1,min_stop+17,(x,(-17+min_stop)/2,-42),'y'),cyl(3.125,min_stop+17.2,(x,(-17+min_stop)/2,-42),'y'),'minimum_travel_collar')
        cradle+=collar
    add('cradle_lower_jaw',cradle,rotation=(0,90,0),note='Regular planar lower/back contacts. Print X side down; supports under jaw overhangs, keep contact pockets clean.')
    upperjaw=box(-uw,uw,upper+proud,upper+7,-30,i.jaw_front_end_z)
    upperjaw=cut('slider_upper_jaw',upperjaw,box(-uw+2,uw-2,upper-1,upper+tc,i.top_pad_z_min,i.top_pad_z_max),'upper_pad_pocket')
    upperjaw+=box(-uw,uw,m.top_lip_y_min,upper+7,i.jaw_front_lip_z,i.jaw_front_end_z)
    upperpad=box(-uw+2,uw-2,upper,upper+tc,i.top_pad_z_min,i.top_pad_z_max)
    # Pair button and its mirror clear full travel. USB channel stays open to
    # rear edge; preserve an intact rear load path outside the cable envelope.
    pair_lo=m.pair_x_min-m.port_clearance-m.port_fit_allowance;pair_hi=m.pair_x_max+m.port_clearance+m.port_fit_allowance
    pair_zlo=math.floor(m.pair_z_min-m.port_clearance);pair_zhi=math.ceil(m.pair_z_max+m.port_clearance+i.pair_slot_extra_z)
    for x0,x1 in ((pair_lo,pair_hi),(-pair_hi,-pair_lo)):
        tool=box(x0,x1,upper-.8,upper+8,pair_zlo,pair_zhi)
        upperjaw=cut('slider_upper_jaw',upperjaw,tool,'pair_clearance')
        upperpad=cut('pad_upper_jaw',upperpad,tool,'pair_clearance')
    if i.keep_USB_cable:
        tool=box(m.usb_x_min-m.port_clearance,m.usb_x_max+m.port_clearance,upper-.8,upper+8,i.USB_cable_back_z,m.usb_z_max+m.port_clearance)
        upperjaw=cut('slider_upper_jaw',upperjaw,tool,'USB_cable_clearance');upperpad=cut('pad_upper_jaw',upperpad,tool,'USB_cable_clearance')
    slider=box(-18,18,-8,32,-50,-34)+box(-uw,uw,25,upper+7,-35,-26.5)+upperjaw
    for x in (-i.guide_x,i.guide_x):slider=cut('slider_upper_jaw',slider,cyl((h.guide_d+t.guide_diametral_clearance)/2,44,(x,12,-42),'y'),'guide_slide')
    slider=cut('slider_upper_jaw',slider,cyl(h.lock_insert_pilot/2,6.2,(0,1,-47),'z'),'lock_insert')
    slider=cut('slider_upper_jaw',slider,cyl(2.25,24,(0,1,-39),'z'),'lock_screw')
    add('slider_upper_jaw',slider,rotation=(-90,0,0),note='40 mm upper jaw; relieved Pair/USB zones. Two springs preload lower datum; M4 set screw locks slide against frame.')
    cap=cut('guide_cap',box(-i.cap_outer_half_width,i.cap_outer_half_width,62,75,-50,-30),box(-22,22,61,76,-36,-29),'frame_window')
    max_stop=32+(i.slider_max_span-m.center_span)
    for x in (-10,10):
        cap+=cyl(4.1,62-max_stop+.2,(x,(62+max_stop)/2+.1,-42),'y')
        cap=cut('guide_cap',cap,cyl(3.125,68-max_stop+.2,(x,(68+max_stop)/2,-42),'y'),'guide_socket')
    for x in (-i.cap_bolt_x,i.cap_bolt_x):cap=cut('guide_cap',cap,cyl(1.75,24,(x,71,-40),'z'),'M3_cap_screw')
    add('guide_cap',cap,rotation=(-90,0,0))
    for n,x in enumerate((-10,10),1):
        add(f'guide_rod_{n}',cyl(3,89,(x,21.5,-42),'y'),rotation=(-90,0,0),note='Print vertical with brim; guide socket coupon first.')
        L=30-h.spring_wire;rad=(h.spring_od-h.spring_wire)/2;pitch=L/h.spring_active_coils
        path=Helix(pitch=pitch,height=L,radius=rad)
        spring=sweep(Plane(origin=(rad,0,0),z_dir=(0,rad*2*math.pi,pitch))*Circle(h.spring_wire/2),path=path)
        add(f'hardware_compression_spring_{n}',spring.moved(Location((x,32+h.spring_wire/2,-42),(-90,0,0))),'Spring_steel')
    add('lock_screw_spacer',cut('lock_screw_spacer',cyl(5,3,(0,1,-51.5)),cyl(2.25,4,(0,1,-51.5)),'M4_clearance'))
    # M8 metal sleeve in right ear transmits shear and slides during release.
    add('hardware_M8_sliding_sleeve',(cyl(6,22,(20,*pivot[1:]),'x')-cyl(4.2,24,(20,*pivot[1:]),'x')),'Steel',note='12 OD x 8.4 ID x 22 mm metal tube, axial slide >=4 mm; right ear double-shear support.')
    knob_x=31+h.pivot_washer_thickness+i.knob_thickness/2
    nut_x=knob_x+i.knob_thickness/2-h.pivot_nut_height/2-.15
    knob=cyl(i.knob_radius,i.knob_thickness,(knob_x,*pivot[1:]),'x')
    knob=cut('M8_hand_knob',knob,cyl((h.pivot_bolt_d+t.clearance_hole)/2,i.knob_thickness+2,(knob_x,*pivot[1:]),'x'),'M8_axis')
    knob=cut('M8_hand_knob',knob,hex_prism(h.pivot_nut_af+t.nut_af_clearance,h.pivot_nut_height+.5,(nut_x,*pivot[1:]),'x'),'M8_nut')
    add('M8_hand_knob',knob,rotation=(0,-90,0),note='60 mm diameter; target torque1.2 N m: hold knob and use a13mm torque wrench on the exposed LEFT bolt head. Optional Belleville washer below knob; no separating spring.')
    bolt_start=-29-h.pivot_washer_thickness
    add('hardware_M8_axis_bolt',cyl(h.pivot_bolt_d/2,h.pivot_bolt_length,(bolt_start+h.pivot_bolt_length/2,*pivot[1:]),'x')+hex_prism(h.pivot_nut_af,h.pivot_bolt_head_height,(bolt_start-h.pivot_bolt_head_height/2,*pivot[1:]),'x'),'Steel')
    for n,x in enumerate((-29-h.pivot_washer_thickness/2,31+h.pivot_washer_thickness/2),1):add(f'hardware_M8_washer_{n}',cyl(h.pivot_washer_od/2,h.pivot_washer_thickness,(x,*pivot[1:]),'x')-cyl(h.pivot_bolt_d/2+.2,h.pivot_washer_thickness+.4,(x,*pivot[1:]),'x'),'Steel')
    add('hardware_M8_nut',hex_prism(h.pivot_nut_af,h.pivot_nut_height,(nut_x,*pivot[1:]),'x')-cyl(h.pivot_bolt_d/2,h.pivot_nut_height+1.5,(nut_x,*pivot[1:]),'x'),'Steel')
    for a in (0,120,240):
        y,z=pivot[1]+h.ring_mount_radius*math.cos(math.radians(a)),pivot[2]+h.ring_mount_radius*math.sin(math.radians(a))
        add(f'hardware_fixed_ring_M3_{a}',cyl(1.5,h.fixed_ring_bolt_length,(-17-h.fixed_ring_bolt_length/2,y,z),'x')+cyl(2.75,3,(-15.5,y,z),'x'),'Steel')
        add(f'hardware_fixed_ring_nut_{a}',hex_prism(5.5,2.4,(-27.6,y,z),'x')-cyl(1.5,3,(-27.6,y,z),'x'),'Steel')
    for a in (60,180,300):
        y,z=pivot[1]+h.ring_mount_radius*math.cos(math.radians(a)),pivot[2]+h.ring_mount_radius*math.sin(math.radians(a))
        add(f'hardware_moving_ring_M3_{a}',cyl(1.5,h.moving_ring_bolt_length,(-12+h.moving_ring_bolt_length/2,y,z),'x')+cyl(2.75,3,(-13.5,y,z),'x'),'Steel')
        add(f'hardware_moving_ring_nut_{a}',hex_prism(5.5,2.4,(7.6,y,z),'x')-cyl(1.5,3,(7.6,y,z),'x'),'Steel')
    for n,(x,y) in enumerate(( (x,y) for x in (-35,35) for y in (cy-16,cy+16))):
        length=h.aluminum_base_bolt_length if i.aluminum_base else 16.
        radius=2. if i.aluminum_base else 2.5
        add(f'hardware_base_M5_{n}',cyl(radius,length,(x,y,floor+8-length/2))+cyl(4.25,5,(x,y,floor+10.5)),'Steel',True,note='Nominal M5x12, threaded core Ø4.0 only; M5x0.8 tapped6.5 deep in Ø4.2 pilot; tooth crests omitted' if i.aluminum_base else 'M5x16 top-down into8mm heat-set insert')
    padmat='TPU95A' if m.use_printed_tpu_pads else m.pad_material
    add('pad_back',backpad,padmat,rotation=(-back_tilt,0,0));add('pad_lower_jaw',lowerpad,padmat,rotation=(90,0,0));add('pad_upper_jaw',upperpad,padmat,rotation=(-90,0,0))
    # Back plane (3), lower plane (2), clearance side stops (1 bounded).
    # Upper jaw supplies preload; it is not a second rigid Y datum.
    for label,normal,fitvalue,free,pad,rigid,window in (
        ('upper',[0,1,0],upper,m.jaw_pad_thickness,'pad_upper_jaw','slider_upper_jaw',[-uw+2,uw-2,i.top_pad_z_min,i.top_pad_z_max]),
        ('lower',[0,-1,0],-lower_y,m.jaw_pad_thickness,'pad_lower_jaw','cradle_lower_jaw',[-lw+2,lw-2,i.lower_pad_z_min,i.lower_pad_z_max]),
        ('back',[0,math.sin(math.radians(back_tilt)),-math.cos(math.radians(back_tilt))],float(np.dot([0,math.sin(math.radians(back_tilt)),-math.cos(math.radians(back_tilt))],[0,m.rear_center_y,back])),m.back_pad_thickness,'pad_back','cradle_lower_jaw',[-rw+2,rw-2,by0+1,by1-1])):
        contacts.append(dict(name=label,pad=pad,rigid_part=rigid,normal=normal,fit_offset_mm=fitvalue,
            free_thickness_mm=free,compression_mm=c,compressed_thickness_mm=free-c,
            groove_depth_mm=free-m.pad_proud_uncompressed,rigid_face_offset_mm=fitvalue+free-c,
            rim_offset_mm=fitvalue+proud,probe_window_mm=window,plane_tilt_deg=back_tilt if label=='back' else 0.,
            plane_point_mm=[0,m.rear_center_y,back] if label=='back' else [0,upper,0] if label=='upper' else [0,lower_y,0]))
    modifiers['base_clamp_band']=box(-120,120,-min(i.clamp_band_depth,i.table_edge_to_apron),0,base_z,floor).moved(location(padF))
    q=Rotation.from_matrix(W[:3,:3]).as_quat();pose=(W[:3,3]/1000).tolist()+[q[3],q[0],q[1],q[2]]
    openings=json.loads(json.dumps(OPENINGS))
    for opening in openings:
        if opening['name']=='pair_button':
            opening['bounds_pad_mm']=[[m.pair_x_min-m.port_clearance,upper-.3,m.pair_z_min-m.port_clearance],[m.pair_x_max+m.port_clearance,upper+7,m.pair_z_max+m.port_clearance]]
        elif opening['name']=='USB':
            opening['bounds_pad_mm']=[[m.usb_x_min-m.port_clearance,upper-6,m.usb_z_min-m.port_clearance],[m.usb_x_max+m.port_clearance,upper+7,m.usb_z_max+m.port_clearance-.5]]
        if opening['name'] in ('headphone','extension'):
            opening['bounds_pad_mm'][0][1]=lower_y
            if opening['name']=='headphone':opening['bounds_pad_mm'][0][0]=m.headphone_x_min-m.port_clearance;opening['bounds_pad_mm'][1][0]=m.headphone_x_max+m.port_clearance
            else:opening['bounds_pad_mm'][0][0]=m.extension_x_min-m.port_clearance;opening['bounds_pad_mm'][1][0]=m.extension_x_max+m.port_clearance
            opening['source']+='; entry boundary is the asset-measured lower support plane, replacing rounded y=-24 estimate; planar bridge stays outside the aperture'
    manifest=dict(design_revision=i.layout_revision,parameters=dataclasses.asdict(p),parts=list(meta.values()),world_from_pad=W.tolist(),
        target_root_pose=pose,derived_root_pose=pose,fixed_frame_world_axes=Fworld.tolist(),fixed_frame_world_origin_mm=[i.table_edge_x,i.table_center_y,i.tabletop_z],
        pivot_base_mm=pivot_F.tolist(),pivot_pad_mm=pivot.tolist(),key_angle_deg=i.tilt_deg,mark_label_deg=40.,clocking_offset_deg=i.tilt_deg-40,
        base_bottom_rigid_plane_above_table_mm=base_z,nominal_span_mm=m.center_span,jaw_span_range_mm=[i.slider_min_span,i.slider_max_span],
        source_scene=str(SCENE.relative_to(ROOT)),mechanics=mechanics(p),contact_fit=fit,contact_planes=contacts,openings=openings,
        cut_audit=cuts,ribs=ribs,printability_exemptions=[],support_regions=[],
        clamp_usable_segments_base_x_mm=[[-120,-45],[45,120]],clamp_usable_length_mm=150.,
        symmetry=dict(plane='pad x=0',structural_parts=['base_aluminum' if i.aluminum_base else 'base_printed','column','cradle_lower_jaw','slider_upper_jaw','guide_cap'],
            exceptions=['Left Hirth locates; right sliding metal sleeve releases without bending fork','Three ring screw holes are functional single-side fasteners','Knob, nut pocket and readable index grooves are single-sided']),
        modifier_expected_volumes_mm3={'base_clamp_band':(parts['base_aluminum' if i.aluminum_base else 'base_printed'] & modifiers['base_clamp_band']).volume},
        assembly_frame='controller local mm',feasible_region=dict(depth_mm=[25,75],height_mm=[190,250]),physical_pose_certified=False)
    # Exact regular rear-contact collision primitives, in the same fitted
    # plane frame as CAD. Residual structure can be coarsened independently.
    B=np.eye(4);B[:3,:3]=Rotation.from_euler('x',back_tilt,degrees=True).as_matrix()
    B[:3,3]=np.array([0,m.rear_center_y,back])-B[:3,:3]@np.array([0,m.rear_center_y,back])
    rear_boxes=[[-rw,rw,by0,by1,-25,back-bc],[-rw,-rw+2,by0,by1,back-bc,back-proud],
        [rw-2,rw,by0,by1,back-bc,back-proud],[-rw+2,rw-2,by0,by0+1,back-bc,back-proud],
        [-rw+2,rw-2,by1-1,by1,back-bc,back-proud]]
    def record_box(v):return dict(bounds_mm=[[v[0],v[2],v[4]],[v[1],v[3],v[5]]],pad_from_box_frame=B.tolist())
    manifest['collision_overrides']={'cradle_lower_jaw':dict(residual_mesh='simulation/cradle_without_backrest_mm.stl',boxes=[record_box(v) for v in rear_boxes]),
        'pad_back':dict(boxes=[record_box([-rw+2,rw-2,by0+1,by1-1,back-bc,back])])}
    manifest['_collision_shapes']={'cradle_without_backrest_mm':cradle-rear}
    manifest['_cut_shapes']=cut_shapes
    return parts,modifiers,manifest


def export_simulation(out,manifest):
    from trimesh.visual.material import PBRMaterial
    from trimesh.visual.texture import TextureVisuals
    sim=trimesh.Scene()
    for part in manifest['parts']:
        name,mat=part['name'],part['material'];mesh=trimesh.load(out/'assembly_mesh'/(name+'.stl'),force='mesh')
        mesh.apply_scale(.001);mesh.units='m'
        col='#247c91' if mat=='PETG' else '#94789e' if name.startswith('pad_') else '#b1bcc5'
        mesh.visual=TextureVisuals(material=PBRMaterial(name=mat,baseColorFactor=trimesh.visual.color.hex_to_rgba(col),roughnessFactor=.7))
        sim.add_geometry(mesh,node_name=name,geom_name=name)
    sim.export(out/'simulation/fixture.glb')
    manifest['simulation_units']='m';manifest['simulation_materials']={p['name']:p['material'] for p in manifest['parts']}
    W=np.asarray(manifest['world_from_pad']);W[:3,3]/=1000;manifest['world_from_simulation_m']=W.tolist()


def export_all(p,out):
    out=Path(out);asset=ControllerAsset();parts,mods,mf=build(p,asset);cutshapes=mf.pop('_cut_shapes');collision_shapes=mf.pop('_collision_shapes')
    for folder in ('print','assembly_mesh','modifiers','simulation','reference','cuts'):(out/folder).mkdir(parents=True,exist_ok=True)
    # Remove stale v2 solids by file role, without erasing unrelated reports.
    for folder,valid in (('print',{n for n,v in zip(parts,mf['parts']) if v['is_printed']}),('assembly_mesh',set(parts)),('modifiers',set(mods)),('cuts',set(cutshapes))):
        for old in (out/folder).iterdir():
            if old.suffix in ('.stl','.step') and old.stem not in valid:old.unlink()
    for name,shape in collision_shapes.items():export_stl(shape,out/'simulation'/(name+'.stl'),tolerance=.04)
    for name,shape in cutshapes.items():export_stl(shape,out/'cuts'/(name+'.stl'),tolerance=.04)
    for row in mf['parts']:
        name=row['name'];shape=parts[name]
        print(name,round(shape.volume,2),len(shape.solids()),flush=True)
        if not shape.is_valid or len(shape.solids())!=1:raise RuntimeError(f'{name}: expected 1 valid solid, got {len(shape.solids())}')
        export_stl(shape,out/'assembly_mesh'/(name+'.stl'),tolerance=.1 if 'spring' in name else p.printing.tooth_mesh_tolerance if 'hirth' in name else p.printing.mesh_tolerance,angular_tolerance=.2)
        row.update(solid_volume_mm3=shape.volume,valid_solid=shape.is_valid,solid_count=len(shape.solids()))
        if row['is_printed']:
            free=shape
            if name.startswith('pad_'):
                # Free stock extends outward from its fixed floor. Assembly
                # surface is compressed by exactly the configured amount.
                contact=next(c for c in mf['contact_planes'] if c['pad']==name)
                normal=np.asarray(contact['normal'],float);free_t=contact['free_thickness_mm']
                ratio=free_t/(free_t-p.measured.pad_compression)
                seat=contact['fit_offset_mm']+contact['compressed_thickness_mm']
                tm=np.eye(4);tm[:3,:3]=np.eye(3)+(ratio-1)*np.outer(normal,normal);tm[:3,3]=(1-ratio)*seat*normal
                mesh=trimesh.load(out/'assembly_mesh'/(name+'.stl'),force='mesh');mesh.apply_transform(tm)
                free=mesh_solid(mesh);row['print_shape_state']='flat free-thickness pad, assembly compressed'
            rot=row['print_rotation_deg'];oriented=free.moved(Location((0,0,0),tuple(rot)));b=oriented.bounding_box();tr=(-b.min.X,-b.min.Y,-b.min.Z);oriented=oriented.moved(Location(tr))
            export_step(oriented,out/'print'/(name+'.step'));export_stl(oriented,out/'print'/(name+'.stl'),tolerance=p.printing.tooth_mesh_tolerance if 'hirth' in name else p.printing.mesh_tolerance)
            tm=np.eye(4);tm[:3,:3]=Rotation.from_euler('xyz',rot,degrees=True).as_matrix();tm[:3,3]=tr
            row['print_transform']=tm.tolist();row['print_bounds_mm']=list(oriented.bounding_box().size)
        elif name=='base_aluminum':export_step(shape,out/'aluminum_plate.step')
    # Build123d Compound preserves all material bodies in STEP.
    export_step(Compound(label='Xbox1708_fixture_v3',children=list(parts.values())),out/'assembly.step')
    basemeta=next(r for r in mf['parts'] if r['name'] in ('base_printed','base_aluminum'))
    if basemeta['is_printed']:
        for name,shape in mods.items():export_stl(shape.moved(location(np.asarray(basemeta['print_transform']))),out/'modifiers'/(name+'.stl'),tolerance=.05)
    export_simulation(out,mf)
    # Copy local asset evidence with provenance. Source is never modified.
    neutral=trimesh.util.concatenate(list(asset.visual_meshes().values()));neutral.export(out/'reference/controller_neutral_mm.stl')
    refs=[]
    sources=[asset.urdf]+[g[-1] for g in asset.geometry]
    vis=asset.urdf.parents[2]/'common/xbox_one_s'
    sources += [vis/'visual/manifest.json',vis/'visual/appearance.json',vis/'physical/full_controls.calibration.json']
    for src in sources:
        refs.append(dict(source=str(src.relative_to(ROOT.parent)),sha256=hashlib.sha256(src.read_bytes()).hexdigest()))
    mf['reference_asset']=dict(upstream='xbox-gamepad-physics 0.4.1 build resources',files=refs)
    # Remove absolute machine paths from output metadata.
    mf['reference_asset']['urdf']='../xbox_gamepad_physics/build/resources/robotwin/full_controls/mobility.urdf'
    from print_policy import apply_policy
    apply_policy(mf)
    (out/'manifest.json').write_text(json.dumps(mf,indent=2)+'\n');(out/'asset_measurements.json').write_text(json.dumps(mf['contact_fit'],indent=2)+'\n')
    templates(out,p,parts)
    return mf


def templates(out,p,parts):
    # True planar cut outlines, including Pair/USB reliefs. Normal section at
    # pad mid-thickness avoids drawing triangulated seams.
    svg=['<svg xmlns="http://www.w3.org/2000/svg" width="210mm" height="297mm" viewBox="0 0 210 297"><rect width="210" height="297" fill="white"/>']
    for name,axis,pos in [('pad_upper_jaw',1,(50,50)),('pad_lower_jaw',1,(130,50)),('pad_back',2,(50,110))]:
        s=parts[name];b=np.asarray(bounds(s));center=b.mean(0);u,v=[k for k in range(3) if k!=axis]
        # Exported STL section yields exact simple polygon, no fabricated slots.
        temp=out/'assembly_mesh'/(name+'.stl');mesh=trimesh.load(temp,force='mesh');sec=mesh.section(plane_origin=center,plane_normal=([0,-p.measured.rear_plane_slope,1] if name=='pad_back' else np.eye(3)[axis]))
        for loop in sec.discrete:
            xy=loop[:,[u,v]]-center[[u,v]]+pos
            svg.append('<polygon points="'+' '.join(f'{x:.3f},{y:.3f}' for x,y in xy)+'" fill="none" stroke="black" stroke-width=".2"/>')
        svg.append(f'<text x="{pos[0]-25}" y="{pos[1]-18}" font-size="3">{name}: 2mm stock</text>')
    svg.append('<path d="M20 170h100m-100 -2v4m100 -4v4" fill="none" stroke="black" stroke-width=".25"/><text x="20" y="180" font-size="4">100 mm / actual size 100%</text></svg>')
    (out/'pad_templates_A4.svg').write_text(''.join(svg))
    old=out/'pad_template_base_A4.svg'
    if old.exists():old.unlink()


def matrix_for_print(shape,rot):
    oriented=shape.moved(Location((0,0,0),tuple(rot)));b=oriented.bounding_box()
    T=np.eye(4);T[:3,:3]=Rotation.from_euler('xyz',rot,degrees=True).as_matrix();T[:3,3]=[-b.min.X,-b.min.Y,-b.min.Z]
    return T.tolist()


def export_coupons(p,out):
    out=Path(out);out.mkdir(parents=True,exist_ok=True);parts,_,mf=build(p);rows=[]
    def add(name,shape,source,rotation=None):
        row=next(r for r in mf['parts'] if r['name']==source);rot=rotation or row['print_rotation_deg']
        flat=shape.moved(Location((0,0,0),tuple(rot)));b=flat.bounding_box();flat=flat.moved(Location((-b.min.X,-b.min.Y,-b.min.Z)))
        if not flat.is_valid or len(flat.solids())!=1:raise ValueError(name+' disconnected coupon')
        export_stl(flat,out/(name+'.stl'),tolerance=.025);export_step(flat,out/(name+'.step'))
        rows.append(dict(name=name,source_part=source,volume_mm3=flat.volume,bounds_mm=list(flat.bounding_box().size),print_rotation_deg=rot,assembly_to_print_transform=matrix_for_print(shape,rot)))
    m,i=p.measured,p.installation;lo=m.top_center_y-m.center_span
    add('upper_jaw_section',parts['slider_upper_jaw'] & box(3,9,m.top_center_y-3,m.top_center_y+8,-30,27),'slider_upper_jaw')
    add('lower_jaw_section',parts['cradle_lower_jaw'] & box(-3,3,lo-8,lo+4,-30,27),'cradle_lower_jaw')
    back_axis=Axis((0,m.rear_center_y,m.rear_plane_z),(1,0,0));back_tilt=math.degrees(math.atan(m.rear_plane_slope))
    add('backrest_section',parts['cradle_lower_jaw'] & box(-3,3,14,32,-24,0).rotate(back_axis,back_tilt),'cradle_lower_jaw')
    add('hirth_fixed',parts['hirth_fixed'],'hirth_fixed');add('hirth_moving',parts['hirth_moving'],'hirth_moving')
    add('guide_rod_segment',parts['guide_rod_1'] & box(-14,-6,-5,25,-46,-38),'guide_rod_1')
    add('slider_segment',parts['slider_upper_jaw'] & box(-18,18,10,22,-50,-34),'slider_upper_jaw')
    add('insert_M3',parts['cradle_lower_jaw'] & box(19,28,65,75,-30,-24),'cradle_lower_jaw')
    add('insert_M4',parts['slider_upper_jaw'] & box(-8,8,-7,9,-50,-34),'slider_upper_jaw')
    name='base_aluminum' if i.aluminum_base else 'base_printed'
    Fpad=np.eye(4);Fpad[:3,:3]=Rotation.from_euler('x',i.tilt_deg,degrees=True).as_matrix();W=np.asarray(mf['world_from_pad']);Fworld=np.asarray(mf['fixed_frame_world_axes']);Fpad[:3,3]=Fworld.T@(W[:3,3]-mf['fixed_frame_world_origin_mm'])
    baseF=parts[name].moved(location(Fpad))
    add('clamp_band_block',baseF & box(60,100,-30,0,0,30),name,(0,0,0))
    add('insert_M5',baseF & box(27,43,-54,-38,0,30),name,(0,0,0))
    valid={r['name'] for r in rows}
    for old in out.iterdir():
        if old.suffix in ('.stl','.step') and old.stem not in valid:old.unlink()
    (out/'coupons.json').write_text(json.dumps(dict(design_revision=i.layout_revision,coupons=rows),indent=2)+'\n')
    # Companion manifest makes every coupon subject to the same print gate.
    coupon_manifest=dict(design_revision=i.layout_revision,artifact_type='coupons',parameters=dataclasses.asdict(p),pivot_pad_mm=mf['pivot_pad_mm'],
        cut_audit=[r for r in mf['cut_audit'] if r['part'] in ('hirth_fixed','hirth_moving')],
        parts=[dict(name=r['name'],is_printed=True,print_file=r['name']+'.stl',print_bounds_mm=r['bounds_mm'],print_transform=r['assembly_to_print_transform']) for r in rows],printability_exemptions=[],support_regions=[])
    from print_policy import apply_policy
    apply_policy(coupon_manifest)
    (out/'manifest.json').write_text(json.dumps(coupon_manifest,indent=2)+'\n')


@dataclasses.dataclass
class MarkerCard:
    squares_x: int = 8
    squares_y: int = 6
    square_mm: float = 20.0
    marker_mm: float = 15.0
    first_id: int = 60
    border_mm: float = 5.0
    plate_thickness: float = 3.0
    center_y: float = 165.0
    face_z: float = -42.0
    tilt_deg: float = 20.0
    slot_clearance: float = 0.2


def export_marker_card(p, out, card=None):
    """Removable keyed accessory on the existing guide-cap screws; body unchanged."""
    import sys
    sys.path.insert(0, str(ROOT/"src"))
    from gamepad_vla.workstation.gamepad_marker import board_rectangles
    from PIL import Image
    import cv2
    card = card or MarkerCard()
    out = Path(out); out.mkdir(parents=True, exist_ok=True)
    i, h, t = p.installation, p.hardware, p.printing
    width, height = card.squares_x*card.square_mm, card.squares_y*card.square_mm
    rotation = Rotation.from_euler("x",180+card.tilt_deg,degrees=True).as_matrix()
    center = np.array([0.,card.center_y,card.face_z])
    origin = center+rotation@np.array([-width/2,-height/2,0.])
    transform = np.eye(4);transform[:3,:3]=rotation;transform[:3,3]=origin/1000
    config = {"revision":"marker-card-v3", "board":{
        "dictionary":"DICT_4X4_100", "squares_x":card.squares_x,"squares_y":card.squares_y,
        "square_m":card.square_mm/1000,"marker_m":card.marker_mm/1000,"first_id":card.first_id,
        "legacy_pattern":False,"border_m":card.border_mm/1000},
        "gamepad_from_board":transform.tolist(),"units":"metres; board origin pattern top-left, x right y down z into backing",
        "card_parameters":dataclasses.asdict(card),
        "mount":{"parent":"moving cradle via existing guide cap", "existing_body_changed":False,
            "interface":"60.2 mm keyed slot around v3 guide-cap sides + two M3 screw axes",
            "bolt_x_mm":[-i.cap_bolt_x,i.cap_bolt_x],"bolt_y_mm":i.frame_top_y-4,
            "hardware":"2 M3x30 + 2 mm metal spacers behind marker anchor; verify 3-4 mm insert engagement and no bottoming",
            "observable":"cradle; pad pose assumes unchanged contact/no slip"}}
    # Nominal sticker geometry is for the synthetic slip test only. Grid
    # clearance checks found saved hand contacts here; do not certify a site.
    sticker_R = np.array([[.99995408946,.00179636540,.00941233468],
        [0.,.98227060178,-.18746857038],[-.00958222170,.18745996360,.98222550521]]) @ np.diag([1.,-1.,-1.])
    sticker_T=np.eye(4);sticker_T[:3,:3]=sticker_R;sticker_T[:3,3]=[-.002,-.015,.0168929854]
    contact_plane=sticker_T.copy();contact_plane[:3,:3]=sticker_R@np.diag([1.,-1.,-1.])
    config["front_contact_plane_in_gamepad"]=contact_plane.tolist()
    config["front_plane_note"]="CAD shell plane fit for backprojection; in-plane sticker placement is unknown and cancels between images"
    config["sticker"]={"dictionary":"DICT_4X4_100","id":90,"size_m":.015,"paper_border_m":.0005,
        "gamepad_from_sticker_synthetic_only":sticker_T.tolist(),"all_hand_states_clear":False,"hand_contact_is_placement_criterion":False,
        "controls_and_visibility_pass":True,
        "placement_status":"control/visibility candidate cleared; saved-hand contacts informational only during hand-free verification",
        "remove_before_production_collection":True}
    # Plate coordinates are board-centred: front paper at z=0, backing behind.
    location = Location(tuple(center),tuple(Rotation.from_matrix(rotation).as_euler("xyz",degrees=True)))
    backing = box(-width/2-card.border_mm,width/2+card.border_mm,
                  -height/2-card.border_mm,height/2+card.border_mm,.2,card.plate_thickness+.2).moved(location)
    anchor = box(-i.cap_outer_half_width-2,i.cap_outer_half_width+2,62,75,-54,-50)
    key_gap = i.cap_outer_half_width+card.slot_clearance/2
    for sign in (-1,1):
        anchor += box(sign*key_gap if sign>0 else -i.cap_outer_half_width-2,
                      i.cap_outer_half_width+2 if sign>0 else -key_gap,62,75,-50.1,-44)
        anchor -= cyl((h.cap_bolt_d+t.clearance_hole)/2,8,(sign*i.cap_bolt_x,71,-52),"z")
    # Short sloping rib reaches the lower edge of the tilted card; it stays
    # above the pad controls, rather than passing in front of the upper jaw.
    start=np.array([72.,-52.]);end=np.array([110.,-63.])
    direction=end-start;normal=np.array([-direction[1],direction[0]])/np.linalg.norm(direction)*5
    neck=prism([(-6.,*v) for v in (start-normal,end-normal,end+normal,start+normal)],(12,0,0))
    shape=anchor+neck+backing
    if not shape.is_valid or len(shape.solids())!=1:
        raise ValueError("Marker accessory must be one valid connected solid")
    print_shape=shape.moved(Location((0,0,0),(-card.tilt_deg,0,0)))
    b=print_shape.bounding_box()
    print_shape=print_shape.moved(Location((-b.min.X,-b.min.Y,-b.min.Z)))
    export_stl(print_shape,out/"marker_bracket.stl",tolerance=.05)
    export_step(shape,out/"marker_bracket.step")
    export_stl(shape,out/"marker_bracket_pad_mm.stl",tolerance=.05)
    config["bracket_bounds_pad_mm"]=[[shape.bounding_box().min.X,shape.bounding_box().min.Y,shape.bounding_box().min.Z],
                                      [shape.bounding_box().max.X,shape.bounding_box().max.Y,shape.bounding_box().max.Z]]
    # Exact vector template, independently rendered as polygons in SAPIEN.
    rectangles=board_rectangles(config)
    svg=['<svg xmlns="http://www.w3.org/2000/svg" width="210mm" height="297mm" viewBox="0 0 210 297">',
         '<rect width="210" height="297" fill="white"/>']
    left=(210-width)/2;top=30.
    for x,y,w,h0 in rectangles:
        svg.append(f'<rect x="{left+x*1000:.6f}" y="{top+y*1000:.6f}" width="{w*1000:.6f}" height="{h0*1000:.6f}" fill="black"/>')
    svg.extend([f'<rect x="{left-card.border_mm}" y="{top-card.border_mm}" width="{width+2*card.border_mm}" height="{height+2*card.border_mm}" fill="none" stroke="#777" stroke-width=".1"/>',
                '<path d="M30 210h100m-100 -2v4m100 -4v4" fill="none" stroke="black" stroke-width=".25"/>',
                '<text x="30" y="219" font-size="4">100 mm - print 100%, no fit to page</text>',
                '<text x="20" y="235" font-size="3.5">8x6 / square 20 mm / marker 15 mm / 4x4_100 IDs 60-83</text></svg>'])
    (out/"marker_card_A4.svg").write_text(''.join(svg))
    # Pillow PDF embeds a 600dpi black/white raster at true A4 physical size.
    dpi=600;px=dpi/25.4
    from PIL import ImageDraw, ImageFont
    label_font=ImageFont.load_default(size=round(4*px))
    page=Image.new("RGB",(round(210*px),round(297*px)),"white");draw=ImageDraw.Draw(page)
    for x,y,w,h0 in rectangles:
        draw.rectangle(tuple(round(v*px) for v in (left+x*1000,top+y*1000,left+(x+w)*1000,top+(y+h0)*1000)),fill="black")
    draw.line([(round(30*px),round(210*px)),(round(130*px),round(210*px))],fill="black",width=round(.25*px))
    for x in (30,130):draw.line([(round(x*px),round(208*px)),(round(x*px),round(212*px))],fill="black",width=round(.25*px))
    draw.text((round(30*px),round(216*px)),"100 mm / print actual size 100%",fill="black",font=label_font)
    page.save(out/"marker_card_A4.pdf","PDF",resolution=dpi)
    code=cv2.aruco.generateImageMarker(cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_100),90,6)
    sticker_svg=['<svg xmlns="http://www.w3.org/2000/svg" width="210mm" height="297mm" viewBox="0 0 210 297"><rect width="210" height="297" fill="white"/>']
    sticker_page=Image.new("RGB",(round(210*px),round(297*px)),"white");sticker_draw=ImageDraw.Draw(sticker_page)
    for yy,xx in np.argwhere(code==0):
        x,y=30+xx*2.5,30+yy*2.5
        sticker_svg.append(f'<rect x="{x}" y="{y}" width="2.5" height="2.5"/>')
        sticker_draw.rectangle(tuple(round(v*px) for v in (x,y,x+2.5,y+2.5)),fill="black")
    sticker_draw.rectangle(tuple(round(v*px) for v in (29.5,29.5,45.5,45.5)),outline=(170,170,170),width=max(1,round(.1*px)))
    sticker_svg.append('<rect x="29.5" y="29.5" width="16" height="16" fill="none" stroke="#888" stroke-width=".1"/><path d="M30 90h100m-100 -2v4m100 -4v4" fill="none" stroke="black" stroke-width=".25"/><text x="30" y="100" font-size="4">100 mm / ID90 black edge 15 mm</text></svg>')
    sticker_draw.line([(round(30*px),round(90*px)),(round(130*px),round(90*px))],fill="black",width=round(.25*px))
    for x in (30,130):sticker_draw.line([(round(x*px),round(88*px)),(round(x*px),round(92*px))],fill="black",width=round(.25*px))
    sticker_draw.text((round(30*px),round(96*px)),"100 mm / ID90 black edge 15 mm",fill="black",font=label_font)
    (out/"sticker_ID90_A4.svg").write_text(''.join(sticker_svg))
    sticker_page.save(out/"sticker_ID90_A4.pdf","PDF",resolution=dpi)
    config.update(design_revision=p.installation.layout_revision, artifact_type='marker', parameters=dataclasses.asdict(p),
        parts=[dict(name='marker_bracket',is_printed=True,print_file='marker_bracket.stl',print_bounds_mm=list(print_shape.bounding_box().size),print_transform=matrix_for_print(shape,(-card.tilt_deg,0,0)))],
        printability_exemptions=[],support_regions=[])
    from print_policy import apply_policy
    apply_policy(config)
    (out/"manifest.json").write_text(json.dumps(config,indent=2)+"\n")
    print(f"Marker card accessory, transform and 1:1 templates: {out}")


def main():
    ap=argparse.ArgumentParser(description=__doc__);ap.add_argument('--out',type=Path,default=HERE/'out');ap.add_argument('--aluminum',action='store_true');ap.add_argument('--tpu-pads',action='store_true');ap.add_argument('--coupons',action='store_true');ap.add_argument('--marker-card',action='store_true');ap.add_argument('--parameters',type=Path)
    a=ap.parse_args();p=Parameters();p.installation.aluminum_base=a.aluminum;p.measured.use_printed_tpu_pads=a.tpu_pads
    if a.parameters:
        overrides=json.loads(a.parameters.read_text())
        for group,fields in overrides.items():
            for k,v in fields.items():setattr(getattr(p,group),k,v)
    if a.coupons:export_coupons(p,HERE/'out_coupons' if a.out==HERE/'out' else a.out)
    elif a.marker_card:export_marker_card(p,HERE/'out_marker' if a.out==HERE/'out' else a.out)
    else:export_all(p,a.out)

if __name__=='__main__':main()
