#!/usr/bin/env python3
"""Task V: printable Xbox 1708 fixture. Units: mm, degrees, N.

Run with .local/cad-env/bin/python hardware/gamepad_fixture/fixture.py.
All exported assembly geometry uses controller-local coordinates; print files
are separate rigid transforms with their lowest point on the build plate.
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
from build123d import (Align, Axis, Box, Circle, Color, Compound, Cone, Cylinder, Face, Helix, Location,
                      Plane, Shell, Solid, Text, Wire, export_step, export_stl,
                      extrude, loft, sweep)

from asset_geometry import ControllerAsset, DEFAULT_URDF

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[1]
SCENE = ROOT / "configs/workcell/station_g20_v4_4_raised_interim.json"


# PARAMETER BLOCK. Change measured values here, then regenerate AND recheck.
@dataclasses.dataclass
class ControllerMeasurements:
    width: float = 153.0
    height: float = 104.6
    depth: float = 50.2
    top_center_y: float = 49.05
    center_span: float = 72.6
    center_span_uncertainty: float = 6.0
    top_center_depth: float = 28.0
    bottom_center_depth: float = 26.0
    shoulder_inner_gap: float = 46.0
    upper_jaw_width: float = 20.0
    lower_jaw_width: float = 24.0
    xbox_button_y_max: float = 45.7
    top_lip_y_min: float = 47.5
    rear_support_width: float = 30.0
    rear_support_height: float = 36.0
    jaw_pad_thickness: float = 2.0
    back_pad_thickness: float = 2.0
    base_pad_thickness: float = 1.0
    shoe_pad_thickness: float = 1.0
    pad_compression: float = 0.25
    base_pad_compression: float = 0.25
    pad_proud_uncompressed: float = 0.5
    use_printed_tpu_pads: bool = False
    pad_material: str = "adhesive_silicone_sheet"


@dataclasses.dataclass
class Installation:
    layout_revision: str = "v2-Z"
    # The quaternion is read from the specified scene; do not round to 40 deg.
    table_edge_x: float = 718.4
    table_center_y: float = 625.2
    tabletop_z: float = 767.0
    pivot_pad_y: float = -40.0
    pivot_pad_z: float = -85.0
    major_mark_deg: float = 40.0
    base_width: float = 220.0
    base_depth: float = 160.0
    printed_base_thickness: float = 20.0
    aluminum_base: bool = False
    aluminum_thickness: float = 6.0
    aluminum_pad_tray_thickness: float = 0.8
    clamp_center_x: float = 70.0
    clamp_center_y: float = -26.0
    clamp_zone_width: float = 44.0
    clamp_zone_depth: float = 40.0
    clamp_band_depth: float = 60.0
    pressure_shoe_thickness: float = 16.0
    flange_width: float = 96.0
    flange_depth: float = 76.0
    flange_thickness: float = 10.0
    flange_center_y: float = -120.0
    bolt_x: float = 34.0
    bolt_y: tuple = (-142.0, -98.0)
    column_x_min: float = -30.0
    column_x_max: float = -16.0
    column_foot_y: float = -130.0
    column_foot_width: float = 40.0
    column_section_depth: float = 32.0
    column_spine_stop_from_axis: float = 38.0
    column_root_gusset_height: float = 24.0
    rib_width: float = 6.0
    rib_height: float = 18.0
    flange_rib_clearance: float = 0.3
    lower_seat_length: float = 10.0
    spring_socket_depth: float = 6.0
    upper_spine_overlap: float = 7.0
    slider_min_span: float = 64.0
    slider_max_span: float = 84.0
    slider_length: float = 40.0
    slider_center_y: float = 12.0
    slider_width: float = 32.0
    slider_depth: float = 16.0
    lock_center_y: float = 1.0
    guide_x: float = 9.0
    guide_z: float = -42.0
    guide_min_y: float = -22.6
    guide_max_y: float = 66.0
    cap_spring_seat_y: float = 62.0
    slider_spring_seat_y: float = 32.0
    lower_guide_seat_top_y: float = -16.6
    frame_back_z: float = -29.0
    frame_front_z: float = -23.0
    frame_outer_half_width: float = 21.0
    frame_slot_half_width: float = 11.0
    frame_slot_min_y: float = 15.0
    cap_outer_half_width: float = 23.0
    cap_bolt_x: float = 16.0
    frame_top_y: float = 75.0
    rear_block_back_z: float = -23.0
    rear_pocket_width: float = 24.0
    rear_pocket_height: float = 28.0
    shell_surface_grid_mm: float = 2.0
    jaw_wall: float = 6.0
    jaw_front_lip_z: float = 20.75
    jaw_front_end_z: float = 25.0
    top_pad_z_min: float = -7.0
    top_pad_z_max: float = 8.0
    lower_pad_z_min: float = -7.0
    lower_pad_z_max: float = 12.0
    pivot_fixed_x_min: float = -28.0
    pivot_fixed_face_x: float = -16.0
    pivot_moving_x_max: float = 8.0
    pivot_disc_radius: float = 27.0
    fixed_disc_radius: float = 34.0
    spring_stop_outer_d: float = 7.6
    tooth_outer_radius: float = 25.0
    tooth_inner_radius: float = 10.0
    teeth: int = 36
    tooth_height: float = 1.6
    knob_radius: float = 22.0
    knob_thickness: float = 14.0
    # Generic C-clamp mounted with fixed jaw ABOVE and screw/tommy bar BELOW.
    # These are provisional procurement envelopes, not a specific measured tool.
    c_upper_arm_height: float = 60.0
    c_arm_width: float = 32.0
    c_spine_y: float = 64.0
    c_spine_depth: float = 16.0
    c_table_stack_thickness: float = 35.0
    c_lower_arm_drop_min: float = 10.0
    c_lower_arm_drop_max: float = 25.0
    c_handle_below_table_bottom: float = 65.0
    c_handle_sweep_radius: float = 50.0
    c_handle_sweep_height: float = 12.0
    c_operator_extra_radius: float = 25.0
    c_operator_hand_height: float = 50.0
    c_operator_approach_y: float = 140.0
    c_spindle_d: float = 12.0
    base_stiffness_fraction: float = 0.4


@dataclasses.dataclass
class Hardware:
    pivot_bolt_d: float = 6.0
    pivot_bolt_length: float = 60.0
    pivot_nut_af: float = 10.0
    pivot_nut_height: float = 5.0
    pivot_washer_od: float = 18.0
    pivot_washer_thickness: float = 1.6
    base_bolt_d: float = 5.0
    base_bolt_head_d: float = 10.0
    base_nut_af: float = 8.0
    base_nut_height: float = 4.0
    cap_bolt_d: float = 3.0
    cap_insert_od: float = 4.6
    cap_insert_length: float = 4.0
    cap_insert_pilot: float = 4.1
    lock_bolt_d: float = 4.0
    lock_insert_od: float = 6.0
    lock_insert_length: float = 6.0
    lock_insert_pilot: float = 5.4
    guide_d: float = 6.0
    spring_od: float = 9.5
    spring_wire: float = 0.7
    spring_free_length: float = 40.0
    spring_rate: float = 0.35
    spring_solid_height_max: float = 10.0
    spring_active_coils: float = 10.0
    clamp_spreader_od: float = 24.0
    clamp_spreader_id: float = 8.4
    clamp_spreader_thickness: float = 2.0
    clamp_force_min: float = 1000.0
    clamp_force_max: float = 2000.0
    clamp_foot_d_min: float = 15.0
    clamp_foot_d_max: float = 20.0
    pivot_hand_torque_Nm: float = 0.5
    pivot_nut_factor: float = 0.2
    finger_force_N: float = 30.0
    finger_lever_mm: float = 110.0
    design_pivot_torque_Nm: float = 4.0


@dataclasses.dataclass
class Printing:
    hard_shell_clearance: float = 0.25
    clearance_hole: float = 0.5
    guide_diametral_clearance: float = 0.6
    guide_socket_clearance: float = 0.25
    tooth_axial_clearance: float = 0.12
    nut_af_clearance: float = 0.3
    counterbore_clearance: float = 0.4
    build_limit: float = 300.0
    mesh_tolerance: float = 0.08
    tooth_mesh_tolerance: float = 0.025
    material_E_MPa: float = 1500.0
    short_term_compression_allow_MPa: float = 10.0
    short_term_bending_allow_MPa: float = 15.0
    tooth_shear_allow_MPa: float = 5.0
    effective_loaded_teeth: int = 4
    effective_solid_area_fraction: float = 0.4
    table_local_gap_span: float = 10.0


@dataclasses.dataclass
class Parameters:
    measured: ControllerMeasurements = dataclasses.field(default_factory=ControllerMeasurements)
    installation: Installation = dataclasses.field(default_factory=Installation)
    hardware: Hardware = dataclasses.field(default_factory=Hardware)
    printing: Printing = dataclasses.field(default_factory=Printing)


def box(x0, x1, y0, y1, z0, z1):
    return Box(x1-x0, y1-y0, z1-z0).moved(Location(((x0+x1)/2, (y0+y1)/2, (z0+z1)/2)))


def cyl(radius, length, center, axis="z"):
    rotation = {"x": (0, 90, 0), "y": (90, 0, 0), "z": (0, 0, 0)}[axis]
    return Cylinder(radius, length).moved(Location(center, rotation))


def prism(points, direction):
    face = Face(Wire.make_polygon(points, close=True))
    length = np.linalg.norm(direction)
    return extrude(face, amount=float(length), dir=tuple(np.asarray(direction)/length))


def hex_prism(af, length, center, axis="z"):
    r = af/math.sqrt(3)
    pts = [(r*math.cos(k*math.pi/3), r*math.sin(k*math.pi/3), -length/2) for k in range(6)]
    s = prism(pts, (0, 0, length))
    return s.moved(Location(center, {"x": (0, 90, 0), "y": (90, 0, 0), "z": (0, 0, 0)}[axis]))


def mesh_solid(mesh):
    """Sew the *actual* convex-decomposition triangles; no fitted shell model."""
    faces = [Face(Wire.make_polygon([tuple(v) for v in tri], close=True)) for tri in mesh.triangles]
    return Solid(Shell(faces))


def shell_cutters(asset, scale_xyz,clearance=0.):
    # Only connected convex pieces intersecting the narrow central support.
    result = []
    for name, link, origin, _mesh, path in asset.geometry:
        if name not in {"shell", "shell_front_plate"}:
            continue
        # Preserve OBJ 'o hull_N' boundaries. Merging touching hull vertices
        # first destroys the decomposition and produces nonmanifold seams.
        vertices,groups,current=[],[],[]
        for line in path.read_text().splitlines():
            fields=line.split()
            if not fields: continue
            if fields[0]=="v": vertices.append([float(v)*1000 for v in fields[1:4]])
            elif fields[0]=="o":
                if current: groups.append(current);current=[]
            elif fields[0]=="f": current.append([int(v.split('/')[0])-1 for v in fields[1:4]])
        if current: groups.append(current)
        for faces in groups:
            component=trimesh.Trimesh(vertices=np.array(vertices),faces=faces,process=True)
            component.remove_unreferenced_vertices()
            component.apply_transform(asset.link_transform(link)@origin)
            component.apply_scale(scale_xyz)
            if clearance:
                # Minkowski sum of each convex hull with an axis-aligned
                # tolerance cube. Its support is >= clearance in EVERY normal
                # direction, unlike a translation along only jaw closing Y.
                import itertools
                corners=np.array(list(itertools.product((-clearance,clearance),repeat=3)))
                component=trimesh.convex.convex_hull((component.vertices[:,None,:]+corners[None,:,:]).reshape(-1,3))
            lo, hi = component.bounds
            if lo[0] < 18 and hi[0] > -18:
                result.append(mesh_solid(component))
    return result


def subtract_shifted(shape, cutters, xyz):
    for tool in cutters:
        shape = shape - tool.moved(Location(xyz))
    return shape


def exterior_cutter(asset, axis, side, bounds, scale, spacing=2.):
    """A solid behind the outermost ray/triangle intersection of the asset.

    CoACD contains internal faces and micron-wide seams. A first-hit envelope
    closes those internal cavities without interpreting their walls as contact
    surfaces. This is a sampled piecewise-planar surface, not an analytic fit.
    """
    mesh=asset.shell().copy();mesh.apply_scale(scale)
    u,v=[a for a in range(3) if a!=axis]
    aa=np.linspace(bounds[0],bounds[1],math.ceil((bounds[1]-bounds[0])/spacing)+1)
    bb=np.linspace(bounds[2],bounds[3],math.ceil((bounds[3]-bounds[2])/spacing)+1)
    origins=np.zeros((len(aa)*len(bb),3));origins[:,axis]=side*100
    origins[:,u]=np.repeat(aa,len(bb));origins[:,v]=np.tile(bb,len(aa))
    directions=np.zeros_like(origins);directions[:,axis]=-side
    hits,indices,_=mesh.ray.intersects_location(origins,directions,multiple_hits=True)
    heights=np.full(len(origins),np.nan)
    for n in range(len(origins)):
        values=hits[indices==n,axis]
        if len(values): heights[n]=max(values) if side>0 else min(values)
    if np.isnan(heights).any():
        raise ValueError(f"No exterior shell at {origins[np.isnan(heights)].tolist()}")
    front=origins.copy();front[:,axis]=heights
    rear=front.copy();rear[:,axis]=-side*40
    verts=np.vstack([front,rear]);N=len(front);faces=[]
    def tri(a,b,c): faces.extend([[a,b,c],[a+N,c+N,b+N]])
    for a in range(len(aa)-1):
        for b in range(len(bb)-1):
            k=a*len(bb)+b
            tri(k,k+len(bb),k+len(bb)+1);tri(k,k+len(bb)+1,k+1)
    boundary=list(range(len(bb)))+[a*len(bb)+len(bb)-1 for a in range(1,len(aa))]+[(len(aa)-1)*len(bb)+b for b in range(len(bb)-2,-1,-1)]+[a*len(bb) for a in range(len(aa)-2,0,-1)]
    for a,b in zip(boundary,boundary[1:]+boundary[:1]):faces.extend([[a,b,b+N],[a,b+N,a+N]])
    envelope=trimesh.Trimesh(vertices=verts,faces=faces,process=True);envelope.fix_normals()
    return mesh_solid(envelope)


def serrations(p, fixed=True):
    """Piecewise-planar radial V teeth with complementary mating surfaces.

    Each half pitch is a ruled ramp. The moving lower face uses the very same
    sampled surface, translated by the axial fit clearance, so assembly is
    complementary rather than two coincident sets of tooth peaks.
    """
    i, t = p.installation, p.printing
    result = []
    for k in range(i.teeth*2):
        a, b = 2*math.pi*k/(i.teeth*2), 2*math.pi*(k+1)/(i.teeth*2)
        ha, hb = (0., i.tooth_height) if k % 2 == 0 else (i.tooth_height, 0.)
        points0, points1 = [], []
        if fixed:
            xbase = i.pivot_fixed_face_x - 0.15
            for r, ang, h in [(i.tooth_inner_radius,a,ha),(i.tooth_outer_radius,a,ha),
                              (i.tooth_outer_radius,b,hb),(i.tooth_inner_radius,b,hb)]:
                points0.append((xbase, i.pivot_pad_y+r*math.cos(ang), i.pivot_pad_z+r*math.sin(ang)))
                points1.append((i.pivot_fixed_face_x+h, i.pivot_pad_y+r*math.cos(ang), i.pivot_pad_z+r*math.sin(ang)))
        else:
            xbase = i.pivot_fixed_face_x+i.tooth_height+0.15+t.tooth_axial_clearance
            for r, ang, h in [(i.tooth_inner_radius,a,ha),(i.tooth_outer_radius,a,ha),
                              (i.tooth_outer_radius,b,hb),(i.tooth_inner_radius,b,hb)]:
                points0.append((i.pivot_fixed_face_x+h+t.tooth_axial_clearance, i.pivot_pad_y+r*math.cos(ang), i.pivot_pad_z+r*math.sin(ang)))
                points1.append((xbase, i.pivot_pad_y+r*math.cos(ang), i.pivot_pad_z+r*math.sin(ang)))
        # Triangulated ramp: radial edge heights are constant, so the four
        # top vertices need not share a plane. Sewing facets avoids a warped
        # quadrilateral being silently flattened by the CAD kernel.
        vertices=np.array(points0+points1)
        faces=[[0,2,1],[0,3,2],[4,5,6],[4,6,7],
               [0,1,5],[0,5,4],[1,2,6],[1,6,5],
               [2,3,7],[2,7,6],[3,0,4],[3,4,7]]
        mesh=trimesh.Trimesh(vertices=vertices,faces=faces,process=True)
        mesh.fix_normals()
        result.append(mesh_solid(mesh))
    return result


def mechanics(p):
    i, h, t = p.installation, p.hardware, p.printing
    area = i.clamp_zone_width*i.clamp_zone_depth
    sheet_area=(i.clamp_zone_width-2)*(i.clamp_zone_depth-2)
    force = h.clamp_force_max
    shoe_t = i.pressure_shoe_thickness
    shoe_I = i.clamp_zone_width*shoe_t**3/12
    base_t = i.aluminum_thickness if i.aluminum_base else i.printed_base_thickness
    modulus = 69000. if i.aluminum_base else t.material_E_MPa
    base_I = i.clamp_zone_width*base_t**3/12
    span = t.table_local_gap_span
    pitch = 2*math.pi*(i.tooth_outer_radius+i.tooth_inner_radius)/2/i.teeth
    tooth_area = (i.tooth_outer_radius-i.tooth_inner_radius)*2*math.pi*i.tooth_inner_radius/i.teeth
    mean_r = (i.tooth_outer_radius+i.tooth_inner_radius)/2
    preload = h.pivot_hand_torque_Nm/(h.pivot_nut_factor*h.pivot_bolt_d/1000)
    separating = h.design_pivot_torque_Nm*1000/mean_r*(pitch/2)/i.tooth_height
    min_span, max_span = i.slider_min_span, i.slider_max_span
    spring_nominal = i.cap_spring_seat_y-i.slider_spring_seat_y
    lengths = [spring_nominal-(s-p.measured.center_span) for s in (min_span,p.measured.center_span,max_span)]
    beam_span=abs(i.flange_center_y-i.clamp_center_y)
    beam_width=i.flange_width
    beam_I=beam_width*base_t**3/12
    beam_E=modulus if i.aluminum_base else modulus*i.base_stiffness_fraction
    applied_M=h.design_pivot_torque_Nm*1000
    beam_stress=(h.finger_force_N*beam_span+applied_M)*(base_t/2)/beam_I
    beam_deflection=h.finger_force_N*beam_span**3/(3*beam_E*beam_I)+applied_M*beam_span**2/(2*beam_E*beam_I)
    beam_rotation=h.finger_force_N*beam_span**2/(2*beam_E*beam_I)+applied_M*beam_span/(beam_E*beam_I)
    return {
        "scope": "Analytic estimates, not FEA or measured printed-material properties; N/mm/MPa.",
        "clamp_each_N": [h.clamp_force_min,h.clamp_force_max],
        "clamp_zone_full_area_mm2": area,
        "shoe_bottom_sheet_actual_contact_area_mm2":sheet_area,
        "base_pressure_under_shoe_sheet_MPa_at_2kN":force/sheet_area,
        "base_compression_nominal_MPa_at_2kN": force/area,
        "base_compression_effective_MPa_at_2kN": force/(area*t.effective_solid_area_fraction),
        "base_elastic_compression_mm": force*base_t/(area*modulus),
        "base_unbacked_edge_assumed_mm": span,
        "base_unbacked_edge_cantilever_stress_MPa": force*span*(base_t/2)/base_I,
        "base_unbacked_edge_cantilever_deflection_mm": force*span**3/(3*modulus*base_I),
        "shoe_foot_local_pressure_MPa_15mm": force/(math.pi*(h.clamp_foot_d_min/2)**2),
        "shoe_pressure_with_standard_spreader_MPa": force/(math.pi*(h.clamp_spreader_od**2-h.clamp_spreader_id**2)/4),
        "shoe_40mm_simply_supported_stress_MPa": (force*i.clamp_zone_depth/4)*(shoe_t/2)/shoe_I,
        "shoe_40mm_simply_supported_deflection_mm": force*i.clamp_zone_depth**3/(48*t.material_E_MPa*shoe_I),
        "spring_installed_lengths_mm_at_min_nominal_max": lengths,
        "spring_total_closing_force_N_at_min_nominal_max": [2*h.spring_rate*(h.spring_free_length-L) for L in lengths],
        "spring_solid_height_margin_mm": min(lengths)-h.spring_solid_height_max,
        "spring_wire_rate_crosscheck_N_per_mm": 79000*h.spring_wire**4/(8*(h.spring_od-h.spring_wire)**3*h.spring_active_coils),
        "pivot_applied_finger_torque_Nm": h.finger_force_N*h.finger_lever_mm/1000,
        "pivot_design_torque_Nm": h.design_pivot_torque_Nm,
        "pivot_tooth_shear_capacity_Nm": t.effective_loaded_teeth*tooth_area*t.tooth_shear_allow_MPa*mean_r/1000,
        "pivot_preload_N": preload,
        "pivot_tooth_separating_force_N": separating,
        "pivot_separation_safety_factor": preload/separating,
        "z_layout_base_root_to_clamp_span_mm":beam_span,
        "z_layout_effective_strip_width_mm":beam_width,
        "z_layout_effective_modulus_MPa":beam_E,
        "z_layout_fully_unbacked_base_stress_MPa":beam_stress,
        "z_layout_fully_unbacked_base_deflection_mm":beam_deflection,
        "z_layout_fully_unbacked_base_rotation_deg":math.degrees(beam_rotation),
        "z_layout_bending_scope":"Conservative fully unsupported root-to-clamp strip with the configured end force and pivot design moment; actual underside is continuously supported by table and thin sheet. No loaded-pose certification.",
        "assumptions": dataclasses.asdict(t),
    }


def build(p=None, asset=None):
    p = p or Parameters()
    asset = asset or ControllerAsset()
    m, i, h, t = p.measured, p.installation, p.hardware, p.printing
    # Derived stop locations keep their specified span when a caliper updates
    # the nominal controller span. Guide length follows the lower blind seat.
    i.lower_guide_seat_top_y=i.slider_center_y-i.slider_length/2+i.slider_min_span-m.center_span
    i.guide_min_y=i.lower_guide_seat_top_y-i.spring_socket_depth
    scene = json.loads(SCENE.read_text())
    pose = scene["gamepad"]["root_pose"]
    world_R = Rotation.from_quat(pose[4:]+pose[3:4]).as_matrix()
    world_F = np.array([[0,1,0],[-1,0,0],[0,0,1]],float)
    F_R_pad = world_F.T @ world_R
    angle = math.degrees(math.atan2(F_R_pad[2,1], F_R_pad[1,1]))
    datum = np.array([i.table_edge_x,i.table_center_y,i.tabletop_z])
    root_F = world_F.T @ (np.array(pose[:3])*1000-datum)
    pivot_F = root_F+F_R_pad @ np.array([0,i.pivot_pad_y,i.pivot_pad_z])
    pad_from_F = Location(tuple(-F_R_pad.T@root_F), tuple(Rotation.from_matrix(F_R_pad.T).as_euler("xyz",degrees=True)))
    base_float = m.pad_proud_uncompressed-m.base_pad_compression
    tray_t = i.aluminum_pad_tray_thickness if i.aluminum_base else 0.
    base_t = i.aluminum_thickness if i.aluminum_base else i.printed_base_thickness
    base_z = base_float+tray_t
    flange_z = base_z+base_t
    parts, meta, modifiers = {}, {}, {}

    def add(name, shape, material="PETG", fixed=False, rotation=(0,0,0), note=""):
        if fixed:
            shape = shape.moved(pad_from_F)
        shape.label = name
        parts[name] = shape
        meta[name] = {"name":name,"material":material,"is_printed": material in {"PETG","TPU95A"},
                      "print_rotation_deg": list(rotation),"fixed_in_base_frame":fixed,"note":note}

    # Flat underside; all structural ribs are on top. Recess stops before edge.
    base = box(-i.base_width/2,i.base_width/2,-i.base_depth,0,base_z,flange_z)
    pad_outline = (-i.base_width/2+4,i.base_width/2-4,-i.base_depth+4,-4)
    if not i.aluminum_base:
        base -= box(*pad_outline,base_float-0.1,base_float+m.base_pad_thickness-m.pad_proud_uncompressed)
    for x in (-i.bolt_x,i.bolt_x):
        for y in i.bolt_y:
            base -= cyl((h.base_bolt_d+t.clearance_hole)/2,base_t+2,(x,y,base_z+base_t/2))
            cs_r=(h.base_bolt_head_d+t.counterbore_clearance)/2
            hole_r=(h.base_bolt_d+t.clearance_hole)/2
            cs_h=cs_r-hole_r
            base-=Cone(cs_r,hole_r,cs_h).moved(Location((x,y,base_z+cs_h/2)))
    if not i.aluminum_base:
        # All ribs remain on the INNER side. The complete outer 60 mm band
        # remains flat over its full width, rather than two corner islands.
        for side in (-1,1):
            for y0,end_y in ((-142.,-116.),(-108.,-65.)):
                x0=side*(i.flange_width/2+i.flange_rib_clearance+i.rib_width/2)
                x1=side*76.
                a=np.array([x0,y0,flange_z]); b=np.array([x1,end_y,flange_z])
                d=b-a; n=np.array([-d[1],d[0],0]);n=n/np.linalg.norm(n)*i.rib_width/2
                rib=prism([tuple(a-n),tuple(b-n),tuple(b-n+[0,0,2]),tuple(a-n+[0,0,i.rib_height])],tuple(2*n))
                base += rib
        base-=box(-i.base_width/2+4,i.base_width/2-4,-i.clamp_band_depth-1.8,-i.clamp_band_depth-1.2,flange_z-.35,flange_z+.1)
    add("base_aluminum" if i.aluminum_base else "base_printed",base,"Aluminum_6061" if i.aluminum_base else "PETG",True,rotation=(angle+90,0,0),note="Z layout: inner foot, continuous flat 220x60 outer clamp band. Print on long inboard edge with brim/support under ribs. One full-band 100% infill modifier.")
    if i.aluminum_base:
        tray=box(-i.base_width/2,i.base_width/2,-i.base_depth,0,base_float,base_z)-box(*pad_outline,base_float-.1,base_float+m.base_pad_thickness-m.pad_proud_uncompressed)
        for x in (-i.bolt_x,i.bolt_x):
            for y in i.bolt_y:
                tray-=cyl(6,2,(x,y,base_float+.4))
        add("aluminum_pad_recess_tray",tray,fixed=True,rotation=(angle+180,0,0),note="0.8 mm printed liner; print flat back down and shallow pocket up, avoiding a wide bridge.")
    # Column with a broad four-bolt foot and a rearward side cheek.
    flange=box(-i.flange_width/2,i.flange_width/2,i.flange_center_y-i.flange_depth/2,i.flange_center_y+i.flange_depth/2,flange_z,flange_z+i.flange_thickness)
    for x in (-i.bolt_x,i.bolt_x):
        for y in i.bolt_y:
            flange-=cyl((h.base_bolt_d+t.clearance_hole)/2,12,(x,y,flange_z+5))
            flange-=hex_prism(h.base_nut_af+t.nut_af_clearance,h.base_nut_height+.2,(x,y,flange_z+i.flange_thickness-h.base_nut_height/2+.1))
    py,pz=pivot_F[1],pivot_F[2]
    foot_center=np.array([i.column_foot_y,flange_z+16])
    head_center=np.array([py,pz])
    direction=head_center-foot_center;direction/=np.linalg.norm(direction)
    normal=np.array([-direction[1],direction[0]])*i.column_section_depth/2
    yz=[foot_center-normal,head_center-normal,head_center+normal,foot_center+normal]
    outline=[(i.column_x_min,*point) for point in yz]
    column=flange+prism(outline,(i.column_x_max-i.column_x_min,0,0))
    # The broad rib follows the SAME slanted load path; it is not a separate
    # fork/prong. Stop 38 mm from the axis to clear the moving 27 mm hub.
    spine_end=head_center-direction*i.column_spine_stop_from_axis
    spine=[(i.column_x_max,*point) for point in (foot_center-normal,spine_end-normal,spine_end+normal,foot_center+normal)]
    column+=prism(spine,(36,0,0))
    # Root gussets, mirrored about the narrow cheek, inside the flange.
    for y in (i.flange_center_y-24,i.flange_center_y+20):
        column+=prism([(-20,y,flange_z+7),(22,y,flange_z+7),(-20,y,flange_z+i.column_root_gusset_height)],(0,6,0))
    # Build the head in pad axes, then map just it into the base frame.
    fixed_disc=cyl(i.fixed_disc_radius,12,(-22,i.pivot_pad_y,i.pivot_pad_z),"x")
    for tooth in serrations(p,True):
        fixed_disc+=tooth
    fixed_disc-=cyl((h.pivot_bolt_d+t.clearance_hole)/2,20,(-22,i.pivot_pad_y,i.pivot_pad_z),"x")
    for label in (10,20,30,40,50,60,70):
        delta=label-i.major_mark_deg
        tick=box(-16.8,-15.8,i.pivot_pad_y+27.5,i.pivot_pad_y+(33 if label==40 else 31.5),i.pivot_pad_z-(.5 if label==40 else .25),i.pivot_pad_z+(.5 if label==40 else .25))
        tick=tick.rotate(Axis((0,i.pivot_pad_y,i.pivot_pad_z),(1,0,0)),delta)
        fixed_disc-=tick
        a=math.radians(delta+4)
        number=extrude(Text(str(label),font_size=2.5),amount=.7).moved(Location((-16.6,i.pivot_pad_y+30.5*math.cos(a),i.pivot_pad_z+30.5*math.sin(a)),(90,0,90)))
        fixed_disc-=number
    fixed_disc_F=fixed_disc.moved(Location(tuple(root_F),tuple(Rotation.from_matrix(F_R_pad).as_euler("xyz",degrees=True))))
    column+=fixed_disc_F
    column-=cyl((h.pivot_bolt_d+t.clearance_hole)/2,32,(-22,py,pz),"x")
    # Reopen foot holes AFTER adding the upright/gussets, including nut access.
    for x in (-i.bolt_x,i.bolt_x):
        for y in i.bolt_y:
            column-=cyl((h.base_bolt_d+t.clearance_hole)/2,32,(x,y,flange_z+10))
            column-=hex_prism(h.base_nut_af+t.nut_af_clearance,20,(x,y,flange_z+i.flange_thickness-h.base_nut_height+10))
    add("column",column,fixed=True,rotation=(0,-90,0),note="M6 axis; keyed nominal 40 mark corresponds to scene quaternion, not rounded 40 deg.")

    # Asset-derived shell cuts; only width is scaled symmetrically. The center
    # contour is translated to follow a changed measured center span.
    scale=[m.width/153.,m.height/104.6,m.depth/50.2]
    print("Sewing actual central shell triangles ...",flush=True)
    cutters=shell_cutters(asset,scale)
    clearance_cutters=shell_cutters(asset,scale,t.hard_shell_clearance)
    rear_tools=[exterior_cutter(asset,2,-1,(-m.rear_support_width/2-1,m.rear_support_width/2+1,-m.rear_support_height/2-1,m.rear_support_height/2+1),scale,i.shell_surface_grid_mm)]
    upper_delta=m.top_center_y-49.05*scale[1]
    lower_y=m.top_center_y-m.center_span
    lower_delta=lower_y+23.55*scale[1]
    upper_tools=[exterior_cutter(asset,1,1,(-m.upper_jaw_width/2-1,m.upper_jaw_width/2+1,i.top_pad_z_min,i.top_pad_z_max),scale,i.shell_surface_grid_mm).moved(Location((0,upper_delta,0)))]
    lower_tools=[exterior_cutter(asset,1,-1,(-m.lower_jaw_width/2-1,m.lower_jaw_width/2+1,i.lower_pad_z_min,i.lower_pad_z_max),scale,i.shell_surface_grid_mm).moved(Location((0,lower_delta,0)))]
    upper_cutters=[s.moved(Location((0,upper_delta,0))) for s in cutters]
    lower_cutters=[s.moved(Location((0,lower_delta,0))) for s in cutters]
    upper_clearance_cutters=[s.moved(Location((0,upper_delta,0))) for s in clearance_cutters]
    lower_clearance_cutters=[s.moved(Location((0,lower_delta,0))) for s in clearance_cutters]
    rear_blank=box(-m.rear_support_width/2,m.rear_support_width/2,-m.rear_support_height/2,m.rear_support_height/2,i.rear_block_back_z,4)
    proud=m.pad_proud_uncompressed-m.pad_compression
    back_rim=subtract_shifted(rear_blank,rear_tools,(0,0,-proud))
    pocket=box(-i.rear_pocket_width/2,i.rear_pocket_width/2,-i.rear_pocket_height/2,i.rear_pocket_height/2,-25,4)
    back_floor=subtract_shifted(rear_blank,rear_tools,(0,0,-(m.back_pad_thickness-m.pad_compression)))
    back=back_rim-(pocket-back_floor)
    contact_back=subtract_shifted(rear_blank,rear_tools,(0,0,0))
    back_pad=subtract_shifted((contact_back-back_floor)&pocket,cutters,(0,0,0))
    back-=back_pad
    back=subtract_shifted(back,clearance_cutters,(0,0,0))
    # Central load path, lower seat, fixed backplate and lower jaw.
    fixed_frame=box(-i.frame_outer_half_width,i.frame_outer_half_width,-25,75,-29,-23)-box(-i.frame_slot_half_width,i.frame_slot_half_width,i.frame_slot_min_y,76,-30,-22)
    cradle=back+fixed_frame+box(-i.slider_width/2,i.slider_width/2,i.lower_guide_seat_top_y-i.lower_seat_length,i.lower_guide_seat_top_y,i.guide_z-i.slider_depth/2+1,i.frame_front_z)
    lower_blank=box(-m.lower_jaw_width/2,m.lower_jaw_width/2,lower_y-7,lower_y+8,i.lower_pad_z_min,i.lower_pad_z_max)
    # Extend into the notch before subtracting the exterior: this supports
    # the complete contoured pad rather than leaving its middle floating.
    lower_rim=subtract_shifted(lower_blank,lower_tools,(0,-proud,0))
    lower_floor=subtract_shifted(lower_blank,lower_tools,(0,-(m.jaw_pad_thickness-m.pad_compression),0))
    lower_pocket=box(-m.lower_jaw_width/2+2,m.lower_jaw_width/2-2,lower_y-9,lower_y+4,i.lower_pad_z_min,i.lower_pad_z_max)
    lower=lower_rim-(lower_pocket-lower_floor)
    lower_depth_delta=m.bottom_center_depth-26.
    lower+=box(-m.lower_jaw_width/2,m.lower_jaw_width/2,lower_y-7,lower_y-3,-29,24+lower_depth_delta)
    lower+=box(-m.lower_jaw_width/2,m.lower_jaw_width/2,lower_y-7,lower_y+3,i.jaw_front_lip_z+lower_depth_delta,i.jaw_front_end_z+lower_depth_delta)
    if lower_y-3 < -31:
        lower+=box(-m.lower_jaw_width/2,m.lower_jaw_width/2,lower_y-7,-30,-29,-23)
    lower_pad_blank=box(-m.lower_jaw_width/2+2,m.lower_jaw_width/2-2,lower_y-5,lower_y+4,i.lower_pad_z_min,i.lower_pad_z_max)
    lower_pad=subtract_shifted(lower_pad_blank,lower_tools,(0,0,0))-subtract_shifted(lower_pad_blank,lower_tools,(0,-(m.jaw_pad_thickness-m.pad_compression),0))
    lower_pad=subtract_shifted(lower_pad,lower_cutters,(0,0,0))
    lower-=lower_pad
    lower=subtract_shifted(lower,lower_clearance_cutters,(0,0,0))
    cradle+=lower
    # Pivot hub and a stout web: no structure reaches the two grip backs.
    moving_start=i.pivot_fixed_face_x+i.tooth_height+t.tooth_axial_clearance
    hub=cyl(i.pivot_disc_radius,i.pivot_moving_x_max-moving_start,((moving_start+i.pivot_moving_x_max)/2,i.pivot_pad_y,i.pivot_pad_z),"x")
    for tooth in serrations(p,False):
        hub+=tooth
    hub-=cyl((h.pivot_bolt_d+t.clearance_hole)/2,32,(-3,i.pivot_pad_y,i.pivot_pad_z),"x")
    cradle+=hub+box(-14,8,-31,-19,i.pivot_pad_z,-23)
    # Pointer is just outside the moving disk; it reads the fixed outer ring.
    cradle+=box(moving_start, moving_start+2,i.pivot_pad_y+25,i.pivot_pad_y+32,i.pivot_pad_z-.4,i.pivot_pad_z+.4)
    for x in (-i.guide_x,i.guide_x):
        cradle-=cyl((h.guide_d+t.guide_socket_clearance)/2,6.2,(x,i.lower_guide_seat_top_y-3,i.guide_z),"y")
    # Cap screws enter from rear into these two heat-set inserts.
    for x in (-i.cap_bolt_x,i.cap_bolt_x):
        cradle-=cyl(h.cap_insert_pilot/2,h.cap_insert_length+.3,(x,71,-27),"z")
    # Preserve real clearance at the sharp center-notch facets where the
    # sampled outer surface otherwise crosses the asset by a fraction of mm.
    # Trimming the sampled notch against actual CoACD facets leaves one
    # disconnected sub-millimetre chip (0.238 mm3 in this asset). It has no
    # load path and cannot be printed as part of the jaw; remove that chip.
    pieces=sorted(cradle.solids(),key=lambda s:s.volume,reverse=True)
    if len(pieces)>1 and sum(s.volume for s in pieces[1:])<1.0:
        cradle=pieces[0]
    add("cradle_lower_jaw",cradle,rotation=(0,90,0),note="Actual asset shell subtraction and sampled exterior pocket, narrow central backrest, integrated lower jaw. Hirth face up; support below webs.")

    upper_y=m.top_center_y
    upper_blank=box(-m.upper_jaw_width/2,m.upper_jaw_width/2,upper_y-8,upper_y+6,i.top_pad_z_min,i.top_pad_z_max)
    upper_rim=subtract_shifted(upper_blank,upper_tools,(0,proud,0))
    upper_floor=subtract_shifted(upper_blank,upper_tools,(0,m.jaw_pad_thickness-m.pad_compression,0))
    upper_pocket=box(-m.upper_jaw_width/2+2,m.upper_jaw_width/2-2,upper_y-2,upper_y+8,i.top_pad_z_min,i.top_pad_z_max)
    upper=upper_rim-(upper_pocket-upper_floor)
    upper_depth_delta=m.top_center_depth-28.
    upper+=box(-m.upper_jaw_width/2,m.upper_jaw_width/2,upper_y+1,upper_y+6,-30,i.jaw_front_end_z+upper_depth_delta)
    upper+=box(-m.upper_jaw_width/2,m.upper_jaw_width/2,m.top_lip_y_min,upper_y+6,i.jaw_front_lip_z+upper_depth_delta,i.jaw_front_end_z+upper_depth_delta)
    upper_pad_blank=box(-m.upper_jaw_width/2+2,m.upper_jaw_width/2-2,upper_y-3,upper_y+5,i.top_pad_z_min,i.top_pad_z_max)
    upper_pad=subtract_shifted(upper_pad_blank,upper_tools,(0,0,0))-subtract_shifted(upper_pad_blank,upper_tools,(0,m.jaw_pad_thickness-m.pad_compression,0))
    upper_pad=subtract_shifted(upper_pad,upper_cutters,(0,0,0))
    upper-=upper_pad
    upper=subtract_shifted(upper,upper_clearance_cutters,(0,0,0))
    sy0=i.slider_center_y-i.slider_length/2;sy1=i.slider_center_y+i.slider_length/2
    sz0=i.guide_z-i.slider_depth/2;sz1=i.guide_z+i.slider_depth/2
    slider=box(-i.slider_width/2,i.slider_width/2,sy0,sy1,sz0,sz1)+box(-m.upper_jaw_width/2,m.upper_jaw_width/2,sy1-i.upper_spine_overlap,upper_y+5,sz1-2,i.frame_front_z)+upper
    for x in (-i.guide_x,i.guide_x):
        slider-=cyl((h.guide_d+t.guide_diametral_clearance)/2,44,(x,12,i.guide_z),"y")
    slider-=cyl(h.lock_insert_pilot/2,h.lock_insert_length+.2,(0,i.lock_center_y,-47),"z")
    slider-=cyl((h.lock_bolt_d+t.clearance_hole)/2,24,(0,i.lock_center_y,-39),"z")
    # Integral travel stops: shoulder seat prevents 64 mm undersize; separate
    # cap limits extension before the spring approaches solid height.
    add("slider_upper_jaw",slider,rotation=(-90,0,0),note="Slide on two printed rods. M4x25 locking screw + 3 mm printed spacer + 1 mm washer; insert from rear, screw bears on fixed backplate.")
    spacer=cyl(5,3,(0,i.lock_center_y,-51.5))-cyl(2.25,4,(0,i.lock_center_y,-51.5))
    add("lock_screw_spacer",spacer)
    cap=box(-i.cap_outer_half_width,i.cap_outer_half_width,62,75,-49,-29)-box(-11,11,61,76,-36,-28)
    for x in (-i.guide_x,i.guide_x):
        stop_y=i.slider_spring_seat_y+(i.slider_max_span-m.center_span)
        sleeve=cyl(i.spring_stop_outer_d/2,i.cap_spring_seat_y-stop_y+.2,(x,(stop_y+i.cap_spring_seat_y)/2+.1,i.guide_z),"y")
        cap+=sleeve
        cap-=cyl((h.guide_d+t.guide_socket_clearance)/2,i.guide_max_y-stop_y+.2,(x,(stop_y+i.guide_max_y)/2,i.guide_z),"y")
    for x in (-i.cap_bolt_x,i.cap_bolt_x):
        cap-=cyl((h.cap_bolt_d+t.clearance_hole)/2,24,(x,71,-39),"z")
        cap-=cyl(3.2,3.2,(x,71,-47.5),"z")
    add("guide_cap",cap,rotation=(-90,0,0),note="Remove to fit rails, slider and springs; 2x M3x22 screws into cradle inserts. Broad end down, sleeves point upward.")
    for n,x in enumerate((-i.guide_x,i.guide_x),1):
        rail=cyl(h.guide_d/2,i.guide_max_y-i.guide_min_y,(x,(i.guide_max_y+i.guide_min_y)/2,i.guide_z),"y")
        add(f"guide_rod_{n}",rail,rotation=(-90,0,0),note="Print vertical with brim; sand lightly, do not lubricate contact pads.")
    # Hand knob traps an ordinary M6 nut; axial bolt and washer are hardware.
    knob_x=i.pivot_moving_x_max+h.pivot_washer_thickness+i.knob_thickness/2
    knob=cyl(i.knob_radius,i.knob_thickness,(knob_x,i.pivot_pad_y,i.pivot_pad_z),"x")
    for k in range(8):
        a=k*math.pi/4
        knob-=cyl(5,18,(knob_x,i.pivot_pad_y+22*math.cos(a),i.pivot_pad_z+22*math.sin(a)),"x")
    knob-=cyl((h.pivot_bolt_d+t.clearance_hole)/2,18,(knob_x,i.pivot_pad_y,i.pivot_pad_z),"x")
    knob-=hex_prism(h.pivot_nut_af+t.nut_af_clearance,h.pivot_nut_height+2.0,(22,i.pivot_pad_y,i.pivot_pad_z),"x")
    add("M6_hand_knob",knob,rotation=(0,-90,0),note="M6 nut pocket opens outward; washer between knob and hub. Nut is not a heat-set insert.")

    # Standard hardware is represented separately (threads intentionally omitted).
    bolt_start=i.column_x_min-h.pivot_washer_thickness
    bolt=cyl(h.pivot_bolt_d/2,h.pivot_bolt_length,(bolt_start+h.pivot_bolt_length/2,i.pivot_pad_y,i.pivot_pad_z),"x")+cyl(5,6,(bolt_start-3,i.pivot_pad_y,i.pivot_pad_z),"x")
    add("hardware_M6x60_axis_bolt",bolt,"Steel")
    for n,xx in enumerate((i.column_x_min-h.pivot_washer_thickness/2,i.pivot_moving_x_max+h.pivot_washer_thickness/2),1):
        washer=cyl(h.pivot_washer_od/2,h.pivot_washer_thickness,(xx,i.pivot_pad_y,i.pivot_pad_z),"x")-cyl(3.2,3,(xx,i.pivot_pad_y,i.pivot_pad_z),"x")
        add(f"hardware_M6_washer_{n}",washer,"Steel")
    nut=hex_prism(h.pivot_nut_af,h.pivot_nut_height,(21,i.pivot_pad_y,i.pivot_pad_z),"x")-cyl(3.05,8,(21,i.pivot_pad_y,i.pivot_pad_z),"x")
    add("hardware_M6_nut",nut,"Steel")
    for n,x in enumerate((-i.guide_x,i.guide_x),1):
        spring_length=i.cap_spring_seat_y-i.slider_spring_seat_y-h.spring_wire
        rad=(h.spring_od-h.spring_wire)/2
        pitch=spring_length/h.spring_active_coils
        path=Helix(pitch=pitch,height=spring_length,radius=rad)
        profile=Plane(origin=(rad,0,0),z_dir=(0,rad*2*math.pi,pitch))*Circle(h.spring_wire/2)
        spring=sweep(profile,path=path).moved(Location((x,i.slider_spring_seat_y+h.spring_wire/2,i.guide_z),(-90,0,0)))
        add(f"hardware_compression_spring_{n}",spring,"Spring_steel",note="Nominal installed coil envelope; exact vendor end turns are not modelled.")

    # Sheet pads are separate thin geometries at their compressed assembly
    # thickness. TPU is optional and exported at free thickness separately.
    padmat="TPU95A" if m.use_printed_tpu_pads else m.pad_material
    add("pad_back",back_pad,padmat,note="2 mm stock, 1.5 mm recess, assumed 0.25 mm compression.")
    add("pad_lower_jaw",lower_pad,padmat)
    add("pad_upper_jaw",upper_pad,padmat)
    bottom=box(*pad_outline,0,m.base_pad_thickness-m.base_pad_compression)
    for x in (-i.bolt_x,i.bolt_x):
        for y in i.bolt_y:
            bottom-=cyl(6,2,(x,y,.5))
    add("pad_base_bottom",bottom,padmat,True,rotation=(angle,0,0))
    zone=box(-i.base_width/2,i.base_width/2,-i.clamp_band_depth,0,base_float,flange_z)
    modifiers["base_clamp_band"]=zone.moved(pad_from_F)
    for n,side in enumerate((-1,1),1):
        x=side*i.clamp_center_x; y=i.clamp_center_y
        sx,sy=i.clamp_zone_width/2,i.clamp_zone_depth/2
        # Both shoe faces have a shallow rectangular recess. Pad proud .5 mm
        # uncompressed; no circular socket which would concentrate load.
        bottomface=flange_z+(m.pad_proud_uncompressed-m.pad_compression)
        topface=bottomface+i.pressure_shoe_thickness
        shoe=box(x-sx,x+sx,y-sy,y+sy,bottomface,topface)
        shoe_recess=m.shoe_pad_thickness-m.pad_proud_uncompressed
        shoe-=box(x-sx+1,x+sx-1,y-sy+1,y+sy-1,bottomface-.1,bottomface+shoe_recess)
        shoe-=box(x-sx+1,x+sx-1,y-sy+1,y+sy-1,topface-shoe_recess,topface+.1)
        add(f"pressure_shoe_{n}",shoe,fixed=True,rotation=(angle+90,0,0),note="Print on narrow edge, entire shoe 100% infill, >=6 walls, >=8 top/bottom; both faces carry 1 mm sheet pads.")
        for face,z in (("lower",flange_z),("upper",topface-shoe_recess)):
            pad=box(x-sx+1,x+sx-1,y-sy+1,y+sy-1,z,z+m.shoe_pad_thickness-m.pad_compression)
            add(f"pad_shoe_{n}_{face}",pad,padmat,True,rotation=(angle,0,0))
        washer_z=topface+m.pad_proud_uncompressed-m.pad_compression+h.clamp_spreader_thickness/2
        washer=cyl(h.clamp_spreader_od/2,h.clamp_spreader_thickness,(x,y,washer_z))-cyl(h.clamp_spreader_id/2,h.clamp_spreader_thickness+1,(x,y,washer_z))
        add(f"hardware_M8_wide_spreader_{n}",washer,"Steel",True,note="DIN 9021 M8, 24 OD x 8.4 ID x 2.0; C-clamp foot bears on this standard washer above the sheet pad.")

    # Mechanical target reconstruction uses the manufactured pivot datum and
    # the key angle, not copying root_pose as the measured result.
    actual_R_F=Rotation.from_euler("x",angle,degrees=True).as_matrix()
    actual_R_world=world_F@actual_R_F
    actual_root=datum+world_F@(pivot_F-actual_R_F@np.array([0,i.pivot_pad_y,i.pivot_pad_z]))
    world_from_pad=np.eye(4);world_from_pad[:3,:3]=actual_R_world;world_from_pad[:3,3]=actual_root
    return parts,modifiers,{"design_revision":i.layout_revision,"modifier_expected_volumes_mm3":{"base_clamp_band":i.base_width*i.clamp_band_depth*base_t-(i.base_width-8)*(i.clamp_band_depth-4)*(m.base_pad_thickness-m.pad_proud_uncompressed)},"parameters":dataclasses.asdict(p),"parts":list(meta.values()),
        "world_from_pad":world_from_pad.tolist(),"target_root_pose":pose,
        "fixed_frame_world_axes":world_F.tolist(),"fixed_frame_world_origin_mm":datum.tolist(),
        "pivot_base_mm":pivot_F.tolist(),"pivot_pad_mm":[0,i.pivot_pad_y,i.pivot_pad_z],
        "key_angle_deg":angle,"mark_label_deg":i.major_mark_deg,"clocking_offset_deg":angle-i.major_mark_deg,
        "base_bottom_rigid_plane_above_table_mm":base_float,
        "nominal_span_mm":m.center_span,"jaw_span_range_mm":[i.slider_min_span,i.slider_max_span],
        "mechanics":mechanics(p),"source_scene":str(SCENE.relative_to(ROOT)),
        "assembly_frame":"controller local, mm; world_from_pad includes mm translation"}


def flat_pad(name,p):
    """Flexible TPU prints flat at free thickness, then conforms on assembly."""
    m,i=p.measured,p.installation
    if name=="pad_base_bottom":
        w,d,t=i.base_width-8,i.base_depth-8,m.base_pad_thickness
    elif name=="pad_back":
        w,d,t=i.rear_pocket_width,i.rear_pocket_height,m.back_pad_thickness
    elif name=="pad_upper_jaw":
        w,d,t=m.upper_jaw_width-4,i.top_pad_z_max-i.top_pad_z_min,m.jaw_pad_thickness
    elif name=="pad_lower_jaw":
        w,d,t=m.lower_jaw_width-4,i.lower_pad_z_max-i.lower_pad_z_min,m.jaw_pad_thickness
    else:
        w,d,t=i.clamp_zone_width-2,i.clamp_zone_depth-2,m.shoe_pad_thickness
    shape=box(-w/2,w/2,-d/2,d/2,0,t)
    if name=="pad_base_bottom":
        for x in (-i.bolt_x,i.bolt_x):
            for y in i.bolt_y:
                shape-=cyl(6,t+2,(x,y+i.base_depth/2,t/2))
    return shape


def export_simulation(out,manifest):
    """glTF uses metres. STEP/STL and world_from_pad retain CAD millimetres."""
    from trimesh.visual.material import PBRMaterial
    from trimesh.visual.texture import TextureVisuals
    sim=trimesh.Scene()
    for part in manifest["parts"]:
        name,material=part["name"],part["material"]
        mesh=trimesh.load(out/"assembly_mesh"/(name+".stl"),force="mesh")
        mesh.apply_scale(.001);mesh.units="m"
        color="#287d94" if material=="PETG" else "#937ca0" if name.startswith("pad_") else "#adb9c4"
        mesh.visual=TextureVisuals(material=PBRMaterial(name=material,baseColorFactor=trimesh.visual.color.hex_to_rgba(color),roughnessFactor=.7))
        mesh.metadata.update({"part_name":name,"physical_material":material,"frame":"controller_local"})
        sim.add_geometry(mesh,node_name=name,geom_name=name)
    sim.metadata.update({"units":"m","frame":"controller_local","source_geometry_units":"mm"})
    sim.export(out/"simulation/fixture.glb")
    manifest["simulation_materials"]={a["name"]:a["material"] for a in manifest["parts"]}
    manifest["simulation_units"]="m"
    transform=np.array(manifest["world_from_pad"]);transform[:3,3]/=1000
    manifest["world_from_simulation_m"]=transform.tolist()


def export_clamp_envelopes(p,out,manifest):
    """Parametric procurement/hand keepouts, separate from physical CAD solids."""
    i,m=p.installation,p.measured
    out=Path(out);folder=out/"envelopes";folder.mkdir(exist_ok=True)
    Fworld=np.eye(4);Fworld[:3,:3]=np.asarray(manifest["fixed_frame_world_axes"])
    Fworld[:3,3]=manifest["fixed_frame_world_origin_mm"]
    pad_from_F=np.linalg.inv(np.asarray(manifest["world_from_pad"]))@Fworld
    top=manifest["base_bottom_rigid_plane_above_table_mm"]+(i.aluminum_pad_tray_thickness+i.aluminum_thickness if i.aluminum_base else i.printed_base_thickness)
    shoe_contact=top+i.pressure_shoe_thickness+2*(m.pad_proud_uncompressed-m.pad_compression)+p.hardware.clamp_spreader_thickness
    lower_bottom=-i.c_table_stack_thickness-i.c_lower_arm_drop_max
    lower_top=-i.c_table_stack_thickness-i.c_lower_arm_drop_min
    handle_z=-i.c_table_stack_thickness-i.c_handle_below_table_bottom
    entries=[]
    def block(lo,hi):
        lo,hi=np.asarray(lo),np.asarray(hi)
        return trimesh.creation.box(hi-lo,transform=trimesh.transformations.translation_matrix((lo+hi)/2))
    def add(index,kind,shape,role):
        bounds=shape.bounds.copy();shape.apply_transform(pad_from_F)
        name=f"c{index}_{kind}";file=f"envelopes/{name}.stl";shape.export(out/file)
        entries.append({"name":name,"file":file,"clamp_index":index,"kind":kind,"role":role,
                        "base_frame_bounds_mm":bounds.tolist(),"frame":"controller_local_mm",
                        "expected_fixture_contacts":[f"pressure_shoe_{index}",f"pad_shoe_{index}_upper",f"hardware_M8_wide_spreader_{index}"] if kind=="upper_arm" else [],
                        "operator_intended_contact":kind in ("spindle","handle_rotation")})
    for n,sign in enumerate((-1,1),1):
        x=sign*i.clamp_center_x;y=i.clamp_center_y;hw=i.c_arm_width/2
        add(n,"upper_arm",block([x-hw,y,shoe_contact],[x+hw,i.c_spine_y,top+i.c_upper_arm_height]),"hardware")
        add(n,"outer_spine",block([x-hw,i.c_spine_y-i.c_spine_depth/2,lower_bottom],[x+hw,i.c_spine_y+i.c_spine_depth/2,top+i.c_upper_arm_height]),"hardware")
        add(n,"lower_arm",block([x-hw,y,lower_bottom],[x+hw,i.c_spine_y,lower_top]),"hardware")
        spindle=trimesh.creation.cylinder(radius=i.c_spindle_d/2,height=-i.c_table_stack_thickness-handle_z,sections=32)
        spindle.apply_translation([x,y,(handle_z-i.c_table_stack_thickness)/2]);add(n,"spindle",spindle,"hardware")
        for kind,radius,height,role in (("handle_rotation",i.c_handle_sweep_radius,i.c_handle_sweep_height,"hardware"),
                                         ("operator_hand_rotation",i.c_handle_sweep_radius+i.c_operator_extra_radius,i.c_operator_hand_height,"operator_access")):
            mesh=trimesh.creation.cylinder(radius=radius,height=height,sections=64);mesh.apply_translation([x,y,handle_z]);add(n,kind,mesh,role)
        add(n,"operator_hand_approach",block([x-i.c_operator_extra_radius,y,handle_z-i.c_operator_hand_height/2],[x+i.c_operator_extra_radius,i.c_operator_approach_y,handle_z+i.c_operator_hand_height/2]),"operator_access")
    manifest["clamp_envelopes"]=entries
    manifest["clamp_mounting"]={"orientation":"Fixed upper C-jaw above table; spindle and R50 tommy-bar below table. Adjust clamps sequentially.",
                               "provisional_table_stack_mm":i.c_table_stack_thickness,"required_throat_depth_mm":i.c_spine_y-i.clamp_center_y,
                               "handle_center_base_z_mm":handle_z,"upper_arm_top_base_z_mm":top+i.c_upper_arm_height,
                               "upper_fixed_contact_base_z_mm":shoe_contact,
                               "scope":"Parametric conservative envelope, not a measured vendor clamp. Confirm all dimensions before purchasing/installing."}


def export_all(p, out):
    asset=ControllerAsset()
    parts,modifiers,manifest=build(p,asset)
    out.mkdir(parents=True,exist_ok=True)
    for d in ("print","assembly_mesh","modifiers","simulation","reference"):
        (out/d).mkdir(exist_ok=True)
    # Switching variants in one output directory must not leave old solids
    # for the checker/renderer to mistake for extra assembled components.
    printed={a["name"] for a in manifest["parts"] if a["is_printed"]}
    for d,valid in (("assembly_mesh",set(parts)),("print",printed),("modifiers",set(modifiers) if not p.installation.aluminum_base else set())):
        for old in (out/d).iterdir():
            if old.suffix in {".stl",".step"} and old.stem not in valid:
                old.unlink()
    print("Exporting solids ...",flush=True)
    metas={a["name"]:a for a in manifest["parts"]}
    children=[]
    for name,shape in parts.items():
        print(name,round(shape.volume,1),len(shape.solids()),flush=True)
        if not shape.is_valid or len(shape.solids())!=1:
            raise RuntimeError(f"{name}: expected one valid solid, got {len(shape.solids())}")
        color="#207f9c" if metas[name]["material"]=="PETG" else "#d79343"
        shape.color=Color(color)
        children.append(shape)
        spring="compression_spring" in name
        export_stl(shape,out/"assembly_mesh"/(name+".stl"),
                   tolerance=.20 if spring else p.printing.mesh_tolerance,
                   angular_tolerance=.4 if spring else .15)
        if name=="base_aluminum":
            plate=shape.moved(Location((0,0,0),(manifest["key_angle_deg"],0,0)))
            bb=plate.bounding_box()
            plate=plate.moved(Location((-bb.min.X,-bb.min.Y,-bb.min.Z)))
            export_step(plate,out/"aluminum_plate.step")
        if metas[name]["is_printed"]:
            rot=metas[name]["print_rotation_deg"]
            flat_tpu=p.measured.use_printed_tpu_pads and name.startswith("pad_")
            to_print=flat_pad(name,p) if flat_tpu else shape
            if flat_tpu:
                rot=(0,0,0)
                metas[name]["print_shape_state"]="Flat free-thickness flexible sheet; assembled shape is conformed and compressed. No rigid transform relates these states."
            oriented=to_print.moved(Location((0,0,0),rot))
            b=oriented.bounding_box(); tr=(-b.min.X,-b.min.Y,-b.min.Z)
            oriented=oriented.moved(Location(tr))
            export_step(oriented,out/"print"/(name+".step"))
            export_stl(oriented,out/"print"/(name+".stl"),tolerance=p.printing.tooth_mesh_tolerance if "cradle" in name or name=="column" else p.printing.mesh_tolerance)
            metas[name]["print_bounds_mm"]=list(oriented.bounding_box().size)
            tm=np.eye(4);tm[:3,:3]=Rotation.from_euler("xyz",rot,degrees=True).as_matrix();tm[:3,3]=tr
            metas[name]["print_transform"]=tm.tolist()
        metas[name]["solid_volume_mm3"]=shape.volume
        metas[name]["valid_solid"]=shape.is_valid
        metas[name]["solid_count"]=len(shape.solids())
    assembly=Compound(label="Xbox_1708_fixture",children=children)
    export_step(assembly,out/"assembly.step")
    # Modifier coordinates exactly match the BASE's recommended print file.
    base_meta=metas["base_aluminum" if p.installation.aluminum_base else "base_printed"]
    if base_meta["is_printed"]:
        rot=base_meta["print_rotation_deg"];tm=np.array(base_meta["print_transform"])
        for name,s in modifiers.items():
            mod=s.moved(Location((0,0,0),rot)).moved(Location(tuple(tm[:3,3])))
            export_stl(mod,out/"modifiers"/(name+".stl"),tolerance=.1)
    # Simplified independent, material-labelled visual/contact meshes; never
    # take one convex hull across the two free grip-back spaces.
    export_simulation(out,manifest)
    export_clamp_envelopes(p,out,manifest)
    refs=[]
    for source in [asset.urdf]+[g[-1] for g in asset.geometry]:
        rel=source.relative_to(asset.urdf.parent)
        dest=out/"reference"/rel;dest.parent.mkdir(parents=True,exist_ok=True);shutil.copy2(source,dest)
        refs.append({"path":str(rel),"sha256":hashlib.sha256(source.read_bytes()).hexdigest(),"bytes":source.stat().st_size})
    manifest["reference_asset"]={"upstream":"xbox-gamepad-physics 0.4.1 working-tree build export","source":str(asset.urdf),"files":refs}
    neutral=trimesh.util.concatenate(list(asset.meshes().values()))
    neutral.apply_scale([p.measured.width/153.,p.measured.height/104.6,p.measured.depth/50.2])
    neutral.export(out/"reference/controller_neutral_mm.stl")
    (out/"manifest.json").write_text(json.dumps(manifest,indent=2)+"\n")
    (out/"asset_measurements.json").write_text(json.dumps(asset.summary(),indent=2)+"\n")
    templates(out,p)
    # Large exports live under .local; keep a convenient relative symlink in
    # the requested output tree. Source and small generated files stay put.
    large_root=ROOT/".local/gamepad_fixture_large"/out.name
    for path in list(out.rglob("*")):
        if path.is_file() and not path.is_symlink() and path.stat().st_size>5_000_000:
            dest=large_root/path.relative_to(out);dest.parent.mkdir(parents=True,exist_ok=True)
            shutil.move(str(path),str(dest))
            import os
            path.symlink_to(os.path.relpath(dest,path.parent))
    return manifest


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
    config = {"revision":"marker-card-v1", "board":{
        "dictionary":"DICT_4X4_100", "squares_x":card.squares_x,"squares_y":card.squares_y,
        "square_m":card.square_mm/1000,"marker_m":card.marker_mm/1000,"first_id":card.first_id,
        "legacy_pattern":False,"border_m":card.border_mm/1000},
        "gamepad_from_board":transform.tolist(),"units":"metres; board origin pattern top-left, x right y down z into backing",
        "card_parameters":dataclasses.asdict(card),
        "mount":{"parent":"moving cradle via existing guide cap", "existing_body_changed":False,
            "interface":"46.2 mm keyed slot around guide-cap sides + two existing M3 screw axes",
            "bolt_x_mm":[-i.cap_bolt_x,i.cap_bolt_x],"bolt_y_mm":i.frame_top_y-4,
            "hardware":"replace 2 M3x22 by M3x30 + nominal 2.5 mm metal spacers; verify 3-4 mm engagement/no bottoming",
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
    anchor = box(-i.cap_outer_half_width-2,i.cap_outer_half_width+2,62,75,-53,-49)
    key_gap = i.cap_outer_half_width+card.slot_clearance/2
    for sign in (-1,1):
        anchor += box(sign*key_gap if sign>0 else -i.cap_outer_half_width-2,
                      i.cap_outer_half_width+2 if sign>0 else -key_gap,62,75,-49.1,-43)
        anchor -= cyl((h.cap_bolt_d+t.clearance_hole)/2,8,(sign*i.cap_bolt_x,71,-51),"z")
    # Short sloping rib reaches the lower edge of the tilted card; it stays
    # above the pad controls, rather than passing in front of the upper jaw.
    start=np.array([72.,-51.]);end=np.array([110.,-63.])
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
    (out/"manifest.json").write_text(json.dumps(config,indent=2)+"\n")
    print(f"Marker card accessory, transform and 1:1 templates: {out}")


def export_coupons(p, out):
    """Cut test pieces from the unchanged production solids; never rewrite out/."""
    out = Path(out)
    out.mkdir(parents=True, exist_ok=True)
    parts, _, manifest = build(p)
    m, i, h = p.measured, p.installation, p.hardware
    meta = {v["name"]: v for v in manifest["parts"]}
    coupons = []

    def add(name, shape, source, rotation=None, purpose=""):
        rotation = rotation if rotation is not None else meta[source]["print_rotation_deg"]
        oriented = shape.moved(Location((0, 0, 0), tuple(rotation)))
        bounds = oriented.bounding_box()
        oriented = oriented.moved(Location((-bounds.min.X, -bounds.min.Y, -bounds.min.Z)))
        if oriented.volume <= 0 or not oriented.is_valid:
            raise ValueError(f"Invalid coupon: {name}")
        export_stl(oriented, out/(name+".stl"), tolerance=p.printing.tooth_mesh_tolerance)
        # Weld sub-micron STL seam duplicates from OCCT tessellation. This is
        # below 1e-6 mm and does not change the CAD design or tooth geometry.
        mesh=trimesh.load(out/(name+".stl"),force="mesh")
        mesh.merge_vertices(digits_vertex=6)
        if not mesh.is_watertight:
            raise ValueError(f"Non-closed coupon STL: {name}")
        mesh.export(out/(name+".stl"))
        b = oriented.bounding_box()
        coupons.append({"name": name, "source_part": source, "purpose": purpose,
                        "volume_mm3": oriented.volume, "solid_count": len(oriented.solids()),
                        "bounds_mm": [b.size.X, b.size.Y, b.size.Z],
                        "print_rotation_deg": list(rotation),
                        "estimated_minutes_range": [round(oriented.volume/60/4+5), round(oriented.volume/60/2+10)]})

    # Central XZ/YZ slices retain the real shell subtraction and pad pockets.
    upper_section = parts["slider_upper_jaw"] & box(-3,3,m.top_center_y-9,m.top_center_y+9,
        i.frame_back_z-2,i.jaw_front_end_z+4)
    # The section plane detaches a 5.53 mm3 shell-trim flake. It has no load
    # path in this coupon; keep the connected jaw, without modifying the part.
    upper_section = max(upper_section.solids(), key=lambda solid: solid.volume)
    add("upper_jaw_section", upper_section, "slider_upper_jaw",
        purpose="Upper contact profile, lip and sheet recess; detached section flake omitted")
    add("lower_jaw_section", parts["cradle_lower_jaw"] & box(-3,3,m.top_center_y-m.center_span-12,
        m.top_center_y-m.center_span+9,i.frame_back_z-1,i.jaw_front_end_z+4),
        "cradle_lower_jaw", purpose="Lower notch contact and front lip")
    add("backrest_section", parts["cradle_lower_jaw"] & box(-3,3,-m.rear_support_height/2+1,
        m.rear_support_height/2-1,i.frame_back_z-1,5),
        "cradle_lower_jaw", purpose="Back-shell profile and sheet recess")
    # Full mating rings preserve all 36 teeth and the original axial clearance.
    fixed = parts["column"] & cyl(i.fixed_disc_radius, 8,
              (i.pivot_fixed_face_x-2, i.pivot_pad_y, i.pivot_pad_z), "x")
    moving = parts["cradle_lower_jaw"] & cyl(i.pivot_disc_radius, 8,
              (i.pivot_fixed_face_x+4, i.pivot_pad_y, i.pivot_pad_z), "x")
    add("hirth_fixed", fixed, "column", (0,-90,0), "Full fixed toothed face, upward")
    add("hirth_moving", moving, "cradle_lower_jaw", (0,90,0), "Full mating toothed face, upward")
    add("guide_rod_segment", parts["guide_rod_1"] & box(-i.guide_x-h.guide_d,-i.guide_x+h.guide_d,
        i.slider_center_y-15,i.slider_center_y+15,i.guide_z-h.guide_d,i.guide_z+h.guide_d),
        "guide_rod_1", purpose="30 mm of the same vertically printed 6 mm guide")
    add("slider_segment", parts["slider_upper_jaw"] & box(-i.slider_width/2,i.slider_width/2,
        i.lock_center_y-5,i.lock_center_y+5,i.guide_z-i.slider_depth/2-1,i.guide_z+i.slider_depth/2+1),
        "slider_upper_jaw", purpose="Both guide bores and actual diametral clearance")
    add("insert_M3", parts["cradle_lower_jaw"] & box(i.cap_bolt_x-5,i.cap_bolt_x+5,
        i.frame_top_y-10,i.frame_top_y,i.frame_back_z-1,i.frame_front_z+1),
        "cradle_lower_jaw", purpose="Actual M3 heat-set pilot and wall")
    add("insert_M4", parts["slider_upper_jaw"] & box(-8,8,i.lock_center_y-8,i.lock_center_y+8,
        i.guide_z-i.slider_depth/2,i.guide_z+h.lock_insert_length),
        "slider_upper_jaw", purpose="Actual M4 pilot and lock screw through-hole")
    # Transform the actual base back to fixture F coordinates for a local cut.
    world_pad = np.asarray(manifest["world_from_pad"])
    world_F = np.asarray(manifest["fixed_frame_world_axes"])
    R = world_F.T @ world_pad[:3,:3]
    origin = world_F.T @ (world_pad[:3,3]-manifest["fixed_frame_world_origin_mm"])
    base_name = "base_aluminum" if i.aluminum_base else "base_printed"
    base_F = parts[base_name].moved(Location(tuple(origin), tuple(Rotation.from_matrix(R).as_euler("xyz", degrees=True))))
    add("clamp_band_block", base_F & box(i.clamp_center_x-i.clamp_zone_width/2,i.clamp_center_x+i.clamp_zone_width/2,
        i.clamp_center_y-i.clamp_zone_depth/2,i.clamp_center_y+i.clamp_zone_depth/2,-1,
        max(i.printed_base_thickness,i.aluminum_thickness)+10), base_name, (90,0,0),
        "Actual clamp band with underside recess; aluminium variant is machining reference only")
    add("pressure_shoe", parts["pressure_shoe_1"], "pressure_shoe_1", purpose="Full pressure shoe, two sheet recesses")
    (out/"coupons.json").write_text(json.dumps({"design_revision": i.layout_revision,
        "source": "Intersections of build(Parameters()) production solids; no structural changes",
        "time_estimate": "PETG 0.16 mm, 6 walls, solid; net 2-4 mm3/s plus per-part overhead, NOT slicer timing",
        "coupons": coupons}, indent=2)+"\n")
    print(f"Exported {len(coupons)} coupons to {out}")


def templates(out,p):
    """A4 SVGs use explicit mm units; print actual size, no fit-to-page."""
    header='<svg xmlns="http://www.w3.org/2000/svg" width="210mm" height="297mm" viewBox="0 0 210 297"><rect width="210" height="297" fill="white"/><g fill="none" stroke="#222" stroke-width="0.25">'
    text=['</g><g fill="#111" font-family="sans-serif" font-size="3">']
    shapes=[]
    m,i=p.measured,p.installation
    pads=[(f"Upper jaw: {m.jaw_pad_thickness:g} mm sheet",m.upper_jaw_width-4,i.top_pad_z_max-i.top_pad_z_min,15,30),
          (f"Lower jaw: {m.jaw_pad_thickness:g} mm sheet",m.lower_jaw_width-4,i.lower_pad_z_max-i.lower_pad_z_min,65,30),
          (f"Backrest: {m.back_pad_thickness:g} mm sheet",i.rear_pocket_width,i.rear_pocket_height,120,30),
          (f"Pressure shoe: {m.shoe_pad_thickness:g} mm sheet, cut 4",i.clamp_zone_width-2,i.clamp_zone_depth-2,15,90)]
    for name,w,h,x,y in pads:
        shapes.append(f'<rect x="{x}" y="{y}" width="{w}" height="{h}" rx="1"/>')
        text.append(f'<text x="{x}" y="{y-4}">{name}</text>')
    shapes.append('<path d="M15 160h100m-100 -2v4m100 -4v4"/>')
    text.append('<text x="15" y="168">Calibration bar must measure exactly 100 mm.</text><text x="15" y="179">Print A4 / 100% / actual size. Do not fit to page.</text><text x="15" y="190">Templates show planar cut outlines; press flexible sheet into curved pockets.</text>')
    (out/"pad_templates_A4.svg").write_text(header+''.join(shapes)+''.join(text)+'</g></svg>')
    # A4 landscape accommodates the entire underside without tiled printing.
    w,d=i.base_width-8,i.base_depth-8;x0=(297-w)/2;y0=26
    a=[f'<rect x="{x0}" y="{y0}" width="{w}" height="{d}"/>']
    for x in (-i.bolt_x,i.bolt_x):
        for y in i.bolt_y:
            a.append(f'<circle cx="{148.5+x}" cy="{y0+y+i.base_depth-4}" r="6"/>')
    a.append('<path d="M15 195h100m-100 -2v4m100 -4v4"/>')
    header2=header.replace('width="210mm" height="297mm" viewBox="0 0 210 297"','width="297mm" height="210mm" viewBox="0 0 297 210"').replace('<rect width="210" height="297"','<rect width="297" height="210"')
    (out/"pad_template_base_A4.svg").write_text(header2+''.join(a)+f'</g><g fill="#111" font-family="sans-serif" font-size="3"><text x="15" y="17">Base underside: {w:g} x {d:g} mm, stock thickness &lt;= 1 mm</text><text x="15" y="205">100 mm calibration bar; A4 landscape / actual size / 100%</text></g></svg>')


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--out",type=Path,default=HERE/"out")
    parser.add_argument("--marker-card",action="store_true",help="export removable marker accessory only")
    parser.add_argument("--coupons",action="store_true",help="export test pieces only; default destination out_coupons")
    parser.add_argument("--aluminum",action="store_true")
    parser.add_argument("--tpu-pads",action="store_true")
    args=parser.parse_args()
    p=Parameters();p.installation.aluminum_base=args.aluminum;p.measured.use_printed_tpu_pads=args.tpu_pads
    if args.marker_card:
        export_marker_card(p,HERE/"out_marker" if args.out==HERE/"out" else args.out)
    elif args.coupons:
        destination = HERE/"out_coupons" if args.out == HERE/"out" else args.out
        export_coupons(p,destination)
    else:
        export_all(p,args.out)


if __name__=="__main__":
    main()
