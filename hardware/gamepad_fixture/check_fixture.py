#!/usr/bin/env python3
"""Recheck exported fixture solids and full URDF control travel (mm, N, Nm)."""
from __future__ import annotations
import argparse
import itertools
import json
import math
from pathlib import Path

import manifold3d as mf
import numpy as np
import trimesh
from scipy.spatial.transform import Rotation
from asset_geometry import ControllerAsset, SHELL_NAMES
from hand_geometry import check_fixture_hand_space

HERE = Path(__file__).resolve().parent


def manifold(mesh):
    solid=mf.Manifold(mf.Mesh(np.asarray(mesh.vertices, np.float32), np.asarray(mesh.faces, np.uint32)))
    if solid.status()!=mf.Error.NoError:
        raise RuntimeError(f"Invalid triangle manifold: {solid.status()}")
    return solid


def aabb_gap(a, b):
    return float(np.linalg.norm(np.maximum(np.maximum(a[0] - b[1], b[0] - a[1]), 0)))


def check_mounting_band_and_world_bounds(out,manifest,meshes,solids):
    install=manifest["parameters"]["installation"]
    world_from_pad=np.asarray(manifest["world_from_pad"])
    world_from_base=np.eye(4);world_from_base[:3,:3]=manifest["fixed_frame_world_axes"]
    world_from_base[:3,3]=manifest["fixed_frame_world_origin_mm"]
    pad_from_base=np.linalg.inv(world_from_pad)@world_from_base
    base_from_pad=np.linalg.inv(pad_from_base)
    width=float(install["base_width"]);depth=float(install.get("clamp_band_depth",60))
    base_name="base_aluminum" if "base_aluminum" in meshes else "base_printed"
    aluminum=base_name=="base_aluminum"
    top=manifest["base_bottom_rigid_plane_above_table_mm"]+(install["aluminum_pad_tray_thickness"]+install["aluminum_thickness"] if aluminum else install["printed_base_thickness"])
    # A continuous flat band means no foot or rib in the first 1 mm above its
    # top. The actual upper C arms receive their own 60 mm envelope check.
    free_box=trimesh.creation.box([width,depth,.99],transform=pad_from_base@trimesh.transformations.translation_matrix([0,-depth/2,top+.505]))
    free=manifold(free_box);intrusions=[]
    structure_names=[part["name"] for part in manifest["parts"] if part["fixed_in_base_frame"] and not part["name"].startswith(("pad_","pressure_shoe_","hardware_"))]
    for name in structure_names:
        if aabb_gap(meshes[name].bounds,free_box.bounds)>0:continue
        volume=abs(float((solids[name]^free).volume()))
        if volume>.05:intrusions.append({"part":name,"overlap_mm3":volume})
    xs=np.linspace(-width/2+.5,width/2-.5,max(3,int(width/10)+1));ys=np.linspace(-depth+.5,-.5,max(3,int(depth/10)+1))
    base_origins=np.array([[x,y,top+.2] for x in xs for y in ys])
    origins=trimesh.transform_points(base_origins,pad_from_base)
    directions=np.tile(-pad_from_base[:3,2],(len(origins),1))
    hits,rays,_=meshes[base_name].ray.intersects_location(origins,directions,multiple_hits=False)
    heights=trimesh.transform_points(hits,base_from_pad)[:,2]
    deviation=float(np.max(abs(heights-top))) if len(heights) else None
    band={"bounds_base_xy_mm":[[-width/2,-depth],[width/2,0]],"base_top_z_mm":top,
          "depth_parameter_present":"clamp_band_depth" in install,"flat_check_height_above_base_mm":1.,
          "flat_surface_probe_count":len(origins),"flat_surface_hit_count":len(set(rays.tolist())),
          "maximum_top_surface_deviation_mm":deviation,"surface_tolerance_mm":.02,
          "fixed_structure_intrusions":intrusions,
          "pass":"clamp_band_depth" in install and not intrusions and len(set(rays.tolist()))==len(origins) and deviation is not None and deviation<.02,
          "scope":"Flat 220 x parameterized-depth mounting band only, from base top to +1 mm. Removable pressure shoes/pads/steel washers are intentional accessories. C upper-arm geometry is checked separately."}
    current={}
    for name,mesh in meshes.items():
        vertices=trimesh.transform_points(mesh.vertices,world_from_pad)
        current[name]=[vertices.min(0).tolist(),vertices.max(0).tolist()]
    boxes=np.asarray(list(current.values()));current_bounds=[boxes[:,0].min(0).tolist(),boxes[:,1].max(0).tolist()]
    snapshot=HERE/"reference/v1_fixture_world_bounds.json"
    if not snapshot.exists():snapshot=HERE.parents[1]/".local/gamepad_fixture_v1/v1_fixture_world_bounds.json"
    previous=json.loads(snapshot.read_text()) if snapshot.exists() else None
    comparison={"v1_snapshot":str(snapshot),"v1_fixture_world_bounds_mm":previous["fixture_world_bounds_mm"] if previous else None,
                "current_fixture_world_bounds_mm":current_bounds,"current_parts_world_bounds_mm":current,
                "v1_pivot_pad_mm":[0,previous["parameters_installation"]["pivot_pad_y"],previous["parameters_installation"]["pivot_pad_z"]] if previous else None,
                "current_pivot_pad_mm":manifest["pivot_pad_mm"],
                "v1_world_from_pad":previous["world_from_pad"] if previous else None,
                "current_world_from_pad":world_from_pad.tolist(),
                "pass":previous is not None and np.allclose(previous["world_from_pad"],world_from_pad,atol=.001),
                "scope":"Actual exported fixture vertices, excluding controller and C-clamp envelopes. Root pose must stay unchanged; world X extent changes are measured, not assumed improvements."}
    return band,comparison


def main(out=HERE / "out"):
    out = Path(out)
    manifest = json.loads((out / "manifest.json").read_text())
    asset = ControllerAsset()
    meshes = {p.stem: trimesh.load(p, force="mesh") for p in sorted((out / "assembly_mesh").glob("*.stl"))}
    solids = {n: manifold(m) for n, m in meshes.items()}
    p = manifest["parameters"]
    measured, install, hardware, printing = [p[k] for k in ("measured", "installation", "hardware", "printing")]
    asset_scale=np.array([measured["width"]/153.,measured["height"]/104.6,measured["depth"]/50.2])
    results = {"units": "mm, N, Nm, degrees", "executed": True,"completed":True,
               "method": "Manifold3D triangle-solid intersections and min_gap; original OBJ hull groups preserved for shell. Control sweeps use full URDF limits, with per-axis analytic arc/chord bounds and a minimum 0.35 mm allowance. No physical validation."}
    results["exported_mesh_validity"]={name:{"manifold_status":str(solid.status()),"pass":solid.status()==mf.Error.NoError,
                                              "watertight":bool(meshes[name].is_watertight)} for name,solid in solids.items()}
    # All physical parts, including steel hardware and compressed sheet pads.
    collisions = []
    expected_contacts=[]
    for na, nb in itertools.combinations(meshes, 2):
        if aabb_gap(meshes[na].bounds, meshes[nb].bounds) > 0:
            continue
        intersection=solids[na]^solids[nb]
        volume = abs(float(intersection.volume()))
        if {na,nb}=={"base_aluminum","aluminum_pad_recess_tray"}:
            # These two designed planar faces coincide. Float32 STL vertices
            # independently round that rotated plane by a few ten-thousandths
            # of a millimetre. This exception belongs only to this named pair.
            plane_z=install["tabletop_z"]+manifest["base_bottom_rigid_plane_above_table_mm"]+install["aluminum_pad_tray_thickness"]
            vertices=np.asarray(intersection.to_mesh().vert_properties)[:,:3]
            world_vertices=trimesh.transform_points(vertices,np.asarray(manifest["world_from_pad"]))
            normal_range=[float(world_vertices[:,2].min()),float(world_vertices[:,2].max())] if len(vertices) else [plane_z,plane_z]
            thickness=normal_range[1]-normal_range[0]
            plane_error=max(abs(z-plane_z) for z in normal_range)
            contact_ok=thickness<=.0001 and plane_error<=.0001
            expected_contacts.append({"parts":[na,nb],"classification":"tessellation_contact" if contact_ok else "interference",
                                      "raw_overlap_mm3":volume,"normal_thickness_mm":thickness,"normal_range_world_z_mm":normal_range,
                                      "expected_plane_normal_world":[0,0,1],"expected_plane_world_z_mm":plane_z,
                                      "maximum_distance_from_expected_plane_mm":plane_error,"normal_tolerance_mm":.0001,
                                      "theoretical_relation":"The aluminum plate underside and printed tray upper face coincide at tabletop + nominal compressed-pad protrusion + tray thickness.",
                                      "pass":contact_ok})
            if contact_ok:
                continue
        if volume > 0.05:
            collisions.append({"parts": [na, nb], "overlap_mm3": round(volume, 6)})
    results["assembly_interference"] = {"pass": not collisions, "ignore_intersection_below_mm3": 0.05, "intersections": collisions}
    results["expected_contacts"]=expected_contacts
    controls = {}
    sweep_meshes = asset.sweep_envelopes(samples=9)
    sampling_bounds=asset.sweep_sampling_bounds(samples=9)
    for name, sweep in sweep_meshes.items():
        sweep.apply_scale(asset_scale)
        sweep_solid = manifold(sweep)
        gap,near=1000.,None
        for bound,part in sorted((aabb_gap(sweep.bounds,mesh.bounds),part) for part,mesh in meshes.items()):
            if bound>gap:break
            value=float(sweep_solid.min_gap(solids[part],gap))
            if near is None or value<gap:gap,near=value,part
        overlaps = [{"part": part, "volume_mm3": float((sweep_solid ^ solid).volume())}
                    for part, solid in solids.items() if aabb_gap(sweep.bounds, meshes[part].bounds) == 0 and (sweep_solid ^ solid).volume() > 0.05]
        analytic_bound=sampling_bounds[name]*float(asset_scale.max())
        allowance=max(.35,analytic_bound)
        controls[name] = {"minimum_gap_mm": round(gap, 4), "nearest_part": near,
                          "analytic_chord_error_bound_mm":analytic_bound,"sampling_allowance_mm":allowance,
                          "conservative_gap_after_sampling_bound_mm": round(gap - allowance, 4),
                          "pass": gap > allowance and not overlaps, "overlaps": overlaps,
                          "sweep_bounds_mm": sweep.bounds.tolist()}
    results["control_full_travel_clearance"] = controls
    # Preserve shell groups: some original groups touch, and a merged mesh is nonmanifold.
    shell_components = asset.components(SHELL_NAMES)
    for _,mesh in shell_components:mesh.apply_scale(asset_scale)
    shell_solids = [manifold(m) for _, m in shell_components]
    shell_checks = {}
    for name, solid in solids.items():
        volume=0.;gap=1000.
        for bound,index in sorted((aabb_gap(meshes[name].bounds,m.bounds),index) for index,(_,m) in enumerate(shell_components)):
            if bound==0:volume+=abs(float((solid^shell_solids[index]).volume()))
            if bound<gap:gap=min(gap,float(solid.min_gap(shell_solids[index],gap)))
        soft = name.startswith("pad_")
        shell_checks[name] = {"overlap_mm3_sum_hulls": round(volume, 5), "minimum_gap_mm": round(gap, 4),
                              "contact_class": "compressed_sheet_contact" if soft else "rigid_clearance",
                              "pass": volume < 0.05,
                              "note": "Soft-pad surface is a 2 mm sampled exterior; contact error is reported, not hidden." if soft else ""}
    results["shell_contact"] = shell_checks
    # Large grip volumes intentionally reserve a finger space behind both handles.
    grips = {}
    for side, lo, hi in [("left", [-80, -60, -48], [-30, 3, -12]), ("right", [30, -60, -48], [80, 3, -12])]:
        box = trimesh.creation.box(extents=np.array(hi)-lo, transform=trimesh.transformations.translation_matrix((np.array(lo)+hi)/2))
        forbidden = manifold(box)
        overlaps = [{"part": name, "volume_mm3": float((solid ^ forbidden).volume())} for name, solid in solids.items()
                    if aabb_gap(meshes[name].bounds, box.bounds) == 0 and (solid ^ forbidden).volume() > .05]
        grips[side] = {"bounds_mm": [lo, hi], "pass": not overlaps, "overlaps": overlaps}
    results["grip_back_keepout"] = grips
    # Compare the assembled registration against the immutable scene quaternion.
    transform = np.asarray(manifest["world_from_pad"])
    pose = manifest["target_root_pose"]
    target_rotation = Rotation.from_quat([pose[4], pose[5], pose[6], pose[3]]).as_matrix()
    translation_error = float(np.linalg.norm(transform[:3, 3]-np.array(pose[:3])*1000))
    angle_error = float(Rotation.from_matrix(transform[:3, :3] @ target_rotation.T).magnitude()*180/math.pi)
    results["nominal_pose"] = {"position_error_mm": translation_error, "angle_error_deg": angle_error,
                               "pass": translation_error <= 1 and angle_error <= .5,
                               "mark_label_deg": manifest["mark_label_deg"], "actual_key_angle_deg": manifest["key_angle_deg"],
                               "scope": "Nominal rigid registration only; physical sheet compression and table-edge registration are separate below."}
    base_name="base_aluminum" if "base_aluminum" in meshes else "base_printed"
    base_world=meshes[base_name].copy();base_world.apply_transform(transform)
    underside=meshes["pad_base_bottom"].copy();underside.apply_transform(transform)
    registration_correction=np.array([install["table_edge_x"]-base_world.bounds[1,0],
                                      install["table_center_y"]-base_world.bounds[:,1].mean(),
                                      install["tabletop_z"]-underside.bounds[0,2]])
    reconstructed_root=transform[:3,3]+registration_correction
    actual_registration_error=float(np.linalg.norm(reconstructed_root-np.array(pose[:3])*1000))
    results["nominal_pose"]["exported_mesh_registration"]={
        "method":"Independently align exported base far edge to table edge, base lateral midpoint to table centre and compressed bottom sheet to tabletop. Base orientation is the prescribed flat-table mounting.",
        "base_far_edge_world_x_mm":float(base_world.bounds[1,0]),
        "bottom_sheet_lowest_world_z_mm":float(underside.bounds[0,2]),
        "registration_correction_world_mm":registration_correction.tolist(),
        "reconstructed_controller_root_world_mm":reconstructed_root.tolist(),
        "root_position_error_mm":actual_registration_error,
        "pass":actual_registration_error<=1}
    builds = {part["name"]: {"dimensions_mm": part["print_bounds_mm"], "pass": max(part["print_bounds_mm"]) <= 300,
                              "valid_solid": part["valid_solid"], "solid_count": part["solid_count"]}
              for part in manifest["parts"] if part["is_printed"]}
    results["build_volume"] = builds
    simulation=trimesh.load(out/"simulation/fixture.glb",force="scene")
    expected_bounds=np.array([np.min([m.bounds[0] for m in meshes.values()],axis=0),np.max([m.bounds[1] for m in meshes.values()],axis=0)])
    simulation_error=float(np.abs(simulation.bounds*1000-expected_bounds).max())
    material_matches={part["name"]:part["name"] in simulation.geometry and simulation.geometry[part["name"]].visual.material.name==part["material"] for part in manifest["parts"]}
    results["simulation_export"]={"units":"m","part_count":len(simulation.geometry),
                                  "bounds_scale_error_mm":simulation_error,"named_material_matches":material_matches,
                                  "pass":manifest.get("simulation_units")=="m" and simulation_error<.001 and all(material_matches.values()) and set(simulation.geometry)==set(meshes)}
    wanted = [measured["center_span"] - measured["center_span_uncertainty"], measured["center_span"] + measured["center_span_uncertainty"]]
    span = manifest["jaw_span_range_mm"]
    results["clamping_span"] = {"required_mm": wanted, "actual_mm": span, "pass": span[0] <= wanted[0] and span[1] >= wanted[1],
                                "spring_force_N_at_min_nominal_max": manifest["mechanics"]["spring_total_closing_force_N_at_min_nominal_max"],
                                "spring_coil_bind_margin_mm": manifest["mechanics"]["spring_solid_height_margin_mm"]}
    moving_names={"slider_upper_jaw","pad_upper_jaw","lock_screw_spacer"}
    motion_collisions=[]
    for width in np.linspace(span[0],span[1],21):
        delta=width-measured["center_span"]
        for moving in moving_names & meshes.keys():
            shifted=meshes[moving].copy();shifted.apply_translation([0,delta,0]);moved=manifold(shifted)
            for fixed in meshes.keys()-moving_names:
                if "compression_spring" in fixed:
                    continue  # Deforms with stroke; never a fixed nominal solid.
                if aabb_gap(shifted.bounds,meshes[fixed].bounds)>0:continue
                volume=abs(float((moved^solids[fixed]).volume()))
                if volume>.05:
                    motion_collisions.append({"span_mm":round(float(width),4),"moving":moving,"fixed":fixed,"overlap_mm3":round(volume,5)})
    results["slider_motion"]={"sampled_spans_mm":np.linspace(span[0],span[1],21).tolist(),"pass":not motion_collisions,
                              "intersections":motion_collisions,"scope":"Sliding assembly vs all fixed fixture solids except deforming compression springs; controller fit outside nominal shell dimensions requires measured shell geometry."}
    locking=[]
    lock_y=float(install["lock_center_y"])
    backing=meshes["cradle_lower_jaw"]
    for width in np.linspace(span[0],span[1],21):
        y=lock_y+width-measured["center_span"]
        # Probe a small disk under the M4 screw end, from behind the backplate.
        offsets=[(0,0)]+[(1.5*math.cos(a),1.5*math.sin(a)) for a in np.linspace(0,2*math.pi,8,endpoint=False)]
        origins=np.array([[x,y+dy,install["frame_back_z"]-1] for x,dy in offsets])
        points,rays,_=backing.ray.intersects_location(origins,np.tile([0,0,1],(len(origins),1)),multiple_hits=True)
        supported={int(ray) for point,ray in zip(points,rays) if point[2]<=install["frame_front_z"]+.05}
        locking.append({"span_mm":float(width),"screw_axis_y_mm":float(y),"supported_probe_count":len(supported),"pass":len(supported)==len(offsets)})
    results["slider_lock_backing"]={"pass":all(row["pass"] for row in locking),"samples":locking,
                                     "scope":"Nine rays across a 3 mm diameter screw-end footprint must hit the fixed frame before reaching the shell; checks continuous stop material at sampled spans."}
    results["spring_travel"]={"lengths_at_min_nominal_max_mm":manifest["mechanics"]["spring_installed_lengths_mm_at_min_nominal_max"],
                              "solid_height_max_mm":hardware["spring_solid_height_max"],
                              "coil_bind_margin_mm":manifest["mechanics"]["spring_solid_height_margin_mm"],
                              "rod_radial_clearance_mm":(hardware["spring_od"]-2*hardware["spring_wire"]-hardware["guide_d"])/2,
                              "stop_sleeve_radial_clearance_mm":(hardware["spring_od"]-2*hardware["spring_wire"]-install["spring_stop_outer_d"])/2,
                              "pass":manifest["mechanics"]["spring_solid_height_margin_mm"]>0 and hardware["spring_od"]-2*hardware["spring_wire"]>install["spring_stop_outer_d"],
                              "scope":"Axial spring length and radial envelopes across the stroke; exact vendor end-turn shape requires inspection."}
    modifiers={}
    if "base_printed" in meshes:
        printbase=manifold(trimesh.load(out/"print/base_printed.stl",force="mesh"))
        expected_definitions=manifest.get("modifier_expected_volumes_mm3")
        if expected_definitions is None:
            legacy=install["clamp_zone_width"]*install["clamp_zone_depth"]*(install["printed_base_thickness"]-measured["base_pad_thickness"]+measured["pad_proud_uncompressed"])
            expected_definitions={"base_clamp_zone_1":legacy,"base_clamp_zone_2":legacy}
        for path in sorted((out/"modifiers").glob("*.stl")):
            modifier=manifold(trimesh.load(path,force="mesh"))
            covered=float((printbase^modifier).volume())
            expected=expected_definitions.get(path.stem)
            modifiers[path.stem]={"intersection_with_print_base_mm3":covered,"expected_material_mm3":expected,
                                  "pass":expected is not None and abs(covered-expected)<.5}
        results["print_modifier_alignment"]={"pass":set(modifiers)==set(expected_definitions) and all(m["pass"] for m in modifiers.values()),"modifiers":modifiers,
                                               "scope":"Actual modifier and base STL intersections in the common recommended print orientation."}
    else:
        results["print_modifier_alignment"]={"pass":True,"applicable":False,"reason":"Metal base variant has no printed-base infill modifier."}
    results["mechanics"] = manifest["mechanics"]
    results["mechanics"]["analytic_pivot_pass"] = manifest["mechanics"]["pivot_tooth_shear_capacity_Nm"] >= hardware["design_pivot_torque_Nm"] and manifest["mechanics"]["pivot_separation_safety_factor"] >= 1
    perturbation = .3
    baseline = install["rear_pocket_height"]
    back_angle = math.degrees(math.atan(2*perturbation/baseline))
    base_baseline = install["base_depth"] - 8
    base_angle = math.degrees(math.atan(2*perturbation/base_baseline))
    height = (transform[2, 3] - install["tabletop_z"])
    results["sheet_compression_sensitivity"] = {
        "uniform_perturbation_each_mm": perturbation,
        "uniform_base_pad_root_shift_world_mm": [0, 0, perturbation],
        "uniform_back_pad_root_shift_world_mm": (transform[:3, 2]*perturbation).tolist(),
        "uniform_base_plus_back_upper_bound_mm": 2*perturbation,
        "uniform_angle_error_deg": 0,
        "differential_back_contact_pitch_upper_bound_deg": back_angle,
        "differential_base_pitch_upper_bound_deg": base_angle,
        "base_tilt_root_lateral_upper_bound_mm": height*math.tan(math.radians(base_angle)),
        "combined_position_upper_bound_mm": height*math.tan(math.radians(base_angle))+2*perturbation,
        "physical_pose_tolerance_certified": False,
        "risk": "Independent +/-0.3 mm sheet-compression differences across the 28 mm back pad can exceed 0.5 deg. Base differential compression plus uniform shifts can exceed 1 mm. Measure compressed stock and loaded pose, shim/recalibrate; nominal CAD alone does not certify physical pose.",
        "validation": "Apply alternating 30 N face loads at upper/lower extremes; measure root fiducials and angle. Support if max shift <=1 mm and <=0.5 deg, otherwise change hard-stop/shim registration."}
    results["mounting_band"],results["revision_world_bounds"]=check_mounting_band_and_world_bounds(out,manifest,meshes,solids)
    results["hand_space"]=check_fixture_hand_space(out,manifest,meshes)
    hand=results["hand_space"]
    results["summary"] = {"assembly_pass": results["assembly_interference"]["pass"],
                          "exported_mesh_validity_pass":all(v["pass"] for v in results["exported_mesh_validity"].values()),
                          "control_travel_pass": all(c["pass"] for c in controls.values()),
                          "grip_back_pass": all(g["pass"] for g in grips.values()),
                          "nominal_pose_pass": results["nominal_pose"]["pass"],
                          "build_volume_pass": all(b["pass"] and b["valid_solid"] for b in builds.values()),
                          "span_pass": results["clamping_span"]["pass"],
                          "slider_motion_pass":results["slider_motion"]["pass"],
                          "slider_lock_backing_pass":results["slider_lock_backing"]["pass"],
                          "print_modifier_alignment_pass":results["print_modifier_alignment"]["pass"],
                          "simulation_export_pass":results["simulation_export"]["pass"],
                          "shell_clearance_pass":all(s["pass"] for s in shell_checks.values()),
                          "analytic_pivot_pass":results["mechanics"]["analytic_pivot_pass"],
                          "mounting_band_pass":results["mounting_band"]["pass"],
                          "root_unchanged_from_v1_pass":results["revision_world_bounds"]["pass"],
                          "hand_saved_states_pass":hand["saved_states_fixture_pass"],
                          "hand_point_ik_diagnostics_pass":hand["diagnostic_states_fixture_pass"],
                          "clamp_hand_saved_states_pass":hand["saved_states_clamp_pass"],
                          "clamp_hand_point_ik_diagnostics_pass":hand["diagnostic_states_clamp_pass"],
                          "clamp_fixture_pass":hand["clamp_vs_fixture_pass"],
                          "cross_clamp_access_pass":hand["cross_clamp_pass"],
                          "operator_own_clamp_access_pass":hand["operator_own_clamp_pass"],
                          "full_control_motion_certified":False,
                          "physical_pose_certified": False}
    (out / "checks.json").write_text(json.dumps(results, indent=2) + "\n")
    print(json.dumps(results["summary"], indent=2))
    print("Control minimum gap:", min((v["minimum_gap_mm"], k) for k, v in controls.items()))
    print("Assembly intersections:", collisions)
    print("Shell nonpasses:", {k:v for k,v in shell_checks.items() if not v["pass"]})
    return all(value for key,value in results["summary"].items() if key.endswith("_pass"))


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--out", type=Path, default=HERE / "out")
    args=parser.parse_args()
    try:
        passed=main(args.out)
        raise SystemExit(0 if passed else 1)
    except RuntimeError as error:
        failure={"executed":True,"completed":False,"summary":{"exported_mesh_validity_pass":False},"error":str(error)}
        (args.out/"checks.json").write_text(json.dumps(failure,indent=2)+"\n")
        raise
