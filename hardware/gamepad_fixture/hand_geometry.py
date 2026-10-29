"""Reconstruct saved G20 hand witnesses using their original URDF triangles.

No reachability search, simulation stepping or hardware command is performed.
Frames and meshes returned here use controller-local millimetres. Successful
point IK endpoints remain diagnostics; they do not become accepted operations.
"""
from __future__ import annotations

import hashlib
import json
from pathlib import Path
import xml.etree.ElementTree as ET

import numpy as np
from scipy.spatial.transform import Rotation
import trimesh

from asset_geometry import origin_transform, rotation

ROOT=Path(__file__).resolve().parents[2]
RUNS=ROOT/".local/two-hand-k0"
REPORT=ROOT/"docs/reports/2026-10-21-two-hand-feasibility"
REMOTE_SOURCE="compute-node:two-hand-k0-20261021-222938"


def pose_matrix(pose):
    """Source poses are xyz in metres followed by quaternion wxyz."""
    matrix=np.eye(4)
    matrix[:3,:3]=Rotation.from_quat([pose[4],pose[5],pose[6],pose[3]]).as_matrix()
    matrix[:3,3]=np.asarray(pose[:3])*1000
    return matrix


class HandAsset:
    def __init__(self, side):
        self.side=side
        self.urdf=ROOT/f"assets/linkerhand_g20_l20_l25_urdf_mesh/g20_{side}/linkerhand_g20_{side}.urdf"
        root=ET.parse(self.urdf).getroot()
        self.joints={}
        self.children={}
        for element in root.findall("joint"):
            mimic=element.find("mimic")
            axis=element.find("axis")
            data={"name":element.get("name"),"type":element.get("type"),
                  "parent":element.find("parent").get("link"),"child":element.find("child").get("link"),
                  "origin":origin_transform(element.find("origin")),
                  "axis":np.fromstring(axis.get("xyz","1 0 0"),sep=" ") if axis is not None else np.array([1.,0,0]),
                  "mimic":None if mimic is None else (mimic.get("joint"),float(mimic.get("multiplier","1")),float(mimic.get("offset","0")))}
            self.joints[data["name"]]=data;self.children[data["child"]]=data
        self.geometry=[]
        for element in root.findall("link"):
            for index,collision in enumerate(element.findall("collision")):
                mesh=collision.find("geometry/mesh")
                if mesh is None:
                    raise ValueError(f"Unsupported non-mesh hand collision geometry: {element.get('name')}")
                source=self.urdf.parent/mesh.get("filename")
                triangles=trimesh.load(source,force="mesh",process=True)
                triangles.apply_scale(np.fromstring(mesh.get("scale","1 1 1"),sep=" ")*1000)
                self.geometry.append((element.get("name"),origin_transform(collision.find("origin")),triangles,source))

    def chain_active(self, finger):
        link=finger+"_distal";names=[]
        while link in self.children:
            joint=self.children[link]
            if joint["type"]!="fixed" and joint["mimic"] is None:names.append(joint["name"])
            link=joint["parent"]
        return names[::-1]

    def transforms(self, state):
        q=state["hand_q"];palm=np.asarray(state["palm_in_pad"],float)
        frames={}
        def value(name):
            joint=self.joints[name]
            if joint["mimic"]:
                source,multiplier,offset=joint["mimic"]
                return multiplier*value(source)+offset
            return float(q.get(name,0.))
        def frame(link):
            if link in frames:return frames[link]
            if link not in self.children:
                frames[link]=palm
            else:
                joint=self.children[link];motion=np.eye(4)
                if joint["type"] in ("revolute","continuous"):
                    motion[:3,:3]=rotation(joint["axis"],value(joint["name"]))
                elif joint["type"]=="prismatic":
                    motion[:3,3]=joint["axis"]*value(joint["name"])*1000
                elif joint["type"]!="fixed":raise ValueError(joint["type"])
                frames[link]=frame(joint["parent"])@joint["origin"]@motion
            return frames[link]
        return {link:frame(link)@origin for link,origin,_,_ in self.geometry}

    def meshes(self, state):
        frames=self.transforms(state)
        result={}
        for link,_,mesh,_ in self.geometry:
            copy=mesh.copy();copy.apply_transform(frames[link]);result[f"{self.side}/{link}"]=copy
        return result

    def manager(self, *, convex=False):
        manager=trimesh.collision.CollisionManager()
        for link,_,mesh,_ in self.geometry:
            manager.add_object(f"{self.side}/{link}",mesh.convex_hull if convex else mesh)
        return manager

    def set_manager(self, manager, state):
        for link,frame in self.transforms(state).items():
            manager.set_transform(f"{self.side}/{link}",frame)


def load_hand_states(runs=RUNS, *, include_diagnostics=True):
    """All b_raise40 baselines, saved endpoints, and computed point-IK states.

    Includes successful point-IK endpoints at every saved grasp, rather than
    fabricating missing controls. Their original full-hand rejection is retained
    in the quality label. No claim is made about continuous transition paths.
    """
    states=[];coverage={"fixture":"b_raise40","frame":"controller local mm",
                        "sources":[],"sides":{},"full_control_motion_certified":False,
                        "scope":"Saved static grasp/endpoints and already-computed successful point IK endpoints; no new search or dynamic path execution. Missing controls stay unverified.",
                        "remote_archive":REMOTE_SOURCE}
    for side in ("left","right"):
        path=Path(runs)/side/"witnesses.json"
        if not path.exists():
            raise FileNotFoundError(f"Missing actual saved hand witnesses: {path}; public static_witnesses.json does not contain the left b_raise40 baseline.")
        raw=json.loads(path.read_text());witnesses=raw["b_raise40"]
        goals=json.loads((REPORT/(side+"_result.json")).read_text())["goals"]
        asset=HandAsset(side)
        coverage["sources"].append({"path":str(path.relative_to(ROOT)),"sha256":hashlib.sha256(path.read_bytes()).hexdigest()})
        accepted={g["name"]:set() for g in goals};point_ik={g["name"]:set() for g in goals}
        certified=set();selected={};cache_path=Path(runs)/side/"kinematics.npz"
        cache=np.load(cache_path) if include_diagnostics and cache_path.exists() else None
        if cache is not None:coverage["sources"].append({"path":str(cache_path.relative_to(ROOT)),"sha256":hashlib.sha256(cache_path.read_bytes()).hexdigest()})
        for witness_index,witness in enumerate(witnesses):
            pi=witness["palm_index"];base={"side":side,"palm_index":pi,"palm_in_pad":pose_matrix(witness["palm_in_pad"]).tolist(),
                                           "source":str(path.relative_to(ROOT)),"hand_q":witness["resting_hand_q"],"gamepad_q":{}}
            states.append({**base,"id":f"{side}_p{pi}_grasp","quality":"saved_grasp_baseline","control":None,"state_index":None})
            certified.update(witness["controls"])
            for index,state in enumerate(witness["states"]):
                accepted[state["control"]].add(int(state["state"]))
                states.append({**base,"id":f"{side}_p{pi}_saved_{state['control']}_{state['state']}",
                               "quality":"saved_static_endpoint","control":state["control"],"state_index":int(state["state"]),
                               "hand_q":state["hand_q"],"gamepad_q":state["gamepad_q"],
                               "control_group_certified_in_original_search":state["control"] in witness["controls"]})
            if cache is not None:
                row=int(np.searchsorted(cache["grip_ids"],pi))
                if int(cache["grip_ids"][row])!=pi:raise ValueError(f"Missing grip index {pi}")
                for goal in goals:
                    name=goal["name"];successful=np.flatnonzero(cache[name+"_ok"][row])
                    point_ik[name].update(int(index) for index in successful)
                    if witness_index==0:selected[name]=[int(index) for index in successful]
                    for index in successful:
                        q=witness["resting_hand_q"].copy()
                        q.update(zip(asset.chain_active(goal["finger"]),[float(x) for x in cache[name+"_q"][row,index]]))
                        states.append({**base,"id":f"{side}_p{pi}_pointIK_{name}_{index}","quality":"point_ik_diagnostic",
                                       "control":name,"state_index":int(index),"hand_q":q,
                                       "gamepad_q":goal["states"][int(index)]["gamepad_q"],
                                       "scope":"Point IK succeeded; original whole-hand/native-contact acceptance is not implied."})
        required={g["name"]:{"states":len(g["states"]),"alternatives":bool(g.get("alternatives",False))} for g in goals}
        coverage["sides"][side]={"saved_grasp_count":len(witnesses),"saved_palm_indices":[w["palm_index"] for w in witnesses],
                                  "reference_palm_index":witnesses[0]["palm_index"] if witnesses else None,
                                  "saved_static_endpoint_indices":{k:sorted(v) for k,v in accepted.items()},
                                  "original_certified_control_groups":sorted(certified),
                                  "required_control_groups":required,
                                  "missing_original_certified_control_groups":sorted(set(required)-certified),
                                  "available_point_ik_at_any_saved_grasp":{k:sorted(v) for k,v in point_ik.items()},
                                  "available_point_ik_at_reference_grasp":selected,
                                  "missing_point_ik_at_reference_grasp":[k for k,v in selected.items() if not v],
                                  "complete_continuous_motion_available":False}
    coverage["extra_physical_controls_without_hand_states"]=["Pair / bind button: no finger assignment or saved hand endpoint in this dataset"]
    coverage["state_counts"]={quality:sum(s["quality"]==quality for s in states) for quality in sorted({s["quality"] for s in states})}
    return states,coverage


def check_fixture_hand_space(out, manifest, fixture_meshes):
    """Actual per-link triangle BVHs, plus a conservative per-link hull check.

    Triangle queries remain meaningful for the manufacturer's non-watertight
    STL surfaces. Hull distances also detect potential solid containment and
    provide a conservative clearance; neither query uses a whole-hand AABB.
    """
    out=Path(out);states,coverage=load_hand_states()
    fixture_managers={}
    for name,mesh in fixture_meshes.items():
        manager=trimesh.collision.CollisionManager();manager.add_object(name,mesh);fixture_managers[name]=manager
    envelope_entries=manifest.get("clamp_envelopes",[])
    if isinstance(envelope_entries,dict):envelope_entries=envelope_entries.get("entries",[])
    envelope_meshes={};envelope_managers={};envelope_meta={}
    for entry in envelope_entries:
        file=Path(entry["file"])
        if not file.is_absolute():file=out/file
        mesh=trimesh.load(file,force="mesh");name=entry["name"]
        envelope_meshes[name]=mesh;envelope_meta[name]=entry
        manager=trimesh.collision.CollisionManager();manager.add_object(name,mesh);envelope_managers[name]=manager

    def measurement(manager,other):
        distance,names,data=manager.min_distance_other(other,return_names=True,return_data=True)
        hit,contacts=manager.in_collision_other(other,return_data=True) if distance<=0 else (False,[])
        # FCL mesh penetration output can be a negative sentinel rather than a
        # signed Euclidean clearance. Keep it raw and name the contact metric.
        return {"surface_gap_mm":round(max(float(distance),0),5),"raw_fcl_distance_mm":round(float(distance),5),
                "nearest_objects":list(names) if names is not None else None,
                "triangle_contact":bool(hit),"maximum_fcl_contact_depth_mm":round(max([float(c.depth) for c in contacts]+[0]),5)}

    state_rows=[];bounds_by_quality={};reference_bounds={}
    for side in ("left","right"):
        asset=HandAsset(side);exact=asset.manager();hulls=asset.manager(convex=True)
        for state in (s for s in states if s["side"]==side):
            asset.set_manager(exact,state);asset.set_manager(hulls,state)
            parts={};envelopes={}
            for name,manager in fixture_managers.items():
                row=measurement(manager,exact);conservative=measurement(manager,hulls)
                row["conservative_link_hull_gap_mm"]=conservative["surface_gap_mm"]
                row["conservative_link_hull_contact"]=conservative["triangle_contact"]
                row["pass"]=not row["triangle_contact"] and not conservative["triangle_contact"]
                parts[name]=row
            for name,manager in envelope_managers.items():
                row=measurement(manager,exact);conservative=measurement(manager,hulls)
                row["conservative_link_hull_gap_mm"]=conservative["surface_gap_mm"]
                row["conservative_link_hull_contact"]=conservative["triangle_contact"]
                row["role"]=envelope_meta[name]["role"]
                row["pass"]=not row["triangle_contact"] and not conservative["triangle_contact"]
                envelopes[name]=row
            state_meshes=asset.meshes(state)
            boxes=np.array([m.bounds for m in state_meshes.values()]);bounds=[boxes[:,0].min(0),boxes[:,1].max(0)]
            world_boxes=[]
            for mesh in state_meshes.values():
                vertices=trimesh.transform_points(mesh.vertices,np.asarray(manifest["world_from_pad"]))
                world_boxes.append([vertices.min(0),vertices.max(0)])
            world_boxes=np.asarray(world_boxes);world_bounds=[world_boxes[:,0].min(0),world_boxes[:,1].max(0)]
            quality_key=side+"/"+state["quality"]
            if quality_key not in bounds_by_quality:bounds_by_quality[quality_key]=bounds
            else:bounds_by_quality[quality_key]=[np.minimum(bounds_by_quality[quality_key][0],bounds[0]),np.maximum(bounds_by_quality[quality_key][1],bounds[1])]
            if state["quality"]=="saved_grasp_baseline" and state["palm_index"]==coverage["sides"][side]["reference_palm_index"]:
                reference_bounds[side]=[b.tolist() for b in bounds]
            state_rows.append({"id":state["id"],"side":side,"palm_index":state["palm_index"],"quality":state["quality"],
                               "control":state["control"],"state_index":state["state_index"],"bounds_pad_mm":[b.tolist() for b in bounds],
                               "bounds_world_mm":[b.tolist() for b in world_bounds],
                               "fixture_parts":parts,"clamp_envelopes":envelopes,
                               "fixture_pass":all(p["pass"] for p in parts.values()),
                               "clamp_pass":bool(envelopes) and all(p["pass"] for p in envelopes.values())})
    # Clamp envelopes are hardware/access reservations, not fixture solids.
    # Only the corresponding clamp's bearing footprint is intentional contact.
    fixture_envelope_rows=[]
    for name,manager in envelope_managers.items():
        entry=envelope_meta[name];index=int(entry["clamp_index"])
        own_pressure_parts=set(entry.get("expected_fixture_contacts",[]))
        for part,other in fixture_managers.items():
            row=measurement(manager,other)
            expected=entry["role"]=="hardware" and part in own_pressure_parts
            row.update({"envelope":name,"fixture_part":part,"role":entry["role"],
                        "classification":"expected_pressure_footprint" if expected else "ordinary_clearance",
                        "pass":expected or not row["triangle_contact"]})
            fixture_envelope_rows.append(row)
    cross_clamp_rows=[]
    own_operator_rows=[]
    for index,(name,manager) in enumerate(envelope_managers.items()):
        first=envelope_meta[name]
        for other_name,other in list(envelope_managers.items())[index+1:]:
            second=envelope_meta[other_name]
            if int(first["clamp_index"])==int(second["clamp_index"]):
                if {first["role"],second["role"]}!={"hardware","operator_access"}:continue
                hardware=first if first["role"]=="hardware" else second
                row=measurement(manager,other)
                intended=bool(hardware.get("operator_intended_contact",False))
                row.update({"envelopes":[name,other_name],"classification":"intended_operator_contact" if intended else "operator_clearance",
                            "pass":intended or not row["triangle_contact"]})
                own_operator_rows.append(row)
                continue
            if first["role"]==second["role"]=="operator_access":
                continue  # C clamps are operated sequentially; ghost hands may overlap.
            row=measurement(manager,other)
            row.update({"envelopes":[name,other_name],"roles":[first["role"],second["role"]],"pass":not row["triangle_contact"]})
            cross_clamp_rows.append(row)
    def worst(rows,field):
        candidates=[(item["surface_gap_mm"],state["id"],name,item) for state in rows for name,item in state[field].items()]
        if not candidates:return None
        gap,state_id,name,item=min(candidates,key=lambda v:v[0])
        return {"state":state_id,"object":name,**item}
    saved=[row for row in state_rows if row["quality"]!="point_ik_diagnostic"]
    diagnostics=[row for row in state_rows if row["quality"]=="point_ik_diagnostic"]
    result={"coverage":coverage,"method":"FCL original per-link URDF triangle BVHs and separate conservative per-link convex hulls; exact URDF FK including all mimic joints. No whole-hand AABB collision verdict.",
            "mesh_quality":{"left_nonwatertight_links":[link for link,_,m,_ in HandAsset("left").geometry if not m.is_watertight],
                            "right_nonwatertight_links":[link for link,_,m,_ in HandAsset("right").geometry if not m.is_watertight],
                            "note":"Original non-watertight STL surfaces are preserved. FCL contact depth is diagnostic, not certified solid penetration volume; per-link hulls conservatively cover solid interiors."},
            "reference_grasp_bounds_pad_mm":reference_bounds,
            "bounds_by_side_quality_pad_mm":{key:[b.tolist() for b in box] for key,box in bounds_by_quality.items()},
            "states":state_rows,"saved_states_fixture_pass":all(row["fixture_pass"] for row in saved),
            "diagnostic_states_fixture_pass":all(row["fixture_pass"] for row in diagnostics),
            "saved_states_clamp_pass":bool(envelope_entries) and all(row["clamp_pass"] for row in saved),
            "diagnostic_states_clamp_pass":bool(envelope_entries) and all(row["clamp_pass"] for row in diagnostics),
            "minimum_saved_fixture":worst(saved,"fixture_parts"),"minimum_diagnostic_fixture":worst(diagnostics,"fixture_parts"),
            "minimum_saved_clamp":worst(saved,"clamp_envelopes"),"minimum_diagnostic_clamp":worst(diagnostics,"clamp_envelopes"),
            "clamp_vs_fixture":fixture_envelope_rows,
            "clamp_vs_fixture_pass":bool(envelope_entries) and all(row["pass"] for row in fixture_envelope_rows),
            "cross_clamp_clearance":cross_clamp_rows,
            "cross_clamp_pass":bool(envelope_entries) and all(row["pass"] for row in cross_clamp_rows),
            "operator_own_clamp_clearance":own_operator_rows,
            "operator_own_clamp_pass":bool(envelope_entries) and all(row["pass"] for row in own_operator_rows),
            "clamp_operation_scope":"The two operator-access hand envelopes may overlap because clamps are operated sequentially. Each hand envelope is still checked against the other clamp's hardware sweep.",
            "full_control_motion_certified":False}
    (out/"hand_states.json").write_text(json.dumps({"coverage":coverage,"states":states},indent=2)+"\n")
    (out/"hand_clearances.json").write_text(json.dumps(result,indent=2)+"\n")
    return result


if __name__=="__main__":
    import argparse
    parser=argparse.ArgumentParser();parser.add_argument("--output",type=Path,default=ROOT/"hardware/gamepad_fixture/out/hand_geometry.json")
    parser.add_argument("--check-out",type=Path)
    args=parser.parse_args()
    if args.check_out:
        out=args.check_out;manifest=json.loads((out/"manifest.json").read_text())
        meshes={p.stem:trimesh.load(p,force="mesh") for p in (out/"assembly_mesh").glob("*.stl")}
        result=check_fixture_hand_space(out,manifest,meshes)
        print(json.dumps({key:value for key,value in result.items() if key.startswith("minimum_") or key.endswith("_pass")},indent=2))
        raise SystemExit(0)
    states,coverage=load_hand_states()
    bounds={}
    for side in ("left","right"):
        asset=HandAsset(side);selected=[s for s in states if s["side"]==side]
        boxes=np.array([mesh.bounds for state in selected for mesh in asset.meshes(state).values()])
        bounds[side]=[boxes[:,0,:].min(0).tolist(),boxes[:,1,:].max(0).tolist()]
    args.output.parent.mkdir(parents=True,exist_ok=True)
    args.output.write_text(json.dumps({"coverage":coverage,"bounds_mm_all_available_states":bounds,"states":states},indent=2)+"\n")
    print(json.dumps({"counts":coverage["state_counts"],"bounds_mm":bounds},indent=2))
