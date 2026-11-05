"""Fusion STEP overrides: input uses the delivered table-up assembly frame, mm."""
import argparse
import hashlib
import json
import shutil
from pathlib import Path
import numpy as np
from build123d import import_step

HERE=Path(__file__).resolve().parent

def apply_overrides(parts, manifest, directory):
    from fixture import bounds,location
    manifest['user_overrides']=[]
    if directory is None or not Path(directory).exists():return
    F=np.eye(4);F[:3,:3]=manifest['fixed_frame_world_axes'];F[:3,3]=manifest['fixed_frame_world_origin_mm']
    table_from_pad=np.linalg.inv(F)@np.asarray(manifest['world_from_pad'])
    for path in sorted(Path(directory).iterdir()):
        if path.suffix.lower() not in ('.step','.stp'):continue
        name=path.stem
        if name not in parts:raise ValueError(f'Unknown override part {name}; use edit_table_up part names')
        if any(r['part']==name for r in manifest['user_overrides']):raise ValueError(f'Duplicate STEP for {name}')
        shape=import_step(path).moved(location(np.linalg.inv(table_from_pad)))
        if not shape.is_valid or len(shape.solids())!=1:raise ValueError(f'{name}: STEP must contain one valid solid')
        ref=parts[name];a,b=np.asarray(bounds(ref)),np.asarray(bounds(shape))
        center_error=float(np.linalg.norm(a.mean(0)-b.mean(0)));extent_error=float(abs(np.diff(a,axis=0)-np.diff(b,axis=0)).max())
        # Also check retained volume: catches rotations of near-square envelopes.
        retained=(ref & shape).volume/ref.volume
        if center_error>3 or extent_error>4 or retained<.85:
            raise ValueError(f'{name}: coordinate/large-edit mismatch (centre {center_error:.2f}mm, extents {extent_error:.2f}mm, retained {retained:.3f}); export mm in original table-up assembly origin/axes; do not centre part or use print orientation')
        if name.startswith('pad_') and ((ref-shape).volume>.01 or (shape-ref).volume>.01):
            raise ValueError(f'{name}: preserve the complete pad; revise measured parameters explicitly for pad changes')
        parts[name]=shape;shape.label=name
        manifest['user_overrides'].append(dict(part=name,file=path.name,sha256=hashlib.sha256(path.read_bytes()).hexdigest(),input_frame='table-up assembly mm',center_error_mm=center_error,extent_error_mm=extent_error,retained_volume_fraction=retained))

def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('source',type=Path);p.add_argument('--part',required=True);p.add_argument('--directory',type=Path,default=HERE/'user_overrides');a=p.parse_args()
    from fixture import build,Parameters
    parts,_,mf=build(Parameters())
    if a.part not in parts:raise ValueError(f'Unknown part: {a.part}')
    # Validate before copying; generation validates the entire active directory again.
    import tempfile
    with tempfile.TemporaryDirectory() as tmp:
        shutil.copyfile(a.source,Path(tmp)/(a.part+'.step'));apply_overrides(parts,mf,Path(tmp))
    a.directory.mkdir(parents=True,exist_ok=True);target=a.directory/(a.part+'.step');shutil.copyfile(a.source,target)
    print(json.dumps(mf['user_overrides'],indent=2));print(target)
if __name__=='__main__':main()
