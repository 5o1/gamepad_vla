#!/usr/bin/env python3
"""Fewest hulls per static CAD part among independently measured candidates.

Parts remain separate. Distance/contact/keepout constraints apply independently
per part, so minimizing their sum is separable in this finite candidate family.
This is not a global minimum convex decomposition claim.
"""
import argparse,json,shutil
from pathlib import Path
from convex_collision import HERE,ROOT


def main(a):
    manifest=json.loads((HERE/'out/manifest.json').read_text());contacts={c['pad']:c['name'] for c in manifest['contact_planes']}
    cad=json.loads((a.cad/'accuracy.json').read_text());candidates={n:[] for n in cad['parts']}
    directories=[('cad',a.cad)]+[(p.name,p) for p in sorted(a.coacd.glob('t*')) if p.is_dir() and (p/'accuracy.json').exists()]
    for label,folder in directories:
        metrics=json.loads((folder/'accuracy.json').read_text())
        for name,r in metrics['parts'].items():
            if name not in candidates:continue
            contact=metrics['contacts'].get(contacts[name]) if name in contacts else None
            eligible=(r['outward']['max_mm']<=1.5 and r['vertices_max']<=255 and not r['protected_intersections'] and
                (name not in contacts or contact is not None and contact['source_samples']>0 and contact['max_mm']<=.1))
            candidates[name].append(dict(method=label,directory=str(folder.relative_to(ROOT)) if folder.is_relative_to(ROOT) else str(folder),hulls=r['hulls'],eligible=bool(eligible),outward=r['outward'],inward=r['inward'],contact=contact,protected_intersections=r['protected_intersections']))
    selected={}
    a.output.mkdir(parents=True,exist_ok=True)
    for name,options in candidates.items():
        valid=[r for r in options if r['eligible']]
        if not valid:raise ValueError(f'{name}: no measured eligible candidate')
        winner=min(valid,key=lambda r:(r['hulls'],r['method']!='cad',r['outward']['max_mm']))
        selected[name]=winner;source=Path(winner['directory']);source=source if source.is_absolute() else ROOT/source;shutil.copyfile(source/(name+'.obj'),a.output/(name+'.obj'))
    result=dict(hulls=sum(r['hulls'] for r in selected.values()),cad_hulls=cad['hulls'],chosen=selected,candidates=candidates,scope='Minimum sum over independently validated per-part candidates only; global geometry is rechecked after composition.')
    (a.output/'selection.json').write_text(json.dumps(result,indent=2)+'\n');print(json.dumps(dict(hulls=result['hulls'],cad_hulls=result['cad_hulls'],choices={n:r['method'] for n,r in selected.items()})),flush=True)
if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--cad',type=Path,default=ROOT/'.local/fixture-convex/cad_adaptive_merged');p.add_argument('--coacd',type=Path,default=ROOT/'.local/fixture-convex/coacd_scan');p.add_argument('--output',type=Path,default=ROOT/'.local/fixture-convex/selected');main(p.parse_args())
