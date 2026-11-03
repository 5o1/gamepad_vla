"""V3 contact measurements using ControllerAsset's upstream visual transform.

No fitted mesh becomes a CAD cutter. Results are scalar caliper-replaceable
planes; ray hit samples and robust-fit residuals are retained for review.
"""
from __future__ import annotations
import json
from pathlib import Path
import numpy as np
from asset_geometry import ControllerAsset

# Exclusion envelopes in controller coordinates (mm). Asset-derived estimates,
# expanded for fitting; actual hardware positions require caliper confirmation.
OPENINGS = [
    dict(name='pair_button', bounds_pad_mm=[[-17.9,48.35,8.1],[-9.0,55.,17.1]], source='visual Connect_Button bounds + >=2 mm lateral/depth allowance'),
    dict(name='USB', bounds_pad_mm=[[-6.,42.,-6.],[6.,55.,8.]], source='visual ray recess + provisional cable envelope; keep cable access'),
    dict(name='headphone', bounds_pad_mm=[[-23.,-24.0,-4.],[-16.,-18.,4.]], source='visual bottom ray recess; inner aperture only, verify on 1708'),
    dict(name='extension', bounds_pad_mm=[[-10.,-24.0,-4.],[10.,-17.,3.]], source='visual bottom ray recess; inner aperture only, verify on 1708'),
]

def scan(mesh, axis, side, window, step=.5):
    u,v=[k for k in range(3) if k!=axis]
    a=np.arange(window[0],window[1]+.01,step);b=np.arange(window[2],window[3]+.01,step)
    o=np.zeros((len(a)*len(b),3));o[:,axis]=100*side
    o[:,u]=np.repeat(a,len(b));o[:,v]=np.tile(b,len(a));d=np.zeros_like(o);d[:,axis]=-side
    points,ids,_=mesh.ray.intersects_location(o,d,multiple_hits=False)
    return points,len(o)

def fit_points(points, axis, exclusions):
    keep=np.ones(len(points),bool)
    for lo,hi in exclusions:
        lo,hi=np.asarray(lo),np.asarray(hi)
        # Exclude by ray footprint, not hit height: rays entering a port can
        # hit far behind the aperture and would evade a full 3-D point mask.
        other=[k for k in range(3) if k!=axis]
        keep &= ~np.all((points[:,other]>=lo[other])&(points[:,other]<=hi[other]),axis=1)
    pts=points[keep]
    # Least-trimmed median/MAD: discard recesses and seam outliers. Report
    # both raw and retained residuals; do not hide rejected geometry.
    med=np.median(pts[:,axis]);r=pts[:,axis]-med
    threshold=max(.12,3*1.4826*np.median(abs(r)))
    inlier=abs(r)<=min(threshold,.8);good=pts[inlier]
    offset=float(np.median(good[:,axis]));res=good[:,axis]-offset
    other=[k for k in range(3) if k!=axis]
    A=np.c_[good[:,other],np.ones(len(good))];coef=np.linalg.lstsq(A,good[:,axis],rcond=None)[0]
    return dict(plane_axis=axis,plane_offset_mm=offset,normal=np.eye(3)[axis].tolist(),
        raw_hit_count=len(points),excluded_opening_samples=int((~keep).sum()),
        rejected_outliers=int((~inlier).sum()),retained_samples=len(good),
        rms_mm=float(np.sqrt(np.mean(res**2))),max_abs_mm=float(abs(res).max()),
        residual_p05_p95_mm=np.percentile(res,[5,95]).tolist(),
        unconstrained_plane_coefficients=coef.tolist(),
        unconstrained_plane_rms_mm=float(np.sqrt(np.mean((good[:,axis]-A@coef)**2))),
        retained_range_mm=[good.min(0).tolist(),good.max(0).tolist()])

def measure(asset=None):
    asset=asset or ControllerAsset()
    meshes={'visual':asset.visual_shell(),'collision':asset.shell()}
    windows=dict(top=(1,1,[-20,20,6,17]),bottom=(1,-1,[-28,28,-8,14]),back=(2,-1,[-20,20,12,34]))
    report=dict(revision='v3',units='mm',parameter_source='asset-derived, not physical measurements',
        method='first-hit ray scans, opening-footprint exclusions, median/MAD robust axis-plane fit; parallel jaw normals constrained to Y',
        openings=OPENINGS,contacts={})
    for label,(axis,side,window) in windows.items():
        exclusions=[o['bounds_pad_mm'] for o in OPENINGS if (axis==1 and ((side==1)==(o['name'] in ('pair_button','USB'))))]
        # Pair's mirror is removed too to preserve structural symmetry.
        if label=='top':exclusions.append([[9,40,8.1],[17.9,60,17.1]])
        rows={}
        for name,mesh in meshes.items():
            points,total=scan(mesh,axis,side,window)
            rows[name]=dict(window_mm=window,ray_count=total,**fit_points(points,axis,exclusions))
        rows['visual_minus_collision_offset_mm']=rows['visual']['plane_offset_mm']-rows['collision']['plane_offset_mm']
        report['contacts'][label]=rows
    visual_parts=asset.visual_meshes();collision_parts=asset.meshes()
    report['obstacles_visual_bounds_mm']={n:visual_parts[n].bounds.tolist() for n in ('pair','left_shoulder','right_shoulder','left_trigger','right_trigger')}
    report['obstacles_collision_bounds_mm']={n:collision_parts[n].bounds.tolist() for n in ('pair','left_shoulder','right_shoulder','left_trigger','right_trigger')}
    report['recommended_widths_mm']={'upper':40.,'lower':56.,'back':44.}
    report['shoulder_inner_gap_mm']=46.2
    report['notes']=['Back window y=12..34 avoids provisional battery-cover envelope y<=8. Real seam/latch position is unmeasured.',
        'Flat collision regions alone do not establish real shell flatness; visual mesh is also an uncalibrated artist asset.',
        'Contact CAD uses the median axis plane only; 2 mm pads absorb residuals. No triangulated fitting surface is exported as contact geometry.']
    return report

if __name__=='__main__':
    import argparse
    p=argparse.ArgumentParser();p.add_argument('--out',type=Path,default=Path('hardware/gamepad_fixture/contact_fit.json'));a=p.parse_args()
    r=measure();a.out.write_text(json.dumps(r,indent=2)+'\n');print(json.dumps(r['contacts'],indent=2))
