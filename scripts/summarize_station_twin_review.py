#!/usr/bin/env python3
"""Make evidence sheets and a small numeric summary from completed G3 output."""
import argparse
import json
from pathlib import Path
import numpy as np
from PIL import Image, ImageDraw


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('run',type=Path)
    args=parser.parse_args()
    metrics=json.loads((args.run/'metrics.json').read_text())
    summary=dict(cameras={},spec_sha256=metrics['spec_sha256'])
    for i in (1,2,3):
        camera=f'camera_{i}'
        sheet=Image.new('RGB',(1920,4*510),(245,245,245));draw=ImageDraw.Draw(sheet)
        records=[]
        for row,(frame,frame_metrics) in enumerate(metrics['frames'].items()):
            record=frame_metrics[camera];records.append(record)
            for column,suffix in enumerate(('real_undistorted','sim','overlay')):
                image=Image.open(args.run/f'{frame}_{camera}_{suffix}.png').convert('RGB')
                sheet.paste(image,(column*640,row*510+30))
                draw.text((column*640+5,row*510+8),frame+' '+camera+' '+suffix,fill='black')
        sheet.save(args.run/f'{camera}_comparison.jpg',quality=90)
        result={}
        for label in ('arm','palm'):
            values=[r['masks'][label]['iou'] for r in records if r['masks'][label]['iou'] is not None]
            result[label+'_mean_iou']=float(np.mean(values)) if values else None
            result[label+'_per_frame_iou']=[r['masks'][label]['iou'] for r in records]
        result['automatic_roi_mean_iou']=float(np.mean([r['automatic_roi_silhouette_iou'] for r in records]))
        result['projection_agreement_max_px']=max(r['sapien_vs_numpy_projection']['max_px'] or 0 for r in records)
        result['home_grayscale_std']=float(np.asarray(Image.open(args.run/f'{camera}_home.png').convert('L')).std())
        summary['cameras'][camera]=result
    home=Image.new('RGB',(1920,510),'white');draw=ImageDraw.Draw(home)
    for i in (1,2,3):
        home.paste(Image.open(args.run/f'camera_{i}_home.png'),((i-1)*640,30))
        draw.text(((i-1)*640+5,8),f'camera_{i} home (calibrated coarse pose)',fill='black')
    home.save(args.run/'home_views.jpg',quality=90)
    pngs=sorted(args.run.glob('*.png'))
    summary['png_count']=len(pngs)
    summary['expected_png_count']=87
    summary['all_expected_images_present']=all((args.run/f'{f}_camera_{i}_{suffix}.png').is_file()
        for f in metrics['frames'] for i in (1,2,3)
        for suffix in ('sim','real_undistorted','overlay','arm_sim_mask','arm_real_mask','palm_sim_mask','palm_real_mask')) and all((args.run/f'camera_{i}_home.png').is_file() for i in (1,2,3))
    (args.run/'metrics_summary.json').write_text(json.dumps(summary,indent=2))
    print(json.dumps(summary,indent=2))


if __name__=='__main__':main()
