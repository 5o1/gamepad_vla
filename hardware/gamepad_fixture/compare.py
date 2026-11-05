"""Measured section overlays from exported v3 and v3.1 solids."""
import argparse,json
from pathlib import Path
import numpy as np
import trimesh
from render import plt

def main(old,new,destination):
    mf=json.loads((new/'manifest.json').read_text());y=mf['parameters']['measured']['top_center_y']-mf['parameters']['measured']['center_span'];upper=mf['parameters']['measured']['top_center_y']
    fig,axs=plt.subplots(1,2,figsize=(12,6),facecolor='white')
    cases=[('cradle_lower_jaw',0,[-18,0,0],[2,1],[20,28,y-2,y+4],'Lower lip | X=-18 mm (mirror +18)'),('slider_upper_jaw',2,[0,0,24],[0,1],[16,22,upper+3,upper+8],'Upper outer corner | Z=24 mm (mirrored)')]
    for ax,(name,axis,origin,cols,limits,title) in zip(axs,cases):
        for root,color,label in ((old,'#d65458','v3'),(new,'#007d84','v3.1')):
            m=trimesh.load(root/'assembly_mesh'/(name+'.stl'),force='mesh');section=m.section(plane_origin=origin,plane_normal=np.eye(3)[axis])
            for n,loop in enumerate(section.discrete):ax.plot(loop[:,cols[0]],loop[:,cols[1]],color=color,lw=2,label=label if n==0 else None,linestyle='--' if root==old else '-')
        ax.set_xlim(limits[:2]);ax.set_ylim(limits[2:]);ax.set_aspect('equal');ax.set_title(title);ax.grid(alpha=.2);ax.legend();ax.set_xlabel('Z mm' if axis==0 else 'X mm');ax.set_ylabel('Y mm')
    axs[0].annotate('2 mm retreat; 1 mm capture remains',xy=(24,y+1),xytext=(21,y-1.4),fontsize=8,arrowprops={'arrowstyle':'->'})
    axs[1].annotate('2 x 2 mm chamfer',xy=(19,upper+6),xytext=(16.2,upper+3.3),fontsize=8,arrowprops={'arrowstyle':'->'})
    fig.suptitle('V3 / V3.1 overlay | actual CAD sections | mm');fig.tight_layout();destination.mkdir(parents=True,exist_ok=True);fig.savefig(destination/'v3_vs_v3_1.png',dpi=240);fig.savefig(destination/'v3_vs_v3_1.pdf');plt.close(fig)
if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--old',type=Path,required=True);p.add_argument('--new',type=Path,required=True);p.add_argument('--destination',type=Path,required=True);a=p.parse_args();main(a.old,a.new,a.destination)
