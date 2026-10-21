#!/usr/bin/env python3
"""Replay recorded MuJoCo qpos with the actual game state and report overlay.

This is recorded-state visualization, not a new contact experiment. Gamepad
collision meshes are shown; no graphics edits or physics corrections are applied.
"""
import argparse
import json
import os
from pathlib import Path
import subprocess

import numpy as np
from PIL import Image, ImageDraw


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--run',type=Path,required=True)
    parser.add_argument('--output',type=Path,required=True)
    parser.add_argument('--fps',type=int,default=25)
    parser.add_argument('--software',action='store_true',help='force Mesa EGL software rendering')
    args=parser.parse_args()
    if args.software:
        os.environ.update(MUJOCO_GL='egl',LIBGL_ALWAYS_SOFTWARE='1',
            __EGL_VENDOR_LIBRARY_FILENAMES='/usr/share/glvnd/egl_vendor.d/50_mesa.json',LP_NUM_THREADS='2')
    import mujoco
    from OpenGL import GL

    summary=json.loads((args.run/'summary.json').read_text())
    controls=[json.loads(s) for s in (args.run/'control_trace.jsonl').read_text().splitlines()]
    games=[json.loads(s) for s in (args.run/'game_trace.jsonl').read_text().splitlines()]
    if not controls or 'qpos' not in controls[0] or 'initial_game_state' not in summary:
        parser.error('run the game loop with --record-state first')
    args.output.mkdir(parents=True,exist_ok=False)
    from gamepad_vla.tasks.fixed_gamepad import resolve_asset_files
    from gamepad_vla.workstation.paths import resolve_workspace_path
    original_xml=(args.run/'controller/model/scene.xml').read_text()
    replay_xml=resolve_asset_files(original_xml,resolve_workspace_path(summary['controller_config']['resource_root']))
    (args.output/'replay_scene.xml').write_text(replay_xml)
    model=mujoco.MjModel.from_xml_path(str(args.output/'replay_scene.xml'))
    data=mujoco.MjData(model)
    # Only visualization attributes change in this replay model.
    for i in range(model.ngeom):
        if model.geom_group[i]==3:
            body=model.body(int(model.geom_bodyid[i])).name
            model.geom_rgba[i]=[.12,.12,.14,1.] if 'dpad' in body else [.32,.35,.4,1.]
    model.vis.global_.offwidth=640;model.vis.global_.offheight=480
    renderer=mujoco.Renderer(model,height=480,width=640)
    renderer_name=GL.glGetString(GL.GL_RENDERER).decode()
    if args.software and not any(s in renderer_name.lower() for s in ('llvmpipe','softpipe','software')):
        renderer.close();raise RuntimeError('requested software renderer was not selected')
    option=mujoco.MjvOption();option.geomgroup[3]=1
    camera=mujoco.MjvCamera()
    data.qpos[:]=controls[0]['qpos'];mujoco.mj_forward(model,data)
    centre=data.body('xbox_controller_body').xpos.copy()
    camera.lookat[:]=centre+[0,0,.08]
    camera.distance=.42;camera.azimuth=135;camera.elevation=-25
    threshold=summary['controller_config']['digital_threshold']
    times=np.array([r['result_time_s'] for r in controls])
    sample_times=np.arange(times[0],times[-1]+1e-9,1/args.fps)
    selection=np.searchsorted(times,sample_times,side='left').clip(0,len(controls)-1)
    command=['ffmpeg','-loglevel','error','-f','rawvideo','-pix_fmt','rgb24','-s','980x480',
             '-r',str(args.fps),'-i','-','-c:v','libx264','-crf','21','-pix_fmt','yuv420p','-y',str(args.output/'replay.mp4')]
    with (args.output/'ffmpeg.log').open('w') as log:
        video=subprocess.Popen(command,stdin=subprocess.PIPE,stderr=log)
        try:
            for frame_number,index in enumerate(selection):
                row=controls[int(index)]
                data.qpos[:]=row['qpos'];data.qvel[:]=row['qvel'];data.time=row.get('physics_time_s',row['result_time_s'])
                mujoco.mj_forward(model,data)
                renderer.update_scene(data,camera=camera,scene_option=option)
                physical=Image.fromarray(renderer.render())
                image=Image.new('RGB',(980,480),'#182029');image.paste(physical,(0,0))
                draw=ImageDraw.Draw(image)
                tick=int(np.floor(row['result_time_s']*summary['logic_fps']+1e-8))-1
                board=summary['initial_game_state'] if tick<0 else games[min(tick,len(games)-1)]['game_state']
                cell=min(10,300//board['width'],250//board['height']);ox,oy=660,50
                draw.text((660,20),f"Actual game | score {board['score']}",fill='white')
                draw.rectangle((ox,oy,ox+board['width']*cell,oy+board['height']*cell),fill='#080f17')
                for j,(x,y) in enumerate(board['snake']):
                    draw.rectangle((ox+x*cell,oy+y*cell,ox+(x+1)*cell-1,oy+(y+1)*cell-1),fill='#f0df4b' if j==0 else '#4bc586')
                if board['food'] is not None:
                    x,y=board['food'];draw.rectangle((ox+x*cell,oy+y*cell,ox+(x+1)*cell-1,oy+(y+1)*cell-1),fill='#fa6675')
                draw.text((660,290),f"t = {row['result_time_s']:.2f} s",fill='white')
                draw.text((660,315),'        Up Down Left Right',fill='white')
                draw.text((660,338),'Target  '+'   '.join(str(int(v)) for v in row['target']),fill='#e8cb60')
                draw.text((660,360),'Actual  '+'   '.join(str(int(v>threshold)) for v in row['dpad']),fill='#65dfaf')
                draw.text((660,389),f"Contact peak: {row['max_contact_force_n']:.2f} N",fill='white')
                draw.text((660,415),'Recorded collision-geometry replay',fill='#bdc8d1')
                draw.text((660,435),'Recorded motion; no new rollout',fill='#bdc8d1')
                if frame_number==len(selection)//2:image.save(args.output/'preview.png')
                video.stdin.write(image.tobytes())
            video.stdin.close()
            if video.wait()!=0:raise RuntimeError('ffmpeg failed; inspect ffmpeg.log')
        finally:
            if video.poll() is None:video.terminate();video.wait()
            renderer.close()
    (args.output/'render.json').write_text(json.dumps(dict(renderer=renderer_name,frames=len(selection),fps=args.fps,
        source_control_indices=selection.tolist(),mode='recorded collision geometry and actual game state'),indent=2)+'\n')
    print(json.dumps(dict(renderer=renderer_name,frames=len(selection),video=str(args.output/'replay.mp4'))),flush=True)


if __name__=='__main__':
    main()
