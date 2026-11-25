"""Render recorded physical states without advancing or changing the experiment.

Frames stream to FFmpeg; no uncompressed frame sequence is stored. Rendering is
an explicitly labeled state replay. Planning measurements are never recomputed.
"""
import argparse
import hashlib
import json
import os
import subprocess
from pathlib import Path
os.environ.setdefault('MUJOCO_GL','egl')
import mujoco
import numpy as np
from PIL import Image,ImageDraw,ImageFont
from .scenes import build_scene,ROOT

FONT='/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf'
BOLD='/usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf'

def camera(preset):
    cam=mujoco.MjvCamera();cam.lookat[:]=preset[:3];cam.distance=preset[3];cam.azimuth=preset[4];cam.elevation=preset[5]
    return cam

def replay(model,data,log,t):
    times=log['time_s'];i=min(max(0,np.searchsorted(times,t)-1),len(times)-2)
    f=np.clip((t-times[i])/(times[i+1]-times[i]),0,1)
    data.qpos[:]=(1-f)*log['qpos'][i]+f*log['qpos'][i+1]
    data.qvel[:]=(1-f)*log['qvel'][i]+f*log['qvel'][i+1]
    mujoco.mj_normalizeQuat(model,data.qpos)
    if model.nmocap:data.mocap_pos[:]=(1-f)*log['mocap_position'][i]+f*log['mocap_position'][i+1]
    data.time=t;mujoco.mj_forward(model,data)
    return i

def trace(scene,points,color,width=.009):
    for a,b in zip(points[:-1],points[1:]):
        if scene.ngeom>=scene.maxgeom:break
        g=scene.geoms[scene.ngeom];mujoco.mjv_initGeom(g,mujoco.mjtGeom.mjGEOM_CAPSULE,np.zeros(3),np.zeros(3),np.eye(3).ravel(),np.array(color,dtype=np.float32))
        mujoco.mjv_connector(g,mujoco.mjtGeom.mjGEOM_CAPSULE,width,a,b);scene.ngeom+=1

def overlay(rgb,meta,planning,log,i,t,layout,secondary=None):
    image=Image.fromarray(rgb);draw=ImageDraw.Draw(image,'RGBA');w,h=image.size;scale=w/1920
    small=ImageFont.truetype(FONT,max(12,int(21*scale)));large=ImageFont.truetype(BOLD,max(18,int(34*scale)))
    title=meta.get('title',meta['scene'].replace('_',' ').title())
    # Every panel is inset from the image edges. The footage occupies the entire
    # frame: no header/footer matte, letterbox band, or outside-frame telemetry.
    x=int(.03*w);y=int(.045*h);panel_w=int(530*scale);panel_h=int(98*scale)
    draw.rounded_rectangle((x,y,x+panel_w,y+panel_h),radius=int(14*scale),fill=(12,24,31,180))
    draw.rounded_rectangle((x+int(17*scale),y+int(20*scale),x+int(22*scale),y+int(76*scale)),radius=max(1,int(2*scale)),fill=(51,199,207,255))
    draw.text((x+int(37*scale),y+int(13*scale)),title,font=large,fill=(242,247,248))
    subtitle=f"FLASK  /  {meta.get('visual_model',meta['robot'].upper())}  /  {meta['condition']}"
    draw.text((x+int(37*scale),y+int(61*scale)),subtitle,font=ImageFont.truetype(FONT,max(10,int(17*scale))),fill=(186,209,219))
    speed=float(np.linalg.norm(log['qvel'][i,:2 if meta['robot']=='mobile' else 3])) if meta['robot'] in ('mobile','drone') else float(np.linalg.norm(log['qvel'][i]))
    units='m/s' if meta['robot'] in ('mobile','drone') else 'rad/s'
    error_units='m' if meta['robot'] in ('mobile','drone') else 'rad'
    status='CONTACT' if log['forbidden_contacts'][i] else 'SATURATED' if log['saturated'][i] else 'CLEAR'
    phase=str(log['task_phase'][i]).replace('_',' ').upper() if 'task_phase' in log and log['task_phase'][i]!='track' else 'TRACK' if t<meta['reference_duration_s'] else 'SETTLE'
    x=int(.03*w);y=h-int(162*scale);panel_w=int(495*scale);panel_h=int(112*scale)
    draw.rounded_rectangle((x,y,x+panel_w,y+panel_h),radius=int(14*scale),fill=(12,24,31,180))
    draw.text((x+int(20*scale),y+int(14*scale)),f'{t:05.2f} s   {phase}',font=ImageFont.truetype(BOLD,max(12,int(22*scale))),fill=(237,244,247))
    draw.text((x+int(20*scale),y+int(49*scale)),f'Speed  {speed:.2f} {units}     Error  {log["tracking_error"][i]:.4f} {error_units}',font=small,fill=(210,226,232))
    draw.text((x+int(20*scale),y+int(79*scale)),f'Plan  {planning["plan_ns"]/1e6:.2f} ms',font=ImageFont.truetype(FONT,max(11,int(18*scale))),fill=(174,200,211))
    draw.text((x+int(342*scale),y+int(78*scale)),status,font=ImageFont.truetype(BOLD,max(11,int(18*scale))),fill=(245,151,100) if status!='CLEAR' else (100,226,194))
    if layout=='multiview' and secondary is not None:
        inset_w=w//4;inset_h=int(inset_w*9/16);x=w-inset_w-int(.03*w);y=int(.055*h)
        image.paste(Image.fromarray(secondary).resize((inset_w,inset_h),Image.Resampling.LANCZOS),(x,y))
        draw=ImageDraw.Draw(image,'RGBA');draw.rectangle((x-2,y-2,x+inset_w+2,y+inset_h+2),outline=(220,232,238,230),width=max(1,int(2*scale)))
        label='SYNCHRONIZED SECONDARY VIEW';label_y=y+inset_h-int(26*scale)
        draw.rounded_rectangle((x+int(8*scale),label_y-int(3*scale),x+inset_w-int(8*scale),y+inset_h-int(7*scale)),radius=int(4*scale),fill=(12,24,31,170))
        draw.text((x+int(17*scale),label_y),label,font=ImageFont.truetype(BOLD,max(9,int(13*scale))),fill=(238,245,247))
    return np.asarray(image)


def render(run,width=1920,height=1080,fps=60,layouts=('clean','multiview'),preview=False,limit=None):
    from flask_repro.verification_gate import require_verified_reproduction
    require_verified_reproduction('Showcase rendering')
    run=Path(run);meta=json.loads((run/'configuration.json').read_text());planning=json.loads((run/'planning.json').read_text());summary=json.loads((run/'summary.json').read_text())
    if (run/'model.xml').exists():model=mujoco.MjModel.from_xml_path(str(run/'model.xml'))
    else:model,_=build_scene(meta['scene'],meta['condition'])
    data=mujoco.MjData(model);log=np.load(run/'execution.npz')
    model.vis.global_.offwidth=max(width,model.vis.global_.offwidth);model.vis.global_.offheight=max(height,model.vis.global_.offheight)
    renderer=mujoco.Renderer(model,height=height,width=width,max_geom=12000)
    secondary_renderer=mujoco.Renderer(model,height=height//4,width=width//4,max_geom=12000) if 'multiview' in layouts else None
    cameras={k:camera(v) for k,v in meta['camera_presets'].items()}
    options=mujoco.MjvOption()
    for group in meta.get('render_hidden_geom_groups',[3,4,5]):options.geomgroup[group]=0
    duration=float(log['time_s'][-1]);duration=min(duration,limit) if limit is not None else duration
    output=run/'media';output.mkdir(exist_ok=True)
    ff={};paths={}
    try:
        if not preview:
            for layout in layouts:
                path=output/f"{meta['scene']}_{meta['condition']}_{layout}_{width}x{height}_{fps}fps.mp4";paths[layout]=path
                ff[layout]=subprocess.Popen(['ffmpeg','-hide_banner','-loglevel','error','-y','-f','rawvideo','-pix_fmt','rgb24','-s',f'{width}x{height}','-r',str(fps),'-i','-','-an','-c:v','libx264','-threads','2','-preset','fast','-crf','18','-pix_fmt','yuv420p','-movflags','+faststart',str(path)],stdin=subprocess.PIPE)
        times=[duration*.5] if preview else np.arange(int(np.ceil(duration*fps))+1)/fps
        for index,t in enumerate(times):
            t=min(t,duration);i=replay(model,data,log,t)
            renderer.update_scene(data,camera=cameras['primary'],scene_option=options)
            points=log['actual_position'][:i+1:20].copy()
            if meta['robot']=='mobile':points[:,2]=.025
            trace(renderer.scene,points,[.12,.78,.84,.85],.009)
            rgb=renderer.render().copy();secondary=None
            if secondary_renderer:
                if meta.get('camera_follow',{}).get('secondary'):
                    follow=meta['camera_follow']['secondary'];position=log['actual_position'][i].copy()
                    if meta['robot']=='mobile':position[2]=follow['height']
                    cameras['secondary'].lookat[:]=position;cameras['secondary'].distance=follow['distance'];cameras['secondary'].azimuth=follow['azimuth'];cameras['secondary'].elevation=follow['elevation']
                secondary_renderer.update_scene(data,camera=cameras['secondary'],scene_option=options);secondary=secondary_renderer.render().copy()
            for layout in layouts:
                frame=overlay(rgb,meta,planning,log,i,t,layout,secondary)
                if preview:Image.fromarray(frame).save(output/f'preview_{layout}.png')
                else:ff[layout].stdin.write(np.ascontiguousarray(frame).tobytes())
            if index%max(1,5*fps)==0:print(f"{meta['scene']}: rendered {t:.2f}/{duration:.2f} s at {width}x{height}",flush=True)
    finally:
        for process in ff.values():
            process.stdin.close()
        codes=[p.wait() for p in ff.values()]
        renderer.close()
        if secondary_renderer:secondary_renderer.close()
    if any(codes):raise RuntimeError(f'FFmpeg failed: {codes}')
    manifest=dict(record_sha256=hashlib.sha256((run/'execution.npz').read_bytes()).hexdigest(),
        renderer_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),state_replay=True,
        physical_execution=summary['physical_execution'],production_ready=summary['production_ready'],width=width,height=height,fps=fps,
        duration_s=duration,artifacts={k:dict(file=v.name,sha256=hashlib.sha256(v.read_bytes()).hexdigest()) for k,v in paths.items()})
    (output/f'manifest_{width}x{height}_{fps}fps.json').write_text(json.dumps(manifest,indent=2)+'\n')
    return paths


def main():
    parser=argparse.ArgumentParser(description=__doc__);parser.add_argument('run',type=Path)
    parser.add_argument('--preset',choices=['master','delivery','preview'],default='preview');parser.add_argument('--limit',type=float)
    args=parser.parse_args();w,h,fps=(3840,2160,30) if args.preset=='master' else (1920,1080,60) if args.preset=='delivery' else (1280,720,30)
    render(args.run,w,h,fps,preview=args.preset=='preview',limit=args.limit)
if __name__=='__main__':main()
