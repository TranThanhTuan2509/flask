"""Measure framing, obstacle occlusion, and overlay overlap on recorded motion."""
import os
os.environ.setdefault('MUJOCO_GL','egl')
import argparse,json,hashlib
from pathlib import Path
import numpy as np
import mujoco
from .render import replay,camera
from .scenes import ROOT


def robot_geometry(model,meta):
    root=meta.get('body_name','panda_link0' if meta['robot']=='panda' else 'base_link')
    root_id=model.body(root).id
    descendants={root_id}
    for i in range(model.nbody):
        parent=i
        while parent>0:
            if parent in descendants:descendants.add(i);break
            parent=int(model.body_parentid[parent])
    return np.array([i for i in range(model.ngeom) if int(model.geom_bodyid[i]) in descendants and model.geom_group[i] not in (3,4,5)],int)


def rectangles(width,height,multiview=True):
    rectangles=[(int(.03*width),int(.045*height),int(.03*width+530*width/1920),int(.045*height+98*width/1920)),
      (int(.03*width),int(height-162*width/1920),int(.03*width+495*width/1920),int(height-50*width/1920))]
    if multiview:rectangles.append((int(.72*width),int(.055*height),int(.97*width),int(.305*height)))
    return rectangles


def mask_from_segmentation(seg,ids):
    return np.isin(seg[:,:,0],ids)&(seg[:,:,1]==int(mujoco.mjtObj.mjOBJ_GEOM))


def audit(run,preset,frames=41,width=640,height=360,return_images=False):
    run=Path(run);meta=json.loads((run/'configuration.json').read_text());log=np.load(run/'execution.npz')
    model=mujoco.MjModel.from_xml_path(str(run/'model.xml'));data=mujoco.MjData(model);renderer=mujoco.Renderer(model,height,width,max_geom=12000)
    renderer.enable_segmentation_rendering();ids=robot_geometry(model,meta);original_groups=model.geom_group.copy();nonrobot=np.array([i for i in range(model.ngeom) if i not in ids],int)
    options=mujoco.MjvOption();options.geomgroup[3:]=0
    rows=[];cam=camera(preset)
    try:
        for t in np.linspace(float(log['time_s'][0]),float(log['time_s'][-1]),frames):
            replay(model,data,log,float(t));model.geom_group[:]=original_groups
            renderer.update_scene(data,camera=cam,scene_option=options);mask=mask_from_segmentation(renderer.render(),ids)
            model.geom_group[nonrobot]=4
            renderer.update_scene(data,camera=cam,scene_option=options);unoccluded=mask_from_segmentation(renderer.render(),ids)
            pixels=int(mask.sum());total=int(unoccluded.sum());ys,xs=np.nonzero(unoccluded)
            cropped=not len(xs) or min(xs.min(),ys.min())<3 or xs.max()>width-4 or ys.max()>height-4
            bbox=[int(xs.min()),int(ys.min()),int(xs.max()),int(ys.max())] if len(xs) else None
            clean=np.zeros_like(unoccluded);multi=np.zeros_like(unoccluded)
            for target,mode in [(clean,False),(multi,True)]:
                for x0,y0,x1,y1 in rectangles(width,height,mode):target[y0:y1,x0:x1]=True
            rows.append(dict(time_s=float(t),visible_robot_pixels=pixels,unoccluded_robot_pixels=total,
                 environment_visibility_fraction=pixels/max(total,1),cropped=bool(cropped),bbox_pixels=bbox,
                 clean_overlay_robot_overlap_fraction=float(np.sum(mask&clean)/max(pixels,1)),
                 multiview_overlay_robot_overlap_fraction=float(np.sum(mask&multi)/max(pixels,1)),
                 robot_bbox_width_fraction=(bbox[2]-bbox[0])/width if bbox else 0.,robot_bbox_height_fraction=(bbox[3]-bbox[1])/height if bbox else 0.))
    finally:renderer.close();model.geom_group[:]=original_groups
    return dict(preset=preset,rows=rows,cropped_frames=sum(r['cropped'] for r in rows),
                minimum_visibility_fraction=min(r['environment_visibility_fraction'] for r in rows),
                mean_visibility_fraction=float(np.mean([r['environment_visibility_fraction'] for r in rows])),
                maximum_clean_overlay_overlap=max(r['clean_overlay_robot_overlap_fraction'] for r in rows),
                maximum_multiview_overlay_overlap=max(r['multiview_overlay_robot_overlap_fraction'] for r in rows),
                minimum_bbox_width_fraction=min(r['robot_bbox_width_fraction'] for r in rows),
                minimum_bbox_height_fraction=min(r['robot_bbox_height_fraction'] for r in rows))


def main():
    parser=argparse.ArgumentParser(description=__doc__);parser.add_argument('run',type=Path);parser.add_argument('--angles',type=float,nargs='*');parser.add_argument('--frames',type=int,default=41);parser.add_argument('--output',type=Path);args=parser.parse_args()
    meta=json.loads((args.run/'configuration.json').read_text());preset=meta['camera_presets']['primary'];results=[]
    for angle in args.angles or [preset[4]]:
        selected=preset.copy();selected[4]=angle;report=audit(args.run,selected,args.frames);results.append(report)
        print({k:v for k,v in report.items() if k!='rows'},flush=True)
    output=args.output or args.run/'camera_audit.json';output.write_text(json.dumps(dict(records_sha256=hashlib.sha256((args.run/'execution.npz').read_bytes()).hexdigest(),audit_code_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),candidates=results),indent=2)+'\n')
if __name__=='__main__':main()
