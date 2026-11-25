"""Compact industrial showcases built around audited upstream robot models.

These scenes are independent of the paper benchmark datasets. Native planning
uses their declared geometry; full meshes and physics validate actual execution.
"""
from pathlib import Path
import xml.etree.ElementTree as ET
import numpy as np
import mujoco
from .robot_assets import load_robot

ROOT=Path(__file__).resolve().parents[1]
RESOURCE=ROOT/'third_party/vamp/resources'
SCENES=('warehouse_navigation','aerial_gate_hall','cluttered_shelf_picking','reactive_sorting_cell')

def vector(a):return ' '.join(map(str,a))
def element(parent,kind,**attrs):return ET.SubElement(parent,kind,{k:vector(v) if isinstance(v,(list,tuple,np.ndarray)) else str(v) for k,v in attrs.items()})


def build_scene(scene,condition='nominal',task_mode='reach_transfer'):
    if scene not in SCENES:raise ValueError(scene)
    if condition not in ('nominal','tight','recovery','backup'):raise ValueError(condition)
    robot='mobile' if scene==SCENES[0] else 'drone' if scene==SCENES[1] else 'panda' if scene==SCENES[2] else 'ur5'
    root,asset_meta=load_robot(robot);root.set('model',scene)
    option=root.find('option')
    if option is not None:root.remove(option)
    element(root,'option',timestep='.001',gravity='0 0 -9.81',integrator='implicitfast',iterations='60',tolerance='1e-10')
    visual=root.find('visual')
    if visual is not None:root.remove(visual)
    visual=element(root,'visual');element(visual,'global',offwidth='3840',offheight='2160',fovy='38')
    element(visual,'quality',shadowsize='4096',offsamples='4')
    element(visual,'headlight',ambient='.25 .25 .25',diffuse='.4 .4 .4',specular='.15 .15 .15')
    element(visual,'rgba',haze='.75 .79 .8 1')
    asset=root.find('asset')
    if asset is None:asset=element(root,'asset')
    element(asset,'texture',name='showcase_sky',type='skybox',builtin='gradient',rgb1='.71 .77 .8',rgb2='.91 .93 .94',width='512',height='512')
    element(asset,'texture',name='showcase_floor_grid',type='2d',builtin='checker',rgb1='.37 .40 .42',rgb2='.39 .42 .44',width='512',height='512')
    element(asset,'material',name='showcase_floor',texture='showcase_floor_grid',texrepeat='20 20',reflectance='.015')
    element(asset,'material',name='showcase_metal',rgba='.55 .61 .64 1',specular='.3',shininess='.15')
    element(asset,'material',name='showcase_trim',rgba='.13 .24 .29 1',specular='.2',shininess='.1')
    element(asset,'material',name='showcase_obstacle',rgba='.78 .39 .17 1',specular='.2',shininess='.1')
    world=root.find('worldbody')
    element(world,'light',name='key_light',pos='1 -3 6',dir='-.1 .25 -1',diffuse='.72 .72 .68',castshadow='true',directional='true')
    element(world,'light',name='fill_light',pos='-3 2 4',dir='.4 -.2 -1',diffuse='.4 .43 .46',castshadow='false')
    element(world,'geom',name='ground',type='plane',size='5 5 .1',material='showcase_floor',friction='1 .01 .001',condim='4')
    actuator=root.find('actuator')
    meta=dict(scene=scene,condition=condition,robot=robot,physics_step_s=.001,control_step_s=.002,replanning_period_s=.05,
              source='independently_designed_showcase',obstacles=[],camera_presets={},render_hidden_geom_groups=[3,4,5],
              title=dict(mobile='Warehouse navigation',drone='Aerial gate traverse',panda='Shelf pick and place',ur5='Sorting around obstacles')[robot],
              hud=dict(title=[.03,.045],telemetry=[.03,.78],inset=[.72,.055,.25]))
    meta.update(asset_meta)
    def box(name,pos,size,color=None,material='showcase_metal'):
        attrs=dict(name=name,type='box',pos=pos,size=np.asarray(size)/2,material=material)
        if color is not None:attrs['rgba']=color
        element(world,'geom',**attrs)
        meta['obstacles'].append(dict(name=name,center=list(pos),size=list(size)))
    if robot=='mobile':
        gap=.58 if condition=='tight' else .72
        half_separation=(gap+.42)/2
        meta.update(start=[-1.55,-.9,0],goal=[1.55,.9,0],bounds=[[-2,-1.4],[2,1.4]],radius=.235,
                    task_waypoints=[[0,-1.,np.pi/2],[0,1.,np.pi/2]],
                    planner_rho=.004,planner_derivative_bound=.08,planner_lift_speed=.06)
        box('left_aisle',[-half_separation,.1,.32],[.42,1.5,.64])
        box('right_aisle',[half_separation,-.1,.32],[.42,1.5,.64])
        if condition=='backup':
            meta['obstacles']=meta['obstacles'][:1];world.remove(next(g for g in world.findall('geom') if g.get('name')=='right_aisle'))
        if condition=='recovery':box('closed_passage',[0,.3,.22],[gap,.16,.44])
        for i,(name,joint) in enumerate(zip(('left','right'),meta['wheel_joint_names'])):
            motor=name+'_wheel_motor';element(actuator,'motor',name=motor,joint=joint,ctrlrange=[-.35,.35],ctrllimited='true');meta['motor_names'].append(motor)
        meta['support_geoms']=['ground']+[g.get('name') for b in world.iter('body') if any(k in b.get('name','') for k in ('wheel','caster')) for g in b.findall('geom')]
        meta['camera_presets']=dict(primary=[0,0,.12,3.6,125,-65],secondary=[0,0,.1,3.8,90,-89],backup=[0,0,.12,3.6,-45,-55])
        meta['camera_follow']=dict(secondary=dict(distance=1.1,azimuth=145,elevation=-28,height=.12))
    elif robot=='drone':
        meta.update(start=[-2.15,-.65,.95,0,0,0,1,0,0,0,0,0,0],goal=[2.15,.65,1.95,0,0,0,1,0,0,0,0,0,0],bounds=[[-2.7,-1.7,.45],[2.7,1.7,2.8]],radius=.39)
        opening=1.1 if condition=='tight' else 1.4
        for index,(x,y,z) in enumerate([(-.8,-.25,1.25),(.8,.25,1.7)]):
            for sign in (-1,1):box(f'gate_{index}_post_{sign}',[x,y+sign*(opening/2+.045),1.35],[.09,.09,2.7],material='showcase_trim')
            box(f'gate_{index}_top',[x,y,z+opening/2+.045],[.09,opening+.18,.09],material='showcase_trim')
            box(f'gate_{index}_bottom',[x,y,z-opening/2-.045],[.09,opening+.18,.09],material='showcase_trim')
        if condition=='recovery':box('blocked_gate',[-.8,-.25,1.25],[.1,opening,opening])
        body=next(b for b in world.findall('body') if b.get('name')=='airframe')
        element(body,'site',name='aggregate_thrust_frame',size='.005',rgba='0 0 0 0',group='5')
        for name,gear,limit in [('thrust',[0,0,1,0,0,0],[0,14.715]),('roll',[0,0,0,1,0,0],[-2,2]),('pitch',[0,0,0,0,1,0],[-2,2]),('yaw',[0,0,0,0,0,1],[-2,2])]:
            element(actuator,'general',name=name,site='aggregate_thrust_frame',gear=gear,ctrlrange=limit,ctrllimited='true');meta['motor_names'].append(name)
        meta['support_geoms']=['ground'];meta['camera_presets']=dict(primary=[0,0,1.35,5.4,135,-20],secondary=[0,0,1.35,4.9,100,-4],backup=[0,0,1.35,5.4,-45,-20])
        meta['camera_follow']=dict(secondary=dict(distance=1.6,azimuth=150,elevation=-12,height=0))
    else:
        joints=[f'panda_joint{i}' for i in range(1,8)] if robot=='panda' else ['shoulder_pan_joint','shoulder_lift_joint','elbow_joint','wrist_1_joint','wrist_2_joint','wrist_3_joint']
        meta['joint_names']=joints;meta['effort_limits']=[87,87,87,87,12,12,12] if robot=='panda' else [150,150,150,28,28,28]
        for name,limit in zip(joints,meta['effort_limits']):
            motor=name+'_motor';element(actuator,'motor',name=motor,joint=name,ctrlrange=[-limit,limit],ctrllimited='true');meta['motor_names'].append(motor)
        contact=root.find('contact')
        if contact is None:contact=element(root,'contact')
        exclusions={tuple(sorted((node.get('body1'),node.get('body2')))) for node in contact}
        for parent in world.iter('body'):
            for child in parent.findall('body'):
                key=tuple(sorted((parent.get('name'),child.get('name'))))
                if key not in exclusions:element(contact,'exclude',body1=parent.get('name'),body2=child.get('name'));exclusions.add(key)
        if robot=='panda':
            meta.update(start=[0,-.65,0,-2.1,0,1.55,.75],goal=[.9,-.3,-.2,-1.8,.2,1.55,.7],radius=0,end_effector='panda_hand')
            box('shelf_back',[.45,.75,.67],[1.1,.04,.55])
            roof=.87 if condition=='tight' else .97
            box('shelf_roof',[.45,.55,roof],[1.1,.4,.035])
            for sign in (-1,1):box(f'shelf_upright_{sign}',[.45+sign*.55,.55,.7],[.035,.4,.55])
            box('left_bin',[-.35,.35,.15],[.22,.25,.3],material='showcase_trim')
            box('right_bin',[.5,-.4,.15],[.22,.25,.3],material='showcase_trim')
            box('transfer_obstacle',[.695,.05,.375],[.12,.12,.75],material='showcase_obstacle')
            meta['planning_obstacle_highlights']=['transfer_obstacle','shelf_roof','shelf_upright_-1','shelf_upright_1']
            for motor,joint in zip(meta['gripper_motor_names'],meta['gripper_joint_names']):element(actuator,'motor',name=motor,joint=joint,ctrlrange='-8 8',ctrllimited='true')
            meta['camera_presets']=dict(primary=[.25,.08,.55,2.1,135,-22],secondary=[.25,.05,.45,2.,90,-85],backup=[.25,.08,.55,2.1,-35,-20])
        else:
            meta.update(start=[0,-1.3,1.5,-1.8,-1.57,0],goal=[.7,-1.2,1.6,-1.9,-1.57,0],radius=0,end_effector='robotiq_85_base_link')
            box('pedestal',[0,0,.4557],[.21,.21,.9114],material='showcase_trim')
            box('worktable',[0,.63,.83],[1.5,.65,.08])
            for index,x in enumerate([-.5,.5]):box(f'sorting_bin_{index}',[x,.65,.94],[.24,.22,.14],material='showcase_trim')
            box('transfer_obstacle',[.11,.565,1.01],[.08,.08,.28],material='showcase_obstacle')
            meta['planning_obstacle_highlights']=['transfer_obstacle']
            obstacle=element(world,'body',name='crossing_obstacle',mocap='true',pos='1.1 .45 1.3')
            element(obstacle,'geom',name='moving_obstacle',type='sphere',size='.1',rgba='.78 .25 .12 1')
            meta['camera_presets']=dict(primary=[0,.38,1.15,2.55,145,-24],secondary=[0,.4,1,2.35,90,-85],backup=[0,.38,1.15,2.55,-30,-22])
        meta['support_geoms']=['ground']
    if task_mode=='pick_place':
        if robot not in ('panda','ur5'):raise ValueError('Pick-place requires an arm')
        from .manipulation import augment_scene
        augment_scene(root,meta)
    model=mujoco.MjModel.from_xml_string(ET.tostring(root,encoding='unicode'))
    return model,meta
