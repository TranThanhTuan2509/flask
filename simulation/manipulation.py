"""Physical parallel-jaw grasping and ordered native planning tasks.

The added grasp pads are a disclosed showcase gripper model. Contact and friction
carry the object; no weld, teleport, or kinematic object attachment is used.
"""
import xml.etree.ElementTree as ET
import numpy as np
import mujoco
from scipy.spatial.transform import Rotation


def add(parent,kind,**attributes):
    return ET.SubElement(parent,kind,{k:' '.join(map(str,v)) if isinstance(v,(list,tuple,np.ndarray)) else str(v) for k,v in attributes.items()})


def augment_scene(root,meta):
    world=root.find('worldbody');actuator=root.find('actuator');robot=meta['robot'];ee=meta['end_effector']
    upstream_gripper=meta.get('upstream_gripper',False)
    source_joints=meta.get('gripper_joint_names',[]);source_motors=meta.get('gripper_motor_names',[])
    meta.update(task_mode='pick_place',gripper_model='finite_force_parallel_jaw_contact_friction',
                gripper_motor_names=source_motors if upstream_gripper else [],gripper_joint_names=source_joints if upstream_gripper else [],object_mass_kg=.05)
    if robot=='panda':
        meta.update(object_start=[.42,.4,.465],object_goal=[.5,-.4,.325],grasp_offset=[0,0,.1])
        add(world,'geom',name='pick_workbench',type='box',pos='.45 .55 .42',size='.55 .2 .02',rgba='.45 .5 .52 1')
        meta['obstacles'].append(dict(name='pick_workbench',center=[.45,.55,.42],size=[1.1,.4,.04]))
    else:meta.update(object_start=[.3,.45,.895],object_goal=[-.3,.45,.895],grasp_offset=[0,0,.15])
    ee_body=next(b for b in world.iter('body') if b.get('name')==ee)
    old_gripper_bodies=[b.get('name') for b in ee_body.iter('body')]
    for sign,label in [(1,'left'),(-1,'right')]:
        if upstream_gripper:continue
        if robot=='panda':
            body=next(b for b in world.iter('body') if b.get('name')==f'panda_{label}finger')
            pad_pos=[0,0,.025]
        else:
            body=add(ee_body,'body',name=f'grasp_finger_{label}',pos=[0,sign*.065,.12]);pad_pos=[0,0,.02]
        joint=f'grasp_joint_{label}';motor=f'grasp_motor_{label}'
        add(body,'joint',name=joint,type='slide',axis=[0,sign,0],range='-.05 0',limited='true',damping='1')
        add(body,'geom',name=f'grasp_pad_{label}',type='box',pos=pad_pos,size='.018 .011 .025',mass='.04',friction='2 .02 .002',rgba='.08 .12 .14 1')
        add(actuator,'motor',name=motor,joint=joint,ctrlrange='-8 8',ctrllimited='true')
        meta['gripper_joint_names'].append(joint);meta['gripper_motor_names'].append(motor)
        contact=root.find('contact')
        if contact is None:contact=add(root,'contact')
        # Mechanically adjoining gripper components need the same collision
        # exclusions as neighboring arm links. Other arm links stay collidable.
        for name in old_gripper_bodies:
            if name!=body.get('name'):add(contact,'exclude',body1=body.get('name'),body2=name)
    body=add(world,'body',name='task_object_body',pos=meta['object_start'])
    add(body,'freejoint',name='task_object_free')
    add(body,'geom',name='task_object',type='box',size='.025 .025 .025',mass='.05',friction='2 .02 .002',rgba='.93 .62 .17 1')
    meta['intentional_contact_geoms']=['task_object','grasp_pad_left','grasp_pad_right']


def inverse_kinematics(model,meta,position,rotation,seed):
    """World-frame damped least squares; the executed data is never modified."""
    data=mujoco.MjData(model)
    qp=np.array([model.joint(n).qposadr[0] for n in meta['joint_names']])
    dv=np.array([model.joint(n).dofadr[0] for n in meta['joint_names']])
    data.qpos[qp]=seed
    for _ in range(500):
        mujoco.mj_forward(model,data);body=data.body(meta['end_effector']);R=body.xmat.reshape(3,3)
        error=np.r_[np.array(position)-body.xpos,Rotation.from_matrix(rotation@R.T).as_rotvec()]
        if np.linalg.norm(error[:3])<2e-5 and np.linalg.norm(error[3:])<1e-4:return data.qpos[qp].copy()
        jp=np.zeros((3,model.nv));jr=np.zeros_like(jp);mujoco.mj_jacBody(model,data,jp,jr,body.id)
        J=np.r_[jp[:,dv],jr[:,dv]];increment=J.T@np.linalg.solve(J@J.T+1e-5*np.eye(6),error)
        scale=min(1.,.15/max(np.linalg.norm(increment),1e-12));data.qpos[qp]+=scale*increment
        for i,j in enumerate(qp):
            joint=model.joint(meta['joint_names'][i]);data.qpos[j]=np.clip(data.qpos[j],joint.range[0]+.005,joint.range[1]-.005)
    raise RuntimeError(f'Inverse kinematics did not converge at {position}; residual {np.linalg.norm(error)}')


def task_waypoints(model,meta):
    data=mujoco.MjData(model)
    ids=[model.joint(n).qposadr[0] for n in meta['joint_names']]
    data.qpos[ids]=meta['start'];mujoco.mj_forward(model,data)
    rotation=data.body(meta['end_effector']).xmat.reshape(3,3).copy();offset=rotation@np.array(meta['grasp_offset'])
    pick=np.array(meta['object_start'])-offset;place=np.array(meta['object_goal'])-offset
    if meta['robot']=='ur5':place=place+[0,0,.035]
    q=np.array(meta['start']);waypoints=[]
    lift_height=.15 if meta['robot']=='panda' else .2
    for phase,point,hold in [('approach_pick',pick+[0,0,.15],0),('pick',pick,.7),('lift',pick+[0,0,lift_height],0),('approach_place',place+[0,0,.2],0),('place',place,.7),('retreat',place+[0,0,.2],0)]:
        q=inverse_kinematics(model,meta,point,rotation,q);waypoints.append(dict(phase=phase,configuration=q.copy(),hold_s=hold))
    waypoints.append(dict(phase='reset',configuration=np.array(meta['start']),hold_s=0))
    return waypoints
