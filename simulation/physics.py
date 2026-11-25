"""Closed-loop execution of native plans with finite forces and recorded states.

No planning or rendering is measured by stepping the physical simulation. All
robot qpos writes occur during initialization; replay belongs to render.py.
"""
import argparse
import hashlib
import json
import time
from pathlib import Path
from dataclasses import asdict,replace
import numpy as np
import mujoco
from scipy.spatial.transform import Rotation
from .scenes import build_scene,SCENES,ROOT
from .controllers import quadrotor_tracking_torque
from flask_repro import _core,native
from flask_repro.robot import get_robot
from flask_repro.planning import plan,validate,PlannerSettings,Trajectory
from flask_repro.flat_planner import World
from flask_repro.flat_models import Unicycle,Quad3D


def make_reference(meta,seed=0,budget=10,physics_model=None):
    start=np.array(meta['start']);goal=np.array(meta['goal']);robot=meta['robot']
    if robot in ('mobile','drone'):
        model=Unicycle(lift_speed=meta.get('planner_lift_speed',.2),velocity_max=meta.get('speed_limit_m_s',1.),angular_velocity_max=meta.get('yaw_rate_limit_rad_s',1.5)) if robot=='mobile' else Quad3D()
        dimensions=2 if robot=='mobile' else 3
        boxes=[(o['center'][:dimensions],o['size'][:dimensions]) for o in meta['obstacles']]
        original=World(*meta['bounds'],boxes=boxes,radius=meta['radius'])
        world=original.sphere_cover(.2);world.prepare_simd()
        settings=PlannerSettings(budget_s=budget,extension_range=.4 if robot=='mobile' else .8,rho=meta.get('planner_rho',1.),derivative_bound=meta.get('planner_derivative_bound',.4),validation_spacing=.008,validation_dt=.01)
        edges=[];measurements=[];current=start
        targets=[*meta.get('task_waypoints',[]),goal]
        for index,target in enumerate(targets):
            record=plan(model,world,current,np.asarray(target),settings,seed+index)
            if not record.solved:return None,dict(solved=False,plan_ns=record.plan_ns,settings=asdict(settings),rejections=record.rejections,failed_waypoint=index)
            checked=validate(record,model,original,settings)
            if not checked['valid']:raise RuntimeError(f'Independent reference validation failed: {checked}')
            edges.extend(record.trajectory.edges);measurements.append(dict(waypoint=index,plan_ns=record.plan_ns,postprocess_ns=record.postprocess_ns,validation=checked));current=np.asarray(target)
        trajectory=Trajectory(edges)
        return trajectory,dict(solved=True,plan_ns=sum(m['plan_ns'] for m in measurements),postprocess_ns=sum(m['postprocess_ns'] for m in measurements),seed=seed,settings=asdict(settings),measurements=measurements,edges=trajectory.to_records())
    env=_core.Environment()
    for obstacle in meta['obstacles']:
        # The robot's own support is physical geometry, not an obstacle to its base.
        if obstacle['name']=='pedestal':continue
        env.add_cuboid(obstacle['center'],[0,0,0],(np.array(obstacle['size'])/2).tolist())
    env.prepare()
    robot_model=get_robot(robot)
    if physics_model is not None:
        ranges=np.array([physics_model.joint(n).range for n in meta['joint_names']])
        robot_model=replace(robot_model,q_lo=np.maximum(robot_model.q_lo,ranges[:,0]),q_hi=np.minimum(robot_model.q_hi,ranges[:,1]))
    settings=native.settings(max_ns=int(budget*1e9),max_iterations=1000000,max_samples=200000)
    if meta.get('task_mode')=='pick_place':
        from .manipulation import task_waypoints
        from flask_repro.reference import Trajectory as JointTrajectory
        waypoints=task_waypoints(physics_model,meta);Q=[start];V=[np.zeros_like(start)];T=[];phases=[];measurements=[];clock=0.
        for index,waypoint in enumerate(waypoints):
            limits=native.joint_limits(robot_model,'strict_development')
            if waypoint['phase'] in ('lift','approach_place','place'):
                limits.max_tool_tilt_rad=.55;limits.max_tool_angular_speed=1.5
            goal=waypoint['configuration'];result=native.plan(robot_model,env,Q[-1],[goal],settings,seed+index,limits=limits)
            measurements.append(dict(phase=waypoint['phase'],solved=result['solved'],plan_ns=result['plan_ns'],postprocess_ns=result['shortcut_ns'],
                        iterations=result['iterations'],limit_rejections=result['limit_rejections'],collision_rejections=result['collision_rejections'],task_rejections=result['task_rejections'],
                        max_tool_tilt_rad=.55 if np.isfinite(limits.max_tool_tilt_rad) else None,max_tool_angular_speed_rad_s=1.5 if np.isfinite(limits.max_tool_angular_speed) else None))
            if not result['solved']:return None,dict(solved=False,plan_ns=sum(r['plan_ns'] for r in measurements),phases=measurements)
            trajectory=result['trajectory'];phases.append(dict(name=waypoint['phase'],start_s=clock,end_s=clock+trajectory.duration()));clock+=trajectory.duration()
            Q.extend(trajectory.Q[1:]);V.extend(trajectory.V[1:]);T.extend(trajectory.T)
            if waypoint['hold_s']:
                phases.append(dict(name='close_gripper' if waypoint['phase']=='pick' else 'open_gripper',start_s=clock,end_s=clock+waypoint['hold_s']))
                clock+=waypoint['hold_s'];Q.append(goal);V.append(np.zeros_like(goal));T.append(waypoint['hold_s'])
        trajectory=JointTrajectory(np.array(Q),np.array(V),np.array(T))
        return trajectory,dict(solved=True,plan_ns=sum(r['plan_ns'] for r in measurements),postprocess_ns=sum(r['postprocess_ns'] for r in measurements),seed=seed,
              positions=trajectory.Q.tolist(),velocities=trajectory.V.tolist(),durations=trajectory.T.tolist(),phases=phases,measurements=measurements,
              task_waypoints_source='declared_showcase_task_targets',validation_profile='strict_development')
    result=native.plan(robot_model,env,start,[goal],settings,seed)
    if not result['solved']:return None,dict(solved=False,plan_ns=result['plan_ns'])
    trajectory=result['trajectory']
    return trajectory,dict(solved=True,plan_ns=result['plan_ns'],postprocess_ns=result['shortcut_ns'],seed=seed,
         positions=trajectory.Q.tolist(),velocities=trajectory.V.tolist(),durations=trajectory.T.tolist())


def desired_attitude(acceleration,yaw):
    z=acceleration/np.linalg.norm(acceleration)
    heading=np.array([-np.sin(yaw),np.cos(yaw),0])
    x=np.cross(heading,z);x/=np.linalg.norm(x)
    return np.column_stack((x,np.cross(z,x),z))


def physical_contacts(model,data,meta):
    """Separate support contacts from forbidden robot/environment contacts."""
    forbidden=[];support=[]
    support_geoms=set(meta['support_geoms'])
    for c in data.contact[:data.ncon]:
        if c.dist>0:continue
        names=[model.geom(int(g)).name for g in (c.geom1,c.geom2)]
        bodies=[model.body(model.geom_bodyid[int(g)]).name for g in (c.geom1,c.geom2)]
        grasp_contact='task_object' in names and any('finger' in n or n.startswith('grasp_') for n in bodies)
        object_support='task_object' in names and any(n in ('pick_workbench','worktable','ground','left_bin','right_bin','sorting_bin_0','sorting_bin_1') for n in names)
        if grasp_contact or object_support:
            support.append(names)
        elif meta['robot']=='mobile' and 'ground' in names and any(n in support_geoms for n in names if n!='ground'):
            support.append(names)
        else:forbidden.append(dict(geoms=names,penetration_m=float(-c.dist)))
    return forbidden,support


def simulate(scene,condition='nominal',seed=0,output=None,budget=10,step=.001,settle=1.,task_mode='reach_transfer'):
    from flask_repro.verification_gate import require_verified_reproduction
    require_verified_reproduction('Showcase simulation')
    model,meta=build_scene(scene,condition,task_mode)
    if abs(.002/step-round(.002/step))>1e-8:raise ValueError('Physics step must divide the 2 ms control period')
    model.opt.timestep=step
    reference,planning=make_reference(meta,seed,budget,model)
    output=Path(output or ROOT/'results/showcase'/scene/condition);output.mkdir(parents=True,exist_ok=True)
    mujoco.mj_saveLastXML(str(output/'model.xml'),model)
    (output/'planning.json').write_text(json.dumps(planning,indent=2)+'\n')
    if reference is None:
        summary=dict(scene=scene,condition=condition,solved=False,completed=False,outcome='planning_failed',seed=seed)
        (output/'summary.json').write_text(json.dumps(summary,indent=2)+'\n');return summary
    robot=meta['robot'];data=mujoco.MjData(model)
    if robot=='mobile':
        x,y,theta=meta['start'];data.qpos[:7]=[x,y,meta.get('initial_body_height_m',.15),np.cos(theta/2),0,0,np.sin(theta/2)]
        model_fn=Unicycle(lift_speed=meta.get('planner_lift_speed',.2),velocity_max=meta.get('speed_limit_m_s',1.),angular_velocity_max=meta.get('yaw_rate_limit_rad_s',1.5))
        position_ids=np.arange(7);velocity_ids=np.arange(6)
    elif robot=='drone':
        start=np.array(meta['start']);data.qpos[:7]=np.r_[start[:3],start[6],start[3:6]]
        data.qvel[:6]=np.r_[start[7:10],start[10:13]]
        model_fn=Quad3D();position_ids=np.arange(7);velocity_ids=np.arange(6)
    else:
        position_ids=np.array([model.joint(n).qposadr[0] for n in meta['joint_names']])
        velocity_ids=np.array([model.joint(n).dofadr[0] for n in meta['joint_names']])
        data.qpos[position_ids]=meta['start']
        inverse=mujoco.MjData(model)
    if meta.get('gripper_joint_names'):
        for name in meta['gripper_joint_names']:data.qpos[model.joint(name).qposadr]=meta.get('gripper_open_position',0.)
    mujoco.mj_forward(model,data)
    duration=reference.duration if robot in ('mobile','drone') else reference.duration()
    motor_ids=np.array([model.actuator(n).id for n in meta['motor_names']])
    gripping=False;phase='track';object_positions=[];phase_log=[];grasp_lifted=False;tool_states=[]
    if meta.get('gripper_joint_names'):
        grip_q=np.array([model.joint(n).qposadr[0] for n in meta['gripper_joint_names']])
        grip_v=np.array([model.joint(n).dofadr[0] for n in meta['gripper_joint_names']])
        grip_u=np.array([model.actuator(n).id for n in meta['gripper_motor_names']])
    total=duration+settle
    samples=[];events=[dict(time_s=0,event='execution_started',measured_plan_ns=planning['plan_ns'])]
    ctrl=np.zeros(len(motor_ids));saturation=False;slip=0.;error=0.;target=np.array(meta['start']);reference_control=np.zeros(len(motor_ids))
    control_stride=round(.002/step);max_torque_ratio=0.;forbidden_frames=0;support_frames=0
    max_errors=[];start_wall=time.perf_counter()
    for iteration in range(int(np.ceil(total/step))+1):
        now=iteration*step
        # Kinematic/contact fields must describe the same timestamp as qpos/qvel.
        mujoco.mj_forward(model,data)
        if iteration%control_stride==0:
            t=min(now,duration)
            if task_mode=='pick_place':
                phase=next((p['name'] for p in planning['phases'] if p['start_s']<=t<p['end_s']),'settle')
                if phase=='close_gripper':gripping=True
                if phase=='open_gripper':gripping=False
            if meta.get('gripper_joint_names'):
                gripper_target=meta.get('gripper_closed_position',-.037) if gripping else meta.get('gripper_open_position',0.)
                data.ctrl[grip_u]=np.clip(500*(gripper_target-data.qpos[grip_q])-10*data.qvel[grip_v],-8,8)
            if robot in ('mobile','drone'):
                jet=reference.jet([t],4)[0];desired=model_fn.recover(jet)
            else:
                q,qd,qdd=reference.evaluate([t]);q=q[0];qd=qd[0];qdd=qdd[0]
                if now>duration:qd=np.zeros_like(qd);qdd=np.zeros_like(qdd)
            if robot=='mobile':
                target=desired['state'];v,w=desired['control']
                if now>duration:v=w=0.
                rotation=Rotation.from_quat(data.qpos[[4,5,6,3]])
                theta=rotation.as_euler('xyz')[2]
                delta=target[:2]-data.qpos[:2]
                forward=np.array([np.cos(theta),np.sin(theta)]);lateral=np.array([-np.sin(theta),np.cos(theta)])
                heading=np.arctan2(np.sin(target[2]-theta),np.cos(target[2]-theta))
                speed_limit=meta.get('speed_limit_m_s',1.);yaw_limit=meta.get('yaw_rate_limit_rad_s',1.5)
                commanded_v=np.clip(v*np.cos(heading)+1.5*delta@forward,-speed_limit,speed_limit)
                commanded_w=np.clip(w+3*heading+2*delta@lateral,-yaw_limit,yaw_limit)
                wheel_radius=meta['wheel_radius_m'];half_track=meta['half_track_m']
                wheel_velocity=np.array([(commanded_v-half_track*commanded_w)/wheel_radius,(commanded_v+half_track*commanded_w)/wheel_radius])
                measured=np.array([data.qvel[model.joint(n).dofadr[0]] for n in meta['wheel_joint_names']])
                ctrl=.02*(wheel_velocity-measured)
                reference_control=np.array([v,w]);slip=float(wheel_radius*measured.mean()-data.qvel[:2]@forward)
                error=float(np.linalg.norm(delta))
            elif robot=='drone':
                target=desired['position'];p=data.qpos[:3];v=data.qvel[:3]
                current_R=Rotation.from_quat(data.qpos[[4,5,6,3]]).as_matrix()
                acceleration=jet[2,:3]+4*(target[:3]-p)+3*(desired['velocity']-v)+[0,0,9.81]
                requested_R=desired_attitude(acceleration,jet[0,3])
                skew=requested_R.T@current_R-current_R.T@requested_R
                attitude_error=np.array([skew[2,1],skew[0,2],skew[1,0]])/2
                omega=data.qvel[3:6];J=np.diag([.1,.1,.2])
                torque=quadrotor_tracking_torque(current_R,desired['rotation'],omega,
                    desired['omega'],desired['omega_dot'],J,attitude_error)
                norm=np.linalg.norm(torque)
                if norm>2:torque*=2/norm
                thrust=max(0,acceleration@current_R[:,2])
                ctrl=np.r_[thrust,torque];reference_control=desired['control'];error=float(np.linalg.norm(target[:3]-p))
            else:
                target=q;inverse.qpos[:]=data.qpos;inverse.qvel[:]=data.qvel;inverse.qacc[:]=0
                inverse.qacc[velocity_ids]=qdd
                mujoco.mj_inverse(model,inverse)
                # Inverse dynamics feedforward plus finite joint-space feedback.
                inertia=np.empty((model.nv,model.nv));mujoco.mj_fullM(model,data,inertia)
                acceleration_feedback=100*(q-data.qpos[position_ids])+20*(qd-data.qvel[velocity_ids])
                ctrl=inverse.qfrc_inverse[velocity_ids]+inertia[np.ix_(velocity_ids,velocity_ids)]@acceleration_feedback
                reference_control=inverse.qfrc_inverse[velocity_ids].copy();error=float(np.linalg.norm(q-data.qpos[position_ids]))
                max_torque_ratio=max(max_torque_ratio,float(np.max(abs(ctrl)/np.array(meta['effort_limits']))))
            limits=model.actuator_ctrlrange[motor_ids]
            saturation=bool(np.any(ctrl<limits[:,0]) or np.any(ctrl>limits[:,1]))
            data.ctrl[motor_ids]=np.clip(ctrl,limits[:,0],limits[:,1])
            if robot=='ur5' and task_mode!='pick_place':
                # A scripted obstacle is an exogenous body, not robot state assignment.
                data.mocap_pos[0]=[1.3-.9*np.exp(-((now-total*.55)/(total*.12))**2),.45,1.3]
        contacts,support=physical_contacts(model,data,meta)
        forbidden_frames+=bool(contacts);support_frames+=bool(support);max_errors.append(error)
        if iteration%control_stride==0:
            if robot in ('mobile','drone'):actual_position=data.qpos[:3].copy()
            else:actual_position=data.body(meta['end_effector']).xpos.copy()
            samples.append((now,data.qpos.copy(),data.qvel.copy(),data.ctrl.copy(),data.qfrc_actuator.copy(),np.array(target),reference_control.copy(),error,int(saturation),len(contacts),len(support),slip,actual_position,data.mocap_pos.copy()))
            phase_log.append(phase)
            if task_mode=='pick_place':
                object_position=data.body('task_object_body').xpos.copy();object_positions.append(object_position)
                ee=data.body(meta['end_effector']);R=ee.xmat.reshape(3,3);expected=ee.xpos+R@meta['grasp_offset']
                grasp_error=float(np.linalg.norm(object_position-expected))
                tilt=float(np.arccos(np.clip(-R[2,2],-1,1)));angular_speed=float(np.linalg.norm(data.cvel[ee.id,:3]))
                tool_states.append([tilt,angular_speed,grasp_error])
                if gripping and object_position[2]>meta['object_start'][2]+.1 and grasp_error<.04:grasp_lifted=True
        if contacts and (not events or events[-1]['event']!='forbidden_contact'):
            events.append(dict(time_s=now,event='forbidden_contact',contacts=contacts))
        if iteration<int(np.ceil(total/step)):mujoco.mj_step(model,data)
        if not np.isfinite(data.qpos).all():raise RuntimeError('Nonfinite physical state')
    keys=['time_s','qpos','qvel','control','generalized_actuator_force','reference_state','reference_control','tracking_error','saturated','forbidden_contacts','support_contacts','wheel_slip_m_s','actual_position','mocap_position']
    arrays={k:np.asarray([s[i] for s in samples]) for i,k in enumerate(keys)}
    arrays['task_phase']=np.array(phase_log)
    if task_mode=='pick_place':
        arrays['task_object_position']=np.array(object_positions)
        arrays['tool_tilt_rad']=np.array(tool_states)[:,0];arrays['tool_angular_speed_rad_s']=np.array(tool_states)[:,1];arrays['grasp_error_m']=np.array(tool_states)[:,2]
    np.savez_compressed(output/'execution.npz',**arrays)
    (output/'events.json').write_text(json.dumps(events,indent=2)+'\n')
    # Reconstruction from the same scene code/configuration, including exact integrator settings.
    meta.update(seed=seed,physics_step_s=step,reference_duration_s=duration,record_step_s=.002,
                model_sha256=hashlib.sha256((ROOT/'simulation/scenes.py').read_bytes()).hexdigest(),
                controller_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
                controller_status='development_not_frozen',task_scope='navigation' if robot=='mobile' else 'flight' if robot=='drone' else 'contact_friction_pick_place_reset' if task_mode=='pick_place' else 'reach_transfer_no_grasp_yet')
    (output/'configuration.json').write_text(json.dumps(meta,indent=2)+'\n')
    position_error=float(arrays['tracking_error'][-1])
    tolerance=.08 if robot=='mobile' else .05
    summary=dict(scene=scene,condition=condition,seed=seed,solved=True,completed=position_error<tolerance and forbidden_frames==0,
            outcome='validated' if position_error<tolerance and forbidden_frames==0 else 'execution_failed',
            forbidden_contact_steps=forbidden_frames,support_contact_steps=support_frames,
            max_tracking_error=float(max(max_errors)),final_tracking_error=position_error,
            actuator_saturation_samples=int(arrays['saturated'].sum()),max_requested_torque_ratio=max_torque_ratio,
            duration_s=total,wall_s=time.perf_counter()-start_wall,physics_steps=len(max_errors),
            physical_execution=True,task_scope=meta['task_scope'],production_ready=False)
    if task_mode=='pick_place':
        placement_error=float(np.linalg.norm(arrays['task_object_position'][-1]-meta['object_goal']))
        carrying=np.isin(arrays['task_phase'],['lift','approach_place','place'])
        summary.update(grasp_lifted=grasp_lifted,placement_error_m=placement_error,
                 max_carrying_tool_tilt_rad=float(arrays['tool_tilt_rad'][carrying].max()),max_carrying_tool_angular_speed_rad_s=float(arrays['tool_angular_speed_rad_s'][carrying].max()),
                 max_carrying_grasp_error_m=float(arrays['grasp_error_m'][carrying].max()))
        summary['completed']=summary['completed'] and grasp_lifted and placement_error<.06
        summary['outcome']='validated' if summary['completed'] else 'execution_failed'
    (output/'summary.json').write_text(json.dumps(summary,indent=2)+'\n')
    return summary


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--scene',choices=SCENES,required=True)
    parser.add_argument('--condition',choices=['nominal','tight','recovery','backup'],default='nominal')
    parser.add_argument('--seed',type=int,default=0);parser.add_argument('--budget',type=float,default=10)
    parser.add_argument('--physics-step',type=float,default=.001);parser.add_argument('--output',type=Path)
    parser.add_argument('--task-mode',choices=['reach_transfer','pick_place'],default='reach_transfer')
    args=parser.parse_args();print(json.dumps(simulate(args.scene,args.condition,args.seed,args.output,args.budget,args.physics_step,task_mode=args.task_mode),indent=2))
if __name__=='__main__':main()
