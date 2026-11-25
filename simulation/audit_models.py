"""Audit source kinematics, mesh transforms, dimensions, and asset identities.

The URDF forward-kinematics oracle uses explicit homogeneous transforms. Visual
geometry is checked against independent source vertices, including COLLADA scene
nodes and millimeter-to-meter mesh scales. No renderer state is used as an oracle.
"""
import argparse,json,hashlib,subprocess
from pathlib import Path
import xml.etree.ElementTree as ET
import numpy as np
from scipy.spatial.transform import Rotation
import mujoco
from .scenes import ROOT,SCENES,build_scene
from .robot_assets import VAMP,ROBOTIS,MENAGERIE


def transform(position=None,rotation=None):
    T=np.eye(4)
    if position is not None:T[:3,3]=position
    if rotation is not None:T[:3,:3]=rotation
    return T

def values(node,name,default):return np.fromstring(node.get(name,default),sep=' ')

def urdf_fk(root,coordinates):
    links={n.get('name') for n in root.findall('link')};children={j.find('child').get('link') for j in root.findall('joint')}
    frames={name:np.eye(4) for name in links-children};axes={};pending=list(root.findall('joint'))
    while pending:
        progressed=False
        for joint in list(pending):
            parent=joint.find('parent').get('link');child=joint.find('child').get('link')
            if parent not in frames:continue
            origin=joint.find('origin');fixed=np.eye(4)
            if origin is not None:fixed=transform(values(origin,'xyz','0 0 0'),Rotation.from_euler('xyz',values(origin,'rpy','0 0 0')).as_matrix())
            kind=joint.get('type');motion=np.eye(4)
            if kind!='fixed':
                axis=values(joint.find('axis'),'xyz','1 0 0');q=coordinates.get(joint.get('name'),0.)
                axes[joint.get('name')]=(frames[parent]@fixed)[:3,:3]@axis
                motion=transform(axis*q) if kind=='prismatic' else transform(rotation=Rotation.from_rotvec(axis*q).as_matrix())
            frames[child]=frames[parent]@fixed@motion;pending.remove(joint);progressed=True
        if not progressed:raise ValueError('Disconnected or cyclic URDF joint graph')
    return frames,axes


def source_vertices(mesh,robot):
    path=mesh.get('filename').removeprefix('package://')
    source=ROOT/'third_party/turtlebot3'/path if path.startswith('turtlebot3_description/') else VAMP/robot/path
    scale=values(mesh,'scale','1 1 1')
    if source.suffix.lower()=='.dae':
        import collada
        c=collada.Collada(str(source));unit=c.assetInfo.unitmeter or 1.
        vertices=np.concatenate([p.vertex for g in c.scene.objects('geometry') for p in g.primitives()])*unit
    else:
        import trimesh
        vertices=trimesh.load(source,force='mesh',process=False).vertices
    return np.asarray(vertices)*scale


def compiled_vertices(model,data,indices):
    result=[]
    for index in indices:
        mesh=int(model.geom_dataid[index]);start=int(model.mesh_vertadr[mesh]);count=int(model.mesh_vertnum[mesh])
        points=model.mesh_vert[start:start+count]
        result.append(points@data.geom_xmat[index].reshape(3,3).T+data.geom_xpos[index])
    return np.concatenate(result)


def run(robot,samples=30):
    name=SCENES[('mobile','drone','panda','ur5').index(robot)]
    model,meta=build_scene(name,task_mode='pick_place' if robot in ('panda','ur5') else 'reach_transfer');data=mujoco.MjData(model)
    assets={};duplicate_names=[]
    for i in range(model.nmesh):
        mesh_name=model.mesh(i).name
        if mesh_name in assets:duplicate_names.append(mesh_name)
        assets[mesh_name]=i
    if duplicate_names:raise AssertionError(f'Duplicate mesh assets: {duplicate_names}')
    if not np.isfinite(model.body_inertia).all() or np.any(model.body_mass<0):raise AssertionError('Invalid inertial properties')
    mass_eigen=[]
    report=dict(robot=robot,visual_model=meta['visual_model'],samples=samples,mesh_count=model.nmesh,joint_count=model.njnt,
         dynamic_dofs=model.nv,unique_mesh_identifiers=True,maximum_frame_position_error_m=0.,maximum_frame_rotation_error_rad=0.,
         maximum_joint_axis_error=0.,maximum_visual_bounds_error_m=0.,source_frame_checks=0,source_visual_checks=0,
         mesh_bbox_m={model.mesh(i).name:[model.mesh_aabb[i,:3].tolist(),model.mesh_aabb[i,3:].tolist()] for i in range(model.nmesh)} if hasattr(model,'mesh_aabb') else {})
    # The only drone joint is a free rigid body. All visual/collision components
    # come from the original single-body source mesh; aggregate dynamics differs
    # explicitly from the stock Skydio model.
    if robot=='drone':
        source=mujoco.MjModel.from_xml_path(str(MENAGERIE/'skydio_x2/x2.xml'));reference=mujoco.MjData(source)
        report.update(source_model='third_party/mujoco_menagerie/skydio_x2/x2.xml',aggregate_dynamics_deviation=meta['asset_dynamics_deviation'])
        rng=np.random.default_rng(107)
        for _ in range(samples):
            q=np.r_[rng.uniform(-1,1,3),Rotation.random(random_state=rng).as_quat()[[3,0,1,2]]]
            data.qpos[:7]=q;reference.qpos[:7]=q;mujoco.mj_forward(model,data);mujoco.mj_forward(source,reference)
            ours=next(i for i in range(model.ngeom) if model.geom_type[i]==mujoco.mjtGeom.mjGEOM_MESH)
            original=next(i for i in range(source.ngeom) if source.geom_type[i]==mujoco.mjtGeom.mjGEOM_MESH)
            a=compiled_vertices(model,data,[ours]);b=compiled_vertices(source,reference,[original])
            error=float(np.max(abs(np.r_[a.min(0)-b.min(0),a.max(0)-b.max(0)])))
            report['maximum_visual_bounds_error_m']=max(report['maximum_visual_bounds_error_m'],error);report['source_visual_checks']+=1
        report['passed']=report['maximum_visual_bounds_error_m']<1e-6
        return report
    source=ROBOTIS/'urdf/turtlebot3_waffle_pi.urdf' if robot=='mobile' else VAMP/robot/f'{robot}.urdf'
    urdf=ET.fromstring(source.read_text().replace('${namespace}',''));report['source_model']=str(source.relative_to(ROOT))
    if robot=='mobile':joint_names=meta['wheel_joint_names'];home=np.zeros(2)
    else:joint_names=meta['joint_names'];home=np.array(meta['start'])
    rng=np.random.default_rng(319)
    visual_cache=[]
    if robot in ('mobile','ur5'):
        for link in urdf.findall('link'):
            for i,visual in enumerate(link.findall('visual')):
                mesh=visual.find('geometry/mesh')
                if mesh is None:continue
                points=source_vertices(mesh,robot);origin=visual.find('origin');T=np.eye(4)
                if origin is not None:T=transform(values(origin,'xyz','0 0 0'),Rotation.from_euler('xyz',values(origin,'rpy','0 0 0')).as_matrix())
                visual_cache.append((link.get('name'),i,points@T[:3,:3].T+T[:3,3]))
    for sample in range(samples):
        q=home+rng.uniform(-.55,.55,len(home))
        for j,name in enumerate(joint_names):
            joint=model.joint(name)
            if joint.limited:q[j]=np.clip(q[j],joint.range[0]+.01,joint.range[1]-.01)
            data.qpos[joint.qposadr]=q[j]
        if robot=='mobile':data.qpos[:7]=[0,0,.01,1,0,0,0]
        for joint in meta.get('gripper_joint_names',[]):data.qpos[model.joint(joint).qposadr]=meta.get('gripper_open_position',0.)
        mujoco.mj_forward(model,data);frames,axes=urdf_fk(urdf,dict(zip(joint_names,q)))
        for link,T in frames.items():
            if robot=='panda' and link not in [f'panda_link{i}' for i in range(8)]+['panda_hand']:continue
            try:body=data.body(link)
            except KeyError:
                if link in ('offset_link','base_footprint','panda_link0'):continue
                raise
            position_error=float(np.linalg.norm(body.xpos-T[:3,3]));rotation_error=float(Rotation.from_matrix(body.xmat.reshape(3,3).T@T[:3,:3]).magnitude())
            report['maximum_frame_position_error_m']=max(report['maximum_frame_position_error_m'],position_error)
            report['maximum_frame_rotation_error_rad']=max(report['maximum_frame_rotation_error_rad'],rotation_error);report['source_frame_checks']+=1
        for name,axis in axes.items():
            if name not in joint_names:continue
            error=float(np.linalg.norm(data.xaxis[model.joint(name).id]-axis));report['maximum_joint_axis_error']=max(report['maximum_joint_axis_error'],error)
        for link,i,points in visual_cache:
            prefix=f'{robot}_{link}_visual_{i:03d}'
            indices=[j for j in range(model.ngeom) if model.geom(j).name==prefix or model.geom(j).name.startswith(prefix+'_')]
            if not indices:raise AssertionError(f'Missing visual geometry: {prefix}')
            T=frames[link];expected=points@T[:3,:3].T+T[:3,3];actual=compiled_vertices(model,data,indices)
            error=float(np.max(abs(np.r_[expected.min(0)-actual.min(0),expected.max(0)-actual.max(0)])))
            report['maximum_visual_bounds_error_m']=max(report['maximum_visual_bounds_error_m'],error);report['source_visual_checks']+=1
        M=np.empty((model.nv,model.nv));mujoco.mj_fullM(model,data,M);mass_eigen.append(float(np.linalg.eigvalsh(M).min()))
    if robot=='panda':
        original=mujoco.MjModel.from_xml_path(str(MENAGERIE/'franka_emika_panda/panda.xml'));d=mujoco.MjData(original)
        for i in range(7):d.qpos[original.joint(f'joint{i+1}').qposadr]=data.qpos[model.joint(joint_names[i]).qposadr]
        for i,name in enumerate(('finger_joint1','finger_joint2')):d.qpos[original.joint(name).qposadr]=data.qpos[model.joint(meta['gripper_joint_names'][i]).qposadr]
        mujoco.mj_forward(original,d)
        for old,new in [('hand','panda_hand'),('left_finger','panda_leftfinger'),('right_finger','panda_rightfinger')]:
            error=float(np.max(abs(d.body(old).xpos-data.body(new).xpos)))
            if error>1e-6:raise AssertionError(f'Panda gripper-frame mismatch: {old}: {error}')
        report['gripper_source_frame_check']=True
        report['gripper_geometry_note']='Menagerie actuated Panda fingers use 0..0.04 m; the benchmark VAMP URDF has fixed 0.065 m fingers. This is an explicit showcase model difference.'
    report['minimum_mass_matrix_eigenvalue']=min(mass_eigen)
    report['passed']=max(report[k] for k in ('maximum_frame_position_error_m','maximum_frame_rotation_error_rad','maximum_joint_axis_error','maximum_visual_bounds_error_m'))<2e-6 and min(mass_eigen)>0
    return report


def main():
    parser=argparse.ArgumentParser(description=__doc__);parser.add_argument('--output',type=Path,default=ROOT/'results/validation/robot_asset_audit.json');parser.add_argument('--samples',type=int,default=30);args=parser.parse_args()
    reports=[]
    for robot in ('mobile','drone','panda','ur5'):
        try:report=run(robot,args.samples)
        except Exception as e:report=dict(robot=robot,passed=False,error=f'{type(e).__name__}: {e}')
        reports.append(report);print(robot,{k:v for k,v in report.items() if not k.startswith('mesh_bbox')},flush=True)
    args.output.parent.mkdir(parents=True,exist_ok=True);args.output.write_text(json.dumps(dict(reports=reports,all_passed=all(r['passed'] for r in reports),mujoco_version=mujoco.__version__,audit_code_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest()),indent=2)+'\n')
    if not all(r['passed'] for r in reports):raise SystemExit(1)
if __name__=='__main__':main()
