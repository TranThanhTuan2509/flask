"""Pinned upstream robot assets with explicit frame-preserving conversion.

UR5 remains UR5; the visually similar UR5e is never substituted. COLLADA visual
meshes are converted with their scene transforms rather than replaced by collision
STLs. Upstream files and licenses are left unchanged.
"""
from pathlib import Path
import hashlib,json,re
from copy import deepcopy
import xml.etree.ElementTree as ET
import numpy as np
import mujoco

ROOT=Path(__file__).resolve().parents[1]
MENAGERIE=ROOT/'third_party/mujoco_menagerie'
VAMP=ROOT/'third_party/vamp/resources'
ROBOTIS=ROOT/'third_party/turtlebot3/turtlebot3_description'
CACHE=ROOT/'assets/converted_meshes'


def add(parent,kind,**attributes):
    return ET.SubElement(parent,kind,{k:' '.join(map(str,v)) if isinstance(v,(list,tuple,np.ndarray)) else str(v) for k,v in attributes.items()})


def collada_to_obj(source):
    import trimesh
    data=source.read_bytes();folder=CACHE/(hashlib.sha256(data).hexdigest()[:16]+'_'+source.stem)
    manifest=folder/'conversion.json'
    if manifest.exists():return folder/'visual.obj'
    folder.mkdir(parents=True,exist_ok=True)
    scene=trimesh.load(source,force='scene',process=False)
    # A scene export applies every COLLADA node transform and preserves distinct
    # materials. MuJoCo imports OBJ geometry; each material is retained separately
    # below when the URDF visual is expanded into its constituent submeshes.
    mesh=scene.to_geometry()
    mesh.export(folder/'visual.obj',include_texture=False)
    manifest.write_text(json.dumps(dict(source=str(source.relative_to(ROOT)),source_sha256=hashlib.sha256(data).hexdigest(),
         tool='trimesh',version=trimesh.__version__,vertex_count=len(mesh.vertices),face_count=len(mesh.faces),bounds_m=mesh.bounds.tolist(),scene_transforms_applied=True),indent=2)+'\n')
    return folder/'visual.obj'


def collada_parts(source):
    import trimesh,collada
    data=source.read_bytes();source_hash=hashlib.sha256(data).hexdigest();folder=CACHE/(source_hash[:16]+'_'+source.stem+'_materials_v2')
    manifest=folder/'conversion.json'
    if manifest.exists():  # cached parts: re-root files in this cache folder (caches may come from another machine)
        return [dict(part,file=str(folder/Path(part['file']).name)) for part in json.loads(manifest.read_text())['parts']]
    folder.mkdir(parents=True,exist_ok=True);scene=trimesh.load(source,force='scene',process=False)
    unit=collada.Collada(str(source),ignore=[collada.common.DaeBrokenRefError,collada.common.DaeUnsupportedError]).assetInfo.unitmeter or 1.;parts=[]
    for index,name in enumerate(scene.graph.nodes_geometry):
        transform,geometry=scene.graph[name];mesh=scene.geometry[geometry].copy()
        material=mesh.visual.material
        color=getattr(material,'baseColorFactor',None)
        if color is None:color=getattr(material,'diffuse',np.array([180,180,180,255]))
        color=np.asarray(color,float)
        if color.max()>1:color/=255
        mesh.apply_transform(transform);mesh.apply_scale(unit)
        file=folder/f'{source.stem}_{source_hash[:8]}_visual_{index:03d}.obj';mesh.export(file,include_texture=False)
        parts.append(dict(file=str(file.resolve().relative_to(ROOT)) if ROOT in file.resolve().parents else file.name,rgba=color.tolist(),bounds_m=mesh.bounds.tolist(),scene_transform=transform.tolist()))
    manifest.write_text(json.dumps(dict(source=str(source.relative_to(ROOT)),source_sha256=hashlib.sha256(data).hexdigest(),tool='trimesh',version=trimesh.__version__,
             unit_scale_m=unit,scene_transforms_applied=True,parts=parts),indent=2)+'\n')
    return parts


def urdf_root(robot):
    source=ROBOTIS/'urdf/turtlebot3_waffle_pi.urdf' if robot=='mobile' else VAMP/robot/f'{robot}.urdf'
    text=source.read_text().replace('${namespace}','')
    root=ET.fromstring(text)
    for parent in root.iter():
        for node in list(parent):
            if node.tag.startswith('{'):parent.remove(node)
    for link in root.findall('link'):
        for index,visual in enumerate(list(link.findall('visual'))):
            mesh=visual.find('geometry/mesh')
            if mesh is None:continue
            path=mesh.get('filename').removeprefix('package://')
            source_mesh=VAMP/robot/path
            if source_mesh.suffix.lower()!='.dae':
                visual.set('name',f'{robot}_{link.get("name")}_visual_{index:03d}')
                continue
            link.remove(visual)
            for part_index,part in enumerate(collada_parts(source_mesh)):
                copy=deepcopy(visual);name=f'{robot}_{link.get("name")}_visual_{index:03d}_{part_index:03d}'
                copy.set('name',name);copy.find('geometry/mesh').set('filename',part['file'])
                old=copy.find('material')
                if old is not None:copy.remove(old)
                add(add(copy,'material',name=name+'_material'),'color',rgba=part['rgba'])
                link.append(copy)
        for index,node in enumerate(link.findall('collision')):node.set('name',f'{robot}_{link.get("name")}_collision_{index:03d}')
    for mesh in root.iter('mesh'):
        path=mesh.get('filename').removeprefix('package://')
        target=Path(path) if Path(path).is_absolute() else ROOT/'third_party/turtlebot3'/path if path.startswith('turtlebot3_description/') else VAMP/robot/path
        if target.suffix.lower()=='.dae':target=collada_to_obj(target)
        if not target.is_file():raise FileNotFoundError(target)
        mesh.set('filename',str(target.resolve()))
    identities={}
    for mesh in root.iter('mesh'):
        path=Path(mesh.get('filename'));key=path.stem
        if key in identities and identities[key]!=path:
            raise ValueError(f'URDF mesh basename collision: {identities[key]} versus {path}')
        identities[key]=path
    # fetch.urdf declares non-positive-definite finger inertias: replace exactly those by a small diagonal
    for inertia in root.iter('inertia'):
        keys=['ixx','ixy','ixz','iyy','iyz','izz']
        if all(k in inertia.attrib for k in keys):
            a=[float(inertia.get(k)) for k in keys]
            M=np.array([[a[0],a[1],a[2]],[a[1],a[3],a[4]],[a[2],a[4],a[5]]])
            if np.linalg.eigvalsh(M).min()<=0:
                d=max(1e-6,float(np.abs(np.diag(M)).max()))
                for k,v in zip(keys,[d,0,0,d,0,d]):inertia.set(k,repr(v))
    mj=add(root,'mujoco');add(mj,'compiler',discardvisual='false',fusestatic='false',balanceinertia='true')
    spec=mujoco.MjSpec.from_string(ET.tostring(root,encoding='unicode'))
    xml=ET.fromstring(spec.to_xml())
    # A URDF's duplicate collision hull must not be rendered over its visual mesh.
    for geom in xml.iter('geom'):
        if geom.get('contype','1')!='0' or geom.get('conaffinity','1')!='0':geom.set('group','3')
        else:geom.set('group','2')
    return xml


def menagerie_root(folder,filename):
    source=MENAGERIE/folder
    xml=ET.parse(source/filename).getroot()
    compiler=xml.find('compiler')
    directory=compiler.get('assetdir',compiler.get('meshdir','assets'))
    for node in xml.find('asset'):
        if 'file' in node.attrib:node.set('file',str((source/directory/node.get('file')).resolve()))
    for name in ('assetdir','meshdir','texturedir'):compiler.attrib.pop(name,None)
    # Cached keyframe actuator vectors become invalid when torque control replaces
    # source position actuators. Initial states are explicitly set by simulation.
    for tag in ('keyframe','actuator','sensor'):
        node=xml.find(tag)
        if node is not None:xml.remove(node)
    return xml


def load_robot(robot):
    meta=dict(asset_profile='upstream_mesh',motor_names=[])
    if robot=='panda':
        root=menagerie_root('franka_emika_panda','panda.xml')
        names={f'link{i}':f'panda_link{i}' for i in range(8)}
        names.update({f'joint{i}':f'panda_joint{i}' for i in range(1,8)})
        names.update(hand='panda_hand',left_finger='panda_leftfinger',right_finger='panda_rightfinger',finger_joint1='grasp_joint_left',finger_joint2='grasp_joint_right')
        for node in root.iter():
            for key,value in list(node.attrib.items()):
                if key in ('name','joint','joint1','joint2','body1','body2','body','objname') and value in names:node.set(key,names[value])
        for tag in ('tendon','equality'):
            node=root.find(tag)
            if node is not None:root.remove(node)
        meta.update(upstream_gripper=True,gripper_joint_names=['grasp_joint_left','grasp_joint_right'],gripper_motor_names=['grasp_motor_left','grasp_motor_right'],
          gripper_open_position=.04,gripper_closed_position=.019,visual_model='Franka Emika Panda',dynamics_source='MuJoCo Menagerie Panda inertias and collision geometry')
    elif robot=='ur5':
        root=urdf_root('ur5');meta.update(visual_model='Universal Robots UR5',dynamics_source='Pinned VAMP UR5 URDF inertias',visual_conversion='COLLADA scene to OBJ; original joint and mesh frames preserved')
    elif robot=='mobile':
        root=urdf_root('mobile');world=root.find('worldbody');base=next(b for b in world.iter('body') if b.get('name')=='base_link')
        parent=next(p for p in world.iter() if base in list(p))
        if parent is not world:
            if parent.get('pos','0 0 0')!='0 0 0' or parent.find('joint') is not None:
                raise ValueError('Unexpected TurtleBot root-frame transform')
            parent.remove(base);world.remove(parent);world.append(base)
        add(base,'freejoint',name='mobile_free')
        meta.update(visual_model='TurtleBot3 Waffle Pi',dynamics_source='ROBOTIS TurtleBot3 URDF',wheel_radius_m=.033,half_track_m=.144,
              wheel_joint_names=['wheel_left_joint','wheel_right_joint'],wheel_torque_limit_nm=.35,initial_body_height_m=.01,
              speed_limit_m_s=.26,yaw_rate_limit_rad_s=1.5,body_name='base_link')
        for joint in root.iter('joint'):
            if joint.get('name') in meta['wheel_joint_names']:joint.set('damping','.001')
        # Explicit source caster contacts; low lateral friction represents a caster,
        # not a lateral steering actuator or a kinematic no-slip constraint.
        for body in world.iter('body'):
            if 'caster' in body.get('name',''):
                for geom in body.findall('geom'):geom.set('friction','.03 .001 .0001')
    elif robot=='drone':
        root=menagerie_root('skydio_x2','x2.xml');body=root.find('worldbody/body');body.set('name','airframe')
        old=body.find('inertial')
        if old is not None:body.remove(old)
        add(body,'inertial',mass='1',diaginertia='.1 .1 .2',pos='0 0 0')
        for node in list(body):
            if node.tag=='camera':body.remove(node)
        free=body.find('freejoint');free.set('name','aerial_free')
        meta.update(visual_model='Skydio X2 mesh',dynamics_source='Paper aggregate model: mass 1 kg; inertia diag(0.1,0.1,0.2) kg m^2',
           asset_dynamics_deviation='Skydio visuals/collision bodies with explicitly reconstructed paper aggregate thrust/body torque actuation; not the stock Skydio rotor model',body_name='airframe')
    else:raise ValueError(robot)
    if root.find('actuator') is None:add(root,'actuator')
    # Give every physical geom a stable English identifier for contact records.
    for index,node in enumerate(root.find('worldbody').iter('geom')):
        if 'name' not in node.attrib:node.set('name',f'{robot}_geometry_{index:03d}')
    return root,meta
