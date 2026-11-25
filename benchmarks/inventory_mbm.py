"""Inventory every supplied Panda pair and audit source-invalid endpoints.

The independent environment check uses URDF transforms and NumPy distances,
not generated VAMP FK/collision code. It checks the same sphere approximation;
it is not a mesh-level or continuous collision certificate. Self-collision
classification remains supplied by VAMP and is reported separately.
"""
import argparse
import json
from pathlib import Path
import xml.etree.ElementTree as ET

import numpy as np
from scipy.spatial.transform import Rotation
import vamp

from flask_repro import mbm, native, robot
from flask_repro.provenance import save_manifest


class SphereOracle:
    def __init__(self,path):
        root=ET.parse(path).getroot()
        self.joints=[]
        for joint in root.findall('joint'):
            origin=joint.find('origin');transform=np.eye(4)
            if origin is not None:
                transform[:3,3]=np.fromstring(origin.get('xyz','0 0 0'),sep=' ')
                transform[:3,:3]=Rotation.from_euler('xyz',np.fromstring(origin.get('rpy','0 0 0'),sep=' ')).as_matrix()
            axis=joint.find('axis')
            self.joints.append(dict(name=joint.get('name'),kind=joint.get('type'),
                parent=joint.find('parent').get('link'),child=joint.find('child').get('link'),
                transform=transform,axis=np.fromstring(axis.get('xyz','1 0 0'),sep=' ') if axis is not None else np.array([1.,0.,0.])))
        children={j['child'] for j in self.joints}
        self.roots={link.get('name') for link in root.findall('link')}-children
        self.spheres=[]
        for link in root.findall('link'):
            for collision in link.findall('collision'):
                sphere=collision.find('geometry/sphere')
                if sphere is None:continue
                origin=collision.find('origin')
                position=np.fromstring(origin.get('xyz','0 0 0'),sep=' ') if origin is not None else np.zeros(3)
                self.spheres.append((link.get('name'),position,float(sphere.get('radius'))))

    def forward(self,coordinates):
        frames={name:np.eye(4) for name in self.roots};pending=list(self.joints)
        while pending:
            progress=False
            for joint in list(pending):
                if joint['parent'] not in frames:continue
                motion=np.eye(4);value=coordinates.get(joint['name'],0.)
                if joint['kind'] in ('revolute','continuous'):
                    motion[:3,:3]=Rotation.from_rotvec(joint['axis']*value).as_matrix()
                elif joint['kind']=='prismatic':motion[:3,3]=joint['axis']*value
                elif joint['kind']!='fixed':raise ValueError('Unsupported URDF joint type')
                frames[joint['child']]=frames[joint['parent']]@joint['transform']@motion
                pending.remove(joint);progress=True
            if not progress:raise ValueError('Disconnected or cyclic URDF')
        return [(link,frames[link][:3,:3]@position+frames[link][:3,3],radius)
                for link,position,radius in self.spheres]


def obstacle_hits(spheres,problem):
    hits=[]
    for link,position,radius in spheres:
        for kind in ('sphere','box','cylinder'):
            for obstacle in problem[kind]:
                delta=position-np.asarray(obstacle['position'])
                if kind=='sphere':clearance=np.linalg.norm(delta)-radius-obstacle['radius']
                else:
                    rotation=Rotation.from_euler('xyz',obstacle['orientation_euler_xyz']).as_matrix()
                    local=rotation.T@delta
                    if kind=='box' or problem['problem']=='box':
                        half=np.asarray(obstacle['half_extents']) if kind=='box' else np.array([obstacle['radius'],obstacle['radius'],obstacle['length']/2])
                        # Signed box distance also gives a meaningful penetration
                        # estimate when the sphere center is inside the cuboid.
                        excess=np.abs(local)-half
                        clearance=np.linalg.norm(np.maximum(excess,0))+min(float(np.max(excess)),0.)-radius
                    else:
                        nearest=local.copy();nearest[2]-=np.clip(local[2],-obstacle['length']/2,obstacle['length']/2)
                        clearance=np.linalg.norm(nearest)-radius-obstacle['radius']
                if clearance<=0:
                    hits.append(dict(link=link,obstacle=obstacle['name'],clearance_m=float(clearance),
                                     robot_sphere_center=position.tolist(),robot_sphere_radius=radius))
    return hits


def run(output):
    output=Path(output)
    if output.exists():raise FileExistsError(output)
    source=mbm.VAMP_ROOT/'resources/panda/problems.pkl'
    urdf=mbm.VAMP_ROOT/'resources/panda/panda_spherized.urdf'
    oracle=SphereOracle(urdf);rb=robot.get_robot('panda');dataset=mbm.load_dataset('panda')
    rows=[]
    for environment in mbm.ENVIRONMENTS:
        for index,problem in enumerate(dataset[environment]):
            upstream_environment=vamp.problem_dict_to_vamp(problem);native_environment=mbm.native_environment(problem)
            endpoints=[]
            for name,q in [('start',problem['start'])]+[(f'goal_{i}',g) for i,g in enumerate(problem['goals'])]:
                spheres=oracle.forward(dict(zip(vamp.panda.joint_names(),q)))
                native_spheres=vamp.panda.fk(q)
                # Match sphere centers geometrically: ordering need not agree.
                centers=np.asarray([x[1] for x in spheres]);reference=np.asarray([x.position for x in native_spheres])
                distances=np.linalg.norm(centers[:,None]-reference[None,:],axis=-1)
                center_error=float(max(distances.min(0).max(),distances.min(1).max()))
                if center_error>1e-5:raise AssertionError(f'URDF/VAMP sphere FK mismatch: {environment}/{index}/{name}')
                hits=obstacle_hits(spheres,problem)
                _,self_pairs=vamp.panda.debug(q,upstream_environment)
                endpoints.append(dict(endpoint=name,configuration=q,
                    upstream_valid=bool(vamp.panda.validate(q,upstream_environment)),
                    native_valid=bool(native.module(rb).validate(native_environment,q)),
                    within_joint_limits=bool(np.all(np.asarray(q)>=rb.q_lo)&np.all(np.asarray(q)<=rb.q_hi)),
                    independent_environment_valid=not hits,environment_hits=hits,
                    upstream_self_collision_pairs=self_pairs,urdf_fk_maximum_center_error_m=center_error))
            rows.append(dict(environment=environment,index=index,source_valid=bool(problem['valid']),
                             supplied_goal_count=len(problem['goals']),endpoints=endpoints))
    summary=dict(available_entries=len(rows),source_eligible_entries=sum(r['source_valid'] for r in rows),
                 supplied_goal_count=sum(r['supplied_goal_count'] for r in rows),
                 source_exclusions=[dict(environment=r['environment'],index=r['index'],endpoints=r['endpoints']) for r in rows if not r['source_valid']],
                 native_upstream_disagreements=sum(e['upstream_valid']!=e['native_valid'] for r in rows for e in r['endpoints']),
                 independent_environment_disagreements=sum(e['upstream_valid']!=(e['independent_environment_valid'] and not e['upstream_self_collision_pairs'] and e['within_joint_limits']) for r in rows for e in r['endpoints']))
    output.parent.mkdir(parents=True,exist_ok=True)
    output.write_text(json.dumps(dict(summary=summary,rows=rows,
        limitations=['Same upstream sphere approximation, not robot meshes',
                     'Independent environment distances; VAMP supplies self-collision exclusions',
                     'Box-scene cylinders use upstream bounding boxes; other cylinders use upstream capsule overapproximation',
                     'Exact scene revision used by FLASK authors is unconfirmed']),indent=2)+'\n')
    save_manifest(output,dict(purpose='all_entries_inventory_and_endpoint_geometry_audit'),[source,urdf])
    print(json.dumps(summary,indent=2))


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__);parser.add_argument('--output',type=Path,required=True)
    run(parser.parse_args().output)
