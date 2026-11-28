"""Manipulator scenes for the pick-and-place showcases (showcase_2): UR5 + Robotiq 2F-85, Franka Panda.

Simulation side only. The planner keeps the VAMP collision models (UR5 with its Robotiq 85, Panda with its hand);
here the fixed URDF Robotiq fingers are replaced by the actuated MuJoCo Menagerie 2F-85 (same gripper, mounted on
the same frame; tests/test_showcase_contracts.py checks that its geometry stays inside the VAMP spheres), and the
Panda keeps its Menagerie fingers, driven by finger motors.

Grasping: the fingers close on the object by contact. A weld equality is switched on only after the gripper's own
encoder reports that the fingers have stopped on something (stalled short of fully closed), and switched off when the
gripper opens. The weld keeps the object from slipping at speed; it is a simulation aid and is reported as such.
Nothing reads the object's true pose to control the robot; only the weld itself is anchored at the pose the object
has when the gripper closes.

Arm control: Ruckig (jerk-limited) velocity reference + computed torque on the arm's own joints at 500 Hz, physics at
1 kHz, with the robot's torque limits (the UR5 tracker of experiments/ur5_mujoco.py, generalized to several arms).
"""
import xml.etree.ElementTree as ET
from dataclasses import dataclass, field

import mujoco
import numpy as np
import ruckig
from scipy.spatial.transform import Rotation

from flask_repro import _core, mbm, robot as R
from simulation.robot_assets import MENAGERIE, load_robot

DT_CONTROL, DT_PHYSICS = .002, .001
KV, KQ, KP, J_MAX = 60., 900., 5., 50.
TORQUE = dict(ur5=np.array([150., 150., 150., 28., 28., 28.]), panda=np.array([87., 87., 87., 87., 12., 12., 12.]))
# grasp frame offset along the VAMP end-effector z axis (the point between the finger pads)
PINCH = dict(ur5=0.150, panda=0.0)          # UR5: robotiq_85_base_link -> pads; Panda: panda_grasptarget is the pinch
GRIP_BODY = dict(ur5='g_base', panda='panda_hand')


def vamp_module(kind):
    return getattr(_core, kind)


def _ur5_root():
    root, _ = load_robot('ur5')
    world = root.find('worldbody')
    base = next(b for b in world.iter('body') if b.get('name') == 'robotiq_85_base_link')
    for c in list(base):                       # fixed URDF Robotiq: replaced by the actuated Menagerie 2F-85
        if c.tag in ('body', 'geom'):
            base.remove(c)
    for g in world.iter('geom'):
        if g.get('group') == '3':
            g.set('contype', '2'); g.set('conaffinity', '1')
        else:
            g.set('contype', '0'); g.set('conaffinity', '0')
    act = root.find('actuator')
    for name, limit in zip(_core.ur5.joint_names() if hasattr(_core.ur5, 'joint_names') else
                           ['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint', 'wrist_1_joint',
                            'wrist_2_joint', 'wrist_3_joint'], TORQUE['ur5']):
        ET.SubElement(act, 'motor', name=f'motor_{name}', joint=name, ctrlrange=f'{-limit} {limit}', ctrllimited='true')
    return root


def _panda_root():
    root, meta = load_robot('panda')
    world = root.find('worldbody')
    for g in world.iter('geom'):
        if g.get('contype', '1') != '0' or g.get('conaffinity', '1') != '0':
            g.set('contype', '2'); g.set('conaffinity', '1')
    act = root.find('actuator')
    for i, limit in enumerate(TORQUE['panda']):
        ET.SubElement(act, 'motor', name=f'motor_panda_joint{i + 1}', joint=f'panda_joint{i + 1}',
                      ctrlrange=f'{-limit} {limit}', ctrllimited='true')
    for j, m in zip(meta['gripper_joint_names'], meta['gripper_motor_names']):
        ET.SubElement(act, 'motor', name=m, joint=j, ctrlrange='-40 40', ctrllimited='true')
    return root


def _spec(root):
    return mujoco.MjSpec.from_string(ET.tostring(root, encoding='unicode'))


@dataclass
class Arm:
    kind: str                       # 'ur5' or 'panda'
    prefix: str                     # name prefix in the composed model
    pos: tuple = (0., 0., 0.)       # base frame in the world
    yaw: float = 0.

    @property
    def rb(self):
        return R.get_robot(self.kind)

    @property
    def mod(self):
        return vamp_module(self.kind)

    @property
    def joints(self):
        n = ['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint', 'wrist_1_joint', 'wrist_2_joint',
             'wrist_3_joint'] if self.kind == 'ur5' else [f'panda_joint{i}' for i in range(1, 8)]
        return [self.prefix + x for x in n]

    @property
    def base_T(self):
        T = np.eye(4); T[:3, :3] = Rotation.from_euler('z', self.yaw).as_matrix(); T[:3, 3] = self.pos
        return T

    def world_ee(self, q):
        """End-effector pose (VAMP frame) in the world."""
        return self.base_T @ np.asarray(self.mod.eefk(list(map(float, q))), float)

    def world_spheres(self, q):
        S = np.asarray(self.mod.sphere_fk(list(map(float, q))), float)
        P = (self.base_T[:3, :3] @ S[:, :3].T).T + self.base_T[:3, 3]
        return np.c_[P, S[:, 3]]

    def pinch(self, q):
        T = self.world_ee(q)
        return T[:3, 3] + T[:3, 2] * PINCH[self.kind]

    def local_boxes(self, boxes):
        """Scene boxes (world, axis-aligned or yawed) expressed in this arm's base frame for the VAMP environment."""
        inv = np.linalg.inv(self.base_T); out = []
        for b in boxes:
            c = inv[:3, :3] @ np.asarray(b['position'], float) + inv[:3, 3]
            e = np.asarray(b.get('orientation_euler_xyz', (0., 0., 0.)), float).copy(); e[2] -= self.yaw
            out.append(dict(b, position=c.tolist(), orientation_euler_xyz=tuple(e)))
        return out


def environment(arm, boxes, padding=0., spheres=None):
    """VAMP environment of `arm` (its base frame): static boxes inflated by `padding`, plus optional world-frame
    spheres (N x 4: another arm's spheres, swept or at rest), one point cloud per radius (exact), inflated by `padding`."""
    local = arm.local_boxes(boxes)
    if padding:
        local = [dict(b, half_extents=[h + padding for h in b['half_extents']]) for b in local]
    env = mbm.native_environment(dict(problem='arm', sphere=[], cylinder=[], box=local))
    if spheres is not None and len(spheres):
        S = np.asarray(spheres, float); inv = np.linalg.inv(arm.base_T)
        P = (inv[:3, :3] @ S[:, :3].T).T + inv[:3, 3]
        R = np.round(S[:, 3], 4)
        for r in np.unique(R):
            k = R == r
            env.add_pointcloud(P[k].astype(np.float32), 0.005, 0.1, float(r + padding))   # robot sphere radii 5 mm - 10 cm
    return env


def held_spheres(half, T_obj_in_ee):
    """Spheres covering a held box (half sizes `half`, its own frame) in the VAMP end-effector frame, given the box
    pose in that frame. Near-cubic cells, each covered by its circumscribed sphere (overhang ~ 0.6 cell half size)."""
    h = np.asarray(half, float); n = np.maximum(3, 3 * np.round(h / h.min())).astype(int)
    cell = h / n; r = float(np.linalg.norm(cell))
    axes = [np.linspace(-h[i] + cell[i], h[i] - cell[i], n[i]) for i in range(3)]
    P = np.array([(x, y, z) for x in axes[0] for y in axes[1] for z in axes[2]])
    Q = (T_obj_in_ee[:3, :3] @ P.T).T + T_obj_in_ee[:3, 3]
    return [[*p, r] for p in Q.tolist()]


def object_T(position, yaw=0.):
    T = np.eye(4); T[:3, :3] = Rotation.from_euler('z', yaw).as_matrix(); T[:3, 3] = position
    return T


def build(arms, boxes, objects, colors, decorate=None, cameras=()):
    """Compose the MuJoCo model: floor, lights, static `boxes` (scene_* geoms), free `objects`
    {name: dict(position, half_extents, rgba)}, the arms at their base frames, and one inactive weld per
    (arm, object)."""
    xml = ET.Element('mujoco', model='showcase_arms')
    ET.SubElement(xml, 'compiler', angle='radian', autolimits='true')
    ET.SubElement(xml, 'option', timestep=str(DT_PHYSICS), integrator='implicitfast', cone='elliptic', impratio='10')
    vis = ET.SubElement(xml, 'visual')
    ET.SubElement(vis, 'global', offwidth='3840', offheight='2160')
    ET.SubElement(vis, 'quality', shadowsize='8192')
    ET.SubElement(vis, 'headlight', ambient='.32 .32 .34', diffuse='.35 .35 .35', specular='.05 .05 .05')
    asset = ET.SubElement(xml, 'asset')
    ET.SubElement(asset, 'texture', name='cell_floor', type='2d', builtin='checker', rgb1='.46 .47 .49', rgb2='.42 .43 .45',
                  mark='edge', markrgb='.36 .37 .39', width='512', height='512')
    ET.SubElement(asset, 'material', name='cell_floor', texture='cell_floor', texrepeat='4 4', texuniform='true', reflectance='.06')
    ET.SubElement(asset, 'texture', name='cell_sky', type='skybox', builtin='gradient', rgb1='.62 .70 .80', rgb2='.92 .94 .96',
                  width='512', height='3072')
    w = ET.SubElement(xml, 'worldbody')
    ET.SubElement(w, 'geom', name='floor', type='plane', size='4 4 0.05', material='cell_floor', contype='1', conaffinity='2')
    ET.SubElement(w, 'light', name='key', pos='1.2 -0.6 3.2', dir='-0.35 0.3 -1', diffuse='.6 .6 .58', cutoff='45',
                  exponent='3', castshadow='true')
    ET.SubElement(w, 'light', name='fill', pos='-1.5 1.5 2.5', dir='0.5 -0.4 -1', diffuse='.25 .25 .27', castshadow='false')
    for name, pos, xyaxes, fovy in cameras:
        ET.SubElement(w, 'camera', name=name, pos=' '.join(map(str, pos)), xyaxes=' '.join(map(str, xyaxes)), fovy=str(fovy))
    for b in boxes:
        e = b.get('orientation_euler_xyz', (0., 0., 0.))
        ET.SubElement(w, 'geom', name=f"scene_{b['name']}", type='box', pos=' '.join(map(str, b['position'])),
                      size=' '.join(map(str, b['half_extents'])), euler=' '.join(map(str, e)),
                      rgba=' '.join(map(str, colors(b['name']))), contype='1', conaffinity='2', friction='0.9 0.01 0.0001')
    for name, o in objects.items():
        body = ET.SubElement(w, 'body', name=f'object_{name}', pos=' '.join(map(str, o['position'])))
        ET.SubElement(body, 'freejoint', name=f'free_{name}')
        ET.SubElement(body, 'geom', name=f'object_{name}', type='box', size=' '.join(map(str, o['half_extents'])),
                      rgba=' '.join(map(str, o['rgba'])), mass=str(o.get('mass', .08)), contype='3', conaffinity='3',
                      friction='1.2 0.02 0.0005', solimp='0.95 0.99 0.001', solref='0.004 1', priority='1')
    if decorate is not None:
        decorate(w, asset)
    spec = mujoco.MjSpec.from_string(ET.tostring(xml, encoding='unicode'))
    for arm in arms:
        sub = _spec(_ur5_root() if arm.kind == 'ur5' else _panda_root())
        if arm.kind == 'ur5':
            grip = mujoco.MjSpec.from_file(str(MENAGERIE / 'robotiq_2f85/2f85.xml'))
            site = sub.body('robotiq_85_base_link').add_site(name='gripper_mount', pos=[0, 0, -0.003])
            site.attach_body(grip.body('base_mount'), 'g_', '')
        root_name = 'offset_link' if arm.kind == 'ur5' else 'panda_link0'
        frame = spec.worldbody.add_frame(pos=list(arm.pos), quat=Rotation.from_euler('z', arm.yaw).as_quat()[[3, 0, 1, 2]].tolist())
        frame.attach_body(sub.body(root_name), arm.prefix, '')
    for arm in arms:
        for name in objects:
            spec.add_equality(type=mujoco.mjtEq.mjEQ_WELD, objtype=mujoco.mjtObj.mjOBJ_BODY,
                              name1=arm.prefix + GRIP_BODY[arm.kind], name2=f'object_{name}', active=False,
                              name=f'weld_{arm.prefix}{name}', solref=[0.01, 1.])
    model = spec.compile()
    return model


@dataclass
class ArmIO:
    """Indices of one arm in the composed model."""
    arm: Arm
    model: object
    qadr: np.ndarray = field(init=False)
    dofadr: np.ndarray = field(init=False)
    act: np.ndarray = field(init=False)
    grip_act: np.ndarray = field(init=False)
    grip_q: np.ndarray = field(init=False)

    def __post_init__(self):
        m, a = self.model, self.arm
        self.qadr = np.array([m.joint(j).qposadr[0] for j in a.joints])
        self.dofadr = np.array([m.joint(j).dofadr[0] for j in a.joints])
        self.act = np.array([m.actuator(a.prefix + 'motor_' + j[len(a.prefix):]).id for j in a.joints])
        if a.kind == 'ur5':
            self.grip_act = np.array([m.actuator(a.prefix + 'g_fingers_actuator').id])
            self.grip_q = np.array([m.joint(a.prefix + 'g_right_driver_joint').qposadr[0],
                                    m.joint(a.prefix + 'g_left_driver_joint').qposadr[0]])
        else:
            self.grip_act = np.array([m.actuator(a.prefix + n).id for n in ('grasp_motor_left', 'grasp_motor_right')])
            self.grip_q = np.array([m.joint(a.prefix + n).qposadr[0] for n in ('grasp_joint_left', 'grasp_joint_right')])


class Gripper:
    """Open/close command and an encoder-only grasp detector (fingers stalled short of fully closed)."""

    def __init__(self, io, data):
        self.io, self.data, self.closing, self.t_cmd = io, data, False, 0.
        self.history = []

    def opening(self):
        """0 = fully open, 1 = fully closed, from the finger encoders."""
        q = self.data.qpos[self.io.grip_q]
        if self.io.arm.kind == 'ur5':
            return float(np.mean(q) / 0.8)
        return float(1. - np.mean(q) / 0.04)

    def command(self, close, t):
        if close != self.closing:
            self.closing, self.t_cmd, self.history = close, t, []

    def apply(self):
        d, io = self.data, self.io
        if io.arm.kind == 'ur5':
            d.ctrl[io.grip_act] = 255. if self.closing else 0.
        else:
            q = d.qpos[io.grip_q]; qd = d.qvel[[self.io.model.joint(io.arm.prefix + n).dofadr[0] for n in ('grasp_joint_left', 'grasp_joint_right')]]
            target = 0. if self.closing else 0.04
            force = np.clip(400. * (target - q) - 8. * qd, -25., 25.) if self.closing else np.clip(400. * (target - q) - 8. * qd, -40., 40.)
            d.ctrl[io.grip_act] = force

    def grasped(self, t):
        """True when, after closing for at least 0.25 s, the encoders have not moved for 0.1 s and the fingers stopped
        short of fully closed (something is between them)."""
        if not self.closing or t - self.t_cmd < 0.25:
            return False
        self.history.append((t, self.opening()))
        recent = [o for (tt, o) in self.history if tt >= t - 0.1]
        return len(recent) > 10 and max(recent) - min(recent) < 2e-3 and recent[-1] < 0.93


class ArmTracker:
    """Ruckig velocity reference + computed torque on one arm's joints (as experiments/ur5_mujoco.Tracker)."""

    def __init__(self, io, data, q0):
        self.io, self.data = io, data
        rb = io.arm.rb; n = rb.n; self.rb, self.n = rb, n
        self.limit = TORQUE[io.arm.kind]
        self.otg, self.inp, self.out = ruckig.Ruckig(n, DT_CONTROL), ruckig.InputParameter(n), ruckig.OutputParameter(n)
        self.inp.control_interface = ruckig.ControlInterface.Velocity
        self.inp.max_velocity, self.inp.max_acceleration = rb.v_max.tolist(), rb.a_max.tolist()
        self.inp.max_jerk = [J_MAX] * n
        self.inp.current_position, self.inp.current_velocity, self.inp.current_acceleration = list(q0), [0.] * n, [0.] * n
        self.saturated = False

    def reference(self, qd, vd, ad):
        d, io = self.data, self.io
        u = np.clip(vd + KP * (qd - d.qpos[io.qadr]), -self.rb.v_max, self.rb.v_max)
        self.inp.target_velocity = u.tolist()
        self.inp.target_acceleration = np.clip(ad, -self.rb.a_max, self.rb.a_max).tolist()
        res = self.otg.update(self.inp, self.out)
        if res not in (ruckig.Result.Working, ruckig.Result.Finished):
            raise RuntimeError(f'Ruckig error {res}')
        self.qc, self.vc, self.ac = (np.asarray(x) for x in (self.out.new_position, self.out.new_velocity, self.out.new_acceleration))
        self.out.pass_to_input(self.inp)

    def torque(self, M):
        d, io = self.data, self.io
        idx = io.dofadr
        tau = M[np.ix_(idx, idx)] @ (self.ac + KV * (self.vc - d.qvel[idx]) + KQ * (self.qc - d.qpos[io.qadr])) + d.qfrc_bias[idx]
        self.saturated |= bool(np.any(np.abs(tau) > self.limit))
        d.ctrl[io.act] = np.clip(tau, -self.limit, self.limit)


class Welds:
    """Switch the (arm, object) weld on at the object's current relative pose, or off."""

    def __init__(self, model, data):
        self.m, self.d = model, data

    def _id(self, arm, obj):
        return self.m.equality(f'weld_{arm.prefix}{obj}').id

    def on(self, arm, obj):
        i = self._id(arm, obj); m, d = self.m, self.d
        b1, b2 = m.eq_obj1id[i], m.eq_obj2id[i]
        p1, q1 = d.xpos[b1], d.xquat[b1]; p2, q2 = d.xpos[b2], d.xquat[b2]
        # pose of body2 (object) in body1's (gripper) frame: MuJoCo weld eq_data[3:6] relpos, [6:10] relquat
        # (checked: the object keeps its pose to 1 mm under gravity; the reverse convention moves it 35 cm)
        r2 = Rotation.from_quat(q2[[1, 2, 3, 0]]); r1 = Rotation.from_quat(q1[[1, 2, 3, 0]])
        relpos = r1.inv().apply(p2 - p1); relq = (r1.inv() * r2).as_quat()[[3, 0, 1, 2]]
        m.eq_data[i, :3] = 0.; m.eq_data[i, 3:6] = relpos; m.eq_data[i, 6:10] = relq; m.eq_data[i, 10] = 1.
        d.eq_active[i] = 1

    def off(self, arm, obj):
        self.d.eq_active[self._id(arm, obj)] = 0

    def active(self):
        return [self.m.equality(i).name for i in range(self.m.neq) if self.d.eq_active[i] and self.m.equality(i).name.startswith('weld_')]


def contacts(model, data, allowed=()):
    """Contacts between robots/objects and scene_* geoms or between the two robots (penetration only).
    `allowed` geom-name prefixes are ignored (e.g. an object resting in its tray)."""
    bad = set()
    for c in data.contact[:data.ncon]:
        if c.dist >= 0:
            continue
        a, b = model.geom(int(c.geom1)).name, model.geom(int(c.geom2)).name
        if any(a.startswith(p) or b.startswith(p) for p in allowed):
            continue
        bad.add(tuple(sorted((a, b))))
    return bad


def down(yaw=0., kind='ur5'):
    """Gripper pointing down (approach -z world), fingers opening along world x rotated by `yaw`."""
    Rz = Rotation.from_euler('z', yaw).as_matrix()
    base = np.array([[1., 0., 0.], [0., -1., 0.], [0., 0., -1.]])      # x stays, z points down
    if kind == 'panda':
        base = np.array([[0., -1., 0.], [-1., 0., 0.], [0., 0., -1.]]) # Panda hand: fingers open along its y axis
    return Rz @ base


def ik(arm, pinch, rotation, q0, env=None, iterations=300, restarts=12, rng=None):
    """Damped least squares on the VAMP FK: the pinch point at `pinch` (world) with end-effector rotation `rotation`
    (world). Tries `restarts` perturbed seeds; returns the solution closest to q0 that is within limits and (if env
    is given) collision free, else None."""
    rb = arm.rb; Tb = arm.base_T; inv = np.linalg.inv(Tb)
    Rl = inv[:3, :3] @ rotation
    p_ee = pinch - rotation[:, 2] * PINCH[arm.kind]
    pl = inv[:3, :3] @ p_ee + inv[:3, 3]
    rng = rng or np.random.default_rng(0)

    def err(q):
        T = np.asarray(arm.mod.eefk(list(map(float, q))), float)
        # rotation-vector error: zero only at the target orientation (the sum of axis cross products also vanishes
        # for 180-degree flips)
        return np.r_[pl - T[:3, 3], Rotation.from_matrix(Rl @ T[:3, :3].T).as_rotvec()]

    best = None
    for r in range(restarts):
        q = np.array(q0, float) + (0 if r == 0 else rng.normal(0, 0.35, rb.n))
        for _ in range(iterations):
            e = err(q)
            if np.linalg.norm(e) < 1e-6:
                break
            J = np.empty((6, rb.n)); h = 1e-6
            for j in range(rb.n):
                dq = np.zeros(rb.n); dq[j] = h
                J[:, j] = (e - err(q + dq)) / h
            step = J.T @ np.linalg.solve(J @ J.T + 1e-6 * np.eye(6), e)
            q = np.clip(q + np.clip(step, -0.3, 0.3), rb.q_lo + 1e-3, rb.q_hi - 1e-3)
        if np.linalg.norm(err(q)[:3]) > 1e-4 or np.linalg.norm(err(q)[3:]) > 1e-4:
            continue
        if env is not None and not arm.mod.validate(env, q.tolist()):
            continue
        if best is None or np.linalg.norm(q - q0) < np.linalg.norm(best - q0):
            best = q.copy()
    return best
