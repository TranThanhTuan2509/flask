"""Online executor for one or more arms doing pick-and-place (showcase_2): every motion is planned by FLASK (native,
VAMP, lazy-certified) at the moment it starts, from the arm's last commanded configuration, and tracked in MuJoCo.

Several arms: prioritized space-time planning in planning order. An arm that plans a motion treats the other arms'
already committed motions as obstacles: the spheres of their VAMP models sampled along the rest of their trajectory
(swept volume, every ~1.5 cm of sphere travel) and their final configuration, plus the objects they hold. Committed
motions are therefore pairwise collision free on the common clock; the swept volume makes this conservative (an arm
also avoids where the other arm *will* be). Handover hand-offs are coordinated by events (wait/signal).

Each arm's queue holds actions:
  ('move', q_goal, label, ignore)     plan and follow; `ignore` = objects the gripper may approach (grasp target)
                                      and, for the short lift/lower moves, the support the held object rests on
  ('close', obj, label)               close; when the encoders report a stall, weld obj to this gripper
  ('open', obj, label, place_xyz)     unweld and open; obj is believed to be at place_xyz afterwards
  ('signal', event) / ('wait', event) cross-arm synchronization
  ('pause', seconds, label)
"""
import time

import mujoco
import numpy as np

from flask_repro import native
from simulation.showcase import arms as A

# experiments/ur5_mujoco.FLASK_SETTINGS with rho = 10 (time weight of the LQMT cost, paper Eq. 20): brisker
# motions than the default 1 (measured peak acceleration 43-56 % of the limits instead of 14-18 %)
SETTINGS = dict(prefix_steering=False, velocity_sample_scale=0., range=2., neighbor_mode='A',
                max_ns=int(1e9), max_iterations=int(1e7), max_samples=100000, rho=10.)
PADDING, DELTA = 0.015, 0.001
ESCALATION = (0., 0.005, 0.01)                # extra nominal padding when certification rejects every path
RETRY_EVERY, RETRY_FOR = 0.25, 20.            # s: hold and replan while another arm's committed motion blocks
OBJECT_PADDING = 0.01


class Station:
    def __init__(self, arm, model, data, q0, queue):
        self.arm, self.queue = arm, list(queue)
        self.io = A.ArmIO(arm, model)
        self.tracker = A.ArmTracker(self.io, data, q0)
        self.gripper = A.Gripper(self.io, data)
        self.q_rest = np.array(q0, float)
        self.traj, self.t0, self.label, self.carrying = None, 0., 'idle', None
        self.grip_T = None                                   # held object pose in the end-effector frame (planned)
        self.action, self.action_t0 = None, 0.
        self.retry_at, self.failed, self.first_try = 0., [], {}
        self.done = False


class Cell:
    def __init__(self, model, arms, q0s, queues, boxes, objects, seed, settings=None):
        self.m, self.d = model, mujoco.MjData(model)
        self.boxes, self.seed = boxes, seed
        self.objects = {k: dict(v) for k, v in objects.items()}      # believed object poses (known map)
        self.welds = A.Welds(model, self.d)
        self.st = [Station(a, model, self.d, q, Q) for a, q, Q in zip(arms, q0s, queues)]
        self.events, self.plans, self.n_motion = set(), [], 0
        self.settings = native.settings(**(settings or SETTINGS))
        for s in self.st:
            self.d.qpos[s.io.qadr] = s.q_rest
            if s.arm.kind == 'panda':
                self.d.qpos[s.io.grip_q] = 0.04
        mujoco.mj_forward(model, self.d)
        for s in self.st:                                     # warm-up, not timed
            env = A.environment(s.arm, boxes, PADDING)
            native.plan_lazy_certified(s.arm.rb, env, A.environment(s.arm, boxes, PADDING + DELTA), s.q_rest, [s.q_rest],
                                       self.settings, seed=0, certify_delta=DELTA)
        self.M = np.zeros((model.nv, model.nv))
        self.log = dict(time=[], qpos=[], phase=[], grip=[], welds=[], bad=[])
        self.t, self.bad_pairs = 0., set()

    # ---------- planning ----------
    def _obstacles(self, s, ignore):
        boxes = [b for b in self.boxes if b['name'] not in ignore]
        for name, o in self.objects.items():
            if name in ignore or any(x.carrying == name for x in self.st):
                continue
            boxes.append(dict(name=f'obj_{name}', position=list(o['position']),
                              half_extents=[h + OBJECT_PADDING - PADDING for h in o['half_extents']],
                              orientation_euler_xyz=(0., 0., 0.)))
        return boxes

    def _swept(self, s, ignore=()):
        """World points and radii of the other arms' committed motion from now on (and their rest pose). Objects in
        `ignore` held by another arm are left out (a hand-over: the receiver approaches the object it will take)."""
        pts = []
        for o in self.st:
            if o is s:
                continue
            qs = [o.q_rest]
            if o.traj is not None:
                T = o.traj.duration(); tau = max(0., self.t - o.t0)
                ts = np.linspace(tau, T, max(2, int((T - tau) / 0.02) + 1))
                qs = list(o.traj.evaluate(ts)[0]) + qs
            for q in qs:
                S = o.arm.world_spheres(q)
                pts.append(S)
                if o.carrying is not None and o.carrying not in ignore:   # the object it holds, at its grasp
                    H = np.asarray(A.held_spheres(self.objects[o.carrying]['half_extents'], o.grip_T))
                    T = o.arm.world_ee(q)
                    pts.append(np.c_[(T[:3, :3] @ H[:, :3].T).T + T[:3, 3], H[:, 3]])
        return np.concatenate(pts) if pts else np.zeros((0, 4))

    def _env(self, s, ignore, pad):
        env = A.environment(s.arm, self._obstacles(s, ignore), pad, self._swept(s, ignore))
        if s.carrying is not None:
            env.attach(A.held_spheres(self.objects[s.carrying]['half_extents'], s.grip_T))
        return env

    def _plan(self, s, q_goal, label, ignore):
        self.n_motion += 1
        # A nominal path may graze an obstacle within the certification margin (then every attempt returns the same
        # edge and is rejected). Escalation: replan with a larger nominal padding; the certified scene is always the
        # nominal one inflated by DELTA, so validity is never relaxed. Every attempt's planning time is counted.
        plan_us = shortcut_us = wall = build_ms = 0.
        probe = self._env(s, ignore, PADDING)                   # blocked start or goal (another arm in the way): wait
        if not (s.arm.mod.validate(probe, list(map(float, s.q_rest))) and s.arm.mod.validate(probe, list(map(float, q_goal)))):
            return dict(plan_us=0., wall_ms=0., blocked=True)
        for extra in ESCALATION:
            t0 = time.perf_counter()
            nominal = self._env(s, ignore, PADDING + extra); cert = self._env(s, ignore, PADDING + extra + DELTA)
            build_ms += (time.perf_counter() - t0) * 1e3
            t1 = time.perf_counter()
            r = native.plan_lazy_certified(s.arm.rb, nominal, cert, s.q_rest, [np.asarray(q_goal)], self.settings,
                                           seed=self.seed * 1000 + self.n_motion, certify_delta=DELTA)
            wall += (time.perf_counter() - t1) * 1e3
            plan_us += r['plan_ns'] / 1e3; shortcut_us += r.get('shortcut_ns', 0) / 1e3
            if r['solved']:
                break
        if not r['solved']:
            return dict(plan_us=plan_us + shortcut_us, wall_ms=wall)
        traj = r['trajectory']; T = traj.duration()
        Q = traj.evaluate(np.linspace(0, T, 120))[0]
        self.plans.append(dict(arm=s.arm.prefix.rstrip('/'), label=label, t_start=self.t, duration=T,
                               plan_us=plan_us, shortcut_us=shortcut_us, padding_extra=extra,
                               total_us=plan_us + shortcut_us, wall_ms=wall, env_build_ms=build_ms,
                               carrying=s.carrying, segments=len(r['T']),
                               pinch_path=[s.arm.pinch(q).tolist() for q in Q]))
        s.traj, s.t0, s.label = traj, self.t, label
        s.q_goal = np.asarray(q_goal, float)
        return None

    # ---------- execution ----------
    def _next(self, s):
        if s.action is not None or s.traj is not None:
            return
        while s.queue:
            a = s.queue[0]
            if a[0] == 'signal':
                self.events.add(a[1]); s.queue.pop(0); continue
            if a[0] == 'wait':
                if a[1] in self.events:
                    s.queue.pop(0); continue
                s.label = 'waiting'
                return
            if a[0] == 'move':
                # online prioritized planning: if the other arm's committed motion blocks this one, hold in place and
                # replan shortly (its remaining swept volume shrinks as it moves); failed attempts are recorded
                if self.t < s.retry_at:
                    return
                fail = self._plan(s, a[1], a[2], a[3] if len(a) > 3 else ())
                if fail is not None:
                    s.failed.append(dict(label=a[2], t=self.t, aid=id(a), **fail))
                    if self.t - s.first_try.setdefault(id(a), self.t) > RETRY_FOR:
                        raise RuntimeError(f'{s.arm.prefix}{a[2]}: plan failed')
                    s.retry_at = self.t + RETRY_EVERY; s.label = f'waiting: {a[2]}'
                    return
                self.plans[-1]['retries'] = sum(f['aid'] == id(a) for f in s.failed)
                s.queue.pop(0)
                return
            s.queue.pop(0); s.action, s.action_t0 = a, self.t
            if a[0] == 'close':
                s.gripper.command(True, self.t); s.label = a[2]
            elif a[0] == 'open':
                self.welds.off(s.arm, a[1]); s.gripper.command(False, self.t); s.label = a[2]
                s.carrying = None
                if a[3] is not None:                            # None: handed over, still held by the other arm
                    self.objects[a[1]]['position'] = list(a[3])
            elif a[0] == 'pause':
                s.label = a[2]
            return
        s.done = True; s.label = 'done'

    def _service(self, s):
        a = s.action
        if a is None:
            return
        if a[0] == 'close':
            if s.gripper.grasped(self.t):
                self.welds.on(s.arm, a[1]); s.carrying = a[1]; s.action = None
                # planned grasp: believed object pose (or the giver's grasp) relative to the commanded end effector
                giver = next((o for o in self.st if o is not s and o.carrying == a[1]), None)
                T_obj = (giver.arm.world_ee(giver.q_rest) @ giver.grip_T if giver is not None
                         else A.object_T(self.objects[a[1]]['position'], self.objects[a[1]].get('yaw', 0.)))
                s.grip_T = np.linalg.inv(s.arm.world_ee(s.q_rest)) @ T_obj
            elif self.t - s.action_t0 > 2.0:
                raise RuntimeError(f'{s.arm.prefix}: grasp of {a[1]} not detected (gripper closure '
                                   f'{s.gripper.opening():.3f}, pinch {np.round(s.arm.pinch(self.d.qpos[s.io.qadr]), 3)}, '
                                   f'object {np.round(self.d.body("object_" + a[1]).xpos, 3)})')
        elif a[0] == 'open' and self.t - s.action_t0 > 0.25 and s.gripper.opening() < 0.12:   # encoders: open
            s.action = None
        elif a[0] == 'open' and self.t - s.action_t0 > 3.0:
            raise RuntimeError(f'{s.arm.prefix}: gripper did not open')
        elif a[0] == 'pause' and self.t - s.action_t0 > a[1]:
            s.action = None

    def run(self, max_time=120.):
        m, d = self.m, self.d
        step = 0
        while not all(s.done for s in self.st) and self.t < max_time:
            for s in self.st:
                self._next(s)
            for s in self.st:
                self._service(s)
                if s.traj is not None:
                    T = s.traj.duration(); tk = self.t - s.t0
                    q, v, a = (x[0] for x in s.traj.evaluate([min(tk, T)])[:3])
                    if tk >= T:
                        v, a = np.zeros_like(v), np.zeros_like(a)
                    if tk >= T + 0.12:                          # settled on the final setpoint
                        s.q_rest = s.q_goal; s.traj = None
                else:
                    q, v, a = s.q_rest, np.zeros(s.arm.rb.n), np.zeros(s.arm.rb.n)
                s.tracker.reference(q, v, a)
            for _ in range(int(round(A.DT_CONTROL / A.DT_PHYSICS))):
                mujoco.mj_fullM(m, d, self.M)
                for s in self.st:
                    s.tracker.torque(self.M); s.gripper.apply()
                mujoco.mj_step(m, d)
            self.t += A.DT_CONTROL; step += 1
            bad = self.check_contacts()
            if step % 5 == 0:
                self.log['time'].append(self.t); self.log['qpos'].append(d.qpos.copy())
                self.log['phase'].append([s.label for s in self.st])
                self.log['grip'].append([s.gripper.opening() for s in self.st])
                self.log['welds'].append([int(d.eq_active[i]) for i in range(m.neq)])
                self.log['bad'].append(len(bad))
        if not all(s.done for s in self.st):
            raise RuntimeError(f'task not finished in {max_time} s: {[s.label for s in self.st]}')

    def check_contacts(self):
        """Forbidden: robot vs scene, robot vs other robot, robot vs an object it does not hold, a held object vs
        the scene. Allowed: an object resting on the scene, fingers on the object they grasp."""
        m, d = self.m, self.d
        held = {(s.carrying, s.arm.prefix) for s in self.st if s.carrying}   # both arms hold it during a hand-over
        held_objects = {o for o, _ in held}
        closing = {s.arm.prefix: s.action[1] for s in self.st if s.action is not None and s.action[0] in ('close', 'open')}
        bad = set()
        for c in d.contact[:d.ncon]:
            if c.dist >= 0:
                continue
            a, b = m.geom(int(c.geom1)).name, m.geom(int(c.geom2)).name
            names = (a, b)
            robot = [n for n in names if '/' in n]
            obj = [n[len('object_'):] for n in names if n.startswith('object_')]
            scene = [n for n in names if n.startswith('scene_') or n == 'floor']
            if len(robot) == 2:
                if robot[0].split('/')[0] != robot[1].split('/')[0]:
                    bad.add(names)
                continue
            if robot and scene:
                bad.add(names); continue
            if robot and obj:
                prefix = robot[0].split('/')[0] + '/'
                if (obj[0], prefix) in held or closing.get(prefix) == obj[0]:
                    continue
                # during a hand-over the receiving fingers touch the object while the giver still holds it
                if obj[0] in held_objects and any(s.arm.prefix == prefix and s.label.startswith('hand') for s in self.st):
                    continue
                bad.add(names); continue
            if obj and scene and obj[0] in held_objects:
                lifting = any(x.carrying == obj[0] and x.label.split()[0] in ('grasp', 'lift', 'lower', 'release') for x in self.st)
                if not lifting:                                 # leaving or reaching its support is allowed
                    bad.add(names)
        self.bad_pairs |= bad
        return bad
