"""Render grid: FLASK implementation and evaluation utilities."""
import argparse
import json
import os
import subprocess
from pathlib import Path
os.environ.setdefault('MUJOCO_GL', 'glfw')
import mujoco
import numpy as np
from PIL import Image, ImageDraw
import render_benchmark as QM
ROOT = QM.ROOT
(W, H) = (640, 360)
FPS = 30
(START_HOLD_S, END_HOLD_S) = (0.7, 1.2)
SEPARATION = 60.0
COLORS = {'F': (0.16, 0.49, 0.88, 1.0), 'B': (0.88, 0.48, 0.1, 1.0)}
RED = (0.9, 0.1, 0.1, 1.0)
JOINT_NAMES = ['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint', 'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint']

def build_scene(urdf, copies):
    """Build scene."""
    sp = mujoco.MjSpec()
    sp.visual.global_.offwidth = W
    sp.visual.global_.offheight = H
    sp.visual.quality.shadowsize = 4096
    sp.add_texture(name='troi', type=mujoco.mjtTexture.mjTEXTURE_SKYBOX, builtin=mujoco.mjtBuiltin.mjBUILTIN_GRADIENT, rgb1=[0.95, 0.96, 0.98], rgb2=[0.7, 0.76, 0.84], width=256, height=256)
    sp.add_texture(name='o', type=mujoco.mjtTexture.mjTEXTURE_2D, builtin=mujoco.mjtBuiltin.mjBUILTIN_CHECKER, rgb1=[0.86, 0.87, 0.89], rgb2=[0.8, 0.81, 0.84], width=300, height=300)
    mat = sp.add_material(name='san', texrepeat=[8, 8], reflectance=0.05)
    mat.textures[mujoco.mjtTextureRole.mjTEXROLE_RGB] = 'o'
    wb = sp.worldbody
    for (name, dx, vc, category) in copies:
        wb.add_light(pos=[dx + 1.0, -1.5, 3.5], dir=[-0.3, 0.4, -1], diffuse=[0.55, 0.55, 0.55], castshadow=True)
        wb.add_geom(name=f'san{name}', type=mujoco.mjtGeom.mjGEOM_PLANE, size=[4, 4, 0.1], pos=[dx, 0, 0], material='san', contype=0, conaffinity=0)
        wb.add_geom(name=f'chan{name}', type=mujoco.mjtGeom.mjGEOM_CYLINDER, size=[0.07, 0.4572, 0], pos=[dx, 0, 0.4572], rgba=[0.35, 0.35, 0.38, 1], contype=0, conaffinity=0)
        k = 0
        for o in vc['box']:
            wb.add_geom(name=f'vc{name}_{k}', type=mujoco.mjtGeom.mjGEOM_BOX, size=o['half_extents'], pos=np.add(o['position'], [dx, 0, 0]), quat=QM.quat_wxyz(o['orientation_quat_xyzw']), rgba=[0.62, 0.68, 0.76, 0.75])
            k += 1
        for o in vc['cylinder']:
            wb.add_geom(name=f'vc{name}_{k}', type=mujoco.mjtGeom.mjGEOM_CYLINDER, size=[o['radius'], o['length'] / 2, 0], pos=np.add(o['position'], [dx, 0, 0]), quat=QM.quat_wxyz(o['orientation_quat_xyzw']), rgba=[0.8, 0.66, 0.36, 1])
            k += 1
        for o in vc['sphere']:
            wb.add_geom(name=f'vc{name}_{k}', type=mujoco.mjtGeom.mjGEOM_SPHERE, size=[o['radius'], 0, 0], pos=np.add(o['position'], [dx, 0, 0]), rgba=[0.8, 0.66, 0.36, 1])
            k += 1
        for pre in (name, 'G' + name):
            fr = wb.add_frame(pos=[dx, 0, 0])
            sp.attach(mujoco.MjSpec.from_string(urdf), prefix=f'{pre}_', frame=fr)
    door_type = {name: category for (name, _, _, category) in copies}
    for g in sp.geoms:
        b = g.parent.name if g.parent is not None else ''
        if '_' not in b:
            continue
        pre = b.split('_', 1)[0]
        if pre.startswith('G') and pre[1:] in door_type:
            g.contype = 0
            g.conaffinity = 0
            g.rgba = [0.2, 0.75, 0.3, 0.22]
        elif pre in door_type:
            g.rgba = COLORS[door_type[pre]]
    m = sp.compile()
    m.vis.headlight.ambient[:] = 0.35
    m.vis.headlight.diffuse[:] = 0.45
    return m

class FourthOrderEdge:

    def __init__(self, robot, copies, mesh_directory):
        self.m = build_scene(QM.convert_urdf(robot, mesh_directory), copies)
        self.d = mujoco.MjData(self.m)
        m = self.m
        body_names = [m.body(i).name for i in range(m.nbody)]
        geom_names = [m.geom(i).name for i in range(m.ngeom)]
        (self.adr, self.geom_robot, self.geom_vc, self.ee) = ({}, {}, {}, {})
        for (name, _, _, _) in copies:
            for pre in (name, 'G' + name):
                self.adr[pre] = [m.joint(f'{pre}_{j}').qposadr[0] for j in JOINT_NAMES]
            self.geom_robot[name] = np.array([i for i in range(m.ngeom) if body_names[m.geom_bodyid[i]].startswith(name + '_')])
            self.geom_vc[name] = set((i for (i, n) in enumerate(geom_names) if n.startswith(f'vc{name}_')))
            self.ee[name] = m.body(f'{name}_{QM.EE_BODY}').id
            self.ghost_geoms = getattr(self, 'ghost_geoms', {})
            self.ghost_geoms[name] = np.array([i for i in range(m.ngeom) if body_names[m.geom_bodyid[i]].startswith('G' + name + '_')])

    def set_state(self, pre, q):
        self.d.qpos[self.adr[pre]] = q

    def collision(self, name):
        rg = set(self.geom_robot[name].tolist())
        vc = self.geom_vc[name]
        return sum((1 for c in self.d.contact[:self.d.ncon] if c.geom1 in rg and c.geom2 in vc or (c.geom2 in rg and c.geom1 in vc)))

    def scan(self, name, Q):
        vc = np.zeros(len(Q), bool)
        trail = []
        for i in range(len(Q)):
            self.set_state(name, Q[i])
            mujoco.mj_forward(self.m, self.d)
            vc[i] = self.collision(name) > 0
            if i % 20 == 0 or i == len(Q) - 1:
                trail.append(self.d.xpos[self.ee[name]].copy())
        return (vc, np.array(trail))

def camera(trails, contact_flags, dx):
    P = np.vstack([v - [d, 0, 0] for (v, d) in trails] + [np.asarray(o['position']) for vc in contact_flags for k in ('box', 'cylinder', 'sphere') for o in vc[k]] + [np.array([0, 0, 0.9])])
    (lower, upper) = (np.percentile(P, 5, axis=0), np.percentile(P, 95, axis=0))
    c = mujoco.MjvCamera()
    c.lookat[:] = (lower + upper) / 2 + [dx, 0, 0]
    c.distance = 1.35 * np.linalg.norm(upper - lower) + 1.2
    (c.azimuth, c.elevation) = (210.0, -24.0)
    return c

def label(img, ms, T, color):
    """Label."""
    im = Image.fromarray(img).convert('RGBA')
    o = Image.new('RGBA', im.size, (0, 0, 0, 0))
    dr = ImageDraw.Draw(o)
    dr.rounded_rectangle([8, 8, 268, 70], 8, fill=(255, 255, 255, 215))
    im = Image.alpha_composite(im, o)
    dr = ImageDraw.Draw(im)
    c = tuple((int(255 * v) for v in color[:3]))
    dr.text((18, 13), f'Planning: {ms:.3f} ms', font=QM.font(20, True), fill=c)
    dr.text((18, 40), f'Execution: {T:.1f} s', font=QM.font(20, True), fill=c)
    return np.asarray(im.convert('RGB'))

def main():
    from flask_repro.verification_gate import require_verified_reproduction
    require_verified_reproduction('Legacy showcase rendering')
    ap = argparse.ArgumentParser()
    ap.add_argument('--robot', default='ur5')
    ap.add_argument('--image_only', action='store_true', help='Write preview images for the first cycle')
    a = ap.parse_args()
    D = ROOT / 'results/legacy' / 'mujoco_mbm' / a.robot
    start_chain = json.loads((D / 'inventory.json').read_text())
    cycle = [start_chain[i:i + 2] for i in range(0, len(start_chain), 2)]
    output = D / f'video_{a.robot}_{len(start_chain)}problem_4o.mp4'
    ff = None if a.image_only else subprocess.Popen(['ffmpeg', '-loglevel', 'error', '-y', '-f', 'rawvideo', '-pix_fmt', 'rgb24', '-s', f'{2 * W}x{2 * H}', '-r', str(FPS), '-i', '-', '-c:v', 'libx264', '-pix_fmt', 'yuv420p', '-crf', '20', str(output)], stdin=subprocess.PIPE)
    renderer = None
    opt = mujoco.MjvOption()
    table = []
    for L in cycle:
        (copies, row) = ([], [])
        for (r, x) in enumerate(L):
            index = x['index']
            zt = np.load(D / f'problem_{index:02d}_rescaled.npz')
            zb = np.load(D / f'problem_{index:02d}.npz')
            vc = json.loads((D / f'problem_{index:02d}_obstacles.json').read_text())
            (dF, dB) = (2 * r * SEPARATION, (2 * r + 1) * SEPARATION)
            copies += [(f'F{r}', dF, vc, 'F'), (f'B{r}', dB, vc, 'B')]
            row.append(dict(x=x, vc=vc, dF=dF, dB=dB, QF=zt['Qf'], QB=zb['Qb'], goalF=zt['Qf'][-1], goalB=zb['Qb'][-1]))
        C = FourthOrderEdge(a.robot, copies, D / '_mesh')
        if renderer is not None:
            renderer.close()
        renderer = mujoco.Renderer(C.m, height=H, width=W, max_geom=20000)
        for (r, h) in enumerate(row):
            (h['vcF'], h['flask_trail']) = C.scan(f'F{r}', h['QF'])
            (h['vcB'], h['baseline_trail']) = C.scan(f'B{r}', h['QB'])
            C.set_state(f'GF{r}', h['goalF'])
            C.set_state(f'GB{r}', h['goalB'])
            h['camF'] = camera([(h['flask_trail'], h['dF']), (h['baseline_trail'], h['dB'])], [h['vc']], h['dF'])
            h['camB'] = camera([(h['flask_trail'], h['dF']), (h['baseline_trail'], h['dB'])], [h['vc']], h['dB'])
            x = h['x']
            table.append(dict(index=x['index'], env=x['env'], problem=x['problem'], flask_rescaled_planning_ms=x['flask_rescaled_planning_ms'], ms_baseline=x['ms_baseline'], flask_rescaled_duration_s=x['flask_rescaled_duration_s'], T_baseline=x['T_baseline'], flask_mesh_collision_samples=int(h['vcF'].sum()), baseline_mesh_collision_samples=int(h['vcB'].sum())))
            print(f"#{x['index']:2d} {x['env']:17s}: iterations {x['flask_rescaled_planning_ms']:.3f} vs {x['ms_baseline']:.2f} ms | run {x['flask_rescaled_duration_s']:.1f} vs {x['T_baseline']:.1f} s | mesh collisions {h['vcF'].sum()} / {h['vcB'].sum()}", flush=True)
        cycle_duration = max((max(len(h['QF']), len(h['QB'])) for h in row)) / 1000.0

        def compose_frame(t):
            o = []
            for r in range(2):
                if r >= len(row):
                    o.append(np.full((H, 2 * W, 3), 245, np.uint8))
                    continue
                h = row[r]
                image_only = []
                for (k, Q, vc, trail, cam, ms, T) in (('F', h['QF'], h['vcF'], h['flask_trail'], h['camF'], h['x']['flask_rescaled_planning_ms'], h['x']['flask_rescaled_duration_s']), ('B', h['QB'], h['vcB'], h['baseline_trail'], h['camB'], h['x']['ms_baseline'], h['x']['T_baseline'])):
                    name = f'{k}{r}'
                    j = min(int(t * 1000), len(Q) - 1)
                    j0 = min(int(max(t - 1 / FPS, 0) * 1000), len(Q) - 1)
                    C.set_state(name, Q[j])
                    C.m.geom_rgba[C.geom_robot[name]] = RED if t > 0 and vc[j0:j + 1].any() else COLORS[k]
                    C.m.geom_rgba[C.ghost_geoms[name], 3] = 0.0 if j >= len(Q) - 1 else 0.22
                mujoco.mj_forward(C.m, C.d)
                for (k, trail, cam, ms, T) in (('F', h['flask_trail'], h['camF'], h['x']['flask_rescaled_planning_ms'], h['x']['flask_rescaled_duration_s']), ('B', h['baseline_trail'], h['camB'], h['x']['ms_baseline'], h['x']['T_baseline'])):
                    renderer.update_scene(C.d, camera=cam, scene_option=opt)
                    QM.draw_trail(renderer.scene, trail, (*COLORS[k][:3], 0.8))
                    image_only.append(label(renderer.render(), ms, T, COLORS[k]))
                image_row = np.hstack(image_only)
                image_row[:, W - 1:W + 1] = 200
                o.append(image_row)
            out = np.vstack(o)
            out[H - 1:H + 1, :] = 120
            return out
        if a.image_only:
            for t in (0.0, cycle_duration / 2):
                Image.fromarray(compose_frame(t)).save(D / f'preview_multiview_{t:.1f}.png')
            break
        f0 = compose_frame(0.0).tobytes()
        for _ in range(int(START_HOLD_S * FPS)):
            ff.stdin.write(f0)
        for t in np.arange(0, cycle_duration + END_HOLD_S, 1 / FPS):
            ff.stdin.write(compose_frame(t).tobytes())
    if ff:
        ff.stdin.close()
        ff.wait()
        print(f'-> {output}')
        (D / 'multiview_table.json').write_text(json.dumps(table, indent=1, ensure_ascii=False))
if __name__ == '__main__':
    main()
