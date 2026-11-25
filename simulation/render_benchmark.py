"""Render benchmark: FLASK implementation and evaluation utilities."""
import argparse
import json
import os
import re
import shutil
import subprocess
from pathlib import Path
os.environ.setdefault('MUJOCO_GL', 'glfw')
import mujoco
import numpy as np
from PIL import Image, ImageDraw, ImageFont
ROOT = Path(__file__).resolve().parent.parent
VAMP_RES = ROOT / 'third_party' / 'vamp' / 'resources'
FONT = '/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf'
FONT_B = '/usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf'
(W, H, HEAD, FOOT) = (640, 500, 84, 56)
FPS = 30
PAUSE_S = 3.0
END_HOLD_S = 1.0
SEPARATION = 60.0
COLORS = {'A': (0.16, 0.49, 0.88, 1.0), 'B': (0.88, 0.48, 0.1, 1.0)}
RED = (0.9, 0.1, 0.1, 1.0)
EE_BODY = 'robotiq_85_base_link'

def convert_urdf(robot, mesh_directory):
    """Convert urdf."""
    V = VAMP_RES / robot
    mesh_directory.mkdir(parents=True, exist_ok=True)
    for p in V.glob('meshes/**/*'):
        if p.is_file() and p.suffix.lower() == '.stl':
            shutil.copy(p, mesh_directory / p.name)
    s = (V / f'{robot}.urdf').read_text()
    s = re.sub('filename="[^"]*/visual/(\\w+)\\.dae"', 'filename="\\1.stl"', s)
    s = re.sub('filename="(?:package://)?meshes/(?:[\\w/]*/)?([^"/]+)"', 'filename="\\1"', s)
    s = re.sub('(<robot[^>]*>)', '\\1<mujoco><compiler meshdir="%s" strippath="true" discardvisual="true" fusestatic="false"/></mujoco>' % mesh_directory.resolve(), s, count=1)
    return s

def quat_wxyz(xyzw):
    (x, y, z, w) = xyzw
    return [w, x, y, z]

def build_scene(urdf, obstacles):
    sp = mujoco.MjSpec()
    sp.visual.global_.offwidth = W
    sp.visual.global_.offheight = H
    sp.visual.quality.shadowsize = 4096
    sp.add_texture(name='troi', type=mujoco.mjtTexture.mjTEXTURE_SKYBOX, builtin=mujoco.mjtBuiltin.mjBUILTIN_GRADIENT, rgb1=[0.95, 0.96, 0.98], rgb2=[0.7, 0.76, 0.84], width=256, height=256)
    sp.add_texture(name='o', type=mujoco.mjtTexture.mjTEXTURE_2D, builtin=mujoco.mjtBuiltin.mjBUILTIN_CHECKER, rgb1=[0.86, 0.87, 0.89], rgb2=[0.8, 0.81, 0.84], width=300, height=300)
    mat = sp.add_material(name='san', texrepeat=[8, 8], reflectance=0.05)
    mat.textures[mujoco.mjtTextureRole.mjTEXROLE_RGB] = 'o'
    wb = sp.worldbody
    wb.add_light(pos=[SEPARATION / 2, -3, 6], dir=[0, 0.4, -1], diffuse=[0.5, 0.5, 0.5], castshadow=False)
    for (name, dx) in (('A', 0.0), ('B', SEPARATION)):
        wb.add_light(pos=[dx + 1.0, -1.5, 3.5], dir=[-0.3, 0.4, -1], diffuse=[0.55, 0.55, 0.55], castshadow=True)
        wb.add_geom(name=f'san{name}', type=mujoco.mjtGeom.mjGEOM_PLANE, size=[4, 4, 0.1], pos=[dx, 0, 0], material='san', contype=0, conaffinity=0)
        wb.add_geom(name=f'chan{name}', type=mujoco.mjtGeom.mjGEOM_CYLINDER, size=[0.07, 0.4572, 0], pos=[dx, 0, 0.4572], rgba=[0.35, 0.35, 0.38, 1], contype=0, conaffinity=0)
        k = 0
        for o in obstacles['box']:
            wb.add_geom(name=f'vc{name}_{k}', type=mujoco.mjtGeom.mjGEOM_BOX, size=o['half_extents'], pos=np.add(o['position'], [dx, 0, 0]), quat=quat_wxyz(o['orientation_quat_xyzw']), rgba=[0.62, 0.68, 0.76, 0.75])
            k += 1
        for o in obstacles['cylinder']:
            wb.add_geom(name=f'vc{name}_{k}', type=mujoco.mjtGeom.mjGEOM_CYLINDER, size=[o['radius'], o['length'] / 2, 0], pos=np.add(o['position'], [dx, 0, 0]), quat=quat_wxyz(o['orientation_quat_xyzw']), rgba=[0.8, 0.66, 0.36, 1])
            k += 1
        for o in obstacles['sphere']:
            wb.add_geom(name=f'vc{name}_{k}', type=mujoco.mjtGeom.mjGEOM_SPHERE, size=[o['radius'], 0, 0], pos=np.add(o['position'], [dx, 0, 0]), rgba=[0.8, 0.66, 0.36, 1])
            k += 1
    for (name, dx) in (('A', 0.0), ('B', SEPARATION), ('GA', 0.0), ('GB', SEPARATION)):
        fr = wb.add_frame(pos=[dx, 0, 0])
        sp.attach(mujoco.MjSpec.from_string(urdf), prefix=f'{name}_', frame=fr)
    for g in sp.geoms:
        b = g.parent.name if g.parent is not None else ''
        if b.startswith(('GA_', 'GB_')):
            g.contype = 0
            g.conaffinity = 0
            g.rgba = [0.2, 0.75, 0.3, 0.22]
            g.group = 1
        elif b.startswith(('A_', 'B_')):
            g.rgba = COLORS[b[0]]
    m = sp.compile()
    m.vis.headlight.ambient[:] = 0.35
    m.vis.headlight.diffuse[:] = 0.45
    return m

class MeshScene:

    def __init__(self, robot, obstacles, mesh_directory):
        self.m = build_scene(convert_urdf(robot, mesh_directory), obstacles)
        self.d = mujoco.MjData(self.m)
        m = self.m
        self.adr = {t: [m.joint(f'{t}_{j}').qposadr[0] for j in ['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint', 'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint']] for t in ('A', 'B', 'GA', 'GB')}
        body_names = [m.body(i).name for i in range(m.nbody)]
        gb = m.geom_bodyid
        self.geom_robot = {t: np.array([i for i in range(m.ngeom) if body_names[gb[i]].startswith(t + '_') and (not body_names[gb[i]].startswith('G'))]) for t in ('A', 'B')}
        geom_names = [m.geom(i).name for i in range(m.ngeom)]
        self.geom_vc = {t: set((i for (i, n) in enumerate(geom_names) if n.startswith(f'vc{t}_'))) for t in ('A', 'B')}
        self.ee = {t: m.body(f'{t}_{EE_BODY}').id for t in ('A', 'B')}

    def set_state(self, t, q):
        self.d.qpos[self.adr[t]] = q

    def collision(self, t):
        """Collision."""
        rg = set(self.geom_robot[t].tolist())
        vc = self.geom_vc[t]
        n = 0
        for c in self.d.contact[:self.d.ncon]:
            if c.geom1 in rg and c.geom2 in vc or (c.geom2 in rg and c.geom1 in vc):
                n += 1
        return n

    def scan(self, t, Q, stride=1):
        """Scan."""
        vc = np.zeros(len(Q), bool)
        trail = []
        for i in range(0, len(Q), stride):
            self.set_state(t, Q[i])
            mujoco.mj_forward(self.m, self.d)
            vc[i] = self.collision(t) > 0
            if i % 20 == 0 or i == len(Q) - 1:
                trail.append(self.d.xpos[self.ee[t]].copy())
        return (vc, np.array(trail))

def font(sz, bold=False):
    return ImageFont.truetype(FONT_B if bold else FONT, sz)

def draw_trail(scene, trail, color):
    for (a, b) in zip(trail[:-1], trail[1:]):
        if scene.ngeom >= scene.maxgeom:
            break
        g = scene.geoms[scene.ngeom]
        mujoco.mjv_initGeom(g, mujoco.mjtGeom.mjGEOM_CAPSULE, np.zeros(3), np.zeros(3), np.zeros(9), np.array(color, np.float32))
        mujoco.mjv_connector(g, mujoco.mjtGeom.mjGEOM_CAPSULE, 0.004, a, b)
        scene.ngeom += 1

def main():
    from flask_repro.verification_gate import require_verified_reproduction
    require_verified_reproduction('Legacy showcase rendering')
    ap = argparse.ArgumentParser()
    ap.add_argument('--robot', default='ur5')
    ap.add_argument('--selected_cases', type=int, nargs='*', help='Render the selected one-based problem indices')
    ap.add_argument('--image_only', action='store_true', help='Write preview images without encoding a video')
    a = ap.parse_args()
    D = ROOT / 'results/legacy' / 'mujoco_mbm' / a.robot
    start_chain = json.loads((D / 'inventory.json').read_text())
    if a.selected_cases:
        start_chain = [x for x in start_chain if x['index'] in a.selected_cases]
    output = D / f'video_{a.robot}_{len(start_chain)}problem.mp4'
    ff = None
    if not a.image_only:
        ff = subprocess.Popen(['ffmpeg', '-loglevel', 'error', '-y', '-f', 'rawvideo', '-pix_fmt', 'rgb24', '-s', f'{2 * W}x{HEAD + H + FOOT}', '-r', str(FPS), '-i', '-', '-c:v', 'libx264', '-pix_fmt', 'yuv420p', '-crf', '20', str(output)], stdin=subprocess.PIPE)
    renderer = None
    table = []
    opt = mujoco.MjvOption()
    opt.geomgroup[:] = 1
    for x in start_chain:
        index = x['index']
        z = np.load(D / f'problem_{index:02d}.npz')
        obstacles = json.loads((D / f'problem_{index:02d}_obstacles.json').read_text())
        C = MeshScene(a.robot, obstacles, D / '_mesh')
        if renderer is None:
            renderer = mujoco.Renderer(C.m, height=H, width=W, max_geom=20000)
        else:
            renderer.close()
            renderer = mujoco.Renderer(C.m, height=H, width=W, max_geom=20000)
        (vcA, flask_trail) = C.scan('A', z['Qf'])
        (vcB, baseline_trail) = C.scan('B', z['Qb'])
        C.set_state('A', z['Qf'][0])
        C.set_state('B', z['Qb'][0])
        C.set_state('GA', z['goal'])
        C.set_state('GB', z['Qb'][-1])
        mujoco.mj_forward(C.m, C.d)
        va0 = (C.collision('A'), C.collision('B'))
        x.update(flask_mesh_collision_samples=int(vcA.sum()), baseline_mesh_collision_samples=int(vcB.sum()), initial_mesh_collisions=list(va0))
        table.append(x)
        print(f"#{index:2d} {x['env']:17s} {x['problem']:3d}: mesh collisions FLASK {vcA.sum():5d} / baseline {vcB.sum():5d} color  (sphere VAMP: {x['flask_sphere_collision_samples']} / {x['baseline_sphere_collision_samples']}); at initialization {va0}", flush=True)
        P = np.vstack([flask_trail, baseline_trail - [SEPARATION, 0, 0]] + [np.asarray(o['position']) for k in ('box', 'cylinder', 'sphere') for o in obstacles[k]] + [np.array([0, 0, 0.9])])
        (lower, upper) = (np.percentile(P, 5, axis=0), np.percentile(P, 95, axis=0))
        cam = mujoco.MjvCamera()
        cam.lookat[:] = (lower + upper) / 2
        cam.distance = 1.1 * np.linalg.norm(upper - lower) + 0.8
        (cam.azimuth, cam.elevation) = (210.0, -24.0)

        def capture(t_A, t_B):
            imgs = []
            for (name, dx, trail, color) in (('A', 0.0, flask_trail, COLORS['A']), ('B', SEPARATION, baseline_trail, COLORS['B'])):
                c = mujoco.MjvCamera()
                c.lookat[:] = cam.lookat + [dx, 0, 0]
                (c.distance, c.azimuth, c.elevation) = (cam.distance, cam.azimuth, cam.elevation)
                renderer.update_scene(C.d, camera=c, scene_option=opt)
                draw_trail(renderer.scene, trail, (*color[:3], 0.8))
                imgs.append(renderer.render())
            return imgs
        (T_A, T_B) = (z['tf'][-1], z['tb'][-1])
        speedup = x['ms_baseline'] / x['flask_ms']
        title = f"Problem {index}/{len(json.loads((D / 'inventory.json').read_text()))}  ·  {x['env']} #{x['problem']}  |  Difficulty rank {x['row']}/{x['total_pairs']} (ordered by FLASK planning time)"

        def compose_frame(imgs, tA, tB, build):
            cv = Image.new('RGB', (2 * W, HEAD + H + FOOT), (250, 250, 252))
            dr = ImageDraw.Draw(cv)
            dr.text((16, 10), title, font=font(20, True), fill=(30, 30, 30))
            dr.text((16, 44), f'UR5 + Robotiq · {a.robot.upper()} MotionBenchMaker | real-time replay | green ghost: goal pose | red: MuJoCo mesh collision', font=font(15), fill=(90, 90, 90))
            for (k, (img, name, ms, t, T, vc, Qn)) in enumerate([(imgs[0], 'FLASK (independent reconstruction)', x['flask_ms'], tA, T_A, vcA, 'A'), (imgs[1], 'Baseline: VAMP RRTC + simplify + TOPP-RA', x['ms_baseline'], tB, T_B, vcB, 'B')]):
                cv.paste(Image.fromarray(img), (k * W, HEAD))
                color = tuple((int(255 * c) for c in COLORS[Qn][:3]))
                dr.rectangle([k * W, HEAD, k * W + W - 1, HEAD + 34], fill=(255, 255, 255))
                dr.text((k * W + 12, HEAD + 6), f'{name} | Planning {ms:.3f} ms', font=font(17, True), fill=color)
                j = min(int(t * 1000), len(vc) - 1)
                j0 = min(int(max(t - 1 / FPS, 0) * 1000), len(vc) - 1)
                in_collision = not build and vc[j0:j + 1].any()
                finished = t >= T
                white = 'COLLISION' if in_collision else 'GOAL REACHED' if finished else '' if build else 'Executing'
                status_color = (210, 30, 30) if in_collision else (20, 140, 50) if finished else (60, 60, 60)
                y = HEAD + H + 8
                dr.text((k * W + 12, y), f't = {min(t, T):5.2f} / {T:.2f} s', font=font(18), fill=(40, 40, 40))
                dr.text((k * W + 230, y), white, font=font(18, True), fill=status_color)
                dr.text((k * W + 12, y + 26), f"Collision samples over full trajectory: {int(vc.sum())}  |  Length {x['flask_length_rad' if k == 0 else 'baseline_length_rad']:.2f} rad", font=font(13), fill=(110, 110, 110))
            dr.line([(W, HEAD), (W, HEAD + H + FOOT)], fill=(200, 200, 205), width=2)
            if build:
                o = Image.new('RGBA', cv.size, (0, 0, 0, 0))
                od = ImageDraw.Draw(o)
                od.rounded_rectangle([2 * W // 2 - 380, HEAD + 150, 2 * W // 2 + 380, HEAD + 330], 18, fill=(255, 255, 255, 235), outline=(180, 180, 190), width=2)
                cv = Image.alpha_composite(cv.convert('RGBA'), o).convert('RGB')
                dr = ImageDraw.Draw(cv)
                dr.text((W - 360, HEAD + 168), 'PLANNING TIME', font=font(20, True), fill=(40, 40, 40))
                dr.text((W - 360, HEAD + 210), f"FLASK:     {x['flask_ms']:9.3f} ms", font=font(24, True), fill=tuple((int(255 * c) for c in COLORS['A'][:3])))
                dr.text((W - 360, HEAD + 248), f"Baseline:  {x['ms_baseline']:9.3f} ms", font=font(24, True), fill=tuple((int(255 * c) for c in COLORS['B'][:3])))
                dr.text((W - 360, HEAD + 290), (f'FLASK planning speed ratio: {speedup:.0f}x' if speedup >= 1 else f'FLASK planning slowdown ratio: {1 / speedup:.0f}x') + f'  |  Execution duration: {T_A:.1f} s vs {T_B:.1f} s', font=font(17), fill=(60, 60, 60))
            return np.asarray(cv)
        if a.image_only:
            for tt in (0.0, min(T_A, T_B) / 2):
                C.set_state('A', z['Qf'][min(int(tt * 1000), len(z['Qf']) - 1)])
                C.set_state('B', z['Qb'][min(int(tt * 1000), len(z['Qb']) - 1)])
                mujoco.mj_forward(C.m, C.d)
                Image.fromarray(compose_frame(capture(tt, tt), tt, tt, tt == 0.0)).save(D / f'preview_problem_{index:02d}_{tt:.1f}.png')
            continue
        C.set_state('A', z['Qf'][0])
        C.set_state('B', z['Qb'][0])
        mujoco.mj_forward(C.m, C.d)
        f0 = compose_frame(capture(0, 0), 0.0, 0.0, True).tobytes()
        for _ in range(int(PAUSE_S * FPS)):
            ff.stdin.write(f0)
        for t in np.arange(0, max(T_A, T_B) + END_HOLD_S, 1 / FPS):
            C.set_state('A', z['Qf'][min(int(t * 1000), len(z['Qf']) - 1)])
            C.set_state('B', z['Qb'][min(int(t * 1000), len(z['Qb']) - 1)])
            mujoco.mj_forward(C.m, C.d)
            for (name, vc) in (('A', vcA), ('B', vcB)):
                j = min(int(t * 1000), len(vc) - 1)
                j0 = min(int(max(t - 1 / FPS, 0) * 1000), len(vc) - 1)
                C.m.geom_rgba[C.geom_robot[name]] = RED if vc[j0:j + 1].any() else COLORS[name]
            imgs = capture(t, t)
            ff.stdin.write(compose_frame(imgs, t, t, False).tobytes())
    if ff:
        ff.stdin.close()
        ff.wait()
        print(f'-> {output}')
    (D / 'mesh_table.json').write_text(json.dumps(table, indent=1, ensure_ascii=False))
if __name__ == '__main__':
    main()
