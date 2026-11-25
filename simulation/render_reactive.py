"""Render reactive: FLASK implementation and evaluation utilities."""
import argparse
import json
import os
import subprocess
os.environ.setdefault('MUJOCO_GL', 'glfw')
import mujoco
import numpy as np
from PIL import Image, ImageDraw
import render_benchmark as QM
from reactive_mesh import JOINT_NAMES, TUBE_SEGMENTS, tube_quaternion, local_tube_centers
ROOT = QM.ROOT
D = ROOT / 'results/legacy' / 'e2'
(W, H, FPS) = (1280, 720, 30)
(START_HOLD_S, END_HOLD_S) = (1.0, 1.0)
BLUE = (0.16, 0.49, 0.88, 1.0)
RED = (0.9, 0.1, 0.1, 1.0)
TUBE_COLOR = (0.17, 0.18, 0.2, 1.0)
CAM = dict(lookat=(-0.08, 0.55, 1.18), distance=1.85, azimuth=245.0, elevation=-24.0)

def build(cj):
    sp = mujoco.MjSpec()
    sp.visual.global_.offwidth = W
    sp.visual.global_.offheight = H
    sp.visual.quality.shadowsize = 4096
    sp.add_texture(name='troi', type=mujoco.mjtTexture.mjTEXTURE_SKYBOX, builtin=mujoco.mjtBuiltin.mjBUILTIN_GRADIENT, rgb1=[0.95, 0.96, 0.98], rgb2=[0.7, 0.76, 0.84], width=256, height=256)
    sp.add_texture(name='o', type=mujoco.mjtTexture.mjTEXTURE_2D, builtin=mujoco.mjtBuiltin.mjBUILTIN_CHECKER, rgb1=[0.86, 0.87, 0.89], rgb2=[0.8, 0.81, 0.84], width=300, height=300)
    mat = sp.add_material(name='san', texrepeat=[8, 8], reflectance=0.05)
    mat.textures[mujoco.mjtTextureRole.mjTEXROLE_RGB] = 'o'
    wb = sp.worldbody
    wb.add_light(pos=[1.0, -1.5, 3.5], dir=[-0.3, 0.4, -1], diffuse=[0.55, 0.55, 0.55], castshadow=True)
    wb.add_geom(name='san', type=mujoco.mjtGeom.mjGEOM_PLANE, size=[4, 4, 0.1], material='san', contype=0, conaffinity=0)
    wb.add_geom(name='chan', type=mujoco.mjtGeom.mjGEOM_CYLINDER, size=[0.07, 0.4572, 0], pos=[0, 0, 0.4572], rgba=[0.35, 0.35, 0.38, 1], contype=0, conaffinity=0)
    for o in cj['edge']['box']:
        wb.add_geom(name=f"vc_{o['name']}", type=mujoco.mjtGeom.mjGEOM_BOX, size=o['half_extents'], pos=o['position'], quat=QM.quat_wxyz(o['orientation_quat_xyzw']), rgba=[0.62, 0.68, 0.76, 0.75], contype=0, conaffinity=0)
    tube = wb.add_body(name='tube', mocap=True)
    C = local_tube_centers(cj['tube'], TUBE_SEGMENTS)
    for i in range(TUBE_SEGMENTS):
        tube.add_geom(name=f'ong_{i}', type=mujoco.mjtGeom.mjGEOM_CAPSULE, size=[cj['tube']['r'], 0, 0], fromto=np.concatenate([C[i], C[i + 1]]), rgba=TUBE_COLOR, contype=0, conaffinity=0)
    urdf = QM.convert_urdf('ur5', D / '_mesh')
    for pre in ('A', 'G'):
        sp.attach(mujoco.MjSpec.from_string(urdf), prefix=f'{pre}_', frame=wb.add_frame(pos=[0, 0, 0]))
    for g in sp.geoms:
        b = g.parent.name if g.parent is not None else ''
        if b.startswith('G_'):
            g.contype = 0
            g.conaffinity = 0
            g.rgba = [0.2, 0.75, 0.3, 0.22]
        elif b.startswith('A_'):
            g.rgba = BLUE
    m = sp.compile()
    m.vis.headlight.ambient[:] = 0.35
    m.vis.headlight.diffuse[:] = 0.45
    return m

def label(img, planning_us, t):
    im = Image.fromarray(img).convert('RGBA')
    o = Image.new('RGBA', im.size, (0, 0, 0, 0))
    dr = ImageDraw.Draw(o)
    dr.rounded_rectangle([14, 14, 300, 92], 10, fill=(255, 255, 255, 215))
    im = Image.alpha_composite(im, o)
    dr = ImageDraw.Draw(im)
    c = tuple((int(255 * v) for v in BLUE[:3]))
    s = '--' if planning_us is None else f'{planning_us:.0f} µs' if planning_us >= 10 else f'{planning_us:.1f} µs'
    dr.text((28, 20), f'Planning: {s}', font=QM.font(26, True), fill=c)
    dr.text((28, 54), f'Execution: {t:5.1f} s', font=QM.font(26, True), fill=c)
    return np.asarray(im.convert('RGB'))

def main():
    from flask_repro.verification_gate import require_verified_reproduction
    require_verified_reproduction('Legacy showcase rendering')
    ap = argparse.ArgumentParser()
    ap.add_argument('--trial', type=int, default=0)
    ap.add_argument('--image_only', action='store_true', help='Write selected preview frames')
    ap.add_argument('--cam', type=float, nargs=4, help='lookat_y distance azimuth elevation')
    a = ap.parse_args()
    if a.cam:
        CAM.update(lookat=(CAM['lookat'][0], a.cam[0], CAM['lookat'][2]), distance=a.cam[1], azimuth=a.cam[2], elevation=a.cam[3])
    cj = json.loads((D / 'edge.json').read_text())
    qs = np.asarray(cj['qs'])
    z = np.load(D / f'trial_{a.trial:02d}.npz')
    (t, Q, X) = (z['t'], z['Qt'].astype(float), z['tube_pose'].astype(float))
    tc = z['cycle_time_s']
    goal = z['cycle_goal_index'].astype(int)
    iterations = np.where(z['cycle_iterations'] > 0, (z['cycle_planning_ns'] + np.nan_to_num(z['cycle_shortcut_ns'])) / 1000.0, np.nan)
    fm = D / f'trial_{a.trial:02d}_tracking_mesh.npy'
    collisions = np.load(fm).any(axis=0) if fm.exists() else np.zeros(len(t), bool)
    print(f'trial {a.trial}: {len(t)} color, color mesh collisions {int(collisions.sum())}', flush=True)
    m = build(cj)
    d = mujoco.MjData(m)
    adr = {p: [m.joint(f'{p}_{j}').qposadr[0] for j in JOINT_NAMES] for p in ('A', 'G')}
    body_names = [m.body(i).name for i in range(m.nbody)]
    g_robot = np.array([i for i in range(m.ngeom) if body_names[m.geom_bodyid[i]].startswith('A_')])
    mid = m.body('tube').mocapid[0]
    cam = mujoco.MjvCamera()
    cam.lookat[:] = CAM['lookat']
    (cam.distance, cam.azimuth, cam.elevation) = (CAM['distance'], CAM['azimuth'], CAM['elevation'])
    R = mujoco.Renderer(m, height=H, width=W, max_geom=20000)
    opt = mujoco.MjvOption()
    end_time = float(tc[-1] + 0.05)

    def compose_frame(tt):
        j = min(np.searchsorted(t, tt), len(t) - 1)
        j0 = min(np.searchsorted(t, tt - 1 / FPS), len(t) - 1)
        k = max(np.searchsorted(tc, tt, side='right') - 1, 0)
        d.qpos[adr['A']] = Q[j]
        d.qpos[adr['G']] = qs[goal[k]]
        d.mocap_pos[mid] = X[j, :3]
        d.mocap_quat[mid] = tube_quaternion(X[j, 3], X[j, 4])
        m.geom_rgba[g_robot] = RED if tt > 0 and collisions[j0:j + 1].any() else BLUE
        mujoco.mj_forward(m, d)
        R.update_scene(d, camera=cam, scene_option=opt)
        kk = np.flatnonzero(~np.isnan(iterations[:k + 1]))
        return label(R.render(), float(iterations[kk[-1]]) if len(kk) else None, min(tt, end_time))
    if a.image_only:
        for tt in (0.0, 6.0, 25.0, 48.0, 92.0):
            Image.fromarray(compose_frame(tt)).save(D / f'preview_reactive_{tt:.0f}.png')
        print('Preview images saved to ', D)
        return
    output = D / f'reactive_trial{a.trial:02d}.mp4'
    ff = subprocess.Popen(['ffmpeg', '-loglevel', 'error', '-y', '-f', 'rawvideo', '-pix_fmt', 'rgb24', '-s', f'{W}x{H}', '-r', str(FPS), '-i', '-', '-c:v', 'libx264', '-pix_fmt', 'yuv420p', '-crf', '20', str(output)], stdin=subprocess.PIPE)
    f0 = compose_frame(0.0).tobytes()
    for _ in range(int(START_HOLD_S * FPS)):
        ff.stdin.write(f0)
    for (i, tt) in enumerate(np.arange(0, end_time + END_HOLD_S, 1 / FPS)):
        ff.stdin.write(compose_frame(tt).tobytes())
        if i % 600 == 0:
            print(f'  {tt:5.1f} s', flush=True)
    ff.stdin.close()
    ff.wait()
    print(f'-> {output}')
if __name__ == '__main__':
    main()
