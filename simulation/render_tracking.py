"""Render tracking: FLASK implementation and evaluation utilities."""
import argparse
import json
import subprocess
import numpy as np
from PIL import Image
import render_grid as Q4
import mujoco
ROOT = Q4.ROOT
(W, H, FPS) = (Q4.W, Q4.H, Q4.FPS)
CAM = dict(lookat=(0.0, 0.35, 0.95), distance=2.5, azimuth=255.0, elevation=-42.0)

def camera_e1(dx):
    c = mujoco.MjvCamera()
    c.lookat[:] = np.add(CAM['lookat'], [dx, 0, 0])
    (c.distance, c.azimuth, c.elevation) = (CAM['distance'], CAM['azimuth'], CAM['elevation'])
    return c

def pick_index(t, Qd):
    """Pick index."""
    build = np.where(np.all(np.abs(np.diff(Qd, axis=0)) < 1e-12, axis=1))[0]
    build = build[t[build] > 0.05]
    return build[0] if len(build) else len(Qd) - 1

def main():
    from flask_repro.verification_gate import require_verified_reproduction
    require_verified_reproduction('Legacy showcase rendering')
    ap = argparse.ArgumentParser()
    ap.add_argument('--mode', default='original_j10_Kp5')
    ap.add_argument('--seed', type=int, nargs='+', default=[0, 1, 2, 16])
    ap.add_argument('--cam', type=float, nargs=4, help='lookat_y distance azimuth elevation (thu camera)')
    ap.add_argument('--image_only', action='store_true', help='Write preview images for the first cycle')
    a = ap.parse_args()
    if a.cam:
        CAM.update(lookat=(0.0, a.cam[0], 0.95), distance=a.cam[1], azimuth=a.cam[2], elevation=a.cam[3])
    D = ROOT / 'results/legacy' / 'e1'
    DM = D / a.mode
    vc = json.loads((D / 'edge.json').read_text())
    total = {x['seed']: x for x in json.loads((DM / 'summary.json').read_text())}
    cycle = [a.seed[i:i + 2] for i in range(0, len(a.seed), 2)]
    output = DM / f'video_e1_{a.mode}_{len(a.seed)}seed_4o.mp4'
    ff = None if a.image_only else subprocess.Popen(['ffmpeg', '-loglevel', 'error', '-y', '-f', 'rawvideo', '-pix_fmt', 'rgb24', '-s', f'{2 * W}x{2 * H}', '-r', str(FPS), '-i', '-', '-c:v', 'libx264', '-pix_fmt', 'yuv420p', '-crf', '20', str(output)], stdin=subprocess.PIPE)
    renderer = None
    opt = mujoco.MjvOption()
    table = []
    for L in cycle:
        (copies, row) = ([], [])
        for (r, s) in enumerate(L):
            x = total[s]
            (dF, dB) = (2 * r * Q4.SEPARATION, (2 * r + 1) * Q4.SEPARATION)
            copies += [(f'F{r}', dF, vc, 'F'), (f'B{r}', dB, vc, 'B')]
            h = dict(x=x, dF=dF, dB=dB)
            for (k, name) in (('F', 'flask'), ('B', 'geometric')):
                z = np.load(DM / f"trial_{x['trial']:02d}_{name}.npz")
                ip = pick_index(z['t'], z['Qd'])
                h[k] = dict(t=z['t'], Q=z['Qt'], goal=(z['t'][ip], z['Qd'][ip], z['Qd'][-1]), ms=x[f'ms_{name}'], T=x[name]['T'])
            row.append(h)
        C = Q4.FourthOrderEdge('ur5', copies, D / '_mesh')
        if renderer is not None:
            renderer.close()
        renderer = mujoco.Renderer(C.m, height=H, width=W, max_geom=20000)
        for (r, h) in enumerate(row):
            for k in ('F', 'B'):
                (h[k]['vc'], h[k]['trail']) = C.scan(f'{k}{r}', h[k]['Q'])
            h['F']['cam'] = camera_e1(h['dF'])
            h['B']['cam'] = camera_e1(h['dB'])
            x = h['x']
            table.append(dict(seed=x['seed'], flask_ms=x['flask_ms'], geometric_ms=x['geometric_ms'], T=x['flask']['T'], flask_mesh_collision_samples=int(h['F']['vc'].sum()), geometric_mesh_collision_samples=int(h['B']['vc'].sum())))
            print(f"seed {x['seed']:2d}: iterations {x['flask_ms']:.3f} vs {x['geometric_ms']:.3f} ms | run {x['flask']['T']:.1f} s | color mesh collisions actual_state {h['F']['vc'].sum()} / {h['B']['vc'].sum()}", flush=True)
        cycle_duration = max((h[k]['t'][-1] for h in row for k in ('F', 'B')))

        def compose_frame(tt):
            o = []
            for r in range(2):
                if r >= len(row):
                    o.append(np.full((H, 2 * W, 3), 245, np.uint8))
                    continue
                h = row[r]
                image_only = []
                for k in ('F', 'B'):
                    g = h[k]
                    name = f'{k}{r}'
                    t = g['t']
                    j = min(np.searchsorted(t, tt), len(t) - 1)
                    j0 = min(np.searchsorted(t, tt - 1 / FPS), len(t) - 1)
                    C.set_state(name, g['Q'][j])
                    C.m.geom_rgba[C.geom_robot[name]] = Q4.RED if tt > 0 and g['vc'][j0:j + 1].any() else Q4.COLORS[k]
                    (pick_time, pick_configuration, place_configuration) = g['goal']
                    C.set_state('G' + name, pick_configuration if tt < pick_time else place_configuration)
                    C.m.geom_rgba[C.ghost_geoms[name], 3] = 0.0 if j >= len(t) - 1 else 0.22
                mujoco.mj_forward(C.m, C.d)
                for k in ('F', 'B'):
                    g = h[k]
                    renderer.update_scene(C.d, camera=g['cam'], scene_option=opt)
                    Q4.QM.draw_trail(renderer.scene, g['trail'], (*Q4.COLORS[k][:3], 0.8))
                    image_only.append(Q4.label(renderer.render(), g['ms'], g['T'], Q4.COLORS[k]))
                image_row = np.hstack(image_only)
                image_row[:, W - 1:W + 1] = 200
                o.append(image_row)
            out = np.vstack(o)
            out[H - 1:H + 1, :] = 120
            return out
        if a.image_only:
            for tt in (0.0, cycle_duration / 2):
                Image.fromarray(compose_frame(tt)).save(DM / f'preview_tracking_{tt:.1f}.png')
            break
        f0 = compose_frame(0.0).tobytes()
        for _ in range(int(Q4.START_HOLD_S * FPS)):
            ff.stdin.write(f0)
        for tt in np.arange(0, cycle_duration + Q4.END_HOLD_S, 1 / FPS):
            ff.stdin.write(compose_frame(tt).tobytes())
    if ff:
        ff.stdin.close()
        ff.wait()
        print(f'-> {output}')
        (DM / f'bang_video_e1_{len(a.seed)}seed.json').write_text(json.dumps(table, indent=1, ensure_ascii=False))
if __name__ == '__main__':
    main()
