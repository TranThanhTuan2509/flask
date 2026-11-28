"""Render the multi-drone showcase (showcase_2) from its log: state replay only.

    python -m simulation.showcase.render_swarm --run showcase/showcase_2/multidrone/seed_1 --preset delivery
"""
import argparse
import json
from pathlib import Path

import mujoco
import numpy as np
from PIL import Image

from simulation.showcase import multidrone_swap as S
from simulation.showcase.render_core import Hud, Writer, add_polyline, free_camera

PRESETS = dict(master=(3840, 2160, 30), delivery=(1920, 1080, 60), preview=(1280, 720, 30))


class SwarmRenderer:
    compact = False
    SPEED = 1.0
    cam_dist = None

    def __init__(self, run, width, height, hud=True, inset=True):
        self.run = Path(run)
        self.log = dict(np.load(self.run / 'execution.npz'))
        self.plans = json.loads((self.run / 'plans.json').read_text())
        self.summary = json.loads((self.run / 'summary.json').read_text())
        self.names = [str(n) for n in self.log['names']]
        self.model = S.build_model(); self.data = mujoco.MjData(self.model)
        self.q = {n: self.model.joint(f'{n}/aerial_free').qposadr[0] for n in self.names}
        self.w, self.h = width, height
        self.main = mujoco.Renderer(self.model, height, width, max_geom=20000)
        iw = int(width * 0.25); ih = int(iw * 0.62)
        self.small = mujoco.Renderer(self.model, ih, iw, max_geom=20000)
        self.hud = Hud(width) if hud else None; self.show_inset = inset
        self.opt = mujoco.MjvOption(); self.opt.sitegroup[:] = 1
        self.pos = {n: self.log['qpos'][:, self.q[n]:self.q[n] + 3] for n in self.names}
        P = np.stack([self.pos[n] for n in self.names], 1)
        d = [np.linalg.norm(P[:, i] - P[:, j], axis=1) for i in range(3) for j in range(i + 1, 3)]
        self.min_sep = np.minimum.accumulate(np.min(d, axis=0))
        self.color = {n: S.DRONES[n][2] for n in self.names}

    def state(self, t):
        T = self.log['time']; i = int(np.clip(np.searchsorted(T, t), 0, len(T) - 1))
        self.data.qpos[:] = self.log['qpos'][i]; mujoco.mj_forward(self.model, self.data)
        return i

    def decorate(self, scene, i, width=1.0):
        for p in self.plans:
            c = self.color[p['drone']]
            add_polyline(scene, p['path'], (c[0], c[1], c[2], 0.55), 0.03 * width, min_step=0.05)
        for n in self.names:
            c = self.color[n]
            trail = self.pos[n][max(0, i - 150):i + 1]
            if len(trail) > 1:
                add_polyline(scene, trail, (min(1, c[0] + .25), min(1, c[1] + .25), min(1, c[2] + .25), 0.95), 0.045 * width, min_step=0.04)

    def frame(self, t, still=False):
        i = self.state(t)
        lookat, dist, az, el = S.CAMERA
        cam = free_camera(lookat, self.cam_dist or dist, az + 10 * np.sin(t / 9), el)
        self.main.update_scene(self.data, cam, self.opt); self.decorate(self.main.scene, i)
        img = Image.fromarray(self.main.render())
        if self.show_inset and not self.compact:
            self.small.update_scene(self.data, 'top', self.opt); self.decorate(self.small.scene, i, width=1.4)
            inset = self.small.render()
        if self.hud:
            total = sum(p['plan_ms'] + p['post_ms'] for p in self.plans)
            self.hud.plan_callout(img, total, None if self.compact else [p['plan_ms'] + p['post_ms'] for p in self.plans],
                                  label='FLASK planning time · 3 drones', flash=float(np.clip(1 - t / 0.8, 0, 1)))
            if not self.compact:
                rows = [('time', f'{t:5.1f} s' + (f'  ({self.SPEED:g}× speed)' if self.SPEED != 1 else ''))]
                for p in self.plans:
                    rows.append((f'#{p["priority"]} {p["drone"]}', f'{p["plan_ms"] + p["post_ms"]:6.1f} ms  {p["duration"]:4.1f} s flight'))
                rows.append(('min distance', f'{self.min_sep[i]:4.2f} m  (contact < 0.74 m)'))
                self.hud.rows(img, rows)
                self.hud.legend(img, [(self.color[n], f'{n} (priority {self.names.index(n) + 1}) plan') for n in self.names])
                if self.show_inset:
                    self.hud.inset(img, inset, 'top view')
        return np.asarray(img)

    def video(self, path, fps):
        end = float(self.log['time'][-1]); writer = Writer(path, self.w, self.h, fps)
        for t in np.arange(0, end, self.SPEED / fps):
            writer.write(self.frame(t))
        writer.close()


if __name__ == '__main__':
    a = argparse.ArgumentParser()
    a.add_argument('--run', type=Path, required=True); a.add_argument('--preset', choices=PRESETS, default='preview')
    a.add_argument('--stills', type=float, nargs='*'); a.add_argument('--clean', action='store_true')
    args = a.parse_args(); w, h, fps = PRESETS[args.preset]
    r = SwarmRenderer(args.run, w, h, hud=not args.clean, inset=not args.clean)
    out = args.run / 'media'; out.mkdir(exist_ok=True)
    if args.stills:
        for t in args.stills:
            Image.fromarray(r.frame(t, still=True)).save(out / f'still_{args.preset}_{t:05.2f}.png')
    else:
        r.video(out / f'{args.preset}{"_clean" if args.clean else ""}.mp4', fps)
