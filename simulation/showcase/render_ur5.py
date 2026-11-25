"""Render the UR5 sorting-cell showcase from its log (state replay only). Videos: no ghosts; stills: ghosts.

    python -m simulation.showcase.render_ur5 --run showcase/showcase_1/ur5/seed_1 --preset delivery
"""
import argparse
import json
from pathlib import Path

import mujoco
import numpy as np
from PIL import Image

from simulation.showcase import ur5_cell as S
from simulation.showcase.render_core import BLUE, ORANGE, GhostPainter, Hud, Writer, add_polyline, free_camera

PRESETS = dict(master=(3840, 2160, 30), delivery=(1920, 1080, 60), preview=(1280, 720, 30))
FADED = (1.0, 0.72, 0.45, 0.40)
TORQUE = np.array([150., 150., 150., 28., 28., 28.])


class UR5Renderer:
    compact = False   # montage tiles: planning time only
    SPEED = 1.0   # playback speed; shown on screen when != 1
    def __init__(self, run, width, height, hud=True, inset=True):
        self.run = Path(run)
        self.log = dict(np.load(self.run / 'execution.npz'))
        self.plans = json.loads((self.run / 'plans.json').read_text())
        self.model = S.build_model(S.scene()); self.data = mujoco.MjData(self.model)
        self.n = self.log['q'].shape[1]
        self.w, self.h = width, height
        self.main = mujoco.Renderer(self.model, height, width, max_geom=30000)
        iw = int(width * 0.25); ih = int(iw * 3 / 4)
        self.small = mujoco.Renderer(self.model, ih, iw, max_geom=30000)
        self.ghosts = GhostPainter(self.model)
        self.hud = Hud(width) if hud else None; self.show_inset = inset
        self.opt = mujoco.MjvOption(); self.opt.sitegroup[:] = 1

    def qpos(self, q):
        full = self.data.qpos.copy(); full[:self.n] = q
        return full

    def state(self, t):
        T = self.log['time']; i = int(np.clip(np.searchsorted(T, t), 0, len(T) - 1))
        self.data.qpos[:self.n] = self.log['q'][i]; mujoco.mj_forward(self.model, self.data)
        return i

    def decorate(self, scene, i, t, ghosts=False, width=1.0):
        leg = int(self.log['leg'][i])
        for p in self.plans[:leg + 1]:
            current = p['leg'] == leg
            add_polyline(scene, p['ee_path'], ORANGE if current else FADED, (0.009 if current else 0.006) * width, min_step=0.004)
        trail = self.log['ee'][:i + 1]
        if len(trail) > 1:
            add_polyline(scene, trail, BLUE, 0.005 * width, min_step=0.004)
        if ghosts:
            for k, tg in enumerate(np.arange(t - 0.25, max(t - 2.6, 0), -0.25)):
                j = int(np.searchsorted(self.log['time'], tg))
                self.ghosts.add(scene, self.qpos(self.log['q'][j]), alpha=max(0.12, 0.42 - 0.04 * k))

    def frame(self, t, still=False):
        i = self.state(t)
        cam = free_camera([0.02, 0.45, 1.05], 2.25, 148 + 8 * np.sin(t / 12), -27)
        self.main.update_scene(self.data, cam, self.opt); self.decorate(self.main.scene, i, t, ghosts=still)
        img = Image.fromarray(self.main.render())
        if self.show_inset and not self.compact:
            self.small.update_scene(self.data, 'top', self.opt); self.decorate(self.small.scene, i, t, width=1.6)
            inset = self.small.render()
        if self.hud:
            leg = int(self.log['leg'][i]); p = self.plans[leg]
            dt = np.gradient(self.log['ee'], self.log['time'], axis=0)[i]
            tau = np.max(np.abs(self.log['tau'][i]) / TORQUE) * 100
            q = np.degrees(self.log['q'][i])
            self.hud.plan_callout(img, p['plan_us'] / 1e3, [x['plan_us'] / 1e3 for x in self.plans[:leg + 1]],
                                  label='FLASK planning time', flash=float(np.clip(1 - (t - p['t_start']) / 0.6, 0, 1)),
                                  detail=f'+ smoothing & certification {p["shortcut_us"] / 1e3:.2f} ms')
            if not self.compact:
                self.hud.rows(img, [('time', f'{t:5.2f} s' + (f'  ({self.SPEED:g}× speed)' if self.SPEED != 1 else '')), ('motion', f'{leg + 1}/{len(self.plans)}'),
                                    ('gripper speed', f'{np.linalg.norm(dt):4.2f} m/s'), ('peak torque', f'{tau:3.0f} %'),
                                    ('q1–q3', ' '.join(f'{x:+5.0f}°' for x in q[:3])),
                                    ('q4–q6', ' '.join(f'{x:+5.0f}°' for x in q[3:6]))])
            if not self.compact:
                self.hud.legend(img, [(ORANGE, 'FLASK plan (gripper path)'), (BLUE, 'executed')])
            if self.show_inset and not self.compact:
                self.hud.inset(img, inset, 'top view')
        return np.asarray(img)

    def video(self, path, fps):
        end = float(self.log['time'][-1]); writer = Writer(path, self.w, self.h, fps)
        for t in np.arange(0, end, 1 / fps):
            writer.write(self.frame(t))
        writer.close()


if __name__ == '__main__':
    a = argparse.ArgumentParser()
    a.add_argument('--run', type=Path, required=True); a.add_argument('--preset', choices=PRESETS, default='preview')
    a.add_argument('--stills', type=float, nargs='*'); a.add_argument('--clean', action='store_true')
    args = a.parse_args(); w, h, fps = PRESETS[args.preset]
    r = UR5Renderer(args.run, w, h, hud=not args.clean, inset=not args.clean)
    out = args.run / 'media'; out.mkdir(exist_ok=True)
    if args.stills:
        for t in args.stills:
            Image.fromarray(r.frame(t, still=True)).save(out / f'still_{args.preset}_{t:05.2f}.png')
    else:
        r.video(out / f'{args.preset}{"_clean" if args.clean else ""}.mp4', fps)
