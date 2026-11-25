"""Render the drone-hall showcase from its log (state replay only). Videos: no ghosts; stills: ghosts.

    python -m simulation.showcase.render_drone --run showcase/showcase_1/drone/seed_1 --preset delivery
"""
import argparse
import json
from pathlib import Path

import mujoco
import numpy as np
from PIL import Image
from scipy.spatial.transform import Rotation

from simulation.showcase import drone_hall as S
from simulation.showcase.render_core import (BLUE, ORANGE, GhostPainter, Hud, Writer, add_frame, add_polyline,
                                             free_camera)

PRESETS = dict(master=(3840, 2160, 30), delivery=(1920, 1080, 60), preview=(1280, 720, 30))
FADED = (1.0, 0.72, 0.45, 0.45)


class DroneRenderer:
    def __init__(self, run, width, height, hud=True, inset=True):
        self.run = Path(run)
        self.log = dict(np.load(self.run / 'execution.npz'))
        self.plans = json.loads((self.run / 'plans.json').read_text())
        self.model = S.build_model(); self.data = mujoco.MjData(self.model)
        self.w, self.h = width, height
        self.main = mujoco.Renderer(self.model, height, width, max_geom=30000)
        iw = int(width * 0.27); ih = int(iw * 9 / 16)
        self.small = mujoco.Renderer(self.model, ih, iw, max_geom=30000)
        self.ghosts = GhostPainter(self.model)
        self.hud = Hud(width) if hud else None; self.show_inset = inset
        self.look = None
        self.opt = mujoco.MjvOption(); self.opt.sitegroup[:] = 1

    def state(self, t):
        T = self.log['time']; i = int(np.clip(np.searchsorted(T, t), 0, len(T) - 1))
        self.data.qpos[:] = self.log['qpos'][i]; mujoco.mj_forward(self.model, self.data)
        return i

    def decorate(self, scene, i, t, ghosts=False, fpv=False):
        leg = int(self.log['leg'][i])
        for p in self.plans[:leg + 1]:
            current = p['leg'] == leg
            add_polyline(scene, p['path'], ORANGE if current else FADED, 0.03 if current else 0.02)
            if current and not fpv:
                for pos, rot in zip(p['frames_pos'], p['frames_rot']):
                    add_frame(scene, np.asarray(pos), np.asarray(rot), 0.22, 0.012)
        if not fpv:
            trail = self.log['qpos'][:i + 1, :3]
            if len(trail) > 1:
                add_polyline(scene, trail, BLUE, 0.014, min_step=0.04)
        if ghosts:
            for k, tg in enumerate(np.arange(t - 0.35, max(t - 4.6, 0), -0.35)):
                j = int(np.searchsorted(self.log['time'], tg))
                self.ghosts.add(scene, self.log['qpos'][j], alpha=max(0.15, 0.55 - 0.035 * k))

    def camera(self, t, i, still=False):
        q = self.log['qpos'][i]
        target = np.array([q[0], 0.5 * q[1] + 2.0, 0.5 * q[2] + 1.0])  # follow the drone, damped across the hall
        a = 1 - np.exp(-max(t - self.last_t, 0) / 1.5)
        self.look = target if (still or self.look is None) else self.look + a * (target - self.look)
        return free_camera(self.look, self.cam_dist, 90 + 14 * np.sin(t / 13), -16)

    last_t = 0.
    cam_dist = 8.0
    compact = False   # montage tiles: planning time only

    def frame(self, t, still=False):
        i = self.state(t)
        self.main.update_scene(self.data, self.camera(t, i, still), self.opt)
        self.decorate(self.main.scene, i, t, ghosts=still)
        img = Image.fromarray(self.main.render()); self.last_t = t
        if self.show_inset and not self.compact:
            self.small.update_scene(self.data, 'fpv', self.opt); self.decorate(self.small.scene, i, t, fpv=True)
            inset = self.small.render()
        if self.hud:
            leg = int(self.log['leg'][i]); p = self.plans[leg]
            q = self.log['qpos'][i]; vel = np.linalg.norm(np.gradient(self.log['qpos'][:, :3], self.log['time'], axis=0)[i])
            R = Rotation.from_quat(q[[4, 5, 6, 3]]).as_matrix(); tilt = np.degrees(np.arccos(np.clip(R[2, 2], -1, 1)))
            self.hud.plan_callout(img, p['plan_ms'], [x['plan_ms'] for x in self.plans[:leg + 1]],
                                  label='FLASK planning time', flash=float(np.clip(1 - (t - p['t_start']) / 0.8, 0, 1)),
                                  detail=f'+ smoothing {p["post_ms"]:.1f} ms · interval-certified')
            if not self.compact:
                self.hud.rows(img, [('time', f'{t:5.1f} s  ({self.SPEED:g}× speed)'), ('leg', f'{leg + 1}/{len(self.plans)}'),
                                    ('speed', f'{vel:4.2f} m/s'), ('altitude', f'{q[2]:4.2f} m'), ('tilt', f'{tilt:4.1f}°'),
                                    ('thrust', f'{self.log["thrust"][i]:4.2f} g'), ('error', f'{100 * self.log["error"][i]:4.1f} cm'),
                                ('state est.', f'{1000 * self.log["est_error"][i]:4.1f} mm (mocap + gyro)')])
            if not self.compact:
                self.hud.legend(img, [(ORANGE, 'FLASK plan (full quadrotor model)'), (BLUE, 'flown')])
            if self.show_inset and not self.compact:
                self.hud.inset(img, inset, 'onboard camera')
        return np.asarray(img)

    SPEED = 2.0   # playback speed; shown on screen

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
    r = DroneRenderer(args.run, w, h, hud=not args.clean, inset=not args.clean)
    out = args.run / 'media'; out.mkdir(exist_ok=True)
    if args.stills:
        for t in args.stills:
            Image.fromarray(r.frame(t, still=True)).save(out / f'still_{args.preset}_{t:05.1f}.png')
    else:
        r.video(out / f'{args.preset}{"_clean" if args.clean else ""}.mp4', fps)
