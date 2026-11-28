"""Render the Fetch warehouse showcase from its log (state replay only). Videos: no ghosts; stills: ghosts.

    python -m simulation.showcase.render_fetch --run showcase/showcase_1/fetch/seed_1 --preset delivery
"""
import argparse
import json
from pathlib import Path

import mujoco
import numpy as np
from PIL import Image

from simulation.showcase import fetch_warehouse as S
from simulation.showcase.render_core import BLUE, ORANGE, GhostPainter, Hud, Writer, add_polyline, free_camera

PRESETS = dict(master=(3840, 2160, 30), delivery=(1920, 1080, 60), preview=(1280, 720, 30))
FADED = (1.0, 0.72, 0.45, 0.45)
LABEL = {0: 'drive to station', 1: 'reach item 1', 2: 'reach item 2', 3: 'tuck arm', 4: 'drive to dock'}


def fmt(ms):
    return f'{ms * 1000:.0f}µs' if ms < 1 else f'{ms:.1f}ms' if ms < 100 else f'{ms:.0f}ms'


class FetchRenderer:
    def __init__(self, run, width, height, hud=True, inset=True):
        self.run = Path(run)
        self.log = dict(np.load(self.run / 'execution.npz'))
        self.plans = json.loads((self.run / 'plans.json').read_text())
        self.model = S.build_model(); self.data = mujoco.MjData(self.model)
        self.w, self.h = width, height
        self.main = mujoco.Renderer(self.model, height, width, max_geom=30000)
        iw = int(width * 0.26); ih = int(iw * 8 / 12)
        self.small = mujoco.Renderer(self.model, ih, iw, max_geom=30000)
        self.ghosts = GhostPainter(self.model)
        self.hud = Hud(width) if hud else None; self.show_inset = inset
        self.cam = None
        self.torso = self.model.joint('torso_lift_joint').qposadr[0]
        self.grip = self.model.body('gripper_link').id

    def state(self, t):
        T = self.log['time']; i = int(np.clip(np.searchsorted(T, t), 0, len(T) - 1))
        self.data.qpos[:] = self.log['qpos'][i]; mujoco.mj_forward(self.model, self.data)
        return i

    def decorate(self, scene, i, t, ghosts=False, width=1.0):
        step = int(self.log['step'][i])
        for p in self.plans[:step + 1]:
            current = p['step'] == step
            if p['kind'] == 'base':
                pts = np.c_[np.asarray(p['path']), np.full(len(p['path']), 0.03)]
                add_polyline(scene, pts, ORANGE if current else FADED, 0.035 * width)
            else:
                add_polyline(scene, p['ee_path'], ORANGE if current else FADED, (0.012 if current else 0.007) * width, min_step=0.005)
        trail = self.log['qpos'][:i + 1, :2]
        if len(trail) > 1:
            add_polyline(scene, np.c_[trail, np.full(len(trail), 0.05)], BLUE, 0.022 * width, min_step=0.05)
        if ghosts:
            for k, tg in enumerate(np.arange(t - 1.6, max(t - 13, 0), -1.6)):
                j = int(np.searchsorted(self.log['time'], tg))
                self.ghosts.add(scene, self.log['qpos'][j], alpha=max(0.12, 0.40 - 0.035 * k))

    def camera(self, t, i, still=False):
        x, y, th = self.log['qpos'][i, :3]
        if self.log['phase'][i] == 'arm':        # close-up on the arm, from the robot's front-left
            target = np.array([x + 0.55, y, 0.95, 3.0, np.degrees(th) + 205, -30])
        else:
            target = np.array([x, y, 0.8, 6.8, -60, -32])
        if still or self.cam is None:
            self.cam = target
        else:
            d = target - self.cam; d[4] = (d[4] + 180) % 360 - 180
            self.cam = self.cam + (1 - np.exp(-max(t - self.last_t, 0) / 1.0)) * d
        c = self.cam
        return free_camera(c[:3], c[3], c[4], c[5])

    last_t = 0.
    compact = False   # montage tiles: planning time only

    def frame(self, t, still=False):
        i = self.state(t)
        self.main.update_scene(self.data, self.camera(t, i, still)); self.decorate(self.main.scene, i, t, ghosts=still)
        img = Image.fromarray(self.main.render()); self.last_t = t
        if self.show_inset and not self.compact:
            self.small.update_scene(self.data, 'top'); self.decorate(self.small.scene, i, t, width=2.2)
            inset = self.small.render()
        if self.hud:
            step = int(self.log['step'][i]); p = self.plans[step]
            ms = p['plan_ms'] if p['kind'] == 'base' else p['plan_us'] / 1e3
            hist = [(q['plan_ms'] if q['kind'] == 'base' else q['plan_us'] / 1e3) for q in self.plans[:step + 1]]
            post = p['post_ms'] if p['kind'] == 'base' else p['shortcut_us'] / 1e3
            self.hud.plan_callout(img, ms, hist, label=f'FLASK {"base" if p["kind"] == "base" else "arm"} planning',
                                  flash=float(np.clip(1 - (t - p['t_start']) / 0.8, 0, 1)),
                                  detail=f'+ smoothing {post * 1000:.0f} µs' if post < 1 else f'+ smoothing {post:.1f} ms')
            v = np.linalg.norm(np.gradient(self.log['qpos'][:, :2], self.log['time'], axis=0)[i])
            if not self.compact:
                self.hud.rows(img, [('time', f'{t:5.1f} s  ({self.SPEED:g}× speed)'), ('task', f'{step + 1}/{len(LABEL)}  {LABEL[step]}'),
                                    ('base speed', f'{v:4.2f} m/s'), ('torso lift', f'{100 * self.data.qpos[self.torso]:4.1f} cm'),
                                    ('gripper height', f'{self.data.xpos[self.grip][2]:4.2f} m'),
                                    ('base error' if p['kind'] == 'base' else 'arm error',
                                     f'{100 * self.log["error"][i]:4.1f} cm' if p['kind'] == 'base' else f'{np.degrees(self.log["error"][i]):4.1f}°'),
                                ('localization', f'{100 * np.linalg.norm(self.log["estimate"][i][:2] - self.log["qpos"][i][:2]):4.1f} cm (SLAM + IMU)')])
            if not self.compact:
                self.hud.legend(img, [(ORANGE, 'FLASK plan (base path / gripper path)'), (BLUE, 'executed base path')])
            if self.show_inset and not self.compact:
                self.hud.inset(img, inset, 'top view')
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
    r = FetchRenderer(args.run, w, h, hud=not args.clean, inset=not args.clean)
    out = args.run / 'media'; out.mkdir(exist_ok=True)
    if args.stills:
        for t in args.stills:
            Image.fromarray(r.frame(t, still=True)).save(out / f'still_{args.preset}_{t:05.1f}.png')
    else:
        r.video(out / f'{args.preset}{"_clean" if args.clean else ""}.mp4', fps)
