"""Render the manipulator pick-and-place showcases (showcase_2) from their logs: full-state replay only.

    python -m simulation.showcase.render_arms --task ur5_pickplace --run showcase/showcase_2/ur5/seed_1 --preset delivery
"""
import argparse
import importlib
import json
from pathlib import Path

import mujoco
import numpy as np
from PIL import Image

from simulation.showcase.render_core import BLUE, ORANGE, TEAL, GhostPainter, Hud, Writer, add_polyline, add_sphere, free_camera

PRESETS = dict(master=(3840, 2160, 30), delivery=(1920, 1080, 60), preview=(1280, 720, 30))
PLAN_COLORS = [ORANGE, (0.85, 0.25, 0.75, 0.95)]        # one per arm
TRAIL_COLORS = [BLUE, (0.20, 0.70, 0.95, 0.95)]


def _fmt(ms):
    return f'{ms * 1000:.0f} µs' if ms < 1 else f'{ms:.1f} ms'


class ArmsRenderer:
    compact = False
    SPEED = 1.0

    def __init__(self, run, width, height, task, hud=True, inset=True):
        self.run, self.S = Path(run), importlib.import_module(f'simulation.showcase.{task}')
        self.log = dict(np.load(self.run / 'execution.npz'))
        self.plans = json.loads((self.run / 'plans.json').read_text())
        self.model = self.S.build_model(); self.data = mujoco.MjData(self.model)
        self.arms = self.S.ARMS if hasattr(self.S, 'ARMS') else [self.S.ARM]
        self.w, self.h = width, height
        self.main = mujoco.Renderer(self.model, height, width, max_geom=30000)
        iw = int(width * 0.25); ih = int(iw * 3 / 4)
        self.small = mujoco.Renderer(self.model, ih, iw, max_geom=30000)
        self.ghosts = GhostPainter(self.model)
        self.hud = Hud(width) if hud else None; self.show_inset = inset
        self.opt = mujoco.MjvOption(); self.opt.sitegroup[:] = 1
        qadr = [[self.model.joint(j).qposadr[0] for j in a.joints] for a in self.arms]
        self.pinch = [np.array([a.pinch(q[ix]) for q in self.log['qpos']]) for a, ix in zip(self.arms, qadr)]
        self.by_arm = [[p for p in self.plans if p['arm'] == a.prefix.rstrip('/')] for a in self.arms]
        self.targets = self.S.targets() if hasattr(self.S, 'targets') else []

    def state(self, t):
        T = self.log['time']; i = int(np.clip(np.searchsorted(T, t), 0, len(T) - 1))
        self.data.qpos[:] = self.log['qpos'][i]; mujoco.mj_forward(self.model, self.data)
        return i

    def current(self, k, t):
        past = [p for p in self.by_arm[k] if p['t_start'] <= t]
        return past[-1] if past else None

    def decorate(self, scene, i, t, width=1.0, ghosts=False):
        for k in range(len(self.arms)):
            p = self.current(k, t)
            if p is not None:
                active = t <= p['t_start'] + p['duration'] + 0.3
                c = PLAN_COLORS[k] if active else tuple(PLAN_COLORS[k][:3]) + (0.35,)
                add_polyline(scene, p['pinch_path'], c, (0.008 if active else 0.005) * width, min_step=0.003)
            j0 = max(0, i - int(3.0 / (self.log['time'][1] - self.log['time'][0])))
            trail = self.pinch[k][j0:i + 1]
            if len(trail) > 1:
                add_polyline(scene, trail, TRAIL_COLORS[k], 0.004 * width, min_step=0.003)
        for x, y, z in self.targets:
            add_sphere(scene, (x, y, z), 0.008 * width, TEAL)
        if ghosts:
            for g, tg in enumerate(np.arange(t - 0.3, max(t - 2.4, 0), -0.3)):
                j = int(np.searchsorted(self.log['time'], tg))
                self.ghosts.add(scene, self.log['qpos'][j], alpha=max(0.12, 0.40 - 0.05 * g))

    def frame(self, t, still=False):
        i = self.state(t)
        lookat, dist, az, el = self.S.CAMERA
        cam = free_camera(lookat, dist, az + 6 * np.sin(t / 10), el)
        self.main.update_scene(self.data, cam, self.opt); self.decorate(self.main.scene, i, t, ghosts=still)
        img = Image.fromarray(self.main.render())
        if self.show_inset and not self.compact:
            self.small.update_scene(self.data, 'top', self.opt); self.decorate(self.small.scene, i, t, width=1.6)
            inset = self.small.render()
        if self.hud:
            latest = [p for p in self.plans if p['t_start'] <= t]
            p = latest[-1] if latest else self.plans[0]
            label = 'FLASK planning time' if len(self.arms) == 1 else f'FLASK planning time · {p["arm"].upper()}'
            self.hud.plan_callout(img, p['total_us'] / 1e3, [x['total_us'] / 1e3 for x in latest][-6:] if not self.compact else None,
                                  label=label, flash=float(np.clip(1 - (t - p['t_start']) / 0.6, 0, 1)))
            if not self.compact:
                phases = str(self.log['phase'][i]).split('|')
                placed = self.S.placed(self.data) if hasattr(self.S, 'placed') else None
                rows = [('time', f'{t:5.1f} s' + (f'  ({self.SPEED:g}× speed)' if self.SPEED != 1 else ''))]
                for a, ph in zip(self.arms, phases):
                    rows.append((a.prefix.rstrip('/').upper() if len(self.arms) > 1 else 'task', ph))
                if placed is not None:
                    rows.append(('placed', f'{placed[0]}/{placed[1]}'))
                rows.append(('motions planned', f'{len(latest)}  ({sum(x["total_us"] for x in latest) / 1e3:.1f} ms total)'))
                self.hud.rows(img, rows)
                items = [(PLAN_COLORS[k], f'FLASK plan{" " + a.prefix.rstrip("/").upper() if len(self.arms) > 1 else ""} (gripper path)')
                         for k, a in enumerate(self.arms)] + [(BLUE, 'executed')]
                self.hud.legend(img, items)
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
    a.add_argument('--task', required=True); a.add_argument('--run', type=Path, required=True)
    a.add_argument('--preset', choices=PRESETS, default='preview'); a.add_argument('--stills', type=float, nargs='*')
    a.add_argument('--clean', action='store_true'); a.add_argument('--speed', type=float, default=1.0)
    args = a.parse_args(); w, h, fps = PRESETS[args.preset]
    r = ArmsRenderer(args.run, w, h, args.task, hud=not args.clean, inset=not args.clean); r.SPEED = args.speed
    out = args.run / 'media'; out.mkdir(exist_ok=True)
    if args.stills:
        for t in args.stills:
            Image.fromarray(r.frame(t, still=True)).save(out / f'still_{args.preset}_{t:05.2f}.png')
    else:
        r.video(out / f'{args.preset}{"_clean" if args.clean else ""}.mp4', fps)
