"""Render the G1 courtyard showcase from its log (state replay only; nothing is re-planned).

Videos: no ghosts. Stills (--stills): translucent ghosts of the past motion.
    python -m simulation.showcase.render_g1 --run showcase/showcase_1/g1/seed_1 --preset delivery
    python -m simulation.showcase.render_g1 --run ... --preset master --stills 20 35
"""
import argparse
import json
from pathlib import Path

import mujoco
import numpy as np
from PIL import Image

from simulation.showcase import g1_courtyard as S
from simulation.showcase.render_core import BLUE, ORANGE, GhostPainter, Hud, Writer, add_polyline, free_camera

PRESETS = dict(master=(3840, 2160, 30), delivery=(1920, 1080, 60), preview=(1280, 720, 30))
FADED = (1.0, 0.72, 0.45, 0.55)


class G1Renderer:
    cam_dist = 6.6
    compact = False   # montage tiles: planning time only
    SPEED = 1.0   # playback speed; shown on screen when != 1
    def __init__(self, run, width, height, hud=True, inset=True):
        self.run = Path(run)
        self.log = dict(np.load(self.run / 'execution.npz'))
        self.plans = json.loads((self.run / 'plans.json').read_text())
        self.model = S.build_model(); self.data = mujoco.MjData(self.model)
        self.w, self.h = width, height
        self.main = mujoco.Renderer(self.model, height, width, max_geom=30000)
        iw = int(width * 0.27); ih = int(iw * 9 / 14)
        self.small = mujoco.Renderer(self.model, ih, iw, max_geom=30000)
        self.ghosts = GhostPainter(self.model)
        self.hud = Hud(width) if hud else None; self.show_inset = inset
        self.look = None

    def state(self, t):
        T = self.log['time']; i = int(np.clip(np.searchsorted(T, t), 0, len(T) - 1))
        self.data.qpos[:] = self.log['qpos'][i]; mujoco.mj_forward(self.model, self.data)
        return i

    def decorate(self, scene, i, t, ghosts=False, width=1.0):
        leg = int(self.log['leg'][i])
        for p in self.plans[:leg + 1]:
            pts = np.c_[np.asarray(p['path']), np.full(len(p['path']), 0.03)]
            add_polyline(scene, pts, ORANGE if p['leg'] == leg else FADED, 0.03 * width)
        trail = self.log['qpos'][:i + 1, :2]
        if len(trail) > 1:
            add_polyline(scene, np.c_[trail, np.full(len(trail), 0.045)], BLUE, 0.022 * width, min_step=0.05)
        if ghosts:
            for k, tg in enumerate(np.arange(t - 1.5, max(t - 12.1, 0), -1.5)):
                j = int(np.searchsorted(self.log['time'], tg))
                self.ghosts.add(scene, self.log['qpos'][j], alpha=max(0.12, 0.40 - 0.035 * k))

    def camera(self, t, i, still=False):
        target = self.log['qpos'][i, :3] * [1, 1, 0] + [0, 0, 0.7]
        if still or self.look is None:
            self.look = target
        else:
            a = 1 - np.exp(-max(t - self.last_t, 0) / 1.2)               # time-based smoothing (tau 1.2 s)
            self.look = self.look + a * (target - self.look)
        return free_camera(self.look, self.cam_dist, -100 + 8 * np.sin(t / 14), -36)

    last_t = 0.

    def frame(self, t, still=False):
        i = self.state(t)
        self.main.update_scene(self.data, self.camera(t, i, still)); self.decorate(self.main.scene, i, t, ghosts=still)
        img = Image.fromarray(self.main.render()); self.last_t = t
        if self.show_inset and not self.compact:
            self.small.update_scene(self.data, 'top'); self.decorate(self.small.scene, i, t, width=2.0)
            inset = self.small.render()
        if self.hud:
            leg = int(self.log['leg'][i]); p = self.plans[leg]; cmd = self.log['command'][i]
            since = t - p['t_start']
            self.hud.plan_callout(img, p['plan_us'] / 1e3, [q['plan_us'] / 1e3 for q in self.plans[:leg + 1]],
                                  label='FLASK planning time', flash=float(np.clip(1 - since / 0.8, 0, 1)),
                                  detail=f'+ smoothing {p["shortcut_us"]:.0f} µs')
            if not self.compact:
                self.hud.rows(img, [('time', f'{t:5.1f} s' + (f'  ({self.SPEED:g}× speed)' if self.SPEED != 1 else '')), ('leg', f'{leg + 1}/{len(self.plans)}'),
                                    ('path', f'{p["length_m"]:.1f} m'),
                                    ('vx / vy', f'{cmd[0]:+.2f} / {cmd[1]:+.2f} m/s'), ('ω', f'{cmd[2]:+.2f} rad/s'),
                                    ('error', f'{100 * self.log["track_error"][i]:.1f} cm'),
                                ('localization', f'{100 * self.log["estimate_error"][i]:.1f} cm (SLAM + IMU)')])
            if not self.compact:
                self.hud.legend(img, [(ORANGE, 'FLASK plan'), (BLUE, 'executed')])
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
    r = G1Renderer(args.run, w, h, hud=not args.clean, inset=not args.clean)
    out = args.run / 'media'; out.mkdir(exist_ok=True)
    if args.stills:
        for t in args.stills:
            Image.fromarray(r.frame(t, still=True)).save(out / f'still_{args.preset}_{t:05.1f}.png')
    else:
        r.video(out / f'{args.preset}{"_clean" if args.clean else ""}.mp4', fps)
