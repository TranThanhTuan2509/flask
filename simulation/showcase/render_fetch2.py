"""Render the Fetch pick-and-place showcase (showcase_2) from its log: state replay only.

    python -m simulation.showcase.render_fetch2 --run showcase/showcase_2/fetch/seed_1 --preset delivery
"""
import argparse
from pathlib import Path

import numpy as np
from PIL import Image

from simulation.showcase import fetch_pickplace as S
from simulation.showcase import render_fetch as RF
from simulation.showcase.render_core import BLUE, ORANGE

PRESETS = RF.PRESETS


class Fetch2Renderer(RF.FetchRenderer):
    def __init__(self, run, width, height, hud=True, inset=True):
        saved = RF.S
        RF.S = S                                     # same renderer, showcase_2 model
        try:
            super().__init__(run, width, height, hud, inset)
        finally:
            RF.S = saved
        self.labels = [p['label'] for p in self.plans]

    ARM_AZIMUTH = 150.            # close-up from the robot's front-right (relative to its heading)

    def camera(self, t, i, still=False):
        x, y, th = self.log['qpos'][i, :3]
        if not str(self.log['phase'][i]).startswith('drive'):   # close-up on the arm, along the robot's heading
            target = np.array([x + 0.55 * np.cos(th), y + 0.55 * np.sin(th), 0.95, 2.8, np.degrees(th) + self.ARM_AZIMUTH, -28])
        else:
            target = np.array([x, y, 0.8, 6.8, -60, -32])
        if still or self.cam is None:
            self.cam = target
        else:
            d = target - self.cam; d[4] = (d[4] + 180) % 360 - 180
            self.cam = self.cam + (1 - np.exp(-max(t - self.last_t, 0) / 1.0)) * d
        c = self.cam
        return RF.free_camera(c[:3], c[3], c[4], c[5])

    def frame(self, t, still=False):
        RF.LABEL = dict(enumerate(self.labels))
        img = super().frame(t, still)
        return img


if __name__ == '__main__':
    a = argparse.ArgumentParser()
    a.add_argument('--run', type=Path, required=True); a.add_argument('--preset', choices=PRESETS, default='preview')
    a.add_argument('--stills', type=float, nargs='*'); a.add_argument('--clean', action='store_true')
    args = a.parse_args(); w, h, fps = PRESETS[args.preset]
    r = Fetch2Renderer(args.run, w, h, hud=not args.clean, inset=not args.clean)
    out = args.run / 'media'; out.mkdir(exist_ok=True)
    if args.stills:
        for t in args.stills:
            Image.fromarray(r.frame(t, still=True)).save(out / f'still_{args.preset}_{t:05.1f}.png')
    else:
        r.video(out / f'{args.preset}{"_clean" if args.clean else ""}.mp4', fps)
