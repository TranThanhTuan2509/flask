"""All media of one showcase campaign from the selected runs (showcase/<FLASK_SHOWCASE>/selection.json):
per robot a 4K master and a 1080p60 video, four 4K stills with ghosts; then GIFs and a montage (2x2 or 3x2: GIF,
video, 4K still).

    python -m simulation.showcase.media                 # everything
    python -m simulation.showcase.media --only stills
"""
import argparse
import json

import numpy as np
from PIL import Image

from simulation.showcase import gif as G
from simulation.showcase.render_core import Hud, Writer

OUT = G.OUT
FRACTIONS = (0.35, 0.60, 0.85, 0.995)


def videos(robot, presets=('delivery', 'master')):
    run = G.selected(robot)
    for preset in presets:
        w, h, fps = dict(master=(3840, 2160, 30), delivery=(1920, 1080, 60))[preset]
        r = G.renderer(robot, run, w, h, hud_scale=1.0)
        (run / 'media').mkdir(exist_ok=True)
        r.video(run / 'media' / f'{preset}.mp4', fps)
        print(robot, preset, 'done', flush=True)


def stills(robot):
    run = G.selected(robot); r = G.renderer(robot, run, 3840, 2160, hud_scale=1.0)
    end = float(r.log['time'][-1]); out = OUT / 'stills'; out.mkdir(parents=True, exist_ok=True)
    for k, f in enumerate(FRACTIONS):
        Image.fromarray(r.frame(end * f, still=True)).save(out / f'{robot}_{k + 1}.png')
    print(robot, 'stills done', flush=True)


def montage_video(robots, seconds=30., fps=30):
    cols, rows = G.grid_shape(len(robots))
    tw, th = (960, 540) if len(robots) == 4 else (960, 540)
    rs = []
    for robot in robots:
        r = G.renderer(robot, G.selected(robot), tw, th, hud_scale=1.6); r.compact = True
        if robot == 'g1':
            r.cam_dist = 5.0
        r.SPEED = round(float(r.log['time'][-1]) / seconds, 1); rs.append(r)
    wr = Writer(OUT / f'{G.montage_name(len(robots))}.mp4', cols * tw, rows * th, fps); n = int(seconds * fps)
    for k in range(n):
        tiles = [r.frame(float(r.log['time'][-1]) * k / n) for r in rs]
        wr.write(G.grid(tiles, cols, tw, th))
    wr.close(); print('montage video done', flush=True)


def montage_still(robots, k=2):
    """Grid of each robot's k-th 4K ghost still: 2x2 of 1920x1080 or 3x2 of 1280x720 (3840 wide)."""
    cols, rows = G.grid_shape(len(robots)); tw, th = 3840 // cols, 3840 // cols * 9 // 16
    tiles = [np.asarray(Image.open(OUT / 'stills' / f'{r}_{k}.png').convert('RGB').resize((tw, th), Image.LANCZOS)) for r in robots]
    Image.fromarray(G.grid(tiles, cols, tw, th)).save(OUT / 'stills' / f'{G.montage_name(len(robots))}.png')
    print('montage still done', flush=True)


if __name__ == '__main__':
    a = argparse.ArgumentParser(); a.add_argument('--robots', nargs='+', default=G.ROBOTS)
    a.add_argument('--only', choices=['videos', 'stills', 'gifs', 'montage'])
    args = a.parse_args()
    for robot in args.robots:
        if args.only in (None, 'stills'):
            stills(robot)
        if args.only in (None, 'gifs'):
            G.robot_gif(robot)
        if args.only in (None, 'videos'):
            videos(robot)
    if args.only in (None, 'montage') and len(args.robots) in (4, 6):
        G.montage(args.robots); montage_video(args.robots); montage_still(args.robots)
