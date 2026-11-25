"""All media of one showcase campaign from the selected runs (showcase/<FLASK_SHOWCASE>/selection.json):
per robot a 4K master and a 1080p60 video, four 4K stills with ghosts; then GIFs and a 2x2 montage (GIF + 1080p video).

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
    tw, th = 960, 540
    rs = []
    for robot in robots:
        r = G.renderer(robot, G.selected(robot), tw, th, hud_scale=1.6); r.compact = True
        if robot == 'g1':
            r.cam_dist = 5.0
        r.SPEED = round(float(r.log['time'][-1]) / seconds, 1); rs.append(r)
    wr = Writer(OUT / 'flask_showcase_2x2.mp4', 2 * tw, 2 * th, fps); n = int(seconds * fps)
    for k in range(n):
        tiles = [r.frame(float(r.log['time'][-1]) * k / n) for r in rs]
        grid = np.vstack([np.hstack(tiles[:2]), np.hstack(tiles[2:])]); grid[th - 1:th + 1] = 255; grid[:, tw - 1:tw + 1] = 255
        wr.write(grid)
    wr.close(); print('montage video done', flush=True)


def montage_still(robots, k=2):
    """2x2 grid (3840x2160) of each robot's k-th 4K ghost still."""
    tiles = [Image.open(OUT / 'stills' / f'{r}_{k}.png').convert('RGB').resize((1920, 1080), Image.LANCZOS) for r in robots]
    grid = Image.new('RGB', (3840, 2160), 'white')
    for i, t in enumerate(tiles):
        grid.paste(t, ((i % 2) * 1920, (i // 2) * 1080))
    a = np.asarray(grid).copy(); a[1078:1082] = 255; a[:, 1918:1922] = 255
    Image.fromarray(a).save(OUT / 'stills' / 'flask_showcase_2x2.png'); print('montage still done', flush=True)


if __name__ == '__main__':
    a = argparse.ArgumentParser(); a.add_argument('--robots', nargs='+', default=['g1', 'drone', 'ur5', 'fetch'])
    a.add_argument('--only', choices=['videos', 'stills', 'gifs', 'montage'])
    args = a.parse_args()
    for robot in args.robots:
        if args.only in (None, 'stills'):
            stills(robot)
        if args.only in (None, 'gifs'):
            G.robot_gif(robot)
        if args.only in (None, 'videos'):
            videos(robot)
    if args.only in (None, 'montage') and len(args.robots) == 4:
        G.montage(args.robots); montage_video(args.robots); montage_still(args.robots)
