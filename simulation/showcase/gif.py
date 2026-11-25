"""GitHub GIFs: each robot's whole run sped up to ~8.5 s, plus a 2x2 montage.

    python -m simulation.showcase.gif --robots g1 drone ur5 fetch
"""
import argparse
import os
import json
import subprocess
from pathlib import Path

import numpy as np

from simulation.showcase.render_core import Hud, ffmpeg

ROOT = Path(__file__).resolve().parents[2]
OUT = ROOT / 'showcase' / os.environ.get('FLASK_SHOWCASE', 'showcase_1')   # new campaign: FLASK_SHOWCASE=showcase_2
W, H, FPS, SECONDS = 640, 360, 10, 8.5


def renderer(robot, run, w=None, h=None, hud_scale=1.45):
    if robot == 'g1':
        from simulation.showcase.render_g1 import G1Renderer as C
    elif robot == 'drone':
        from simulation.showcase.render_drone import DroneRenderer as C
    elif robot == 'ur5':
        from simulation.showcase.render_ur5 import UR5Renderer as C
    else:
        from simulation.showcase.render_fetch import FetchRenderer as C
    w, h = w or W, h or H
    r = C(run, w, h)
    r.hud = Hud(int(w * hud_scale))       # larger text relative to the small frame
    if robot == 'drone':
        r.cam_dist = 6.0
    return r


def palette_gif(frames_cmd_input, out, w, h, fps):
    return subprocess.Popen([ffmpeg(), '-y', '-loglevel', 'error', '-f', 'rawvideo', '-pix_fmt', 'rgb24', '-s', f'{w}x{h}',
                             '-r', str(fps), '-i', '-', '-vf',
                             'split[a][b];[a]palettegen=max_colors=96:stats_mode=diff[p];[b][p]paletteuse=dither=bayer:bayer_scale=4:diff_mode=rectangle',
                             '-loop', '0', str(out)], stdin=subprocess.PIPE)


def selected(robot):
    sel = json.loads((OUT / 'selection.json').read_text())[robot]
    return OUT / robot / 'seeds' / f'seed_{sel["primary"]}'


def robot_gif(robot):
    run = selected(robot)
    r = renderer(robot, run)
    end = float(r.log['time'][-1]); n = int(SECONDS * FPS)
    r.SPEED = round(end / SECONDS, 1)
    out = OUT / 'gif' / f'{robot}.gif'; out.parent.mkdir(parents=True, exist_ok=True)
    p = palette_gif(None, out, W, H, FPS)
    for t in np.linspace(0, end, n, endpoint=False):
        p.stdin.write(np.ascontiguousarray(r.frame(t), np.uint8).tobytes())
    p.stdin.close(); p.wait()
    print(robot, run.name, f'{out.stat().st_size / 1e6:.1f} MB', f'{r.SPEED}x')


def montage(robots):
    """2x2 grid of compact tiles (planning time only), each run sped up to the same 8.5 s."""
    tw, th = W // 2, (H // 2) // 2 * 2
    rs = []
    for robot in robots:
        r = renderer(robot, selected(robot), tw, th, hud_scale=2.6); r.compact = True
        if robot == 'g1':
            r.cam_dist = 4.6
        r.SPEED = round(float(r.log['time'][-1]) / SECONDS, 1); rs.append(r)
    out = OUT / 'gif' / 'flask_showcase_2x2.gif'
    p = palette_gif(None, out, 2 * tw, 2 * th, FPS)
    n = int(SECONDS * FPS)
    for k in range(n):
        tiles = [r.frame(float(r.log['time'][-1]) * k / n) for r in rs]
        grid = np.vstack([np.hstack(tiles[:2]), np.hstack(tiles[2:])])
        grid[th - 1:th + 1, :] = 255; grid[:, tw - 1:tw + 1] = 255           # thin white separators
        p.stdin.write(np.ascontiguousarray(grid, np.uint8).tobytes())
    p.stdin.close(); p.wait()
    print('montage', f'{out.stat().st_size / 1e6:.1f} MB')


if __name__ == '__main__':
    a = argparse.ArgumentParser(); a.add_argument('--robots', nargs='+', default=['g1', 'drone', 'ur5', 'fetch'])
    a.add_argument('--montage-only', action='store_true')
    args = a.parse_args()
    if not args.montage_only:
        for robot in args.robots:
            robot_gif(robot)
    if len(args.robots) == 4:
        montage(args.robots)
