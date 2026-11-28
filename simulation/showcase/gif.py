"""GitHub GIFs: each robot's whole run sped up to ~8.5 s, plus a montage (2x2 for four robots, 3x2 for six).

    python -m simulation.showcase.gif --robots g1 drone ur5 fetch                                   # showcase_1
    FLASK_SHOWCASE=showcase_2 python -m simulation.showcase.gif --robots g1 drone ur5 fetch multiarm multidrone
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
W, H, FPS, SECONDS = 640, 360, 10, 12.0


SHOWCASE_2 = OUT.name != 'showcase_1'
ROBOTS = ['g1', 'drone', 'ur5', 'fetch'] + (['multiarm', 'multidrone'] if SHOWCASE_2 else [])


def grid_shape(n):
    """(columns, rows) of the montage: six robots in 3 rows x 2 columns."""
    return {4: (2, 2), 6: (2, 3)}.get(n, (n, 1))


def grid(tiles, cols, tw, th):
    rows = [np.hstack(tiles[i:i + cols]) for i in range(0, len(tiles), cols)]
    g = np.vstack(rows)
    for r in range(1, len(rows)):
        g[r * th - 1:r * th + 1, :] = 255                                   # thin white separators
    for c in range(1, cols):
        g[:, c * tw - 1:c * tw + 1] = 255
    return g


def montage_name(n):
    c, r = grid_shape(n)
    return f'flask_showcase_{r}x{c}'              # rows x columns


def renderer(robot, run, w=None, h=None, hud_scale=1.45):
    if SHOWCASE_2 and robot in ('ur5', 'multiarm'):
        from simulation.showcase.render_arms import ArmsRenderer
        w, h = w or W, h or H
        r = ArmsRenderer(run, w, h, 'ur5_pickplace' if robot == 'ur5' else 'multiarm_handover')
        r.hud = Hud(int(w * hud_scale))
        return r
    if SHOWCASE_2 and robot == 'multidrone':
        from simulation.showcase.render_swarm import SwarmRenderer
        w, h = w or W, h or H
        r = SwarmRenderer(run, w, h); r.hud = Hud(int(w * hud_scale))
        return r
    if SHOWCASE_2 and robot == 'fetch':
        from simulation.showcase.render_fetch2 import Fetch2Renderer
        w, h = w or W, h or H
        r = Fetch2Renderer(run, w, h); r.hud = Hud(int(w * hud_scale))
        return r
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


def palette_gif(frames_cmd_input, out, w, h, fps, colors=96):
    return subprocess.Popen([ffmpeg(), '-y', '-loglevel', 'error', '-f', 'rawvideo', '-pix_fmt', 'rgb24', '-s', f'{w}x{h}',
                             '-r', str(fps), '-i', '-', '-vf',
                             f'split[a][b];[a]palettegen=max_colors={colors}:stats_mode=diff[p];[b][p]paletteuse=dither=bayer:bayer_scale=4:diff_mode=rectangle',
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
    """Grid of compact tiles (planning time only), each run sped up to the same 8.5 s: 2x2 tiles of 320x180 for four
    robots (640x360), 3 rows x 2 columns of 448x252 for six (896x756)."""
    cols, rows = grid_shape(len(robots))
    tw, th = (W // 2, (H // 2) // 2 * 2) if len(robots) == 4 else (448, 252)
    rs = []
    for robot in robots:
        r = renderer(robot, selected(robot), tw, th, hud_scale=2.6); r.compact = True
        if robot == 'g1':
            r.cam_dist = 4.6
        r.SPEED = round(float(r.log['time'][-1]) / SECONDS, 1); rs.append(r)
    out = OUT / 'gif' / f'{montage_name(len(robots))}.gif'
    fps = FPS if len(robots) == 4 else 8                      # 3x2: keep the GIF under ~10 MB for GitHub
    p = palette_gif(None, out, cols * tw, rows * th, fps, colors=96 if len(robots) == 4 else 72)
    n = int(SECONDS * fps)
    for k in range(n):
        tiles = [r.frame(float(r.log['time'][-1]) * k / n) for r in rs]
        p.stdin.write(np.ascontiguousarray(grid(tiles, cols, tw, th), np.uint8).tobytes())
    p.stdin.close(); p.wait()
    print('montage', f'{out.stat().st_size / 1e6:.1f} MB')


if __name__ == '__main__':
    a = argparse.ArgumentParser(); a.add_argument('--robots', nargs='+', default=ROBOTS)
    a.add_argument('--montage-only', action='store_true')
    args = a.parse_args()
    if not args.montage_only:
        for robot in args.robots:
            robot_gif(robot)
    if len(args.robots) in (4, 6):
        montage(args.robots)
