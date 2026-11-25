"""Shared showcase renderer: main view + picture-in-picture inset, translucent ghosts, planned/executed paths,
an in-frame HUD, and streaming to ffmpeg. Rendering only replays logged states; it never re-plans.
"""
import os
import shutil
import subprocess

os.environ.setdefault('MUJOCO_GL', 'egl')
import mujoco
import numpy as np
from PIL import Image, ImageDraw, ImageFont

FONT = '/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf'
BOLD = '/usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf'
MONO = '/usr/share/fonts/truetype/dejavu/DejaVuSansMono.ttf'
ORANGE = (1.0, 0.55, 0.10, 0.95)      # FLASK planned trajectory
BLUE = (0.16, 0.47, 0.95, 0.95)       # executed trajectory
TEAL = (0.10, 0.65, 0.55, 0.9)        # goals


def free_camera(lookat, distance, azimuth, elevation):
    cam = mujoco.MjvCamera(); cam.type = mujoco.mjtCamera.mjCAMERA_FREE
    cam.lookat[:] = lookat; cam.distance = distance; cam.azimuth = azimuth; cam.elevation = elevation
    return cam


def add_polyline(scene, points, rgba, width, min_step=None):
    points = np.asarray(points, float)
    min_step = width if min_step is None else min_step
    kept = [points[0]] if len(points) else []
    for p in points[1:]:                       # drop near-duplicate points: tiny capsules render as speckles
        if np.linalg.norm(p - kept[-1]) >= min_step:
            kept.append(p)
    if len(points) > 1 and len(kept) > 1 and np.linalg.norm(points[-1] - kept[-1]) > 0:
        kept[-1] = points[-1]
    for a, b in zip(kept[:-1], kept[1:]):
        if scene.ngeom >= scene.maxgeom:
            break
        g = scene.geoms[scene.ngeom]
        mujoco.mjv_initGeom(g, mujoco.mjtGeom.mjGEOM_CAPSULE, np.zeros(3), np.zeros(3), np.eye(3).ravel(),
                            np.asarray(rgba, np.float32))
        mujoco.mjv_connector(g, mujoco.mjtGeom.mjGEOM_CAPSULE, width, a, b)
        scene.ngeom += 1


def add_sphere(scene, pos, radius, rgba):
    if scene.ngeom < scene.maxgeom:
        mujoco.mjv_initGeom(scene.geoms[scene.ngeom], mujoco.mjtGeom.mjGEOM_SPHERE, np.array([radius, 0, 0]),
                            np.asarray(pos, float), np.eye(3).ravel(), np.asarray(rgba, np.float32))
        scene.ngeom += 1


def add_frame(scene, pos, rot, length=0.18, width=0.012):
    """RGB body axes (x red, y green, z blue)."""
    for k, rgba in enumerate([(0.9, 0.15, 0.15, 1), (0.15, 0.8, 0.2, 1), (0.15, 0.35, 0.95, 1)]):
        add_polyline(scene, [pos, pos + length * rot[:, k]], rgba, width)


class GhostPainter:
    """Adds translucent copies of the robot (dynamic geoms only) at logged configurations."""

    def __init__(self, model):
        self.model = model; self.data = mujoco.MjData(model)
        self.opt = mujoco.MjvOption(); self.pert = mujoco.MjvPerturb()

    def add(self, scene, qpos, alpha, tint=None):
        self.data.qpos[:] = qpos; mujoco.mj_forward(self.model, self.data)
        start = scene.ngeom
        mujoco.mjv_addGeoms(self.model, self.data, self.opt, self.pert, mujoco.mjtCatBit.mjCAT_DYNAMIC, scene)
        for g in scene.geoms[start:scene.ngeom]:
            if tint is not None:
                g.rgba[:3] = 0.5 * np.asarray(g.rgba[:3]) + 0.5 * np.asarray(tint)
            g.rgba[3] = alpha


class Hud:
    """Text drawn straight onto the frame (no boxes): dark outline for legibility on any background.
    Values are passed in from the log; nothing is computed here."""

    def __init__(self, width):
        s = width / 1920
        self.s = s
        self.key = ImageFont.truetype(FONT, int(26 * s)); self.val = ImageFont.truetype(MONO, int(28 * s))
        self.big = ImageFont.truetype(BOLD, int(64 * s)); self.big_unit = ImageFont.truetype(BOLD, int(30 * s))
        self.cap = ImageFont.truetype(BOLD, int(24 * s)); self.small = ImageFont.truetype(MONO, int(22 * s))
        self.label = ImageFont.truetype(BOLD, int(20 * s))

    def _text(self, d, xy, text, font, fill=(255, 255, 255), anchor='la'):
        d.text(xy, text, font=font, fill=fill, anchor=anchor, stroke_width=max(2, int(3 * self.s)),
               stroke_fill=(10, 14, 20, 210))

    def rows(self, image, rows, origin=(40, 34)):
        """Telemetry rows top-left: key in light grey, value in white monospace."""
        d = ImageDraw.Draw(image, 'RGBA'); s = self.s
        x, y = int(origin[0] * s), int(origin[1] * s)
        kw = max(d.textlength(k, font=self.key) for k, _ in rows)
        for k, v in rows:
            self._text(d, (x, y), k, self.key, (214, 222, 230))
            self._text(d, (x + kw + int(22 * s), y - int(1 * s)), v, self.val)
            y += int(40 * s)

    def plan_callout(self, image, ms, history, label='FLASK planning time', flash=0., detail=None):
        """Highlighted planning time top-right; `history` lists earlier plan times (ms) of this run."""
        d = ImageDraw.Draw(image, 'RGBA'); s = self.s; W = image.size[0]
        x = W - int(44 * s); y = int(30 * s)
        self._text(d, (x, y), label.upper(), self.cap, (255, 176, 80), anchor='ra')
        text = f'{ms * 1000:.0f}' if ms < 1 else f'{ms:.1f}' if ms < 100 else f'{ms:.0f}'
        unit = ' µs' if ms < 1 else ' ms'
        uw = d.textlength(unit, font=self.big_unit)
        glow = int(255 * flash)
        self._text(d, (x - uw, y + int(30 * s)), text, self.big, (255, 255 - glow // 3, 255 - glow), anchor='ra')
        self._text(d, (x, y + int(64 * s)), unit, self.big_unit, (255, 230, 200), anchor='ra')
        if detail:
            self._text(d, (x, y + int(108 * s)), detail, self.small, (255, 214, 170), anchor='ra')
            y += int(34 * s)
        if history:
            line = '  '.join((f'{h * 1000:.0f}µs' if h < 1 else f'{h:.1f}ms' if h < 100 else f'{h:.0f}ms') for h in history[-6:])
            self._text(d, (x, y + int(112 * s)), f'plans: {line}', self.small, (225, 232, 238), anchor='ra')

    def status(self, image, text, ok=True, origin=(40, None)):
        d = ImageDraw.Draw(image, 'RGBA'); s = self.s
        self._text(d, (int(origin[0] * s), image.size[1] - int(52 * s)), ('● ' if ok else '▲ ') + text, self.cap,
                   (90, 220, 150) if ok else (255, 90, 80))

    def legend(self, image, items, y_from_bottom=96):
        d = ImageDraw.Draw(image, 'RGBA'); s = self.s
        x = int(40 * s); y = image.size[1] - int(y_from_bottom * s) - int(32 * s) * (len(items) - 1)
        for rgb, text in items:
            c = tuple(int(255 * v) for v in rgb[:3])
            d.rounded_rectangle((x, y + int(11 * s), x + int(40 * s), y + int(19 * s)), radius=int(3 * s), fill=c + (255,),
                                outline=(10, 14, 20, 200), width=max(1, int(2 * s)))
            self._text(d, (x + int(52 * s), y), text, self.label, (236, 240, 244))
            y += int(32 * s)

    def inset(self, image, inset_rgb, label):
        W, H = image.size; s = self.s
        im = Image.fromarray(inset_rgb)
        x, y = W - im.width - int(40 * s), H - im.height - int(40 * s)
        d = ImageDraw.Draw(image, 'RGBA'); b = max(2, int(3 * s))
        d.rectangle((x - b, y - b, x + im.width + b - 1, y + im.height + b - 1), fill=(245, 247, 250, 255))
        image.paste(im, (x, y))
        self._text(d, (x + int(14 * s), y + int(10 * s)), label, self.label)


def ffmpeg():
    """System ffmpeg if present, else the static binary shipped by imageio-ffmpeg (requirements.txt)."""
    exe = shutil.which('ffmpeg')
    if exe:
        return exe
    import imageio_ffmpeg
    return imageio_ffmpeg.get_ffmpeg_exe()


class Writer:
    def __init__(self, path, width, height, fps, crf=16):
        self.p = subprocess.Popen([ffmpeg(), '-y', '-loglevel', 'error', '-f', 'rawvideo', '-pix_fmt', 'rgb24',
                                   '-s', f'{width}x{height}', '-r', str(fps), '-i', '-', '-c:v', 'libx264',
                                   '-preset', 'slow', '-crf', str(crf), '-pix_fmt', 'yuv420p', '-movflags', '+faststart',
                                   str(path)], stdin=subprocess.PIPE)

    def write(self, frame):
        self.p.stdin.write(np.ascontiguousarray(frame, np.uint8).tobytes())

    def close(self):
        self.p.stdin.close(); self.p.wait()
        if self.p.returncode:
            raise RuntimeError('ffmpeg failed')
