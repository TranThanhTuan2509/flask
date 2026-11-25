"""Tracking laws shared by the showcases (no simulator or learning dependencies)."""
import numpy as np


def kanayama(pose, ref, v_r, w_r, kx=1.2, ky=4.0, kth=2.0):
    dx, dy = ref[0] - pose[0], ref[1] - pose[1]
    c, s = np.cos(pose[2]), np.sin(pose[2])
    ex, ey = c * dx + s * dy, -s * dx + c * dy
    eth = np.arctan2(np.sin(ref[2] - pose[2]), np.cos(ref[2] - pose[2]))
    v = v_r * np.cos(eth) + kx * ex
    w = w_r + v_r * (ky * ey + kth * np.sin(eth))
    return np.clip(v, 0.0, 0.8), np.clip(w, -1.0, 1.0), np.hypot(ex, ey)
