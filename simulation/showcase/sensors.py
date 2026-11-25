"""Non-privileged state for the showcase controllers.

Controllers never read MuJoCo's qpos/qvel of a floating base. They read the measurements a real robot has, each with
noise, its own rate and latency, and estimate what they need from them:

  * IMU (MuJoCo gyro / accelerometer sensors on the body)     -> white noise + constant bias
  * pose fix (motion capture for the drone, SLAM / AMCL-like localization for G1 and Fetch)
                                                              -> position / yaw noise, update rate, latency
  * joint encoders                                            -> quantization (and velocity by finite differences)

Ground-truth simulator state is used only to *evaluate* runs (contacts, tracking error vs. truth), never to control.
Noise levels are typical published figures; see SENSOR_PROFILES.
"""
from collections import deque

import mujoco
import numpy as np

# per-sensor noise (1 sigma), rates and latency
SENSOR_PROFILES = dict(
    mocap=dict(pos_sigma=0.002, yaw_sigma=np.radians(0.3), att_sigma=np.radians(0.3), rate=100., latency=0.010),
    ahrs=dict(pos_sigma=0.0, yaw_sigma=0.0, att_sigma=np.radians(0.5), rate=500., latency=0.0),   # IMU orientation filter
    slam=dict(pos_sigma=0.02, yaw_sigma=np.radians(1.0), att_sigma=np.radians(0.5), rate=30., latency=0.040),
    imu=dict(gyro_sigma=0.005, gyro_bias=0.002, acc_sigma=0.05),
    encoder=dict(counts_per_rad=2 ** 17 / (2 * np.pi)),          # 17-bit absolute encoders
)


def quat_to_R(q):
    w, x, y, z = q
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
                     [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
                     [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)]])


def small_rotation(rng, sigma):
    v = rng.normal(0, sigma, 3); a = np.linalg.norm(v)
    if a < 1e-12:
        return np.eye(3)
    k = v / a; K = np.array([[0, -k[2], k[1]], [k[2], 0, -k[0]], [-k[1], k[0], 0]])
    return np.eye(3) + np.sin(a) * K + (1 - np.cos(a)) * K @ K


class IMU:
    """Body-frame gyro (and accelerometer) from MuJoCo sensors, with noise and a fixed random bias."""

    def __init__(self, model, data, gyro, acc=None, seed=0):
        self.d = data; self.rng = np.random.default_rng(seed); p = SENSOR_PROFILES['imu']
        self.gi = model.sensor(gyro).adr[0]; self.ai = model.sensor(acc).adr[0] if acc else None
        self.bias = self.rng.normal(0, p['gyro_bias'], 3); self.p = p

    def gyro(self):
        return self.d.sensordata[self.gi:self.gi + 3] + self.bias + self.rng.normal(0, self.p['gyro_sigma'], 3)


class PoseFix:
    """Position + attitude fixes of a body (site), delivered at `rate` with `latency` and noise."""

    def __init__(self, model, data, site, profile, seed=0):
        self.m, self.d = model, data; self.sid = model.site(site).id
        self.p = SENSOR_PROFILES[profile]; self.rng = np.random.default_rng(seed + 1)
        self.queue = deque(); self.next_t = 0.; self.latest = None

    def update(self, t):
        """Call every control step; returns the newest fix available at time t: (stamp, position, rotation)."""
        if t + 1e-12 >= self.next_t:
            R = self.d.site_xmat[self.sid].reshape(3, 3) @ small_rotation(self.rng, self.p['att_sigma'])
            pos = self.d.site_xpos[self.sid] + self.rng.normal(0, self.p['pos_sigma'], 3)
            self.queue.append((t, pos, R)); self.next_t = t + 1. / self.p['rate']
        while self.queue and self.queue[0][0] + self.p['latency'] <= t + 1e-12:
            self.latest = self.queue.popleft()
        return self.latest


class PlanarEstimator:
    """(x, y, yaw) and planar velocity of a ground robot from localization fixes + IMU yaw rate.

    Yaw is propagated with the gyro between fixes and corrected by each fix (complementary filter); position is
    propagated with the commanded/estimated velocity and corrected by each fix; velocity is low-pass filtered."""

    def __init__(self, fix, imu, gain=0.25):
        self.fix, self.imu, self.k = fix, imu, gain
        self.x = None; self.v = np.zeros(2); self.last_stamp = None; self.t = None

    def update(self, t, v_cmd_world=None):
        f = self.fix.update(t)
        if f is None:
            return None
        stamp, pos, R = f; yaw_meas = np.arctan2(R[1, 0], R[0, 0])
        if self.x is None:
            self.x = np.array([pos[0], pos[1], yaw_meas]); self.t = t; self.last_stamp = stamp; self.prev = (stamp, pos[:2].copy())
            return self.x.copy()
        dt = t - self.t; self.t = t
        if dt > 0.2:                                             # stale: re-anchor on the fix instead of extrapolating
            self.x = np.array([pos[0], pos[1], yaw_meas]); self.v[:] = 0; self.last_stamp = stamp; self.prev = (stamp, pos[:2].copy())
            return self.x.copy()
        wz = self.imu.gyro()[2] if self.imu else 0.
        self.x[2] += wz * dt                                     # gyro propagation (body z ~ world z on flat ground)
        self.x[:2] += (v_cmd_world if v_cmd_world is not None else self.v) * dt
        if stamp != self.last_stamp:                             # new fix: correct and update velocity
            e = np.arctan2(np.sin(yaw_meas - self.x[2]), np.cos(yaw_meas - self.x[2]))
            self.x[2] += self.k * 2 * e
            self.x[:2] += self.k * (pos[:2] - self.x[:2])
            ds = stamp - self.prev[0]
            if ds > 0:
                self.v = 0.7 * self.v + 0.3 * (pos[:2] - self.prev[1]) / ds
            self.prev = (stamp, pos[:2].copy()); self.last_stamp = stamp
        return self.x.copy()


class RigidEstimator:
    """Drone: position/velocity from mocap fixes (alpha-beta filter), attitude from mocap, rates from the gyro."""

    def __init__(self, fix, imu, alpha=0.5, beta=0.2):
        self.fix, self.imu, self.a, self.b = fix, imu, alpha, beta
        self.p = None; self.v = np.zeros(3); self.R = np.eye(3); self.t = None; self.stamp = None

    def update(self, t):
        f = self.fix.update(t)
        if f is None:
            return None
        stamp, pos, R = f
        if self.p is None:
            self.p, self.R, self.t, self.stamp = pos.copy(), R.copy(), t, stamp
        dt = t - self.t; self.t = t
        self.p = self.p + self.v * dt                            # predict
        if stamp != self.stamp:                                  # correct on each new fix (latency compensated)
            lag = t - stamp; r = pos + self.v * lag - self.p
            self.p += self.a * r; self.v += self.b * r / max(1. / self.fix.p['rate'], 1e-6)
            self.R = R; self.stamp = stamp
        return self.p.copy(), self.v.copy(), self.R.copy(), self.imu.gyro().copy()


def encoder(q, rng=None):
    c = SENSOR_PROFILES['encoder']['counts_per_rad']
    return np.round(np.asarray(q) * c) / c
