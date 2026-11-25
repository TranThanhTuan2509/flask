"""Unitree G1 walking with Unitree's released velocity policy (unitree_rl_gym, deploy/pre_train/g1/motion.pt).

Observation, PD gains, action scaling and gait phase follow deploy/deploy_mujoco/deploy_mujoco.py and
configs/g1.yaml of third_party/unitree_rl_gym (commit in third_party/SOURCES). FLASK only supplies the base
command (vx, vy, yaw rate); the locomotion policy is Unitree's, unmodified.
"""
from pathlib import Path

import mujoco
import numpy as np
import torch
import yaml

ROOT = Path(__file__).resolve().parents[2]
RLGYM = ROOT / 'third_party/unitree_rl_gym'


def gravity_orientation(q):
    w, x, y, z = q
    return np.array([2 * (-z * x + w * y), -2 * (z * y + w * x), 1 - 2 * (w * w + z * z)])


class G1Walker:
    """Steps a G1 MuJoCo model at 500 Hz with the 50 Hz policy; `command` is (vx, vy, yaw_rate) in the body frame."""

    def __init__(self, model, data, joint_offset=7, dof_offset=6, actuator_offset=0):
        cfg = yaml.safe_load((RLGYM / 'deploy/deploy_mujoco/configs/g1.yaml').read_text())
        self.cfg = cfg
        self.model, self.data = model, data
        self.kp, self.kd = np.asarray(cfg['kps'], np.float32), np.asarray(cfg['kds'], np.float32)
        self.default = np.asarray(cfg['default_angles'], np.float32)
        self.decimation = int(cfg['control_decimation'])
        self.n = int(cfg['num_actions'])
        self.policy = torch.jit.load(str(RLGYM / 'deploy/pre_train/g1/motion.pt'))
        self.action = np.zeros(self.n, np.float32); self.target = self.default.copy()
        self.obs = np.zeros(int(cfg['num_obs']), np.float32)
        self.qi, self.vi, self.ai = joint_offset, dof_offset, actuator_offset
        self.counter = 0
        self.command = np.zeros(3, np.float32)
        self.limits = np.array([0.8, 0.0, 1.0])  # |vx| (m/s), |vy|, |yaw rate| (rad/s) used for FLASK tracking
        # measurement sources (non-privileged deployments override these with sensor models, see
        # simulation/showcase/sensors.py); the defaults reproduce Unitree's deploy_mujoco.py exactly
        self.read_gyro = lambda: self.data.qvel[3:6]
        self.read_quat = lambda: self.data.qpos[3:7]
        self.read_joints = lambda: (self.data.qpos[self.qi:self.qi + self.n], self.data.qvel[self.vi:self.vi + self.n])
        model.opt.timestep = float(cfg['simulation_dt'])

    def step(self):
        d, n = self.data, self.n
        q, dq = self.read_joints()
        d.ctrl[self.ai:self.ai + n] = (self.target - q) * self.kp - dq * self.kd
        mujoco.mj_step(self.model, d)
        self.counter += 1
        if self.counter % self.decimation == 0:
            c = self.cfg
            o = self.obs
            o[:3] = self.read_gyro() * c['ang_vel_scale']
            o[3:6] = gravity_orientation(self.read_quat())
            o[6:9] = self.command * np.asarray(c['cmd_scale'], np.float32)
            q, dq = self.read_joints()
            o[9:9 + n] = (q - self.default) * c['dof_pos_scale']
            o[9 + n:9 + 2 * n] = dq * c['dof_vel_scale']
            o[9 + 2 * n:9 + 3 * n] = self.action
            phase = (self.counter * self.model.opt.timestep) % 0.8 / 0.8
            o[9 + 3 * n:9 + 3 * n + 2] = [np.sin(2 * np.pi * phase), np.cos(2 * np.pi * phase)]
            with torch.no_grad():
                self.action = self.policy(torch.from_numpy(o).unsqueeze(0)).numpy().squeeze()
            self.target = self.action * c['action_scale'] + self.default


def base_pose(data):
    """(x, y, yaw) of the pelvis."""
    w, x, y, z = data.qpos[3:7]
    return np.array([data.qpos[0], data.qpos[1], np.arctan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))])


def load_model():
    return mujoco.MjModel.from_xml_path(str(RLGYM / 'resources/robots/g1_description/scene.xml'))
