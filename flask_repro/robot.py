"""Robot: FLASK implementation and evaluation utilities."""
from dataclasses import dataclass, replace
import numpy as np
import vamp

@dataclass(frozen=True)
class Robot:
    name: str
    q_lo: np.ndarray
    q_hi: np.ndarray
    v_max: np.ndarray
    a_max: np.ndarray
    resolution: int
    K: int = 8

    @property
    def n(self):
        return self.q_lo.shape[0]

    @property
    def vamp(self):
        return getattr(vamp, self.name)

    def scaled_limits(self, scale_factor):
        """Scaled limits."""
        return replace(self, v_max=self.v_max * scale_factor, a_max=self.a_max * scale_factor)

def _from_vamp(name, v_max, a_max):
    m = getattr(vamp, name)
    return Robot(name, np.asarray(m.lower_bounds(), float), np.asarray(m.upper_bounds(), float), np.asarray(v_max, float), np.asarray(a_max, float), int(m.resolution()))

def get_robot(name):
    if name == 'panda':
        return _from_vamp('panda', [2.175] * 4 + [2.61] * 3, [15, 7.5, 10, 12.5, 15, 20, 20])
    if name == 'ur5':
        return _from_vamp('ur5', [np.pi] * 6, [5.0] * 6)
    if name == 'fetch':
        # velocities: fetch.urdf <limit velocity>; accelerations are not in the URDF: 0.5 m/s^2 torso and
        # 1.5 rad/s^2 arm, the values of Fetch's MoveIt joint_limits.yaml (assumption, used for the showcase only)
        return _from_vamp('fetch', [0.1, 1.256, 1.454, 1.571, 1.521, 1.571, 2.268, 2.268], [0.5] + [1.5] * 7)
    if name == 'sphere':
        # point robot for an omnidirectional base: limits set per use (see Robot.replace)
        return _from_vamp('sphere', [1.0] * 3, [1.0] * 3)
    raise ValueError(name)
