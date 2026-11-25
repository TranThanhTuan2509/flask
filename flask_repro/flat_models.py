"""Flat models: FLASK implementation and evaluation utilities."""
from dataclasses import dataclass, field
import numpy as np

class SingularFlatState(ValueError):
    pass

def _jet(jet, degree, outputs):
    a = np.asarray(jet, float)
    if a.ndim < 2 or a.shape[-2] <= degree or a.shape[-1] != outputs or (not np.isfinite(a).all()):
        raise ValueError(f'Jet requires derivative {degree} and {outputs} outputs')
    return a

def _normalized_jet(a, already, dda):
    norm = np.linalg.norm(a, axis=-1, keepdims=True)
    if np.any(norm <= 1e-10):
        raise SingularFlatState('Cannot normalize a zero vector')
    e = a / norm
    dn = np.sum(e * already, axis=-1, keepdims=True)
    de = (already - e * dn) / norm
    ddn = (np.sum(already * already + a * dda, axis=-1, keepdims=True) - dn * dn) / norm
    dde = (dda - e * ddn - 2 * de * dn) / norm
    return (e, de, dde)

def _vee(S):
    return np.stack([S[..., 2, 1] - S[..., 1, 2], S[..., 0, 2] - S[..., 2, 0], S[..., 1, 0] - S[..., 0, 1]], axis=-1) / 2

@dataclass
class Manipulator:
    outputs: int
    order: int = 2
    inverse_dynamics: object = None
    velocity_max: object = None
    acceleration_max: object = None

    def lift(self, state):
        (q, v) = state
        return _jet(np.stack([q, v]), 1, self.outputs).copy()

    def recover(self, jet):
        a = _jet(jet, 2, self.outputs)
        (q, v, acc) = (a[..., j, :] for j in range(3))
        out = dict(position=q, velocity=v, acceleration=acc, state=np.concatenate([q, v], axis=-1))
        if self.inverse_dynamics is not None:
            out['control'] = self.inverse_dynamics(q, v, acc)
        return out

    def valid(self, value):
        shape = value['position'].shape[:-1]
        ok = np.ones(shape, dtype=bool)
        for (key, limit) in [('velocity', self.velocity_max), ('acceleration', self.acceleration_max)]:
            if limit is not None:
                ok &= np.all(np.abs(value[key]) <= np.asarray(limit) + 1e-10, axis=-1)
        return ok

@dataclass
class Unicycle:
    velocity_max: float = 1.0
    angular_velocity_max: float = 1.5
    reverse: bool = False
    lift_speed: float = 0.1
    # v8 stop-turn-go protocol (flask_repro/unicycle_turns.py): boundaries are lifted to rest and the
    # heading is restored by in-place turns; requires PlannerSettings.unicycle_rest_turns.
    rest_turns: bool = False
    outputs: int = field(default=2, init=False)
    order: int = field(default=2, init=False)

    def lift(self, state):
        (x, y, theta) = np.asarray(state, float)
        if self.rest_turns:
            return np.array([[x, y], [0., 0.]])
        if self.lift_speed <= 1e-10 or self.lift_speed > self.velocity_max:
            raise ValueError('Lift speed must be positive and within the velocity limit')
        v = (-1 if self.reverse else 1) * self.lift_speed
        return np.array([[x, y], [v * np.cos(theta), v * np.sin(theta)]])

    def recover(self, jet):
        a = _jet(jet, 2, 2)
        (p, v, acc) = (a[..., j, :] for j in range(3))
        vv = np.sum(v * v, axis=-1)
        if np.any(vv <= 1e-20):
            raise SingularFlatState('Unicycle flatness is singular at zero speed')
        theta = np.arctan2(v[..., 1], v[..., 0]) + (np.pi if self.reverse else 0)
        speed = (-1 if self.reverse else 1) * np.sqrt(vv)
        omega = (v[..., 0] * acc[..., 1] - v[..., 1] * acc[..., 0]) / vv
        return dict(position=p, velocity=v, state=np.concatenate([p, theta[..., None]], axis=-1), control=np.stack([speed, omega], axis=-1))

    def valid(self, value):
        return (np.abs(value['control'][..., 0]) <= self.velocity_max + 1e-10) & (np.abs(value['control'][..., 1]) <= self.angular_velocity_max + 1e-10)

    def edge_valid(self, edge):
        """Edge valid."""
        from numpy.polynomial import polynomial as poly
        v = poly.polyder(edge.coefficients, axis=0) / edge.duration
        a = poly.polyder(v, axis=0) / edge.duration
        D = poly.polyadd(poly.polymul(v[:, 0], v[:, 0]), poly.polymul(v[:, 1], v[:, 1]))
        N = poly.polysub(poly.polymul(v[:, 0], a[:, 1]), poly.polymul(a[:, 0], v[:, 1]))

        def candidates(p):
            return [0.0, 1.0] + [z.real for z in poly.polyroots(p) if abs(z.imag) < 1e-08 and 0 < z.real < 1]
        speed_points = candidates(poly.polyder(D))
        sampled_v = np.moveaxis(poly.polyval(speed_points, v), 0, -1)
        values = np.sum(sampled_v ** 2, axis=-1)
        if values.min() <= 1e-20 or values.max() > self.velocity_max ** 2 + 1e-10:
            return False
        stationary = poly.polysub(poly.polymul(poly.polyder(N), D), poly.polymul(N, poly.polyder(D)))
        t = candidates(stationary)
        sampled_v = np.moveaxis(poly.polyval(t, v), 0, -1)
        denominator = np.sum(sampled_v ** 2, axis=-1)
        return bool(np.all(np.abs(poly.polyval(t, N) / denominator) <= self.angular_velocity_max + 1e-10))

@dataclass
class Quad2D:
    mass: float = 0.034
    inertia: float = 0.0001
    arm: float = 0.1
    gravity: float = 9.81
    velocity_max: float = np.inf
    angular_velocity_max: float = np.inf
    outputs: int = field(default=2, init=False)
    # 4: full model (Example 3). 2: reduced pseudo-control order permitted by the paper in practice
    # (Sec. V-C1); attitude/body rate then jump at joins and are not boundary conditions.
    order: int = 4

    def lift(self, state):
        (x, z, theta, vx, vz, omega) = np.asarray(state, float)
        thrust_m = self.gravity
        a = np.array([-thrust_m * np.sin(theta), thrust_m * np.cos(theta) - self.gravity])
        j = -thrust_m * omega * np.array([np.cos(theta), np.sin(theta)])
        return np.array([[x, z], [vx, vz], a, j])[:self.order]

    def recover(self, jet):
        a = _jet(jet, 4, 2)
        (p, v, acc, jerk, snap) = (a[..., j, :] for j in range(5))
        thrust_vector = acc + np.array([0.0, self.gravity])
        squared = np.sum(thrust_vector ** 2, axis=-1)
        if np.any(squared <= 1e-20):
            raise SingularFlatState('Planar quadrotor thrust is singular')
        theta = np.arctan2(-thrust_vector[..., 0], thrust_vector[..., 1])
        num = thrust_vector[..., 0] * jerk[..., 1] - thrust_vector[..., 1] * jerk[..., 0]
        omega = num / squared
        dnum = thrust_vector[..., 0] * snap[..., 1] - thrust_vector[..., 1] * snap[..., 0]
        angular_acc = dnum / squared - num * 2 * np.sum(thrust_vector * jerk, axis=-1) / squared ** 2
        f = self.mass * np.sqrt(squared)
        torque = self.inertia * angular_acc
        motors = np.stack([(f + torque / self.arm) / 2, (f - torque / self.arm) / 2], axis=-1)
        return dict(position=p, velocity=v, theta=theta, omega=omega, state=np.concatenate([p, theta[..., None], v, omega[..., None]], axis=-1), control=motors, thrust=f, torque=torque)

    def valid(self, value):
        return np.all(value['control'] >= -1e-10, axis=-1) & np.all(value['control'] <= 0.65 * self.mass * self.gravity + 1e-10, axis=-1) & (np.linalg.norm(value['velocity'], axis=-1) <= self.velocity_max) & (np.abs(value['omega']) <= self.angular_velocity_max)

@dataclass
class Quad3D:
    mass: float = 1.0
    inertia: object = field(default_factory=lambda : np.diag([0.1, 0.1, 0.2]))
    gravity: float = 9.81
    velocity_max: float = 4.0
    angular_velocity_max: float = 8.0
    torque_max: float = 2.0
    outputs: int = field(default=4, init=False)
    # 4: full model (Example 3). 2: reduced pseudo-control order permitted by the paper in practice
    # (Sec. V-C1); attitude/body rate then jump at joins and are not boundary conditions.
    order: int = 4

    def lift(self, state):
        from scipy.spatial.transform import Rotation
        x = np.asarray(state, float)
        if x.shape != (13,):
            raise ValueError('Spatial quadrotor state must have 13 entries')
        R = Rotation.from_quat(x[3:7]).as_matrix()
        omega = x[10:13]
        hat = np.array([[0.0, -omega[2], omega[1]], [omega[2], 0.0, -omega[0]], [-omega[1], omega[0], 0.0]])
        dR = R @ hat
        psi = np.arctan2(R[1, 0], R[0, 0])
        denom = R[0, 0] ** 2 + R[1, 0] ** 2
        if denom < 1e-20:
            raise SingularFlatState('Yaw is undefined at this orientation')
        psidot = (R[0, 0] * dR[1, 0] - R[1, 0] * dR[0, 0]) / denom
        z = np.zeros((4, 4))
        z[0] = np.r_[x[:3], psi]
        z[1] = np.r_[x[7:10], psidot]
        z[2, :3] = self.gravity * (R[:, 2] - np.array([0.0, 0.0, 1.0]))
        z[3, :3] = self.gravity * dR[:, 2]
        recovered = self.recover(np.r_[z, np.zeros((1, 4))])
        if not np.allclose(recovered['rotation'], R, atol=1e-08) or not np.allclose(recovered['omega'], omega, atol=1e-08):
            raise SingularFlatState('Boundary orientation/body rate is outside the selected flatness chart')
        return z[:self.order]

    def recover(self, jet):
        a = _jet(jet, 4, 4)
        (p, v) = (a[..., 0, :3], a[..., 1, :3])
        thrust = self.mass * (a[..., 2, :3] + np.array([0.0, 0.0, self.gravity]))
        dz = _normalized_jet(thrust, self.mass * a[..., 3, :3], self.mass * a[..., 4, :3])
        (rz, drz, ddrz) = dz
        (psi, dpsi, ddpsi) = (a[..., j, 3] for j in range(3))
        zero = np.zeros_like(psi)
        h = np.stack([-np.sin(psi), np.cos(psi), zero], axis=-1)
        dh = np.stack([-np.cos(psi) * dpsi, -np.sin(psi) * dpsi, zero], axis=-1)
        ddh = np.stack([np.sin(psi) * dpsi ** 2 - np.cos(psi) * ddpsi, -np.cos(psi) * dpsi ** 2 - np.sin(psi) * ddpsi, zero], axis=-1)
        (rx, drx, ddrx) = _normalized_jet(np.cross(h, rz), np.cross(dh, rz) + np.cross(h, drz), np.cross(ddh, rz) + 2 * np.cross(dh, drz) + np.cross(h, ddrz))
        ry = np.cross(rz, rx)
        dry = np.cross(drz, rx) + np.cross(rz, drx)
        ddry = np.cross(ddrz, rx) + 2 * np.cross(drz, drx) + np.cross(rz, ddrx)
        R = np.stack([rx, ry, rz], axis=-1)
        dR = np.stack([drx, dry, drz], axis=-1)
        ddR = np.stack([ddrx, ddry, ddrz], axis=-1)
        Rt = np.swapaxes(R, -1, -2)
        omega = _vee(Rt @ dR)
        domega = _vee(np.swapaxes(dR, -1, -2) @ dR + Rt @ ddR)
        J = np.asarray(self.inertia, float)
        torque = np.einsum('ij,...j->...i', J, domega) + np.cross(omega, np.einsum('ij,...j->...i', J, omega))
        f = np.linalg.norm(thrust, axis=-1)
        return dict(position=p, velocity=v, rotation=R, rotation_dot=dR, omega=omega, omega_dot=domega, torque=torque, thrust=f, control=np.concatenate([f[..., None], torque], axis=-1))

    def valid(self, value):
        return (np.linalg.norm(value['velocity'], axis=-1) <= self.velocity_max + 1e-10) & (np.linalg.norm(value['omega'], axis=-1) <= self.angular_velocity_max + 1e-10) & (value['thrust'] <= 1.5 * self.mass * self.gravity + 1e-10) & (np.linalg.norm(value['torque'], axis=-1) <= self.torque_max + 1e-10)
