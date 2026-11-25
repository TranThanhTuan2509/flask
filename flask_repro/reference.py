"""Reference: FLASK implementation and evaluation utilities."""
import math
from dataclasses import dataclass, field
import numpy as np
from . import lqmt as L
(_GL_X, _GL_W) = np.polynomial.legendre.leggauss(16)

@dataclass
class Edge:
    q0: np.ndarray
    v0: np.ndarray
    q1: np.ndarray
    v1: np.ndarray
    T: float

    @property
    def coef(self):
        return L.hermite_coefficients(self.q0, self.v0, self.q1, self.v1, self.T)

    def evaluate(self, t):
        return L.evaluate(self.coef, t)

    def length(self):
        """Length."""
        t = 0.5 * self.T * (_GL_X + 1.0)
        (_, trajectory, _, _) = self.evaluate(t)
        return 0.5 * self.T * float(_GL_W @ np.linalg.norm(trajectory, axis=1))

    def slice(self, ta, tb):
        """Slice."""
        (q, trajectory, _, _) = self.evaluate(np.array([ta, tb]))
        return Edge(q[0], trajectory[0], q[1], trajectory[1], tb - ta)

def bvp(q0, v0, q1, v1, rho=1.0):
    """Bvp."""
    T = L.optimal_duration(q0, v0, q1, v1, rho)
    if T is None:
        return None
    return Edge(np.asarray(q0, float), np.asarray(v0, float), np.asarray(q1, float), np.asarray(v1, float), T)

def sample_count(edge, robot):
    """Sample count."""
    batch_count = max(math.ceil(edge.length() / robot.K * robot.resolution), 1)
    return batch_count * robot.K

def batch_order(N, K):
    """Batch order."""
    m = math.ceil(N / K)
    return [[i + k * m for k in range(K) if i + k * m <= N] for i in range(1, m + 1)]

def flask_cc(edge, robot, env, check_limits=True, counter=None):
    """Flask cc."""
    if check_limits and (not L.within_limits(edge.coef, edge.T, robot.q_lo, robot.q_hi, robot.v_max, robot.a_max)):
        return False
    N = sample_count(edge, robot)
    t = edge.T * np.arange(1, N + 1) / N
    (q, _, _, _) = edge.evaluate(t)
    validate = robot.vamp.validate
    if not validate(edge.q0, env):
        return False
    for lower in batch_order(N, robot.K):
        if counter is not None:
            counter['lower'] = counter.get('lower', 0) + 1
        for j in lower:
            if not validate(q[j - 1], env):
                return False
    return True

@dataclass
class Trajectory:
    """Trajectory state and operations."""
    Q: np.ndarray
    V: np.ndarray
    T: np.ndarray

    @property
    def t(self):
        return np.concatenate([[0.0], np.cumsum(self.T)])

    def edge(self, i):
        return Edge(self.Q[i], self.V[i], self.Q[i + 1], self.V[i + 1], float(self.T[i]))

    def edges(self):
        return [self.edge(i) for i in range(len(self.T))]

    def length(self):
        return sum((c.length() for c in self.edges()))

    def duration(self):
        return float(np.sum(self.T))

    def evaluate(self, t):
        t = np.atleast_1d(np.asarray(t, float))
        if len(self.T) == 0:
            if np.any(t != 0):
                raise ValueError('Invalid planner configuration')
            return [np.broadcast_to(a, (len(t), len(a))).copy() for a in (self.Q[0], self.V[0], np.zeros_like(self.Q[0]))]
        knots = self.t
        k = np.clip(np.searchsorted(knots, t, side='right') - 1, 0, len(self.T) - 1)
        out = [np.empty((len(t), self.Q.shape[1])) for _ in range(3)]
        for i in np.unique(k):
            m = k == i
            (q, trajectory, qdd, _) = self.edge(i).evaluate(t[m] - knots[i])
            (out[0][m], out[1][m], out[2][m]) = (q, trajectory, qdd)
        return out

@dataclass
class PlannerSettings:
    range: float = 2.0
    w: float = 1.0
    neighbor_mode: str = 'A'
    k: int = 10
    rho: float = 1.0
    dynamic_domain: bool = True
    radius: float = 4.0
    alpha: float = 0.0001
    min_radius: float = 1.0
    balance: bool = True
    tree_ratio: float = 1.0
    max_iterations: int = 100000
    max_samples: int = 100000
    velocity_sample_scale: float = 1.0

@dataclass
class _Tree:
    is_start: bool
    Q: list = field(default_factory=list)
    V: list = field(default_factory=list)
    parent: list = field(default_factory=list)
    T: list = field(default_factory=list)
    r: list = field(default_factory=list)

    def insert(self, q, v, parent, T):
        self.Q.append(np.asarray(q, float))
        self.V.append(np.asarray(v, float))
        self.parent.append(len(self.Q) - 1 if parent is None else parent)
        self.T.append(T)
        self.r.append(math.inf)
        return len(self.Q) - 1

    def __len__(self):
        return len(self.Q)

    def connection(self, node_index, q, v, rho):
        """Connection."""
        if self.is_start:
            return bvp(self.Q[node_index], self.V[node_index], q, v, rho)
        return bvp(q, v, self.Q[node_index], self.V[node_index], rho)

    def nearest(self, q, v, settings):
        """Nearest."""
        i = self._nearest(q, v, settings)
        d = math.sqrt(float(np.sum((self.Q[i] - q) ** 2) + settings.w ** 2 * np.sum((self.V[i] - v) ** 2)))
        return (i, d)

    def _nearest(self, q, v, settings):
        Q = np.asarray(self.Q)
        V = np.asarray(self.V)
        d = np.linalg.norm(Q - q, axis=1) ** 2 + settings.w ** 2 * np.linalg.norm(V - v, axis=1) ** 2
        if settings.neighbor_mode == 'A':
            return int(np.argmin(d))
        candidates = np.argsort(d)[:settings.k]
        (best, best_c) = (int(candidates[0]), math.inf)
        for i in candidates:
            (Q0, V0, Q1, V1) = (Q[i], V[i], q, v) if self.is_start else (q, v, Q[i], V[i])
            T = L.optimal_duration(Q0, V0, Q1, V1, settings.rho)
            if T is None:
                return int(i)
            c = L.cost(T, *L.cost_coefficients(Q0, V0, Q1, V1), settings.rho)
            if c < best_c:
                (best, best_c) = (int(i), c)
        return best

def _root_chain(tree, i):
    node = [i]
    while tree.parent[node[-1]] != node[-1]:
        node.append(tree.parent[node[-1]])
    return node

def _join_trees(start, start_index, goal, goal_index):
    """ join trees."""
    start_chain = _root_chain(start, start_index)[::-1]
    goal_chain = _root_chain(goal, goal_index)
    Q = [start.Q[i] for i in start_chain] + [goal.Q[i] for i in goal_chain[1:]]
    V = [start.V[i] for i in start_chain] + [goal.V[i] for i in goal_chain[1:]]
    T = [start.T[i] for i in start_chain[1:]] + [goal.T[i] for i in goal_chain[:-1]]
    return Trajectory(np.asarray(Q), np.asarray(V), np.asarray(T, float))

@dataclass
class PlanningResult:
    trajectory: Trajectory = None
    iterations: int = 0
    node_count: tuple = (0, 0)

    @property
    def solved(self):
        return self.trajectory is not None

def rrtc(start, goals, robot, env, settings=None, seed=0, v_start=None, goal_velocity=None):
    """Rrtc."""
    settings = settings or PlannerSettings()
    rng = np.random.default_rng(seed)
    n = robot.n
    start_velocity = np.zeros(n) if v_start is None else np.asarray(v_start, float)
    goal_velocity = np.zeros(n) if goal_velocity is None else np.asarray(goal_velocity, float)
    start = np.asarray(start, float)
    for g in goals:
        c = bvp(start, start_velocity, g, goal_velocity, settings.rho)
        if c is not None and flask_cc(c, robot, env):
            return PlanningResult(Trajectory(np.stack([start, g]), np.stack([start_velocity, goal_velocity]), np.array([c.T])), 0, (1, 1))
    start_tree = _Tree(True)
    goal_tree = _Tree(False)
    start_tree.insert(start, start_velocity, None, math.nan)
    for g in goals:
        goal_tree.insert(g, goal_velocity, None, math.nan)
    (a, b) = (goal_tree, start_tree)
    iterations = 0
    while iterations < settings.max_iterations and len(start_tree) + len(goal_tree) < settings.max_samples:
        iterations += 1
        if not settings.balance or abs(len(a) - len(b)) / len(a) < settings.tree_ratio:
            (a, b) = (b, a)
        sample_q = rng.uniform(robot.q_lo, robot.q_hi)
        sample_v = rng.uniform(-robot.v_max, robot.v_max) * settings.velocity_sample_scale
        (nearest_index, nearest_distance) = a.nearest(sample_q, sample_v, settings)
        if settings.dynamic_domain and a.r[nearest_index] < nearest_distance:
            continue
        dq = sample_q - a.Q[nearest_index]
        dist = float(np.linalg.norm(dq))
        target_q = sample_q if dist < settings.range else a.Q[nearest_index] + dq * (settings.range / dist)
        c = a.connection(nearest_index, target_q, sample_v, settings.rho)
        if c is None or not flask_cc(c, robot, env):
            if settings.dynamic_domain:
                a.r[nearest_index] = settings.radius if a.r[nearest_index] == math.inf else max(a.r[nearest_index] * (1 - settings.alpha), settings.min_radius)
            continue
        new_index = a.insert(target_q, sample_v, nearest_index, c.T)
        if settings.dynamic_domain and a.r[nearest_index] != math.inf:
            a.r[nearest_index] *= 1 + settings.alpha
        (j, _) = b.nearest(a.Q[new_index], a.V[new_index], settings)
        if a.is_start:
            c = bvp(a.Q[new_index], a.V[new_index], b.Q[j], b.V[j], settings.rho)
        else:
            c = bvp(b.Q[j], b.V[j], a.Q[new_index], a.V[new_index], settings.rho)
        if c is None:
            connected = True
        else:
            m = max(math.ceil(float(np.linalg.norm(b.Q[j] - a.Q[new_index])) / settings.range), 1)
            critical_points = np.linspace(0.0, c.T, m + 1)
            segments = [(critical_points[k], critical_points[k + 1]) for k in range(m)]
            if not a.is_start:
                segments = segments[::-1]
            previous = new_index
            connected = True
            for (ta, tb) in segments:
                segment = c.slice(ta, tb)
                if not flask_cc(segment, robot, env):
                    connected = False
                    break
                if a.is_start:
                    previous = a.insert(segment.q1, segment.v1, previous, segment.T)
                else:
                    previous = a.insert(segment.q0, segment.v0, previous, segment.T)
            new_index = previous
        if connected:
            if a.is_start:
                trajectory = _join_trees(a, new_index, b, j)
            else:
                trajectory = _join_trees(b, j, a, new_index)
            return PlanningResult(trajectory, iterations, (len(start_tree), len(goal_tree)))
    return PlanningResult(None, iterations, (len(start_tree), len(goal_tree)))

def shortcut(trajectory, robot, env, rho=1.0):
    """Shortcut."""
    (Q, V, T) = (list(trajectory.Q), list(trajectory.V), list(trajectory.T))
    a = 0
    while a < len(Q) - 2:
        for b in range(len(Q) - 1, a + 1, -1):
            c = bvp(Q[a], V[a], Q[b], V[b], rho)
            if c is not None and flask_cc(c, robot, env):
                Q = Q[:a + 1] + Q[b:]
                V = V[:a + 1] + V[b:]
                T = T[:a] + [c.T] + T[b:]
                break
        a += 1
    return Trajectory(np.asarray(Q), np.asarray(V), np.asarray(T, float))

def validate_trajectory(trajectory, robot, env):
    """Validate trajectory."""
    return all((flask_cc(c, robot, env) for c in trajectory.edges()))
