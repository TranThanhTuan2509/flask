"""Legacy Python search diagnostics and independent scalar edge validation.

Use flat_bvp for the numerical steering oracle and planning for audited native
planners. This retained search loop has an approximate SST schedule/pruning
policy and must not substantiate SST* guarantees or paper speed claims.
"""
from dataclasses import dataclass, field
from time import perf_counter_ns
import numpy as np
from .flat_bvp import connect, propagate
from .flat_models import SingularFlatState

@dataclass
class Settings:
    rho: float = 1.0
    max_iterations: int = 10000
    budget_s: float = 1.0
    range: float = 1.0
    goal_bias: float = 0.1
    validation_dt: float = 0.02
    validation_spacing: float = 0.03
    propagation_min_s: float = 0.05
    propagation_max_s: float = 0.5
    control_bound: float = 1.0
    selection_radius: float = 1.0
    witness_radius: float = 0.2
    shrink_factor: float = 0.8
    phase_iterations: int = 1000
    sample_derivative_bound: float = 0.3
    goal_tolerance: float = 0.1

    def check(self):
        for name in ['rho', 'budget_s', 'range', 'validation_dt', 'validation_spacing', 'propagation_min_s', 'propagation_max_s', 'selection_radius', 'witness_radius', 'control_bound', 'sample_derivative_bound', 'goal_tolerance']:
            if not np.isfinite(getattr(self, name)) or getattr(self, name) <= 0:
                raise ValueError(f'{name} must be finite and positive')
        if self.max_iterations < 1 or self.phase_iterations < 1 or (not 0 <= self.goal_bias <= 1) or (not 0 < self.shrink_factor < 1) or (self.propagation_min_s > self.propagation_max_s):
            raise ValueError('Invalid planner iteration, probability, or propagation settings')

@dataclass
class World:
    lower: np.ndarray
    upper: np.ndarray
    boxes: list = field(default_factory=list)
    spheres: list = field(default_factory=list)
    radius: float = 0.1
    cpp_environment: object = None
    # The world a sphere cover was built from (exact boxes); used to reject start/goal states inside hollow shells.
    source: object = field(default=None, repr=False, compare=False)

    def __post_init__(self):
        (self.lower, self.upper) = (np.asarray(self.lower, float), np.asarray(self.upper, float))
        if self.lower.shape != self.upper.shape or len(self.lower) not in (2, 3) or (not np.isfinite(self.lower).all()) or (not np.isfinite(self.upper).all()) or np.any(self.lower >= self.upper) or (not np.isfinite(self.radius)) or (self.radius < 0):
            raise ValueError('Workspace bounds must be finite and ordered; radius must be nonnegative')

    def positions_valid(self, p, independent=False):
        p = np.atleast_2d(np.asarray(p, float))
        if not np.isfinite(p).all() or np.any(p < self.lower) or np.any(p > self.upper):
            return False
        if self.cpp_environment is not None and (not independent):
            from . import _core
            xyz = np.zeros((len(p), 3))
            xyz[:, :p.shape[1]] = p
            return _core.sphere_positions_valid(self.cpp_environment, xyz, self.radius)
        for (center, size) in self.boxes:
            delta = np.maximum(np.abs(p - np.asarray(center)) - np.asarray(size) / 2, 0)
            if np.any(np.sum(delta * delta, axis=1) <= self.radius ** 2):
                return False
        for (center, radius) in self.spheres:
            if np.any(np.sum((p - np.asarray(center)) ** 2, axis=1) <= (self.radius + radius) ** 2):
                return False
        return True

    def prepare_simd(self):
        from . import require_native
        _core = require_native()
        env = _core.Environment()
        d = len(self.lower)
        for (center, size) in self.boxes:
            xyz = np.zeros(3)
            xyz[:d] = center
            half = np.ones(3)
            half[:d] = np.asarray(size) / 2
            env.add_cuboid(xyz.tolist(), [0.0, 0.0, 0.0], half.tolist())
        for (center, radius) in self.spheres:
            xyz = np.zeros(3)
            xyz[:d] = center
            env.add_sphere(xyz.tolist(), float(radius))
        env.prepare()
        self.cpp_environment = env

    def shell_thickness(self, spacing=0.15):
        """Smallest cell width of the boundary layer over boxes whose interior a shell cover removes.

        Any continuous motion from outside a box into its removed interior crosses a full boundary cell
        layer, which the circumsphere cover occupies; a validator whose consecutive samples are closer
        than this width therefore cannot step over the shell."""
        widths = []
        for (_, size) in self.boxes:
            size = np.asarray(size, float)
            counts = np.maximum(1, np.ceil(size / spacing).astype(int))
            if np.all(counts >= 3):
                widths.extend((size / counts).tolist())
        return min(widths) if widths else np.inf

    def sphere_cover(self, spacing=0.15, shell_only=False, keep_boxes=True):
        """Conservative cover of each box by circumspheres of a regular cell grid.

        shell_only retains boundary-cell spheres plus the original filled boxes.
        Boundary spheres alone do not represent pointwise interior occupancy;
        no sampled no-tunneling equivalence is assumed."""
        if spacing <= 0:
            raise ValueError('Sphere-cover spacing must be positive')
        covered = list(self.spheres)
        for (center, size) in self.boxes:
            (center, size) = (np.asarray(center), np.asarray(size))
            counts = np.maximum(1, np.ceil(size / spacing).astype(int))
            width = size / counts
            axes = [center[j] - size[j] / 2 + (np.arange(counts[j]) + 0.5) * width[j] for j in range(len(size))]
            grid = np.stack(np.meshgrid(*[np.arange(c) for c in counts], indexing='ij'), axis=-1).reshape(-1, len(size))
            points = np.stack(np.meshgrid(*axes, indexing='ij'), axis=-1).reshape(-1, len(size))
            if shell_only:
                points = points[np.any((grid == 0) | (grid == counts - 1), axis=1)]
            covered.extend(((p, np.linalg.norm(width) / 2) for p in points))
        # Surface cells alone leave deep obstacle interiors unoccupied. Retain
        # the original solids in the optimized shell representation as well.
        # keep_boxes=False (v8, tree planners only): a pure-sphere shell is equivalent for planners whose
        # nodes all arise from validated edges, provided consecutive edge samples are closer than the
        # shell thickness (enforced by benchmarks/native_campaign.py); returned paths are re-checked
        # against the original boxes.
        return World(self.lower, self.upper, boxes=list(self.boxes) if shell_only and keep_boxes else [],
                     spheres=covered, radius=self.radius, source=self.source or self)

def edge_times(edge, settings, refinement=1):
    from numpy.polynomial import polynomial as poly
    c = edge.coefficients
    dc = poly.polyder(c, axis=0) / edge.duration
    ddc = poly.polyder(dc, axis=0)
    bound = []
    for j in range(c.shape[1]):
        roots = poly.polyroots(ddc[:, j])
        times = [0.0, 1.0] + [z.real for z in roots if abs(z.imag) < 1e-10 and 0 < z.real < 1]
        bound.append(max((abs(poly.polyval(t, dc[:, j])) for t in times)))
    speed_bound = np.linalg.norm(bound)
    count = max(2, int(np.ceil(edge.duration * refinement / settings.validation_dt)) + 1, int(np.ceil(edge.duration * speed_bound * refinement / settings.validation_spacing)) + 1)
    return np.linspace(0.0, edge.duration, count)

def valid_edge(edge, model, world, settings, independent=False, refinement=1):
    if hasattr(model, 'edge_valid') and (not model.edge_valid(edge)):
        return False
    times = edge_times(edge, settings, refinement)
    try:
        for start in range(0, len(times), 256):
            value = model.recover(edge.jet(times[start:start + 256], max(model.order, 4) if hasattr(model, 'gravity') else model.order))
            if not np.all(model.valid(value)) or not world.positions_valid(value['position'], independent):
                return False
    except SingularFlatState:
        return False
    return True

@dataclass
class Result:
    solved: bool = False
    edges: list = field(default_factory=list)
    plan_ns: int = 0
    postprocess_ns: int = 0
    first_solution_ns: object = None
    iterations: int = 0
    nodes: int = 0
    reason: str = 'budget_exhausted'
    terminal_error: object = None

def shortcut(edges, model, world, settings):
    edges = list(edges)
    a = 0
    while a + 1 < len(edges):
        for b in range(len(edges) - 1, a, -1):
            e = connect(edges[a].state(0), edges[b].state(edges[b].duration), settings.rho)
            if e is not None and valid_edge(e, model, world, settings):
                edges[a:b + 1] = [e]
                break
        a += 1
    return edges

def _sample(rng, model, world, settings, goal):
    if rng.random() < settings.goal_bias:
        return goal.copy()
    z = rng.uniform(-settings.sample_derivative_bound, settings.sample_derivative_bound, size=(model.order, model.outputs))
    z[0, :len(world.lower)] = rng.uniform(world.lower, world.upper)
    if model.outputs == 4:
        z[0, 3] = rng.uniform(-np.pi, np.pi)
    return z

def _chain(nodes, parents, edges, i):
    path = []
    while parents[i] != i:
        path.append(edges[i])
        i = parents[i]
    return list(reversed(path))

def solve(model, world, start, goal, settings=None, seed=0, variant='rrt_connect'):
    """Solve."""
    cfg = settings or Settings()
    cfg.check()
    if variant not in ['rrt_connect', 'sst_bvp', 'sst_simd', 'rrt_dp']:
        raise ValueError('Unknown planner variant')
    (z0, zg) = (model.lift(start), model.lift(goal))
    begin = perf_counter_ns()
    deadline = begin + int(cfg.budget_s * 1000000000.0)
    result = Result()
    if not world.positions_valid(z0[0, :len(world.lower)]) or not world.positions_valid(zg[0, :len(world.lower)]):
        result.reason = 'invalid_endpoint'
        result.plan_ns = perf_counter_ns() - begin
        return result
    if np.array_equal(z0, zg):
        result.solved = True
        result.reason = 'identity'
        result.terminal_error = 0.0
        result.plan_ns = result.first_solution_ns = perf_counter_ns() - begin
        return result
    rng = np.random.default_rng(seed)

    def bvp(a, b):
        e = connect(a, b, cfg.rho)
        return e if e is not None and valid_edge(e, model, world, cfg) else None
    if variant == 'rrt_connect':
        direct = bvp(z0, zg)
        if direct is not None:
            result.solved = True
            result.edges = [direct]
            result.nodes = 2
        else:
            trees = [dict(z=[z0], parent=[0], edge=[None]), dict(z=[zg], parent=[0], edge=[None])]
            for iteration in range(cfg.max_iterations):
                if perf_counter_ns() >= deadline:
                    break
                result.iterations = iteration + 1
                side = iteration % 2
                (a, b) = (trees[side], trees[1 - side])
                target = _sample(rng, model, world, cfg, zg if side == 0 else z0)
                ai = int(np.argmin(np.linalg.norm(np.array(a['z']) - target, axis=(1, 2))))
                delta = target - a['z'][ai]
                distance = np.linalg.norm(delta)
                if distance > cfg.range:
                    target = a['z'][ai] + delta * cfg.range / distance
                edge = bvp(a['z'][ai], target) if side == 0 else bvp(target, a['z'][ai])
                if edge is None:
                    continue
                a['z'].append(target)
                a['parent'].append(ai)
                a['edge'].append(edge)
                ai = len(a['z']) - 1
                bi = int(np.argmin(np.linalg.norm(np.array(b['z']) - target, axis=(1, 2))))
                bridge = bvp(target, b['z'][bi]) if side == 0 else bvp(b['z'][bi], target)
                same = np.array_equal(target, b['z'][bi])
                if bridge is not None or same:
                    (si, gi) = (ai, bi) if side == 0 else (bi, ai)
                    left = _chain(trees[0]['z'], trees[0]['parent'], trees[0]['edge'], si)
                    right = list(reversed(_chain(trees[1]['z'], trees[1]['parent'], trees[1]['edge'], gi)))
                    result.edges = left + ([] if bridge is None else [bridge]) + right
                    result.solved = True
                    break
            result.nodes = sum((len(t['z']) for t in trees))
        if result.solved:
            result.first_solution_ns = perf_counter_ns() - begin
    else:
        (nodes, parents, edges, costs, active) = ([z0], [0], [None], [0.0], [True])
        (witnesses, representatives) = ([z0], [0])
        best_cost = np.inf
        phase = 0
        is_sst = variant.startswith('sst')
        for iteration in range(cfg.max_iterations):
            if perf_counter_ns() >= deadline:
                break
            result.iterations = iteration + 1
            if is_sst and iteration and (iteration % cfg.phase_iterations == 0):
                phase += 1
                (nodes, parents, edges, costs, active) = ([z0], [0], [None], [0.0], [True])
                (witnesses, representatives) = ([z0], [0])
            target = _sample(rng, model, world, cfg, zg)
            ids = np.flatnonzero(active)
            start_chain = np.linalg.norm(np.array(nodes)[ids] - target, axis=(1, 2))
            near = ids[start_chain <= cfg.selection_radius * cfg.shrink_factor ** phase] if is_sst else []
            i = min(near, key=lambda j: costs[j]) if len(near) else int(ids[np.argmin(start_chain)])
            u = rng.uniform(-cfg.control_bound, cfg.control_bound, model.outputs)
            e = propagate(nodes[i], u, rng.uniform(cfg.propagation_min_s, cfg.propagation_max_s))
            if not valid_edge(e, model, world, cfg):
                continue
            z = e.state(e.duration)
            cost = costs[i] + e.cost(cfg.rho)
            wi = None
            if is_sst:
                start_chain = np.linalg.norm(np.array(witnesses) - z, axis=(1, 2))
                wi = int(np.argmin(start_chain))
                if start_chain[wi] > cfg.witness_radius * cfg.shrink_factor ** phase:
                    witnesses.append(z)
                    representatives.append(None)
                    wi = len(witnesses) - 1
                old = representatives[wi]
                if old is not None and costs[old] <= cost:
                    continue
                if old is not None:
                    active[old] = False
            nodes.append(z)
            parents.append(i)
            edges.append(e)
            costs.append(cost)
            active.append(True)
            j = len(nodes) - 1
            if wi is not None:
                representatives[wi] = j
            candidate = None
            if variant == 'sst_bvp':
                tail = bvp(z, zg)
                if tail is not None:
                    candidate = _chain(nodes, parents, edges, j) + [tail]
            else:
                err = terminal_error(model, z, zg)
                if err <= cfg.goal_tolerance:
                    candidate = _chain(nodes, parents, edges, j)
            if candidate is not None:
                total = sum((e.cost(cfg.rho) for e in candidate))
                if result.first_solution_ns is None:
                    result.first_solution_ns = perf_counter_ns() - begin
                if total < best_cost:
                    result.solved = True
                    result.edges = candidate
                    best_cost = total
        result.nodes = len(nodes)
    result.plan_ns = perf_counter_ns() - begin
    if result.solved:
        t = perf_counter_ns()
        result.edges = shortcut(result.edges, model, world, cfg)
        result.postprocess_ns = perf_counter_ns() - t
        result.reason = 'solved'
        result.terminal_error = terminal_error(model, result.edges[-1].state(result.edges[-1].duration), zg)
    return result

def terminal_error(model, z, goal):
    a = model.recover(np.r_[z, np.zeros((1, model.outputs))])
    b = model.recover(np.r_[goal, np.zeros((1, model.outputs))])
    if 'state' in a:
        delta = a['state'] - b['state']
        if model.outputs == 2:
            delta[2] = np.arctan2(np.sin(delta[2]), np.cos(delta[2]))
        return float(np.linalg.norm(delta))
    from scipy.spatial.transform import Rotation
    rotation_error = Rotation.from_matrix(a['rotation'].T @ b['rotation']).magnitude()
    return float(np.linalg.norm(np.r_[a['position'] - b['position'], a['velocity'] - b['velocity'], a['omega'] - b['omega'], rotation_error]))
