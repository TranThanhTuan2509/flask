import numpy as np
import pytest
from flask_repro.flat_models import Unicycle, Quad2D, Quad3D
from flask_repro.flat_bvp import fixed_time
from flask_repro.flat_planner import World, Settings, solve, valid_edge, edge_times, terminal_error
from flask_repro.provenance import summarize

def test_sphere_simd_against_independent_geometry():
    from flask_repro import _core
    rng = np.random.default_rng(8)
    world = World([-3.0, -3.0], [3.0, 3.0], boxes=[([0.0, 0.0], [0.4, 2.0])], spheres=[([1.5, 1.5], 0.3)], radius=0.1)
    world.prepare_simd()
    for _ in range(200):
        p = rng.uniform(-3, 3, size=(8, 2))
        assert world.positions_valid(p) == world.positions_valid(p, independent=True)
    assert not _core.sphere_positions_valid(world.cpp_environment, np.array([[np.nan, 0.0, 0.0]]), 0.1)
    assert _core.sphere_positions_valid(world.cpp_environment, np.empty((0, 3)), 0.1)

@pytest.mark.parametrize('model,start,goal', [(Unicycle(), [0.0, 0.0, 0.0], [1.0, 0.0, 0.0]), (Quad2D(), [0.0, 0.0, 0.0, 0.0, 0.0, 0.0], [1.0, 0.0, 0.0, 0.0, 0.0, 0.0]), (Quad3D(), [0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0], [1.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0])])
def test_direct_bvp_path_is_valid_and_matches_full_original_state(model, start, goal):
    d = 3 if model.outputs == 4 else 2
    world = World(-5 * np.ones(d), 5 * np.ones(d))
    cfg = Settings()
    result = solve(model, world, start, goal, cfg)
    assert result.solved and result.edges
    assert result.terminal_error < 1e-08
    assert all((valid_edge(e, model, world, cfg, independent=True, refinement=3) for e in result.edges))
    for (a, b) in zip(result.edges, result.edges[1:]):
        np.testing.assert_allclose(a.state(a.duration), b.state(0), atol=1e-08)

def test_collision_and_endpoint_rejection():
    model = Unicycle()
    cfg = Settings(max_iterations=3, budget_s=0.05)
    world = World([-1, -1], [2, 1], spheres=[([0, 0], 0.1)])
    result = solve(model, world, [0, 0, 0], [1, 0, 0], cfg)
    assert not result.solved and result.reason == 'invalid_endpoint'
    edge = fixed_time(model.lift([0, 0, 0]), model.lift([1, 0, 0]), 3.0)
    assert not valid_edge(edge, model, world, cfg)

def test_velocity_bound_schedule_and_pruning_variants():
    model = Unicycle()
    cfg = Settings(max_iterations=12, phase_iterations=4, budget_s=5.0)
    world = World([-1, -1], [4, 4])
    edge = fixed_time([[0, 0], [0, 0]], [[1, 0], [0, 0]], 2.0)
    times = edge_times(edge, cfg)
    steps = np.linalg.norm(np.diff(edge.derivative(times), axis=0), axis=1)
    assert np.max(steps) <= cfg.validation_spacing * (1 + 1e-10)
    for variant in ['sst_bvp', 'sst_simd', 'rrt_dp']:
        a = solve(model, world, [0, 0, 0], [3, 3, 1.0], cfg, seed=12, variant=variant)
        b = solve(model, world, [0, 0, 0], [3, 3, 1.0], cfg, seed=12, variant=variant)
        assert a.solved == b.solved and a.nodes == b.nodes and (a.iterations == b.iterations)
        for e in a.edges:
            assert valid_edge(e, model, world, cfg, independent=True, refinement=4)

def test_original_goal_metric_wraps_angle_and_keeps_velocity():
    model = Quad2D()
    a = model.lift([0, 0, np.pi - 0.001, 0, 0, 0])
    b = model.lift([0, 0, -np.pi + 0.001, 0, 0, 0])
    assert terminal_error(model, a, b) == pytest.approx(0.002)
    b[1, 0] = 1.0
    assert terminal_error(model, a, b) > 1.0

def test_summary_preserves_failures_and_marks_excluded_inputs():
    rows = [dict(solved=True, validated=True, total_ms=1.0, wall_ms=2.0, length=3.0, collision=False), dict(solved=False, total_ms=100.0, wall_ms=101.0, reason='timeout'), dict(solved=False, eligible=False, reason='invalid_dataset')]
    summary = summarize(rows)
    assert summary['attempted'] == 2 and summary['available'] == 3 and (summary['success_rate'] == 0.5)
    assert summary['successful_total_ms']['mean'] == 1.0
    assert summary['attempt_wall_ms']['mean'] == 51.5
    assert summary['failure_reasons'] == {'timeout': 1}
    unchecked = summarize([dict(solved=True, total_ms=1.0)])
    assert unchecked['collision_risk'] is None and unchecked['unchecked_successes'] == 1
