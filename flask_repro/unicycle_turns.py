"""Unicycle stop-turn-go reconstruction and independent validation (v8 protocol).

DynoBench ``unicycle1`` is a first-order model: state (x, y, theta), controls (v, omega) with
|v| <= v_max and |omega| <= omega_max, so the robot may stop at any node, rotate in place and continue
(the speed is a control and may jump). Along each FLASK edge the heading follows the flat velocity:

* straight edge (collinear polynomial coefficients): constant heading along the line, forward or reverse
  (kappa in Eq. 10, chosen per edge for the smaller turn), omega = 0, zero speed admissible;
* curved edge: theta = atan2(y') + kappa*pi with nonzero speed throughout and |omega| <= omega_max
  (exact extrema, Unicycle.edge_valid).

At every node, and at the start and goal, the robot turns in place at |omega| = omega_max from its
current heading to the next one. The robot is a disk, so turning does not change its occupancy.
With all planner nodes at rest every order-two BVP edge is the straight segment
y0 + (yf - y0)(3s^2 - 2s^3) (Example 4 with zero boundary velocities).

This module is independent of the native planner: it re-derives every quantity from the returned
polynomial coefficients and the ORIGINAL world, and reports the turn time explicitly.
"""
import numpy as np
from numpy.polynomial import polynomial as poly


def _wrap(a):
    return float(np.arctan2(np.sin(a), np.cos(a)))


def straight_direction(edge, tol=1e-9):
    """Unit line direction if all coefficient vectors c1..c3 are collinear, else None."""
    c = np.asarray(edge.coefficients, float)[1:]
    norms = np.linalg.norm(c, axis=1)
    if not np.isfinite(norms).all() or norms.max() <= 0:
        return None
    u = c[np.argmax(norms)] / norms.max()
    if np.max(np.abs(c[:, 0] * u[1] - c[:, 1] * u[0])) > tol * norms.max():
        return None
    return u


def _edge_headings(edge):
    """(start heading, end heading) options for kappa = 0 and 1."""
    u = straight_direction(edge)
    if u is not None:
        h = float(np.arctan2(u[1], u[0]))
        return [(h, h, 0), (_wrap(h + np.pi), _wrap(h + np.pi), 1)]
    v0, v1 = edge.derivative(0., 1), edge.derivative(edge.duration, 1)
    h0, h1 = float(np.arctan2(v0[1], v0[0])), float(np.arctan2(v1[1], v1[0]))
    return [(h0, h1, 0), (_wrap(h0 + np.pi), _wrap(h1 + np.pi), 1)]


def reconstruct(trajectory, start_state, goal_state, omega_max, reverse_allowed=True):
    """Turn schedule: in-place turns before every edge and at the goal, plus totals."""
    theta = float(start_state[2])
    turns, turn_time, reverse = [], 0., 0
    for i, e in enumerate(trajectory.edges):
        options = _edge_headings(e)[:2 if reverse_allowed else 1]
        h0, h1, kappa = min(options, key=lambda o: abs(_wrap(o[0] - theta)))
        turn = abs(_wrap(h0 - theta))
        if turn > 0:
            turns.append((i, theta, h0, turn / omega_max))
            turn_time += turn / omega_max
        reverse += kappa
        theta = h1
    final = abs(_wrap(float(goal_state[2]) - theta))
    if final > 0:
        turns.append((len(trajectory.edges), theta, float(goal_state[2]), final / omega_max))
        turn_time += final / omega_max
    motion = sum(e.duration for e in trajectory.edges)
    return dict(turn_list=turns, turn_time_s=turn_time, motion_time_s=motion, duration_s=motion + turn_time,
                turns=len(turns), reverse_edges=reverse, straight_edges=sum(straight_direction(e) is not None for e in trajectory.edges),
                final_heading=float(goal_state[2]) if final > 0 else theta)


def _edge_checks(e, model, world, times):
    """Original-model checks along one edge; returns list of error strings."""
    errors = []
    c = np.asarray(e.coefficients, float)
    if e.order != 2 or c.shape[0] != 4:
        return ['not_order_two']
    s = np.asarray(times, float) / e.duration
    p = np.moveaxis(poly.polyval(s, c), 0, -1)
    u = straight_direction(e)
    if u is None:
        if not model.edge_valid(e):  # exact speed and yaw-rate extrema, nonzero speed throughout
            errors.append('curved_speed_or_yaw_rate')
    else:
        offset = p - c[0]
        if np.max(np.abs(offset[:, 0] * u[1] - offset[:, 1] * u[0])) > 1e-9 * max(1., np.max(np.abs(offset))):
            errors.append('not_straight')
        v = poly.polyder(c, axis=0) / e.duration
        D = poly.polyadd(poly.polymul(v[:, 0], v[:, 0]), poly.polymul(v[:, 1], v[:, 1]))
        cand = [0., 1., .5] + [z.real for z in poly.polyroots(poly.polyder(D)) if abs(z.imag) < 1e-9 and 0 < z.real < 1]
        if np.sqrt(max(poly.polyval(cand, D))) > model.velocity_max + 1e-9:
            errors.append('speed')
    if not world.positions_valid(p, independent=True):
        errors.append('collision_or_bounds')
    return errors


def validate(record, model, world, cfg, refinement=4):
    """Independent original-model validation of a stop-turn-go unicycle solution."""
    from .flat_planner import Settings, edge_times
    from .original_sst import dynobench_distance
    errors = []
    if not record.solved:
        errors.append('no_solution')
    edges = record.trajectory.edges
    independent = Settings(validation_dt=cfg.validation_dt, validation_spacing=cfg.validation_spacing)
    if record.solved and not edges:
        errors.append('empty_trajectory')
    stops = 0
    for a, b in zip(edges, edges[1:]):
        jump = a.state(a.duration) - b.state(0.)
        if np.max(np.abs(jump[0])) > 1e-5:
            errors.append('position_discontinuity'); break
        # a flat-velocity jump is an instantaneous stop of the first-order unicycle (speed is a control)
        stops += bool(np.max(np.abs(jump[1])) > 1e-5)
    for i, e in enumerate(edges):
        for err in _edge_checks(e, model, world, edge_times(e, independent, refinement)):
            errors.append(f'edge_{i}_{err}')
    info = None
    if record.solved and edges:
        start, goal = np.asarray(record.start_state, float), np.asarray(record.goal_state, float)
        if np.linalg.norm(edges[0].state(0.)[0] - start[:2]) > 1e-5:
            errors.append('original_start_mismatch')
        info = reconstruct(record.trajectory, start, goal, model.angular_velocity_max)
        info['velocity_jump_stops'] = stops
        end = edges[-1].state(edges[-1].duration)[0]
        final = np.r_[end, info['final_heading']]
        goal_error = dynobench_distance('unicycle', final, goal)
        info['goal_error'] = float(goal_error)
        if goal_error > cfg.goal_tolerance + 1e-9:
            errors.append('original_goal_mismatch')
        # in-place turns: v = 0, position fixed (disk occupancy unchanged), |omega| = turn / time
        for (_, a, b, duration) in info['turn_list']:
            if abs(_wrap(b - a)) / duration > model.angular_velocity_max + 1e-9:
                errors.append('turn_rate')
        info.pop('turn_list')
    return dict(valid=not errors, errors=errors, validation_scope='original_model_stop_turn',
                original_model_valid=not errors, stop_turn=info)
