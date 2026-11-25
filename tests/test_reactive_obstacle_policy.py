"""A reactive-avoidance experiment cannot silently make its obstacle yield."""
from types import SimpleNamespace
import numpy as np
from experiments import reactive_experiment as reactive


def test_default_obstacle_clock_is_independent_of_robot_clearance(monkeypatch):
    def forbidden(*args):raise AssertionError('Robot clearance must not control a scripted obstacle')
    monkeypatch.setattr(reactive.CE,'tube_clearance',forbidden)
    trail=SimpleNamespace(pose=forbidden)
    for q in [np.zeros(6),np.ones(6)]:
        assert reactive.obstacle_clock(1.,3.,q,trail)==(3.,False)


def test_cooperative_yield_is_only_an_explicit_diagnostic(monkeypatch):
    monkeypatch.setattr(reactive.CE,'tube_clearance',lambda q,t:-100.*t)
    trail=SimpleNamespace(pose=lambda t:t)
    q=np.zeros(6)
    assert reactive.obstacle_clock(1.,3.,q,trail,diagnostic_yield=True)==(1.,True)
    assert reactive.obstacle_clock(1.,3.,q,trail)==(3.,False)
