"""A missing or incompatible native extension gives an actionable ImportError, not `'NoneType' has no attribute`.

Found in practice: running with an interpreter other than the one ./install.sh built for (e.g. a conda Python 3.12)
made `flask_repro._core` None, and every plan crashed with an AttributeError deep inside the world setup.
"""
import numpy as np
import pytest

import flask_repro
from flask_repro import flat_planner, native, planning
from flask_repro.flat_models import Unicycle
from flask_repro.flat_planner import World


@pytest.fixture
def without_extension(monkeypatch):
    monkeypatch.setattr(flask_repro, '_core', None)
    monkeypatch.setattr(flask_repro, '_core_error', ImportError('simulated: built for another Python'))


def test_require_native_returns_the_module():
    assert flask_repro.require_native() is flask_repro._core is not None


def test_every_entry_point_explains_the_missing_extension(without_extension):
    world = World(np.zeros(2), np.ones(2) * 5, [], [], 0.1)
    calls = [lambda: flask_repro.require_native(), lambda: world.prepare_simd(), lambda: native.settings(),
             lambda: native.plan(None, None, [], []), lambda: native.module(None),
             lambda: planning.plan(Unicycle(), world, np.array([1., 1, 0, 0]), np.array([2., 2, 0, 0]))]
    for call in calls:
        with pytest.raises(ImportError, match=r'install\.sh.*simulated|simulated.*install\.sh'):
            call()
