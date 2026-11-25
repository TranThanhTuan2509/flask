"""CONNECT must respect resource limits even with very short range steps."""
import json
import subprocess
import sys
import numpy as np
import pytest
from flask_repro import mbm,native,robot


@pytest.mark.parametrize('fraction',[.02,.002,.0002])
def test_ompl_connect_obeys_capacity_and_returns_before_external_watchdog(fraction):
    # The old inner CONNECT ignored both the node cap and the one-millisecond
    # deadline. Each of these probes reached a three-second external watchdog.
    code=f'''
import json
from flask_repro import mbm,native,robot
rb=robot.get_robot('panda');p=mbm.load_dataset('panda')['table_pick'][1]
settings=native.settings(extend_mode=1,range_fraction={fraction!r},max_ns=1000000,
    max_iterations=1000,max_samples=4,velocity_sample_scale=0.)
r=native.plan(rb,mbm.native_environment(p),p['start'],p['goals'],settings,
    seed=1001,shortcut=False)
print(json.dumps({{k:r[k] for k in ['start_nodes','goal_nodes','plan_ns']}}))
'''
    result=subprocess.run([sys.executable,'-c',code],capture_output=True,text=True,timeout=3.,check=True)
    row=json.loads(result.stdout)
    assert row['start_nodes']+row['goal_nodes']<=4


@pytest.mark.parametrize('mode',[0,1])
def test_resting_identity_query_needs_no_search(mode):
    rb=robot.get_robot('panda');p=mbm.load_dataset('panda')['table_pick'][1]
    result=native.plan(rb,mbm.native_environment(p),p['start'],[p['start']],
        native.settings(extend_mode=mode,max_iterations=0,max_ns=1,max_samples=4),shortcut=False)
    assert result['solved'] and result['iterations']==0
    np.testing.assert_array_equal(result['trajectory'].Q,[p['start']])
    assert len(result['trajectory'].T)==0


@pytest.mark.parametrize('mode',[0,1])
def test_expired_search_budget_does_not_enter_expansion(mode):
    rb=robot.get_robot('panda');p=mbm.load_dataset('panda')['table_pick'][1]
    result=native.plan(rb,mbm.native_environment(p),p['start'],p['goals'],
        native.settings(extend_mode=mode,max_ns=1,max_iterations=1000,max_samples=10),shortcut=False)
    assert not result['solved']
    # A resource check may count the first attempted iteration, but cannot
    # advance through sixteen unchecked iterations or enter CONNECT.
    assert result['iterations']<=1 and result['accepted_extensions']==0
