"""A stalled reference must terminate while preserving its completed raw output."""
import json
import os
import sys
from benchmarks.author_reference import run_driver


def test_watchdog_preserves_partial_result_and_kills_stalled_child(tmp_path):
    raw=tmp_path/'partial.jsonl'
    program="from pathlib import Path; import sys,time; Path(sys.argv[1]).write_text('{\"completed\":1}\\n'); time.sleep(60)"
    result=run_driver([sys.executable,'-c',program,str(raw)],raw,tmp_path/'child.log',os.environ,.25)
    assert result['idle_watchdog_triggered'] and result['returncode']!=0
    assert result['batch_wrapper_s']<5
    assert json.loads(raw.read_text())=={'completed':1}


def test_watchdog_preserves_normal_exit(tmp_path):
    raw=tmp_path/'normal.jsonl'
    program="from pathlib import Path; import sys; Path(sys.argv[1]).write_text('{\"completed\":1}\\n')"
    result=run_driver([sys.executable,'-c',program,str(raw)],raw,tmp_path/'child.log',os.environ,2.)
    assert result['returncode']==0 and not result['idle_watchdog_triggered']
