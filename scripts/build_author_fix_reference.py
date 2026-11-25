"""Build the authors' RRTConnect reference with THEIR OWN loop fix (KavrakiLab/ompl-flask, branch
fix-rrt-connect-loop, commit 3026754e, 2026-09-28: "stop growing RRTConnect when interpolation gets no nearer").

The pinned `flat` tree (a81c3f09) predates the merge of that fix. The derived class is an exact renamed copy of
the pinned author RRTConnect with only the authors' growTree hunk applied verbatim (no change of ours, unlike
scripts/build_budgeted_author_reference.py, which adds a ptc check in CONNECT). Upstream files stay untouched.
The driver is benchmarks/author_flask/mbm_flat.cpp compiled with -DFLASK_REFERENCE_BUDGETED
-DFLASK_REFERENCE_AUTHOR_FIX (method name `author_flask_rrt_connect_author_fix`); `--rest` in
benchmarks/author_reference.py installs zero-velocity proposals through the derivative-sampler extension point.

Usage: python scripts/build_author_fix_reference.py   (needs build/ompl_flask_author, see README.md)
"""
import hashlib
import json
import subprocess
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
SOURCE = ROOT / 'third_party/ompl_flask_author'
BUILD = ROOT / 'build/ompl_flask_author'
OUT = ROOT / 'build/author_flask_fixed'
OLD = '''        /* Check if we have moved at all. Due to some stranger state spaces (e.g., the constrained state spaces),
         * interpolate can fail and no progress is made. Without this check, the algorithm gets stuck in a loop as it
         * thinks it is making progress, when none is actually occurring. */
        if (si_->equalStates(nmotion->state, tgi.xstate))
            return TRAPPED;'''
NEW = '''        /* Check if we have moved any nearer.
         * Interpolation can fail to move in constrained state spaces.
         * In others, such as spaces whose interpolation follows a trajectory, the interpolated state can land farther
         * from rmotion than nmotion is.
         * Either way the next call finds the same nearest motion and adds the same state again, so the connect loop in
         * solve() never ends. */
        if (si_->distance(tgi.xstate, rmotion->state) >= d)
            return TRAPPED;'''


def sha(p):
    return hashlib.sha256(Path(p).read_bytes()).hexdigest()


def main():
    OUT.mkdir(parents=True, exist_ok=True)
    header = SOURCE / 'src/ompl/geometric/planners/rrt/RRTConnect.h'
    source = SOURCE / 'src/ompl/geometric/planners/rrt/src/RRTConnect.cpp'
    cc = source.read_text()
    if cc.count(OLD) != 1:
        raise RuntimeError('Pinned growTree check changed; inspect before rebuilding')
    cc = cc.replace(OLD, NEW).replace('"ompl/geometric/planners/rrt/RRTConnect.h"', '"BudgetedRRTConnect.h"')
    cc = cc.replace('RRTConnect', 'BudgetedRRTConnect').replace('BudgetedBudgetedRRTConnect.h', 'BudgetedRRTConnect.h')
    h = header.read_text().replace('RRTConnect', 'BudgetedRRTConnect').replace('RRTCONNECT', 'BUDGETED_RRTCONNECT')
    (OUT / 'BudgetedRRTConnect.cpp').write_text(cc)
    (OUT / 'BudgetedRRTConnect.h').write_text(h)
    command = ['c++', '-O3', '-DNDEBUG', '-march=native', '-mavx2', '-std=c++17', '-DFLASK_REFERENCE_BUDGETED',
               '-DFLASK_REFERENCE_AUTHOR_FIX', '-DBOOST_ALL_NO_LIB', '-DBOOST_MATH_NO_LONG_DOUBLE_MATH_FUNCTIONS',
               '-DBOOST_PROGRAM_OPTIONS_DYN_LINK', '-DBOOST_SERIALIZATION_DYN_LINK', f'-I{OUT}', f'-I{SOURCE}/src',
               f'-I{BUILD}/src', f'-I{SOURCE}/external/vamp/src/impl', f'-I{BUILD}/_deps/simdxorshift-src/include',
               '-isystem', '/usr/include/eigen3', '-isystem', f'{BUILD}/_deps/nigh-src/src', '-isystem', f'{BUILD}/_deps/pdqsort-src',
               str(ROOT / 'benchmarks/author_flask/mbm_flat.cpp'), str(OUT / 'BudgetedRRTConnect.cpp'), '-o', str(OUT / 'mbm_flat'),
               f'-Wl,-rpath,{BUILD}/src/ompl', str(next((BUILD / 'src/ompl').glob('libompl.so.*.*.*'))),
               '/usr/lib/x86_64-linux-gnu/libboost_serialization.so.1.74.0', str(BUILD / 'external/vamp/libsimdxorshift.a')]
    subprocess.run(command, check=True)
    files = [header, source, OUT / 'BudgetedRRTConnect.h', OUT / 'BudgetedRRTConnect.cpp', ROOT / 'benchmarks/author_flask/mbm_flat.cpp', OUT / 'mbm_flat']
    (OUT / 'build_manifest.json').write_text(json.dumps(dict(
        author_commit='a81c3f09ab5ebce4a83a151e8170e5bdf19b94f9',
        author_fix=dict(branch='fix-rrt-connect-loop', commit='3026754e', date='2026-09-28',
                        subject='fix: stop growing RRTConnect when interpolation gets no nearer'),
        changes="pinned flat-branch RRTConnect + the authors' growTree hunk verbatim; class renamed; nothing else",
        command=command, sha256={str(f.relative_to(ROOT)): sha(f) for f in files}), indent=2) + '\n')
    print('built', OUT / 'mbm_flat')


if __name__ == '__main__':
    main()
