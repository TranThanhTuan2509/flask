"""Build a source-preserving author RRTConnect reference with bounded CONNECT.

The generated class is an exact renamed copy of the pinned author class with
one additional termination-condition check. Upstream files remain untouched.
"""
from pathlib import Path
import difflib
import hashlib
import json
import subprocess

ROOT=Path(__file__).resolve().parents[1]
SOURCE=ROOT/'third_party/ompl_flask_author'
BUILD=ROOT/'build/ompl_flask_author'
OUT=ROOT/'build/author_flask_budgeted'


def main():
    OUT.mkdir(parents=True,exist_ok=True)
    header=SOURCE/'src/ompl/geometric/planners/rrt/RRTConnect.h'
    source=SOURCE/'src/ompl/geometric/planners/rrt/src/RRTConnect.cpp'
    original_h=header.read_text();original_cc=source.read_text()
    needle='while (gsc == ADVANCED)'
    if original_cc.count(needle)!=1:
        raise RuntimeError('Pinned CONNECT loop changed; inspect before rebuilding')
    derived_h=original_h.replace('RRTConnect','BudgetedRRTConnect').replace('RRTCONNECT','BUDGETED_RRTCONNECT')
    derived_cc=original_cc.replace(needle,'while (gsc == ADVANCED && !ptc)')
    derived_cc=derived_cc.replace('"ompl/geometric/planners/rrt/RRTConnect.h"','"BudgetedRRTConnect.h"')
    derived_cc=derived_cc.replace('RRTConnect','BudgetedRRTConnect')
    # The include was already renamed before replacing class tokens.
    derived_cc=derived_cc.replace('BudgetedBudgetedRRTConnect.h','BudgetedRRTConnect.h')
    (OUT/'BudgetedRRTConnect.h').write_text(derived_h)
    (OUT/'BudgetedRRTConnect.cpp').write_text(derived_cc)
    diff=''.join(difflib.unified_diff(original_cc.splitlines(True),
        original_cc.replace(needle,'while (gsc == ADVANCED && !ptc)').splitlines(True),
        fromfile='upstream/RRTConnect.cpp',tofile='derived/RRTConnect.cpp'))
    (OUT/'termination_only.patch').write_text(diff)
    command=['c++','-O3','-DNDEBUG','-march=native','-mavx2','-std=c++17',
        '-DFLASK_REFERENCE_BUDGETED','-DBOOST_ALL_NO_LIB',
        '-DBOOST_MATH_NO_LONG_DOUBLE_MATH_FUNCTIONS','-DBOOST_PROGRAM_OPTIONS_DYN_LINK','-DBOOST_SERIALIZATION_DYN_LINK',
        '-I'+str(OUT),'-I'+str(SOURCE/'src'),'-I'+str(BUILD/'src'),
        '-I'+str(SOURCE/'external/vamp/src/impl'),'-I'+str(BUILD/'_deps/simdxorshift-src/include'),
        '-isystem','/usr/include/eigen3','-isystem',str(BUILD/'_deps/nigh-src/src'),
        '-isystem',str(BUILD/'_deps/pdqsort-src'),
        str(ROOT/'benchmarks/author_flask/mbm_flat.cpp'),str(OUT/'BudgetedRRTConnect.cpp'),
        '-o',str(OUT/'mbm_flat'),'-Wl,-rpath,'+str(BUILD/'src/ompl'),
        str(BUILD/'src/ompl/libompl.so.2.0.2'),
        '/usr/lib/x86_64-linux-gnu/libboost_serialization.so.1.74.0',
        str(BUILD/'external/vamp/libsimdxorshift.a')]
    subprocess.run(command,check=True)
    hashed=[header,source,OUT/'BudgetedRRTConnect.h',OUT/'BudgetedRRTConnect.cpp',
            ROOT/'benchmarks/author_flask/mbm_flat.cpp',OUT/'mbm_flat',OUT/'termination_only.patch',
            BUILD/'src/ompl/libompl.so.2.0.2',BUILD/'external/vamp/libsimdxorshift.a']
    record=dict(author_commit=subprocess.check_output(['git','-C',str(SOURCE),'rev-parse','HEAD'],text=True).strip(),
        changes='Class/header renamed to avoid symbol interposition; sole algorithm change is checking !ptc during CONNECT.',
        upstream_modified=False,command=command,
        sha256={str(p.relative_to(ROOT)):hashlib.sha256(p.read_bytes()).hexdigest() for p in hashed})
    (OUT/'build_manifest.json').write_text(json.dumps(record,indent=2)+'\n')
    print(json.dumps(dict(driver=str(OUT/'mbm_flat'),algorithm_patch=diff)))


if __name__=='__main__':main()
