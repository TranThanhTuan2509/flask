"""Run registered paper-case reconstruction measurements sequentially.

This driver never renders or simulates a robot. See the pinned measurement
profiles for undisclosed author settings and reconstructed stopping rules.
"""
import argparse
from datetime import datetime, timezone
import json
import os
from pathlib import Path
import subprocess
import sys

ROOT = Path(__file__).resolve().parents[1]


def main(args):
    args.output.mkdir(parents=True, exist_ok=False)
    environment = os.environ.copy()
    environment.update(OPENBLAS_NUM_THREADS='1', OMP_NUM_THREADS='1', MKL_NUM_THREADS='1',
                       NUMEXPR_NUM_THREADS='1')
    jobs = [
        ('table_ii_panda', [sys.executable, '-m', 'benchmarks.manipulator_campaign',
            '--output', str(args.output / 'table_ii_panda'), '--cpu', str(args.cpu)]),
        ('table_i_low_dof', [sys.executable, '-m', 'benchmarks.native_campaign',
            '--output', str(args.output / 'table_i_low_dof'),
            '--profile', str(ROOT / 'config/low_dof_measurement_v1.json'),
            '--cpu', str(args.cpu), '--campaign-class', 'paper_case_reconstruction_measurement']),
    ]
    records = []
    for name, command in jobs:
        record = dict(job=name, command=command, start_utc=datetime.now(timezone.utc).isoformat())
        records.append(record)
        (args.output / 'jobs.json').write_text(json.dumps(records, indent=2) + '\n')
        print('START ' + name, flush=True)
        with (args.output / (name + '.log')).open('w') as stream:
            result = subprocess.run(command, cwd=ROOT, env=environment, stdout=stream, stderr=subprocess.STDOUT)
        record.update(end_utc=datetime.now(timezone.utc).isoformat(), exit_code=result.returncode)
        (args.output / 'jobs.json').write_text(json.dumps(records, indent=2) + '\n')
        print('END ' + name + ' exit=' + str(result.returncode), flush=True)
        # A driver error is preserved separately from a planner timeout. Run the
        # other independent campaign to retain useful measurements either way.
    if any(r['exit_code'] for r in records):
        raise SystemExit('One or more measurement drivers failed; inspect retained logs')


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--cpu', type=int, default=2)
    main(parser.parse_args())
