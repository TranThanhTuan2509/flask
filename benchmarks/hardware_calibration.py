"""Hardware calibration factor k_hw against the paper's CPU (Intel i7-6900K).

Table II reports the time of VAMP's geometric RRTConnect ("RRTConnect time" of the baseline) on the
same MotionBenchMaker problems. Running the same planner on the same problems here gives a direct,
planner-independent ratio local/paper. Raw local FLASK times are always reported; report_v10 multiplies
paper times by this ratio only to place them in the two-sided comparison band.

Caveats recorded in the output: the paper's exact VAMP revision/settings are unknown, medians of
~0.05-0.1 ms are close to timer and cache noise, and laptop turbo/thermal state matters. Run with the
machine otherwise idle.

Usage: python -m benchmarks.hardware_calibration --output config/hardware_calibration.json --cpu 3
"""
import argparse
import glob
import json
import os
import platform
from pathlib import Path

import numpy as np
import vamp

from flask_repro import mbm
from benchmarks.measurement_env import default_cpu  # $FLASK_CPU or config/machine.json

ROOT = Path(__file__).resolve().parents[1]


def sensors():
    out = {}
    for pattern in ['/sys/class/thermal/thermal_zone*/temp', '/sys/devices/system/cpu/cpu*/cpufreq/scaling_cur_freq',
                    '/sys/devices/system/cpu/cpu*/cpufreq/scaling_governor', '/sys/class/power_supply/*/online',
                    '/sys/class/power_supply/*/status']:
        for name in glob.glob(pattern):
            try:
                out[name] = Path(name).read_text().strip()
            except OSError:
                pass
    return out


def main(args):
    os.sched_setaffinity(0, {args.cpu})
    paper = json.loads((ROOT / 'config/paper_table_ii_components.json').read_text())['vamp_toppra']
    dataset = mbm.load_dataset('panda')
    module, planner, settings, _ = vamp.configure_robot_and_planner_with_kwargs('panda', 'rrtc')
    before = sensors()
    times = {env: [] for env in mbm.ENVIRONMENTS}
    for repeat in range(args.repeats):
        for env in mbm.ENVIRONMENTS:
            for problem in dataset[env]:
                if not problem['valid']:
                    continue
                ev = vamp.problem_dict_to_vamp(problem)
                sampler = module.halton()
                sampler.skip(repeat * 2048)
                result = planner(list(problem['start']), [list(g) for g in problem['goals']], ev, settings, sampler)
                if result.solved:
                    times[env].append(result.nanoseconds / 1e6)
    after = sensors()
    rows = {}
    all_times = []
    for env in mbm.ENVIRONMENTS + ['overall']:
        t = np.asarray(all_times if env == 'overall' else times[env])
        if env != 'overall':
            all_times += times[env]
            t = np.asarray(times[env])
        ref = paper[env]['rrtc']
        rows[env] = dict(n=int(t.size), local_mean_ms=float(t.mean()), local_median_ms=float(np.median(t)),
                         local_p75_ms=float(np.percentile(t, 75)), paper_mean_ms=ref[0], paper_median_ms=ref[3],
                         paper_p75_ms=ref[4], ratio_mean=float(t.mean() / ref[0]),
                         ratio_median=float(np.median(t) / ref[3]), ratio_p75=float(np.percentile(t, 75) / ref[4]))
    ratios = [rows[e][k] for e in mbm.ENVIRONMENTS for k in ['ratio_mean', 'ratio_median', 'ratio_p75']]
    k_hw = float(np.median(ratios))
    report = dict(k_hw=k_hw, definition='median over 7 environments x (mean, median, p75) of local/paper VAMP RRTConnect time',
                  k_hw_overall_mean=rows['overall']['ratio_mean'], k_hw_overall_median=rows['overall']['ratio_median'],
                  k_hw_range=[float(min(ratios)), float(max(ratios))], per_environment=rows,
                  vamp_settings={n: getattr(settings, n) for n in dir(settings) if not n.startswith('_') and not callable(getattr(settings, n))},
                  vamp_version=getattr(vamp, '__version__', None), repeats=args.repeats, cpu_affinity=args.cpu,
                  processor=platform.processor(), cpu_model=_cpu_model(), sensors_before=before, sensors_after=after,
                  caveats=['Paper VAMP revision and settings unknown; local VAMP is third_party/vamp (VAMP_COMMIT)',
                           'Sub-0.1 ms medians are sensitive to timer, cache and frequency noise',
                           'Laptop turbo and thermal state are not controlled; governor and power state recorded'])
    args.output.write_text(json.dumps(report, indent=2) + '\n')
    print(json.dumps({k: report[k] for k in ['k_hw', 'k_hw_overall_mean', 'k_hw_overall_median', 'k_hw_range']}, indent=2))
    for env, r in rows.items():
        print(f"{env:17s} local {r['local_mean_ms']:.3f}/{r['local_median_ms']:.3f}  paper {r['paper_mean_ms']}/{r['paper_median_ms']}"
              f"  ratio {r['ratio_mean']:.2f}/{r['ratio_median']:.2f}/{r['ratio_p75']:.2f}")


def _cpu_model():
    for line in Path('/proc/cpuinfo').read_text().splitlines():
        if line.startswith('model name'):
            return line.split(':', 1)[1].strip()


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, required=True)  # never overwrite another machine's record
    parser.add_argument('--cpu', type=int, default=default_cpu())
    parser.add_argument('--repeats', type=int, default=10)
    main(parser.parse_args())
