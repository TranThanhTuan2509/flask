"""Controlled timing environment: snapshot, preflight and a fixed-work probe.

Timing campaigns must record the power policy, not only instantaneous clocks: an energy preference
change (e.g. 'performance' -> 'power') silently slowed earlier campaigns by about 1.9x with identical
search work. `preflight` refuses to start unless the measurement CPU's energy preference is
'performance', mains/AC power is online (laptops), the SMT sibling of the measurement CPU is idle and, on hybrid
CPUs, the measurement CPU is a performance core.

The measurement CPU defaults to $FLASK_CPU, else to config/machine.json's "measurement_cpu" (machine specific,
never copied between machines), else 3 (the laptop record).

Usage:
  python -m benchmarks.measurement_env snapshot [--cpu N]
  python -m benchmarks.measurement_env preflight [--cpu N] [--set-performance]
  python -m benchmarks.measurement_env probe [--cpu N] --output probe.json
"""
import argparse
import glob
import json
import os
import subprocess
import time
from datetime import datetime, timezone
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]


def default_cpu():
    """Measurement CPU: $FLASK_CPU, else config/machine.json, else 3 (laptop record)."""
    if os.environ.get('FLASK_CPU'):
        return int(os.environ['FLASK_CPU'])
    machine = ROOT / 'config/machine.json'
    if machine.exists():
        return int(json.loads(machine.read_text())['measurement_cpu'])
    return 3


def _read(path):
    try:
        return Path(path).read_text().strip()
    except OSError:
        return None


def siblings(cpu):
    text = _read(f'/sys/devices/system/cpu/cpu{cpu}/topology/thread_siblings_list') or str(cpu)
    out = set()
    for part in text.split(','):
        if '-' in part:
            a, b = map(int, part.split('-'))
            out.update(range(a, b + 1))
        else:
            out.add(int(part))
    return sorted(out - {cpu})


def _cpu_times():
    rows = {}
    for line in Path('/proc/stat').read_text().splitlines():
        if line.startswith('cpu') and line[3:4].isdigit():
            name, *values = line.split()
            values = list(map(int, values))
            idle = values[3] + values[4]
            rows[int(name[3:])] = (sum(values), idle)
    return rows


def cpu_load(seconds=2.):
    a = _cpu_times(); time.sleep(seconds); b = _cpu_times()
    return {cpu: 1. - (b[cpu][1] - a[cpu][1]) / max(1, b[cpu][0] - a[cpu][0]) for cpu in a}


def snapshot(cpu=None, load_seconds=2.):
    cpu = default_cpu() if cpu is None else cpu
    per_cpu = {}
    for path in sorted(glob.glob('/sys/devices/system/cpu/cpu[0-9]*/cpufreq')):
        n = int(Path(path).parent.name[3:])
        per_cpu[n] = dict(governor=_read(f'{path}/scaling_governor'),
                          energy_performance_preference=_read(f'{path}/energy_performance_preference'),
                          cur_khz=_read(f'{path}/scaling_cur_freq'), max_khz=_read(f'{path}/scaling_max_freq'))
    power = {Path(p).parent.name + '/' + Path(p).name: _read(p)
             for p in glob.glob('/sys/class/power_supply/*/online') + glob.glob('/sys/class/power_supply/*/status')
             + glob.glob('/sys/class/power_supply/*/type')}
    max_freqs = {n: _read(f'/sys/devices/system/cpu/cpu{n}/cpufreq/cpuinfo_max_freq') for n in per_cpu}
    thermal = {Path(p).parent.name: _read(p) for p in glob.glob('/sys/class/thermal/thermal_zone*/temp')}
    try:
        profile = subprocess.run(['powerprofilesctl', 'get'], capture_output=True, text=True, timeout=5).stdout.strip()
    except (OSError, subprocess.SubprocessError):
        profile = None
    try:
        top = subprocess.run(['ps', '-eo', 'pid,psr,pcpu,comm', '--sort=-pcpu'], capture_output=True, text=True,
                             timeout=5).stdout.splitlines()[:12]
    except (OSError, subprocess.SubprocessError):
        top = []
    load = cpu_load(load_seconds) if load_seconds else {}
    return dict(timestamp_utc=datetime.now(timezone.utc).isoformat(), measurement_cpu=cpu, smt_siblings=siblings(cpu),
                affinity=sorted(os.sched_getaffinity(0)), power_profile=profile,
                no_turbo=_read('/sys/devices/system/cpu/intel_pstate/no_turbo'), per_cpu=per_cpu, power_supply=power,
                cpuinfo_max_khz=max_freqs,
                thermal_millicelsius=thermal, cpu_load=load, top_processes=top)


def check(snap, max_sibling_load=.05):
    """Return the list of violated measurement conditions (empty when acceptable)."""
    problems = []
    cpu = snap['measurement_cpu']
    epp = (snap['per_cpu'].get(cpu) or {}).get('energy_performance_preference')
    if epp not in ('performance', None):
        problems.append(f'energy_performance_preference={epp}')
    if snap['power_profile'] not in ('performance', None):
        problems.append(f"power_profile={snap['power_profile']}")
    if snap['no_turbo'] == '1':
        problems.append('turbo disabled')
    # Only mains adapters indicate AC power; USB-C/UCSI 'USB' entries (desktop ports) are device sources.
    power = snap['power_supply']
    mains = [k.split('/')[0] for k, v in power.items() if k.endswith('/type') and v == 'Mains']
    if mains and not any(power.get(f'{m}/online') == '1' for m in mains):
        problems.append('AC power offline')
    if any(v == 'Discharging' for k, v in power.items() if k.endswith('/status')):
        problems.append('battery discharging')
    # Hybrid CPUs: the measurement CPU must be a performance core (highest cpuinfo_max_freq class).
    freqs = {int(k): int(v) for k, v in snap.get('cpuinfo_max_khz', {}).items() if v}
    if freqs and freqs.get(cpu, 0) < .9 * max(freqs.values()):
        problems.append(f'cpu{cpu} is not a performance core ({freqs.get(cpu)} kHz < {max(freqs.values())} kHz)')
    for s in snap['smt_siblings']:
        if snap['cpu_load'].get(s, 0.) > max_sibling_load:
            problems.append(f'SMT sibling cpu{s} load {snap["cpu_load"][s]:.0%}')
    if snap['cpu_load'].get(cpu, 0.) > max_sibling_load:
        problems.append(f'measurement cpu{cpu} busy {snap["cpu_load"][cpu]:.0%}')
    return problems


def preflight(cpu=None, set_performance=False, max_sibling_load=.05):
    cpu = default_cpu() if cpu is None else cpu
    if set_performance:
        subprocess.run(['powerprofilesctl', 'set', 'performance'], check=False)
    snap = snapshot(cpu, load_seconds=5.)  # 5 s window: short windows over-weight transient UI bursts
    return snap, check(snap, max_sibling_load)


def fixed_work_probe(cpu=None, problems=50, repeats=5):
    """Deterministic workload: VAMP geometric RRTConnect on fixed Halton subsequences + native FLASKCC."""
    import numpy as np
    import vamp
    from flask_repro import lqmt, mbm, native, robot
    cpu = default_cpu() if cpu is None else cpu
    os.sched_setaffinity(0, {cpu})
    dataset = mbm.load_dataset('panda')
    module, planner, settings, _ = vamp.configure_robot_and_planner_with_kwargs('panda', 'rrtc')
    items = [p for env in mbm.ENVIRONMENTS for p in dataset[env] if p['valid']][:problems]
    vamp_ns = []
    for r in range(repeats):
        total = 0
        for p in items:
            ev = vamp.problem_dict_to_vamp(p)
            sampler = module.halton(); sampler.skip(0)
            result = planner(list(p['start']), [list(g) for g in p['goals']], ev, settings, sampler)
            total += result.nanoseconds
        vamp_ns.append(total)
    rb = robot.get_robot('panda'); mod = native.module(rb); limits = native.joint_limits(rb, 'published_resolution')
    rng = np.random.default_rng(7)
    edges = []
    for p in items[:20]:
        ec = mbm.native_environment(p)
        for _ in range(100):
            q0 = rng.uniform(rb.q_lo, rb.q_hi); q1 = rng.uniform(rb.q_lo, rb.q_hi)
            T = lqmt.optimal_duration(q0, 0 * q0, q1, 0 * q1, 1.)
            edges.append((ec, q0.tolist(), q1.tolist(), T))
    cc_ns = []
    for r in range(repeats):
        t0 = time.perf_counter_ns()
        for ec, q0, q1, T in edges:
            mod.flask_cc(ec, q0, [0.] * 7, q1, [0.] * 7, T, limits, False)
        cc_ns.append(time.perf_counter_ns() - t0)
    return dict(vamp_rrtc_ns=vamp_ns, vamp_rrtc_median_ns=float(np.median(vamp_ns)),
                flask_cc_ns=cc_ns, flask_cc_median_ns=float(np.median(cc_ns)), problems=len(items), edges=len(edges))


if __name__ == '__main__':
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('command', choices=['snapshot', 'preflight', 'probe'])
    ap.add_argument('--cpu', type=int, default=None)
    ap.add_argument('--set-performance', action='store_true')
    ap.add_argument('--output', type=Path)
    a = ap.parse_args()
    if a.command == 'snapshot':
        out = snapshot(a.cpu)
    elif a.command == 'preflight':
        snap, problems = preflight(a.cpu, a.set_performance)
        out = dict(snapshot=snap, problems=problems, acceptable=not problems)
    else:
        out = fixed_work_probe(a.cpu)
    text = json.dumps(out, indent=2)
    if a.output:
        a.output.write_text(text + '\n')
    print(text if a.command != 'snapshot' else json.dumps({k: out[k] for k in ['power_profile', 'smt_siblings', 'no_turbo']}))
    if a.command == 'preflight' and out['problems']:
        raise SystemExit('Measurement conditions not met: ' + '; '.join(out['problems']))
