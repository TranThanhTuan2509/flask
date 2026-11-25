"""Provenance: FLASK implementation and evaluation utilities."""
from dataclasses import asdict, is_dataclass
from datetime import datetime
from hashlib import sha256
from importlib.metadata import version, PackageNotFoundError
from pathlib import Path
import json
import os
import platform
import subprocess
from zoneinfo import ZoneInfo
import numpy as np
ROOT = Path(__file__).resolve().parents[1]

def file_hash(path):
    return sha256(Path(path).read_bytes()).hexdigest()

def _json(value):
    if is_dataclass(value):
        return asdict(value)
    if isinstance(value, np.ndarray):
        return value.tolist()
    if isinstance(value, np.generic):
        return value.item()
    if isinstance(value, Path):
        return str(value)
    raise TypeError(type(value).__name__)

def manifest(configuration, inputs=(), fidelity='implemented_different_conditions'):
    packages = {}
    for name in ['numpy', 'scipy', 'mpmath', 'vamp-planner', 'nanobind', 'toppra', 'ruckig', 'mujoco']:
        try:
            packages[name] = version(name)
        except PackageNotFoundError:
            packages[name] = None
    paths = list(ROOT.glob('src/**/*.hh')) + list(ROOT.glob('src/**/*.cc'))
    paths += list(ROOT.glob('flask_repro/*.py')) + list(ROOT.glob('benchmarks/*.py'))
    paths += list(ROOT.glob('experiments/*.py')) + list(ROOT.glob('simulation/*.py'))
    paths += list(ROOT.glob('scripts/*.py')) + list(ROOT.glob('tests/*.py'))
    paths += list(ROOT.glob('config/**/*.json'))
    paths += [ROOT / 'CMakeLists.txt', ROOT / 'third_party/VAMP_COMMIT']
    paths += list(ROOT.glob('requirements*.txt'))
    paths += list(ROOT.glob('scripts/*.sh')) + list(ROOT.glob('results/paper_campaigns/launch_*.sh'))  # v10: launch/setup scripts
    paths += [ROOT / 'build.sh', ROOT / 'environment.sh']
    paths += list(ROOT.glob('flask_repro/_core*.so'))
    paths += [ROOT / 'third_party/vamp_capture_validation.patch', ROOT / 'third_party/vamp/src/impl/vamp/collision/capt.hh', ROOT / 'third_party/vamp/src/impl/vamp/collision/filter.hh']
    cpu = next((s.split(':', 1)[1].strip() for s in Path('/proc/cpuinfo').read_text().splitlines() if s.startswith('model name')), platform.processor())
    compiler = subprocess.run(['c++', '--version'], capture_output=True, text=True).stdout.splitlines()[0]
    flags = ROOT / 'build/CMakeFiles/_core.dir/flags.make'
    return dict(schema_version=1, created_at=datetime.now(ZoneInfo('Asia/Ho_Chi_Minh')).isoformat(), fidelity=fidelity, configuration=configuration, machine=dict(platform=platform.platform(), cpu=cpu, affinity=sorted(os.sched_getaffinity(0)), compiler=compiler, compile_flags=flags.read_text() if flags.exists() else None), dependencies=packages, source_sha256={str(p.relative_to(ROOT)): file_hash(p) for p in sorted(paths) if p.exists()}, input_sha256={str(p): (file_hash(p) if Path(p).exists() else None) for p in inputs}, timing_note='Consult experiment configuration for timing boundaries; source hashes identify this workspace without a Git revision')

def save_manifest(output, configuration, inputs=(), **kw):
    path = Path(str(output) + '.manifest.json')
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(manifest(configuration, inputs, **kw), default=_json, indent=2), encoding='utf-8')
    return path

def distribution(values):
    a = np.asarray(values, float)
    if not len(a):
        return dict(n=0, mean=None, sd=None, p25=None, p50=None, p75=None, p95=None)
    if not np.isfinite(a).all():
        raise ValueError('Distribution values must all be finite')
    return dict(n=len(a), mean=float(a.mean()), sd=float(a.std(ddof=0)), **dict(zip(['p25', 'p50', 'p75', 'p95'], np.percentile(a, [25, 50, 75, 95]).tolist())))

def summarize(rows):
    """Summarize."""
    eligible = [r for r in rows if r.get('eligible', True)]
    success = [r for r in eligible if r['solved']]
    validated = [r for r in success if r.get('validated') is True]
    collision_checked = [r for r in success if isinstance(r.get('collision'), bool)]
    return dict(available=len(rows), attempted=len(eligible), excluded=len(rows) - len(eligible), solved=len(success), validated=len(validated), success_rate=len(success) / len(eligible) if eligible else None, failure_reasons={reason: sum((r.get('reason') == reason for r in eligible if not r['solved'])) for reason in sorted({r.get('reason', 'unknown') for r in eligible if not r['solved']})}, attempt_wall_ms=distribution([r['wall_ms'] for r in eligible if 'wall_ms' in r]), successful_total_ms=distribution([r['total_ms'] for r in success]), successful_length=distribution([r['length'] for r in success if r.get('length') is not None]), collision_risk=sum((r['collision'] for r in collision_checked)) / len(collision_checked) if collision_checked else None, collision_checked_successes=len(collision_checked), unchecked_successes=sum((r.get('collision') is None for r in success)))
