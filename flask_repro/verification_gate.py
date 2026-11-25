"""Require verified algorithm and experiment evidence before showcase production."""
from pathlib import Path
import hashlib,json
ROOT=Path(__file__).resolve().parents[1]
GATE=ROOT/'results/verification/reproduction_gate.json'
REQUIRED=('mathematical_oracles','native_algorithm_contracts','low_dof_repeat_campaign','matched_cpu_baselines','manipulator_repeat_campaign','collision_resolution_convergence','paper_findings_comparison')

def require_verified_reproduction(action):
    if not GATE.is_file():raise RuntimeError(f'{action} is locked: reproduction evidence has not passed verification.')
    record=json.loads(GATE.read_text());failed=[name for name in REQUIRED if record.get('criteria',{}).get(name,{}).get('passed') is not True]
    for relative,digest in record.get('verified_source_sha256',{}).items():
        path=ROOT/relative
        if not path.is_file() or hashlib.sha256(path.read_bytes()).hexdigest()!=digest:failed.append(f'source_changed:{relative}')
    if not record.get('verified_source_sha256'):failed.append('missing_verified_source_manifest')
    if failed:raise RuntimeError(f'{action} is locked until algorithm and experiment verification passes: '+', '.join(failed)+'. See results/verification/reproduction_gate.json.')
