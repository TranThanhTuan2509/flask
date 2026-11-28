"""How deep and how long are the contacts of the 92/6990 Panda paths that fail the dense check (campaign v10b,
default `native.plan`, peak-speed edge validation)? Reads the raw per-attempt records of table_ii_v10b_rest.

    python results/diagnostics/panda_default_contacts/contacts.py <path to table_ii_v10b_rest/attempts.jsonl>
"""
import json
import sys
from pathlib import Path

import numpy as np

rows = [json.loads(line) for line in open(sys.argv[1])]
bad = [r for r in rows if r['solved'] and not r['validation']['valid']]
ms = np.array([r['validation']['dense']['collision_count'] for r in bad], float)      # 1 kHz samples = ms
clearance = np.array([r['validation']['independent_environment']['minimum_sampled_clearance_m'] for r in bad])
out = dict(source='results/paper_campaigns/2026-11-25_final_v10b/table_ii_v10b_rest/attempts.jsonl (raw, local)',
           attempts=len(rows), solved=sum(r['solved'] for r in rows), returned_invalid=len(bad),
           all_invalid_collide_in_dense_check=bool(all(r['validation']['dense']['collision'] for r in bad)),
           contact_duration_ms=dict(median=float(np.median(ms)), p90=float(np.percentile(ms, 90)), max=float(ms.max())),
           deepest_sampled_penetration_m=float(-clearance.min()),
           caught_by_sparse_independent_check=int(sum(r['validation']['independent_environment']['environment_collision'] for r in bad)),
           note='Penetration is the most negative sampled clearance of the independent environment check (sparse samples), '
                'so the true depth can be larger. Durations come from the dense 1 kHz VAMP check.')
Path(__file__).with_name('summary.json').write_text(json.dumps(out, indent=1) + '\n')
print(json.dumps(out, indent=1))
