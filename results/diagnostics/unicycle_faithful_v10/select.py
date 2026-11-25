"""Apply the pre-declared rule (config/ablation_unicycle_faithful_v10.json, _selection_rule) to rounds 1-2."""
import json, statistics as st
from pathlib import Path
HERE = Path(__file__).resolve().parent
rows = {}
for d in sorted(HERE.glob('[UR]*_*')):
    if d.is_dir():
        for f in d.glob('*.jsonl'):
            rows[d.name] = [json.loads(l) for l in f.read_text().splitlines() if l.strip()]
table = []
for name, g in rows.items():
    per = {}
    for case in ['unicycle_bugtrap', 'unicycle_wall']:
        c = [r for r in g if r['case'] == case]
        ok = [r for r in c if r['valid']]
        per[case] = dict(valid=len(ok), attempts=len(c), edges50=st.median([r['rejections']['edges'] for r in c]),
                         iterations50=st.median([r['iterations'] for r in c]),
                         median_ms_informative=st.median([r['plan_ms'] + r['post_ms'] for r in ok]) if ok else None,
                         length_informative=st.mean([r['length'] for r in ok]) if ok else None,
                         angular_rejection_share=sum(r['rejections']['angular'] for r in c) / max(1, sum(r['rejections']['edges'] for r in c)))
    valid = sum(p['valid'] for p in per.values()); attempts = sum(p['attempts'] for p in per.values())
    table.append(dict(name=name, valid=valid, attempts=attempts, eligible=valid == attempts == 20,
                      edges=sum(p['edges50'] for p in per.values()), iterations=sum(p['iterations50'] for p in per.values()), cases=per))
eligible = [t for t in table if t['eligible']]
pool = eligible or sorted(table, key=lambda t: -t['valid'])[:1]
winner = min(pool, key=lambda t: (-t['valid'], t['edges'], t['iterations']))
out = dict(rule='see config/ablation_unicycle_faithful_v10.json _selection_rule', winner=winner['name'],
           candidates=sorted(table, key=lambda t: (not t['eligible'], t['edges'])))
(HERE / 'selection.json').write_text(json.dumps(out, indent=1) + '\n')
for t in out['candidates']:
    print(f"{t['name']:26s} valid {t['valid']}/{t['attempts']} eligible {t['eligible']!s:5s} edges {t['edges']:>9.0f}  "
          + ' | '.join(f"{c[9:]} {p['median_ms_informative'] and round(p['median_ms_informative'],1)} ms len {p['length_informative'] and round(p['length_informative'],2)} ang {p['angular_rejection_share']:.2f}" for c, p in t['cases'].items()))
print('WINNER', winner['name'])
