#!/bin/bash
# V4: rerun every order-4 attempt unsolved at 60 s (v10 + v10b) with a 600 s budget; 8 E-cores, one job per (case, algorithm, seed group).
cd "$(dirname "$0")/../../../.."
O=results/verification/2026-11-25_acceptance/v4_long_budget
U=results/verification/2026-11-25_acceptance/v4_long_budget/unsolved_at_60s.json
jobs=$(python3 -c "
import json;d=json.load(open('$U'));out=[]
for k,s in d.items():
  c,a=k.split('|')
  for i in range(0,len(s),6): out.append(f'{c} {a} '+' '.join(map(str,s[i:i+6])))
print('\n'.join(out))")
cpus=(16 17 19 20 22 24 26 28 29 30 31)
i=0
while read -r c a seeds; do
  cpu=${cpus[$((i % ${#cpus[@]}))]}
  .venv/bin/python -m benchmarks.native_campaign --output $O/${c}_${a}_$i --profile $O/profile_budget600.json --cases $c --algorithms $a --seeds $seeds --cpu $cpu > $O/${c}_${a}_$i.log 2>&1 &
  i=$((i+1))
  if (( i % ${#cpus[@]} == 0 )); then wait; fi
done <<< "$jobs"
wait
