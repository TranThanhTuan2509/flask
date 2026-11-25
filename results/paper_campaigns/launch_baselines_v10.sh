#!/bin/bash
# v10 baselines on this machine, AFTER the flask-final-v10 unit: (1) seeded iDb-A* primitive banks (untimed, E-cores,
# counts/generators declared on the laptop for v9: unicycle 5000 optimize, quad2d 2000 optimize, quad3d 5000 random
# rollouts), (2) upstream Dynoplan SST and (3) iDb-A*, 180 s anytime budget, held-out seeds 5001-5030, measurement CPU.
set -uo pipefail
ROOT="$(cd "$(dirname "$0")/../.." && pwd)"
cd "$ROOT" || exit 3
export PYTHONPATH="$ROOT"
PY=.venv/bin/python
while systemctl --user is-active --quiet flask-final-v10; do sleep 60; done
DAY=$(date +%Y-%m-%d)
$PY -m benchmarks.idbastar_campaign primitives --model unicycle1_flask --count 5000 --generator optimize --cpu 20 --seed 1 > build/idbastar_primitives_unicycle.log 2>&1 &
$PY -m benchmarks.idbastar_campaign primitives --model quad2d_flask --count 2000 --generator optimize --cpu 24 --seed 1 > build/idbastar_primitives_quad2d.log 2>&1 &
$PY -m benchmarks.idbastar_campaign primitives --model quad3d_flask --count 5000 --generator rand --cpu 28 --seed 1 > build/idbastar_primitives_quad3d.log 2>&1 &
wait
$PY -m benchmarks.upstream_sst_campaign run --output results/paper_campaigns/${DAY}_upstream_sst_v10 --seeds $(seq 5001 5030) > results/paper_campaigns/${DAY}_upstream_sst_v10.log 2>&1
$PY -m benchmarks.idbastar_campaign run --output results/paper_campaigns/${DAY}_idbastar_v10 --seeds $(seq 5001 5030) > results/paper_campaigns/${DAY}_idbastar_v10.log 2>&1
