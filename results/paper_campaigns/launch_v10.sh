#!/bin/bash
# v10 controlled campaign on this machine (CPU from config/machine.json or $FLASK_CPU).
# Retries only while the campaign stops at its own preflight (no measurement data yet).
set -uo pipefail
ROOT="$(cd "$(dirname "$0")/../.." && pwd)"
cd "$ROOT" || exit 3
export PYTHONPATH="$ROOT"
OUT=results/paper_campaigns/$(date +%Y-%m-%d)_final_v10
for i in $(seq 240); do
  if .venv/bin/python -m benchmarks.measurement_env preflight --set-performance > /dev/null 2>&1; then
    .venv/bin/python -m benchmarks.run_final_campaign --plan v10 --output "$OUT" && exit 0
    if [ "$(ls "$OUT" 2>/dev/null)" = "preflight.json" ]; then rm -r "$OUT"; else exit 1; fi
  fi
  sleep 30
done
exit 2
