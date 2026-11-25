#!/usr/bin/env bash
# Run registered Panda experiments sequentially on one logical CPU.
set -euo pipefail
cd "$(dirname "$0")/.."
export OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 MKL_NUM_THREADS=1
exec .venv/bin/python -m benchmarks.manipulator_campaign "$@"
