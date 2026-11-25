#!/usr/bin/env bash
set -euo pipefail
cd "$(dirname "$0")"
source environment.sh
JOBS="${1:-3}"
mkdir -p build
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release -DPython_EXECUTABLE="${PYTHON:-$PWD/.venv/bin/python}" > build/cmake.log 2>&1 || { cat build/cmake.log; exit 1; }
cmake --build build -j "$JOBS"
