#!/usr/bin/env bash
# Core install: Python venv, VAMP (pinned + patch), MotionBenchMaker problems, native FLASK extension, tests.
# Tested on Ubuntu 22.04, Python 3.10, GCC 11.4, CMake 3.22, x86-64 with AVX2.
#   sudo apt install build-essential cmake git python3.10-venv python3.10-dev libeigen3-dev
#   ./install.sh               # core: planners, validators, tests
#   ./install.sh --showcase    # + MuJoCo showcases (Unitree G1 policy, Skydio X2, CPU torch)
set -euo pipefail
cd "$(dirname "$0")"
JOBS="${JOBS:-$(nproc)}"

scripts/fetch_third_party.sh ${1:-}
python3.10 -m venv .venv
.venv/bin/pip install -q --upgrade pip
.venv/bin/pip install -q -r requirements.txt
.venv/bin/pip install -q --no-build-isolation ./third_party/vamp
# MotionBenchMaker problems exactly as used for every reported result (sha256 in data/mbm/SHA256SUMS);
# regenerating them from VAMP's problems.tar.bz2 differs by ~1e-16 rad in a few box orientations.
(cd data/mbm && sha256sum -c --quiet SHA256SUMS)
for robot in panda ur5; do cp data/mbm/${robot}_problems.pkl third_party/vamp/resources/$robot/problems.pkl; done
PYTHON="$PWD/.venv/bin/python" ./build.sh "$JOBS"
.venv/bin/python -c "import site,sys; open(site.getsitepackages()[0]+'/flask_repro.pth','w').write(sys.argv[1])" "$PWD"
if [ "${1:-}" = "--showcase" ]; then
  .venv/bin/pip install -q -r requirements_showcase.txt --extra-index-url https://download.pytorch.org/whl/cpu
fi
.venv/bin/python -m pytest -q tests
echo "FLASK installed. Try: .venv/bin/python examples/panda_quickstart.py"
