#!/usr/bin/env bash
# Rebuild every environment and binary of the FLASK reproduction on a new machine (see README.md, Installation).
# Reconstructed from the laptop's CMakeCache files and pip freezes (2026-11-24). Run step by step the first time;
# every step ends with a check. Binaries built with -march=native (native extension, author drivers) MUST be
# rebuilt on each CPU; nothing under build/ or .venv*/ is portable.
#
# Laptop reference toolchain: Ubuntu 22.04.5, python3.10.12 (venvs), gcc 11.4, cmake 3.22.1, Boost 1.74, Eigen 3.4.
# System packages (once, needs sudo):
#   sudo apt install build-essential cmake python3.10-venv python3.10-dev libboost-all-dev libeigen3-dev \
#        power-profiles-daemon git libyaml-cpp-dev liblz4-dev
set -euo pipefail
ROOT="$(cd "$(dirname "$0")/.." && pwd)"
cd "$ROOT"
JOBS="${JOBS:-$(nproc)}"
step() { echo; echo "=== $*"; }

step "0. move binaries copied from another machine out of the tree (never reuse them)"
# A copied build/ records the old source directory and may contain AVX-512 code (-march=native).
if [ -f build/CMakeCache.txt ] && ! grep -q "CMAKE_HOME_DIRECTORY:INTERNAL=$ROOT\$" build/CMakeCache.txt; then
  DEST="$ROOT/../flask_repro_backups/foreign_build_$(date +%Y%m%d_%H%M%S)"; mkdir -p "$DEST"
  mv build "$DEST/build"; [ -d third_party/vamp/build ] && mv third_party/vamp/build "$DEST/vamp_build"
  [ -f flask_repro/_core.cpython-310-x86_64-linux-gnu.so ] && mv flask_repro/_core.cpython-310-x86_64-linux-gnu.so "$DEST/"
  echo "moved foreign binaries to $DEST"
fi

step "1. planning venv (.venv) + VAMP + native extension"
python3.10 -m venv --clear .venv                  # --clear: copied venvs keep the old machine's shebangs
grep -v '^vamp-planner' requirements_lock_laptop.txt > /tmp/flask_requirements_lock.txt   # VAMP is built from third_party below
.venv/bin/pip install -r /tmp/flask_requirements_lock.txt
.venv/bin/pip install --no-build-isolation ./third_party/vamp
PYTHON="$ROOT/.venv/bin/python" ./build.sh "$JOBS"
for so in flask_repro/_core*.so .venv/lib/python3.10/site-packages/vamp/_core/_core_ext*.so; do
  n=$(objdump -d "$so" | grep -c zmm || true)
  grep -q avx512f /proc/cpuinfo || [ "$n" = 0 ] || { echo "AVX-512 code in $so on a CPU without AVX-512"; exit 1; }
done
.venv/bin/python -m pytest -q tests -x            # expect: all pass (480 on the laptop)

step "1b. simulation venv (.venv_mujoco)"
python3.10 -m venv --clear .venv_mujoco
.venv_mujoco/bin/pip install -r requirements_mujoco.txt

step "2. baseline toolchain prefix (no root): extracted .debs + cmeel wheels"
for d in third_party/baseline_prefix/debs/*.deb; do dpkg -x "$d" third_party/baseline_prefix; done
python3.10 -m venv --clear .venv_baselines
.venv_baselines/bin/pip install -r requirements_baselines.txt
CMEEL="$ROOT/.venv_baselines/lib/python3.10/site-packages/cmeel.prefix"
PREFIX="$ROOT/third_party/baseline_prefix/usr"

step "3. fcl, OMPL 1.6, crocoddyl -> third_party/baseline_prefix/usr"
cmake -S third_party/fcl -B build/fcl -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX="$PREFIX" \
  -DCMAKE_PREFIX_PATH="$CMEEL;$PREFIX" -DBUILD_TESTING=OFF -DFCL_BUILD_EXAMPLES=OFF -Doctomap_DIR="$CMEEL/share/octomap"
cmake --build build/fcl -j "$JOBS"
cmake --install build/fcl
cmake -S third_party/ompl_1_6 -B build/ompl_1_6 -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX="$PREFIX" \
  -DBoost_NO_BOOST_CMAKE=ON -DBoost_NO_SYSTEM_PATHS=ON -DOMPL_BUILD_DEMOS=OFF -DOMPL_BUILD_PYBINDINGS=OFF \
  -DOMPL_BUILD_PYTESTS=OFF -DOMPL_BUILD_TESTS=OFF -DOMPL_REGISTRATION=OFF -DCMAKE_PREFIX_PATH="$CMEEL;$PREFIX" \
  -DCMAKE_DISABLE_FIND_PACKAGE_ODE=ON   # as on the laptop (ODE_FOUND empty there)
cmake --build build/ompl_1_6 -j "$JOBS"
cmake --install build/ompl_1_6
cmake -S third_party/crocoddyl -B build/crocoddyl -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX="$PREFIX" \
  -DCMAKE_PREFIX_PATH="$CMEEL;$PREFIX" -DBUILD_BENCHMARK=OFF -DBUILD_EXAMPLES=OFF -DBUILD_PYTHON_INTERFACE=OFF \
  -DBUILD_TESTING=OFF -DBUILD_WITH_CODEGEN_SUPPORT=OFF -DBUILD_WITH_MULTITHREADS=OFF -DENABLE_VECTORIZATION=OFF \
  -DPYTHON_EXECUTABLE="$ROOT/.venv_baselines/bin/python" -DPython3_EXECUTABLE="$ROOT/.venv_baselines/bin/python"   # eigenpy config needs an interpreter with NumPy
cmake --build build/crocoddyl -j "$JOBS"
cmake --install build/crocoddyl

step "4. Dynoplan/DynoBench (SST, iDb-A*, primitive generator) -> build/baselines"
# pinocchio's cmeel release targets use PACKAGE_PREFIX_DIR, which urdfdom_headers' config (from the extracted .debs)
# overwrites to $PREFIX during find_dependency; expose the same cmeel files under $PREFIX so the import checks pass
# (found on the PC, 2026-11-25; the linked libraries are unchanged).
for f in "$CMEEL"/lib/libpinocchio*.so*; do ln -sf "$f" "$PREFIX/lib/$(basename "$f")"; done
mkdir -p "$PREFIX/lib/python3.10/site-packages"
ln -sfn "$CMEEL/lib/python3.10/site-packages/pinocchio" "$PREFIX/lib/python3.10/site-packages/pinocchio"
# octomap_DIR: the same octomap (1.10, cmeel) as fcl; also loading the .deb 1.9 aborted every tool at exit (PC, 2026-11-25).
cmake -S third_party/dynoplan -B build/baselines -DCMAKE_BUILD_TYPE=Release \
  -DCMAKE_PREFIX_PATH="$PREFIX;$CMEEL" -DCMAKE_LIBRARY_PATH="$PREFIX/lib/x86_64-linux-gnu" \
  -Dfcl_DIR="$PREFIX/lib/cmake/fcl" -Dcrocoddyl_DIR="$PREFIX/lib/cmake/crocoddyl" -Dompl_DIR="$PREFIX/share/ompl/cmake" \
  -Doctomap_DIR="$CMEEL/share/octomap" \
  -DCMAKE_EXE_LINKER_FLAGS="-L$PREFIX/lib -Wl,-rpath,$PREFIX/lib -Wl,-rpath,$CMEEL/lib -Wl,-rpath-link,$CMEEL/lib -Wl,-rpath-link,$PREFIX/lib/x86_64-linux-gnu" \
  -DBUILD_DYNOBENCH_TOOLS=ON -DBUILD_DYNOBENCH_TESTS=OFF -DBUILD_DYNOBENCH_PYBINDINGS=OFF \
  -DPYTHON_EXECUTABLE="$ROOT/.venv_baselines/bin/python" -DPython3_EXECUTABLE="$ROOT/.venv_baselines/bin/python"
cmake --build build/baselines -j "$JOBS" --target main_sst main_idbastar main_primitives
.venv/bin/python scripts/build_seeded_baselines.py          # RNG-seeded main_sst / main_idbastar objects
PYTHONPATH=. .venv/bin/python -m benchmarks.upstream_sst_campaign build   # geometry-fixed libdynobench + main_sst
PYTHONPATH=. .venv/bin/python -m benchmarks.idbastar_campaign build       # main_idbastar against the same library

step "5. authors' library (KavrakiLab/ompl-flask, pinned flat a81c3f09) + reference drivers"
cmake -S third_party/ompl_flask_author -B build/ompl_flask_author -DCMAKE_BUILD_TYPE=Release -DOMPL_BUILD_VAMP=ON \
  -DOMPL_REGISTRATION=OFF -DOMPL_BUILD_PYTHON_BINDINGS=OFF -DOMPL_BUILD_DEMOS=ON -DOMPL_BUILD_TESTS=ON \
  -DVAMP_BUILD_PYTHON_BINDINGS=OFF -DVAMP_PORTABLE_BUILD=OFF
cmake --build build/ompl_flask_author -j "$JOBS"
bash benchmarks/author_flask/build.sh                        # stock author RRTConnect
.venv/bin/python scripts/build_budgeted_author_reference.py  # + our ptc check (v7 reference)
.venv/bin/python scripts/build_author_fix_reference.py       # + the authors' own loop fix (v9 reference)

step "6. sanity"
[ -f config/machine.json ] || echo "WARNING: create config/machine.json (measurement_cpu) for this machine first"
.venv/bin/python -m benchmarks.measurement_env snapshot
echo "Then: python -m benchmarks.measurement_env preflight; hardware calibration with --output config/hardware_calibration_<machine>.json"
