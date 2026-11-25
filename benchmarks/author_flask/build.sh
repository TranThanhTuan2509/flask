#!/usr/bin/env bash
# Build the reference driver against the authors' OMPL fork build tree (build/ompl_flask_author).
# Flags mirror the fork's own demo_FlatManipulatorPlanning target (Release, -O3 -march=native -mavx2).
set -euo pipefail
ROOT="$(cd "$(dirname "$0")/../.." && pwd)"
SRC="$ROOT/third_party/ompl_flask_author"
BLD="$ROOT/build/ompl_flask_author"
OUT="$ROOT/build/author_flask"
mkdir -p "$OUT"
c++ -O3 -DNDEBUG -march=native -mavx2 -std=c++17 -W -Wall -Wextra -Wno-system-headers \
  -DBOOST_ALL_NO_LIB -DBOOST_MATH_NO_LONG_DOUBLE_MATH_FUNCTIONS -DBOOST_PROGRAM_OPTIONS_DYN_LINK -DBOOST_SERIALIZATION_DYN_LINK \
  -I"$SRC/src" -I"$BLD/src" -I"$SRC/external/vamp/src/impl" -I"$BLD/_deps/simdxorshift-src/include" \
  -isystem /usr/include/eigen3 -isystem "$BLD/_deps/nigh-src/src" -isystem "$BLD/_deps/pdqsort-src" \
  "$ROOT/benchmarks/author_flask/mbm_flat.cpp" -o "$OUT/mbm_flat" \
  -Wl,-rpath,"$BLD/src/ompl" "$BLD/src/ompl/libompl.so.2.0.2" \
  /usr/lib/x86_64-linux-gnu/libboost_serialization.so.1.74.0 "$BLD/external/vamp/libsimdxorshift.a"
echo "built $OUT/mbm_flat"
