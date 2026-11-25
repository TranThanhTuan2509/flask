#!/usr/bin/env bash
# v2 measurement campaign (timing-critical part). Sequential, one logical CPU (3), machine otherwise idle.
# Do not modify any file covered by flask_repro.provenance.manifest while this runs.
set -uo pipefail
cd "$(dirname "$0")/.."
OUT="${1:-results/paper_campaigns/2026-11-23_native_v2}"
CPU="${CPU:-3}"
PY=".venv/bin/python"
export OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 MKL_NUM_THREADS=1 NUMEXPR_NUM_THREADS=1
mkdir -p "$OUT"
LOG="$OUT/campaign.log"
run() { local name="$1"; shift; echo "START $name $(date -Is)" | tee -a "$LOG"; "$@" > "$OUT/$name.log" 2>&1; local rc=$?; echo "END $name rc=$rc $(date -Is)" | tee -a "$LOG"; }
run hardware_before $PY -m benchmarks.hardware_calibration --output "$OUT/hardware_calibration_before.json" --cpu "$CPU"
run table_ii_panda_v2_res1 $PY -m benchmarks.manipulator_campaign --output "$OUT/table_ii_panda_v2_res1" --profile config/manipulator_measurement_v2.json --cpu "$CPU"
run table_ii_panda_v2_res4 $PY -m benchmarks.manipulator_campaign --output "$OUT/table_ii_panda_v2_res4" --profile config/manipulator_measurement_v2_res4.json --cpu "$CPU" --methods flask_rrt_connect
run table_ii_panda_v2_res16 $PY -m benchmarks.manipulator_campaign --output "$OUT/table_ii_panda_v2_res16" --profile config/manipulator_measurement_v2_res16.json --cpu "$CPU" --methods flask_rrt_connect
run table_ii_panda_full_velocity $PY -m benchmarks.manipulator_campaign --output "$OUT/table_ii_panda_full_velocity" --profile config/manipulator_sensitivity_full_velocity_v2.json --cpu "$CPU" --methods flask_rrt_connect
run table_ii_author $PY -m benchmarks.author_reference run --output "$OUT/table_ii_author" --seeds 1 2 3 4 5 6 7 8 9 10 --budget 1.0 --accel 0 --cpu "$CPU"
run table_i_low_dof_v2 $PY -m benchmarks.native_campaign --output "$OUT/table_i_low_dof_v2" --profile config/low_dof_measurement_v2.json --cpu "$CPU" --campaign-class paper_case_reconstruction_measurement
run table_i_order4_sensitivity $PY -m benchmarks.native_campaign --output "$OUT/table_i_order4_sensitivity" --profile config/low_dof_sensitivity_order4_v2.json --cpu "$CPU" --campaign-class paper_case_reconstruction_measurement --cases quad2d_bugtrap quad2d_hole quad3d_block quad3d_window --algorithms rrt_connect sst_bvp
run hardware_after $PY -m benchmarks.hardware_calibration --output "$OUT/hardware_calibration_after.json" --cpu "$CPU"
echo "CAMPAIGN DONE $(date -Is)" | tee -a "$LOG"
