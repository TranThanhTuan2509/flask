#!/usr/bin/env bash
# Fetch pinned third-party sources into third_party/.
#   scripts/fetch_third_party.sh            core: VAMP (+ local patch) and the MotionBenchMaker problem sets
#   scripts/fetch_third_party.sh --showcase core + MuJoCo showcase assets (Unitree G1 policy, Skydio X2 model)
#   scripts/fetch_third_party.sh --all      core + showcase + baselines/reference code (OMPL 1.6, FCL, crocoddyl,
#                                           Dynoplan, the authors' ompl-flask, TurtleBot3)
set -euo pipefail
ROOT="$(cd "$(dirname "$0")/.." && pwd)"
cd "$ROOT/third_party"

fetch() {  # name url commit [--recursive]
  local name=$1 url=$2 commit=$3
  if [ -d "$name/.git" ] || [ -f "$name/.fetched" ]; then echo "[skip] $name"; return; fi
  git clone -q "$url" "$name"
  git -C "$name" checkout -q "$commit"
  [ "${4:-}" = "--recursive" ] && git -C "$name" submodule update --init --recursive -q
  echo "[ok]   $name @ $commit"
}

# --- core ------------------------------------------------------------------------------------------------------
fetch vamp https://github.com/KavrakiLab/vamp.git "$(cat VAMP_COMMIT)" --recursive
if [ ! -f vamp/.flask_patch_applied ]; then
  git -C vamp apply "$ROOT/third_party/vamp_capture_validation.patch"
  touch vamp/.flask_patch_applied
  echo "[ok]   vamp patch (vamp_capture_validation.patch)"
fi

# --- optional: MuJoCo showcase assets --------------------------------------------------------------------------------
if [ "${1:-}" = "--showcase" ] || [ "${1:-}" = "--all" ]; then
  fetch unitree_rl_gym https://github.com/unitreerobotics/unitree_rl_gym.git 276801e46c5d433564f24658bac64f254b7d2d4b
  if [ ! -d mujoco_menagerie/skydio_x2 ]; then           # sparse: only the models the showcases use
    git clone -q --filter=blob:none --sparse https://github.com/google-deepmind/mujoco_menagerie.git mujoco_menagerie
    git -C mujoco_menagerie sparse-checkout set skydio_x2 universal_robots_ur5e robotiq_2f85 franka_emika_panda
    git -C mujoco_menagerie checkout -q 370c43965323c0944b433b202aac5058b5f907a9
    echo "[ok]   mujoco_menagerie (sparse) @ 370c439"
  fi
fi

# --- optional: baselines and the authors' released code ------------------------------------------------------------
if [ "${1:-}" = "--all" ]; then
  fetch ompl_flask_author https://github.com/KavrakiLab/ompl-flask.git a81c3f09ab5ebce4a83a151e8170e5bdf19b94f9 --recursive
  fetch ompl_1_6          https://github.com/ompl/ompl.git e2994e580fcba0eb117e38bd1dc7439c4beb35ef
  fetch fcl               https://github.com/flexible-collision-library/fcl.git df2702ca5e703dec98ebd725782ce13862e87fc8
  fetch crocoddyl         https://github.com/loco-3d/crocoddyl.git 0500e398861564b6986d99206a1e0ccec0d66a33 --recursive
  fetch dynoplan          https://github.com/quimortiz/dynoplan.git dc938a2ac7d6a699781e0bd80370f15b74b8b0eb --recursive
  fetch turtlebot3        https://github.com/ROBOTIS-GIT/turtlebot3.git 0d4e59cf34da9c0661f80008bb0fa02636c713b2
fi
