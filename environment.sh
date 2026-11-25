# Activate the isolated planning environment.
_PROJECT_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
unset PYTHONPATH
export PYTHONNOUSERSITE=1
source "$_PROJECT_ROOT/.venv/bin/activate"
export FLASK_REPRO_ROOT="$_PROJECT_ROOT"
