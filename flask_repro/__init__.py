"""  init  : FLASK implementation and evaluation utilities."""
import sys

try:
    from . import _core
    _core_error = None
except ImportError as error:              # pure-Python parts (flat BVP, models, oracles) still import without it
    _core = None
    _core_error = error


def require_native():
    """The native extension, or an ImportError that says why it cannot be loaded and how to fix it."""
    if _core is None:
        raise ImportError(
            f'the FLASK native extension (flask_repro/_core*.so) cannot be loaded by this interpreter '
            f'(Python {sys.version.split()[0]}, {sys.executable}): {_core_error}. It is compiled for the Python of '
            f'./install.sh: activate that environment (source .venv/bin/activate) or rebuild with ./build.sh.') from _core_error
    return _core
