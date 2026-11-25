"""Export the public repository (the folder pushed to GitHub) from this development workspace.

Copies the git-tracked and non-ignored files, minus development-only material (internal notes, reference images
from other papers, superseded launch scripts), into ../flask_re. An existing ../flask_re/.git is preserved, so
re-exporting updates a working tree that can be committed and pushed.

    python scripts/export_release.py [--dest ../flask_re]
"""
import argparse
import shutil
import subprocess
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
EXCLUDE_PREFIXES = (
    'docs/',                    # internal development notes (kept: docs/figures/, see below)
    'assets/references/',       # screenshots from other papers (third-party copyright)
    'assets/scene_sources/', 'assets/scenarios/', 'assets/showcase/',
    'package.sh', 'requirements_research.txt',
    'benchmarks/generate_tables.py', 'benchmarks/verify_migration.py', 'scripts/flask_deep_dive_checks.py',   # internal-doc tools
    'results/paper_campaigns/launch_v8.sh', 'results/paper_campaigns/launch_v9.sh',
    'results/paper_campaigns/launch_idbastar_v9.sh',
)
KEEP_PREFIXES = ('docs/figures/',)


def files():
    out = subprocess.run(['git', 'ls-files', '-co', '--exclude-standard', '-z'], cwd=ROOT, capture_output=True, check=True).stdout
    for f in out.decode().split('\0'):
        if not f or not (ROOT / f).is_file():
            continue
        if f.startswith(KEEP_PREFIXES) or not f.startswith(EXCLUDE_PREFIXES):
            yield f


def main():
    a = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    a.add_argument('--dest', type=Path, default=ROOT.parent / 'flask_re')
    dest = a.parse_args().dest.resolve()
    if dest == ROOT or ROOT in dest.parents:
        raise SystemExit('destination must be outside the workspace')
    dest.mkdir(parents=True, exist_ok=True)
    for p in dest.iterdir():                          # clean everything except the release's own git history
        if p.name != '.git':
            shutil.rmtree(p) if p.is_dir() else p.unlink()
    n = size = 0
    for f in files():
        target = dest / f; target.parent.mkdir(parents=True, exist_ok=True)
        shutil.copy2(ROOT / f, target); n += 1; size += target.stat().st_size
    print(f'exported {n} files, {size / 1e6:.1f} MB -> {dest}')


if __name__ == '__main__':
    main()
