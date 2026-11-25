# Pinned DynoBench subset

Source: https://github.com/quimortiz/dynobench

Commit: `9d41a6207169d6c55578fd1451bb2e7bf7c7ec48` (also in `COMMIT`).
Files retain their upstream contents and MIT license. This is a small data
and dynamics-source subset, not a build or installation of DynoBench/iDb-A*.

The FLASK paper does not identify the exact dataset revision or its sphere
approximation. The mapping in `flask_repro/dynobench.py` is explicitly a set
of candidate inputs, not a claim of exact experimental correspondence.
Robot parameters/constraints used by the new reference implementation come
from FLASK Sec. VII-A, with deviations listed in each run's manifest.

Quad2D's upstream controls are normalized by nominal motor thrust; the
reference returns motor thrust in Newtons. Quad3D's upstream implementation
offers motor/control modes; it must not be used as a paper baseline without
matching FLASK's specified constraints and units.
