New benchmark results after migrating the pyhpp problem configuration to
setter methods.

All 13 saved benchmarks completed 20 rounds with 100% success. In particular,
pyrene-on-the-ground now completes, unlike in 2026/05-15.

During the first ur3-spheres pass, the planner reached its 5000-iteration limit
on attempt 15. Its exception handler stopped the remaining attempts, unlike the
other benchmarks. The break was removed and the complete replacement run saved
here completed 20/20.

The Romeo results are not directly comparable with 2026/05-15: the current
future scripts use the default path validation instead of an explicit
Dichotomy validator.
