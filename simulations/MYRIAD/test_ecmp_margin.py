#!/usr/bin/env python3
"""ecmpCostMargin must widen co-best sets, never shrink them. Margin 0 is
covered by test_rib_oracle.py (it would fail if 0 changed behaviour)."""
import os, re, sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
from eval_harness import BIN, run_sim, overrides, warmup_s  # noqa: E402

OUT = os.path.join(HERE, "out")


def routes(margin):
    dump = os.path.join(OUT, f"ecmp_m{margin}.rib")
    if os.path.exists(dump):
        os.remove(dump)
    run_sim([BIN, "-u", "Cmdenv",
             os.path.join(HERE, "myriadgp.ini"), os.path.join(HERE, "eval_common.ini"),
             "-c", "Grid10",
             *overrides(sim_time_limit=f"{warmup_s() + 150:g}s", snapshotInterval="0s"),
             f"--**.ecmpCostMargin={margin}",
             f'--**.ribDumpFile="{dump}"',
             "--result-dir", os.path.join(OUT, f"ecmp_m{margin}")],
            allow_budget_expired=True)
    # Each dump line is one (node, origin) with its co-best width in ecmp=N.
    with open(dump) as f:
        return [int(m.group(1)) for m in
                (re.search(r"ecmp=(\d+)", l) for l in f) if m]


base = routes(0)
assert base, "margin 0 produced no routes"
wide = routes(0.2)
b, w = sum(base) / len(base), sum(wide) / len(wide)
assert w >= b, f"margin 0.2 narrowed ECMP: {w:.2f} < {b:.2f}"
print(f"OK  mean ECMP width  margin 0: {b:.2f}   margin 0.2: {w:.2f}")
