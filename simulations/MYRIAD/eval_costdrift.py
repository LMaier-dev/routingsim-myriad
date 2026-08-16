#!/usr/bin/env python3
"""Path cost-drift: MyriadGP (production, rankUseSpof=true) vs oracle (LS/Dijkstra).

GP's spof-first ranking deliberately picks costlier but more-redundant paths.
This measures by how much — the "redundancy premium" per (node, origin) route.
Any negative drift (GP cheaper than optimal) is a bug flag.

    python3 simulations/MYRIAD/eval_costdrift.py

Outputs histogram.png + scatter.png under simulations/MYRIAD/eval/costdrift/.
"""
import os
import subprocess
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
SIM_ROOT = os.path.dirname(HERE)
REPO = os.path.dirname(SIM_ROOT)
BIN = os.path.join(REPO, "src", "RoutingSim")
GP_INI = os.path.join(HERE, "myriadgp.ini")
LS_INI = os.path.join(HERE, "myriadls.ini")
COMMON_INI = os.path.join(HERE, "eval_common.ini")
OUT = os.path.join(HERE, "eval", "costdrift")

CONFIG = "Grid10"
TOPO = "grid_10,10.txt"
SNAP = "5s"
SIMTIME = "80s"   # two refresh cycles (Grid10 refresh=15s); enough to converge


def parse_snapshots(path):
    """Timestamped dump -> {t(float): {(node, 'origin=r'): (cost, length)}}.
    length = GP's `len` (chain ring count) or the oracle's `hops` (min-cost path
    ring count) — the same quantity from either side."""
    snaps = {}
    for line in open(path):
        line = line.strip()
        if not line.startswith("t="):
            continue
        t_tok, rest = line.split(" ", 1)
        toks = rest.split()
        node = toks[0]
        kv = dict(p.split("=", 1) for p in toks[1:]
                  if "=" in p and not p.startswith("chain="))
        length = int(kv.get("len", kv.get("hops", 0)))
        snaps.setdefault(float(t_tok[2:]), {})[(node, f"origin={kv['origin']}")] = (
            int(kv["cost"]), length)
    return snaps


def _run(ini, config, dumpfile, resdir, extra):
    if os.path.exists(dumpfile):
        os.remove(dumpfile)
    dump_param = f'--**.{"ribDumpFile" if ini == GP_INI else "oracleDumpFile"}="{dumpfile}"'
    cmd = [BIN, "-u", "Cmdenv", ini, COMMON_INI, "-c", config,
           f"--sim-time-limit={SIMTIME}", f"--**.snapshotInterval={SNAP}",
           *extra, dump_param, "--result-dir", resdir]
    r = subprocess.run(cmd, cwd=SIM_ROOT, capture_output=True, text=True)
    log = r.stdout + r.stderr
    if r.returncode != 0 or "budget expired" in log or "ring broken" in log:
        sys.exit(f"FAIL: {os.path.basename(ini)} run errored\n{log[-1200:]}")
    if not os.path.exists(dumpfile):
        sys.exit(f"FAIL: no dump produced ({dumpfile})")
    snaps = parse_snapshots(dumpfile)
    if not snaps:
        sys.exit(f"FAIL: dump is empty ({dumpfile})")
    return snaps[max(snaps)]


def main():
    os.makedirs(OUT, exist_ok=True)
    if not os.path.exists(BIN):
        sys.exit(f"RoutingSim binary not found at {BIN} — build first.")

    r = subprocess.run([sys.executable, os.path.join(HERE, "topo2rings.py"),
                        os.path.join(SIM_ROOT, "topologies", TOPO),
                        "--out-dir", HERE, "--no-png"], capture_output=True, text=True)
    if r.returncode != 0:
        sys.exit(f"topo2rings.py failed:\n{r.stdout}{r.stderr}")

    print("Running LS (oracle truth)…")
    oracle = _run(LS_INI, CONFIG,
                  os.path.join(OUT, "drift_ls.txt"),
                  os.path.join(OUT, "res_ls"), [])

    print("Running GP production (rankUseSpof=true, default)…")
    gp = _run(GP_INI, CONFIG,
              os.path.join(OUT, "drift_gp.txt"),
              os.path.join(OUT, "res_gp"), [])

    # --- compute drift (cost and chain length, both GP − oracle) ---
    common = [k for k in oracle if k in gp]
    cost_drift = {k: gp[k][0] - oracle[k][0] for k in common}
    len_drift = {k: gp[k][1] - oracle[k][1] for k in common}
    missing = [k for k in oracle if k not in gp]
    negative = {k: v for k, v in cost_drift.items() if v < 0}
    cvals = list(cost_drift.values())
    lvals = list(len_drift.values())

    print(f"\n=== Cost-drift: GP(production) − oracle  [{CONFIG}] ===")
    print(f"  routes in oracle : {len(oracle)}")
    print(f"  routes in GP     : {len(gp)}")
    print(f"  missing in GP    : {len(missing)}"
          + (" (BUG)" if missing else ""))
    print(f"  negative drift   : {len(negative)}"
          + (" (BUG)" if negative else ""))
    if cvals:
        print(f"  cost   min/mean/max: "
              f"{min(cvals)} / {sum(cvals)/len(cvals):.1f} / {max(cvals)}")
        print(f"  length min/mean/max: "
              f"{min(lvals)} / {sum(lvals)/len(lvals):.1f} / {max(lvals)}")
        zero_frac = sum(1 for v in cvals if v == 0) / len(cvals) * 100
        print(f"  routes at exact optimum: {zero_frac:.1f}%")

    try:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
    except ImportError:
        print("\n(matplotlib not available — skipping plots)")
        return

    # histogram: x-axis = cost-drift bucket. Both drifts share one y-axis (drift
    # magnitude): the cost premium and the mean chain-length premium of those
    # routes, side by side. Route count per bucket annotated above.
    from collections import defaultdict
    by_cost = defaultdict(list)
    for k in common:
        by_cost[cost_drift[k]].append(len_drift[k])
    cxs = sorted(by_cost)
    pos = list(range(len(cxs)))                     # categorical: even spacing
    counts = [len(by_cost[c]) for c in cxs]
    mean_len = [sum(by_cost[c]) / len(by_cost[c]) for c in cxs]

    fig, ax = plt.subplots()
    ax.bar([p - 0.2 for p in pos], cxs, width=0.4, color="C0",
           label="cost drift (Σ members²)")
    ax.bar([p + 0.2 for p in pos], mean_len, width=0.4, color="C1",
           label="mean chain-length drift (hops)")
    for p, n in zip(pos, counts):                   # keep the route-count info
        ax.text(p, 0, f"n={n}", ha="center", va="bottom", fontsize=6, rotation=90)
    ax.set_xticks(pos)
    ax.set_xticklabels([str(c) for c in cxs], rotation=45, ha="right", fontsize=7)
    ax.set(xlabel="cost-drift bucket  GP − oracle", ylabel="drift (GP − oracle)",
           title=f"Cost vs length drift: GP production vs oracle  ({CONFIG})")
    ax.legend(fontsize=8)
    if negative:
        ax.text(0.02, 0.95, f"BUG: {len(negative)} negative cost drift",
                transform=ax.transAxes, color="C3", va="top", fontsize=9)
    fig.tight_layout()
    fig.savefig(os.path.join(OUT, "histogram.png"), dpi=120)

    # scatter: oracle_cost (x) vs GP_cost (y)
    ox = [oracle[k][0] for k in common]
    gy = [gp[k][0] for k in common]
    lo, hi = min(ox + gy), max(ox + gy)

    fig, ax = plt.subplots()
    ax.scatter(ox, gy, s=4, alpha=0.4, color="C0")
    ax.plot([lo, hi], [lo, hi], "k--", lw=0.8, label="y = x  (optimal)")
    ax.set(xlabel="oracle cost (Dijkstra)", ylabel="GP cost (production)",
           title=f"GP vs oracle path cost  ({CONFIG})")
    ax.legend(fontsize=8)
    fig.tight_layout()
    fig.savefig(os.path.join(OUT, "scatter.png"), dpi=120)

    print(f"\nplots -> {OUT}/{{histogram,scatter}}.png")


if __name__ == "__main__":
    main()
