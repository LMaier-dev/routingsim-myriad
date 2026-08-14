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
    """Timestamped dump -> {t(float): {(node, 'origin=r'): cost}}."""
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
        snaps.setdefault(float(t_tok[2:]), {})[(node, f"origin={kv['origin']}")] = int(kv["cost"])
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

    # --- compute drift ---
    drifts = {k: gp[k] - oracle[k] for k in oracle if k in gp}
    missing = [k for k in oracle if k not in gp]
    negative = {k: v for k, v in drifts.items() if v < 0}
    vals = list(drifts.values())

    print(f"\n=== Cost-drift: GP(production) − oracle  [{CONFIG}] ===")
    print(f"  routes in oracle : {len(oracle)}")
    print(f"  routes in GP     : {len(gp)}")
    print(f"  missing in GP    : {len(missing)}"
          + (" (BUG)" if missing else ""))
    print(f"  negative drift   : {len(negative)}"
          + (" (BUG)" if negative else ""))
    if vals:
        print(f"  drift  min/mean/max: "
              f"{min(vals)} / {sum(vals)/len(vals):.1f} / {max(vals)}")
        zero_frac = sum(1 for v in vals if v == 0) / len(vals) * 100
        print(f"  routes at exact optimum: {zero_frac:.1f}%")

    try:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
    except ImportError:
        print("\n(matplotlib not available — skipping plots)")
        return

    # histogram of drift values
    from collections import Counter
    counts = Counter(vals)
    xs = sorted(counts)
    ys = [counts[x] for x in xs]
    colors = ["C3" if x < 0 else "C0" for x in xs]

    fig, ax = plt.subplots()
    ax.bar(xs, ys, color=colors, width=max(1, max(xs) // 60) if xs else 1)
    ax.axvline(0, ls="--", color="k", lw=0.8, alpha=0.6)
    ax.set(xlabel="GP cost − oracle cost  (redundancy premium)",
           ylabel="route count",
           title=f"Cost drift: GP production vs oracle  ({CONFIG})")
    if negative:
        ax.text(0.02, 0.95, f"BUG: {len(negative)} negative",
                transform=ax.transAxes, color="C3", va="top", fontsize=9)
    fig.tight_layout()
    fig.savefig(os.path.join(OUT, "histogram.png"), dpi=120)

    # scatter: oracle_cost (x) vs GP_cost (y)
    ox = [oracle[k] for k in drifts]
    gy = [gp[k] for k in drifts]
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
