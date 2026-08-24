#!/usr/bin/env python3
"""Path cost-drift: MyriadGP (production, rankUseSpof=true) vs oracle (LS/Dijkstra).

GP's spof-first ranking deliberately picks costlier but more-redundant paths.
This measures by how much — the "redundancy premium" per (node, origin) route.
Any negative drift (GP cheaper than optimal) is a bug flag.

    python3 simulations/MYRIAD/eval_costdrift.py [--topo PATH] [--basis {min,all}]

Outputs histogram.png + scatter.png under simulations/MYRIAD/eval/costdrift/.
"""
import argparse
import os
import sys

from eval_harness import (BIN, SIM_ROOT, MYRIAD_DIR,
                          ensure_ring_labels, run_sim, write_proto_ini, add_topo_args)

HERE = str(MYRIAD_DIR)
OUT = os.path.join(HERE, "eval", "costdrift")

SNAP = "5s"
SIMTIME = "80s"   # two refresh cycles at 15s; enough to heal transients

# Fast refresh so multiple cycles complete within SIMTIME.
_REFRESH_LINES = ["**.refreshInterval = 15s", "**.refreshJitter = 0s"]


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


def _run(ini, dump_param, dumpfile, resdir):
    if os.path.exists(dumpfile):
        os.remove(dumpfile)
    cmd = [BIN, "-u", "Cmdenv", ini,
           f"--sim-time-limit={SIMTIME}", f"--**.snapshotInterval={SNAP}",
           f'--**.{dump_param}="{dumpfile}"', "--result-dir", resdir]
    run_sim(cmd)
    if not os.path.exists(dumpfile):
        sys.exit(f"FAIL: no dump produced ({dumpfile})")
    snaps = parse_snapshots(dumpfile)
    if not snaps:
        sys.exit(f"FAIL: dump is empty ({dumpfile})")
    return snaps[max(snaps)]


def main(args=None):
    if args is None:
        p = argparse.ArgumentParser()
        add_topo_args(p)
        args = p.parse_args()

    os.makedirs(OUT, exist_ok=True)
    if not os.path.exists(BIN):
        sys.exit(f"RoutingSim binary not found at {BIN} — build first.")

    labels = ensure_ring_labels(args.topo, args.basis)
    gp_ini = write_proto_ini(os.path.join(OUT, "gp.ini"), "gp",
                             args.topo, labels, extra_lines=_REFRESH_LINES,
                             update_jitter=args.update_jitter, warmup=args.warmup)
    ls_ini = write_proto_ini(os.path.join(OUT, "ls.ini"), "ls",
                             args.topo, labels, extra_lines=_REFRESH_LINES)

    print("  → LS oracle …", flush=True)
    oracle = _run(ls_ini, "oracleDumpFile",
                  os.path.join(OUT, "drift_ls.txt"),
                  os.path.join(OUT, "res_ls"))
    print(f"  ✓ LS done ({len(oracle)} entries)")

    print("  → GP production (rankUseSpof=true) …", flush=True)
    gp = _run(gp_ini, "ribDumpFile",
              os.path.join(OUT, "drift_gp.txt"),
              os.path.join(OUT, "res_gp"))
    print(f"  ✓ GP done ({len(gp)} entries)")

    # --- compute drift (cost and chain length, both GP − oracle) ---
    common = [k for k in oracle if k in gp]
    cost_drift = {k: gp[k][0] - oracle[k][0] for k in common}
    len_drift = {k: gp[k][1] - oracle[k][1] for k in common}
    missing = [k for k in oracle if k not in gp]
    negative = {k: v for k, v in cost_drift.items() if v < 0}
    cvals = list(cost_drift.values())
    lvals = list(len_drift.values())

    print(f"\n=== Cost-drift: GP(production) − oracle ===")
    print(f"  routes in oracle : {len(oracle)}")
    print(f"  routes in GP     : {len(gp)}")
    print(f"  missing in GP    : {len(missing)}" + (" (BUG)" if missing else ""))
    print(f"  negative drift   : {len(negative)}" + (" (BUG)" if negative else ""))
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

    from collections import defaultdict
    by_cost = defaultdict(list)
    for k in common:
        by_cost[cost_drift[k]].append(len_drift[k])
    cxs = sorted(by_cost)
    pos = list(range(len(cxs)))
    counts = [len(by_cost[c]) for c in cxs]
    mean_len = [sum(by_cost[c]) / len(by_cost[c]) for c in cxs]

    fig, ax = plt.subplots()
    ax.bar([p - 0.2 for p in pos], cxs, width=0.4, color="C0",
           label="cost drift (Σ members²)")
    ax.bar([p + 0.2 for p in pos], mean_len, width=0.4, color="C1",
           label="mean chain-length drift (hops)")
    for p, n in zip(pos, counts):
        ax.text(p, 0, f"n={n}", ha="center", va="bottom", fontsize=6, rotation=90)
    ax.set_xticks(pos)
    ax.set_xticklabels([str(c) for c in cxs], rotation=45, ha="right", fontsize=7)
    ax.set(xlabel="cost-drift bucket  GP − oracle", ylabel="drift (GP − oracle)",
           title="Cost vs length drift: GP production vs oracle")
    ax.legend(fontsize=8)
    if negative:
        ax.text(0.02, 0.95, f"BUG: {len(negative)} negative cost drift",
                transform=ax.transAxes, color="C3", va="top", fontsize=9)
    fig.tight_layout()
    fig.savefig(os.path.join(OUT, "histogram.png"), dpi=120)

    ox = [oracle[k][0] for k in common]
    gy = [gp[k][0] for k in common]
    lo, hi = min(ox + gy), max(ox + gy)

    fig, ax = plt.subplots()
    ax.scatter(ox, gy, s=4, alpha=0.4, color="C0")
    ax.plot([lo, hi], [lo, hi], "k--", lw=0.8, label="y = x  (optimal)")
    ax.set(xlabel="oracle cost (Dijkstra)", ylabel="GP cost (production)",
           title="GP vs oracle path cost")
    ax.legend(fontsize=8)
    fig.tight_layout()
    fig.savefig(os.path.join(OUT, "scatter.png"), dpi=120)

    print(f"\nplots -> {OUT}/{{histogram,scatter}}.png")


if __name__ == "__main__":
    main()
