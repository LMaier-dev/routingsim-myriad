#!/usr/bin/env python3
"""Path cost-drift: MyriadGP (production, rankUseSpof=true) vs oracle (LS/Dijkstra).

PV's spof-first ranking deliberately picks costlier but more-redundant paths.
This measures by how much — the "redundancy premium" per (node, origin) route.
Any negative drift (PV cheaper than optimal) is a bug flag.

    python3 simulations/MYRIAD/eval_costdrift.py [--topo PATH] [--basis {min,tree}]

Outputs scatter.png under simulations/MYRIAD/eval/costdrift/<topo>[_<tag>]/.
"""
import argparse
import os
import sys

from eval_harness import (BIN, SIM_ROOT, MYRIAD_DIR, announce, apply_repeat_args,
                          ensure_ring_labels, overrides, pm, rep_path, repeat_mean,
                          repeats_note, run_sim, simtime, title, write_proto_ini,
                          PROTO_COLOR,
                          add_topo_args)

HERE = str(MYRIAD_DIR)
OUT = os.path.join(HERE, "eval", "costdrift")

# One refresh cycle: PV converges to the ideal cost within the first flood.
SIMTIME = simtime(cycles=1)
SNAPSHOT = "30s"


def parse_snapshots(path):
    """Timestamped dump -> {t(float): {(node, 'origin=r'): (cost, length, spof)}}.
    length = PV's `len` (chain ring count) or the oracle's `hops` (min-cost path
    ring count) — the same quantity from either side. spof = single-router ring
    transitions on the chain (PV only; absent in the oracle dump -> 0)."""
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
            int(kv["cost"]), length, int(kv.get("spof", 0)))
    return snaps


def _run(ini, dump_param, dumpfile, resdir, rep=0, extra=()):
    dumpfile, resdir = rep_path(dumpfile, rep), rep_path(resdir, rep)
    if os.path.exists(dumpfile):
        os.remove(dumpfile)
    cmd = [BIN, "-u", "Cmdenv", ini, *overrides(sim_time_limit=SIMTIME), *extra,
           f'--**.{dump_param}="{dumpfile}"', "--result-dir", resdir]
    run_sim(cmd, rep=rep)
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
    # Coarse snapshots unless the caller asked for a rate (see SNAPSHOT).
    if not getattr(args, "snapshot", None):
        args.snapshot = SNAPSHOT
    apply_repeat_args(args)

    # Per topology (and --tag), like dynamics: OUT was a bare constant, so two
    # costdrift runs silently overwrote each other's plots -- and --tag was
    # accepted but had nothing to act on.
    global OUT
    OUT = os.path.join(OUT, os.path.splitext(os.path.basename(args.topo))[0]
                       + (f"_{args.tag}" if getattr(args, "tag", None) else ""))
    os.makedirs(OUT, exist_ok=True)
    if not os.path.exists(BIN):
        sys.exit(f"RoutingSim binary not found at {BIN} — build first.")

    labels = ensure_ring_labels(args.topo, args.basis,
                                getattr(args, 'max_ring_size', 0),
                                not getattr(args, 'no_echo_rings', False))
    want_nospof = getattr(args, "nospof", False)
    pv_extra = (["**.refreshFloodsUpdate = false"]
                if getattr(args, "no_refresh_flood", False) else None)
    pv_ini = write_proto_ini(os.path.join(OUT, "pv.ini"), "pv", args.topo, labels,
                             extra_lines=pv_extra,
                             update_jitter=args.update_jitter)
    ls_ini = write_proto_ini(os.path.join(OUT, "ls.ini"), "ls", args.topo, labels)
    announce("costdrift", opts=overrides(sim_time_limit=SIMTIME))

    # One repeat = the LS+PV pair on the same seed (the drift is a per-run
    # comparison, so both sides must come from the same repeat). Rep 0's raw
    # drift is kept for the plots — the per-route dicts aren't averagable.
    raw = {}

    def one(rep):
        if rep == 0:
            print("  → LS oracle …", flush=True)
        oracle = _run(ls_ini, "oracleDumpFile",
                      os.path.join(OUT, "drift_ls.txt"),
                      os.path.join(OUT, "res_ls"), rep)
        if rep == 0:
            print(f"  ✓ LS done ({len(oracle)} entries)")
            print("  → PV production (rankUseSpof=true) …", flush=True)
        pv = _run(pv_ini, "ribDumpFile",
                  os.path.join(OUT, "drift_gp.txt"),
                  os.path.join(OUT, "res_gp"), rep)
        if rep == 0:
            print(f"  ✓ PV done ({len(pv)} entries)")
            if want_nospof:
                print("  → PV no-spof (rankUseSpof=false) …", flush=True)
        # Pure-cost ranking at production margin 0.2: with the selectPreferred
        # spof-first bug fixed, cost-only ranking lands on the oracle optimum, so
        # its scatter cloud sits on y=x and isolates the spof premium from any real
        # PV suboptimality.
        nospof = _run(pv_ini, "ribDumpFile",
                      os.path.join(OUT, "drift_gp_nospof.txt"),
                      os.path.join(OUT, "res_gp_nospof"), rep,
                      extra=["--**.rankUseSpof=false",
                             "--**.ecmpCostMargin=0.2"]) if want_nospof else {}
        if rep == 0 and want_nospof:
            print(f"  ✓ PV no-spof done ({len(nospof)} entries)")

        # --- compute drift (cost and chain length, both PV − oracle) ---
        common = [k for k in oracle if k in pv]
        cost_drift = {k: pv[k][0] - oracle[k][0] for k in common}
        len_drift = {k: pv[k][1] - oracle[k][1] for k in common}
        missing = [k for k in oracle if k not in pv]
        negative = {k: v for k, v in cost_drift.items() if v < 0}
        cvals, lvals = list(cost_drift.values()), list(len_drift.values())
        ns_common = [k for k in oracle if k in nospof]
        ns_drift = {k: nospof[k][0] - oracle[k][0] for k in ns_common}
        nsvals = list(ns_drift.values())
        if rep == 0:
            raw.update(oracle=oracle, pv=pv, common=common,
                       nospof=nospof, ns_common=ns_common)
        # Chains crossing a single-router ring transition: 0 on a 2-connected
        # topology (grid), non-zero means the RIB had no redundant alternative.
        spofs = [pv[k][2] for k in common]
        return {
            "n_oracle": len(oracle), "n_pv": len(pv),
            "missing": len(missing), "negative": len(negative),
            "spof_chains": sum(1 for s in spofs if s),
            "spof_frac": (sum(1 for s in spofs if s) / len(spofs) * 100) if spofs else 0.0,
            "spof_max": max(spofs) if spofs else 0,
            "ns_mean": (sum(nsvals) / len(nsvals)) if nsvals else 0.0,
            "ns_zero_frac": (sum(1 for v in nsvals if v == 0) / len(nsvals) * 100) if nsvals else 0.0,
            "cost": (min(cvals), sum(cvals) / len(cvals), max(cvals)) if cvals else None,
            "length": (min(lvals), sum(lvals) / len(lvals), max(lvals)) if lvals else None,
            "zero_frac": (sum(1 for v in cvals if v == 0) / len(cvals) * 100) if cvals else 0.0,
        }

    r = repeat_mean(one)
    oracle, pv, common = raw["oracle"], raw["pv"], raw["common"]

    print(f"\n=== Cost-drift: PV(production) − oracle{repeats_note()} ===")
    print(f"  routes in oracle : {r['n_oracle']:.0f}{pm(r['n_oracle'], '.1f')}")
    print(f"  routes in PV     : {r['n_pv']:.0f}{pm(r['n_pv'], '.1f')}")
    print(f"  missing in PV    : {r['missing']:.1f}" + (" (BUG)" if r["missing"] else ""))
    print(f"  negative drift   : {r['negative']:.1f}" + (" (BUG)" if r["negative"] else ""))
    if r["cost"]:
        lo, mean, hi = r["cost"]
        print(f"  cost   min/mean/max: {lo:.1f} / {mean:.1f}{pm(mean)} / {hi:.1f}")
        lo, mean, hi = r["length"]
        print(f"  length min/mean/max: {lo:.1f} / {mean:.1f}{pm(mean)} / {hi:.1f}")
        print(f"  routes at exact optimum: {r['zero_frac']:.1f}"
              f"{pm(r['zero_frac'])}%")
    if want_nospof:
        print(f"  no-spof mean drift  : {r['ns_mean']:.1f}{pm(r['ns_mean'])} "
              f"({r['ns_zero_frac']:.1f}% at optimum)")
    print(f"  chains with spof != 0: {r['spof_chains']:.1f}"
          f"{pm(r['spof_chains'], '.1f')} ({r['spof_frac']:.1f}%, "
          f"max spof/chain {r['spof_max']:.0f})")

    try:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
    except ImportError:
        print("\n(matplotlib not available — skipping the plot)")
        return

    nospof, ns_common = raw["nospof"], raw["ns_common"]
    ox = [oracle[k][0] for k in common]
    gy = [pv[k][0] for k in common]
    nx_ = [oracle[k][0] for k in ns_common]
    ny = [nospof[k][0] for k in ns_common]
    lo, hi = min(ox + gy + ny), max(ox + gy + ny)

    fig, ax = plt.subplots()
    ax.scatter(ox, gy, s=4, alpha=0.4, color=PROTO_COLOR["pv"],
               label="Path-Vector (PV), spof-first")
    if ns_common:
        ax.scatter(nx_, ny, s=4, alpha=0.4, color=PROTO_COLOR["pv_nospof"],
                   label="Path-Vector (spof-blind)")
    ax.plot([lo, hi], [lo, hi], "k--", lw=0.8, label="y = x  (optimal)")
    ax.set(xlabel="Link-State (LS), cost (Dijkstra)",
           ylabel="Path-Vector (PV) cost",
           title=title("Path-Vector vs Link-State path cost"))
    ax.legend(fontsize=8)
    fig.tight_layout()
    fig.savefig(os.path.join(OUT, "scatter.png"), dpi=120)

    print(f"\nplot -> {OUT}/scatter.png")


if __name__ == "__main__":
    main()
