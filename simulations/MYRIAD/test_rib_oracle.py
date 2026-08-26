#!/usr/bin/env python3
"""Optimality gate for MyriadGP, checked against an independent Dijkstra oracle.

test_rib_batching.py proves batching preserves REACHABILITY. It cannot tell an
optimal RIB from a merely-connected one. This does: MyriadLS floods a full
link-state DB and (via RingSpfOracle) runs Dijkstra on the ring graph to compute
the OPTIMAL least-cost path from every node to every origin ring. MyriadGP,
running the same topology with rankUseSpof=false (pure cost ranking), must agree
on every (node, origin) cost. Two independently-converged protocols + two
different algorithms (path-vector vs. shortest-path) matching = both reached the
same optimum.

    python3 simulations/MYRIAD/test_rib_oracle.py

Exit 0 = GP is cost-optimal on every fixture.

Note: both fixtures here happen to be spof=0 everywhere (every ring transition is
double-bridged), so rankUseSpof=false is a no-op on them — GP is already
cost-ranked. The knob's discriminating power (declining a costlier redundant path
in favour of the least-cost one) only shows on a topology with a single-bridge
alternative, which we don't yet have a fixture for.
"""
import os
import sys

from eval_harness import (BIN, SIM_ROOT, add_topo_args, apply_repeat_args,
                          ensure_ring_labels, overrides, run_sim, write_proto_ini)

HERE = os.path.dirname(os.path.abspath(__file__))
GP_INI = os.path.join(HERE, "myriadgp.ini")
LS_INI = os.path.join(HERE, "myriadls.ini")
COMMON_INI = os.path.join(HERE, "eval_common.ini")
OUT = os.path.join(HERE, "eval", "oracle")

# name -> topology to regenerate ring labels from, or None if the ini carries
# inline labels.
CONFIGS = {
    "Grid10": "grid_10,10.txt",
    "PaperTopology": None,
}

# GP must run several re-collect (refresh) cycles before the snapshot. A single
# collect round leaves first-round size-propagation transients: a node can
# transition a chain out of a ring before that ring's member-count trailer
# reached it, freezing an UNKNOWN_RING_SIZE (or a slightly-long path) into the RIB
# because re-forwarding the same (chain, seq) is dup-guarded. A seq advance
# (refresh) re-propagates with all sizes known and heals it. The fast
# refreshInterval lives in myriadgp.ini's Grid10/PaperTopology configs (not here),
# so the run condition travels with the config; test_rib_batching disables refresh
# on the CLI because it only needs reachability, which holds mid-convergence.
# Final RIB only: snapshots off, or the t=-prefixed samples land in the dump.
CONV = overrides(sim_time_limit="120s", snapshotInterval="0s")

# For self-contained inis (arbitrary topo), replicate the fast-refresh that the
# named fixtures get via myriadgp.ini's [Config Grid10] section.
_REFRESH_LINES = ["**.refreshInterval = 15s", "**.refreshJitter = 0s"]


def _parse_gp(dump):
    out = {}
    with open(dump) as f:
        for l in f:
            if not l.strip():
                continue
            kv = dict(p.split("=", 1) for p in l.split()
                      if "=" in p and not p.startswith("chain="))
            out[(l.split()[0], f"origin={kv['origin']}")] = (
                int(kv["cost"]), int(kv["spof"]))
    return out


def _parse_oracle(dump):
    out = {}
    with open(dump) as f:
        for l in f:
            if not l.strip():
                continue
            kv = dict(p.split("=", 1) for p in l.split() if "=" in p)
            out[(l.split()[0], f"origin={kv['origin']}")] = int(kv["cost"])
    return out


def _compare(gp, oracle):
    """Print comparison results; return True if all checks pass."""
    print(f"  {len(gp)} GP entries vs {len(oracle)} oracle entries")
    missing = [k for k in oracle if k not in gp]
    SENTINEL = 10 ** 12
    unconverged = [k for k in oracle if k in gp and gp[k][0] > SENTINEL]
    mismatch = [(k, gp[k][0], oracle[k]) for k in oracle
                if k in gp and gp[k][0] <= SENTINEL and gp[k][0] != oracle[k]]
    max_spof = max((s for _, s in gp.values()), default=0)
    ok = True
    if missing:
        ok = False
        print(f"  FAIL: {len(missing)} (node,origin) reachable in oracle but MISSING in GP")
        for k in sorted(missing)[:8]:
            print(f"        node={k[0]} {k[1]}")
    if unconverged:
        ok = False
        print(f"  FAIL: {len(unconverged)} entries UNCONVERGED (UNKNOWN_RING_SIZE in preferred chain)")
        for k in sorted(unconverged)[:8]:
            print(f"        node={k[0]} {k[1]}")
    if mismatch:
        ok = False
        print(f"  FAIL: {len(mismatch)} cost MISMATCHES (GP converged but not least-cost)")
        for k, gc, oc in sorted(mismatch)[:8]:
            print(f"        node={k[0]} {k[1]} gp_cost={gc} optimal={oc} (+{gc - oc})")
    if ok:
        print(f"  OK: every oracle (node,origin) cost matched by GP "
              f"(max spof on chosen paths = {max_spof})")
    return ok


def _run_gp_ini(ini, dump, label):
    if os.path.exists(dump):
        os.remove(dump)
    print(f"  -> GP {label} …", flush=True)
    run_sim([BIN, "-u", "Cmdenv", ini, *CONV,
             "--**.rankUseSpof=false",
             f'--**.ribDumpFile="{dump}"',
             "--result-dir", os.path.join(OUT, f"gp_{label}")])
    if not os.path.exists(dump):
        sys.exit(f"FAIL: GP {label} produced no RIB dump")
    return _parse_gp(dump)


def _run_oracle_ini(ini, dump, label):
    if os.path.exists(dump):
        os.remove(dump)
    print(f"  -> LS oracle {label} …", flush=True)
    run_sim([BIN, "-u", "Cmdenv", ini, *CONV,
             f'--**.oracleDumpFile="{dump}"',
             "--result-dir", os.path.join(OUT, f"ls_{label}")])
    if not os.path.exists(dump):
        sys.exit(f"FAIL: LS {label} produced no oracle dump")
    return _parse_oracle(dump)


def run_gp(config, dump):
    """GP with pure-cost ranking; returns {(node, 'origin=r'): (cost, spof)}."""
    if os.path.exists(dump):
        os.remove(dump)
    print(f"  -> GP {config} …", flush=True)
    run_sim([BIN, "-u", "Cmdenv", GP_INI, COMMON_INI, "-c", config, *CONV,
             "--**.rankUseSpof=false",
             f'--**.ribDumpFile="{dump}"',
             "--result-dir", os.path.join(OUT, f"gp_{config}")])
    if not os.path.exists(dump):
        sys.exit(f"FAIL: GP {config} produced no RIB dump")
    return _parse_gp(dump)


def run_oracle(config, dump):
    """MyriadLS Dijkstra oracle; returns {(node, 'origin=r'): cost}."""
    if os.path.exists(dump):
        os.remove(dump)
    print(f"  -> LS oracle {config} …", flush=True)
    run_sim([BIN, "-u", "Cmdenv", LS_INI, COMMON_INI, "-c", config, *CONV,
             f'--**.oracleDumpFile="{dump}"',
             "--result-dir", os.path.join(OUT, f"ls_{config}")])
    if not os.path.exists(dump):
        sys.exit(f"FAIL: LS {config} produced no oracle dump")
    return _parse_oracle(dump)


def main(args=None):
    import argparse
    from pathlib import Path
    if args is None:
        p = argparse.ArgumentParser()
        add_topo_args(p)
        args = p.parse_args()
        apply_repeat_args(args)

    os.makedirs(OUT, exist_ok=True)
    if not os.path.exists(BIN):
        sys.exit(f"RoutingSim binary not found at {BIN} — build it first.")

    for topo in CONFIGS.values():
        if topo is None:
            continue
        ensure_ring_labels(SIM_ROOT / "topologies" / topo, "min")

    all_ok = True
    for config, _ in CONFIGS.items():
        print(f"\n=== {config} ===")
        gp = run_gp(config, os.path.join(OUT, f"gp_{config}.txt"))
        oracle = run_oracle(config, os.path.join(OUT, f"oracle_{config}.txt"))
        if not _compare(gp, oracle):
            all_ok = False

    # Also test the caller's topology on the SAME basis the eval will use, so the
    # gate validates the exact ring labeling the eval runs on (the named fixtures
    # above are pinned to the min-basis labels baked into myriadgp.ini).
    topo = Path(args.topo)
    label = topo.stem
    print(f"\n=== {label} (custom, basis={args.basis}) ===")
    labels = ensure_ring_labels(topo, args.basis)
    gp_ini = write_proto_ini(os.path.join(OUT, f"gp_{label}.ini"), "gp",
                             topo, labels, extra_lines=_REFRESH_LINES,
                             update_jitter=args.update_jitter, warmup=args.warmup)
    ls_ini = write_proto_ini(os.path.join(OUT, f"ls_{label}.ini"), "ls",
                             topo, labels, extra_lines=_REFRESH_LINES)
    gp = _run_gp_ini(gp_ini, os.path.join(OUT, f"gp_{label}.txt"), label)
    oracle = _run_oracle_ini(ls_ini, os.path.join(OUT, f"oracle_{label}.txt"), label)
    if not _compare(gp, oracle):
        all_ok = False

    if not all_ok:
        sys.exit("\nOPTIMALITY CHECK FAILED.")
    print("\nPASS: MyriadGP matches the independent Dijkstra optimum on all fixtures.")


if __name__ == "__main__":
    main()
