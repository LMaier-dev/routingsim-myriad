#!/usr/bin/env python3
"""Optimality gate for MyriadPV, checked against an independent Dijkstra oracle.

test_rib_batching.py proves batching preserves REACHABILITY. It cannot tell an
optimal RIB from a merely-connected one. This does: MyriadLS floods a full
link-state DB and (via RingSpfOracle) runs Dijkstra on the ring graph to compute
the OPTIMAL least-cost path from every node to every origin ring. MyriadPV,
running the same topology with rankUseSpof=false (pure cost ranking), must agree
on every (node, origin) cost. Two independently-converged protocols + two
different algorithms (path-vector vs. shortest-path) matching = both reached the
same optimum.

    python3 simulations/MYRIAD/test_rib_oracle.py

Exit 0 = PV is cost-optimal on every fixture.

Note: both fixtures here happen to be spof=0 everywhere (every ring transition is
double-bridged), so rankUseSpof=false is a no-op on them — PV is already
cost-ranked. The knob's discriminating power (declining a costlier redundant path
in favour of the least-cost one) only shows on a topology with a single-bridge
alternative, which we don't yet have a fixture for.
"""
import os
import sys

from eval_harness import (BIN, SIM_ROOT, add_topo_args, apply_repeat_args,
                          ensure_ring_labels, overrides, run_sim, warmup_s,
                          write_proto_ini)

HERE = os.path.dirname(os.path.abspath(__file__))
OUT = os.path.join(HERE, "eval", "oracle")

# The 7-node fixture's labels, which have no generator: its rings are the
# hand-drawn ones from the paper figure, not a basis of the cycle space.
_PAPER_LABELS = [f"**.Node{n}.*.ringLabel = \"{r}\"" for n, r in enumerate(
    ["1000000", "1000000,999999", "999999,999998", "1000000,999999",
     "999999", "999998", "999999,999998"])]

# Pinned regression fixtures: name -> (topology, ring labels). A label list is
# used as-is; None means generate the min-basis labels from the topology.
# Built into their own inis rather than run out of myriad{pv,ls}.ini -c NAME:
# OMNeT++ resolves every `include` in the file, including ones under configs you
# did not select, so an unrelated section's missing ring .ini aborts the run.
CONFIGS = {
    "Grid10": (SIM_ROOT / "topologies" / "grid_10,10.txt", None),
    "PaperTopology": (SIM_ROOT / "MYRIAD" / "myriad_topologies" / "csv"
                      / "myriad_paperexample.txt", _PAPER_LABELS),
}

# One refresh cycle, well above convergence time so generations never overlap
# (an overlapping flood would be sampled mid-propagation). The horizon leaves
# 20s after the single refresh at t=_REFRESH for it to settle.
_REFRESH = 60.0
_REFRESH_LINES = [f"**.refreshInterval = {_REFRESH:g}s", "**.refreshJitter = 0s"]

# Final RIB only: snapshots off, or the t=-prefixed samples land in the dump.
CONV = overrides(sim_time_limit=f"{warmup_s() + _REFRESH + 20:g}s",
                 snapshotInterval="0s", refreshInterval=f"{_REFRESH:g}s",
                 refreshJitter="0s")


def _parse_pv(dump):
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


def _compare(pv, oracle, fatal=True):
    """Print comparison results; return True if all checks pass.

    fatal=False downgrades the wording to WARN: used for the caller's own
    topology, where a suboptimal chain is a finding to report, not a reason to
    withhold the eval. The pinned fixtures stay fatal — they are the regression."""
    tag = "FAIL" if fatal else "WARN"
    print(f"  {len(pv)} PV entries vs {len(oracle)} oracle entries")
    missing = [k for k in oracle if k not in pv]
    SENTINEL = 10 ** 12
    unconverged = [k for k in oracle if k in pv and pv[k][0] > SENTINEL]
    mismatch = [(k, pv[k][0], oracle[k]) for k in oracle
                if k in pv and pv[k][0] <= SENTINEL and pv[k][0] != oracle[k]]
    max_spof = max((s for _, s in pv.values()), default=0)
    ok = True
    if missing:
        ok = False
        print(f"  {tag}: {len(missing)} (node,origin) reachable in oracle but MISSING in PV")
        for k in sorted(missing)[:8]:
            print(f"        node={k[0]} {k[1]}")
    if unconverged:
        ok = False
        print(f"  {tag}: {len(unconverged)} entries UNCONVERGED (UNKNOWN_RING_SIZE in preferred chain)")
        for k in sorted(unconverged)[:8]:
            print(f"        node={k[0]} {k[1]}")
    if mismatch:
        ok = False
        print(f"  {tag}: {len(mismatch)} cost MISMATCHES (PV converged but not least-cost)")
        for k, gc, oc in sorted(mismatch)[:8]:
            print(f"        node={k[0]} {k[1]} pv_cost={gc} optimal={oc} (+{gc - oc})")
    if ok:
        print(f"  OK: every oracle (node,origin) cost matched by PV "
              f"(max spof on chosen paths = {max_spof})")
    return ok


def _run_pv_ini(ini, dump, label):
    if os.path.exists(dump):
        os.remove(dump)
    print(f"  -> PV {label} …", flush=True)
    run_sim([BIN, "-u", "Cmdenv", ini, *CONV,
             "--**.rankUseSpof=false",
             "--**.ecmpCostMargin=0",   # oracle is exact least-cost
             f'--**.ribDumpFile="{dump}"',
             "--result-dir", os.path.join(OUT, f"pv_{label}")])
    if not os.path.exists(dump):
        sys.exit(f"FAIL: PV {label} produced no RIB dump")
    return _parse_pv(dump)


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


def _labels_for(label, topo, labels):
    """Path to the ring labels for a fixture: generated from the topology, or
    written out from an inline list."""
    if labels is None:
        return ensure_ring_labels(topo, "min")
    out = os.path.join(OUT, f"{label}.ring.ini")
    with open(out, "w") as f:
        f.write("\n".join(labels) + "\n")
    return out


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

    all_ok = True
    for config, (topo, labels) in CONFIGS.items():
        print(f"\n=== {config} ===")
        lbl = _labels_for(config, topo, labels)
        pv_ini = write_proto_ini(os.path.join(OUT, f"pv_{config}.ini"), "pv",
                                 topo, lbl, extra_lines=_REFRESH_LINES)
        ls_ini = write_proto_ini(os.path.join(OUT, f"ls_{config}.ini"), "ls",
                                 topo, lbl, extra_lines=_REFRESH_LINES)
        pv = _run_pv_ini(pv_ini, os.path.join(OUT, f"pv_{config}.txt"), config)
        oracle = _run_oracle_ini(ls_ini, os.path.join(OUT, f"oracle_{config}.txt"),
                                 config)
        if not _compare(pv, oracle):
            all_ok = False

    # Also test the caller's topology on the SAME basis the eval will use, so the
    # gate validates the exact ring labeling the eval runs on (the fixtures above
    # are pinned to their own min-basis labels).
    topo = Path(args.topo)
    label = topo.stem
    print(f"\n=== {label} (custom, basis={args.basis}) ===")
    labels = ensure_ring_labels(topo, args.basis)
    pv_ini = write_proto_ini(os.path.join(OUT, f"pv_{label}.ini"), "pv",
                             topo, labels, extra_lines=_REFRESH_LINES,
                             update_jitter=args.update_jitter, warmup=args.warmup)
    ls_ini = write_proto_ini(os.path.join(OUT, f"ls_{label}.ini"), "ls",
                             topo, labels, extra_lines=_REFRESH_LINES)
    pv = _run_pv_ini(pv_ini, os.path.join(OUT, f"pv_{label}.txt"), label)
    oracle = _run_oracle_ini(ls_ini, os.path.join(OUT, f"oracle_{label}.txt"), label)
    if not _compare(pv, oracle, fatal=False):
        print(f"  (warning only: {label} is the eval's own topology, not a pinned "
              "regression fixture — continuing)")

    if not all_ok:
        sys.exit("\nOPTIMALITY CHECK FAILED.")
    print("\nPASS: MyriadPV matches the independent Dijkstra optimum on all fixtures.")


if __name__ == "__main__":
    main()
