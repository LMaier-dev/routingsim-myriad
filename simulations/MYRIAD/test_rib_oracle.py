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
import subprocess
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
SIM_ROOT = os.path.dirname(HERE)
REPO = os.path.dirname(SIM_ROOT)
BIN = os.path.join(REPO, "src", "RoutingSim")
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
CONV = ["--sim-time-limit=120s"]


def _run(cmd, what):
    r = subprocess.run(cmd, cwd=SIM_ROOT, capture_output=True, text=True)
    log = r.stdout + r.stderr
    if r.returncode != 0 or "budget expired" in log or "ring broken" in log:
        sys.exit(f"FAIL: {what} run errored\n{log[-1200:]}")
    return log


def run_gp(config, dump):
    """GP with pure-cost ranking; returns {(node, 'origin=r'): (cost, spof)}."""
    if os.path.exists(dump):
        os.remove(dump)
    _run([BIN, "-u", "Cmdenv", GP_INI, COMMON_INI, "-c", config, *CONV,
          "--**.rankUseSpof=false", "--**.holdInterval=0s",
          f'--**.ribDumpFile="{dump}"',
          "--result-dir", os.path.join(OUT, f"gp_{config}")], f"GP {config}")
    if not os.path.exists(dump):
        sys.exit(f"FAIL: GP {config} produced no RIB dump")
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


def run_oracle(config, dump):
    """MyriadLS Dijkstra oracle; returns {(node, 'origin=r'): cost}."""
    if os.path.exists(dump):
        os.remove(dump)
    _run([BIN, "-u", "Cmdenv", LS_INI, COMMON_INI, "-c", config,
          "--sim-time-limit=120s", f'--**.oracleDumpFile="{dump}"',
          "--result-dir", os.path.join(OUT, f"ls_{config}")], f"LS {config}")
    if not os.path.exists(dump):
        sys.exit(f"FAIL: LS {config} produced no oracle dump")
    out = {}
    with open(dump) as f:
        for l in f:
            if not l.strip():
                continue
            kv = dict(p.split("=", 1) for p in l.split() if "=" in p)
            out[(l.split()[0], f"origin={kv['origin']}")] = int(kv["cost"])
    return out


def main():
    os.makedirs(OUT, exist_ok=True)
    if not os.path.exists(BIN):
        sys.exit(f"RoutingSim binary not found at {BIN} — build it first.")

    # Regenerate ring labels for fixtures that derive them from a topology (grids);
    # inline-label fixtures (paper) already carry matched labels in both inis.
    for topo in CONFIGS.values():
        if topo is None:
            continue
        r = subprocess.run(
            [sys.executable, os.path.join(HERE, "topo2rings.py"),
             os.path.join(SIM_ROOT, "topologies", topo), "--out-dir", HERE, "--no-png"],
            capture_output=True, text=True)
        if r.returncode != 0:
            sys.exit(f"topo2rings.py failed for {topo}:\n{r.stdout}{r.stderr}")

    all_ok = True
    for config, _ in CONFIGS.items():
        gp = run_gp(config, os.path.join(OUT, f"gp_{config}.txt"))
        oracle = run_oracle(config, os.path.join(OUT, f"oracle_{config}.txt"))
        print(f"\n=== {config}: {len(gp)} GP entries vs {len(oracle)} oracle entries ===")

        missing = [k for k in oracle if k not in gp]
        # UNKNOWN_RING_SIZE (1<<20) squared dominates any real cost: a preferred
        # chain carrying it never resolved a ring size = GP didn't converge here.
        SENTINEL = 10 ** 12
        unconverged = [k for k in oracle if k in gp and gp[k][0] > SENTINEL]
        mismatch = [(k, gp[k][0], oracle[k]) for k in oracle
                    if k in gp and gp[k][0] <= SENTINEL and gp[k][0] != oracle[k]]
        max_spof = max((s for _, s in gp.values()), default=0)

        if missing:
            all_ok = False
            print(f"  FAIL: {len(missing)} (node,origin) reachable in oracle but MISSING in GP")
            for k in sorted(missing)[:8]:
                print(f"        node={k[0]} {k[1]}")
        if unconverged:
            all_ok = False
            print(f"  FAIL: {len(unconverged)} entries UNCONVERGED (UNKNOWN_RING_SIZE in preferred chain)")
            for k in sorted(unconverged)[:8]:
                print(f"        node={k[0]} {k[1]}")
        if mismatch:
            all_ok = False
            print(f"  FAIL: {len(mismatch)} cost MISMATCHES (GP converged but not least-cost)")
            for k, gc, oc in sorted(mismatch)[:8]:
                print(f"        node={k[0]} {k[1]} gp_cost={gc} optimal={oc} (+{gc - oc})")
        if not (missing or unconverged or mismatch):
            # spof>0 is expected here: rankUseSpof=false lets GP take least-cost
            # paths through single-node (corner) ring adjacencies.
            print(f"  OK: every oracle (node,origin) cost matched by GP "
                  f"(max spof on chosen paths = {max_spof})")

    if not all_ok:
        sys.exit("\nOPTIMALITY CHECK FAILED.")
    print("\nPASS: MyriadGP matches the independent Dijkstra optimum on all fixtures.")


if __name__ == "__main__":
    main()
