#!/usr/bin/env python3
"""Node-failure recovery: how fast does a rebooted router get its RIB back?

A crashed MyriadGP (path-vector) router loses all its chains and, unlike
MyriadLS (link-state, which re-syncs its LSDB from any neighbor), has no way to
regain them except waiting for every origin's next refresh. The recovery-replay
mechanism (recoveryReplayEnabled, see MyriadGp) closes that gap: on the COLLECT
that readmits a recovered member, the ring king replays its current front-of-ring
chains as seq=0 UPDATEs, so the recovered node repopulates within one COLLECT
round.

This eval fails nodes with the CatastrophicFailure dynamics model (a node-only
model that reboots them after `duration`) on a dedicated RecoveryNetwork, and
snapshots every node's route count over time (snapshotFormat="stats"). Three
configs on the SAME failure schedule (dynamics RNG isolated): GP replay-off, GP
replay-on, LS. For every node that fails we measure recovery latency = time from
its reboot until its RIB is full again.

    python3 simulations/MYRIAD/eval_recovery.py [--topo PATH] [--basis {min,all}]

Outputs a latency table + one route-count-vs-time plot under
simulations/MYRIAD/eval/recovery/.
"""
import argparse
import glob
import os
import re
import sys
from collections import defaultdict

from eval_harness import (BIN, SIM_ROOT, MYRIAD_DIR, apply_repeat_args,
                          ensure_ring_labels, pm, rep_path, repeat_mean,
                          repeats_note, run_sim, write_proto_ini, add_topo_args)

HERE = str(MYRIAD_DIR)
OUT = os.path.join(HERE, "eval", "recovery")

SIMTIME = "180s"
SNAP = "5s"
# CatastrophicFailure fails `fraction` of nodes per wave; the first failure lands
# at warmup+period (occurrence is a delay), then repeats each period. Duration <
# refreshInterval (100s) so the OFF case visibly lags the refresh-driven refill.
WARMUP, PERIOD, DURATION, FRACTION = "8s", "20s", "5s", 0.08

RECOVERY_NETWORK = "routingsim.simulations.MYRIAD.RecoveryNetwork"

# stats snapshot line: "t=<t> <nodeid> n=<count> cost_sum=.. cost_max=.."
SNAP_RE = re.compile(r"^t=([0-9.]+)s?\s+(\d+)\s+n=(\d+)", re.M)

# GP dumps to ribDumpFile, LS to oracleDumpFile (its routes come from the
# RingSpfOracle over the LSDB); both honour snapshotInterval/snapshotFormat.
CONFIGS = [
    ("gp_off", "gp", "ribDumpFile",    "false"),
    ("gp_on",  "gp", "ribDumpFile",    "true"),
    ("ls",     "ls", "oracleDumpFile", None),
]


def run(name, ini, dumppar, replay, rep=0):
    resdir = rep_path(os.path.join(OUT, f"res_{name}"), rep)
    dump = rep_path(os.path.join(OUT, f"{name}.rib"), rep)
    for stale in (dump, *glob.glob(resdir + "/*.sca")):
        if os.path.exists(stale):
            os.remove(stale)
    args = [BIN, "-u", "Cmdenv", ini, f"--sim-time-limit={SIMTIME}",
            f'--**.{dumppar}="{dump}"',
            # isolate the dynamics RNG so every config in a repeat sees the SAME
            # failures (run_sim pins the seed-set per repeat)
            "--num-rngs=2", "--**.dynamics.rng-0=1",
            "--result-dir", resdir]
    if replay is not None:
        args.append(f"--**.recoveryReplayEnabled={replay}")
    run_sim(args, rep=rep)
    return dump


def load(dump):
    """nodeid -> {t: route_count}"""
    d = defaultdict(dict)
    txt = open(dump).read()
    for m in SNAP_RE.finditer(txt):
        d[int(m.group(2))][float(m.group(1))] = int(m.group(3))
    return d


def recovery_latencies(series, duration_s):
    """For each node that fails (route count dips low), seconds from its reboot
    (dip-start + failure duration) until its RIB is full again (>=95% of its own
    max). Route count alone can't mark the reboot — GP-off stays low past it — so
    we anchor on the known failure duration. First downtime per node only."""
    out = {}
    for nid, ser in series.items():
        ts = sorted(ser)
        full = max(ser.values())
        if full == 0:
            continue
        low = max(10, 0.2 * full)
        dips = [t for t in ts if ser[t] < low]
        if not dips:
            continue
        reboot = dips[0] + duration_s
        done = next((t for t in ts if t >= reboot and ser[t] >= 0.95 * full), None)
        if done is not None:
            out[nid] = max(0.0, done - reboot)
    return out


def main(args=None):
    if args is None:
        p = argparse.ArgumentParser()
        add_topo_args(p)
        args = p.parse_args()
        apply_repeat_args(args)

    os.makedirs(OUT, exist_ok=True)
    if not os.path.exists(BIN):
        sys.exit(f"RoutingSim binary not found at {BIN} — build it first.")

    labels = ensure_ring_labels(args.topo, args.basis)
    recovery_extra = [
        f"**.dynamics.warmup = {WARMUP}",
        f"**.dynamics.period = {PERIOD}",
        f"**.dynamics.duration = {DURATION}",
        f"**.dynamics.fraction = {FRACTION}",
        "**.dynamics.minDegree = 0",   # grid corners are degree 2; fail any node
        f"**.snapshotInterval = {SNAP}",
        '**.snapshotFormat = "stats"',
    ]
    inis = {
        proto: write_proto_ini(
            os.path.join(OUT, f"{proto}.ini"), proto,
            args.topo, labels,
            extra_lines=recovery_extra,
            update_jitter=args.update_jitter if proto == "gp" else None,
            warmup=args.warmup if proto == "gp" else None,
            network=RECOVERY_NETWORK)
        for proto in ("gp", "ls")
    }

    dur_s = float(DURATION.rstrip("s"))
    # One repeat = all three configs on the same seed (they must share a failure
    # schedule to be comparable). Rep 0's raw series feeds the plot: which node
    # fails differs per seed, so per-node traces can't be averaged.
    series0 = {}

    def one(rep):
        stats = {}
        for name, proto, dumppar, replay in CONFIGS:
            if rep == 0:
                print(f"  → {name} …", flush=True)
            series = load(run(name, inis[proto], dumppar, replay, rep))
            if rep == 0:
                series0[name] = series
                print(f"  ✓ {name} done ({len(series)} nodes tracked)")
            vals = sorted(recovery_latencies(series, dur_s).values())
            # keep every key even when nothing recovered — the repeat aggregation
            # keeps only keys present in ALL repeats
            stats[name] = {"n": len(vals),
                           "mean": sum(vals) / len(vals) if vals else 0.0,
                           "median": vals[len(vals) // 2] if vals else 0.0,
                           "max": max(vals) if vals else 0.0}
        return stats

    lat = repeat_mean(one)

    print(f"\n=== node-failure recovery ({SIMTIME}, {FRACTION:.0%}/wave, "
          f"dur {DURATION}, refresh 100s){repeats_note()} ===")
    print(f"{'config':>8}{'nodes recovered':>17}{'mean latency':>17}"
          f"{'median':>9}{'max':>8}")
    for name, *_ in CONFIGS:
        s = lat[name]
        if s["n"]:
            mean = f"{s['mean']:.1f}{pm(s['mean'])}s"
            n = f"{s['n']:.0f}{pm(s['n'])}"
            print(f"{name:>8}{n:>17}{mean:>17}"
                  f"{s['median']:>8.0f}s{s['max']:>7.0f}s")
        else:
            print(f"{name:>8}{0:>17}{'-':>17}{'-':>9}{'-':>8}")
    series = series0
    print("\nlatency = seconds from reboot until RIB is full again "
          "(GP-off waits for refresh; GP-on/LS repopulate at once).")

    off = series["gp_off"]
    fn, tz = None, 1e9
    for nid, ser in off.items():
        zs = [t for t, v in ser.items() if v == 0]
        if zs and min(zs) < tz:
            tz, fn = min(zs), nid
    try:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
    except ImportError:
        print("\n(matplotlib not available — skipping plot)")
        return
    if fn is None:
        print("\n(no node failed — check dynamics params)")
        return
    fig, ax = plt.subplots()
    for name, color in (("gp_off", "C3"), ("gp_on", "C0"), ("ls", "C2")):
        ser = series[name].get(fn, {})
        ts = sorted(ser)
        ax.plot(ts, [ser[t] for t in ts], marker=".", label=name, color=color)
    ax.set(xlabel="time (s)", ylabel=f"routes at node {fn}",
           title=f"Recovery of a rebooted node (node {fn})")
    ax.legend(); fig.tight_layout()
    p = os.path.join(OUT, "recovery_node.png")
    fig.savefig(p, dpi=120)
    print(f"\nplot -> {p}")


if __name__ == "__main__":
    main()
