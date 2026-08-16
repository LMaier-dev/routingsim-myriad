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
configs on the SAME failure schedule (dynamics RNG isolated, as in
eval_overhead.py): GP replay-off, GP replay-on, LS. For every node that fails we
measure recovery latency = time from its reboot until its RIB is full again.

    python3 simulations/MYRIAD/eval_recovery.py

Outputs a latency table + one route-count-vs-time plot under
simulations/MYRIAD/eval/recovery/.
"""
import glob
import os
import re
import subprocess
import sys
from collections import defaultdict

HERE = os.path.dirname(os.path.abspath(__file__))
SIM_ROOT = os.path.dirname(HERE)
REPO = os.path.dirname(SIM_ROOT)
SRC = os.path.join(REPO, "src")
BIN = os.path.join(SRC, "RoutingSim")
COMMON_INI = os.path.join(HERE, "eval_common.ini")
TOPO2RINGS = os.path.join(HERE, "topo2rings.py")
OUT = os.path.join(HERE, "eval", "recovery")
TOPO = os.path.join(SIM_ROOT, "topologies", "grid_10,10.txt")

SIMTIME = "180s"
SNAP = "5s"
# CatastrophicFailure fails `fraction` of nodes per wave; the first failure lands
# at warmup+period (occurrence is a delay), then repeats each period. Duration <
# refreshInterval (100s) so the OFF case visibly lags the refresh-driven refill.
WARMUP, PERIOD, DURATION, FRACTION = "8s", "20s", "5s", 0.08

# stats snapshot line: "t=<t> <nodeid> n=<count> cost_sum=.. cost_max=.."
SNAP_RE = re.compile(r"^t=([0-9.]+)s?\s+(\d+)\s+n=(\d+)", re.M)


def ring_label_lines():
    stem = os.path.splitext(os.path.basename(TOPO))[0]
    labels = os.path.join(HERE, f"{stem}.ringlabels.ini")
    r = subprocess.run([sys.executable, TOPO2RINGS, TOPO, "--out-dir", HERE, "--no-png"],
                       capture_output=True, text=True)
    if not os.path.exists(labels):
        sys.exit(f"topo2rings.py failed:\n{r.stdout}{r.stderr}")
    with open(labels) as f:
        return [ln.rstrip("\n") for ln in f if ln.strip() and not ln.startswith("#")]


# GP dumps to ribDumpFile, LS to oracleDumpFile (its routes come from the
# RingSpfOracle over the LSDB); both honour snapshotInterval/snapshotFormat.
CONFIGS = [
    ("gp_off", "gp", "ribDumpFile",    "false"),
    ("gp_on",  "gp", "ribDumpFile",    "true"),
    ("ls",     "ls", "oracleDumpFile", None),
]


def write_ini(proto, ring_lines):
    ini = os.path.join(OUT, f"{proto}.ini")
    lines = [
        f"include {COMMON_INI}",
        "[General]",
        f"ned-path={SRC};{SIM_ROOT}",
        f'**.protocols = "routingsim.routing.MYRIAD.myriad{proto}"',
        "network = routingsim.simulations.MYRIAD.RecoveryNetwork",
        f'*.manager.topologyFile = "{TOPO}"',
        "**.mobility = false",
        f"**.dynamics.warmup = {WARMUP}",
        f"**.dynamics.period = {PERIOD}",
        f"**.dynamics.duration = {DURATION}",
        f"**.dynamics.fraction = {FRACTION}",
        "**.dynamics.minDegree = 0",   # grid corners are degree 2; fail any node
        f"**.snapshotInterval = {SNAP}",
        '**.snapshotFormat = "stats"',
    ] + ring_lines
    with open(ini, "w") as f:
        f.write("\n".join(lines) + "\n")
    return ini


def run(name, proto, dumppar, replay, ring_lines):
    ini = write_ini(proto, ring_lines)
    resdir = os.path.join(OUT, f"res_{name}")
    dump = os.path.join(OUT, f"{name}.rib")
    for stale in (dump, *glob.glob(resdir + "/*.sca")):
        if os.path.exists(stale):
            os.remove(stale)
    args = [BIN, "-u", "Cmdenv", ini, f"--sim-time-limit={SIMTIME}",
            f'--**.{dumppar}="{dump}"',
            # isolate the dynamics RNG so every config sees the SAME failures
            "--num-rngs=2", "--**.dynamics.rng-0=1", "--seed-set=1",
            "--result-dir", resdir]
    if replay is not None:
        args.append(f"--**.recoveryReplayEnabled={replay}")
    r = subprocess.run(args, cwd=SIM_ROOT, capture_output=True, text=True)
    if r.returncode != 0:
        sys.exit(f"FAIL: {name}\n{(r.stdout + r.stderr)[-1500:]}")
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


def main():
    os.makedirs(OUT, exist_ok=True)
    if not os.path.exists(BIN):
        sys.exit(f"RoutingSim binary not found at {BIN} — build it first.")
    ring_lines = ring_label_lines()

    series = {name: load(run(name, proto, dp, rep, ring_lines))
              for name, proto, dp, rep in CONFIGS}

    print(f"\n=== Grid10 node-failure recovery ({SIMTIME}, {FRACTION:.0%}/wave, "
          f"dur {DURATION}, refresh 100s) ===")
    print(f"{'config':>8}{'nodes recovered':>17}{'mean latency':>14}"
          f"{'median':>9}{'max':>8}")
    lat = {}
    dur_s = float(DURATION.rstrip("s"))
    for name, *_ in CONFIGS:
        L = recovery_latencies(series[name], dur_s)
        lat[name] = L
        vals = sorted(L.values())
        if vals:
            med = vals[len(vals) // 2]
            print(f"{name:>8}{len(vals):>17}{sum(vals)/len(vals):>13.1f}s"
                  f"{med:>8.0f}s{max(vals):>7.0f}s")
        else:
            print(f"{name:>8}{0:>17}{'-':>14}{'-':>9}{'-':>8}")
    print("\nlatency = seconds from reboot until RIB is full again "
          "(GP-off waits for refresh; GP-on/LS repopulate at once).")

    # Plot: route count vs time for the earliest-failing node, all three configs.
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
           title=f"Recovery of a rebooted node (Grid10, node {fn})")
    ax.legend(); fig.tight_layout()
    p = os.path.join(OUT, "recovery_node.png")
    fig.savefig(p, dpi=120)
    print(f"\nplot -> {p}")


if __name__ == "__main__":
    main()
