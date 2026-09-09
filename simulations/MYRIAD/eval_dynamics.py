#!/usr/bin/env python3
"""Control-plane cost and failure response of MyriadGP vs MyriadLS.

Three scenarios, one run each per protocol, every run scored on BOTH families:
traffic (messages/bytes per PDU type) and, where nodes fail, reconvergence time.

    churn       NodeChurn, a node down/up every  → overhead under a background
                --churn-period                      failure rate the network absorbs
    link        LinkFailure, repeating flaps     → overhead per unit of link churn
    node        EscalatingFailure, growing waves → reconvergence vs failure size,
                swept over --fractions, with the PV recovery-replay ablation

    python3 simulations/MYRIAD/eval_dynamics.py [--topo PATH] [--only churn|link|node]

--overview is a second, traffic-only mode: the same churn and node runs with RIB
snapshotting off, scored as messages and bytes per failure window (churn steady
state, then one window per wave fraction), repeated over --repeat RNG seeds and
drawn as boxplots — over a whole directory of topologies with --dir. Every
(topology, scenario, config, seed) run is independent, so --jobs spreads the
cross product over one pool.

Replaces eval_overhead.py, eval_recovery.py and eval_reconverge.py: same runs,
one set of conditions, and the two metric families now describe the same sim.
Outputs tables + plots under simulations/MYRIAD/eval/dynamics/.
"""
import argparse
import csv
import glob
import os
import re
import shutil
import subprocess
import sys
from collections import defaultdict
from concurrent.futures import ThreadPoolExecutor, as_completed

csv.field_size_limit(sys.maxsize)  # a node's whole txBytes series is one CSV-R cell

import eval_harness as H   # REPEATS/JOBS/PROGRESS are bound at parse time
from eval_harness import (BIN, MYRIAD_DIR, _sec, add_topo_args, announce,
                          apply_repeat_args, byte_unit, common, dynamics_args,
                          ensure_ring_labels, fmt_bytes, overrides, pm,
                          refresh_s, rep_path,
                          repeat_mean, repeats_note, run_sim, simtime, title, warmup_s,
                          write_proto_ini, PROTO_COLOR, PROTO_LABEL)

OUT = os.path.join(str(MYRIAD_DIR), "eval", "dynamics")
RECOVERY_NETWORK = "routingsim.simulations.MYRIAD.RecoveryNetwork"
CHURN_NETWORK = "routingsim.simulations.MYRIAD.ChurnNetwork"

DEFAULT_FRACTIONS = [0.05, 0.1, 0.2, 0.4, 0.8]
CHURN_NODES, CHURN_PERIOD = 1, 1.0  # churn scenario: nodes down per period, seconds
FAILURE_START = 100.0              # churn / first node wave, and the measurement cut
QUIET_LEN = 25.0                   # idle-baseline window, ending at FAILURE_START
DOWN_MIN, DOWN_JITTER = 2.0, 8.0   # per-node downtime: DOWN_MIN + U(0, JITTER)
COLLECT_RETRY_LIMIT = 3            # mirrors MyriadGp.h

SENT_RE = re.compile(r"^scalar\s+\S+\s+sent_(\S+)\s+([0-9.]+)", re.M)
BYTES_RE = re.compile(r"^scalar\s+\S+\s+bytes_(\S+)\s+([0-9.]+)", re.M)
STATS_RE = re.compile(r"^t=([0-9.]+)s?\s+(\d+)\s+n=(\d+)\s+cost_sum=(\d+)"
                      r"\s+cost_max=(\d+)(?:\s+present=(\d+))?", re.M)
UNKNOWN_RING_SIZE = 1 << 20   # mirrors MyriadGp.h
# EscalatingFailure records when each node it failed came back: down_<node>_<wave>
DOWN_RE = re.compile(r"^scalar\s+\S+\s+down_(\d+)_(\d+)\s+([0-9.]+)", re.M)

# name -> (protocol, dump parameter, recoveryReplayEnabled, refreshFloodsUpdate).
# Last two: None = don't override (NED default, true). pv_noreplay is the
# refresh-bound baseline; opt in with --noreplay. It NEEDS refreshFloodsUpdate's
# default (true) — periodic reflood is its only propagation path without replay
# — so --no-refresh-flood (below) only touches the "gp" entry, never this one.
# A blanket --set refreshFloodsUpdate=... would hit both and cripple pv_noreplay.
CONFIGS = {"pv": ("pv", "ribDumpFile", "true", None),
           "pv_noreplay": ("pv", "ribDumpFile", "false", None),
           # rankUseSpof=false: pure-cost ranking, so PV targets the same optimum
           # LS computes and the stretch table isolates recovery damage from the
           # redundancy premium production ranking pays on purpose.
           "pv_nospof": ("pv", "ribDumpFile", "true", None),
           "ls": ("ls", "oracleDumpFile", None, None)}

EXTRA_OPTS = {"pv_nospof": ["--**.rankUseSpof=false"]}

# Colours and legend names are the harness's, so every figure in the eval uses
# the same ones. Ordered to match CONFIGS.
COLORS = {n: PROTO_COLOR[n] for n in CONFIGS}
DISPLAY = {n: PROTO_LABEL[n] for n in CONFIGS}


def churn_start():
    """When the first failure fires (churn and node alike) and where every
    post-warmup measurement starts, so what is scored is failure cost, not
    leftover initial convergence. --failure-start."""
    return FAILURE_START


def traffic_simtime():
    """Long enough that the post-start window is a full 1.5 refresh cycles even
    when --failure-start pushes churn past the default horizon."""
    return f"{max(_sec(simtime(cycles=1.5)), churn_start() + 1.5 * refresh_s()):g}s"


def node_simtime(nwaves, spacing):
    return f"{churn_start() + nwaves * spacing:g}s"


def wave_times(fractions, spacing):
    """First wave at --failure-start (same as churn), then one every `spacing`."""
    return [(f, churn_start() + i * spacing) for i, f in enumerate(fractions)]


def grace_note():
    """Which side of the crash-probe budget this run's downtime falls on. Churn
    never re-announces a ring: a dead ring-neighbour only draws a PROBE lap, whose
    sole outcomes are 'closed, ring whole' or 'expired, flood DEGRADE'. The budget
    decides which."""
    b = (COLLECT_RETRY_LIMIT + 0.5) * _sec(common("rxmtInterval"))
    lo, hi = DOWN_MIN, DOWN_MIN + DOWN_JITTER
    verdict = ("probes close on reboot, so no DEGRADE" if hi <= b else
               "probes expire into DEGRADE" if lo >= b else
               "probes close only for the shorter outages")
    return (f"downtime {lo:g}-{hi:g}s vs {b:g}s probe budget "
            f"(({COLLECT_RETRY_LIMIT}+0.5)*rxmtInterval): {verdict}. A rebooted "
            "router refills from its own replay lap; everything else waits for "
            "the king's refresh or a watchdog coup")


def node_refresh(spacing):
    """One refresh cycle per wave. Origins whose ring king co-failed need its
    re-election first (2*refresh watchdog) and so land a wave or more later —
    that lag is a result of the wave series, not something to tune away."""
    return spacing


def node_conditions(spacing):
    """One refresh per wave. collectWarmupDuration is left at eval_common's value
    so node and churn warm up identically — the first wave is delayed to
    --failure-start instead, which is what keeps formation out of the window."""
    return overrides(refreshInterval=f"{node_refresh(spacing):g}s")


def scenario_dynamics(scenario, fractions=None, spacing=None):
    """(cli options, human description) for one scenario. Periods derive from the
    central refresh interval so churn scales with --refresh."""
    if scenario == "churn":
        # Not warmup_s(): collectWarmupDuration is also when PV unblocks
        # forwardChainInterRing, so churn starting there superimposes the initial
        # propagation burst on the first failure. Settle first, then churn.
        return dynamics_args("churn", numNodes=CHURN_NODES,
                             period=f"{CHURN_PERIOD:g}s",
                             warmup=f"{churn_start():g}s",
                             downMin=f"{DOWN_MIN:g}s", downJitter=f"{DOWN_JITTER:g}s",
                             minDegree=0)
    if scenario == "link":
        r = refresh_s()
        return dynamics_args("link", numLinksFail=4,
                             period=f"{max(5, round(r / 6)):g}s",
                             duration=f"{max(2, round(r / 15)):g}s",
                             oneFailure="false")
    return dynamics_args("node", fractions='"' + ",".join(f"{f:g}" for f in fractions) + '"',
                         waveSpacing=f"{spacing:g}s",
                         warmup=f"{churn_start():g}s",
                         downMin=f"{DOWN_MIN:g}s", downJitter=f"{DOWN_JITTER:g}s",
                         minDegree=0)


def sim_cmd(name, ini, dyn, simt, resdir, dump=None):
    """Command line for one config's run. dump=None turns RIB snapshotting off —
    overview mode reads traffic only, and the dump is the expensive part."""
    _, dumppar, replay, refreshFlood = CONFIGS[name]
    cmd = [BIN, "-u", "Cmdenv", ini, *overrides(sim_time_limit=simt), *dyn,
           "--result-dir", resdir]
    if dump:
        cmd += [f'--**.{dumppar}="{dump}"', '--**.snapshotFormat="stats"']
    else:
        cmd.append("--**.snapshotInterval=0s")
    if replay is not None:
        cmd.append(f"--**.recoveryReplayEnabled={replay}")
    if refreshFlood is not None:
        cmd.append(f"--**.refreshFloodsUpdate={refreshFlood}")
    return cmd + EXTRA_OPTS.get(name, [])


def measure(name, ini, scenario, dyn, simt=None, rep=0, run=True):
    """One run: traffic totals always, route-count series when nodes fail. run=False
    (post mode) skips the sim and re-parses the on-disk result dir a prior run left."""
    tag = f"{name}_{scenario}"
    resdir = rep_path(os.path.join(OUT, f"res_{tag}"), rep)
    dump = rep_path(os.path.join(OUT, f"{tag}.rib"), rep)
    if run:
        for stale in (dump, *glob.glob(resdir + "/*.sca")):
            if os.path.exists(stale):
                os.remove(stale)
        # churn dumps stats too: same series, but sampled against a constant
        # failure rate instead of escalating waves — steady-state stretch.
        run_sim(sim_cmd(name, ini, dyn, simt or traffic_simtime(), resdir,
                        dump if scenario in ("node", "churn") else None),
                allow_budget_expired=True, rep=rep)

    counts, byts, maxb, reboots = {}, {}, 0, defaultdict(dict)
    for sca in glob.glob(resdir + "/*.sca"):
        txt = open(sca).read()
        for m in DOWN_RE.finditer(txt):
            reboots[int(m.group(2))][int(m.group(1))] = float(m.group(3))
        for m in SENT_RE.finditer(txt):
            if m.group(1) != "total":
                counts[m.group(1)] = counts.get(m.group(1), 0) + int(float(m.group(2)))
        for m in BYTES_RE.finditer(txt):
            t, v = m.group(1), int(float(m.group(2)))
            if t == "max":
                maxb = max(maxb, v)
            elif not t.startswith("max") and t != "total":
                byts[t] = byts.get(t, 0) + v
    if not counts:
        sys.exit(f"FAIL: no .sca in {resdir}" +
                 ("" if run else f" — run this config first (e.g. --protos {name.split('_')[0]}) before --post"))

    # series[node][t] = (route count, summed chain cost, max chain cost,
    # cost-basis count) — [0] drives reconvergence/presence (unfiltered, so LS
    # and PV compare on the same table-refill basis), [1] drives stretch, [3] is
    # the origin count the cost sum ranges over (LS: live-reachable only; PV: ==
    # [0]) so stretch is scored only where both price the same set.
    series = defaultdict(dict)
    if scenario in ("node", "churn") and os.path.exists(dump):
        for m in STATS_RE.finditer(open(dump).read()):
            n_live = int(m.group(3))
            present = int(m.group(6)) if m.group(6) else n_live
            series[int(m.group(2))][float(m.group(1))] = (
                present, int(m.group(4)), int(m.group(5)), n_live)
    # Every latency in the node/churn scenarios is read off these snapshots, so
    # an interval longer than the run silently yields nothing to score.
    if scenario in ("node", "churn") and not series:
        sys.exit(f"FAIL: {name} produced no RIB snapshots over the "
                 f"{simt or traffic_simtime()} run — snapshotInterval is "
                 f"{common('snapshotInterval')}. Lower it (e.g. --snapshot 200ms).")
    return ({"counts": counts, "msgs": sum(counts.values()),
             "bytes": sum(byts.values()), "per_type_bytes": byts, "max_b": maxb},
            series, reboots)


def _scavetool():
    """Path to opp_scavetool. Not on PATH when OMNeT++'s setenv wasn't sourced
    (login shells on the testbed), so fall back to the install dir. Returns None
    if it is genuinely absent — the byte-rate plots are then skipped, not fatal."""
    return (shutil.which("opp_scavetool")
            or next((p for p in (os.path.join(r, "bin", "opp_scavetool")
                                 for r in (os.environ.get("OMNETPP_ROOT", ""),
                                           "/opt/omnetpp-6.3.0",
                                           os.path.expanduser("~/omnetpp-6.3.0")) if r)
                     if os.access(p, os.X_OK)), None))


def _tx_samples(resdir):
    """Yield (time, byte length) for every control-plane PDU any node sent, from
    the per-node txBytes OMNeT++ vector (see MyriadHelper::countSend) rather than
    the finish()-time byte scalars, which only give the run total. One sample IS
    one PDU, so callers get both volume and byte rate over time. A generator: big
    runs hold tens of millions of samples."""
    vecs = glob.glob(os.path.join(resdir, "*.vec"))
    tool = _scavetool()
    if not vecs or not tool:
        if vecs:
            print("  ! opp_scavetool not found — skipping byte-rate plot "
                  "(source omnetpp/setenv or set OMNETPP_ROOT)", flush=True)
        return
    for stale in glob.glob(os.path.join(resdir, "*.vci")):
        os.remove(stale)  # reused result-dir: old index's byte offsets don't match this run's .vec
    csv_path = os.path.join(resdir, "txbytes_export.csv")
    subprocess.run([tool, "export", "-f", "name =~ txBytes",
                    "-F", "CSV-R", "-o", csv_path, vecs[0]],
                   check=True, capture_output=True, text=True)
    try:
        with open(csv_path, newline="") as f:
            for row in csv.DictReader(f):
                if row["type"] != "vector":
                    continue
                for t, v in zip(row["vectime"].split(), row["vecvalue"].split()):
                    yield float(t), float(v)
    finally:
        os.remove(csv_path)


def bytes_over_time(resdir, bin_s=1.0):
    """(bin start times, network-wide bytes/s in that bin) — a failure wave's
    traffic spike is then visible directly."""
    bins = defaultdict(float)
    for t, v in _tx_samples(resdir):
        bins[int(t // bin_s)] += v
    if not bins:
        return [], []
    lo, hi = min(bins), max(bins)
    times = [b * bin_s for b in range(lo, hi + 1)]
    return times, [bins.get(b, 0.0) / bin_s for b in range(lo, hi + 1)]


def bytes_after(series, t0):
    """Total bytes in the (times, bytes/s) series from t0 on, or None if the
    series is unavailable (no opp_scavetool). Bins are 1s wide by construction,
    so bytes/s summed over bins IS the byte total."""
    ts, bps = series
    return sum(b for t, b in zip(ts, bps) if t >= t0) if ts else None


# --- overview mode: traffic per failure window, no snapshots -----------------

def stem(path):
    return os.path.splitext(os.path.basename(path))[0]


def topo_set(args):
    """The topologies this run covers: every *.txt in --dir, else --topo alone.

    A sweep only makes sense where the result is a distribution — overview mode
    — so --dir is rejected elsewhere rather than silently running the first file."""
    d = getattr(args, "dir", None)
    if not d:
        return [args.topo]
    if not getattr(args, "overview", False):
        sys.exit("--dir sweeps a topology set, which only --overview scores; "
                 "use --topo for the per-topology tables and plots")
    found = sorted(glob.glob(os.path.join(d, "*.txt")))
    if not found:
        sys.exit(f"--dir {d}: no *.txt topologies here (the search is not recursive)")
    if getattr(args, "tr_extremes", False):
        # The ends of a transition sweep, every seed of each: two points far
        # apart on the axis, with the seed spread intact at both, instead of the
        # whole sweep at N times the sim cost.
        tr = {f: float(m.group(1)) for f in found
              for m in [re.search(r"_tr([0-9]+)", os.path.basename(f))] if m}
        if len(set(tr.values())) < 2:
            sys.exit(f"--tr-extremes: {d} has no two different _tr<N> values "
                     f"({sorted(set(tr.values())) or 'none found'})")
        lo, hi = min(tr.values()), max(tr.values())
        found = [f for f in found if tr.get(f) in (lo, hi)]
        print(f"  [tr-extremes] keeping tr={lo:g} and tr={hi:g}: {len(found)} files")
    H.TOPO_LABEL = os.path.basename(str(d).rstrip("/"))   # plot titles name the set
    print(f"  [topologies] {len(found)} from {d}: "
          + ", ".join(stem(f) for f in found[:4])
          + (f", … (+{len(found) - 4})" if len(found) > 4 else ""))
    return found


def overview_windows(scenario, fractions, spacing, conv=True):
    """(label, t0, t1) measurement windows for one scenario.

    `conv` is [0, --failure-start): the network forming its RIB from nothing,
    before any failure fires. That is the initial-convergence cost in the same
    units as the failure windows, which is the only way to read "a wave costs X"
    against "coming up costs Y".

    After it, churn's steady state is one window to the end of the run, and each
    node wave owns the span from its own failures to the next wave. No failure
    window carries any of the formation burst."""
    # quiet rides with conv: same run, same reason. Measuring it per scenario
    # would compare windows under different refresh intervals -- the node
    # scenario sets refreshInterval = --wave-spacing, which fires inside [0,
    # --failure-start) and is not the hello floor at all (see the warning in
    # overview()).
    conv = ([("conv", 0.0, churn_start()),
             ("quiet", max(0.0, churn_start() - QUIET_LEN), churn_start())]
            if conv else [])
    # The last QUIET_LEN seconds before the first failure: converged, nothing
    # failing, and (at the default 120s refresh) no refresh cycle yet either, so
    # what is left is the hello floor. Every variant should read the SAME value
    # here -- they run the same hello protocol -- and a variant that reads high
    # is still finishing its formation when the failures land, which would put
    # convergence cost inside the failure windows. It is the tail of `conv`, not
    # a window beside it: conv stays the whole formation cost so it can be
    # compared against a wave as "coming up costs Y".
    if scenario == "churn":
        return conv + [("churn", churn_start(), _sec(traffic_simtime()))]
    waves = wave_times(fractions, spacing)
    ends = [t for _, t in waves[1:]] + [_sec(node_simtime(len(fractions), spacing))]
    return conv + [(f"{f:.0%}", t, e) for (f, t), e in zip(waves, ends)]


def tx_windows(resdir, windows):
    """{label: (bytes, messages)} per window. One vector sample is one PDU, so
    counting samples in a window IS that window's message volume."""
    # No break on the first hit: windows may NEST. quiet is the tail of conv, so
    # a sample in [75,100) belongs to both, and stopping at conv would leave the
    # idle baseline reading a flat zero.
    out = {lab: [0.0, 0] for lab, _, _ in windows}
    for t, v in _tx_samples(resdir):
        for lab, t0, t1 in windows:
            if t0 <= t < t1:
                out[lab][0] += v
                out[lab][1] += 1
    return {k: tuple(v) for k, v in out.items()}


def overview(inis, topos, scenarios, fractions, spacing, run_sims):
    """Message volume and bytes per failure window, over REPEATS RNG seeds and
    every topology in `topos` (--dir).

    Every (topology, scenario, config, seed) run is independent, so they all go
    into one --jobs pool instead of the nested per-measurement pools the normal
    mode uses. No RIB dump: this mode never scores routes, and snapshotting
    dominates the runtime of the big runs."""
    plan = {}
    for i, scenario in enumerate([s for s in scenarios if s != "link"]):
        dyn, desc = scenario_dynamics(scenario, fractions, spacing)
        if scenario == "node":
            dyn += node_conditions(spacing)
            simt, desc = node_simtime(len(fractions), spacing), desc + "\n            " + grace_note()
        else:
            simt = traffic_simtime()
        # conv goes to the first scenario only; see overview_windows.
        wins = overview_windows(scenario, fractions, spacing, conv=(i == 0))
        if scenario == "node" and node_refresh(spacing) < churn_start():
            if i:
                print(f"  ! this scenario refreshes every {node_refresh(spacing):g}s, "
                      f"inside the [0,{churn_start():g})s formation window, so its "
                      f"convergence is not the one measured above")
            else:
                # It owns conv/quiet, and quiet is meant to be the hello floor.
                print(f"  ! conv/quiet come from this scenario, which refreshes "
                      f"every {node_refresh(spacing):g}s — a refresh lands inside "
                      f"the [{max(0.0, churn_start() - QUIET_LEN):g},"
                      f"{churn_start():g})s idle window, so it is NOT the hello "
                      f"floor. Run churn alongside, or raise --wave-spacing.")
        print(f"\n=== {scenario} ({simt}) ===")
        announce(scenario, opts=dyn + overrides(sim_time_limit=simt), dynamics=desc)
        print("  windows: " + ", ".join(f"{l} [{a:g},{b:g})s" for l, a, b in wins))
        plan[scenario] = (dyn, simt, wins,
                          "churn" if scenario == "churn" else "recovery")

    if not plan:
        sys.exit("--overview scores the churn and node scenarios; --only link "
                 "leaves it nothing to measure")
    tasks = [(t, s, n, r) for t in topos for s in plan
             for n in CONFIGS for r in range(H.REPEATS)]
    jobs = max(1, min(H.JOBS or os.cpu_count() or 1, len(tasks)))
    H.PROGRESS = jobs == 1   # a shared \r status line is unreadable from N runs

    def work(task):
        topo, scenario, name, rep = task
        dyn, simt, wins, net = plan[scenario]
        resdir = rep_path(os.path.join(OUT, f"res_{stem(topo)}_{name}_{scenario}"), rep)
        if run_sims:
            run_sim(sim_cmd(name, inis[(topo, CONFIGS[name][0], net)],
                            dyn, simt, resdir), allow_budget_expired=True, rep=rep)
        return tx_windows(resdir, wins)

    print(f"\n{len(tasks)} runs ({len(topos)} topologies × {len(plan)} scenarios "
          f"× {len(CONFIGS)} configs × {H.REPEATS} seeds) on {jobs} job(s)"
          + ("" if run_sims else " — post mode, re-parsing on disk"))
    # (scenario, label, config) -> [(bytes, msgs, topology, seed)], one entry per
    # run: topology and seed are both spread the boxplot draws, kept apart only
    # in the CSV.
    res = defaultdict(list)
    with ThreadPoolExecutor(max_workers=jobs) as ex:
        futs = {ex.submit(work, t): t for t in tasks}
        for i, f in enumerate(as_completed(futs), 1):
            topo, scenario, name, rep = futs[f]
            for lab, (b, m) in f.result().items():
                res[(scenario, lab, name)].append((b, m, stem(topo), rep))
            print(f"  ✓ [{i}/{len(tasks)}] {stem(topo)} {name} {scenario} "
                  f"seed {rep}", flush=True)
    if not any(res.values()):
        sys.exit("FAIL: no txBytes vectors parsed — opp_scavetool missing, or "
                 "no runs on disk for --post")
    overview_report(plan, res)
    overview_plots(plan, res)


def overview_report(plan, res):
    """Median per window over the runs, plus the spread the boxplots draw."""
    ntopo = len({v[2] for vals in res.values() for v in vals})
    print(f"\n=== overview: control-plane volume per failure window "
          f"({ntopo} topolog{'y' if ntopo == 1 else 'ies'} × {H.REPEATS} seed(s), "
          f"warmup excluded) ===")
    # One unit for the volume columns
    rows = []   # built first: the header can't print until the unit is known
    with open(os.path.join(OUT, "overview.csv"), "w", newline="") as fh:
        w = csv.writer(fh)
        w.writerow(["scenario", "window", "t0", "t1", "config", "topology",
                    "seed", "MB", "messages"])
        for scenario, (_, _, wins, _) in plan.items():
            for lab, t0, t1 in wins:
                ref = _median([v[0] for v in res.get((scenario, lab, "ls"), [])])
                for name in CONFIGS:
                    vals = res.get((scenario, lab, name), [])
                    if not vals:
                        continue
                    for b, m, topo, seed in sorted(vals, key=lambda v: (v[2], v[3])):
                        w.writerow([scenario, lab, f"{t0:g}", f"{t1:g}", name,
                                    topo, seed, f"{b / 1e6:.6f}", m])
                    b, m = _median([v[0] for v in vals]), _median([v[1] for v in vals])
                    mu, sd = _mean_sd([v[0] for v in vals])
                    # conv is the run set's, not this scenario's (see
                    # overview_windows), so it carries no scenario prefix.
                    win = (lab if lab in (scenario, "conv", "quiet")
                           else f"{scenario}/{lab}")
                    rows.append((win, name, len(vals), m, b, mu, sd,
                                 b / m if m else 0, b / (t1 - t0),
                                 b / ref if ref else 0))
    # Volume and rate scale independently
    vdiv, vname = byte_unit([v for r in rows for v in (r[4], r[5], r[6])])
    rdiv, rname = byte_unit([r[8] for r in rows])
    print(f"{'window':>10}{'config':>13}{'runs':>6}{'messages':>14}{vname:>14}"
          f"{f'mean {vname} ± sd':>22}{'B/msg':>9}{rname + '/s':>10}{'×LS':>8}")
    for win, name, n, m, b, mu, sd, per, rate, rel in rows:
        print(f"{win:>10}{name:>13}{n:>6}{m:>14,.0f}{b / vdiv:>14,.2f}"
              f"{f'{mu / vdiv:,.2f} ± {sd / vdiv:,.2f}':>22}"
              f"{per:>9.0f}{rate / rdiv:>10,.3f}{(f'{rel:.2f}×' if rel else '-'):>8}")
    print("\nmedian over every run; per-run values (topology, seed) in "
          "overview.csv, spread in the boxplots. conv is [0,--failure-start), "
          "measured once for the whole set since no scenario fails before that "
          "cut. Node windows run from one wave's failures to the next, churn's "
          "from --failure-start to the end of the run." + (" With --dir the boxes pool topologies and seeds — "
                           "slice overview.csv by topology to separate them."
                           if ntopo > 1 else ""))


def _median(v):
    return sorted(v)[len(v) // 2] if v else 0.0


def _mean_sd(v):
    """Mean and population sd, the spread the boxes draw. Runs differ in both
    topology and failure schedule, so this is one number over both."""
    if not v:
        return 0.0, 0.0
    mu = sum(v) / len(v)
    return mu, (sum((x - mu) ** 2 for x in v) / len(v)) ** 0.5


def _win_title(lab, t0, t1):
    """Plot-facing name for a measurement window."""
    if lab == "churn":
        return f"Network Churn\nover {t1 - t0:g}s"
    if lab == "conv":
        return "Initial\nConvergence"
    if lab == "quiet":
        return f"Idle Baseline\n({t1 - t0:g}s pre-failure)"
    return f"Catastrophic {lab}\nNetwork Outage"


def overview_plots(plan, res):
    try:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
    except ImportError:
        print("\n(matplotlib not available — skipping plots)")
        return
    # The figure is read away from the console, so it spells its windows out;
    # the table keeps the terse keys, whose column is 10 characters wide.
    labels = [(s, lab, t0, t1) for s, (_, _, wins, _) in plan.items()
              for lab, t0, t1 in wins]
    names = [n for n in CONFIGS if any((s, l, n) in res for s, l, _, _ in labels)]
    bdiv, bname = byte_unit([v[0] for vals in res.values() for v in vals])
    for idx, (ylabel, fname, scale) in enumerate(
            ((f"{bname} per window (log)", "overview_bytes.png", bdiv),
             ("messages per window (log)", "overview_messages.png", 1))):
        fig, ax = plt.subplots(figsize=(1.6 * len(labels) + 3, 4.5))
        w = 0.8 / len(names)
        for i, name in enumerate(names):
            data = [[v[idx] / scale for v in res.get((s, l, name), [])] or [0]
                    for s, l, _, _ in labels]
            bp = ax.boxplot(data, positions=[x + i * w - 0.4 + w / 2
                                             for x in range(len(labels))],
                            widths=w * 0.5, patch_artist=True, manage_ticks=False)
            for part in ("boxes", "medians", "whiskers", "caps", "fliers"):
                for art in bp[part]:
                    # markeredgecolor too: a flier carries its own black edge
                    # colour that set(color=) does not touch, so outliers came
                    # out black and unattributable to a config.
                    art.set(color=COLORS[name], markeredgecolor=COLORS[name])
            for box in bp["boxes"]:
                box.set(facecolor=COLORS[name], alpha=0.35)
            ax.plot([], [], color=COLORS[name], label=DISPLAY.get(name, name))
        ax.set_xticks(range(len(labels)))
        ax.set_xticklabels([_win_title(l, t0, t1) for _, l, t0, t1 in labels],
                           fontsize=8)
        ax.set_yscale("log")
        # No topology/seed count in the title: it goes in the caption, where
        # there is room to say what the spread actually spans.
        ax.set(ylabel=ylabel, xlabel="failure window",
               title=title("Control-plane volume per failure window"))
        ax.legend(fontsize=8); ax.grid(axis="y", alpha=0.3); fig.tight_layout()
        fig.savefig(os.path.join(OUT, fname), dpi=120)
        print(f"  wrote {fname}")


def _reach_total(series):
    """(sorted times, network-wide route total at each time). Samples are ragged —
    a down node stops emitting — so each node's last value is held."""
    if not series:
        return [], []
    last, ts, tot = {}, sorted({t for s in series.values() for t in s}), []
    for t in ts:
        for nid, s in series.items():
            if t in s:
                last[nid] = s[t][0]
        tot.append(sum(last.values()))
    return ts, tot


def reach_base(series, wave0):
    """Pre-wave peak of a series' network-wide total (its converged route count)."""
    ts, tot = _reach_total(series)
    return max((v for t, v in zip(ts, tot) if t < wave0), default=0)


def network_reachability(series, wave0, base=None):
    """(times, % of a reference route total present network-wide). base defaults to
    this series' own pre-wave peak; pass LS's peak (reach_base of the ls series) to
    normalise every config against the same converged optimum, so PV cannot read
    >100% merely from a slower initial fill undercounting its own base."""
    ts, tot = _reach_total(series)
    if not ts:
        return [], []
    if base is None:
        base = max((v for t, v in zip(ts, tot) if t < wave0), default=0)
    if not base:
        return [], []
    return ts, [100.0 * v / base for v in tot]


def reconverge_latencies(series, wave_t, reboots, end_t=float("inf")):
    """Per affected router: seconds from ITS OWN reboot until its route count is
    back at the count it held just BEFORE the wave, and stays there.

    Scored inside [wave_t, end_t) so escalating waves in one run stay separate.
    The pre-wave count is each router's own baseline. A cross-protocol target is
    not usable here: after the wave the rebooted routers are exactly the ones
    whose counts may still be degraded at the end of the run, in EVERY config, so
    taking one config's final counts as truth silently lowers the bar for the
    others. Returns (latencies of recovered routers, {id: % of its routes still
    missing at the end of the window} for those that did not recover)."""
    out, never = {}, {}
    for nid, ser in series.items():
        pre = [t for t in ser if t < wave_t]
        if not pre:
            continue
        tgt = ser[max(pre)][0]
        post = sorted(t for t in ser if wave_t <= t < end_t)
        if not tgt or not any(ser[t][0] < tgt for t in post):
            continue
        rec = next((t for i, t in enumerate(post)
                    if all(ser[t2][0] >= tgt for t2 in post[i:])), None)
        if rec is None:
            never[nid] = 100.0 * (tgt - ser[post[-1]][0]) / tgt
        else:
            out[nid] = max(0.0, rec - reboots.get(nid, wave_t))
    return out, never


def sizeToCost(size):
    """Ring size -> cost, mirroring MyriadGpRib::sizeToCost."""
    return size ** int(common("ringCostExponent", "2"))


def _at(ser, t):
    """The node's last stats sample at or before t (samples are ragged — a down
    node stops emitting), or None if it had none yet."""
    pre = [u for u in ser if u <= t]
    return ser[max(pre)] if pre else None


def stretch_at(series, ref_series, t):
    """Mean path-cost stretch over the whole network at time t: PV chain cost
    divided by the LS (Dijkstra) optimum for the same routes.

    Scored only on routers whose cost-basis count equals LS's live-reachable
    count at t (tuple index 3). A router still short of those routes has a
    smaller cost sum for the trivial reason that it is missing chains, which
    would read as a stretch BELOW 1 and mask exactly the degradation this
    measures — reconvergence time already reports those.

    Cost-weighted (ratio of sums, not mean of per-chain ratios): the stats dump
    carries one cost sum per router, not per chain. Long chains therefore count
    more, which is the conservative direction — they are also where a replayed
    detour costs most.

    Routers holding a chain priced at UNKNOWN_RING_SIZE are dropped too: that
    placeholder is ~10^12 per hop and drowns every real cost in the sum. It means
    a ring whose size this router has not learned yet, i.e. missing information,
    not a bad path."""
    num = den = 0
    n_nodes = 0
    unknown = sizeToCost(UNKNOWN_RING_SIZE)
    for nid, ser in series.items():
        ref = ref_series.get(nid)
        if not ref:
            continue
        a, b = _at(ser, t), _at(ref, t)
        if not a or not b or a[3] != b[3] or not b[1]:
            continue
        if a[2] >= unknown or b[2] >= unknown:
            continue
        num += a[1]
        den += b[1]
        n_nodes += 1
    return (num / den if den else None), n_nodes


def stretch_over_time(series, ref_series):
    """(times, stretch) on the sample grid, same picture as network_reachability
    but for path cost. stretch_at scores only routers whose route count matches
    LS's at that instant, so mid-wave — when nobody matches — it yields None and
    the line breaks: a gap here means "not comparable yet", not "stretch 1.0"."""
    if not series or not ref_series:
        return [], []
    ts, out = sorted({t for s in series.values() for t in s}), []
    for t in ts:
        s, _ = stretch_at(series, ref_series, t)
        out.append(s if s else float("nan"))
    return ts, out


def main(args=None):
    global DOWN_MIN, DOWN_JITTER, CHURN_NODES, CHURN_PERIOD, FAILURE_START
    if args is None:
        p = argparse.ArgumentParser()
        add_topo_args(p)
        p.add_argument("--only", choices=["churn", "link", "node"])
        p.add_argument("--churn-nodes", type=int, default=None,
                       help="nodes failing per churn period (default 1)")
        p.add_argument("--churn-period", type=float, default=None,
                       help="seconds between churn failures (default 1)")
        p.add_argument("--fractions", default=None,
                       help="node-wave failure fractions, e.g. 0.1,0.3")
        p.add_argument("--down-min", type=float, default=None,
                       help="minimum downtime in seconds (default 2)")
        p.add_argument("--down-jitter", type=float, default=None,
                       help="downtime jitter in seconds (default 8)")
        p.add_argument("--wave-spacing", type=float, default=None,
                       help="settle time between node waves, i.e. seconds from "
                            "one failure fraction to the next; also the refresh "
                            "interval of the node scenario "
                            "(default 2*(down-min+down-jitter))")
        p.add_argument("--extra-warmup", type=float, default=0.0, metavar="S",
                       help="give the network S more seconds to converge before "
                            "the first failure (adds to --failure-start, and the "
                            "run horizon follows). Raise it when the idle-baseline "
                            "window still shows formation traffic")
        p.add_argument("--failure-start", type=float, default=None,
                       help="seconds of warmup before failures begin — churn "
                            "start and first node wave alike, and the cut every "
                            f"post-warmup measurement uses (default {FAILURE_START:g})")
        p.add_argument("--noreplay", action="store_true",
                       help="also run PV with recoveryReplayEnabled=false")
        p.add_argument("--nospof", action="store_true",
                       help="also run PV with rankUseSpof=false, so its stretch "
                            "against LS is pure path cost")
        p.add_argument("--no-refresh-flood", action="store_true",
                       help="PV's routine (already-king) refresh only re-verifies "
                            "ring state, no reflood — only the 'pv' config, since "
                            "pv_noreplay needs its default (true) as its sole "
                            "propagation path without replay")
        p.add_argument("--protos", default=None,
                       help="comma list of exact config names to run (see "
                            "--list-configs); default all. Split across parallel "
                            "panes, then --post to draw the combined plots")
        p.add_argument("--list-configs", action="store_true",
                       help="print the config names the other flags select, one "
                            "per line, and exit (run_evals.sh SPLIT uses this)")
        p.add_argument("--post", action="store_true",
                       help="don't simulate; re-parse the result dirs prior runs "
                            "left and (re)draw the combined tables + plots")
        p.add_argument("--dir", default=None, metavar="PATH",
                       help="overview mode only: run every *.txt topology in "
                            "this directory instead of --topo, so the boxes "
                            "span topologies as well as RNG seeds")
        p.add_argument("--tr-extremes", action="store_true",
                       help="with --dir: run only the lowest-_tr and "
                            "highest-_tr topologies in it, every seed of each — "
                            "the two ends of a transition sweep at a fraction of "
                            "the sim cost")
        p.add_argument("--overview", action="store_true",
                       help="traffic-only mode: messages and bytes per failure "
                            "window (churn, and each node-wave fraction), "
                            "boxplotted over --repeat RNG seeds. No RIB "
                            "snapshots, so no reconvergence/stretch output")
        args = p.parse_args()
        apply_repeat_args(args)

    if not hasattr(args, "wave_spacing"):
        args.wave_spacing = None
    # A failure is only detected once neighbors miss enough hellos to declare the
    # node dead. Worst case that is a full deadInterval after the crash (the node
    # may die right after a hello), so downtime must clear it or nothing fires;
    # the half-hello on top is margin, not a second detection round. --down-min
    # overrides (e.g. to probe the sub-detection regime deliberately).
    if getattr(args, "down_min", None) is None:
        DOWN_MIN = (_sec(common("deadInterval", "40s"))
                    + _sec(common("helloInterval", "10s")) / 2)
    else:
        DOWN_MIN = args.down_min
    DOWN_JITTER = getattr(args, "down_jitter", None) or DOWN_JITTER
    CHURN_NODES = getattr(args, "churn_nodes", None) or CHURN_NODES
    CHURN_PERIOD = getattr(args, "churn_period", None) or CHURN_PERIOD
    FAILURE_START = ((getattr(args, "failure_start", None) or FAILURE_START)
                     + getattr(args, "extra_warmup", 0.0))
    if not getattr(args, "noreplay", False):
        CONFIGS.pop("pv_noreplay", None)
    if not getattr(args, "nospof", False):
        CONFIGS.pop("pv_nospof", None)
    if getattr(args, "no_refresh_flood", False):
        # Every PV config but pv_noreplay, whose sole propagation path without
        # replay is the periodic reflood. pv_nospof differs from pv in ranking
        # alone, so it has to share pv's refresh regime or the two-series
        # comparison carries two variables.
        for name, (proto, dumppar, replay, _) in list(CONFIGS.items()):
            if proto == "pv" and name != "pv_noreplay":
                CONFIGS[name] = (proto, dumppar, replay, "false")

    # PV and LS can run in separate panes (--protos pv / --protos ls), then --post
    # recombines. Post mode simulates nothing — it re-parses whatever result dirs
    # the prior runs left under the (identical) topo-scoped OUT.
    run_sims = not getattr(args, "post", False)
    if getattr(args, "protos", None) and run_sims:
        # Exact config names, so a split can put every variant in its own pane.
        # NOT prefix/family matching: "pv" is itself a config name, so "pv" also
        # selecting pv_nospof would make two panes write one result dir.
        want = {p.strip() for p in args.protos.split(",")}
        if not want <= set(CONFIGS):
            sys.exit(f"--protos {args.protos}: unknown config "
                     f"{sorted(want - set(CONFIGS))} — these flags select "
                     f"{sorted(CONFIGS)} (see --list-configs)")
        for k in [k for k in CONFIGS if k not in want]:
            CONFIGS.pop(k)

    # Names the flags above resolved to, for run_evals.sh's SPLIT fan-out.
    if getattr(args, "list_configs", False):
        print("\n".join(CONFIGS))
        return

    fractions = ([float(x) for x in args.fractions.split(",")]
                 if getattr(args, "fractions", None) else DEFAULT_FRACTIONS)
    scenarios = [args.only] if getattr(args, "only", None) else ["churn", "link", "node"]

    # Scope the output dir to topology + scenario: several eval_dynamics runs
    # execute in parallel (run_evals.sh), and a shared dir means they overwrite
    # each other's inis and .vec/.sca mid-write — opp_scavetool then dies on a
    # truncated vector file.
    global OUT
    # A --dir sweep is one measurement over many topologies, so it lands in one
    # dir named after the topology set; result dirs inside carry the topo stem.
    topos = topo_set(args)
    # …and to the mode: overview runs write the same res_* dirs but WITHOUT the
    # RIB snapshots the normal mode's tables read, so the two must not share a dir.
    OUT = os.path.join(OUT, (os.path.basename(str(args.dir).rstrip("/"))
                             if getattr(args, "dir", None) else stem(args.topo))
                       + (f"_{args.only}" if getattr(args, "only", None) else "")
                       + ("_overview" if getattr(args, "overview", False) else "")
                       + (f"_{args.tag}" if getattr(args, "tag", None) else ""))
    os.makedirs(OUT, exist_ok=True)
    if not os.path.exists(BIN):
        sys.exit(f"RoutingSim binary not found at {BIN} — build it first.")

    inis = {}
    need_protos = {CONFIGS[n][0] for n in CONFIGS}  # "pv"/"ls" bases actually used
    for topo in topos if run_sims else []:
        labels = ensure_ring_labels(topo, args.basis,
                                    getattr(args, 'max_ring_size', 0),
                                    not getattr(args, 'no_echo_rings', False))
        for proto in [p for p in ("pv", "ls") if p in need_protos]:
            for net, key in ((None, "flat"), (RECOVERY_NETWORK, "recovery"),
                             (CHURN_NETWORK, "churn")):
                inis[(topo, proto, key)] = write_proto_ini(
                    os.path.join(OUT, f"{stem(topo)}_{proto}_{key}.ini"),
                    proto, topo, labels,
                    update_jitter=args.update_jitter if proto == "pv" else None,
                    **({"network": net} if net else {}))

    traffic, series0, bytes0, churn_bytes = {}, {}, {}, {}
    churn_series = {}
    reconv = {name: {} for name in CONFIGS}
    # Waves must be spaced so that a failed node's downtime AND its refill both
    # finish before the next wave.
    spacing = args.wave_spacing or round(2 * (DOWN_MIN + DOWN_JITTER))
    waves = []

    if getattr(args, "overview", False):
        return overview(inis, topos, scenarios, fractions, spacing, run_sims)

    # --- traffic scenarios ---
    for scenario in [s for s in scenarios if s != "node"]:
        dyn, desc = scenario_dynamics(scenario)
        print(f"\n=== {scenario} ({traffic_simtime()}) ===")
        announce(scenario, opts=dyn + overrides(sim_time_limit=traffic_simtime()),
                 dynamics=desc)
        net = "churn" if scenario == "churn" else "flat"
        # Under churn the stretch configs run too, so steady-state path quality
        # is scored on the same runs as the byte rates; elsewhere only the two
        # production protocols carry traffic totals.
        want = ["pv", "ls"] + ([n for n in ("pv_nospof", "pv_noreplay")
                                if n in CONFIGS] if scenario == "churn" else [])
        for name in [n for n in want if n in CONFIGS]:
            print(f"  → {name.upper()} …", flush=True)
            got = {}
            r = repeat_mean(lambda rep, n=name, s=scenario, d=dyn:
                            got.setdefault(rep, measure(
                                n, inis.get((args.topo, CONFIGS[n][0], net)), s, d,
                                rep=rep, run=run_sims))[0])
            if scenario == "churn" and 0 in got:
                churn_series[name] = got[0][1]
            traffic[(name, scenario)] = r
            # The scalars are whole-run sums, so they carry the formation burst
            # — which on RGG n=100 read 6.6x LS where the steady state is 1.4x.
            # Only the .vec series has a time axis, so post-warmup bytes come
            # from there; messages have no such series and stay run-total.
            ser = bytes_over_time(os.path.join(OUT, f"res_{name}_{scenario}"))
            r["bytes_post"] = bytes_after(ser, churn_start())
            if scenario == "churn":   # same byte-rate series the churn plots use
                churn_bytes[name] = ser
            print(f"  ✓ {r['msgs']:.0f} msgs, {fmt_bytes(r['bytes'])}")

    # --- escalating node waves: ONE run per config, scored per wave ---
    if "node" in scenarios:
        waves = wave_times(fractions, spacing)
        dyn, desc = scenario_dynamics("node", fractions, spacing)
        dyn += node_conditions(spacing)
        desc += "\n            " + grace_note()
        st = node_simtime(len(fractions), spacing)
        print(f"\n=== escalating node waves ({st}) ===")
        announce("node", opts=dyn + overrides(sim_time_limit=st), dynamics=desc)

        def one(rep):
            out, sers = {}, {}
            for name in CONFIGS:
                if rep == 0:
                    print(f"  → {name} …", flush=True)
                tr, ser, reboots = measure(
                    name, inis.get((args.topo, CONFIGS[name][0], "recovery")),
                    "node", dyn, st, rep, run=run_sims)
                if rep == 0:
                    series0.setdefault(name, ser)
                    resdir = rep_path(os.path.join(OUT, f"res_{name}_node"), rep)
                    bytes0.setdefault(name, bytes_over_time(resdir))
                res = {"traffic": tr}
                for i, (frac, wt) in enumerate(waves):
                    end = waves[i + 1][1] if i + 1 < len(waves) else float("inf")
                    lat, never = reconverge_latencies(ser, wt, reboots.get(i, {}), end)
                    vals = sorted(lat.values())
                    res[frac] = {"n": len(vals) + len(never), "never": len(never),
                                 "short": (sum(never.values()) / len(never)
                                           if never else 0.0),
                                 "mean": sum(vals) / len(vals) if vals else 0.0,
                                 "median": vals[len(vals) // 2] if vals else 0.0,
                                 "max": max(vals) if vals else 0.0}
                out[name] = res
                sers[name] = ser

            # Stretch needs LS's optimum as the denominator, so it is scored
            # after every config in this repeat has run. Sampled at the END of
            # each wave window (just before the next wave), i.e. once recovery
            # has settled: what is left is the residual damage, not the dip.
            ref = sers.get("ls")
            for name in CONFIGS:
                for i, (frac, wt) in enumerate(waves):
                    end = (waves[i + 1][1] if i + 1 < len(waves)
                           else max(t for s in sers[name].values() for t in s))
                    s, n = (stretch_at(sers[name], ref, end - 1e-9)
                            if ref else (None, 0))
                    out[name][frac]["stretch"] = s
                    out[name][frac]["stretch_n"] = n
            return out

        agg = repeat_mean(one)
        ls_base = reach_base(series0.get("ls", {}), waves[0][1])
        for name in series0:                 # stacking waves must not overlap
            ts, pct = network_reachability(series0[name], waves[0][1], base=ls_base)
            for frac, wt in waves:
                at = max((p for t, p in zip(ts, pct) if t <= wt), default=100.0)
                if at < 99.9:
                    print(f"  ! {name}: the {frac:.0%} wave hit a network still at "
                          f"{at:.0f}% from the previous one — widen --wave-spacing")
        for name, r in agg.items():
            traffic[(name, "node")] = r["traffic"]
            # Same cut as churn's: drop the formation burst so the bar chart
            # shows what the failures cost, not what building the RIB cost.
            traffic[(name, "node")]["bytes_post"] = bytes_after(
                bytes0.get(name, ([], [])), waves[0][1])
            for frac, _ in waves:
                reconv[name][frac] = r[frac]
                warn = (f", {r[frac]['never']:.0f} still {r[frac]['short']:.0f}% "
                        "short when the next wave hit" if r[frac]["never"] else "")
                print(f"  ✓ {name} {frac:.0%}: {r[frac]['n']:.0f} affected, "
                      f"mean={r[frac]['mean']:.1f}{pm(r[frac]['mean'])}s, "
                      f"net(max)={r[frac]['max']:.0f}s{warn}")

    report(traffic, reconv, fractions, scenarios, spacing, churn_bytes)
    # A --protos pane holds only half the data; the PNGs overlay PV and LS, so let
    # --post (or a full run) draw them once both halves exist on disk.
    if run_sims and getattr(args, "protos", None):
        print(f"\nrun-only (--protos {args.protos}); once the other pane is done, "
              f"draw the combined plots with:\n  … dynamics --post "
              f"[same --topo/--only/--tag …]")
    else:
        plots(traffic, reconv, fractions, scenarios, series0, waves, bytes0,
              churn_bytes, churn_series)


def report(traffic, reconv, fractions, scenarios, spacing, churn_bytes=None):
    rows = [(n, s) for (n, s) in traffic if not s.startswith("node")]
    if rows:
        print(f"\n=== control-plane traffic over {traffic_simtime()}{repeats_note()} ===")
        # bytes(post) excludes the formation burst; blank where no time series
        # exists (link scenario, or opp_scavetool missing).
        # One unit for the volume columns
        srt = sorted(rows, key=lambda x: (x[1], x[0]))
        vdiv, vname = byte_unit([v for k in srt
                                 for v in (traffic[k]["bytes"],
                                           traffic[k].get("bytes_post"))])
        print(f"{'config':>12}{'scenario':>10}{'messages':>20}{vname:>18}"
              f"{vname + '(post)':>18}{'B/msg':>9}{'max B':>9}")
        for name, scenario in srt:
            r = traffic[(name, scenario)]
            m, b, bp = r["msgs"], r["bytes"], r.get("bytes_post")
            ms = f"{m:.0f}" + pm(m, ".0f")
            bs = f"{b / vdiv:.2f}" + pm(b / vdiv, ".2f")
            bps = f"{bp / vdiv:.2f}" if bp is not None else "-"
            print(f"{name:>12}{scenario:>10}{ms:>20}{bs:>18}{bps:>18}"
                  f"{(b/m if m else 0):>9.1f}{r['max_b']:>9.0f}")
        for name in ("pv", "ls"):
            base, lnk = traffic.get((name, "churn")), traffic.get((name, "link"))
            if base and lnk:
                print(f"  {name.upper()} link-churn cost (link−churn): "
                      f"+{lnk['msgs']-base['msgs']:.0f} msgs, "
                      f"+{fmt_bytes(lnk['bytes'] - base['bytes'])}")

        for name in ("pv", "ls"):
            r = traffic.get((name, "churn"))
            if not r:
                continue
            print(f"\n--- {name.upper()} churn per-type ---")
            tdiv, tname = byte_unit(list(r["per_type_bytes"].values()))
            print(f"{'type':>14}{'messages':>20}{tname:>18}{'B/msg':>9}"
                  f"{'%' + tname:>8}")
            tot = sum(r["per_type_bytes"].values()) or 1
            for t in sorted(r["counts"], key=lambda k: -r["per_type_bytes"].get(k, 0)):
                c, b = r["counts"][t], r["per_type_bytes"].get(t, 0)
                cs = f"{c:.0f}" + pm(c, ".0f")
                bs = f"{b / tdiv:.2f}" + pm(b / tdiv, ".2f")
                print(f"{t:>14}{cs:>20}{bs:>18}{(b/c if c else 0):>9.0f}"
                      f"{100*b/tot:>7.1f}%")

    if churn_bytes and any(churn_bytes.get(n, ([], []))[0] for n in COLORS):
        # Only after warmup: the formation burst is a one-off, the number that
        # matters is what the protocol costs per second to hold steady state.
        print(f"\n=== steady-state cost under churn ({CHURN_NODES} node(s) every "
              f"{CHURN_PERIOD:g}s, after t={churn_start():g}s) ===")
        means = {}
        for name in COLORS:
            ts, bps = churn_bytes.get(name, ([], []))
            v = [b for t, b in zip(ts, bps) if t >= churn_start()]
            if v:
                means[name] = (sum(v) / len(v), max(v))
        cdiv, cname = byte_unit([v for mp in means.values() for v in mp])
        print(f"{'config':>12}{f'mean {cname}/s':>14}{f'peak {cname}/s':>14}"
              f"{'vs LS':>9}")
        ref = means.get("ls", (0, 0))[0]
        for name, (mean, peak) in means.items():
            rel = f"{mean/ref:.1f}×" if ref else "-"
            print(f"{name:>12}{mean / cdiv:>14.3f}{peak / cdiv:>14.3f}{rel:>9}")

    if "node" not in scenarios:
        return
    r_s = node_refresh(spacing)
    print(f"\n=== reconvergence after a node wave (down {DOWN_MIN:g}-"
          f"{DOWN_MIN + DOWN_JITTER:g}s, first wave t={churn_start():g}s, "
          f"then every {spacing:g}s, "
          f"refresh {r_s:g}s){repeats_note()} ===")
    print(f"{'fraction':>9}{'config':>13}{'affected':>10}{'late':>6}{'late -%':>9}"
          f"{'mean':>15}{'median':>9}{'net(max)':>13}{'mean/refresh':>14}"
          f"{'stretch':>10}{'scored':>8}")
    for frac in fractions:
        for name in CONFIGS:
            r = reconv[name].get(frac)
            if r is None or not r["n"]:
                print(f"{frac:>8.0%}{name:>13}{'-':>10}{'-':>6}{'-':>9}{'-':>15}"
                      f"{'-':>9}{'-':>13}{'-':>14}{'-':>10}{'-':>8}")
                continue
            recovered = r["n"] - r["never"]
            mean = (f"{r['mean']:.1f}" + pm(r["mean"]) + "s") if recovered else "-"
            net = (f"{r['max']:.0f}" + pm(r["max"], ".0f") + "s") if recovered else "-"
            med = f"{r['median']:.0f}s" if recovered else "-"
            ratio = f"{r['mean']/r_s:.2f}×" if recovered else "-"
            late = f"{r['short']:.0f}%" if r["never"] else "-"
            st = r.get("stretch")
            sts = f"{st:.4f}" if st else "-"
            print(f"{frac:>8.0%}{name:>13}{r['n']:>10.0f}{r['never']:>6.0f}{late:>9}"
                  f"{mean:>15}{med:>9}{net:>13}{ratio:>14}{sts:>10}"
                  f"{r.get('stretch_n', 0):>8.0f}")
    print("\naffected = routers whose route count dropped below its pre-wave value; "
          "late = still below it when the next wave hit, and by how much on average "
          "(mean/median/max cover the rest). Target is each router's OWN pre-wave "
          "count.")
    print("stretch = cost(config)/cost(LS optimum) at the END of the wave window, "
          "over the `scored` routers — those whose route count matches LS's there "
          "and that price no chain at the unknown-ring-size placeholder (a short "
          "router would read <1 purely from missing chains). Rising = replay "
          "handing back worse chains than the crash took away. Compare pv_nospof "
          "(pure-cost ranking) against pv, whose spof-first ranking pays a "
          "redundancy premium on top.")


def plots(traffic, reconv, fractions, scenarios, series0, waves, bytes0=None,
          churn_bytes=None, churn_series=None):
    try:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
    except ImportError:
        print("\n(matplotlib not available — skipping plots)")
        return

    made = []
    # Bytes are post-warmup (bytes_post, from the .vec time series): the run
    # totals carry the RIB formation burst, which on a short run dwarfs what the
    # failures themselves cost. Messages have no such series, so they stay run
    # totals and say so. Every config that ran gets a bar, not just pv/ls.
    if scenarios:
        protos = [n for n in CONFIGS if any((n, s) in traffic for s in scenarios)]
        x = range(len(protos))
        tdiv, tname = byte_unit([traffic.get((n, s), {}).get("bytes_post")
                                 for n in protos for s in scenarios])
        for key, ylabel, fname, scale, note in (
                ("msgs", "sent messages", "traffic_messages.png", 1, "whole run"),
                ("bytes_post", f"bytes ({tname})", "traffic_bytes.png", tdiv,
                 "after warmup")):
            avail = [s for s in scenarios
                     if any(traffic.get((n, s), {}).get(key) is not None
                            for n in protos)]
            if not avail:
                continue
            w = 0.8 / len(avail)
            fig, ax = plt.subplots()
            for i, s in enumerate(avail):
                vals = [(traffic.get((n, s), {}).get(key) or 0) / scale
                        for n in protos]
                bars = ax.bar([p + i * w - 0.4 + w/2 for p in x], vals, w,
                              label=s, color=f"C{i}")
                ax.bar_label(bars, fmt="%.0f", fontsize=8)
            ax.set_xticks(list(x))
            ax.set_xticklabels([DISPLAY.get(p, p.upper()) for p in protos],
                               fontsize=8)
            ax.set(ylabel=ylabel,
                   title=title(f"Control plane ({traffic_simtime()}, {note})"))
            ax.legend(fontsize=8); fig.tight_layout()
            fig.savefig(os.path.join(OUT, fname), dpi=120)
            made.append(fname)

    # Steady churn: the same byte-rate series recovery_node.png draws under its
    # waves, but with a constant failure rate — a flat trace is the protocol's
    # sustained cost of keeping the topology converged, a rising one is damage
    # accumulating. Log y, since PV and LS differ by more than an order.
    if churn_bytes and any(churn_bytes.get(n, ([], []))[0] for n in COLORS):
        fig, ax = plt.subplots(figsize=(8, 4.5))
        rdiv, rname = byte_unit([b for n in COLORS
                                 for b in churn_bytes.get(n, ([], []))[1]])
        for name, color in COLORS.items():
            ts, bps = churn_bytes.get(name, ([], []))
            if not ts:
                continue
            ax.plot(ts, [b / rdiv for b in bps], color=color, lw=1.0,
                    label=DISPLAY.get(name, name))
        ax.axvline(churn_start(), color="gray", lw=0.8)
        ax.text(churn_start(), ax.get_ylim()[1], " churn starts", fontsize=8,
                color="gray", va="top")
        ax.set_yscale("log")
        ax.set(xlabel="time (s)", ylabel=f"control-plane {rname}/s (log)",
               title=title(f"Control-plane cost under steady churn "
                           f"({CHURN_NODES} node(s) every {CHURN_PERIOD:g}s)"))
        ax.legend(fontsize=8); fig.tight_layout()
        fig.savefig(os.path.join(OUT, "churn_bytes.png"), dpi=120)
        made.append("churn_bytes.png")

    if "node" in scenarios and any(reconv[n] for n in reconv):
        # Residual path quality after each wave settles: replay restores
        # reachability, not the pre-crash optimum, so a rising curve is the
        # cost of that tradeoff accumulating with failure size.
        if any(reconv.get(n, {}).get(f, {}).get("stretch") for n in COLORS
               for f in fractions):
            fig, ax = plt.subplots()
            for name, color in COLORS.items():
                fr = [f for f in sorted(reconv.get(name, {}))
                      if reconv[name][f].get("stretch")]
                if not fr:
                    continue
                ax.plot([f * 100 for f in fr],
                        [reconv[name][f]["stretch"] for f in fr],
                        "o-", color=color, label=DISPLAY.get(name, name))
            ax.axhline(1.0, ls=":", color="gray", label="LS optimum")
            ax.set(xlabel="nodes failed (%)", ylabel="mean path cost / LS optimum",
                   title=title("Path-cost stretch after recovery"))
            ax.legend(fontsize=8); fig.tight_layout()
            fig.savefig(os.path.join(OUT, "stretch_vs_fraction.png"), dpi=120)
            made.append("stretch_vs_fraction.png")

        # One run, waves growing over time: the whole network's route total as a
        # percentage of its pre-wave value, so a 5% node wave shows as a ~5% dip.
        fig, ax = plt.subplots(figsize=(3 + 1.6 * len(waves), 4.5))
        drew = False
        # pv_nospof only differs in path *cost* ranking, not in reachability —
        # it traces pv here. It earns its line on stretch_vs_fraction.png.
        pcolors = {n: c for n, c in COLORS.items() if n != "pv_nospof"}
        ls_base = reach_base(series0.get("ls", {}), waves[0][1])
        for name, color in pcolors.items():
            ts, pct = network_reachability(series0.get(name, {}), waves[0][1],
                                           base=ls_base)
            if not ts:
                continue
            ax.plot(ts, pct, color=color, lw=1.4, ls="--",
                    label=DISPLAY.get(name, name))
            drew = True
        if drew:
            dead = _sec(common("deadInterval", "9s"))
            for frac, wt in waves:
                ax.axvline(wt, color="gray", lw=0.8)
                # Every node of the wave is down for DOWN_MIN, then reboots
                # somewhere in the next DOWN_JITTER — two distinct regimes.
                ax.axvspan(wt, wt + DOWN_MIN, color="gray", alpha=0.18, lw=0)
                ax.axvspan(wt + DOWN_MIN, wt + DOWN_MIN + DOWN_JITTER,
                           color="gray", alpha=0.09, lw=0)
                # Worst case a neighbour still believes the last-rebooting node
                # is alive: its reboot deadline plus a full dead-timer.
                ax.axvline(wt + DOWN_MIN + DOWN_JITTER + dead, color="crimson",
                           ls="-.", lw=0.7, alpha=0.5)
                ax.text(wt, 101, f" {frac:.0%}", fontsize=9, color="gray")
            ax.plot([], [], color="crimson", ls="-.", lw=0.7, alpha=0.5,
                    label=f"worst-case detection (+deadInterval {dead:g}s)")
            ax.set(xlim=(0, max(ts)),
                   xlabel="time (s) — solid: wave fails, dark: all down, "
                          "light: reboot window",
                   ylabel="network-wide routes present (%)",
                   title=title("Route table refill under growing node failures"))
            handles, labels = ax.get_legend_handles_labels()

            # Right axis: network-wide control-plane bytes/s, from the per-node
            # txBytes OMNeT++ vector (bytes_over_time) — real send events, not a
            # post-hoc estimate, so the failure-wave spike is the protocol's
            # actual instantaneous cost, not an artifact of message counting.
            if bytes0 and any(bytes0.get(n, ([], []))[0] for n in COLORS):
                ax2 = ax.twinx()
                b2div, b2name = byte_unit([b for n in COLORS
                                           for b in bytes0.get(n, ([], []))[1]])
                for name, color in pcolors.items():
                    bts, bps = bytes0.get(name, ([], []))
                    if not bts:
                        continue
                    ax2.plot(bts, [b / b2div for b in bps], color=color, lw=1.0,
                             ls=":", alpha=0.95,
                             label=f"{DISPLAY.get(name, name)} {b2name}/s")
                ax2.set_yscale("log")
                ax2.set_ylabel(f"control-plane {b2name}/s (log)")
                h2, l2 = ax2.get_legend_handles_labels()
                handles, labels = handles + h2, labels + l2

            ax.legend(handles, labels, fontsize=7, loc="lower right")
            fig.tight_layout()
            fig.savefig(os.path.join(OUT, "recovery_node.png"), dpi=120)
            made.append("recovery_node.png")

    # Path cost over time under CONSTANT churn, not escalating waves: a steady
    # failure rate keeps the network intact enough that every sample is a real
    # comparison, where the 80% wave shatters the grid and the mean stops meaning
    # anything. pv_nospof (not pv) is the config comparable to LS — production
    # spof-first ranking pays a redundancy premium that is not churn damage.
    scolors = {n: c for n, c in COLORS.items()
               if n != "ls" and (churn_series or {}).get(n)}
    if scolors and (churn_series or {}).get("ls"):
        fig, ax = plt.subplots(figsize=(8, 4.5))
        drew, allst, curves = False, [], {}
        for name, color in scolors.items():
            ts, st = stretch_over_time(churn_series[name], churn_series["ls"])
            if not ts:
                continue
            curves[name] = (ts, st)
            ax.plot(ts, st, color=color, lw=1.4, ls="--",
                    label=DISPLAY.get(name, name))
            allst += [v for v in st if v == v]
            drew = True
        if drew:
            ax.axhline(1.0, ls=":", color="gray", label="LS optimum")
            # One router replaying a wildly-priced chain drags the network mean
            # into the thousands and flattens everything else. Scale to the 99th
            # percentile and mark the clipped spikes.
            if allst:
                q = sorted(allst)[max(0, int(0.99 * len(allst)) - 1)]
                lim = max(1.05 * q, 1.1)
                if max(allst) > lim:
                    ax.set_ylim(min(allst) - 0.02, lim)
                    ax.plot([], [], marker="^", ls="", color="k", ms=5,
                            label=f"clipped (peak {max(allst):.0f}x)")
                    for name, (ts, st) in curves.items():
                        hi = [t for t, v in zip(ts, st) if v == v and v > lim]
                        ax.plot(hi, [lim] * len(hi), marker="^", ls="",
                                color=scolors[name], ms=5, clip_on=False)
            ax.axvline(churn_start(), color="gray", lw=0.8)
            ax.text(churn_start(), ax.get_ylim()[1], " churn starts", fontsize=8,
                    color="gray", va="top")
            xmax = max(t for ts, _ in curves.values() for t in ts)
            ax.set(xlim=(0, xmax),
                   xlabel="time (s)  (gaps: no router comparable)",
                   ylabel="mean path cost / LS optimum",
                   title=title(f"Path-cost stretch under steady churn "
                               f"({CHURN_NODES} node(s) every {CHURN_PERIOD:g}s)"))
            ax.legend(fontsize=7, loc="upper left")
            fig.tight_layout()
            fig.savefig(os.path.join(OUT, "stretch_churn.png"), dpi=120)
            made.append("stretch_churn.png")
        else:
            plt.close(fig)

    if made:
        print(f"\nplots -> {OUT}/{{{','.join(sorted(made))}}}")


if __name__ == "__main__":
    main()
