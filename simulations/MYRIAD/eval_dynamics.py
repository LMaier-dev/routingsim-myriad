#!/usr/bin/env python3
"""Control-plane cost and failure response of MyriadGP vs MyriadLS.

Three scenarios, one run each per protocol, every run scored on BOTH families:
traffic (messages/bytes per PDU type) and, where nodes fail, reconvergence time.

    churn       NodeChurn, a node down/up every  → overhead under a background
                --churn-period                      failure rate the network absorbs
    link        LinkFailure, repeating flaps     → overhead per unit of link churn
    node        EscalatingFailure, growing waves → reconvergence vs failure size,
                swept over --fractions, with the GP recovery-replay ablation

    python3 simulations/MYRIAD/eval_dynamics.py [--topo PATH] [--only churn|link|node]

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

csv.field_size_limit(sys.maxsize)  # a node's whole txBytes series is one CSV-R cell

from eval_harness import (BIN, MYRIAD_DIR, _sec, add_topo_args, announce,
                          apply_repeat_args, common, dynamics_args,
                          ensure_ring_labels, overrides, pm, refresh_s, rep_path,
                          repeat_mean, repeats_note, run_sim, simtime, title, warmup_s,
                          write_proto_ini)

OUT = os.path.join(str(MYRIAD_DIR), "eval", "dynamics")
RECOVERY_NETWORK = "routingsim.simulations.MYRIAD.RecoveryNetwork"
CHURN_NETWORK = "routingsim.simulations.MYRIAD.ChurnNetwork"

DEFAULT_FRACTIONS = [0.05, 0.1, 0.2, 0.4, 0.8]
CHURN_NODES, CHURN_PERIOD = 1, 1.0  # churn scenario: nodes down per period, seconds
CHURN_SETTLE = 10.0                # extra seconds after warmup before churn starts
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
# Last two: None = don't override (NED default, true). gp_noreplay is the
# refresh-bound baseline; opt in with --noreplay. It NEEDS refreshFloodsUpdate's
# default (true) — periodic reflood is its only propagation path without replay
# — so --no-refresh-flood (below) only touches the "gp" entry, never this one.
# A blanket --set refreshFloodsUpdate=... would hit both and cripple gp_noreplay.
CONFIGS = {"gp": ("gp", "ribDumpFile", "true", None),
           "gp_noreplay": ("gp", "ribDumpFile", "false", None),
           # rankUseSpof=false: pure-cost ranking, so GP targets the same optimum
           # LS computes and the stretch table isolates recovery damage from the
           # redundancy premium production ranking pays on purpose.
           "gp_nospof": ("gp", "ribDumpFile", "true", None),
           "ls": ("ls", "oracleDumpFile", None, None)}

EXTRA_OPTS = {"gp_nospof": ["--**.rankUseSpof=false"]}

COLORS = {"gp": "C0", "gp_noreplay": "C3", "gp_nospof": "C1", "ls": "C2"}


def churn_start():
    """When the first churn failure fires: warmup plus a settle window, so the
    measured steady state is churn cost, not leftover initial convergence."""
    return warmup_s() + CHURN_SETTLE


def traffic_simtime():
    return simtime(cycles=1.5)


def node_simtime(nwaves, spacing):
    return f"{(nwaves + 1) * spacing:g}s"


def wave_times(fractions, spacing):
    """Wave i at (i+1)*spacing: equally spaced from t=0, so the first window is
    as long as the rest."""
    return [(f, (i + 1) * spacing) for i, f in enumerate(fractions)]


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
    """Refresh phase half a period off the waves, so the cycle falls between the
    reboot and the next wave (the king clock re-arms from the last warmup
    collect)."""
    return overrides(refreshInterval=f"{node_refresh(spacing):g}s",
                     collectWarmupDuration=f"{spacing / 2:g}s")


def scenario_dynamics(scenario, fractions=None, spacing=None):
    """(cli options, human description) for one scenario. Periods derive from the
    central refresh interval so churn scales with --refresh."""
    if scenario == "churn":
        # Not warmup_s(): collectWarmupDuration is also when GP unblocks
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
                         waveSpacing=f"{spacing:g}s", warmup=f"{spacing:g}s",
                         downMin=f"{DOWN_MIN:g}s", downJitter=f"{DOWN_JITTER:g}s",
                         minDegree=0)


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

        proto, dumppar, replay, refreshFlood = CONFIGS[name]
        cmd = [BIN, "-u", "Cmdenv", ini,
               *overrides(sim_time_limit=simt or traffic_simtime()), *dyn,
               "--result-dir", resdir]
        # churn dumps stats too: same series, but sampled against a constant
        # failure rate instead of escalating waves — steady-state stretch.
        if scenario in ("node", "churn"):
            cmd += [f'--**.{dumppar}="{dump}"', '--**.snapshotFormat="stats"']
        else:
            cmd.append("--**.snapshotInterval=0s")
        if replay is not None:
            cmd.append(f"--**.recoveryReplayEnabled={replay}")
        if refreshFlood is not None:
            cmd.append(f"--**.refreshFloodsUpdate={refreshFlood}")
        cmd += EXTRA_OPTS.get(name, [])
        run_sim(cmd, allow_budget_expired=True, rep=rep)

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
    # and GP compare on the same table-refill basis), [1] drives stretch, [3] is
    # the origin count the cost sum ranges over (LS: live-reachable only; GP: ==
    # [0]) so stretch is scored only where both price the same set.
    series = defaultdict(dict)
    if scenario in ("node", "churn") and os.path.exists(dump):
        for m in STATS_RE.finditer(open(dump).read()):
            n_live = int(m.group(3))
            present = int(m.group(6)) if m.group(6) else n_live
            series[int(m.group(2))][float(m.group(1))] = (
                present, int(m.group(4)), int(m.group(5)), n_live)
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


def bytes_over_time(resdir, bin_s=1.0):
    """(bin start times, network-wide bytes/s in that bin), from the per-node
    txBytes OMNeT++ vector (see MyriadHelper::countSend) rather than the
    finish()-time byte scalars, which only give the run total. Every node
    records one sample per PDU sent (time, byte length); summing all nodes'
    samples into fixed windows gives the network-wide control-plane rate over
    time, so a failure wave's traffic spike is visible directly."""
    vecs = glob.glob(os.path.join(resdir, "*.vec"))
    tool = _scavetool()
    if not vecs or not tool:
        if vecs:
            print("  ! opp_scavetool not found — skipping byte-rate plot "
                  "(source omnetpp/setenv or set OMNETPP_ROOT)", flush=True)
        return [], []
    for stale in glob.glob(os.path.join(resdir, "*.vci")):
        os.remove(stale)  # reused result-dir: old index's byte offsets don't match this run's .vec
    csv_path = os.path.join(resdir, "txbytes_export.csv")
    subprocess.run([tool, "export", "-f", "name =~ txBytes",
                    "-F", "CSV-R", "-o", csv_path, vecs[0]],
                   check=True, capture_output=True, text=True)
    bins = defaultdict(float)
    with open(csv_path, newline="") as f:
        for row in csv.DictReader(f):
            if row["type"] != "vector":
                continue
            for t, v in zip(row["vectime"].split(), row["vecvalue"].split()):
                bins[int(float(t) // bin_s)] += float(v)
    os.remove(csv_path)
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
    normalise every config against the same converged optimum, so GP cannot read
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
    """Mean path-cost stretch over the whole network at time t: GP chain cost
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
    global DOWN_MIN, DOWN_JITTER, CHURN_NODES, CHURN_PERIOD
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
                       help="seconds between waves; also the refresh interval "
                            "of the node scenario (default 30)")
        p.add_argument("--noreplay", action="store_true",
                       help="also run GP with recoveryReplayEnabled=false")
        p.add_argument("--nospof", action="store_true",
                       help="also run GP with rankUseSpof=false, so its stretch "
                            "against LS is pure path cost")
        p.add_argument("--no-refresh-flood", action="store_true",
                       help="GP's routine (already-king) refresh only re-verifies "
                            "ring state, no reflood — only the 'gp' config, since "
                            "gp_noreplay needs its default (true) as its sole "
                            "propagation path without replay")
        p.add_argument("--protos", default=None,
                       help="comma list of protocol families to run: gp, ls "
                            "(default both). Split GP and LS across parallel panes, "
                            "then --post to draw the combined plots")
        p.add_argument("--post", action="store_true",
                       help="don't simulate; re-parse the result dirs prior runs "
                            "left and (re)draw the combined tables + plots")
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
    if not getattr(args, "noreplay", False):
        CONFIGS.pop("gp_noreplay", None)
    if not getattr(args, "nospof", False):
        CONFIGS.pop("gp_nospof", None)
    if getattr(args, "no_refresh_flood", False):
        # Every GP config but gp_noreplay, whose sole propagation path without
        # replay is the periodic reflood. gp_nospof differs from gp in ranking
        # alone, so it has to share gp's refresh regime or the two-series
        # comparison carries two variables.
        for name, (proto, dumppar, replay, _) in list(CONFIGS.items()):
            if proto == "gp" and name != "gp_noreplay":
                CONFIGS[name] = (proto, dumppar, replay, "false")

    # GP and LS can run in separate panes (--protos gp / --protos ls), then --post
    # recombines. Post mode simulates nothing — it re-parses whatever result dirs
    # the prior runs left under the (identical) topo-scoped OUT.
    run_sims = not getattr(args, "post", False)
    if getattr(args, "protos", None) and run_sims:
        fam = {p.strip() for p in args.protos.split(",")}
        for k in [k for k in CONFIGS if k.split("_")[0] not in fam]:
            CONFIGS.pop(k)
        if not CONFIGS:
            sys.exit(f"--protos {args.protos}: no matching config (gp, ls)")

    fractions = ([float(x) for x in args.fractions.split(",")]
                 if getattr(args, "fractions", None) else DEFAULT_FRACTIONS)
    scenarios = [args.only] if getattr(args, "only", None) else ["churn", "link", "node"]

    # Scope the output dir to topology + scenario: several eval_dynamics runs
    # execute in parallel (run_evals.sh), and a shared dir means they overwrite
    # each other's inis and .vec/.sca mid-write — opp_scavetool then dies on a
    # truncated vector file.
    global OUT
    OUT = os.path.join(OUT, os.path.splitext(os.path.basename(args.topo))[0]
                       + (f"_{args.only}" if getattr(args, "only", None) else "")
                       + (f"_{args.tag}" if getattr(args, "tag", None) else ""))
    os.makedirs(OUT, exist_ok=True)
    if not os.path.exists(BIN):
        sys.exit(f"RoutingSim binary not found at {BIN} — build it first.")

    labels = ensure_ring_labels(args.topo, args.basis,
                                getattr(args, 'max_ring_size', 0),
                                not getattr(args, 'no_echo_rings', False))
    inis = {}
    need_protos = {CONFIGS[n][0] for n in CONFIGS}  # "gp"/"ls" bases actually used
    for proto in [p for p in ("gp", "ls") if p in need_protos] if run_sims else []:
        for net, key in ((None, "flat"), (RECOVERY_NETWORK, "recovery"),
                         (CHURN_NETWORK, "churn")):
            inis[(proto, key)] = write_proto_ini(
                os.path.join(OUT, f"{proto}_{key}.ini"), proto, args.topo, labels,
                update_jitter=args.update_jitter if proto == "gp" else None,
                **({"network": net} if net else {}))

    traffic, series0, bytes0, churn_bytes = {}, {}, {}, {}
    churn_series = {}
    reconv = {name: {} for name in CONFIGS}
    # Waves must be spaced so that a failed node's downtime AND its refill both
    # finish before the next wave.
    spacing = args.wave_spacing or round(2 * (DOWN_MIN + DOWN_JITTER))
    waves = []

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
        want = ["gp", "ls"] + ([n for n in ("gp_nospof", "gp_noreplay")
                                if n in CONFIGS] if scenario == "churn" else [])
        for name in [n for n in want if n in CONFIGS]:
            print(f"  → {name.upper()} …", flush=True)
            got = {}
            r = repeat_mean(lambda rep, n=name, s=scenario, d=dyn:
                            got.setdefault(rep, measure(
                                n, inis.get((CONFIGS[n][0], net)), s, d,
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
            print(f"  ✓ {r['msgs']:.0f} msgs, {r['bytes']/1000:.0f} KB")

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
                tr, ser, reboots = measure(name, inis.get((CONFIGS[name][0], "recovery")),
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
    # A --protos pane holds only half the data; the PNGs overlay GP and LS, so let
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
        print(f"{'config':>12}{'scenario':>10}{'messages':>20}{'bytes':>22}"
              f"{'bytes(post)':>22}{'B/msg':>9}{'max B':>9}")
        for name, scenario in sorted(rows, key=lambda x: (x[1], x[0])):
            r = traffic[(name, scenario)]
            m, b, bp = r["msgs"], r["bytes"], r.get("bytes_post")
            ms, bs = f"{m:.0f}" + pm(m, ".0f"), f"{b:.0f}" + pm(b, ".0f")
            bps = f"{bp:.0f}" if bp is not None else "-"
            print(f"{name:>12}{scenario:>10}{ms:>20}{bs:>22}{bps:>22}"
                  f"{(b/m if m else 0):>9.1f}{r['max_b']:>9.0f}")
        for name in ("gp", "ls"):
            base, lnk = traffic.get((name, "churn")), traffic.get((name, "link"))
            if base and lnk:
                print(f"  {name.upper()} link-churn cost (link−churn): "
                      f"+{lnk['msgs']-base['msgs']:.0f} msgs, "
                      f"+{lnk['bytes']-base['bytes']:.0f} bytes")

        for name in ("gp", "ls"):
            r = traffic.get((name, "churn"))
            if not r:
                continue
            print(f"\n--- {name.upper()} churn per-type ---")
            print(f"{'type':>14}{'messages':>20}{'bytes':>22}{'B/msg':>9}{'%bytes':>8}")
            tot = sum(r["per_type_bytes"].values()) or 1
            for t in sorted(r["counts"], key=lambda k: -r["per_type_bytes"].get(k, 0)):
                c, b = r["counts"][t], r["per_type_bytes"].get(t, 0)
                cs, bs = f"{c:.0f}" + pm(c, ".0f"), f"{b:.0f}" + pm(b, ".0f")
                print(f"{t:>14}{cs:>20}{bs:>22}{(b/c if c else 0):>9.0f}"
                      f"{100*b/tot:>7.1f}%")

    if churn_bytes and any(churn_bytes.get(n, ([], []))[0] for n in COLORS):
        # Only after warmup: the formation burst is a one-off, the number that
        # matters is what the protocol costs per second to hold steady state.
        print(f"\n=== steady-state cost under churn ({CHURN_NODES} node(s) every "
              f"{CHURN_PERIOD:g}s, after t={churn_start():g}s) ===")
        print(f"{'config':>12}{'mean B/s':>14}{'peak B/s':>14}{'vs LS':>9}")
        means = {}
        for name in COLORS:
            ts, bps = churn_bytes.get(name, ([], []))
            v = [b for t, b in zip(ts, bps) if t >= churn_start()]
            if v:
                means[name] = (sum(v) / len(v), max(v))
        ref = means.get("ls", (0, 0))[0]
        for name, (mean, peak) in means.items():
            rel = f"{mean/ref:.1f}×" if ref else "-"
            print(f"{name:>12}{mean:>14.0f}{peak:>14.0f}{rel:>9}")

    if "node" not in scenarios:
        return
    r_s = node_refresh(spacing)
    print(f"\n=== reconvergence after a node wave (down {DOWN_MIN:g}-"
          f"{DOWN_MIN + DOWN_JITTER:g}s, waves every {spacing:g}s, "
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
          "handing back worse chains than the crash took away. Compare gp_nospof "
          "(pure-cost ranking) against gp, whose spof-first ranking pays a "
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
    # totals and say so. Every config that ran gets a bar, not just gp/ls.
    if scenarios:
        protos = [n for n in CONFIGS if any((n, s) in traffic for s in scenarios)]
        x = range(len(protos))
        for key, ylabel, fname, scale, note in (
                ("msgs", "sent messages", "traffic_messages.png", 1, "whole run"),
                ("bytes_post", "bytes (KB)", "traffic_bytes.png", 1000,
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
            ax.set_xticklabels([p.upper() for p in protos], fontsize=8)
            ax.set(ylabel=ylabel,
                   title=title(f"Control plane ({traffic_simtime()}, {note})"))
            ax.legend(fontsize=8); fig.tight_layout()
            fig.savefig(os.path.join(OUT, fname), dpi=120)
            made.append(fname)

    # Steady churn: the same byte-rate series recovery_node.png draws under its
    # waves, but with a constant failure rate — a flat trace is the protocol's
    # sustained cost of keeping the topology converged, a rising one is damage
    # accumulating. Log y, since GP and LS differ by more than an order.
    if churn_bytes and any(churn_bytes.get(n, ([], []))[0] for n in COLORS):
        fig, ax = plt.subplots(figsize=(8, 4.5))
        for name, color in COLORS.items():
            ts, bps = churn_bytes.get(name, ([], []))
            if not ts:
                continue
            ax.plot(ts, bps, color=color, lw=1.0, label=name)
        ax.axvline(churn_start(), color="gray", lw=0.8)
        ax.text(churn_start(), ax.get_ylim()[1], " churn starts", fontsize=8,
                color="gray", va="top")
        ax.set_yscale("log")
        ax.set(xlabel="time (s)", ylabel="control-plane bytes/s (log)",
               title=title(f"Control-plane cost under steady churn "
                           f"({CHURN_NODES} node(s) every {CHURN_PERIOD:g}s)"))
        ax.legend(fontsize=8); fig.tight_layout()
        fig.savefig(os.path.join(OUT, "churn_bytes.png"), dpi=120)
        made.append("churn_bytes.png")

    if "node" in scenarios and any(reconv[n] for n in reconv):
        fig, ax = plt.subplots()
        for name, color in COLORS.items():
            fr = sorted(f for f in reconv.get(name, {}) if reconv[name][f]["n"])
            if not fr:
                continue
            ax.plot([f * 100 for f in fr], [reconv[name][f]["max"] for f in fr],
                    "o-", color=color, label=f"{name} network-wide (max)")
            ax.plot([f * 100 for f in fr], [reconv[name][f]["mean"] for f in fr],
                    "s--", color=color, alpha=0.6, label=f"{name} mean")
        ax.axhline(refresh_s(), ls=":", color="gray",
                   label=f"refreshInterval ({common('refreshInterval')})")
        ax.set(xlabel="nodes failed (%)", ylabel="reconvergence time (s)",
               title=title("Reconvergence vs failure size"))
        ax.legend(fontsize=7); fig.tight_layout()
        fig.savefig(os.path.join(OUT, "reconverge_vs_fraction.png"), dpi=120)
        made.append("reconverge_vs_fraction.png")

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
                        "o-", color=color, label=name)
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
        # gp_nospof only differs in path *cost* ranking, not in reachability —
        # it traces gp here. It earns its line on stretch_vs_fraction.png.
        pcolors = {n: c for n, c in COLORS.items() if n != "gp_nospof"}
        ls_base = reach_base(series0.get("ls", {}), waves[0][1])
        for name, color in pcolors.items():
            ts, pct = network_reachability(series0.get(name, {}), waves[0][1],
                                           base=ls_base)
            if not ts:
                continue
            ax.plot(ts, pct, color=color, lw=1.4, ls="--", label=name)
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
                for name, color in pcolors.items():
                    bts, bps = bytes0.get(name, ([], []))
                    if not bts:
                        continue
                    ax2.plot(bts, bps, color=color, lw=1.0, ls=":", alpha=0.95,
                             label=f"{name} bytes/s")
                ax2.set_yscale("log")
                ax2.set_ylabel("control-plane bytes/s (log)")
                h2, l2 = ax2.get_legend_handles_labels()
                handles, labels = handles + h2, labels + l2

            ax.legend(handles, labels, fontsize=7, loc="lower right")
            fig.tight_layout()
            fig.savefig(os.path.join(OUT, "recovery_node.png"), dpi=120)
            made.append("recovery_node.png")

    # Path cost over time under CONSTANT churn, not escalating waves: a steady
    # failure rate keeps the network intact enough that every sample is a real
    # comparison, where the 80% wave shatters the grid and the mean stops meaning
    # anything. gp_nospof (not gp) is the config comparable to LS — production
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
            ax.plot(ts, st, color=color, lw=1.4, ls="--", label=name)
            allst += [v for v in st if v == v]
            drew = True
        if drew:
            ax.axhline(1.0, ls=":", color="gray", label="ls optimum")
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
