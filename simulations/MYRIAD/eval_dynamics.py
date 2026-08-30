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
STATS_RE = re.compile(r"^t=([0-9.]+)s?\s+(\d+)\s+n=(\d+)", re.M)
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
           "ls": ("ls", "oracleDumpFile", None, None)}

COLORS = {"gp": "C0", "gp_noreplay": "C3", "ls": "C2"}


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


def measure(name, ini, scenario, dyn, simt=None, rep=0):
    """One run: traffic totals always, route-count series when nodes fail."""
    tag = f"{name}_{scenario}"
    resdir = rep_path(os.path.join(OUT, f"res_{tag}"), rep)
    dump = rep_path(os.path.join(OUT, f"{tag}.rib"), rep)
    for stale in (dump, *glob.glob(resdir + "/*.sca")):
        if os.path.exists(stale):
            os.remove(stale)

    proto, dumppar, replay, refreshFlood = CONFIGS[name]
    cmd = [BIN, "-u", "Cmdenv", ini,
           *overrides(sim_time_limit=simt or traffic_simtime()), *dyn,
           "--result-dir", resdir]
    if scenario == "node":
        cmd += [f'--**.{dumppar}="{dump}"', '--**.snapshotFormat="stats"']
    else:
        cmd.append("--**.snapshotInterval=0s")
    if replay is not None:
        cmd.append(f"--**.recoveryReplayEnabled={replay}")
    if refreshFlood is not None:
        cmd.append(f"--**.refreshFloodsUpdate={refreshFlood}")
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
        sys.exit(f"FAIL: no .sca in {resdir}")

    series = defaultdict(dict)
    if scenario == "node" and os.path.exists(dump):
        for m in STATS_RE.finditer(open(dump).read()):
            series[int(m.group(2))][float(m.group(1))] = int(m.group(3))
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


def network_reachability(series, wave0):
    """(times, % of the pre-wave route total present network-wide). Samples are
    ragged — a down node stops emitting — so each node's last value is held."""
    if not series:
        return [], []
    last, ts, tot = {}, sorted({t for s in series.values() for t in s}), []
    for t in ts:
        for nid, s in series.items():
            if t in s:
                last[nid] = s[t]
        tot.append(sum(last.values()))
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
        tgt = ser[max(pre)]
        post = sorted(t for t in ser if wave_t <= t < end_t)
        if not tgt or not any(ser[t] < tgt for t in post):
            continue
        rec = next((t for i, t in enumerate(post)
                    if all(ser[t2] >= tgt for t2 in post[i:])), None)
        if rec is None:
            never[nid] = 100.0 * (tgt - ser[post[-1]]) / tgt
        else:
            out[nid] = max(0.0, rec - reboots.get(nid, wave_t))
    return out, never


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
        p.add_argument("--no-refresh-flood", action="store_true",
                       help="GP's routine (already-king) refresh only re-verifies "
                            "ring state, no reflood — only the 'gp' config, since "
                            "gp_noreplay needs its default (true) as its sole "
                            "propagation path without replay")
        args = p.parse_args()
        apply_repeat_args(args)

    if not hasattr(args, "wave_spacing"):
        args.wave_spacing = None
    DOWN_MIN = getattr(args, "down_min", None) or DOWN_MIN
    DOWN_JITTER = getattr(args, "down_jitter", None) or DOWN_JITTER
    CHURN_NODES = getattr(args, "churn_nodes", None) or CHURN_NODES
    CHURN_PERIOD = getattr(args, "churn_period", None) or CHURN_PERIOD
    if not getattr(args, "noreplay", False):
        CONFIGS.pop("gp_noreplay", None)
    if getattr(args, "no_refresh_flood", False):
        proto, dumppar, replay, _ = CONFIGS["gp"]
        CONFIGS["gp"] = (proto, dumppar, replay, "false")

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
    for proto in ("gp", "ls"):
        for net, key in ((None, "flat"), (RECOVERY_NETWORK, "recovery"),
                         (CHURN_NETWORK, "churn")):
            inis[(proto, key)] = write_proto_ini(
                os.path.join(OUT, f"{proto}_{key}.ini"), proto, args.topo, labels,
                update_jitter=args.update_jitter if proto == "gp" else None,
                **({"network": net} if net else {}))

    traffic, series0, bytes0, churn_bytes = {}, {}, {}, {}
    reconv = {name: {} for name in CONFIGS}
    spacing, waves = args.wave_spacing or 30.0, []

    # --- traffic scenarios ---
    for scenario in [s for s in scenarios if s != "node"]:
        dyn, desc = scenario_dynamics(scenario)
        print(f"\n=== {scenario} ({traffic_simtime()}) ===")
        announce(scenario, opts=dyn + overrides(sim_time_limit=traffic_simtime()),
                 dynamics=desc)
        net = "churn" if scenario == "churn" else "flat"
        for name in ("gp", "ls"):
            print(f"  → {name.upper()} …", flush=True)
            r = repeat_mean(lambda rep, n=name, s=scenario, d=dyn:
                            measure(n, inis[(CONFIGS[n][0], net)], s, d, rep=rep)[0])
            traffic[(name, scenario)] = r
            if scenario == "churn":   # same byte-rate series the wave plot uses
                churn_bytes[name] = bytes_over_time(
                    os.path.join(OUT, f"res_{name}_churn"))
                # The scalars are whole-run sums, so they carry the formation
                # burst — which on RGG n=100 read 6.6x LS where the steady state
                # is 1.4x. Only the .vec series has a time axis, so post-warmup
                # bytes come from there; messages have no such series and stay
                # run-total, hence reported separately below.
                r["bytes_post"] = bytes_after(churn_bytes[name], churn_start())
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
            out = {}
            for name in CONFIGS:
                if rep == 0:
                    print(f"  → {name} …", flush=True)
                tr, ser, reboots = measure(name, inis[(CONFIGS[name][0], "recovery")],
                                           "node", dyn, st, rep)
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
            return out

        agg = repeat_mean(one)
        for name in series0:                 # stacking waves must not overlap
            ts, pct = network_reachability(series0[name], waves[0][1])
            for frac, wt in waves:
                at = max((p for t, p in zip(ts, pct) if t <= wt), default=100.0)
                if at < 99.9:
                    print(f"  ! {name}: the {frac:.0%} wave hit a network still at "
                          f"{at:.0f}% from the previous one — widen --wave-spacing")
        for name, r in agg.items():
            traffic[(name, "node")] = r["traffic"]
            for frac, _ in waves:
                reconv[name][frac] = r[frac]
                warn = (f", {r[frac]['never']:.0f} still {r[frac]['short']:.0f}% "
                        "short when the next wave hit" if r[frac]["never"] else "")
                print(f"  ✓ {name} {frac:.0%}: {r[frac]['n']:.0f} affected, "
                      f"mean={r[frac]['mean']:.1f}{pm(r[frac]['mean'])}s, "
                      f"net(max)={r[frac]['max']:.0f}s{warn}")

    report(traffic, reconv, fractions, scenarios, spacing, churn_bytes)
    plots(traffic, reconv, fractions, scenarios, series0, waves, bytes0,
          churn_bytes)


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
          f"{'mean':>15}{'median':>9}{'net(max)':>13}{'mean/refresh':>14}")
    for frac in fractions:
        for name in CONFIGS:
            r = reconv[name].get(frac)
            if r is None or not r["n"]:
                print(f"{frac:>8.0%}{name:>13}{'-':>10}{'-':>6}{'-':>9}{'-':>15}"
                      f"{'-':>9}{'-':>13}{'-':>14}")
                continue
            recovered = r["n"] - r["never"]
            mean = (f"{r['mean']:.1f}" + pm(r["mean"]) + "s") if recovered else "-"
            net = (f"{r['max']:.0f}" + pm(r["max"], ".0f") + "s") if recovered else "-"
            med = f"{r['median']:.0f}s" if recovered else "-"
            ratio = f"{r['mean']/r_s:.2f}×" if recovered else "-"
            late = f"{r['short']:.0f}%" if r["never"] else "-"
            print(f"{frac:>8.0%}{name:>13}{r['n']:>10.0f}{r['never']:>6.0f}{late:>9}"
                  f"{mean:>15}{med:>9}{net:>13}{ratio:>14}")
    print("\naffected = routers whose route count dropped below its pre-wave value; "
          "late = still below it when the next wave hit, and by how much on average "
          "(mean/median/max cover the rest). Target is each router's OWN pre-wave "
          "count.")


def plots(traffic, reconv, fractions, scenarios, series0, waves, bytes0=None,
          churn_bytes=None):
    try:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
    except ImportError:
        print("\n(matplotlib not available — skipping plots)")
        return

    made = []
    tscen = [s for s in scenarios if s != "node"]
    if tscen:
        protos = ["gp", "ls"]
        x = range(len(protos))
        w = 0.8 / len(tscen)
        for key, ylabel, fname, scale in (("msgs", "sent messages", "traffic_messages.png", 1),
                                          ("bytes", "bytes (KB)", "traffic_bytes.png", 1000)):
            fig, ax = plt.subplots()
            for i, s in enumerate(tscen):
                bars = ax.bar([p + i * w - 0.4 + w/2 for p in x],
                              [traffic[(n, s)][key] / scale for n in protos], w,
                              label=s, color=f"C{i}")
                ax.bar_label(bars, fmt="%.0f", fontsize=8)
            ax.set_xticks(list(x)); ax.set_xticklabels([p.upper() for p in protos])
            ax.set(ylabel=ylabel, title=title(f"Control plane ({traffic_simtime()})"))
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

        # One run, waves growing over time: the whole network's route total as a
        # percentage of its pre-wave value, so a 5% node wave shows as a ~5% dip.
        fig, ax = plt.subplots(figsize=(3 + 1.6 * len(waves), 4.5))
        drew = False
        for name, color in COLORS.items():
            ts, pct = network_reachability(series0.get(name, {}), waves[0][1])
            if not ts:
                continue
            ax.plot(ts, pct, color=color, lw=1.2, label=name)
            drew = True
        if drew:
            for frac, wt in waves:
                ax.axvline(wt, color="gray", lw=0.8)
                ax.axvspan(wt + DOWN_MIN, wt + DOWN_MIN + DOWN_JITTER,
                           color="gray", alpha=0.12, lw=0)
                ax.text(wt, 101, f" {frac:.0%}", fontsize=9, color="gray")
            ax.set(xlim=(0, max(ts)),
                   xlabel="time (s) — solid: wave fails, shaded: reboot window",
                   ylabel="network-wide routes present (%)",
                   title=title("Route table refill under growing node failures"))
            handles, labels = ax.get_legend_handles_labels()

            # Right axis: network-wide control-plane bytes/s, from the per-node
            # txBytes OMNeT++ vector (bytes_over_time) — real send events, not a
            # post-hoc estimate, so the failure-wave spike is the protocol's
            # actual instantaneous cost, not an artifact of message counting.
            if bytes0 and any(bytes0.get(n, ([], []))[0] for n in COLORS):
                ax2 = ax.twinx()
                for name, color in COLORS.items():
                    bts, bps = bytes0.get(name, ([], []))
                    if not bts:
                        continue
                    ax2.plot(bts, bps, color=color, lw=0.8, ls=":", alpha=0.6,
                             label=f"{name} bytes/s")
                ax2.set_yscale("log")
                ax2.set_ylabel("control-plane bytes/s (log)")
                h2, l2 = ax2.get_legend_handles_labels()
                handles, labels = handles + h2, labels + l2

            ax.legend(handles, labels, fontsize=7, loc="lower right")
            fig.tight_layout()
            fig.savefig(os.path.join(OUT, "recovery_node.png"), dpi=120)
            made.append("recovery_node.png")

    if made:
        print(f"\nplots -> {OUT}/{{{','.join(sorted(made))}}}")


if __name__ == "__main__":
    main()
