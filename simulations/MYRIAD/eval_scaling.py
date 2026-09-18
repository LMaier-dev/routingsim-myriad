#!/usr/bin/env python3
"""Compare MyriadLS (link-state) vs MyriadGP (path-vector) message counts as
grid topologies scale:

  square : 2x2, 3x3, ... 10x10   (both dimensions grow -> dense 2D ring-mesh)

For each topology it generates ring labels, writes a self-contained .ini per
(protocol, topology), runs Cmdenv, sums the per-type `sent_total` scalars, and
(for sweeps) plots LS vs PV.

Everything generated lands in eval/<basis>_cycles/ (git-ignored). Nothing touches
the tracked myriad{gp,ls}.ini. Run from anywhere:

    # full grid sweeps + plots (minimum_cycle_basis rings)
    python3 simulations/MYRIAD/eval_scaling.py
    python3 simulations/MYRIAD/eval_scaling.py --quick        # small smoke test

    # one topology, counts only (no sweep/plot) — pass a name under
    # simulations/topologies/ or an absolute path:
    python3 simulations/MYRIAD/eval_scaling.py --single powerlaw_cluster_graph_n100_m3_p0.5_seed0.txt
    python3 simulations/MYRIAD/eval_scaling.py --single powerlaw_cluster_graph_n200_m3_p0.5_seed0.txt

    # ring basis: --basis min (minimum_cycle_basis, default) or tree (cycle_basis).
    # minimum_cycle_basis is ~O(dim^3) in the cycle-space dimension (E-V+C), so it
    # is only feasible up to dim~400 (e.g. n200 m3 ~47s; n500 m3 / n1000+ too slow).
    # For large graphs (e.g. the full ASgraph-2024, dim~98k) use --basis tree, which
    # is linear and tractable:
    python3 simulations/MYRIAD/eval_scaling.py --single ASgraph-2024-10-09-1200.txt --basis tree
"""
import argparse
import glob
import os
import re
import subprocess
import sys
from concurrent.futures import ThreadPoolExecutor

import eval_harness as harness
from eval_harness import (BIN, SIM_ROOT, SRC, REPO, MYRIAD_DIR, add_repeat_args,
                          announce, apply_repeat_args, byte_unit, common,
                          ensure_ring_labels, fmt_bytes,
                          NED_MODULE, PROTO_COLOR, overrides, pm, pyplot,
                          rep_path, repeat_mean, repeats_note, run_sim, simtime,
                          warmup_s)

HERE = str(MYRIAD_DIR)
GRAPHGEN = str(REPO / "scripts" / "graphgen.py")

EVAL_DIR = os.path.join(HERE, "eval")
# TOPO_DIR/INI_DIR/PLOT_DIR are rebound under a per-basis subfolder in main()
# (min_cycles / tree_cycles) so the two ring bases don't share cached labels.
TOPO_DIR = os.path.join(EVAL_DIR, "topologies")
INI_DIR = os.path.join(EVAL_DIR, "ini")
PLOT_DIR = os.path.join(EVAL_DIR, "plots")
BASIS = "min"   # ring basis for topo2rings; set from --basis in main()
# Drop rings above this many members (0 = keep all); set from --max-ring-size.
MAX_RING_SIZE = 0
CONV_SNAP = "0.1s"
CONV_SIMTIME = simtime(cycles=1)

# Canonical fair-timing knobs live in eval_common.ini (single source of truth,
# shared with test_rib_batching.py). Each generated per-run ini `include`s it;
# only per-run knobs (topology, ring labels, holdInterval) are written here.
COMMON_INI = os.path.join(HERE, "eval_common.ini")

# File-name keys -> what to call them on a chart axis. ringpipe's names are terse
# so they glob well; a reader of the plot wants words.
XNAME = {"rl": "length", "len": "length", "A": "#rings", "fan": "fan",
         "share": "share", "n": "nodes", "spof": "spof",
         "tr": "transitions (ring pairs)"}

SENT_TOTAL_RE = re.compile(r"^scalar\s+\S+\s+sent_total\s+([0-9.]+)", re.M)
# bytes_total only. The old `bytes_(\S+)` also matched the per-type bytes_<type>
# (which bytes_total already sums) and the bytes_max_<type> largest-PDU gauges,
# so every byte figure came out roughly doubled.
BYTES_RE = re.compile(r"^scalar\s+\S+\s+bytes_total\s+([0-9.]+)", re.M)


def _sum_scalar(scas, name):
    """Sum a named scalar across all .sca files (0 if absent)."""
    rx = re.compile(rf"^scalar\s+\S+\s+{re.escape(name)}\s+([0-9.]+)", re.M)
    return sum(int(float(m.group(1))) for sca in scas
               for m in rx.finditer(open(sca).read()))


def _max_scalar(scas, name):
    """Max of a named scalar across all .sca files (0 if absent)."""
    vals = [int(float(m.group(1)))
            for sca in scas
            for m in re.finditer(rf"^scalar\s+\S+\s+{re.escape(name)}\s+([0-9.]+)",
                                 open(sca).read(), re.M)]
    return max(vals) if vals else 0


def parse_stats(path):
    """Stats-mode dump -> {t(float): {node: (n, cost_sum, cost_max)}}. One line
    per node per snapshot: `t=T <node> n=N cost_sum=X cost_max=Y`. Full per-route
    lines (finish()'s final dump, which carry `origin=`) are skipped."""
    snaps = {}
    for line in open(path):
        line = line.strip()
        if not line.startswith("t=") or "origin=" in line:
            continue
        t_tok, rest = line.split(" ", 1)
        toks = rest.split()
        node = toks[0]
        kv = dict(p.split("=", 1) for p in toks[1:] if "=" in p)
        snaps.setdefault(float(t_tok[2:]), {})[node] = (
            int(kv["n"]), int(kv["cost_sum"]), int(kv["cost_max"]))
    return snaps


def convergence_time(snaps):
    """Last t at which the (aggregate) RIB changed -> first stable moment."""
    ts = sorted(snaps)
    last_change = ts[0] if ts else None
    for i in range(1, len(ts)):
        if snaps[ts[i]] != snaps[ts[i - 1]]:
            last_change = ts[i]
    return last_change


def run(cmd, **kw):
    return subprocess.run(cmd, capture_output=True, text=True, **kw)


def gen_topology(dimx, dimy):
    """Generate grid_X,Y.txt in EVAL/topologies, return its path."""
    name = f"grid_{dimx},{dimy}.txt"
    path = os.path.join(TOPO_DIR, name)
    if not os.path.exists(path):
        r = run(["python3", GRAPHGEN, "-n", "-d", TOPO_DIR, "grid", f"{dimx},{dimy}"])
        if not os.path.exists(path):
            sys.exit(f"graphgen failed for {dimx}x{dimy}:\n{r.stdout}\n{r.stderr}")
    return path


def gen_flower(N):
    """Kept for the representative-topologies figure only; the flower SWEEP is
    gone. The `myriad_large` flower, generalised: one central ring of 2N routers
    plus N petal rings of N routers each. Each petal shares an ADJACENT PAIR of
    central nodes (a 2-node redundant bridge -> spof=0), the shared pairs
    disjoint and consecutive around the central ring. The ring-adjacency graph
    is a pure STAR at every N (central adjacent to all petals, petals mutually
    non-adjacent), so path diversity stays minimal; this sweep grows RING SIZE,
    not diversity. minimum_cycle_basis recovers exactly the central + N petals.
    Total = 2N (central) + N*(N-2) (petal-unique) = N^2 nodes. Writes
    flower_N.txt, returns its path. (Same star SHAPE as the 50-node myriad_large,
    but that one uses size-2N petals; here petals are size N by request.)"""
    path = os.path.join(TOPO_DIR, f"flower_{N}.txt")
    if os.path.exists(path):
        return path
    edges = []
    # Central ring: nodes 0..2N-1 in a cycle.
    C = 2 * N
    for i in range(C):
        edges.append((i, (i + 1) % C))
    # Each petal p shares central nodes (2p, 2p+1) and adds N-2 unique nodes to
    # close an N-cycle: 2p -> u0 -> u1 -> ... -> u(N-3) -> 2p+1 (plus the shared
    # central edge 2p--2p+1 already present) = an N-node ring.
    nxt = C
    for p in range(N):
        a, b = 2 * p, 2 * p + 1          # the shared adjacent central pair
        uniq = list(range(nxt, nxt + (N - 2)))
        nxt += (N - 2)
        chain = [a] + uniq + [b]
        for x, y in zip(chain, chain[1:]):
            edges.append((x, y))
    n_nodes = nxt
    with open(path, "w") as f:
        f.write(f"# flower N={N}: central ring 2N={C} + {N} petals of {N} routers "
                f"(2-node bridges, spof=0). Ring-graph = star. {n_nodes} nodes.\n")
        f.write(f"{n_nodes}\t{len(edges)}\n")
        for a, b in edges:
            f.write(f"{a}\t{b}\n")
    return path


def ring_label_lines(topo_path):
    """The per-node ringLabel lines (inlined into the ini), generating them if
    needed. ensure_ring_labels is the shared chokepoint: it prefers a shipped
    native plan, generates via topo2rings otherwise, and validates either."""
    return harness.ring_label_lines(
        ensure_ring_labels(topo_path, BASIS, MAX_RING_SIZE))


def write_ini(proto, topo_path, ring_lines):
    """Write a self-contained .ini: include the shared eval_common.ini for the
    fair-timing knobs, then only the per-run bits (protocol, topology, inlined
    ring labels). Absolute paths so it runs from SIM_ROOT."""
    stem = os.path.splitext(os.path.basename(topo_path))[0]
    topo_path = os.path.abspath(topo_path)   # the sim runs from SIM_ROOT
    ini_path = os.path.join(INI_DIR, f"{proto}_{stem}.ini")
    proto_ned = f"routingsim.routing.MYRIAD.myriad{NED_MODULE[proto]}"
    lines = [
        f"include {COMMON_INI}",
        "[General]",
        f"ned-path={SRC};{SIM_ROOT}",
        f'**.protocols = "{proto_ned}"',
        "network = routingsim.simulations.DefaultNetwork",
        f'*.manager.topologyFile = "{topo_path}"',
        "*.mainStatistics.samplingInterval = 0.01s",
        # This script reads scalars only (sent_total / bytes_*). The vectors are
        # a per-node time series that nobody here opens and that ran the eval
        # tree to 17GB on an n=1000 sweep -- enough to fill the disk and fail
        # runs. eval_dynamics does need them, and writes its own ini.
        "**.vector-recording = false",
    ]
    lines += [ln.rstrip("\n") for ln in ring_lines]
    with open(ini_path, "w") as f:
        f.write("\n".join(lines) + "\n")
    return ini_path


def message_count(proto, ini_path, tag="", extra_args=None, rep=0):
    """Run the sim (from SIM_ROOT so relative NED lookups work) and return the
    total application-level messages sent, summed from the per-type `sent_total`
    scalars each protocol records in finish(). This counts routing messages on
    the wire — NOT the kernel's gross `created:` total, which also includes
    timers, packet wrappers, and per-send dups. `tag` disambiguates result dirs
    for the same ini run under different overrides (e.g. holdInterval); the
    trailing .sca is left in place for batch_stats to read."""
    res = rep_path(
        os.path.join(INI_DIR, f"res_{proto}{tag}_{os.path.basename(ini_path)}"), rep)
    for old in glob.glob(os.path.join(res, "*.sca")):
        os.remove(old)
    cmd = [BIN, "-u", "Cmdenv", ini_path, "--result-dir", res] + (extra_args or [])
    run_sim(cmd, cwd=SIM_ROOT, rep=rep)
    scas = glob.glob(os.path.join(res, "*.sca"))
    if not scas:
        print(f"  !! {proto} produced no .sca.")
        return None
    total = 0
    for sca in scas:
        for m in SENT_TOTAL_RE.finditer(open(sca).read()):
            total += int(float(m.group(1)))
    return total


def byte_count(proto, ini_path, tag="", rep=0):
    """Total routing bytes on the wire, from the per-type `bytes_*` scalars.
    Reads the .sca message_count already produced — no second run."""
    res = rep_path(
        os.path.join(INI_DIR, f"res_{proto}{tag}_{os.path.basename(ini_path)}"), rep)
    return sum(int(float(m.group(1)))
               for sca in glob.glob(os.path.join(res, "*.sca"))
               for m in BYTES_RE.finditer(open(sca).read()))


def _mean_scalar(scas, name):
    """Mean of a named per-node scalar across all .sca files (0 if absent)."""
    vals = [float(m.group(1))
            for sca in scas
            for m in re.finditer(rf"^scalar\s+\S+\s+{re.escape(name)}\s+([0-9.]+)",
                                 open(sca).read(), re.M)]
    return (sum(vals) / len(vals)) if vals else 0.0


def state_stats(proto, ini_path, tag="", rep=0):
    """(mean state_bytes per router, max link bytes, median link bytes) from the
    already-written .sca. state_bytes is what ONE router holds, so the mean is
    the per-router figure and mean*n the network total.

    Link bytes are per link, and the max is taken over every link in the run --
    the hotspot, which a per-node total cannot show. The median comes along
    because the max alone can't distinguish a hot network from one hot link."""
    res = rep_path(
        os.path.join(INI_DIR, f"res_{proto}{tag}_{os.path.basename(ini_path)}"), rep)
    scas = glob.glob(os.path.join(res, "*.sca"))
    return (_mean_scalar(scas, "state_bytes"),
            _max_scalar(scas, "linkbytes_max"),
            _mean_scalar(scas, "linkbytes_med"))


def norr_stats(ini_path, tag="", rep=0):
    """(total bytes, max link bytes, median link bytes) with PV's reachability
    records subtracted out, from the PV run's own .sca — no second simulation.

    MyriadGp tallies the RR bytes it puts on each link (gpRrBytes) and finish()
    writes the subtracted figures, so this is the same run's packets minus the RR
    fields -- the cost of RR AS A FIELD, which is what the series claims and all
    it claims. It is an upper bound on removing RR altogether, and a loose one:
    batches pack to maxBatchBytes (4096 in eval_common.ini), so RR also displaces
    chains and multiplies the batch laps. On ringgrow tr500/n2500 the bound reads 3.79 GB where a real
    carryRr=false run measures 2.08 GB, against 6.19 GB carrying RR.

    Zeros on a .sca written before the scalars existed, which drops the series."""
    res = rep_path(os.path.join(INI_DIR, f"res_pv{tag}_{os.path.basename(ini_path)}"), rep)
    scas = glob.glob(os.path.join(res, "*.sca"))
    return (_sum_scalar(scas, "bytes_norr_total"),
            _max_scalar(scas, "linkbytes_norr_max"),
            _mean_scalar(scas, "linkbytes_norr_med"))


def batch_stats(ini_path, tag="", rep=0):
    """Read the PV run's already-written .sca (message_count ran first) and
    return (n_batches, avg_entries, max_entries) for the inter-ring UPDATE
    batching. n_batches = batches actually built (batches_created);
    avg = batch_entries_total / batches_created (mean chains coalesced INTO one
    batch)."""
    res = rep_path(os.path.join(INI_DIR, f"res_pv{tag}_{os.path.basename(ini_path)}"), rep)
    scas = glob.glob(os.path.join(res, "*.sca"))
    if not scas:
        return 0, 0.0, 0
    created = _sum_scalar(scas, "batches_created")
    entries = _sum_scalar(scas, "batch_entries_total")
    mx = _max_scalar(scas, "batch_max_entries")
    avg = (entries / created) if created else 0.0
    return created, avg, mx


def measure(topo, nobatch=False, conv=False, norr=False):
    """LS + PV on one topology -> the result dict for one sweep point.

    norr runs a THIRD sim of the same PV ini with carryRr=false and uses its real
    numbers for the RR-excluded series, instead of subtracting RR out of the PV
    run (norr_stats). Truer — a real no-RR run also packs fuller batches, which
    the subtraction cannot see — but it is a whole extra simulation per point, so
    it stays opt-in: the 10k sweeps are 12h and memory-bound."""
    rings = ring_label_lines(topo)
    ls_ini = write_ini("ls", topo, rings)
    pv_ini = write_ini("pv", topo, rings)

    # One repeat = this topology's LS+PV pair (plus its batch/convergence
    # readouts) on one seed; repeat_mean averages the numbers across repeats.
    def one(rep):
        ls_extra = []
        pv_extra = overrides(holdInterval="0s") if nobatch else []
        if conv:
            stem = os.path.splitext(os.path.basename(topo))[0]
            ls_dump = rep_path(os.path.join(INI_DIR, f"dump_ls_{stem}.txt"), rep)
            pv_dump = rep_path(os.path.join(INI_DIR, f"dump_pv_{stem}.txt"), rep)
            for d in (ls_dump, pv_dump):   # C++ appends; clear stale prior runs
                if os.path.exists(d):
                    os.remove(d)
            conv_opts = overrides(snapshotInterval=CONV_SNAP,
                                  snapshotFormat='"stats"',
                                  sim_time_limit=CONV_SIMTIME)
            ls_extra += [f'--**.oracleDumpFile="{ls_dump}"', *conv_opts]
            pv_extra += [f'--**.ribDumpFile="{pv_dump}"', *conv_opts]
        r = {"ls": message_count("ls", ls_ini, extra_args=ls_extra or None, rep=rep),
             "pv": message_count("pv", pv_ini, extra_args=pv_extra or None, rep=rep)}
        r["ls_b"] = byte_count("ls", ls_ini, rep=rep)
        r["pv_b"] = byte_count("pv", pv_ini, rep=rep)
        if norr:
            message_count("pv", pv_ini, tag="_norr", rep=rep,
                          extra_args=(pv_extra or []) + overrides(carryRr="false"))
            r["pv_nr_b"] = byte_count("pv", pv_ini, tag="_norr", rep=rep)
            _, r["pv_nr_lmax"], r["pv_nr_lmed"] = state_stats("pv", pv_ini,
                                                              tag="_norr", rep=rep)
        else:
            r["pv_nr_b"], r["pv_nr_lmax"], r["pv_nr_lmed"] = norr_stats(pv_ini, rep=rep)
        r["nb"], r["avg_sz"], r["max_sz"] = batch_stats(pv_ini, rep=rep)
        r["ls_s"], r["ls_lmax"], r["ls_lmed"] = state_stats("ls", ls_ini, rep=rep)
        r["pv_s"], r["pv_lmax"], r["pv_lmed"] = state_stats("pv", pv_ini, rep=rep)
        if conv:
            r["ls_ct"] = (convergence_time(parse_stats(ls_dump))
                          if os.path.exists(ls_dump) else None)
            raw = (convergence_time(parse_stats(pv_dump))
                   if os.path.exists(pv_dump) else None)
            r["pv_ct"] = (raw - warmup_s()) if raw is not None else None
        return r

    return repeat_mean(one)


def _mean_over_seeds(rs):
    """Average the per-topology result dicts of one sweep point (its seeds).

    harness._agg returns a Stat -- a float carrying .sd -- so the spread across
    seeds survives into the plot as error bars. Fields are None when a run
    produced no .sca or no convergence dump; those average over whatever IS
    there, and stay None if nothing is."""
    out = {}
    for k in rs[0]:
        vals = [r[k] for r in rs if r.get(k) is not None]
        out[k] = harness._agg(vals) if vals else None
    return out


def _sd(series):
    """The .sd of each Stat in a series, or None if none of them carry one --
    matplotlib takes None for `yerr` and draws no bars."""
    sds = [getattr(v, "sd", 0.0) or 0.0 for v in series]
    return sds if any(sds) else None


def sweep(items, nobatch=False, conv=False, norr=False):
    """items: list of (label, [topo_path, ...]) -- one path per seed, averaged. Runs LS and ONE PV per topology. PV
    uses the holdInterval pinned in eval_common.ini (batching on); pass
    nobatch=True to override it to 0s — the pre-batching baseline, which is far
    slower (no coalescing) so it's opt-in, not run every sweep. Returns a dict of
    parallel series: labels, ls, pv, avg_sz, max_sz, and (if conv) ls_ct, pv_ct.

    With --jobs>1 the topologies run concurrently (each writes its own ini and
    result dir, so they don't collide); results still print in sweep order."""
    out = {k: [] for k in ("labels", "ls", "pv", "ls_b", "pv_b", "avg_sz", "max_sz",
                           "ls_s", "pv_s", "ls_lmax", "pv_lmax",
                           "ls_lmed", "pv_lmed",
                           "pv_nr_b", "pv_nr_lmax", "pv_nr_lmed")}
    # Which flavour filled the pv_nr_* series: a real carryRr=false run, or RR
    # subtracted out of the PV run. They are not the same measurement, so the
    # plot must not label them the same way.
    out["norr_real"] = norr
    if conv:
        out["ls_ct"] = []
        out["pv_ct"] = []
    # A point is a LIST of topologies (its seeds); the grid sweeps and --single
    # have only one each and pass it bare, which would otherwise iterate as
    # characters.
    items = [(lab, [p] if isinstance(p, str) else p) for lab, p in items]
    # One job per (point, seed) rather than per point: with 5 seeds and 6 points
    # that is 30 independent runs, which fills --jobs far better than 6 would.
    jobs = [(i, t) for i, (_, paths) in enumerate(items) for t in paths]
    run_one = lambda j: measure(j[1], nobatch, conv, norr)
    if (harness.JOBS or 1) > 1 and len(jobs) > 1:
        harness.PROGRESS = False    # N concurrent sims would fight over the \r line
        with ThreadPoolExecutor(max_workers=harness.JOBS) as ex:
            done = list(ex.map(run_one, jobs))
    else:
        done = [run_one(j) for j in jobs]
    per_point = [[] for _ in items]
    for (i, _), r in zip(jobs, done):
        per_point[i].append(r)
    results = [_mean_over_seeds(rs) for rs in per_point]

    for (label, paths), r in zip(items, results):
        out["labels"].append(label)
        out["ls"].append(r["ls"])
        out["pv"].append(r["pv"])
        out["ls_b"].append(r["ls_b"])
        out["pv_b"].append(r["pv_b"])
        out["avg_sz"].append(r["avg_sz"])
        out["max_sz"].append(r["max_sz"])
        for k in ("ls_s", "pv_s", "ls_lmax", "pv_lmax", "ls_lmed", "pv_lmed",
                  "pv_nr_b", "pv_nr_lmax", "pv_nr_lmed"):
            out[k].append(r[k])
        mode = "no-batch" if nobatch else "batched"
        ct_str = ""
        if conv:
            out["ls_ct"].append(r["ls_ct"])
            out["pv_ct"].append(r["pv_ct"])
            def _ct(v):
                return "None" if v is None else f"{v:.2f}{pm(v, '.2f')}s"
            ct_str = f"  LS_ct={_ct(r['ls_ct'])}  PV_ct={_ct(r['pv_ct'])}"
        # message_count returns None when a run produced no .sca
        def _n(v):
            return "None" if v is None else f"{v:.0f}{pm(v, '.0f')}"
        seeds = f" (mean of {len(paths)} seeds)" if len(paths) > 1 else ""
        # The label is two lines for the plot's tick; flatten it for the console.
        flat = label.replace("\n", " ")
        # Per value, not per column
        print(f"  {flat:<14}{seeds} LS={_n(r['ls'])}/{fmt_bytes(r['ls_b'])}  "
              f"PV({mode})={_n(r['pv'])}/{fmt_bytes(r['pv_b'])}  "
              f"batches={r['nb']:.0f} avg_sz={r['avg_sz']:.1f} "
              f"max_sz={r['max_sz']:.0f}{ct_str}")
        print(f"  {'':<14}{'':<{len(seeds)}} state/router "
              f"LS={fmt_bytes(r['ls_s'])} PV={fmt_bytes(r['pv_s'])}   "
              f"link max/med LS={fmt_bytes(r['ls_lmax'])}/{fmt_bytes(r['ls_lmed'])} "
              f"PV={fmt_bytes(r['pv_lmax'])}/{fmt_bytes(r['pv_lmed'])}")
        if r["pv_nr_b"]:
            print(f"  {'':<14}{'':<{len(seeds)}} "
                  f"{'PV no RR (own run)' if norr else 'PV minus RR      '}  "
                  f"{fmt_bytes(r['pv_nr_b'])} "
                  f"({100 * (1 - r['pv_nr_b'] / r['pv_b']):.0f}% of PV bytes are RR)"
                  f"   link max/med={fmt_bytes(r['pv_nr_lmax'])}/"
                  f"{fmt_bytes(r['pv_nr_lmed'])}")
    return out


def _regress(ax, xs, ys, color, ls="-"):
    """Least-squares line through every measurement of one series, drawn across
    the sampled range. Returns the slope and R2 for the legend, or '' when there
    is nothing to fit."""
    import numpy as np
    pts = [(x, float(y)) for x, y in zip(xs, ys) if y is not None]
    if len(pts) < 2:
        return ""
    x, y = np.array([p[0] for p in pts]), np.array([p[1] for p in pts])
    m, b = np.polyfit(x, y, 1)
    ss_res = float(((y - (m * x + b)) ** 2).sum())
    ss_tot = float(((y - y.mean()) ** 2).sum())
    lo, hi = x.min(), x.max()
    ax.plot([lo, hi], [m * lo + b, m * hi + b], ls, color=color, lw=1.2, alpha=0.8)
    r2 = 1 - ss_res / ss_tot if ss_tot else 1.0
    return f" (fit: {m:,.3g}/unit, R²={r2:.3f})"


def plot(res, title, xlabel, outfile, nobatch=False, batch_panel=True,
         state_panel=True, link_panel=True,
         xs=None, invert=False, regress=False, rings=None):
    """Message counts, optionally the batch-size panel, and a convergence panel
    when res carries ls_ct/pv_ct. `res` is the dict from sweep().

    Points measured over several seeds carry their spread in Stat.sd, drawn as
    error bars: a sweep's curve means little without a sense of how much of it is
    the draw rather than the knob.

    xs places the points at the value of the swept knob instead of at even
    intervals, so the curve's shape is the real one: sweep points are rarely
    evenly spaced (fan 3100 vs 17248, length 8 vs 32), and even spacing
    straightens curvature that is actually there. invert flips the axis for
    knobs that run backwards against what scales (ring length vs ring count).

    regress: one marker per measurement and a least-squares line instead of a
    connected curve — for a --dir sweep, where each topology is its own point and
    per-point tick labels would be dozens of overlapping strings. The axis then
    keeps its own evenly spaced numeric ticks from 0, with `rings` (the ring
    count of each measurement) read off the same positions on a top axis."""
    plt = pyplot()
    labels = res["labels"]
    if xs is None:
        xs = list(range(len(labels)))
    has_conv = "ls_ct" in res and any(v is not None for v in res["ls_ct"])
    has_state = state_panel and "ls_s" in res and any(res["ls_s"])
    # State and link load are chosen independently: an n sweep wants state next to
    # the volume panel but not link load (fan and the ring graph are pinned there,
    # so per-link load barely moves), a fan sweep wants exactly the opposite.
    has_link = link_panel and "ls_lmax" in res and any(res["ls_lmax"])
    # Batching off (--nobatch, or --set holdInterval=0s) leaves nothing coalesced,
    # so the panel would be a flat zero taking half the figure. Gate on the data,
    # not on the flag: both routes to "off" then drop it.
    batched = any(res.get("avg_sz") or ())
    batch_panel = batch_panel and batched
    # The volume panel is always drawn: message count and bytes are the headline
    # of every sweep.
    ncols = 1 + bool(batch_panel) + bool(has_conv) + bool(has_state) + bool(has_link)
    fig, axes = plt.subplots(1, ncols, figsize=(7.5 * ncols, 6), squeeze=False)
    axes = list(axes[0])
    axL = axes.pop(0)
    axR = axes.pop(0) if batch_panel else None

    def ticks(ax):
        if regress:
            # Evenly spaced numeric ticks (matplotlib's own locator), not one per
            # measurement: with a point per topology the old per-point labels
            # would be dozens of overlapping strings.
            #
            # Spanning the measurements rather than starting at 0: the swept knob
            # never goes near 0 (tr >= A-1, ring length >= 3), so the empty run-up
            # only compresses the data and leaves the #rings axis unlabelled where
            # it cannot interpolate.
            lo, hi = min(xs), max(xs)
            pad = (hi - lo) * 0.05 or max(abs(hi) * 0.05, 1)
            ax.set_xlim(lo - pad, hi + pad)
            if invert:
                ax.invert_xaxis()
            if rings:
                _rings_axis(ax)
            return
        ax.set_xticks(xs)
        # Proportional spacing can put two points close together; small type is
        # what keeps their two-line ticks from overlapping.
        ax.set_xticklabels(labels, rotation=45, ha="right", fontsize=10)
        if invert:
            ax.invert_xaxis()

    def _rings_axis(ax):
        """Ring count on a top axis, at the same tick positions as the swept
        knob. Interpolated from the measurements, so it is only labelled where
        they actually cover — extrapolated ring counts would be invented."""
        import numpy as np
        order = sorted(zip(xs, rings))
        kx = [p[0] for p in order]
        ky = [p[1] for p in order]
        top = ax.twiny()
        top.set_xlim(ax.get_xlim())
        locs = [t for t in ax.get_xticks()]
        top.set_xticks(locs)
        top.set_xticklabels([f"{np.interp(t, kx, ky):.0f}"
                             if kx[0] <= t <= kx[-1] else "" for t in locs],
                            fontsize=10)
        top.set_xlabel("#rings")

    # Read the label off the data too: --set holdInterval=0s turns batching off
    # without setting --nobatch, and the figure said "batched" over a run with
    # zero batches.
    pv_label = ("Path-Vector (PV), no batching" if (nobatch or not batched)
                else "Path-Vector (PV), batched")
    # In regress mode the trend is the fitted line, so the markers stay
    # unconnected — a line through 50 scattered topologies is noise, not shape.
    fmt = (lambda m: m if regress else m + "-")
    ms = 4 if regress else 6
    fit = ((lambda ax, ser, c, ls="-": _regress(ax, xs, ser, c, ls)) if regress
           else (lambda ax, ser, c, ls="-": ""))
    axL.errorbar(xs, res["ls"], yerr=_sd(res["ls"]), fmt=fmt("o"), capsize=3, ms=ms,
                 label="Link-State (LS)" + fit(axL, res["ls"], PROTO_COLOR["ls"]),
                 color=PROTO_COLOR["ls"])
    axL.errorbar(xs, res["pv"], yerr=_sd(res["pv"]), fmt=fmt("s"), capsize=3, ms=ms,
                 label=pv_label + fit(axL, res["pv"], PROTO_COLOR["pv"]), color=PROTO_COLOR["pv"])
    ticks(axL)
    axL.set_xlabel(xlabel); axL.set_ylabel("routing messages sent")
    axL.set_title(title); axL.grid(True, alpha=0.3)
    # Bytes share the panel on a twin axis: message count alone hides the fact
    # that PV's PDUs are far fatter, so the two curves can rank opposite ways.
    axB = axL.twinx()
    # One unit for both series
    bdiv, bname = byte_unit(list(res["ls_b"]) + list(res["pv_b"]))
    # Bytes only, no message series: the RR-excluded figure is subtracted
    # from the SAME packets (see norr_stats), so its message count is PV's
    # by construction and a second identical curve would only mislead.
    has_norr = "pv_nr_b" in res and any(res["pv_nr_b"])
    nr_lab = ("PV bytes, no RR (own run)" if res.get("norr_real")
              else "PV bytes, RR fields excluded")
    mb = lambda ser: [b / bdiv for b in ser]
    mb_sd = lambda ser: (None if _sd(ser) is None else [v / bdiv for v in _sd(ser)])
    axB.errorbar(xs, mb(res["ls_b"]), yerr=mb_sd(res["ls_b"]), fmt=fmt("o"),
                 ls="" if regress else "--", capsize=3, ms=ms, color=PROTO_COLOR["ls"],
                 alpha=0.45, label="LS bytes" + fit(axB, mb(res["ls_b"]), PROTO_COLOR["ls"], "--"))
    axB.errorbar(xs, mb(res["pv_b"]), yerr=mb_sd(res["pv_b"]), fmt=fmt("s"),
                 ls="" if regress else "--", capsize=3, ms=ms, color=PROTO_COLOR["pv"],
                 alpha=0.45, label="PV bytes" + fit(axB, mb(res["pv_b"]), PROTO_COLOR["pv"], "--"))
    if has_norr:
        axB.errorbar(xs, mb(res["pv_nr_b"]), yerr=mb_sd(res["pv_nr_b"]), fmt=fmt("D"),
                     ls="" if regress else "--", capsize=3, ms=ms,
                     color=PROTO_COLOR["pv_norr"], alpha=0.7,
                     label=nr_lab
                           + fit(axB, mb(res["pv_nr_b"]), PROTO_COLOR["pv_norr"], "--"))
    axB.set_ylabel(f"routing bytes sent ({bname}, dashed)")
    h1, l1 = axL.get_legend_handles_labels()
    h2, l2 = axB.get_legend_handles_labels()
    axL.legend(h1 + h2, l1 + l2)

    if axR is not None:
        axR.errorbar(xs, res["avg_sz"], yerr=_sd(res["avg_sz"]), fmt=fmt("s"),
                     capsize=3, ms=ms, color="tab:green",
                     label="mean entries / batch"
                           + fit(axR, res["avg_sz"], "tab:green"))
        axR.errorbar(xs, res["max_sz"], yerr=_sd(res["max_sz"]), fmt=fmt("^"),
                     ls="" if regress else "--", capsize=3, ms=ms,
                     color="tab:olive", alpha=0.7,
                     label="max entries / batch"
                           + fit(axR, res["max_sz"], "tab:olive", "--"))
        ticks(axR)
        axR.set_xlabel(xlabel); axR.set_ylabel("chains coalesced into one batch")
        axR.set_title("Inter-ring UPDATE batch size"); axR.legend()
        axR.grid(True, alpha=0.3)

    if has_conv:
        axC = axes.pop(0)
        ls_cts = [v for v in res["ls_ct"]]
        pv_cts = [v for v in res["pv_ct"]]
        axC.errorbar(xs, ls_cts, yerr=_sd(ls_cts), fmt=fmt("o"), capsize=3, ms=ms,
                     label="LS convergence time" + fit(axC, ls_cts, PROTO_COLOR["ls"]),
                     color=PROTO_COLOR["ls"])
        axC.errorbar(xs, pv_cts, yerr=_sd(pv_cts), fmt=fmt("s"), capsize=3, ms=ms,
                     label="PV convergence time" + fit(axC, pv_cts, PROTO_COLOR["pv"]),
                     color=PROTO_COLOR["pv"])
        ticks(axC)
        axC.set_xlabel(xlabel); axC.set_ylabel("convergence time (s)")
        axC.set_title("Convergence time vs network size")
        axC.legend(); axC.grid(True, alpha=0.3)

    if has_state:
        # Memory. LS holds ~one LSA per origin regardless of the ring plan, so at
        # fixed n it is a flat, high floor; PV holds chains per origin RING, so it
        # starts far below and climbs with the ring count. The crossing is the
        # result -- neither protocol dominates, and this says where the boundary is.
        axS = axes.pop(0)
        # Its own unit, not the byte panel's
        sdiv, sname = byte_unit(list(res["ls_s"]) + list(res["pv_s"]))
        mbv = lambda ser: [v / sdiv for v in ser]
        axS.errorbar(xs, mbv(res["ls_s"]), yerr=_sd(res["ls_s"]), fmt=fmt("o"),
                     capsize=3, ms=ms, color=PROTO_COLOR["ls"],
                     label="LS (LSDB)" + fit(axS, mbv(res["ls_s"]), PROTO_COLOR["ls"]))
        axS.errorbar(xs, mbv(res["pv_s"]), yerr=_sd(res["pv_s"]), fmt=fmt("s"),
                     capsize=3, ms=ms, color=PROTO_COLOR["pv"],
                     label="PV (RIB)" + fit(axS, mbv(res["pv_s"]), PROTO_COLOR["pv"]))
        ticks(axS)
        axS.set_xlabel(xlabel)
        axS.set_ylabel(f"routing state per router ({sname})")
        axS.set_title("Per-router control-plane state")
        axS.legend(); axS.grid(True, alpha=0.3)

    # Hotspotting. Bytes are per LINK: a node on many rings relays between
    # k(k-1) ring pairs, which is exactly the per-node term in `fan`, so PV's
    # busiest link should climb with the axis while LS floods per edge and
    # stays flat. The max-vs-median GAP is the hotspot; the max alone cannot
    # tell a uniformly busy network from one overloaded link.
    if has_link:
        axH = axes.pop(0)
        # RR rides chains, so it concentrates on the links chains transit: the
        # busiest link is where excluding it pays most, which the totals hide.
        nr = (("pv_nr_lmax", "pv_nr_lmed", PROTO_COLOR["pv_norr"], "D",
               "PV, no RR (own run)" if res.get("norr_real")
               else "PV, RR fields excluded"),) \
            if ("pv_nr_lmax" in res and any(res["pv_nr_lmax"])) else ()
        ldiv, lname = byte_unit([v for k in ("ls_lmax", "pv_lmax", "ls_lmed",
                                             "pv_lmed") for v in res[k]])
        lkv = lambda ser: [v / ldiv for v in ser]
        for key, med, c, mk, lab in (("ls_lmax", "ls_lmed", PROTO_COLOR["ls"], "o", "LS"),
                                     ("pv_lmax", "pv_lmed", PROTO_COLOR["pv"], "s", "PV")) + nr:
            axH.errorbar(xs, lkv(res[key]), yerr=_sd(res[key]), fmt=fmt(mk),
                         capsize=3, ms=ms, color=c,
                         label=f"{lab} busiest link" + fit(axH, lkv(res[key]), c))
            axH.errorbar(xs, lkv(res[med]), yerr=_sd(res[med]), fmt=fmt(mk),
                         ls="" if regress else "--", capsize=3, ms=ms, color=c,
                         alpha=0.45,
                         label=f"{lab} median link" + fit(axH, lkv(res[med]), c, "--"))
        ticks(axH)
        axH.set_xlabel(xlabel); axH.set_ylabel(f"bytes on one link ({lname})")
        axH.set_title("Link load: busiest vs median")
        axH.legend(); axH.grid(True, alpha=0.3)

    fig.tight_layout()
    fig.savefig(outfile, dpi=200)
    print(f"  wrote {outfile}")


def _read_edges(topo_path):
    """Return (n_nodes, [edges]) from a routingsim .txt (skip #/blank; first
    remaining line is the N M header)."""
    lines = [l for l in open(topo_path) if l.strip() and not l.startswith("#")]
    if not lines:            # header-only file, e.g. an interrupted ringpipe
        return 0, []
    n = int(lines[0].split()[0])
    edges = [tuple(map(int, l.split()[:2])) for l in lines[1:]]
    return n, edges


def draw_topologies(outfile):
    """One representative picture per topology TYPE, side by side, using EXPLICIT
    coordinates (not spring layout — that garbles structured graphs). Grids sit
    on their integer lattice; the flower on a central circle with each petal laid
    out as a closed regular polygon bulging outward. Each node is a PIE of the
    colours of every ring it belongs to (multi-colour = bridge node on 2 rings)."""
    plt = pyplot()
    from matplotlib.patches import Wedge, Circle
    import math

    def grid_pos(dimx, dimy):
        # graphgen 'grid X,Y' numbers COLUMN-major with stride X: node = col*X+row.
        # Place on the integer lattice, flip y so row 0 is on top.
        return {col * dimx + row: (col, dimx - 1 - row)
                for col in range(dimy) for row in range(dimx)}

    def flower_pos(N):
        # Central ring 2N on a circle; each petal (shared pair 2p,2p+1 + N-2 unique
        # nodes) laid out as a closed regular N-gon whose one edge is the shared
        # chord, bulging radially outward -> reads as a proper ring, not a line.
        pos = {}
        C = 2 * N
        Rc = N * 0.6
        for i in range(C):
            a = 2 * math.pi * i / C
            pos[i] = (Rc * math.cos(a), Rc * math.sin(a))
        nxt = C
        for p in range(N):
            a, b = 2 * p, 2 * p + 1
            ax_, ay_ = pos[a]; bx_, by_ = pos[b]
            mx, my = (ax_ + bx_) / 2, (ay_ + by_) / 2
            s = math.hypot(bx_ - ax_, by_ - ay_)          # chord (polygon edge)
            apo = s / (2 * math.tan(math.pi / N))          # apothem
            nrm = math.hypot(mx, my) or 1.0
            ox, oy = mx + mx / nrm * apo, my + my / nrm * apo   # polygon centre (outward)
            R = s / (2 * math.sin(math.pi / N))            # circumradius
            angA = math.atan2(ay_ - oy, ax_ - ox)
            angB = math.atan2(by_ - oy, bx_ - ox)
            short = ((angB - angA + math.pi) % (2 * math.pi)) - math.pi
            stepdir = -1 if short > 0 else 1               # unique nodes take the long (outward) arc
            uniq = list(range(nxt, nxt + (N - 2))); nxt += (N - 2)
            for k, u in enumerate(uniq, start=1):
                ang = angA + stepdir * (2 * math.pi / N) * k
                pos[u] = (ox + R * math.cos(ang), oy + R * math.sin(ang))
        return pos

    reps = [
        ("square 5x5",           gen_topology(5, 5), grid_pos(5, 5)),
        ("flower N=5",           gen_flower(5),      flower_pos(5)),
    ]

    fig, axes = plt.subplots(1, len(reps), figsize=(4.8 * len(reps), 5.2))
    for ax, (label, topo, pos) in zip(axes, reps):
        n, edges = _read_edges(topo)
        # full ring membership per node (all rings, for the pie wedges).
        # Generate if absent: --only topos runs no sims, so nothing else would.
        ini = str(ensure_ring_labels(topo, BASIS, MAX_RING_SIZE))
        ring_of = {}
        if os.path.exists(ini):
            for l in open(ini):
                mo = re.search(r'Node(\d+)\D+ringLabel\s*=\s*"([0-9,]+)"', l)
                if mo:
                    ring_of[int(mo.group(1))] = [int(x) for x in mo.group(2).split(",") if x]
        rings = sorted({r for rs in ring_of.values() for r in rs})
        cmap = plt.get_cmap("tab20")
        color = {rid: cmap(i % 20) for i, rid in enumerate(rings)}

        undirected = {frozenset((a, b)) for (a, b) in edges if a != b}
        for pair in undirected:                           # edges under the nodes
            a, b = tuple(pair) if len(pair) == 2 else (next(iter(pair)),) * 2
            if a in pos and b in pos:
                ax.plot([pos[a][0], pos[b][0]], [pos[a][1], pos[b][1]],
                        "-", color="0.55", lw=1.0, zorder=1)

        # node radius from the smallest drawn edge -> even, non-overlapping
        dmin = min((math.hypot(pos[a][0] - pos[b][0], pos[a][1] - pos[b][1])
                    for pair in undirected
                    for (a, b) in [tuple(pair)] if len(pair) == 2
                    and a in pos and b in pos), default=1.0)
        r = 0.28 * dmin
        for i in range(n):
            if i not in pos:
                continue
            x, y = pos[i]
            mine = ring_of.get(i, [])
            if not mine:
                ax.add_patch(Circle((x, y), r, facecolor="white", edgecolor="k",
                                    linewidth=0.6, zorder=2))
            else:
                step = 360.0 / len(mine)
                for j, rid in enumerate(mine):            # pie: one wedge per ring
                    ax.add_patch(Wedge((x, y), r, j * step, (j + 1) * step,
                                       facecolor=color[rid], edgecolor="k",
                                       linewidth=0.5, zorder=2))
            ax.text(x, y, str(i), ha="center", va="center", fontsize=6, zorder=3)

        xs = [pos[i][0] for i in range(n) if i in pos]
        ys = [pos[i][1] for i in range(n) if i in pos]
        pad = 2.2 * r                                     # margin so nodes aren't clipped
        ax.set_xlim(min(xs) - pad, max(xs) + pad)
        ax.set_ylim(min(ys) - pad, max(ys) + pad)
        ax.set_title(f"{label}\n{n} nodes, {len(rings)} rings")
        ax.set_aspect("equal"); ax.axis("off")
    fig.suptitle("Representative topology per sweep type "
                 "(each node a pie of its ring memberships)")
    fig.tight_layout()
    fig.savefig(outfile, dpi=200)
    print(f"  wrote {outfile}")


def main(args=None):
    # Declared up front: --conv-snapshot's help text reads CONV_SNAP, and Python
    # rejects a `global` that follows any use of the name in the same function.
    global BASIS, MAX_RING_SIZE, TOPO_DIR, INI_DIR, PLOT_DIR, CONV_SNAP
    if args is None:
        ap = argparse.ArgumentParser(description="PV-vs-LS message-count scaling eval")
        ap.add_argument("--quick", action="store_true", help="short sweeps for a smoke test")
        ap.add_argument("--only", choices=["square", "topos"],
                        help="run just one sweep instead of all four ('topos' = draw "
                             "the representative topology pictures only, no sims)")
        ap.add_argument("--no-batch", dest="nobatch", action="store_true",
                        help="run PV with holdInterval=0s (pre-batching baseline). "
                             "Far slower (no coalescing) — off by default; the normal "
                             "run uses the holdInterval pinned in eval_common.ini.")
        ap.add_argument("--convergence", action="store_true",
                        help="also measure convergence time per topology (extends "
                             f"sim-time-limit to {CONV_SIMTIME}; adds a third plot panel).")
        ap.add_argument("--conv-snapshot", dest="conv_snapshot", default=None,
                        metavar="T",
                        help=f"snapshot interval under --convergence, which sets "
                             f"the resolution of the measured convergence time "
                             f"(default {CONV_SNAP})")
        ap.add_argument("--basis", choices=["min", "tree", "short", "all"], default="min",
                        help="ring basis: min=minimum_cycle_basis (default), "
                             "tree=cycle_basis (spanning-tree; 'all' is a legacy "
                             "alias), short=greedy shortest-cycle-per-edge (small "
                             "rings bounded by --max-ring-size, no min-basis cost). "
                             "Results nest under eval/<basis>_cycles/.")
        ap.add_argument("--max-ring-size", dest="max_ring_size", type=int, default=0,
                        help="drop rings larger than this; members fall back to "
                             "echo rings (0 = keep all, the default). Long rings "
                             "dominate PV's fan-out, and --basis tree produces "
                             "very large ones. Results nest under a .mN label file.")
        ap.add_argument("--dir", metavar="DIR",
                        help="sweep every .txt in DIR instead of the built-in "
                             "grid sweeps (e.g. a ringpipe family folder under "
                             "myriad_topologies/csv/). ringpipe's own ring labels "
                             "are used when present.")
        ap.add_argument("--x-key", dest="x_key", default="fan", metavar="WORD",
                        help="with --dir: the word in the file names whose "
                             "trailing number is the x-axis (default fan; e.g. "
                             "tr = transitions, rl, len, share, n). rl/len "
                             "sweeps plot DESCENDING, "
                             "so the ring count -- which is inverse to length at "
                             "fixed n -- grows left to right")
        ap.add_argument("--no-state-panel", action="store_true",
                        help="drop the per-router state panel (RIB vs LSDB). It "
                             "scales with NODE COUNT, so it says nothing on a "
                             "sweep that holds n fixed")
        ap.add_argument("--no-link-panel", action="store_true",
                        help="drop the link-load panel. Per-link load is a "
                             "function of FAN, so it says nothing on an n sweep "
                             "where the ring graph is pinned")
        ap.add_argument("--no-batch-panel", action="store_true",
                        help="accepted for compatibility and now the only "
                             "behaviour: dir sweeps never draw the batch-size "
                             "panel. Not --nobatch, which turns PV's batching OFF")
        ap.add_argument("--norr", action="store_true",
                        help="measure the RR-excluded series with a real extra "
                             "carryRr=false run per point instead of subtracting "
                             "RR out of the PV run. Truer (a real no-RR run also "
                             "packs fuller batches) but one more simulation per "
                             "point, so it is off by default")
        ap.add_argument("--single", metavar="TOPO",
                        help="run PV+LS on ONE topology .txt and print counts (no "
                             "sweep, no plot). Path relative to simulations/topologies "
                             "or absolute.")
        ap.add_argument("--tag", default=None, metavar="NAME",
                        help="suffix the output folder, so two runs that differ "
                             "only in --set keep their own inis, result dirs and "
                             "plots instead of overwriting each other "
                             "(e.g. --no-batch --tag no-batch)")
        add_repeat_args(ap)
        args = ap.parse_args()
        apply_repeat_args(args)

    # Nest all outputs under a per-basis subfolder so min_cycles and tree_cycles
    # keep separate cached labels, inis, and plots.
    BASIS = args.basis
    if getattr(args, "conv_snapshot", None):
        v = args.conv_snapshot
        CONV_SNAP = f"{v}s" if str(v)[-1].isdigit() else str(v)
    MAX_RING_SIZE = getattr(args, "max_ring_size", 0) or 0
    basis_dir = os.path.join(EVAL_DIR, f"{BASIS}_cycles"
                             + (f"_m{MAX_RING_SIZE}" if MAX_RING_SIZE else "")
                             + (f"_{args.tag}" if getattr(args, "tag", None) else ""))
    TOPO_DIR = os.path.join(basis_dir, "topologies")
    INI_DIR = os.path.join(basis_dir, "ini")
    PLOT_DIR = os.path.join(basis_dir, "plots")

    for d in (TOPO_DIR, INI_DIR, PLOT_DIR):
        os.makedirs(d, exist_ok=True)
    # Snapshot the shared conditions beside the plots so a results bundle is
    # self-documenting and reproducible.
    import shutil
    if os.path.exists(COMMON_INI):
        shutil.copy(COMMON_INI, os.path.join(PLOT_DIR, "eval_common.ini"))
    if args.only == "topos":
        print("== drawing representative topologies ==")
        draw_topologies(os.path.join(PLOT_DIR, "topologies.png"))
        return
    if not os.path.exists(BIN):
        sys.exit(f"RoutingSim binary not found at {BIN} — build it first.")

    if getattr(args, "dir", None):
        topos = sorted(glob.glob(os.path.join(os.path.abspath(args.dir), "*.txt")))
        if not topos:
            sys.exit(f"no .txt topologies in {args.dir}")
        # An interrupted ringpipe leaves a header-only .txt behind. Drop those
        # here rather than letting topo2rings die on the null graph an hour into
        # the sweep.
        empty = [p for p in topos if not _read_edges(p)[1]]
        for p in empty:
            print(f"  ! skipping {os.path.basename(p)}: no edges "
                  "(truncated topology — rerun ringpipe or delete it)")
        topos = [p for p in topos if p not in empty]
        # Sort and label by the --x-key number in the file name (ringpipe encodes
        # share, rl, fan, n, ... there). Lexical order is wrong: "fan1000" <
        # "fan296", and "share0.1" < "share0".
        def num(p, key):
            mo = re.search(rf"_{key}([0-9]+\.?[0-9]*)", os.path.basename(p))
            return float(mo.group(1)) if mo else 0.0

        # Every topology is its own measurement: seeds of the same sweep point
        # differ in the fan/ring count they actually landed on, so averaging them
        # into one point throws away both the spread and the x-resolution the
        # scatter+regression reads the trend off.
        items, xs, rings = [], [], []
        for p in sorted(topos, key=lambda q: num(q, args.x_key)):
            rl, A = num(p, "rl"), num(p, "A")
            xs.append(num(p, args.x_key))
            rings.append(A)
            # share too: two sweep points can land on the same fan/length/ring
            # count, and without it their console lines read as duplicates.
            # ringgrow files carry no _share; a bare " share 0" there reads as a
            # setting rather than as absence.
            share = ("" if args.x_key == "share" or "_share" not in p
                     else f" share {num(p, 'share'):g}")
            items.append((f"{XNAME.get(args.x_key, args.x_key)} {xs[-1]:g} "
                          f"len {rl:.0f} #rings {A:.0f}{share} "
                          f"seed {num(p, 'seed'):.0f}", [p]))
        # A key no file carries reads as 0 everywhere and would silently plot a
        # column of points on the y axis. _tr is the usual case: it was added to
        # ringpipe's names later than the rest.
        if not any(xs):
            sys.exit(f"--x-key {args.x_key}: no _{args.x_key}<number> in these "
                     f"file names (e.g. {os.path.basename(topos[0])}) — "
                     f"regenerate the sweep with ringpipe.py, or pick a key "
                     f"they do carry")
        name = os.path.basename(os.path.normpath(args.dir))
        print(f"== dir sweep: {name} ({len(items)} topologies) ==")
        # Length sweeps read right-to-left: at a fixed n the ring COUNT is what
        # scales, and it is inverse to length (n = rings x length - surplus), so
        # the axis is inverted to put the growing quantity on the growing side.
        plot(sweep(items, args.nobatch, args.convergence, getattr(args, "norr", False)),
             f"Path-Vector vs Link-State: {name}", XNAME.get(args.x_key, args.x_key),
             os.path.join(PLOT_DIR, f"dir_{name}_{args.x_key}.png"), args.nobatch,
             # Batch size is a property of the UPDATE encoding, not of the swept
             # knob; on a dir sweep the panel competes for width with the state
             # and link-load ones, which is what this axis is actually about.
             batch_panel=False,
             state_panel=not getattr(args, "no_state_panel", False),
             link_panel=not getattr(args, "no_link_panel", False),
             xs=xs, invert=args.x_key in ("rl", "len"), regress=True,
             # ringgrow pins the ring count (A300/A1200), so the top axis would
             # read "300 300 300 ..." across the whole sweep.
             rings=rings if len(set(rings)) > 1 else None)
        return

    if args.single:
        # One topology, counts only (no sweep/plot). Resolve relative to the
        # tracked topologies dir if not an existing path as given.
        topo = args.single
        if not os.path.exists(topo):
            topo = os.path.join(SIM_ROOT, "topologies", args.single)
        if not os.path.exists(topo):
            sys.exit(f"topology not found: {args.single}")
        label = os.path.splitext(os.path.basename(topo))[0]
        print(f"== single topology ({BASIS} cycles): {label} ==")
        sweep([(label, topo)], args.nobatch, norr=getattr(args, "norr", False))
        return

    announce("scaling", opts=overrides(sim_time_limit=CONV_SIMTIME) if args.convergence else [])
    if repeats_note():
        print(f" repeats{repeats_note()}")

    if args.quick:
        square_dims = [(n, n) for n in range(3, 6)]
    else:
        square_dims = [(n, n) for n in range(3, 11)]       # 3x3 .. 10x10

    if args.only in (None, "square"):
        print("== square sweep (NxN) ==")
        s_items = [(f"{x}x{y}", gen_topology(x, y)) for (x, y) in square_dims]
        plot(sweep(s_items, args.nobatch, args.convergence, getattr(args, "norr", False)),
             "Path-Vector vs Link-State: square grid scaling (dense 2D ring-mesh)",
             "grid size", os.path.join(PLOT_DIR, "square_scaling.png"), args.nobatch,
             state_panel=False, link_panel=False)

    if args.only is None:
        print("== drawing representative topologies ==")
        draw_topologies(os.path.join(PLOT_DIR, "topologies.png"))


if __name__ == "__main__":
    main()
