#!/usr/bin/env python3
"""Compare MyriadLS (link-state) vs MyriadGP (path-vector) message counts as
grid topologies scale two ways:

  1. square   : 2x2, 3x3, ... 10x10   (both dimensions grow -> dense 2D ring-mesh)
  2. width    : 3x3, 3x4, ... 3x30    (one dimension grows -> ring-adjacency stays
                                        a narrow ladder, GP's favourable case)

For each topology it generates ring labels, writes a self-contained .ini per
(protocol, topology), runs Cmdenv, sums the per-type `sent_total` scalars, and
(for sweeps) plots LS vs GP.

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
                          announce, apply_repeat_args, common, ensure_ring_labels,
                          overrides, pm,
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
CONV_SNAP = "0.5s"      # stats snapshots are tiny; poll finely for convergence times
CONV_SIMTIME = simtime(cycles=1)

# Canonical fair-timing knobs live in eval_common.ini (single source of truth,
# shared with test_rib_batching.py). Each generated per-run ini `include`s it;
# only per-run knobs (topology, ring labels, holdInterval) are written here.
COMMON_INI = os.path.join(HERE, "eval_common.ini")

SENT_TOTAL_RE = re.compile(r"^scalar\s+\S+\s+sent_total\s+([0-9.]+)", re.M)
BYTES_RE = re.compile(r"^scalar\s+\S+\s+bytes_(\S+)\s+([0-9.]+)", re.M)


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


def gen_ladder(W):
    """2xW ladder: W columns of 2 nodes, every rung present and top/bottom rails
    present. minimum_cycle_basis decomposes it into W-1 unit-square (4-node)
    rings chained in a line, each sharing a rung (2-node bridge, spof=0) with its
    neighbours only. The ring-adjacency graph is thus a pure 1-D LINE (path of
    W-1 rings) -> zero path diversity, GP's very best case. Writes ladder2_W.txt,
    returns its path."""
    path = os.path.join(TOPO_DIR, f"ladder2_{W}.txt")
    if os.path.exists(path):
        return path
    nid = lambda r, c: r * W + c
    edges = []
    for c in range(W):                      # rungs
        edges.append((nid(0, c), nid(1, c)))
    for c in range(W - 1):                  # top + bottom rails
        edges.append((nid(0, c), nid(0, c + 1)))
        edges.append((nid(1, c), nid(1, c + 1)))
    with open(path, "w") as f:
        f.write(f"# 2x{W} ladder: {W-1} unit-square rings chained in a 1-D line\n")
        f.write("# pure 1-D ring-adjacency graph (no path diversity), GP best case\n")
        f.write(f"{2 * W}\t{len(edges)}\n")
        for a, b in edges:
            f.write(f"{a}\t{b}\n")
    return path


def gen_flower(N):
    """The `myriad_large` flower, generalised: one central ring of 2N routers
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
    proto_ned = f"routingsim.routing.MYRIAD.myriad{proto}"
    lines = [
        f"include {COMMON_INI}",
        "[General]",
        f"ned-path={SRC};{SIM_ROOT}",
        f'**.protocols = "{proto_ned}"',
        "network = routingsim.simulations.DefaultNetwork",
        f'*.manager.topologyFile = "{topo_path}"',
        "*.mainStatistics.samplingInterval = 0.01s",
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
    return sum(int(float(m.group(2)))
               for sca in glob.glob(os.path.join(res, "*.sca"))
               for m in BYTES_RE.finditer(open(sca).read()))


def batch_stats(ini_path, tag="", rep=0):
    """Read the GP run's already-written .sca (message_count ran first) and
    return (n_batches, avg_entries, max_entries) for the inter-ring UPDATE
    batching. n_batches = sum of sent_update_batch (packets on the wire);
    avg = batch_entries_total / n_batches (mean chains coalesced per packet)."""
    res = rep_path(os.path.join(INI_DIR, f"res_gp{tag}_{os.path.basename(ini_path)}"), rep)
    scas = glob.glob(os.path.join(res, "*.sca"))
    if not scas:
        return 0, 0.0, 0
    n_batches = _sum_scalar(scas, "sent_update_batch")
    entries = _sum_scalar(scas, "batch_entries_total")
    mx = _max_scalar(scas, "batch_max_entries")
    avg = (entries / n_batches) if n_batches else 0.0
    return n_batches, avg, mx


def measure(topo, nobatch=False, conv=False):
    """LS + GP on one topology -> the result dict for one sweep point."""
    rings = ring_label_lines(topo)
    ls_ini = write_ini("ls", topo, rings)
    gp_ini = write_ini("gp", topo, rings)

    # One repeat = this topology's LS+GP pair (plus its batch/convergence
    # readouts) on one seed; repeat_mean averages the numbers across repeats.
    def one(rep):
        ls_extra = []
        gp_extra = overrides(holdInterval="0s") if nobatch else []
        if conv:
            stem = os.path.splitext(os.path.basename(topo))[0]
            ls_dump = rep_path(os.path.join(INI_DIR, f"dump_ls_{stem}.txt"), rep)
            gp_dump = rep_path(os.path.join(INI_DIR, f"dump_gp_{stem}.txt"), rep)
            for d in (ls_dump, gp_dump):   # C++ appends; clear stale prior runs
                if os.path.exists(d):
                    os.remove(d)
            conv_opts = overrides(snapshotInterval=CONV_SNAP,
                                  snapshotFormat='"stats"',
                                  sim_time_limit=CONV_SIMTIME)
            ls_extra += [f'--**.oracleDumpFile="{ls_dump}"', *conv_opts]
            gp_extra += [f'--**.ribDumpFile="{gp_dump}"', *conv_opts]
        r = {"ls": message_count("ls", ls_ini, extra_args=ls_extra or None, rep=rep),
             "gp": message_count("gp", gp_ini, extra_args=gp_extra or None, rep=rep)}
        r["ls_b"] = byte_count("ls", ls_ini, rep=rep)
        r["gp_b"] = byte_count("gp", gp_ini, rep=rep)
        r["nb"], r["avg_sz"], r["max_sz"] = batch_stats(gp_ini, rep=rep)
        if conv:
            r["ls_ct"] = (convergence_time(parse_stats(ls_dump))
                          if os.path.exists(ls_dump) else None)
            raw = (convergence_time(parse_stats(gp_dump))
                   if os.path.exists(gp_dump) else None)
            r["gp_ct"] = (raw - warmup_s()) if raw is not None else None
        return r

    return repeat_mean(one)


def _mean_over_seeds(rs):
    """Average the per-topology result dicts of one sweep point (its seeds).

    Some fields are None when a run produced no .sca or no convergence dump;
    they average over whatever IS there, and stay None if nothing is."""
    out = {}
    for k in rs[0]:
        vals = [r[k] for r in rs if r.get(k) is not None]
        out[k] = (sum(vals) / len(vals)) if vals else None
    return out


def sweep(items, nobatch=False, conv=False):
    """items: list of (label, [topo_path, ...]) -- one path per seed, averaged. Runs LS and ONE GP per topology. GP
    uses the holdInterval pinned in eval_common.ini (batching on); pass
    nobatch=True to override it to 0s — the pre-batching baseline, which is far
    slower (no coalescing) so it's opt-in, not run every sweep. Returns a dict of
    parallel series: labels, ls, gp, avg_sz, max_sz, and (if conv) ls_ct, gp_ct.

    With --jobs>1 the topologies run concurrently (each writes its own ini and
    result dir, so they don't collide); results still print in sweep order."""
    out = {k: [] for k in ("labels", "ls", "gp", "ls_b", "gp_b", "avg_sz", "max_sz")}
    if conv:
        out["ls_ct"] = []
        out["gp_ct"] = []
    # One job per (point, seed) rather than per point: with 5 seeds and 6 points
    # that is 30 independent runs, which fills --jobs far better than 6 would.
    jobs = [(i, t) for i, (_, paths) in enumerate(items) for t in paths]
    run_one = lambda j: measure(j[1], nobatch, conv)
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
        out["gp"].append(r["gp"])
        out["ls_b"].append(r["ls_b"])
        out["gp_b"].append(r["gp_b"])
        out["avg_sz"].append(r["avg_sz"])
        out["max_sz"].append(r["max_sz"])
        mode = "no-batch" if nobatch else "batched"
        ct_str = ""
        if conv:
            out["ls_ct"].append(r["ls_ct"])
            out["gp_ct"].append(r["gp_ct"])
            def _ct(v):
                return "None" if v is None else f"{v:.2f}{pm(v, '.2f')}s"
            ct_str = f"  LS_ct={_ct(r['ls_ct'])}  GP_ct={_ct(r['gp_ct'])}"
        # message_count returns None when a run produced no .sca
        def _n(v):
            return "None" if v is None else f"{v:.0f}{pm(v, '.0f')}"
        seeds = f" (mean of {len(paths)} seeds)" if len(paths) > 1 else ""
        # The label is two lines for the plot's tick; flatten it for the console.
        flat = label.replace("\n", " ")
        print(f"  {flat:<14}{seeds} LS={_n(r['ls'])}/{r['ls_b']/1000:.0f}KB  "
              f"GP({mode})={_n(r['gp'])}/{r['gp_b']/1000:.0f}KB  "
              f"batches={r['nb']:.0f} avg_sz={r['avg_sz']:.1f} "
              f"max_sz={r['max_sz']:.0f}{ct_str}")
    return out


def plot(res, title, xlabel, outfile, nobatch=False):
    """Two panels sharing the sweep's x-axis (message count + batch stats), plus
    an optional third panel (convergence time) when res contains ls_ct/gp_ct.
    `res` is the dict from sweep()."""
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    labels = res["labels"]
    xs = list(range(len(labels)))
    has_conv = "ls_ct" in res and any(v is not None for v in res["ls_ct"])
    ncols = 3 if has_conv else 2
    fig, axes = plt.subplots(1, ncols, figsize=(7.5 * ncols, 6))
    axL, axR = axes[0], axes[1]

    gp_label = "Path-Vector, no batching" if nobatch else "Path-Vector, batched"
    axL.plot(xs, res["ls"], "o-", label="Link-State (LS)", color="tab:blue")
    axL.plot(xs, res["gp"], "s-", label=gp_label, color="tab:red")
    axL.set_xticks(xs); axL.set_xticklabels(labels, rotation=45, ha="right")
    axL.set_xlabel(xlabel); axL.set_ylabel("routing messages sent")
    axL.set_title(title); axL.grid(True, alpha=0.3)
    # Bytes share the panel on a twin axis: message count alone hides the fact
    # that GP's PDUs are far fatter, so the two curves can rank opposite ways.
    axB = axL.twinx()
    axB.plot(xs, [b / 1000 for b in res["ls_b"]], "o--", color="tab:blue",
             alpha=0.45, label="LS bytes")
    axB.plot(xs, [b / 1000 for b in res["gp_b"]], "s--", color="tab:red",
             alpha=0.45, label="GP bytes")
    axB.set_ylabel("routing bytes sent (KB, dashed)")
    h1, l1 = axL.get_legend_handles_labels()
    h2, l2 = axB.get_legend_handles_labels()
    axL.legend(h1 + h2, l1 + l2, fontsize=8)

    axR.plot(xs, res["avg_sz"], "s-", label="mean entries / batch",
             color="tab:green")
    axR.plot(xs, res["max_sz"], "^--", label="max entries / batch",
             color="tab:olive", alpha=0.7)
    axR.set_xticks(xs); axR.set_xticklabels(labels, rotation=45, ha="right")
    axR.set_xlabel(xlabel); axR.set_ylabel("chains coalesced per batch packet")
    axR.set_title("Inter-ring UPDATE batch size"); axR.legend()
    axR.grid(True, alpha=0.3)

    if has_conv:
        axC = axes[2]
        ls_cts = [v for v in res["ls_ct"]]
        gp_cts = [v for v in res["gp_ct"]]
        axC.plot(xs, ls_cts, "o-", label="LS convergence time", color="tab:blue")
        axC.plot(xs, gp_cts, "s-", label="GP convergence time", color="tab:red")
        axC.set_xticks(xs); axC.set_xticklabels(labels, rotation=45, ha="right")
        axC.set_xlabel(xlabel); axC.set_ylabel("convergence time (s)")
        axC.set_title("Convergence time vs network size")
        axC.legend(); axC.grid(True, alpha=0.3)

    fig.tight_layout()
    fig.savefig(outfile, dpi=110)
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
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from matplotlib.patches import Wedge, Circle
    import math

    def grid_pos(dimx, dimy):
        # graphgen 'grid X,Y' numbers COLUMN-major with stride X: node = col*X+row.
        # Place on the integer lattice, flip y so row 0 is on top.
        return {col * dimx + row: (col, dimx - 1 - row)
                for col in range(dimy) for row in range(dimx)}

    def ladder_pos(W):
        return {r * W + c: (c, -r) for r in range(2) for c in range(W)}

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
        ("width 3x8",            gen_topology(3, 8), grid_pos(3, 8)),
        ("ladder 2x8 (1-D line)", gen_ladder(8),     ladder_pos(8)),
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
        ax.set_title(f"{label}\n{n} nodes, {len(rings)} rings", fontsize=10)
        ax.set_aspect("equal"); ax.axis("off")
    fig.suptitle("Representative topology per sweep type "
                 "(each node a pie of its ring memberships)", fontsize=13)
    fig.tight_layout()
    fig.savefig(outfile, dpi=120)
    print(f"  wrote {outfile}")


def main(args=None):
    if args is None:
        ap = argparse.ArgumentParser(description="GP-vs-LS message-count scaling eval")
        ap.add_argument("--quick", action="store_true", help="short sweeps for a smoke test")
        ap.add_argument("--only", choices=["square", "width", "ladder", "flower", "topos"],
                        help="run just one sweep instead of all four ('topos' = draw "
                             "the representative topology pictures only, no sims)")
        ap.add_argument("--no-batch", dest="nobatch", action="store_true",
                        help="run GP with holdInterval=0s (pre-batching baseline). "
                             "Far slower (no coalescing) — off by default; the normal "
                             "run uses the holdInterval pinned in eval_common.ini.")
        ap.add_argument("--convergence", action="store_true",
                        help="also measure convergence time per topology (extends "
                             f"sim-time-limit to {CONV_SIMTIME}; adds a third plot panel).")
        ap.add_argument("--basis", choices=["min", "tree", "short", "all"], default="min",
                        help="ring basis: min=minimum_cycle_basis (default), "
                             "tree=cycle_basis (spanning-tree; 'all' is a legacy "
                             "alias), short=greedy shortest-cycle-per-edge (small "
                             "rings bounded by --max-ring-size, no min-basis cost). "
                             "Results nest under eval/<basis>_cycles/.")
        ap.add_argument("--max-ring-size", dest="max_ring_size", type=int, default=0,
                        help="drop rings larger than this; members fall back to "
                             "echo rings (0 = keep all, the default). Long rings "
                             "dominate GP's fan-out, and --basis tree produces "
                             "very large ones. Results nest under a .mN label file.")
        ap.add_argument("--dir", metavar="DIR",
                        help="sweep every .txt in DIR instead of the built-in "
                             "grid sweeps (e.g. a ringpipe family folder under "
                             "myriad_topologies/csv/). ringpipe's own ring labels "
                             "are used when present.")
        ap.add_argument("--x-key", dest="x_key", default="fan", metavar="WORD",
                        help="with --dir: the word in the file names whose "
                             "trailing number is the x-axis (default fan; e.g. "
                             "n, share, spof)")
        ap.add_argument("--single", metavar="TOPO",
                        help="run GP+LS on ONE topology .txt and print counts (no "
                             "sweep, no plot). Path relative to simulations/topologies "
                             "or absolute.")
        add_repeat_args(ap)
        args = ap.parse_args()
        apply_repeat_args(args)

    # Nest all outputs under a per-basis subfolder so min_cycles and tree_cycles
    # keep separate cached labels, inis, and plots.
    global BASIS, MAX_RING_SIZE, TOPO_DIR, INI_DIR, PLOT_DIR
    BASIS = args.basis
    MAX_RING_SIZE = getattr(args, "max_ring_size", 0) or 0
    basis_dir = os.path.join(EVAL_DIR, f"{BASIS}_cycles"
                             + (f"_m{MAX_RING_SIZE}" if MAX_RING_SIZE else ""))
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

        # Files differing only in _seed<N> are the same sweep point measured on
        # different draws, so they are averaged rather than plotted separately.
        # Everything a seed can shift is stripped from the key: the seed itself,
        # the achieved ring length, and the fan/n/A the solver landed on. What
        # remains -- family, spof, and the sweep point (share<v> or len<v>) -- is
        # the point's identity.
        vary = re.compile(r"_(seed|rl|fan|n|A)\d+(?:\.\d+)?")
        groups = {}
        for p in topos:
            groups.setdefault(vary.sub("", os.path.basename(p)), []).append(p)
        # The label carries the achieved ring length (_rl) and ring count (_A) as
        # well as the sweep variable: length is what GP's lap cost keys off, and
        # the count is what a held sweep spends to pin the other two, so both
        # belong on the axis whatever is being swept. Averaged over the seeds,
        # like the measurements themselves.
        def mean(paths, key):
            vals = [num(q, key) for q in paths]
            return sum(vals) / len(vals)

        items = []
        for key, paths in groups.items():
            x = mean(paths, args.x_key)
            rl, rings = mean(paths, "rl"), mean(paths, "A")
            sub = " ".join(t for t in (f"rl{rl:.0f}" if rl else "",
                                       f"A{rings:.0f}" if rings else "") if t)
            label = f"{args.x_key}{x:g}" + (f"\n{sub}" if sub else "")
            items.append((x, label, sorted(paths)))
        items = [(lab, ps) for _, lab, ps in sorted(items)]
        nseed = max(len(ps) for _, ps in items)
        name = os.path.basename(os.path.normpath(args.dir))
        print(f"== dir sweep: {name} ({len(items)} points x up to {nseed} seeds) ==")
        plot(sweep(items, args.nobatch, args.convergence),
             f"GP vs LS: {name}", args.x_key,
             os.path.join(PLOT_DIR, f"dir_{name}_{args.x_key}.png"), args.nobatch)
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
        sweep([(label, topo)], args.nobatch)
        return

    announce("scaling", opts=overrides(sim_time_limit=CONV_SIMTIME) if args.convergence else [])
    if repeats_note():
        print(f" repeats{repeats_note()}")

    if args.quick:
        square_dims = [(n, n) for n in range(3, 6)]
        width_dims = [(3, w) for w in range(3, 9)]
        ladder_widths = list(range(3, 9))
        flower_N = list(range(3, 8))
    else:
        square_dims = [(n, n) for n in range(3, 11)]       # 3x3 .. 10x10
        width_dims = [(3, w) for w in range(3, 31)]        # 3x3 .. 3x30
        ladder_widths = list(range(3, 31))                 # ladder3_3 .. ladder3_30
        flower_N = list(range(3, 13))                      # flower_3 .. flower_12

    if args.only in (None, "square"):
        print("== square sweep (NxN) ==")
        s_items = [(f"{x}x{y}", gen_topology(x, y)) for (x, y) in square_dims]
        plot(sweep(s_items, args.nobatch, args.convergence),
             "GP vs LS: square grid scaling (dense 2D ring-mesh)",
             "grid size", os.path.join(PLOT_DIR, "square_scaling.png"), args.nobatch)

    if args.only in (None, "width"):
        print("== width sweep (3xW) ==")
        w_items = [(f"3x{w}", gen_topology(3, w)) for (_, w) in width_dims]
        plot(sweep(w_items, args.nobatch, args.convergence),
             "GP vs LS: width scaling (3xW ladder ring-mesh)",
             "grid size", os.path.join(PLOT_DIR, "width_scaling.png"), args.nobatch)

    if args.only in (None, "ladder"):
        # 2xW ladder = W-1 unit-square rings chained in a pure 1-D line. Zero
        # path diversity (each ring adjacent only to its two neighbours), so this
        # is GP's very best case and the linear-scaling floor for the sweep.
        print("== ladder sweep (2xW, 1-D line of unit-square rings) ==")
        l_items = [(f"L2x{w}", gen_ladder(w)) for w in ladder_widths]
        plot(sweep(l_items, args.nobatch, args.convergence),
             "GP vs LS: ladder scaling (2xW, pure 1-D line of rings)",
             "ladder width", os.path.join(PLOT_DIR, "ladder_scaling.png"), args.nobatch)

    if args.only in (None, "flower"):
        # Ring-SIZE sweep (orthogonal to the others, which grow path diversity):
        # the myriad_large flower generalised — central ring of 2N + N petals of
        # N routers each, 2-node bridges (spof=0). Ring-graph stays a pure STAR
        # at every N, so diversity is minimal and constant; only ring size (and
        # node count = N^2) grows. Isolates the cost of larger rings from the
        # cost of more paths. Same star shape as myriad_large (which uses
        # size-2N petals); here petals are size N by request.
        print("== flower sweep (central 2N + N petals of N, ring size grows) ==")
        f_items = [(f"F{N}", gen_flower(N)) for N in flower_N]
        plot(sweep(f_items, args.nobatch, args.convergence),
             "GP vs LS: flower scaling (N petals of N routers, star ring-graph)",
             "flower N (ring size; N^2 nodes)", os.path.join(PLOT_DIR, "flower_scaling.png"),
             args.nobatch)

    if args.only is None:
        print("== drawing representative topologies ==")
        draw_topologies(os.path.join(PLOT_DIR, "topologies.png"))


if __name__ == "__main__":
    main()
