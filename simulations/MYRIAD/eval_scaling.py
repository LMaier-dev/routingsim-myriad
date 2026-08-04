#!/usr/bin/env python3
"""Compare MyriadLS (link-state) vs MyriadGP (path-vector) message counts as
grid topologies scale two ways:

  1. square   : 2x2, 3x3, ... 10x10   (both dimensions grow -> dense 2D ring-mesh)
  2. width    : 3x3, 3x4, ... 3x30    (one dimension grows -> ring-adjacency stays
                                        a narrow ladder, GP's favourable case)

For each grid it generates the topology + ring labels, writes a self-contained
.ini per (protocol, grid), runs Cmdenv, scrapes the total "created:" message
count, and plots LS vs GP for both sweeps.

Everything generated lands in eval/ (git-ignored). Nothing touches the tracked
myriad{gp,ls}.ini. Run from anywhere:

    python3 simulations/MYRIAD/eval_scaling.py            # full sweep + plots
    python3 simulations/MYRIAD/eval_scaling.py --quick    # small sweep for a smoke test
"""
import argparse
import glob
import os
import re
import subprocess
import sys

HERE = os.path.dirname(os.path.abspath(__file__))          # .../simulations/MYRIAD
SIM_ROOT = os.path.dirname(HERE)                            # .../simulations
REPO = os.path.dirname(SIM_ROOT)                           # repo root
SRC = os.path.join(REPO, "src")
BIN = os.path.join(SRC, "RoutingSim")
GRAPHGEN = os.path.join(REPO, "scripts", "graphgen.py")
TOPO2RINGS = os.path.join(HERE, "topo2rings.py")

EVAL_DIR = os.path.join(HERE, "eval")
TOPO_DIR = os.path.join(EVAL_DIR, "topologies")
INI_DIR = os.path.join(EVAL_DIR, "ini")
PLOT_DIR = os.path.join(EVAL_DIR, "plots")

# Aligned, fair timing for BOTH protocols (see the GP-vs-LS analysis): identical
# hello/dead/rxmt, jitter off (LS's DBD exchange breaks under hello jitter), a
# uniform 1ms control-plane delay, and enough runtime for both to converge.
SIM_TIME = "10s"
DELAY_NS = "1000000ns"   # 1 ms
PARAMS = {
    "helloInterval": "2s",
    "deadInterval": "4s",
    "rxmtInterval": "1s",
    "helloJitter": "0s",
    "dbExchangeBatchSize": "10",
    "refreshInterval": "15s",
    "refreshJitter": "1s",
}

SENT_TOTAL_RE = re.compile(r"^scalar\s+\S+\s+sent_total\s+([0-9.]+)", re.M)


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
    """Run topo2rings (skipped if labels already exist), return the per-node
    ringLabel lines (inlined into the ini)."""
    stem = os.path.splitext(os.path.basename(topo_path))[0]
    ini = os.path.join(TOPO_DIR, f"{stem}.ringlabels.ini")
    if not os.path.exists(ini):
        r = run(["python3", TOPO2RINGS, topo_path, "--out-dir", TOPO_DIR, "--no-png"])
        if not os.path.exists(ini):
            sys.exit(f"topo2rings failed for {topo_path}:\n{r.stdout}\n{r.stderr}")
    with open(ini) as f:
        return [ln for ln in f if ln.strip() and not ln.startswith("#")]


def write_ini(proto, topo_path, ring_lines):
    """Write a self-contained .ini (absolute paths, inlined ringlabels)."""
    stem = os.path.splitext(os.path.basename(topo_path))[0]
    ini_path = os.path.join(INI_DIR, f"{proto}_{stem}.ini")
    proto_ned = f"routingsim.routing.MYRIAD.myriad{proto}"
    lines = [
        "[General]",
        f"ned-path={SRC};{SIM_ROOT}",
        f'**.protocols = "{proto_ned}"',
        "network = routingsim.simulations.DefaultNetwork",
        f"sim-time-limit = {SIM_TIME}",
        f"**.nodeMinControlPlaneDelay = {DELAY_NS}",
        f"**.nodeMaxControlPlaneDelay = {DELAY_NS}",
        "cmdenv-express-mode = true",
        '**.logLevel = "warn"',
        "cmdenv-event-banners = false",
        f'*.manager.topologyFile = "{topo_path}"',
        "*.mainStatistics.samplingInterval = 0.01s",
        "*.dynamics.fraction = 0",
        "*.dynamics.warmup = 0s",
        "**.mobility = false",
    ]
    lines += [f"**.{k} = {v}" for k, v in PARAMS.items()]
    lines += [ln.rstrip("\n") for ln in ring_lines]
    with open(ini_path, "w") as f:
        f.write("\n".join(lines) + "\n")
    return ini_path


def message_count(proto, ini_path):
    """Run the sim (from SIM_ROOT so relative NED lookups work) and return the
    total application-level messages sent, summed from the per-type `sent_total`
    scalars each protocol records in finish(). This counts routing messages on
    the wire — NOT the kernel's gross `created:` total, which also includes
    timers, packet wrappers, and per-send dups."""
    res = os.path.join(INI_DIR, f"res_{proto}_{os.path.basename(ini_path)}")
    for old in glob.glob(os.path.join(res, "*.sca")):
        os.remove(old)
    r = run([BIN, "-u", "Cmdenv", ini_path, "--result-dir", res], cwd=SIM_ROOT)
    scas = glob.glob(os.path.join(res, "*.sca"))
    if not scas:
        print(f"  !! {proto} produced no .sca. stderr tail:\n{r.stderr[-400:]}")
        return None
    total = 0
    for sca in scas:
        for m in SENT_TOTAL_RE.finditer(open(sca).read()):
            total += int(float(m.group(1)))
    return total


def sweep(items):
    """items: list of (label, topo_path). Returns (labels, ls_counts, gp_counts)."""
    labels, ls, gp = [], [], []
    for (label, topo) in items:
        rings = ring_label_lines(topo)
        ls_ini = write_ini("ls", topo, rings)
        gp_ini = write_ini("gp", topo, rings)
        ls_n = message_count("ls", ls_ini)
        gp_n = message_count("gp", gp_ini)
        labels.append(label)
        ls.append(ls_n)
        gp.append(gp_n)
        print(f"  {label:<6} LS={ls_n}  GP={gp_n}")
    return labels, ls, gp


def plot(labels, ls, gp, title, xlabel, outfile):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    xs = list(range(len(labels)))
    fig, ax = plt.subplots(figsize=(10, 6))
    ax.plot(xs, ls, "o-", label="Link-State (MyriadLS)", color="tab:blue")
    ax.plot(xs, gp, "s-", label="Path-Vector (MyriadGP)", color="tab:red")
    ax.set_xticks(xs)
    ax.set_xticklabels(labels, rotation=45, ha="right")
    ax.set_xlabel(xlabel)
    ax.set_ylabel("routing messages sent")
    ax.set_title(title)
    ax.legend()
    ax.grid(True, alpha=0.3)
    fig.tight_layout()
    fig.savefig(outfile, dpi=110)
    print(f"  wrote {outfile}")


def _read_edges(topo_path):
    """Return (n_nodes, [edges]) from a routingsim .txt (skip #/blank; first
    remaining line is the N M header)."""
    lines = [l for l in open(topo_path) if l.strip() and not l.startswith("#")]
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
        # full ring membership per node (all rings, for the pie wedges)
        stem = os.path.splitext(os.path.basename(topo))[0]
        ini = os.path.join(TOPO_DIR, f"{stem}.ringlabels.ini")
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


def main():
    ap = argparse.ArgumentParser(description="GP-vs-LS message-count scaling eval")
    ap.add_argument("--quick", action="store_true", help="short sweeps for a smoke test")
    ap.add_argument("--only", choices=["square", "width", "ladder", "flower", "topos"],
                    help="run just one sweep instead of all four ('topos' = draw "
                         "the representative topology pictures only, no sims)")
    args = ap.parse_args()

    for d in (TOPO_DIR, INI_DIR, PLOT_DIR):
        os.makedirs(d, exist_ok=True)
    if args.only == "topos":
        print("== drawing representative topologies ==")
        draw_topologies(os.path.join(PLOT_DIR, "topologies.png"))
        return
    if not os.path.exists(BIN):
        sys.exit(f"RoutingSim binary not found at {BIN} — build it first.")

    if args.quick:
        square_dims = [(n, n) for n in range(2, 6)]
        width_dims = [(3, w) for w in range(3, 9)]
        ladder_widths = list(range(3, 9))
        flower_N = list(range(3, 8))
    else:
        square_dims = [(n, n) for n in range(2, 11)]       # 2x2 .. 10x10
        width_dims = [(3, w) for w in range(3, 31)]        # 3x3 .. 3x30
        ladder_widths = list(range(3, 31))                 # ladder3_3 .. ladder3_30
        flower_N = list(range(3, 13))                      # flower_3 .. flower_12

    if args.only in (None, "square"):
        print("== square sweep (NxN) ==")
        s_items = [(f"{x}x{y}", gen_topology(x, y)) for (x, y) in square_dims]
        s_lbl, s_ls, s_gp = sweep(s_items)
        plot(s_lbl, s_ls, s_gp,
             "GP vs LS: square grid scaling (dense 2D ring-mesh)",
             "grid size", os.path.join(PLOT_DIR, "square_scaling.png"))

    if args.only in (None, "width"):
        print("== width sweep (3xW) ==")
        w_items = [(f"3x{w}", gen_topology(3, w)) for (_, w) in width_dims]
        w_lbl, w_ls, w_gp = sweep(w_items)
        plot(w_lbl, w_ls, w_gp,
             "GP vs LS: width scaling (3xW ladder ring-mesh)",
             "grid size", os.path.join(PLOT_DIR, "width_scaling.png"))

    if args.only in (None, "ladder"):
        # 2xW ladder = W-1 unit-square rings chained in a pure 1-D line. Zero
        # path diversity (each ring adjacent only to its two neighbours), so this
        # is GP's very best case and the linear-scaling floor for the sweep.
        print("== ladder sweep (2xW, 1-D line of unit-square rings) ==")
        l_items = [(f"L2x{w}", gen_ladder(w)) for w in ladder_widths]
        l_lbl, l_ls, l_gp = sweep(l_items)
        plot(l_lbl, l_ls, l_gp,
             "GP vs LS: ladder scaling (2xW, pure 1-D line of rings)",
             "ladder width", os.path.join(PLOT_DIR, "ladder_scaling.png"))

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
        f_lbl, f_ls, f_gp = sweep(f_items)
        plot(f_lbl, f_ls, f_gp,
             "GP vs LS: flower scaling (N petals of N routers, star ring-graph)",
             "flower N (ring size; N^2 nodes)", os.path.join(PLOT_DIR, "flower_scaling.png"))

    if args.only is None:
        print("== drawing representative topologies ==")
        draw_topologies(os.path.join(PLOT_DIR, "topologies.png"))


if __name__ == "__main__":
    main()
