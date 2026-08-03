#!/usr/bin/env python3
"""Compare MyriadLS (link-state) vs MyriadGP (path-vector) message counts as
grid topologies scale two ways:

  1. square   : 2x2, 3x3, ... 10x10   (both dimensions grow -> dense 2D ring-mesh)
  2. width    : 3x3, 3x4, ... 3x40    (one dimension grows -> ring-adjacency stays
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

CREATED_RE = re.compile(r"created:\s*(\d+)")


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
    """3xW grid with the middle-row horizontal link removed at EVERY SECOND
    column gap (even gaps). Odd gaps keep all three horizontals (full unit
    squares -> 2 small rings); even gaps drop the middle link (top+bottom
    merge into one 6-node ring). An intermediate ring-adjacency density between
    the full 3xW mesh and the fully-merged line. Two links per column pair
    always, so still 2-edge-connected. Writes ladder3_W.txt, returns its path."""
    path = os.path.join(TOPO_DIR, f"ladder3_{W}.txt")
    if os.path.exists(path):
        return path
    nid = lambda r, c: r * W + c
    edges = []
    for c in range(W):                      # vertical rungs (all present)
        edges.append((nid(0, c), nid(1, c)))
        edges.append((nid(1, c), nid(2, c)))
    for c in range(W - 1):                  # horizontals
        edges.append((nid(0, c), nid(0, c + 1)))       # top always
        edges.append((nid(2, c), nid(2, c + 1)))       # bottom always
        if c % 2 == 1:                                 # middle only on odd gaps
            edges.append((nid(1, c), nid(1, c + 1)))
    with open(path, "w") as f:
        f.write(f"# 3x{W} grid, middle-row horizontal link removed at every second gap\n")
        f.write("# intermediate ring-adjacency density (between full mesh and line)\n")
        f.write(f"{3 * W}\t{len(edges)}\n")
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
    """Run the sim from SIM_ROOT (so relative NED lookups work), scrape 'created:'."""
    r = run([BIN, "-u", "Cmdenv", ini_path], cwd=SIM_ROOT)
    counts = CREATED_RE.findall(r.stdout)
    if not counts:
        print(f"  !! {proto} produced no message count. stderr tail:\n{r.stderr[-400:]}")
        return None
    return int(counts[-1])   # last periodic report = final total


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
    ax.set_ylabel("total messages created")
    ax.set_title(title)
    ax.legend()
    ax.grid(True, alpha=0.3)
    fig.tight_layout()
    fig.savefig(outfile, dpi=110)
    print(f"  wrote {outfile}")


def main():
    ap = argparse.ArgumentParser(description="GP-vs-LS message-count scaling eval")
    ap.add_argument("--quick", action="store_true", help="short sweeps for a smoke test")
    ap.add_argument("--only", choices=["square", "width", "ladder"],
                    help="run just one sweep instead of all three")
    args = ap.parse_args()

    for d in (TOPO_DIR, INI_DIR, PLOT_DIR):
        os.makedirs(d, exist_ok=True)
    if not os.path.exists(BIN):
        sys.exit(f"RoutingSim binary not found at {BIN} — build it first.")

    if args.quick:
        square_dims = [(n, n) for n in range(2, 6)]
        width_dims = [(3, w) for w in range(3, 9)]
        ladder_widths = list(range(3, 9))
    else:
        square_dims = [(n, n) for n in range(2, 11)]       # 2x2 .. 10x10
        width_dims = [(3, w) for w in range(3, 41)]        # 3x3 .. 3x40
        ladder_widths = list(range(3, 41))                 # ladder3_3 .. ladder3_40

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
        # Extension: 3xW grid with the middle-row horizontal link removed at
        # every second column gap — an intermediate ring-adjacency density
        # between the full 3xW mesh and a pure 1D line. Tests how GP's scaling
        # responds to partially thinning the ring-mesh.
        print("== ladder sweep (3xW, middle rung removed every 2nd gap) ==")
        l_items = [(f"L3x{w}", gen_ladder(w)) for w in ladder_widths]
        l_lbl, l_ls, l_gp = sweep(l_items)
        plot(l_lbl, l_ls, l_gp,
             "GP vs LS: ladder scaling (3xW, middle rung removed every 2nd gap)",
             "ladder width", os.path.join(PLOT_DIR, "ladder_scaling.png"))


if __name__ == "__main__":
    main()
