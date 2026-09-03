#!/usr/bin/env python3
"""MyriadGP cost of LARGE RINGS: sweep topo2rings' --max-ring-size and measure
the bytes one formation round costs.

Companion to eval_scaling.py (which shows the path-diversity axis). Here the
topology is held fixed and only the ring DECOMPOSITION changes: capping ring
size splits long rings into echo rings, so the sweep isolates ring size from
every other property.

One formation UPDATE origination, not two: collectWarmupDuration=0 removes the
warmup COLLECT storm and its post-warmup re-origination (MyriadGp.cc:559-563),
and refreshInterval is pushed past the horizon so nothing re-originates.
--verify asserts that by re-running a point at 2x sim-time for equal bytes.

    # small smoke test (what fits on an 8-core box)
    python3 eval_ringsize.py --topos 3 --nodes 60 --sizes 4,8,16 --jobs 8

    # server run
    python3 eval_ringsize.py --topos 20 --nodes 400 --degree 4.5 \
        --sizes 4,6,8,12,16,24,32,48 --jobs 64

Writes plots + ringsize.csv to eval_ringsize/; topologies, inis and .sca dirs
are deleted afterwards unless --keep.
"""
import argparse
import csv
import glob
import math
import os
import re
import shutil
import subprocess
import sys
import threading
import time
from concurrent.futures import ThreadPoolExecutor
from pathlib import Path

sys.path.insert(0, str(Path(__file__).parent))
from eval_harness import BIN, COMMON_INI, MYRIAD_DIR, SIM_ROOT, SRC, run_sim

OUT = MYRIAD_DIR / "eval/ringsize"
WORK = OUT / "work"          # topologies + inis + result dirs; removed at the end

BYTES_RE = re.compile(r"^scalar\s+\S+\s+bytes_total\s+([0-9.]+)", re.M)
STAT_RE = re.compile(r"rings=(\d+).*?ring size: max=(\d+) mean=([0-9.]+)", re.S)


class Progress:
    """Thread-safe 'k/N done, elapsed, ETA' line. Workers run in parallel, so
    per-run detail would interleave; only the counter and the last finished
    item are shown."""
    def __init__(self, total, what):
        self.n, self.total, self.what = 0, total, what
        self.t0, self.lock = time.time(), threading.Lock()

    def step(self, label=""):
        with self.lock:
            self.n += 1
            el = time.time() - self.t0
            eta = el / self.n * (self.total - self.n)
            print(f"  [{self.what} {self.n}/{self.total}] {el:.0f}s elapsed, "
                  f"ETA {eta:.0f}s  {label}", flush=True)


def gen_topo(i, nodes, degree, seed0):
    path = WORK / f"rgg_{i}.txt"
    if not path.exists():
        subprocess.run([sys.executable, str(MYRIAD_DIR / "gen_rgg.py"), str(nodes),
                        "--avg-degree", str(degree), "--seed", str(seed0 + i),
                        "--out", str(path)], check=True, capture_output=True)
    return path


def gen_labels(topo, msize):
    """Ring labels for one (topology, max ring size); returns (path, stats)."""
    out = WORK / f"{topo.stem}.m{msize}.ringlabels.ini"
    if not out.exists():
        subprocess.run([sys.executable, str(MYRIAD_DIR / "topo2rings.py"), str(topo),
                        "--out", str(out), "--no-png", "--basis", "tree",
                        "--max-ring-size", str(msize)], check=True, capture_output=True)
    head = "".join(l for l in out.read_text().splitlines(True) if l.startswith("#"))
    m = STAT_RE.search(head)
    stats = dict(zip(("rings", "ring_max", "ring_mean"), m.groups())) if m else {}
    return out, stats


def write_ini(proto, topo, labels, msize, sim_time):
    ini = WORK / f"{proto}_{topo.stem}_m{msize}.ini"
    ini.write_text("\n".join([
        f"include {COMMON_INI}",
        "[General]",
        f"ned-path={SRC};{SIM_ROOT}",
        f'**.protocols = "routingsim.routing.MYRIAD.myriad{proto}"',
        "network = routingsim.simulations.DefaultNetwork",
        f'*.manager.topologyFile = "{topo.resolve()}"',
        "*.mainStatistics.samplingInterval = 0.01s",
        # exactly one formation origination, nothing periodic after it
        "**.collectWarmupDuration = 0s",
        "**.refreshInterval = 1000000s",
        "**.snapshotInterval = 0s",          # RIB snapshotting off: pure cost run
        f"sim-time-limit = {sim_time}s",
    ] + [l for l in labels.read_text().splitlines() if l and not l.startswith("#")]) + "\n")
    return ini


def bytes_of(proto, topo, labels, msize, sim_time, tag=""):
    """Run one sim, return summed bytes_total over all nodes."""
    ini = write_ini(proto, topo, labels, msize, sim_time)
    res = WORK / f"res_{ini.stem}{tag}"
    shutil.rmtree(res, ignore_errors=True)
    run_sim([BIN, "-u", "Cmdenv", str(ini), "--result-dir", str(res)],
            cwd=SIM_ROOT, rep=1)
    scas = glob.glob(str(res / "*.sca"))
    if not scas:
        sys.exit(f"no .sca from {ini}")
    return sum(int(float(m.group(1)))
               for s in scas for m in BYTES_RE.finditer(open(s).read()))


def ring_failure_plot(p, kmax, path):
    """P(a ring of k links loses >=2 links at once) for i.i.d. link failure p.
    A ring survives one break (it degrades to a chain); two breaks partition it,
    which is why capping ring size caps the blast radius."""
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    ks = list(range(2, kmax + 1))
    pf = [1 - (1 - p) ** k - k * p * (1 - p) ** (k - 1) for k in ks]
    fig, ax = plt.subplots(figsize=(7, 5))
    ax.plot(ks, pf, "-", color="tab:red")
    ax.set_xlabel("ring size (nodes = links)")
    ax.set_ylabel(f"P(>=2 simultaneous link failures)")
    ax.set_title(f"Probability a ring is partitioned, link failure p={p:.0%}")
    ax.grid(True, alpha=0.3)
    fig.tight_layout()
    fig.savefig(path, dpi=110)
    print(f"  wrote {path}")
    return ks, pf


def plot(sizes, gp, ls, ring_mean, path, nodes, degree, ntopo):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    fig, (ax, ax2) = plt.subplots(1, 2, figsize=(15, 6))
    w = 0.35
    xs = list(range(len(sizes)))
    for series, off, color, name in ((gp, -w / 2, "tab:red", "MyriadGP"),
                                     (ls, +w / 2, "tab:blue", "MyriadLS")):
        b = ax.boxplot([series[s] for s in sizes],
                       positions=[x + off for x in xs], widths=w,
                       patch_artist=True, manage_ticks=False)
        for box in b["boxes"]:
            box.set(facecolor=color, alpha=0.5)
        for med in b["medians"]:
            med.set(color="black")
        ax.plot([], [], color=color, lw=6, alpha=0.5, label=name)
    ax.set_xticks(xs)
    ax.set_xticklabels([str(s) for s in sizes])
    ax.set_xlabel("max ring size (topo2rings --max-ring-size)")
    ax.set_ylabel("total bytes transmitted, one formation round")
    ax.set_title(f"Formation cost vs ring-size cap\n"
                 f"{ntopo} RGG topologies, n={nodes}, avg degree {degree}")
    ax.legend()
    ax.grid(True, alpha=0.3)

    ax2.boxplot([ring_mean[s] for s in sizes], positions=xs, widths=0.5,
                patch_artist=True, manage_ticks=False)
    ax2.set_xticks(xs)
    ax2.set_xticklabels([str(s) for s in sizes])
    ax2.set_xlabel("max ring size")
    ax2.set_ylabel("mean realized ring size")
    ax2.set_title("What the cap actually buys (rings above it become echo rings)")
    ax2.grid(True, alpha=0.3)
    fig.tight_layout()
    fig.savefig(path, dpi=110)
    print(f"  wrote {path}")


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--topos", type=int, default=5, help="number of RGG topologies")
    ap.add_argument("--nodes", type=int, default=200)
    ap.add_argument("--degree", type=float, default=4.5, help="target average degree")
    ap.add_argument("--sizes", default="4,8,16,32",
                    help="max-ring-size values, comma separated")
    ap.add_argument("--seed", type=int, default=0, help="first RGG seed")
    ap.add_argument("--sim-time", type=float, default=100.0,
                    help="seconds of sim time; must cover formation (see --verify)")
    ap.add_argument("--jobs", type=int, default=os.cpu_count())
    ap.add_argument("--fail-prob", type=float, default=0.05)
    ap.add_argument("--keep", action="store_true", help="keep topologies/inis/.sca")
    ap.add_argument("--verify", action="store_true",
                    help="re-run one point at 2x sim-time and assert equal bytes, "
                         "i.e. only one UPDATE wave happened inside the window")
    a = ap.parse_args()
    sizes = [int(s) for s in a.sizes.split(",")]

    OUT.mkdir(parents=True, exist_ok=True)
    WORK.mkdir(parents=True, exist_ok=True)
    print(f"  {a.topos} RGG topologies, n={a.nodes}, avg degree {a.degree}, "
          f"sizes {sizes}, {a.jobs} parallel jobs", flush=True)

    pg = Progress(a.topos, "topology")
    topos = []
    for i in range(a.topos):
        topos.append(gen_topo(i, a.nodes, a.degree, a.seed))
        pg.step(topos[-1].name)

    jobs = [(t, m) for m in sizes for t in topos]
    with ThreadPoolExecutor(max_workers=a.jobs) as ex:
        pg = Progress(len(jobs), "ring labels")

        def lab(tm):
            r = gen_labels(*tm)
            pg.step(f"{tm[0].stem} m={tm[1]} -> {r[1].get('rings','?')} rings, "
                    f"max {r[1].get('ring_max','?')}")
            return r
        labels = list(ex.map(lab, jobs))

        pgs = Progress(len(jobs), "sim pair")

        def one(k):
            (t, m), (l, st) = jobs[k], labels[k]
            t0 = time.time()
            r = {"topo": t.stem, "msize": m,
                 "gp": bytes_of("gp", t, l, m, a.sim_time),
                 "ls": bytes_of("ls", t, l, m, a.sim_time),
                 **st}
            pgs.step(f"{t.stem} m={m}: GP {r['gp']:,} B, LS {r['ls']:,} B "
                     f"({time.time()-t0:.0f}s)")
            return r
        rows = list(ex.map(one, range(len(jobs))))

    if a.verify:
        print("  verifying a single UPDATE wave (re-run at 2x sim-time) …", flush=True)
        (t, m), (lab, _) = jobs[-1], labels[-1]
        base = next(r["gp"] for r in rows if r["topo"] == t.stem and r["msize"] == m)
        long = bytes_of("gp", t, lab, m, a.sim_time * 2, tag="_2x")
        assert long == base, (f"bytes grew from {base} to {long} at 2x sim-time: "
                              f"more than one UPDATE wave inside the window")
        print(f"  verify OK: {base} bytes at both {a.sim_time}s and {2*a.sim_time}s")

    with open(OUT / "ringsize.csv", "w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=list(rows[0]))
        w.writeheader()
        w.writerows(rows)
    print(f"  wrote {OUT/'ringsize.csv'}")

    by = lambda key: {m: [float(r[key]) for r in rows if r["msize"] == m] for m in sizes}
    med = lambda v: sorted(v)[len(v) // 2]
    print(f"\n  {'cap':>5}{'GP median B':>16}{'LS median B':>16}"
          f"{'GP/LS':>8}{'mean ring':>11}{'max ring':>10}")
    for m in sizes:
        g, l = by("gp")[m], by("ls")[m]
        print(f"  {m:>5}{med(g):>16,.0f}{med(l):>16,.0f}{med(g)/med(l):>8.1f}"
              f"{sum(by('ring_mean')[m])/len(g):>11.1f}"
              f"{max(by('ring_max')[m]):>10.0f}")
    print()
    plot(sizes, by("gp"), by("ls"), by("ring_mean"), OUT / "ringsize_bytes.png",
         a.nodes, a.degree, a.topos)
    ring_failure_plot(a.fail_prob, max(sizes) * 2, OUT / "ring_failure_prob.png")

    if not a.keep:
        shutil.rmtree(WORK, ignore_errors=True)
        print(f"  cleaned {WORK} (--keep to retain topologies/inis/.sca)")


if __name__ == "__main__":
    main()
