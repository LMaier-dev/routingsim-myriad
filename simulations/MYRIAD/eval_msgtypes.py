#!/usr/bin/env python3
"""Run MyriadLS and MyriadGP on one topology .txt and pie-chart the per-message-
type breakdown of total traffic for each.

Message counts come from the per-type send scalars the protocols record in
finish() (`sent_hello`, `sent_lsu`, `sent_collect`, ...), read back from the
OMNeT++ .sca result file — packet-accurate, framework-native (not log scraping).

    python3 eval_msgtypes.py ../topologies/myriad_paperexample.txt
    python3 eval_msgtypes.py ../topologies/grid_5,5.txt --out /tmp/pie.png

Self-contained: generates ring labels, writes its own inlined .ini per protocol
(no include fragility), runs Cmdenv, parses .sca, plots. Aligned, fair timing
matches eval_scaling.py (jitter off, 1ms delay, identical timers).
"""
import argparse
import glob
import os
import re
import subprocess
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
SIM_ROOT = os.path.dirname(HERE)
REPO = os.path.dirname(SIM_ROOT)
SRC = os.path.join(REPO, "src")
BIN = os.path.join(SRC, "RoutingSim")
TOPO2RINGS = os.path.join(HERE, "topo2rings.py")
WORK = os.path.join(HERE, "eval", "msgtypes")   # git-ignored (eval/)

SIM_TIME = "10s"
DELAY_NS = "1000000ns"
PARAMS = {
    "helloInterval": "2s", "deadInterval": "4s", "rxmtInterval": "1s",
    "helloJitter": "0s", "dbExchangeBatchSize": "10",
    "refreshInterval": "15s", "refreshJitter": "1s",
}
# scalar line: scalar <module> sent_<type> <value>
SCALAR_RE = re.compile(r"^scalar\s+\S+\s+sent_(\S+)\s+([0-9.]+)", re.M)


def run(cmd, **kw):
    return subprocess.run(cmd, capture_output=True, text=True, **kw)


def ring_labels(topo_path):
    stem = os.path.splitext(os.path.basename(topo_path))[0]
    ini = os.path.join(WORK, f"{stem}.ringlabels.ini")
    if not os.path.exists(ini):
        r = run(["python3", TOPO2RINGS, topo_path, "--out-dir", WORK, "--no-png"])
        if not os.path.exists(ini):
            sys.exit(f"topo2rings failed:\n{r.stdout}\n{r.stderr}")
    with open(ini) as f:
        return [ln.rstrip("\n") for ln in f if ln.strip() and not ln.startswith("#")]


def write_ini(proto, topo_path, ring_lines):
    stem = os.path.splitext(os.path.basename(topo_path))[0]
    ini_path = os.path.join(WORK, f"{proto}_{stem}.ini")
    lines = [
        "[General]",
        f"ned-path={SRC};{SIM_ROOT}",
        f'**.protocols = "routingsim.routing.MYRIAD.myriad{proto}"',
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
    lines += ring_lines
    with open(ini_path, "w") as f:
        f.write("\n".join(lines) + "\n")
    return ini_path


def run_proto(proto, ini_path):
    """Run one protocol, return {type: total_count} summed over all nodes."""
    res = os.path.join(WORK, f"res_{proto}")
    for old in glob.glob(os.path.join(res, "*.sca")):
        os.remove(old)
    r = run([BIN, "-u", "Cmdenv", ini_path, "--result-dir", res], cwd=SIM_ROOT)
    scas = glob.glob(os.path.join(res, "*.sca"))
    if not scas:
        sys.exit(f"{proto}: no .sca produced.\n{r.stdout[-500:]}\n{r.stderr[-500:]}")
    counts = {}
    for sca in scas:
        for m in SCALAR_RE.finditer(open(sca).read()):
            typ, val = m.group(1), float(m.group(2))
            if typ == "total":
                continue          # derived; we re-sum from the parts
            counts[typ] = counts.get(typ, 0) + val
    return {k: int(v) for k, v in counts.items()}


def plot(topo_name, ls_counts, gp_counts, outfile):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    fig, axes = plt.subplots(1, 2, figsize=(13, 6))
    for ax, (title, counts) in zip(
        axes, [("MyriadLS (link-state)", ls_counts),
               ("MyriadGP (path-vector)", gp_counts)]):
        total = sum(counts.values())
        labels = [f"{k}\n{v}" for k, v in sorted(counts.items())]
        sizes = [counts[k] for k in sorted(counts)]
        ax.pie(sizes, labels=labels, autopct="%1.0f%%", startangle=90,
               counterclock=False)
        ax.set_title(f"{title}\ntotal = {total} messages")
        ax.axis("equal")
    fig.suptitle(f"Message-type breakdown on {topo_name}", fontsize=13)
    fig.tight_layout()
    fig.savefig(outfile, dpi=110)
    print(f"  wrote {outfile}")


def main():
    ap = argparse.ArgumentParser(description="Per-message-type pie charts, LS vs GP")
    ap.add_argument("topology", help="path to a routingsim .txt")
    ap.add_argument("--out", help="output PNG (default: eval/msgtypes/<stem>_msgtypes.png)")
    args = ap.parse_args()

    os.makedirs(WORK, exist_ok=True)
    if not os.path.exists(BIN):
        sys.exit(f"RoutingSim binary not found at {BIN} — build it first.")
    topo = os.path.abspath(args.topology)
    if not os.path.exists(topo):
        sys.exit(f"topology not found: {topo}")
    stem = os.path.splitext(os.path.basename(topo))[0]

    rings = ring_labels(topo)
    ls = run_proto("ls", write_ini("ls", topo, rings))
    gp = run_proto("gp", write_ini("gp", topo, rings))
    print(f"  LS: {ls}  total={sum(ls.values())}")
    print(f"  GP: {gp}  total={sum(gp.values())}")

    out = args.out or os.path.join(WORK, f"{stem}_msgtypes.png")
    plot(stem, ls, gp, out)


if __name__ == "__main__":
    main()
