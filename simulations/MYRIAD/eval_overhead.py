#!/usr/bin/env python3
"""Steady-state control-plane overhead: MyriadGP (path-vector) vs MyriadLS (link-state).

Question: during normal operation with links flapping (failing and recovering at
regular intervals), how much routing traffic does each protocol generate — in
message COUNT and in true BANDWIDTH (bytes)? Every PDU now carries a modelled
on-wire byte length (see MyriadPduSize.h), and Node already tallies bytes per
link, surfaced as the per-node `bytes_total` scalar; `sent_<type>`/`sent_total`
give the counts. We sum both across all nodes.

Config is built exactly like eval_scaling.py: a self-contained [General] ini that
`include`s eval_common.ini for the fair-timing knobs (refreshInterval, delays,
holdInterval, ...). We deliberately do NOT load myriadgp.ini/myriadls.ini — their
[Config Grid10] sections override eval_common (e.g. refreshInterval 15s vs 100s),
which would inflate GP's refresh churn ~6x and make the comparison meaningless.

Four runs on Grid10: {GP, LS} x {baseline (no failures), churn (periodic link
fail/recover)}. baseline = static-topology floor; churn - baseline = the
reconvergence overhead each protocol pays per unit of topology change. The
link-failure schedule is made IDENTICAL for both protocols by isolating the
dynamics module's RNG (num-rngs=2, dynamics.rng-0=1, fixed seed) so it can't be
perturbed by protocol-side randomness — same links, same times.

    python3 simulations/MYRIAD/eval_overhead.py

Outputs three plots + a summary table under simulations/MYRIAD/eval/overhead/.
"""
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
COMMON_INI = os.path.join(HERE, "eval_common.ini")
TOPO2RINGS = os.path.join(HERE, "topo2rings.py")
OUT = os.path.join(HERE, "eval", "overhead")

TOPO = os.path.join(SIM_ROOT, "topologies", "grid_10,10.txt")
SIMTIME = "150s"                 # initial convergence + several flap cycles
# Periodic link churn: fail NUM_LINKS random links every PERIOD for DURATION,
# then recover; repeats. DURATION < PERIOD so links are up between flaps.
NUM_LINKS = 4
PERIOD = "20s"
DURATION = "8s"

SENT_RE = re.compile(r"^scalar\s+\S+\s+sent_(\S+)\s+([0-9.]+)", re.M)
BYTES_RE = re.compile(r"^scalar\s+\S+\s+bytes_total\s+([0-9.]+)", re.M)
BYTES_TYPE_RE = re.compile(r"^scalar\s+\S+\s+bytes_(\S+)\s+([0-9.]+)", re.M)


def ring_label_lines():
    """Regenerate Grid10 ring labels (git-ignored) and return them inlined, so a
    clean checkout works and no per-config include path is needed."""
    stem = os.path.splitext(os.path.basename(TOPO))[0]
    labels = os.path.join(HERE, f"{stem}.ringlabels.ini")
    r = subprocess.run([sys.executable, TOPO2RINGS, TOPO, "--out-dir", HERE, "--no-png"],
                       capture_output=True, text=True)
    if not os.path.exists(labels):
        sys.exit(f"topo2rings.py failed:\n{r.stdout}{r.stderr}")
    with open(labels) as f:
        return [ln.rstrip("\n") for ln in f if ln.strip() and not ln.startswith("#")]


def write_ini(proto, ring_lines):
    """Self-contained ini: include eval_common.ini (fair-timing knobs), then only
    the per-run bits. No myriad{gp,ls}.ini -> no [Config] override of eval_common."""
    ini_path = os.path.join(OUT, f"{proto}.ini")
    lines = [
        f"include {COMMON_INI}",
        "[General]",
        f"ned-path={SRC};{SIM_ROOT}",
        f'**.protocols = "routingsim.routing.MYRIAD.myriad{proto}"',
        "network = routingsim.simulations.DefaultNetwork",
        f'*.manager.topologyFile = "{TOPO}"',
        "*.mainStatistics.samplingInterval = 0.01s",
        "**.mobility = false",
    ] + ring_lines
    with open(ini_path, "w") as f:
        f.write("\n".join(lines) + "\n")
    return ini_path


def measure(ini, resdir, churn):
    """Run one scenario; return (per_type_counts, total_msgs, total_bytes,
    per_type_bytes)."""
    for sca in glob.glob(resdir + "/*.sca"):
        os.remove(sca)
    dyn = [f'--**.dynamics.linkSelection="{"random" if churn else "none"}"']
    if churn:
        dyn += [f"--**.dynamics.numLinksFail={NUM_LINKS}",
                # linkFraction defaults to 0.01 and, if >0, OVERRIDES numLinksFail
                # in LinkFailure.cc — zero it so exactly NUM_LINKS links fail.
                "--**.dynamics.linkFraction=0",
                f"--**.dynamics.period={PERIOD}",
                f"--**.dynamics.duration={DURATION}",
                "--**.dynamics.oneFailure=false",
                # Isolate the dynamics RNG so both protocols see the same failures.
                "--num-rngs=2", "--**.dynamics.rng-0=1", "--seed-set=1"]
    cmd = [BIN, "-u", "Cmdenv", ini, f"--sim-time-limit={SIMTIME}", *dyn,
           "--result-dir", resdir]
    r = subprocess.run(cmd, cwd=SIM_ROOT, capture_output=True, text=True)
    # Under churn, "budget expired"/"ring broken" are the expected DEGRADE grace
    # path, not a bug — so only a nonzero exit code is fatal.
    if r.returncode != 0:
        sys.exit(f"FAIL: {os.path.basename(ini)} churn={churn}\n{(r.stdout + r.stderr)[-1500:]}")

    scas = glob.glob(resdir + "/*.sca")
    if not scas:
        sys.exit(f"FAIL: produced no .sca in {resdir}")
    per_type, per_type_bytes, total_bytes = {}, {}, 0
    for sca in scas:
        txt = open(sca).read()
        for m in SENT_RE.finditer(txt):
            t, v = m.group(1), int(float(m.group(2)))
            if t == "total":
                continue          # re-summed from parts below
            per_type[t] = per_type.get(t, 0) + v
        for m in BYTES_TYPE_RE.finditer(txt):
            t, v = m.group(1), int(float(m.group(2)))
            if t == "total":
                continue
            per_type_bytes[t] = per_type_bytes.get(t, 0) + v
        total_bytes += sum(int(float(m.group(1))) for m in BYTES_RE.finditer(txt))
    return per_type, sum(per_type.values()), total_bytes, per_type_bytes


def main():
    os.makedirs(OUT, exist_ok=True)
    if not os.path.exists(BIN):
        sys.exit(f"RoutingSim binary not found at {BIN} — build it first.")

    ring_lines = ring_label_lines()
    res = {}  # (proto, churn) -> (per_type, msgs, bytes)
    for proto in ("gp", "ls"):
        ini = write_ini(proto, ring_lines)
        for churn in (False, True):
            tag = f"{proto}_{'churn' if churn else 'base'}"
            res[(proto, churn)] = measure(ini, os.path.join(OUT, f"res_{tag}"), churn)

    # Summary table.
    print(f"\n=== Grid10: steady-state overhead over {SIMTIME} "
          f"(churn: {NUM_LINKS} links, period {PERIOD}, dur {DURATION}) ===")
    print(f"{'scenario':>12}{'messages':>12}{'bytes':>14}{'B/msg':>9}")
    for proto in ("gp", "ls"):
        for churn in (False, True):
            pt, m, b, _ = res[(proto, churn)]
            lbl = f"{proto.upper()} {'churn' if churn else 'base'}"
            print(f"{lbl:>12}{m:>12}{b:>14}{(b/m if m else 0):>9.1f}")
    for proto in ("gp", "ls"):
        base_m, base_b = res[(proto, False)][1], res[(proto, False)][2]
        ch_m, ch_b = res[(proto, True)][1], res[(proto, True)][2]
        print(f"  {proto.upper()} churn overhead (churn-base): "
              f"+{ch_m-base_m} msgs, +{ch_b-base_b} bytes")

    # Per-type breakdown (baseline, static topology): where the bytes actually go.
    for proto in ("gp", "ls"):
        counts, _, _, tbytes = res[(proto, False)]
        print(f"\n--- {proto.upper()} baseline per-type ---")
        print(f"{'type':>14}{'messages':>11}{'bytes':>14}{'B/msg':>9}{'%bytes':>8}")
        tot_b = sum(tbytes.values()) or 1
        for t in sorted(counts, key=lambda k: -tbytes.get(k, 0)):
            c, b = counts[t], tbytes.get(t, 0)
            print(f"{t:>14}{c:>11}{b:>14}{(b/c if c else 0):>9.0f}{100*b/tot_b:>7.1f}%")

    try:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
    except ImportError:
        print("\n(matplotlib not available — skipping plots)")
        return

    protos = ["gp", "ls"]
    labels = ["GP", "LS"]
    x = range(len(protos))
    w = 0.38

    def grouped(idx, ylabel, title, fname, scale=1.0):
        base = [res[(p, False)][idx] / scale for p in protos]
        churn = [res[(p, True)][idx] / scale for p in protos]
        fig, ax = plt.subplots()
        b1 = ax.bar([i - w/2 for i in x], base, w, label="baseline (static)", color="C0")
        b2 = ax.bar([i + w/2 for i in x], churn, w, label="with link churn", color="C3")
        ax.bar_label(b1, fmt="%.0f", fontsize=8)
        ax.bar_label(b2, fmt="%.0f", fontsize=8)
        ax.set_xticks(list(x)); ax.set_xticklabels(labels)
        ax.set(ylabel=ylabel, title=title)
        ax.legend(fontsize=8); fig.tight_layout()
        fig.savefig(os.path.join(OUT, fname), dpi=120)

    grouped(1, "total sent messages", f"Control-plane messages (Grid10, {SIMTIME})",
            "overhead_messages.png")
    grouped(2, "total bytes (KB)", f"Control-plane bandwidth (Grid10, {SIMTIME})",
            "overhead_bytes.png", scale=1000.0)

    # Per-type message breakdown under churn (which PDUs dominate each protocol).
    all_types = sorted({t for p in protos for t in res[(p, True)][0]})
    fig, ax = plt.subplots()
    for i, p in enumerate(protos):
        pt = res[(p, True)][0]
        bottom = 0
        for j, t in enumerate(all_types):
            v = pt.get(t, 0)
            if not v:
                continue
            ax.bar(i, v, 0.5, bottom=bottom, color=f"C{j}",
                   label=t if i == 0 else None)
            bottom += v
    ax.set_xticks(list(x)); ax.set_xticklabels(labels)
    ax.set(ylabel="sent messages (churn run)", title="Message mix under churn (Grid10)")
    ax.legend(fontsize=8, title="PDU type"); fig.tight_layout()
    fig.savefig(os.path.join(OUT, "overhead_bytype.png"), dpi=120)

    print(f"\nplots -> {OUT}/{{overhead_messages,overhead_bytes,overhead_bytype}}.png")


if __name__ == "__main__":
    main()
