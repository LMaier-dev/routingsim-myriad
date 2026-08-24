#!/usr/bin/env python3
"""Steady-state control-plane overhead: MyriadGP (path-vector) vs MyriadLS (link-state).

Question: during normal operation with links flapping (failing and recovering at
regular intervals), how much routing traffic does each protocol generate — in
message COUNT and in true BANDWIDTH (bytes)? Every PDU now carries a modelled
on-wire byte length (see MyriadPduSize.h), and Node already tallies bytes per
link, surfaced as the per-node `bytes_total` scalar; `sent_<type>`/`sent_total`
give the counts, and `bytes_max`/`bytes_max_<type>` give the largest single PDU
seen (per node). We sum the counts/totals and take the max of the maxes across
all nodes.

Config is built exactly like eval_scaling.py: a self-contained [General] ini that
`include`s eval_common.ini for the fair-timing knobs (refreshInterval, delays,
holdInterval, ...). We deliberately do NOT load myriad{gp,ls}.ini — their
[Config Grid10] sections override eval_common (e.g. refreshInterval 15s vs 100s),
which would inflate GP's refresh churn ~6x and make the comparison meaningless.

Four runs: {GP, LS} x {baseline (no failures), churn (periodic link fail/recover)}.
baseline = static-topology floor; churn - baseline = the reconvergence overhead
each protocol pays per unit of topology change. The link-failure schedule is made
IDENTICAL for both protocols by isolating the dynamics module's RNG.

    python3 simulations/MYRIAD/eval_overhead.py [--topo PATH] [--basis {min,all}]

Outputs three plots + a summary table under simulations/MYRIAD/eval/overhead/.
"""
import argparse
import glob
import os
import re
import sys

from eval_harness import (BIN, SIM_ROOT, MYRIAD_DIR,
                          ensure_ring_labels, run_sim, write_proto_ini, add_topo_args)

HERE = str(MYRIAD_DIR)
OUT = os.path.join(HERE, "eval", "overhead")

SIMTIME = "150s"                 # initial convergence + several flap cycles
# Periodic link churn: fail NUM_LINKS random links every PERIOD for DURATION,
# then recover; repeats. DURATION < PERIOD so links are up between flaps.
NUM_LINKS = 4
PERIOD = "20s"
DURATION = "8s"

SENT_RE = re.compile(r"^scalar\s+\S+\s+sent_(\S+)\s+([0-9.]+)", re.M)
BYTES_RE = re.compile(r"^scalar\s+\S+\s+bytes_total\s+([0-9.]+)", re.M)
BYTES_TYPE_RE = re.compile(r"^scalar\s+\S+\s+bytes_(\S+)\s+([0-9.]+)", re.M)
BYTES_MAX_RE = re.compile(r"^scalar\s+\S+\s+bytes_max\s+([0-9.]+)", re.M)
BYTES_MAX_TYPE_RE = re.compile(r"^scalar\s+\S+\s+bytes_max_(\S+)\s+([0-9.]+)", re.M)


def measure(ini, resdir, churn):
    """Run one scenario; return (per_type_counts, total_msgs, total_bytes,
    per_type_bytes, max_bytes, per_type_max_bytes)."""
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
    run_sim(cmd, allow_budget_expired=True)

    scas = glob.glob(resdir + "/*.sca")
    if not scas:
        sys.exit(f"FAIL: produced no .sca in {resdir}")
    per_type, per_type_bytes, total_bytes = {}, {}, 0
    per_type_max_bytes, max_bytes = {}, 0
    for sca in scas:
        txt = open(sca).read()
        for m in SENT_RE.finditer(txt):
            t, v = m.group(1), int(float(m.group(2)))
            if t == "total":
                continue
            per_type[t] = per_type.get(t, 0) + v
        for m in BYTES_TYPE_RE.finditer(txt):
            t, v = m.group(1), int(float(m.group(2)))
            if t == "total" or t.startswith("max"):
                continue
            per_type_bytes[t] = per_type_bytes.get(t, 0) + v
        total_bytes += sum(int(float(m.group(1))) for m in BYTES_RE.finditer(txt))
        for m in BYTES_MAX_TYPE_RE.finditer(txt):
            t, v = m.group(1), int(float(m.group(2)))
            per_type_max_bytes[t] = max(per_type_max_bytes.get(t, 0), v)
        max_bytes = max([max_bytes] + [int(float(m.group(1))) for m in BYTES_MAX_RE.finditer(txt)])
    return per_type, sum(per_type.values()), total_bytes, per_type_bytes, max_bytes, per_type_max_bytes


def main(args=None):
    if args is None:
        p = argparse.ArgumentParser()
        add_topo_args(p)
        args = p.parse_args()

    os.makedirs(OUT, exist_ok=True)
    if not os.path.exists(BIN):
        sys.exit(f"RoutingSim binary not found at {BIN} — build it first.")

    labels = ensure_ring_labels(args.topo, args.basis)
    res = {}
    for proto in ("gp", "ls"):
        ini = write_proto_ini(os.path.join(OUT, f"{proto}.ini"), proto,
                              args.topo, labels,
                              update_jitter=args.update_jitter if proto == "gp" else None,
                              warmup=args.warmup if proto == "gp" else None)
        for churn in (False, True):
            tag = f"{proto}_{'churn' if churn else 'base'}"
            print(f"  → {proto.upper()} {'churn' if churn else 'baseline'} …", flush=True)
            res[(proto, churn)] = measure(ini, os.path.join(OUT, f"res_{tag}"), churn)
            print(f"  ✓ done ({res[(proto, churn)][1]} msgs, "
                  f"{res[(proto, churn)][2]//1000} KB)")

    # Summary table.
    print(f"\n=== steady-state overhead over {SIMTIME} "
          f"(churn: {NUM_LINKS} links, period {PERIOD}, dur {DURATION}) ===")
    print(f"{'scenario':>12}{'messages':>12}{'bytes':>14}{'B/msg':>9}{'max B':>9}")
    protos = ["gp", "ls"]
    for proto in protos:
        for churn in (False, True):
            pt, m, b, _, mx, _ = res[(proto, churn)]
            lbl = f"{proto.upper()} {'churn' if churn else 'base'}"
            print(f"{lbl:>12}{m:>12}{b:>14}{(b/m if m else 0):>9.1f}{mx:>9}")
    for proto in protos:
        base_m, base_b = res[(proto, False)][1], res[(proto, False)][2]
        ch_m, ch_b = res[(proto, True)][1], res[(proto, True)][2]
        print(f"  {proto.upper()} churn overhead (churn-base): "
              f"+{ch_m-base_m} msgs, +{ch_b-base_b} bytes")

    for proto in protos:
        counts, _, _, tbytes, _, tmax = res[(proto, False)]
        print(f"\n--- {proto.upper()} baseline per-type ---")
        print(f"{'type':>14}{'messages':>11}{'bytes':>14}{'B/msg':>9}{'max B':>8}{'%bytes':>8}")
        tot_b = sum(tbytes.values()) or 1
        for t in sorted(counts, key=lambda k: -tbytes.get(k, 0)):
            c, b, mx = counts[t], tbytes.get(t, 0), tmax.get(t, 0)
            print(f"{t:>14}{c:>11}{b:>14}{(b/c if c else 0):>9.0f}{mx:>8}{100*b/tot_b:>7.1f}%")

    try:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
    except ImportError:
        print("\n(matplotlib not available — skipping plots)")
        return

    labels_list = ["GP", "LS"]
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
        ax.set_xticks(list(x)); ax.set_xticklabels(labels_list)
        ax.set(ylabel=ylabel, title=title)
        ax.legend(fontsize=8); fig.tight_layout()
        fig.savefig(os.path.join(OUT, fname), dpi=120)

    grouped(1, "total sent messages", f"Control-plane messages ({SIMTIME})",
            "overhead_messages.png")
    grouped(2, "total bytes (KB)", f"Control-plane bandwidth ({SIMTIME})",
            "overhead_bytes.png", scale=1000.0)

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
    ax.set_xticks(list(x)); ax.set_xticklabels(labels_list)
    ax.set(ylabel="sent messages (churn run)", title="Message mix under churn")
    ax.legend(fontsize=8, title="PDU type"); fig.tight_layout()
    fig.savefig(os.path.join(OUT, "overhead_bytype.png"), dpi=120)

    print(f"\nplots -> {OUT}/{{overhead_messages,overhead_bytes,overhead_bytype}}.png")


if __name__ == "__main__":
    main()
