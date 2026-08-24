#!/usr/bin/env python3
"""Convergence comparison: MyriadGP (batching sweep) vs MyriadLS (reference).

The oracle test proves both protocols reach the same optimum. This measures how
fast and how cheaply each gets there. GP is swept over the batching holdInterval
{0s..1s}; LS has no such knob so it's a single reference run. Every run enables
observational RIB snapshots (snapshotInterval), which are provably non-intrusive
(identical message counts / final RIB with snapshots off), so the measurement
doesn't perturb convergence.

Per run we parse the timestamped snapshots into {t: {(node,origin): cost}} and
derive: convergence time (last t at which the RIB changed), the %-optimal-vs-time
curve (vs the LS final costs = the proven optimum), and total sent messages.

    python3 simulations/MYRIAD/eval_convergence.py [--topo PATH] [--basis {min,all}]

Outputs two plots + a summary table under simulations/MYRIAD/eval/convergence/.
Convergence is diffed on COST (refresh-invariant once ring sizes are known), so
king re-origination bumps seq/chain without falsely resetting convergence.
"""
import argparse
import glob
import os
import re
import sys

from eval_harness import (BIN, SIM_ROOT, MYRIAD_DIR,
                          ensure_ring_labels, run_sim, write_proto_ini, add_topo_args)

HERE = str(MYRIAD_DIR)
OUT = os.path.join(HERE, "eval", "convergence")

# GP batching sweep, strict geometric x4 from 1us to 1s; 0s = batching off
# (baseline). Straddles the per-hop delay spread (eval_common: 0.01-0.25ms) and
# runs up to the ~1s working ceiling (hold>=~1.5s livelocks on Grid10 as
# overlapping refresh generations compound; 2s never converges).
HOLDS = ["0s", "1us", "4us", "16us", "64us", "256us", "1024us", "4096us",
         "16384us", "65536us", "262144us", "1s"]
SNAP = "2s"
SIMTIME = "120s"     # warmup=10s + convergence + one refresh cycle headroom
# Warmup: run COLLECTs without UPDATEs so memberCounts are pre-populated before
# the measurement window starts. Eliminates the ~100s refresh-cycle artifact
# (some inter-ring UPDATEs dropped on first round if local COLLECT hasn't set
# memberCount yet). Convergence times are reported relative to warmup end.
GP_WARMUP = "10s"

# Fast refresh so several cycles complete within SIMTIME; eval_common defaults
# to 100s which would leave only one cycle — not enough to heal transients.
_REFRESH_LINES = ["**.refreshInterval = 15s", "**.refreshJitter = 0s"]


def _to_seconds(s):
    for suf, div in (("ms", 1e3), ("us", 1e6), ("ns", 1e9)):
        if s.endswith(suf):
            return float(s[:-2]) / div
    return float(s.rstrip("s"))


def _sum_sent_total(resdir):
    rx = re.compile(r"^scalar\s+\S+\s+sent_total\s+([0-9.]+)", re.M)
    return sum(int(float(m.group(1))) for sca in glob.glob(resdir + "/*.sca")
               for m in rx.finditer(open(sca).read()))


def parse_snapshots(path):
    """Timestamped dump -> {t(float): {(node,'origin=r'): cost}}."""
    snaps = {}
    for line in open(path):
        line = line.strip()
        if not line.startswith("t="):
            continue
        t_tok, rest = line.split(" ", 1)
        toks = rest.split()
        node = toks[0]
        kv = dict(p.split("=", 1) for p in toks[1:]
                  if "=" in p and not p.startswith("chain="))
        snaps.setdefault(float(t_tok[2:]), {})[(node, f"origin={kv['origin']}")] = int(kv["cost"])
    return snaps


def convergence_time(snaps):
    """Last t at which the RIB (cost per (node,origin)) differed from the prior
    snapshot -> the moment after which everything is stable."""
    ts = sorted(snaps)
    last_change = ts[0] if ts else None
    for i in range(1, len(ts)):
        if snaps[ts[i]] != snaps[ts[i - 1]]:
            last_change = ts[i]
    return last_change


def frac_optimal(snap, truth):
    """Fraction of truth (node,origin) pairs present and at optimal cost."""
    return sum(1 for k, v in truth.items() if snap.get(k) == v) / len(truth) if truth else 0.0


def run_proto(ini, dump_param, dumpfile, resdir, extra):
    if os.path.exists(dumpfile):
        os.remove(dumpfile)
    cmd = [BIN, "-u", "Cmdenv", ini,
           f"--sim-time-limit={SIMTIME}", f"--**.snapshotInterval={SNAP}",
           *extra, f'--**.{dump_param}="{dumpfile}"', "--result-dir", resdir]
    run_sim(cmd)
    if not os.path.exists(dumpfile):
        sys.exit(f"FAIL: produced no snapshot dump ({dumpfile})")
    return parse_snapshots(dumpfile), _sum_sent_total(resdir)


def main(args=None):
    if args is None:
        p = argparse.ArgumentParser()
        add_topo_args(p)
        args = p.parse_args()

    os.makedirs(OUT, exist_ok=True)
    if not os.path.exists(BIN):
        sys.exit(f"RoutingSim binary not found at {BIN} — build it first.")

    labels = ensure_ring_labels(args.topo, args.basis)
    gp_ini = write_proto_ini(os.path.join(OUT, "gp.ini"), "gp",
                             args.topo, labels, extra_lines=_REFRESH_LINES,
                             update_jitter=args.update_jitter)
    ls_ini = write_proto_ini(os.path.join(OUT, "ls.ini"), "ls",
                             args.topo, labels, extra_lines=_REFRESH_LINES)

    # LS reference; its final costs are the proven optimum (truth for the curves).
    print("  → LS reference …", flush=True)
    ls_snaps, ls_msgs = run_proto(ls_ini, "oracleDumpFile",
                                  os.path.join(OUT, "ls.txt"),
                                  os.path.join(OUT, "res_ls"), [])
    truth = ls_snaps[max(ls_snaps)]
    ls_ct = convergence_time(ls_snaps)
    print(f"  ✓ LS done: conv={ls_ct}s, msgs={ls_msgs}")

    # GP over the holdInterval sweep. rankUseSpof=false ranks purely by cost, so
    # GP's converged RIB targets the same optimum LS computes (see the oracle test)
    # — required for the %-optimal curve to be meaningful (else spof-first ranking
    # picks costlier redundant paths and never matches the pure-cost truth).
    gp = {}  # hold -> (snaps, msgs, conv_time)
    warmup = args.warmup or GP_WARMUP        # harness override; NED default is 10s
    if warmup[-1].isdigit():
        warmup += "s"
    warmup_s = _to_seconds(warmup)
    for h in HOLDS:
        print(f"  → GP hold={h} …", flush=True)
        dump = os.path.join(OUT, f"gp_{h}.txt")
        snaps, msgs = run_proto(gp_ini, "ribDumpFile", dump,
                                os.path.join(OUT, f"res_gp_{h}"),
                                [f"--**.holdInterval={h}", "--**.rankUseSpof=false",
                                 f"--**.collectWarmupDuration={warmup}"])
        raw_ct = convergence_time(snaps)
        ct = (raw_ct - warmup_s) if raw_ct is not None else None
        fo = frac_optimal(snaps[max(snaps)], truth) * 100
        gp[h] = (snaps, msgs, ct)
        print(f"  ✓ hold={h}: conv={ct}s, msgs={msgs}, {fo:.1f}% optimal")

    # Summary table.
    print(f"\n=== convergence vs holdInterval "
          f"(LS ref: conv={ls_ct}s, msgs={ls_msgs}, {len(truth)} pairs) ===")
    print(f"  GP warmup={warmup} (subtracted from GP conv times)")
    print(f"{'hold':>8}{'conv_time':>12}{'sent_total':>14}{'final %opt':>12}")
    for h in HOLDS:
        snaps, msgs, ct = gp[h]
        fo = frac_optimal(snaps[max(snaps)], truth) * 100
        print(f"{h:>8}{str(ct)+'s':>12}{msgs:>14}{fo:>11.1f}%")
    print(f"{'LS':>8}{str(ls_ct)+'s':>12}{ls_msgs:>14}"
          f"{frac_optimal(ls_snaps[max(ls_snaps)], truth)*100:>11.1f}%")

    try:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
    except ImportError:
        print("\n(matplotlib not available — skipping plots)")
        return

    geo = [h for h in HOLDS if _to_seconds(h) > 0]
    gx = [_to_seconds(h) for h in geo]
    has_off = "0s" in HOLDS

    fig, axL = plt.subplots()
    axR = axL.twinx()
    axL.set_xscale("log")
    lines = [
        axL.plot(gx, [gp[h][2] for h in geo], "o-", color="C0",
                 label="GP convergence time")[0],
        axR.plot(gx, [gp[h][1] for h in geo], "s-", color="C3",
                 label="GP sent messages")[0],
    ]
    if has_off:
        lines.append(axR.axhline(gp["0s"][1], ls=":", color="C3", alpha=.6,
                                 label="GP messages, no batching (0s)"))
    lines.append(axL.axhline(ls_ct, ls="--", color="C0", alpha=.5,
                             label=f"LS convergence ({ls_ct}s)"))
    lines.append(axR.axhline(ls_msgs, ls="--", color="C3", alpha=.5,
                             label=f"LS messages ({ls_msgs})"))
    axL.set(xlabel="holdInterval (s, log scale)", ylabel="convergence time (s)")
    axR.set_ylabel("total sent messages")
    axL.set_title("Batching tradeoff: convergence time vs messages")
    axL.legend(handles=lines, fontsize=7, loc="center left")
    fig.tight_layout()
    fig.savefig(os.path.join(OUT, "tradeoff.png"), dpi=120)

    fig, ax = plt.subplots()
    for h in HOLDS:
        snaps = gp[h][0]
        ts = sorted(snaps)
        ax.plot([t - warmup_s for t in ts], [frac_optimal(snaps[t], truth) * 100 for t in ts],
                "-", label=f"GP hold={h}")
    ts = sorted(ls_snaps)
    ax.plot(ts, [frac_optimal(ls_snaps[t], truth) * 100 for t in ts], "k--", label="LS")
    ax.set(xlabel="sim time (s)", ylabel="% of optimal RIB reached",
           title="Convergence curve")
    ax.legend(fontsize=8)
    fig.savefig(os.path.join(OUT, "optimal_vs_time.png"), dpi=120)

    print(f"\nplots -> {OUT}/{{tradeoff,optimal_vs_time}}.png")


if __name__ == "__main__":
    main()
