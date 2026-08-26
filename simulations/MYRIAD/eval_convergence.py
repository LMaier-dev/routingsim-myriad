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

from eval_harness import (BIN, SIM_ROOT, MYRIAD_DIR, announce, apply_repeat_args,
                          common, ensure_ring_labels, overrides, pm, rep_path,
                          repeat_mean, repeats_note, run_sim, simtime, warmup_s,
                          write_proto_ini, add_topo_args)

HERE = str(MYRIAD_DIR)
OUT = os.path.join(HERE, "eval", "convergence")

# GP batching sweep, strict geometric x4 from 1us to 1s; 0s = batching off
# (baseline). Straddles the per-hop delay spread (eval_common: 0.01-0.25ms) and
# runs up to the ~1s working ceiling (hold>=~1.5s livelocks on Grid10 as
# overlapping refresh generations compound; 2s never converges).
HOLDS = ["0s", "1us", "4us", "16us", "64us", "256us", "1024us", "4096us",
         "16384us", "65536us", "262144us", "1s"]

# Warmup + one refresh cycle: convergence lands well inside the first cycle, the
# rest is headroom for a refresh generation to heal first-round transients.
SIMTIME = simtime(cycles=1)


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


def run_proto(ini, dump_param, dumpfile, resdir, extra, rep=0):
    dumpfile, resdir = rep_path(dumpfile, rep), rep_path(resdir, rep)
    if os.path.exists(dumpfile):
        os.remove(dumpfile)
    cmd = [BIN, "-u", "Cmdenv", ini, *overrides(sim_time_limit=SIMTIME),
           *extra, f'--**.{dump_param}="{dumpfile}"', "--result-dir", resdir]
    run_sim(cmd, rep=rep)
    if not os.path.exists(dumpfile):
        sys.exit(f"FAIL: produced no snapshot dump ({dumpfile})")
    return parse_snapshots(dumpfile), _sum_sent_total(resdir)


def derive(snaps, msgs, truth, shift=0.0):
    """Per-run numbers that are safe to average across repeats: convergence time,
    messages, final %optimal, and the %optimal-vs-time curve on the (fixed)
    snapshot grid. Raw snapshots are NOT averaged — mean costs would be
    fractional and break the equality diffing in convergence_time()."""
    ct = convergence_time(snaps)
    return {"ct": (ct - shift) if ct is not None else None,
            "msgs": msgs,
            "fo": frac_optimal(snaps[max(snaps)], truth) * 100,
            "curve": {t - shift: frac_optimal(s, truth) * 100
                      for t, s in snaps.items()}}


def main(args=None):
    if args is None:
        p = argparse.ArgumentParser()
        add_topo_args(p)
        args = p.parse_args()
        apply_repeat_args(args)

    os.makedirs(OUT, exist_ok=True)
    if not os.path.exists(BIN):
        sys.exit(f"RoutingSim binary not found at {BIN} — build it first.")

    labels = ensure_ring_labels(args.topo, args.basis)
    gp_ini = write_proto_ini(os.path.join(OUT, "gp.ini"), "gp",
                             args.topo, labels, update_jitter=args.update_jitter)
    ls_ini = write_proto_ini(os.path.join(OUT, "ls.ini"), "ls", args.topo, labels)
    announce("convergence", opts=overrides(sim_time_limit=SIMTIME))

    # LS reference; its final costs are the proven optimum (truth for the curves).
    # The optimum is seed-independent, so rep 0 alone defines truth; the other
    # repeats only contribute LS's own conv time / messages / curve.
    print("  → LS reference …", flush=True)
    ls0 = run_proto(ls_ini, "oracleDumpFile", os.path.join(OUT, "ls.txt"),
                    os.path.join(OUT, "res_ls"), [], 0)
    truth = ls0[0][max(ls0[0])]
    ls = repeat_mean(lambda rep: derive(*(ls0 if rep == 0 else run_proto(
        ls_ini, "oracleDumpFile", os.path.join(OUT, "ls.txt"),
        os.path.join(OUT, "res_ls"), [], rep)), truth))
    ls_ct, ls_msgs = ls["ct"], ls["msgs"]
    print(f"  ✓ LS done: conv={ls_ct}s, msgs={ls_msgs:.0f}")

    # GP over the holdInterval sweep. rankUseSpof=false ranks purely by cost, so
    # GP's converged RIB targets the same optimum LS computes (see the oracle test)
    # — required for the %-optimal curve to be meaningful (else spof-first ranking
    # picks costlier redundant paths and never matches the pure-cost truth).
    gp = {}  # hold -> {"ct", "msgs", "fo", "curve"}
    for h in HOLDS:
        print(f"  → GP hold={h} …", flush=True)
        dump = os.path.join(OUT, f"gp_{h}.txt")
        gp[h] = repeat_mean(lambda rep, h=h, dump=dump: derive(
            *run_proto(gp_ini, "ribDumpFile", dump,
                       os.path.join(OUT, f"res_gp_{h}"),
                       overrides(holdInterval=h, rankUseSpof="false"), rep),
            truth, warmup_s()))
        print(f"  ✓ hold={h}: conv={gp[h]['ct']}s, msgs={gp[h]['msgs']:.0f}, "
              f"{gp[h]['fo']:.1f}% optimal")

    # Summary table.
    print(f"\n=== convergence vs holdInterval "
          f"(LS ref: conv={ls_ct:.1f}s, msgs={ls_msgs:.0f}, {len(truth)} pairs)"
          f"{repeats_note()} ===")
    print(f"  GP warmup={common('collectWarmupDuration')} (subtracted from GP conv times)")
    print(f"{'hold':>8}{'conv_time':>16}{'sent_total':>18}{'final %opt':>12}")

    def row(label, r):
        ct, msgs = r["ct"], r["msgs"]
        cts = ("-" if ct is None else f"{ct:.1f}{pm(ct)}s")
        print(f"{label:>8}{cts:>16}{f'{msgs:.0f}' + pm(msgs, '.0f'):>18}"
              f"{r['fo']:>11.1f}%")

    for h in HOLDS:
        row(h, gp[h])
    row("LS", ls)

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
        axL.plot(gx, [gp[h]["ct"] for h in geo], "o-", color="C0",
                 label="GP convergence time")[0],
        axR.plot(gx, [gp[h]["msgs"] for h in geo], "s-", color="C3",
                 label="GP sent messages")[0],
    ]
    if has_off:
        lines.append(axR.axhline(gp["0s"]["msgs"], ls=":", color="C3", alpha=.6,
                                 label="GP messages, no batching (0s)"))
    lines.append(axL.axhline(ls_ct, ls="--", color="C0", alpha=.5,
                             label=f"LS convergence ({ls_ct:.1f}s)"))
    lines.append(axR.axhline(ls_msgs, ls="--", color="C3", alpha=.5,
                             label=f"LS messages ({ls_msgs:.0f})"))
    axL.set(xlabel="holdInterval (s, log scale)", ylabel="convergence time (s)")
    axR.set_ylabel("total sent messages")
    axL.set_title("Batching tradeoff: convergence time vs messages")
    axL.legend(handles=lines, fontsize=7, loc="center left")
    fig.tight_layout()
    fig.savefig(os.path.join(OUT, "tradeoff.png"), dpi=120)

    fig, ax = plt.subplots()
    for h in HOLDS:
        curve = gp[h]["curve"]
        ts = sorted(curve)
        ax.plot(ts, [curve[t] for t in ts], "-", label=f"GP hold={h}")
    ts = sorted(ls["curve"])
    ax.plot(ts, [ls["curve"][t] for t in ts], "k--", label="LS")
    ax.set(xlabel="sim time (s)", ylabel="% of optimal RIB reached",
           title="Convergence curve")
    ax.legend(fontsize=8)
    fig.savefig(os.path.join(OUT, "optimal_vs_time.png"), dpi=120)

    print(f"\nplots -> {OUT}/{{tradeoff,optimal_vs_time}}.png")


if __name__ == "__main__":
    main()
