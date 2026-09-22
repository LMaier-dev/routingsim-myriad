#!/usr/bin/env python3
"""Convergence comparison: MyriadPV (batching sweep) vs MyriadLS (reference).

The oracle test proves both protocols reach the same optimum. This measures how
fast and how cheaply each gets there. PV is swept over the batching holdInterval
{0s..1s}; LS has no such knob so it's a single reference run. Every run enables
observational RIB snapshots (snapshotInterval), which are provably non-intrusive
(identical message counts / final RIB with snapshots off), so the measurement
doesn't perturb convergence.

Per run we parse the timestamped snapshots into {t: {(node,origin): cost}} and
derive: convergence time (last t at which the RIB changed), the final %-optimal
(vs the LS final costs = the proven optimum), and total sent messages.

    python3 simulations/MYRIAD/eval_convergence.py [--topo PATH] [--basis {min,tree}]

Outputs a plot + a summary table under simulations/MYRIAD/eval/convergence/.
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
                          repeat_mean, repeats_note, run_sim, simtime, title, warmup_s,
                          write_proto_ini, add_topo_args, PROTO_COLOR, PROTO_LABEL,
                          pyplot)

HERE = str(MYRIAD_DIR)
OUT = os.path.join(HERE, "eval", "convergence")

# PV batching sweep, strict geometric x4 from 1us to 262144us; 0s = batching off
# (baseline). Straddles the per-hop delay spread (eval_common: 0.01-0.25ms).
# 1s is left out: it sits at the working ceiling (hold>=~1.5s livelocks on
# Grid10 as overlapping refresh generations compound, 2s never converges), so it
# reports the edge of the livelock rather than the batching tradeoff the sweep
# is about.
HOLDS = ["0s", "1us", "4us", "16us", "64us", "256us", "1024us", "4096us",
         "16384us", "65536us", "262144us"]

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
    messages, final %optimal. Raw snapshots are NOT averaged — mean costs would be
    fractional and break the equality diffing in convergence_time()."""
    ct = convergence_time(snaps)
    return {"ct": (ct - shift) if ct is not None else None,
            "msgs": msgs,
            "fo": frac_optimal(snaps[max(snaps)], truth) * 100}


def main(args=None):
    if args is None:
        p = argparse.ArgumentParser()
        add_topo_args(p)
        args = p.parse_args()
        apply_repeat_args(args)

    # Scope OUT to topology (+ optional --tag), like eval_dynamics: otherwise
    # two topologies — or two configs on one topology — overwrite each other.
    global OUT
    OUT = os.path.join(OUT, os.path.splitext(os.path.basename(args.topo))[0]
                       + (f"_{args.tag}" if getattr(args, "tag", None) else ""))
    os.makedirs(OUT, exist_ok=True)
    if not os.path.exists(BIN):
        sys.exit(f"RoutingSim binary not found at {BIN} — build it first.")

    labels = ensure_ring_labels(args.topo, args.basis,
                                getattr(args, 'max_ring_size', 0),
                                not getattr(args, 'no_echo_rings', False))
    pv_ini = write_proto_ini(os.path.join(OUT, "pv.ini"), "pv",
                             args.topo, labels, update_jitter=args.update_jitter)
    ls_ini = write_proto_ini(os.path.join(OUT, "ls.ini"), "ls", args.topo, labels)
    announce("convergence", opts=overrides(sim_time_limit=SIMTIME))

    # LS reference; its final costs are the proven optimum (truth for %optimal).
    # The optimum is seed-independent, so rep 0 alone defines truth; the other
    # repeats only contribute LS's own conv time / messages.
    print("  → LS reference …", flush=True)
    ls0 = run_proto(ls_ini, "oracleDumpFile", os.path.join(OUT, "ls.txt"),
                    os.path.join(OUT, "res_ls"), [], 0)
    truth = ls0[0][max(ls0[0])]
    ls = repeat_mean(lambda rep: derive(*(ls0 if rep == 0 else run_proto(
        ls_ini, "oracleDumpFile", os.path.join(OUT, "ls.txt"),
        os.path.join(OUT, "res_ls"), [], rep)), truth))
    ls_ct, ls_msgs = ls["ct"], ls["msgs"]
    print(f"  ✓ LS done: conv={ls_ct}s, msgs={ls_msgs:.0f}")

    # PV over the holdInterval sweep. rankUseSpof=false ranks purely by cost, so
    # PV's converged RIB targets the same optimum LS computes (see the oracle test)
    # — required for the %-optimal numbers to be meaningful (else spof-first ranking
    # picks costlier redundant paths and never matches the pure-cost truth).
    pv = {}  # hold -> {"ct", "msgs", "fo"}
    for h in HOLDS:
        print(f"  → PV hold={h} …", flush=True)
        dump = os.path.join(OUT, f"pv_{h}.txt")
        pv[h] = repeat_mean(lambda rep, h=h, dump=dump: derive(
            *run_proto(pv_ini, "ribDumpFile", dump,
                       os.path.join(OUT, f"res_pv_{h}"),
                       overrides(holdInterval=h, rankUseSpof="false"), rep),
            truth, warmup_s()))
        print(f"  ✓ hold={h}: conv={pv[h]['ct']}s, msgs={pv[h]['msgs']:.0f}, "
              f"{pv[h]['fo']:.1f}% optimal")

    # Summary table.
    print(f"\n=== convergence vs holdInterval "
          f"(LS ref: conv={ls_ct:.1f}s, msgs={ls_msgs:.0f}, {len(truth)} pairs)"
          f"{repeats_note()} ===")
    print(f"  PV warmup={common('collectWarmupDuration')} (subtracted from PV conv times)")
    print(f"{'hold':>8}{'conv_time':>16}{'sent_total':>18}{'final %opt':>12}")

    def row(label, r):
        ct, msgs = r["ct"], r["msgs"]
        cts = ("-" if ct is None else f"{ct:.1f}{pm(ct)}s")
        print(f"{label:>8}{cts:>16}{f'{msgs:.0f}' + pm(msgs, '.0f'):>18}"
              f"{r['fo']:>11.1f}%")

    for h in HOLDS:
        row(h, pv[h])
    row("LS", ls)

    try:
        plt = pyplot()
    except ImportError:
        print("\n(matplotlib not available — skipping plots)")
        return

    geo = [h for h in HOLDS if _to_seconds(h) > 0]
    gx = [_to_seconds(h) for h in geo]

    # One panel per metric rather than a second y-axis: seconds and message
    # counts share no scale, and twinning them forces one range to be read
    # against the other's gridlines.
    fig, (axT, axB) = plt.subplots(2, 1, sharex=True, figsize=(7, 6.5),
                                   layout="constrained")
    axT.set_xscale("log")
    # Colour is the protocol, never the metric: the panel says which metric.
    pv_c, ls_c = PROTO_COLOR["pv"], PROTO_COLOR["ls"]
    pv_line = axT.plot(gx, [pv[h]["ct"] for h in geo], "o-", color=pv_c,
                       label=PROTO_LABEL["pv"])[0]
    ls_line = axT.axhline(ls_ct, ls="--", color=ls_c, label=PROTO_LABEL["ls"])
    axB.plot(gx, [pv[h]["msgs"] / 1e3 for h in geo], "o-", color=pv_c)
    axB.axhline(ls_msgs / 1e3, ls="--", color=ls_c)

    # The reference values belong on the lines, not in the legend, so the
    # legend stays about identity.
    axT.annotate(f"{ls_ct:.0f}s", xy=(gx[0], ls_ct), xytext=(0, 4),
                 textcoords="offset points", color=ls_c, fontsize=10)
    axB.annotate(f"{ls_msgs / 1e3:.0f}k", xy=(gx[0], ls_msgs / 1e3), xytext=(0, 4),
                 textcoords="offset points", color=ls_c, fontsize=10)

    # Where the rest of the evaluation runs, so the reader can price the choice
    # off the two curves. Not a measured crossover: convergence is sampled at
    # snapshotInterval, so these times are whole seconds and the point at which
    # PV crosses LS falls between two swept holds.
    cfg = _to_seconds(common("holdInterval", "50ms"))
    if gx[0] <= cfg <= gx[-1]:
        for ax in (axT, axB):
            ax.axvline(cfg, color="0.35", lw=1, ls=":", zorder=0)
        axT.annotate(f"configured: {cfg * 1e3:g} ms", xy=(cfg, 1),
                     xycoords=("data", "axes fraction"),
                     xytext=(-5, -12), textcoords="offset points",
                     ha="right", va="top", color="0.35", fontsize=10)

    axB.set_xlabel("holdInterval (s, log scale)")
    axT.set_ylabel("convergence time (s)")
    axB.set_ylabel("sent messages (thousands)")
    axT.set_title(title("Convergence time and message count vs hold interval"))
    axT.legend(handles=[pv_line, ls_line], loc="upper left", frameon=False)
    for ax in (axT, axB):
        ax.grid(axis="y", color="0.9", lw=0.8)
        ax.set_axisbelow(True)
        ax.margins(y=0.14)
        for side in ("top", "right"):
            ax.spines[side].set_visible(False)
    fig.savefig(os.path.join(OUT, "tradeoff.png"), dpi=200)

    print(f"\nplots -> {OUT}/tradeoff.png")


if __name__ == "__main__":
    main()
