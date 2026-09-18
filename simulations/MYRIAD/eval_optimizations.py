#!/usr/bin/env python3
"""Impact of the PV propagation optimizations, each switched on alone against an
all-off baseline, then all together. One fan level of the ringpipe fan sweep,
averaged over every seed of that level; LS runs once per seed as the reference.

Every run covers the warmup plus two refresh cycles with refreshFloodsUpdate=true,
so the totals include what the refreshes re-flood, not only the formation burst.
ecmpCostMargin is 50% in every PV config. maxEcmpPaths counts total paths; the
baseline's 1 is the "0 slots" case (0 behaves like 1: the first chain is always
admitted). spofHoldMultiplier=1 is the NED's "off" for the spof bucket. ECMP
slots are filled by cost alone here: eval_common pins ecmpDiverseEviction off,
and no config turns it on.

    python3 simulations/MYRIAD/eval_entrypoint.py optimizations --jobs 2   # as eval_runs.txt runs it
    python3 simulations/MYRIAD/eval_optimizations.py --dry-run
    python3 simulations/MYRIAD/eval_optimizations.py --match share0.84 --jobs 2

Reuses eval_scaling's ini writer and .sca readers; results nest under
eval/optimizations[_<tag>]/.
"""
import argparse
import glob
import os
import re
import sys
from concurrent.futures import ThreadPoolExecutor

import eval_harness as harness
import eval_scaling as S
from eval_harness import (BIN, SRC, MYRIAD_DIR, PROTO_COLOR, add_repeat_args, announce,
                          apply_repeat_args, byte_unit, fmt_bytes, overrides, repeat_mean,
                          pyplot, rep_path, simtime)

FAN_DIR = os.path.join(str(MYRIAD_DIR), "myriad_topologies", "csv",
                       "delaunay_shr1000_rl16_spof0.5")
DEFAULT_MATCH = "share0.74"


def add_opt_args(ap):
    """The run's own flags, shared with eval_entrypoint's subparser."""
    ap.add_argument("--dir", default=None,
                    help=f"topology folder (default: the fan sweep {os.path.basename(FAN_DIR)})")
    ap.add_argument("--match", default=None,
                    help=f"substring selecting one sweep level; its seeds are "
                         f"averaged (default {DEFAULT_MATCH})")
    ap.add_argument("--tag", default=None, help="suffix for the output folder")
    ap.add_argument("--dry-run", action="store_true",
                    help="print configs, topologies and run horizon, simulate nothing")

COMMON = {"ecmpCostMargin": 0.50, "refreshFloodsUpdate": "true"}
# Every optimization off; each config below switches on exactly what its name says.
BASE = {"maxEcmpPaths": 1, "suppressRedundantTransitions": "false",
        "spofHoldMultiplier": 1, "holdCostScaled": "false"}
HOLD_POLICY = {"spofHoldMultiplier": 4, "holdCostScaled": "true"}   # in-network policy
CONFIGS = [
    ("baseline", {}),
    ("ecmp 2", {"maxEcmpPaths": 2}),
    ("ecmp 8", {"maxEcmpPaths": 8}),
    # On ecmp 2, not the baseline's 1: suppression removes chains, so at one
    # path it undercut "all" and read as cheaper than every optimization
    # together. It is what "all" adds on top of ecmp 2 that this bar shows.
    ("suppress (ecmp 2)", {"maxEcmpPaths": 2,
                           "suppressRedundantTransitions": "true"}),
    ("hold policy", HOLD_POLICY),
    ("all", {"maxEcmpPaths": 2, "suppressRedundantTransitions": "true",
             **HOLD_POLICY}),
]


def pv_params(cfg):
    return {**BASE, **COMMON, **cfg}


def check_params():
    """An override naming no parameter is silently ignored by OMNeT++, which would
    turn a typo into a config identical to the baseline. Fail before simulating."""
    ned = open(os.path.join(SRC, "routing", "MYRIAD", "MyriadGp.ned")).read()
    missing = sorted({k for _, cfg in CONFIGS for k in pv_params(cfg)
                      if not re.search(rf"\b{re.escape(k)}\s*(@unit\([^)]*\)\s*)?=", ned)})
    if missing:
        sys.exit(f"not MyriadGP NED parameters: {', '.join(missing)}")


def tag_of(name):
    return "_" + re.sub(r"\W+", "_", name)


def run_one(proto, ini, name, opts, rep):
    """One sim -> its scalar readouts (eval_scaling's readers, same result dirs)."""
    tag = tag_of(name)
    msgs = S.message_count(proto, ini, tag=tag, extra_args=opts, rep=rep)
    state, lmax, _ = S.state_stats(proto, ini, tag=tag, rep=rep)
    res = rep_path(os.path.join(S.INI_DIR, f"res_{proto}{tag}_{os.path.basename(ini)}"), rep)
    return {"msgs": msgs, "bytes": S.byte_count(proto, ini, tag=tag, rep=rep),
            "state": state, "lmax": lmax,
            "calc": S._sum_scalar(glob.glob(os.path.join(res, "*.sca")), "calc_ops")}


def pct(v, ref):
    return f"{100 * (v / ref - 1):+.0f}%" if ref else "-"


def report(results, ls, outfile):
    """outfile is a PREFIX: one <prefix>_<metric>.png per metric."""
    base = results["baseline"]
    print(f"\n{'config':<18}{'messages':>12}{'Δ':>6}{'bytes':>11}{'Δ':>6}"
          f"{'link max':>11}{'Δ':>6}{'state/rtr':>11}{'Δ':>6}{'calc ops':>11}{'Δ':>6}")
    rows = list(results.items()) + [("LS (ref)", ls)]
    # One unit per column, or "7.84 GB" and "533 MB" sit under each other.
    mb = byte_unit([])
    for name, r in rows:
        print(f"{name:<18}{r['msgs']:>12.0f}{pct(r['msgs'], base['msgs']):>6}"
              f"{fmt_bytes(r['bytes'], mb):>11}{pct(r['bytes'], base['bytes']):>6}"
              f"{fmt_bytes(r['lmax'], mb):>11}{pct(r['lmax'], base['lmax']):>6}"
              f"{fmt_bytes(r['state'], mb):>11}{pct(r['state'], base['state']):>6}"
              f"{r['calc']:>11.3g}{pct(r['calc'], base['calc']):>6}")
    print("Δ = vs. baseline (all optimizations off); mean over seeds.")

    try:
        plt = pyplot()
    except ImportError:
        print("(matplotlib not available — skipping plot)")
        return
    names = list(results)
    # One file each: the four metrics are cited in different places and a
    # 4-panel strip left every bar chart a sliver wide.
    panels = [("msgs", "messages"), ("bytes", "bytes"),
              ("lmax", "busiest link"), ("state", "state per router")]
    for key, label in panels:
        div, uname = (byte_unit([float(ls[key])]
                                + [float(results[n][key]) for n in names])
                      if key != "msgs" else (1.0, ""))
        vals = [float(results[n][key]) / div for n in names]
        sds = [(getattr(results[n][key], "sd", 0.0) or 0.0) / div for n in names]
        fig, ax = plt.subplots(figsize=(7.0, 4.8))
        ax.bar(range(len(names)), vals, yerr=sds if any(sds) else None,
               color=PROTO_COLOR["pv"], capsize=3, label="Path-Vector (PV)")
        ax.axhline(float(ls[key]) / div, color=PROTO_COLOR["ls"], ls="--",
                   label="Link-State (LS)")
        ax.set_xticks(range(len(names)), names, rotation=45, ha="right")
        ax.set_title(label + (f" ({uname})" if uname else ""))
        ax.legend()
        fig.tight_layout()
        out = f"{outfile}_{key}.png"
        fig.savefig(out, dpi=200)
        plt.close(fig)
        print(f"plot -> {out}")


def main(args=None):
    if args is None:
        ap = argparse.ArgumentParser(description="PV optimization impact, one fan level")
        add_opt_args(ap)
        add_repeat_args(ap)
        args = ap.parse_args()
    apply_repeat_args(args)
    check_params()

    # eval_entrypoint passes its own namespace, where the defaults are None.
    topo_dir = getattr(args, "dir", None) or FAN_DIR
    match = getattr(args, "match", None) or DEFAULT_MATCH
    topos = sorted(p for p in glob.glob(os.path.join(os.path.abspath(topo_dir), "*.txt"))
                   if match in os.path.basename(p))
    if not topos:
        sys.exit(f"no .txt in {topo_dir} matching '{match}'")
    simt = simtime(cycles=2)
    out = os.path.join(S.EVAL_DIR, "optimizations" + (f"_{args.tag}" if args.tag else ""))
    S.INI_DIR = os.path.join(out, "ini")
    plot_dir = os.path.join(out, "plots")

    print(f"== {len(topos)} topologies matching '{match}', horizon {simt} "
          f"(warmup + 2 refresh cycles) ==")
    for p in topos:
        print(f"  {os.path.basename(p)}")
    for name, cfg in CONFIGS:
        print(f"  {name:<16}{' '.join(overrides(**pv_params(cfg)))}")
    if getattr(args, "dry_run", False):
        return
    if not os.path.exists(BIN):
        sys.exit(f"RoutingSim binary not found at {BIN} — build it first.")
    for d in (S.INI_DIR, plot_dir):
        os.makedirs(d, exist_ok=True)
    announce("optimizations", opts=overrides(sim_time_limit=simt, **COMMON))

    # Inis once per topology before fanning out: parallel jobs on the same
    # topology would otherwise rewrite the same ini file under each other.
    inis = {}
    for p in topos:
        rings = S.ring_label_lines(p)
        inis[p] = (S.write_ini("ls", p, rings), S.write_ini("pv", p, rings))

    jobs = [(p, None) for p in topos] + [(p, c) for c in CONFIGS for p in topos]

    def do(job):
        p, cfg = job
        ls_ini, pv_ini = inis[p]
        if cfg is None:
            opts = overrides(sim_time_limit=simt)
            return repeat_mean(lambda rep: run_one("ls", ls_ini, "ref", opts, rep))
        opts = overrides(sim_time_limit=simt, **pv_params(cfg[1]))
        return repeat_mean(lambda rep: run_one("pv", pv_ini, cfg[0], opts, rep))

    if (harness.JOBS or 1) > 1:
        harness.PROGRESS = False   # concurrent sims would fight over the \r line
        with ThreadPoolExecutor(max_workers=harness.JOBS) as ex:
            done = list(ex.map(do, jobs))
    else:
        done = [do(j) for j in jobs]

    by = {}
    for (p, cfg), r in zip(jobs, done):
        by.setdefault(None if cfg is None else cfg[0], []).append(r)
    ls = S._mean_over_seeds(by.pop(None))
    results = {name: S._mean_over_seeds(by[name]) for name, _ in CONFIGS}
    report(results, ls, os.path.join(plot_dir, f"optimizations_{match}"))


if __name__ == "__main__":
    main()
