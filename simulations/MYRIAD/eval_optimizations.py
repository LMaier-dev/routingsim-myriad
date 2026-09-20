#!/usr/bin/env python3
"""Impact of the PV propagation optimizations, each switched on alone against the
baseline bar, then all together. One topology (--topo) or one sweep level of a
ringgrow set averaged over its seeds, which is where the error bars come from;
LS runs once per seed as the reference.

Every run covers the warmup plus two refresh cycles with refreshFloodsUpdate=true,
so the totals include what the refreshes re-flood, not only the formation burst.
maxEcmpPaths is fixed at ECMP for every config except the bar named for it, so a
Δ is the optimization and not a path count; ecmpCostMargin is 10% throughout, and
spofHoldMultiplier=1 is the NED's "off" for the spof bucket. ECMP slots are
filled by cost alone.

The byte panels split each bar: solid is the run with the reachability-record
fields subtracted, the faded extension above it is what RR cost. Both come out
of the SAME simulation (MyriadPV tallies its own RR bytes per link).

    python3 simulations/MYRIAD/eval_entrypoint.py optimizations --jobs 2   # as eval_runs.txt runs it
    python3 simulations/MYRIAD/eval_optimizations.py --dry-run
    python3 simulations/MYRIAD/eval_optimizations.py --topo "myriad_topologies/csv/grid_10,10.txt"
    python3 simulations/MYRIAD/eval_optimizations.py --match tr300 --jobs 2

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
                          pyplot, rep_path, simtime, title)

CORPUS_DIR = os.path.join(str(MYRIAD_DIR), "myriad_topologies", "csv",
                          "delaunay_grow2500_A300_spof0.5")
DEFAULT_MATCH = "tr500"


def add_opt_args(ap):
    """The run's own flags, shared with eval_entrypoint's subparser."""
    ap.add_argument("--topo", default=None,
                    help="one topology file instead of a folder sweep "
                         "(e.g. myriad_topologies/csv/grid_10,10.txt)")
    ap.add_argument("--basis", choices=["min", "tree", "all", "native"], default="min",
                    help="ring basis for --topo (default: min; a shipped "
                         ".native.ring.ini always wins)")
    ap.add_argument("--dir", default=None,
                    help=f"topology folder; its --match level is averaged over seeds "
                         f"(default {os.path.basename(CORPUS_DIR)})")
    ap.add_argument("--match", default=None,
                    help=f"substring selecting one sweep level; its seeds are "
                         f"averaged (default {DEFAULT_MATCH})")
    ap.add_argument("--tag", default=None, help="suffix for the output folder")
    ap.add_argument("--dry-run", action="store_true",
                    help="print configs, topologies and run horizon, simulate nothing")

ECMP = 3   # every config, so a bar's Δ is the optimization and not a path count
COMMON = {"refreshFloodsUpdate": "true",
          "maxEcmpPaths": ECMP}
# Every optimization off; each config below switches on exactly what its name says.
BASE = {"suppressRedundantTransitions": "false",
        "spofHoldMultiplier": 1, "holdCostScaled": "false"}
HOLD_POLICY = {"spofHoldMultiplier": 3, "holdCostScaled": "true"}   # in-network policy
# The first entry is the reference every Δ is taken against.
CONFIGS = [
    ("ecmp 8/0.5", {"maxEcmpPaths": 8, "ecmpCostMargin": 0.5}),          # the one config off the fixed ECMP
    ("base 3/0.25", {"ecmpCostMargin": 0.25}),
    ("hold policy", {**HOLD_POLICY, "ecmpCostMargin": 0.25}),
    ("suppress", {"suppressRedundantTransitions": "true", "ecmpCostMargin": 0.25}),
    ("both 3/0.25", {"suppressRedundantTransitions": "true", **HOLD_POLICY, "ecmpCostMargin": 0.25}),
    ("both 8/0.5", {"suppressRedundantTransitions": "true", **HOLD_POLICY, "maxEcmpPaths": 8, "ecmpCostMargin": 0.5}),          # the one config off the fixed ECMP
]


def pv_params(cfg):
    return {**BASE, **COMMON, **cfg}


def check_params():
    """An override naming no parameter is silently ignored by OMNeT++, which would
    turn a typo into a config identical to the baseline. Fail before simulating."""
    ned = open(os.path.join(SRC, "routing", "MYRIAD", "MyriadPv.ned")).read()
    missing = sorted({k for _, cfg in CONFIGS for k in pv_params(cfg)
                      if not re.search(rf"\b{re.escape(k)}\s*(@unit\([^)]*\)\s*)?=", ned)})
    if missing:
        sys.exit(f"not MyriadPV NED parameters: {', '.join(missing)}")


def tag_of(name):
    return "_" + re.sub(r"\W+", "_", name)


FILL_RE = re.compile(r"^scalar\s+(\S+)\s+ecmp_(chains|frames)_total\s+([0-9.]+)", re.M)


def router_fills(scas):
    """Chains held per origin ring, per ROUTER. _sum_scalar pools the two totals
    over the whole network, which gives the mean but no spread; the min and max
    need them paired by module."""
    per = {}
    for sca in scas:
        for m in FILL_RE.finditer(open(sca).read()):
            per.setdefault(m.group(1), {})[m.group(2)] = float(m.group(3))
    return sorted(v["chains"] / v["frames"] for v in per.values() if v.get("frames"))


def run_one(proto, ini, name, opts, rep):
    """One sim -> its scalar readouts (eval_scaling's readers, same result dirs).

    The RR-excluded figures come out of the SAME run: MyriadPV tallies the RR
    bytes it put on each link and finish() writes the subtracted totals, so no
    second simulation is needed (see eval_scaling.norr_stats)."""
    tag = tag_of(name)
    msgs = S.message_count(proto, ini, tag=tag, extra_args=opts, rep=rep)
    state, lmax, _ = S.state_stats(proto, ini, tag=tag, rep=rep)
    res = rep_path(os.path.join(S.INI_DIR, f"res_{proto}{tag}_{os.path.basename(ini)}"), rep)
    scas = glob.glob(os.path.join(res, "*.sca"))
    nr_bytes, nr_lmax, _ = S.norr_stats(ini, tag=tag, rep=rep) if proto == "pv" \
        else (None, None, None)
    # Pooled over all routers, not a mean of per-router means: routers know
    # different numbers of rings and the question is about slots, not routers.
    chains = S._sum_scalar(scas, "ecmp_chains_total")
    frames = S._sum_scalar(scas, "ecmp_frames_total")
    fills = router_fills(scas)
    return {"msgs": msgs, "bytes": S.byte_count(proto, ini, tag=tag, rep=rep),
            "state": state, "lmax": lmax,
            "bytes_norr": nr_bytes, "lmax_norr": nr_lmax,
            "ecmp_fill": (chains / frames) if frames else None,
            "ecmp_fill_min": fills[0] if fills else None,
            "ecmp_fill_max": fills[-1] if fills else None,
            "calc": S._sum_scalar(scas, "calc_ops")}


def pct(v, ref):
    return f"{100 * (v / ref - 1):+.0f}%" if ref else "-"


def report(results, ls, outfile):
    """outfile is a PREFIX: one <prefix>_<metric>.png per metric."""
    ref = CONFIGS[0][0]
    base = results[ref]
    print(f"\n{'config':<14}{'messages':>12}{'Δ':>6}{'bytes':>11}{'Δ':>6}"
          f"{'no RR':>11}{'link max':>11}{'Δ':>6}{'state/rtr':>11}{'Δ':>6}"
          f"{'calc ops':>11}{'Δ':>6}{'ecmp min/mean/max':>19}")
    rows = list(results.items()) + [("LS (ref)", ls)]
    # One unit per column, or "7.84 GB" and "533 MB" sit under each other.
    mb = byte_unit([])
    for name, r in rows:
        fill = r.get("ecmp_fill")
        span = (f"{float(r['ecmp_fill_min']):.2f}/{float(fill):.2f}/"
                f"{float(r['ecmp_fill_max']):.2f}") if fill else "-"
        print(f"{name:<14}{r['msgs']:>12.0f}{pct(r['msgs'], base['msgs']):>6}"
              f"{fmt_bytes(r['bytes'], mb):>11}{pct(r['bytes'], base['bytes']):>6}"
              f"{fmt_bytes(r['bytes_norr'], mb) if r.get('bytes_norr') else '-':>11}"
              f"{fmt_bytes(r['lmax'], mb):>11}{pct(r['lmax'], base['lmax']):>6}"
              f"{fmt_bytes(r['state'], mb):>11}{pct(r['state'], base['state']):>6}"
              f"{r['calc']:>11.3g}{pct(r['calc'], base['calc']):>6}"
              f"{span:>19}")
    print(f"Δ = vs. {ref}; mean over seeds. maxEcmpPaths={ECMP} except where named.")
    print("no RR = same packets with the reachability-record fields subtracted; "
          "ecmp = chains held per origin ring, over the thinnest / all / the "
          "richest router.")

    try:
        plt = pyplot()
    except ImportError:
        print("(matplotlib not available — skipping plot)")
        return
    names = list(results)
    x = range(len(names))
    # One file each: the metrics are cited in different places and a 4-panel
    # strip left every bar chart a sliver wide.
    # norr: the RR-excluded figure of the same metric, drawn as the solid part
    # of the bar with the RR fields as a faded extension above it.
    panels = [("msgs", "messages", None), ("bytes", "bytes", "bytes_norr"),
              ("lmax", "busiest link", "lmax_norr"),
              ("state", "state per router", None)]
    for key, label, nrkey in panels:
        div, uname = (byte_unit([float(ls[key])]
                                + [float(results[n][key]) for n in names])
                      if key != "msgs" else (1.0, ""))
        val = lambda n, k: float(results[n][k]) / div
        sd = lambda n, k: (getattr(results[n][k], "sd", 0.0) or 0.0) / div
        has_nr = nrkey and all(results[n].get(nrkey) is not None for n in names)
        fig, ax = plt.subplots(figsize=(7.0, 4.8))
        if has_nr:
            # Faded full bar first, solid RR-excluded one over it: what stays
            # visible on top is what the reachability records cost.
            ax.bar(x, [val(n, key) for n in names], color=PROTO_COLOR["pv"],
                   alpha=0.3, edgecolor="black", linewidth=0.4, linestyle=":",
                   label="Path-Vector (PV)")
            ax.bar(x, [val(n, nrkey) for n in names],
                   yerr=[sd(n, nrkey) for n in names], capsize=3,
                   color=PROTO_COLOR["pv_norr"], edgecolor="black", linewidth=0.4,
                   label="PV, RR fields excluded")
        else:
            ax.bar(x, [val(n, key) for n in names],
                   yerr=[sd(n, key) for n in names], capsize=3,
                   color=PROTO_COLOR["pv"], label="Path-Vector (PV)")
        ax.axhline(float(ls[key]) / div, color=PROTO_COLOR["ls"], ls="--",
                   label="Link-State (LS)")
        ax.set_xticks(x, names, rotation=45, ha="right")
        ax.set_title(title(label + (f" ({uname})" if uname else "")))
        ax.legend()
        fig.tight_layout()
        out = f"{outfile}_{key}.png"
        fig.savefig(out, dpi=200)
        plt.close(fig)
        print(f"plot -> {out}")

    # How many of the ECMP slots a config actually fills. LS has no counterpart,
    # so no reference line. The whiskers are the thinnest and the richest ROUTER,
    # not the seed spread: the question is whether some routers sit empty while
    # the mean looks healthy, which a standard deviation over seeds cannot show.
    if all(results[n].get("ecmp_fill") is not None for n in names):
        fig, ax = plt.subplots(figsize=(7.0, 4.8))
        mean = [float(results[n]["ecmp_fill"]) for n in names]
        lo = [m - float(results[n]["ecmp_fill_min"]) for m, n in zip(mean, names)]
        hi = [float(results[n]["ecmp_fill_max"]) - m for m, n in zip(mean, names)]
        ax.bar(x, mean, yerr=[lo, hi], capsize=3,
               color=PROTO_COLOR["pv"], edgecolor="black", linewidth=0.4,
               label="chains held (mean, whiskers = thinnest/richest router)")
        for i, (m, h) in enumerate(zip(mean, hi)):
            ax.annotate(f"{m:.2f}", (i, m + h), textcoords="offset points",
                        xytext=(0, 3), ha="center", fontsize=10)
        ax.set_xticks(x, names, rotation=45, ha="right")
        ax.set_ylabel("chains per origin ring")
        ax.set_title(title("ECMP slots filled"))
        # Headroom for the labels and the legend, which sits over the tallest bar.
        ax.set_ylim(top=max(m + h for m, h in zip(mean, hi)) * 1.25)
        ax.legend()
        fig.tight_layout()
        out = f"{outfile}_ecmpfill.png"
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
    # TOPO_LABEL is what title() puts on the figures' second line, the way every
    # other eval names its topology.
    one = getattr(args, "topo", None)
    if one:
        if not os.path.exists(one):
            one = os.path.join(str(MYRIAD_DIR), "myriad_topologies", "csv", one)
        if not os.path.exists(one):
            sys.exit(f"--topo {args.topo}: no such topology file")
        S.BASIS = getattr(args, "basis", None) or "min"
        topos, match = [os.path.abspath(one)], os.path.splitext(os.path.basename(one))[0]
        harness.TOPO_LABEL = harness.topo_label(one)
    else:
        topo_dir = getattr(args, "dir", None) or CORPUS_DIR
        match = getattr(args, "match", None) or DEFAULT_MATCH
        topos = sorted(p for p in glob.glob(os.path.join(os.path.abspath(topo_dir), "*.txt"))
                       if match in os.path.basename(p))
        if not topos:
            sys.exit(f"no .txt in {topo_dir} matching '{match}'")
        harness.TOPO_LABEL = f"{os.path.basename(os.path.normpath(topo_dir))} {match}".upper()
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
