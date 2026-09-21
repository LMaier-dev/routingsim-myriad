#!/usr/bin/env python3
"""What a refresh costs against what coming up costs, for MyriadPV alone.

Convergence pays for every chain that was ever tried. A refresh re-floods only
what survived: the quote a ring made in the previous generation still sits in
its front-ring slot, and a chain that is neither better nor co-best with that
quote is dropped at the transition instead of propagating. What a refresh
cannot avoid is re-flooding the co-best set itself, which is deliberate, since
the forwarding router cannot know what the neighbour ring already holds.

The run is therefore cut into one formation window and one window per refresh,
and the ratio between them is what the suppression buys. LS takes no part: it
never re-originates an LSA, so it has no refresh to price.

    python3 simulations/MYRIAD/eval_refresh.py                     # German HV, 3 refreshes
    python3 simulations/MYRIAD/eval_refresh.py --refreshes 4 --repeat 3
    python3 simulations/MYRIAD/eval_refresh.py --dir myriad_topologies/csv/delaunay_grow2500_A300_spof0.5 --match tr500
    python3 simulations/MYRIAD/eval_refresh.py --control           # add the suppression-off bar
    python3 simulations/MYRIAD/eval_refresh.py --dry-run
"""
import argparse
import glob
import os
import sys
from collections import defaultdict
from concurrent.futures import ThreadPoolExecutor, as_completed

import eval_dynamics as D
import eval_harness as H
from eval_harness import (BIN, MYRIAD_DIR, SIM_ROOT, add_topo_args, announce,
                          apply_repeat_args, byte_unit, ensure_ring_labels,
                          overrides, pyplot, refresh_s, rep_path, run_sim, title,
                          warmup_s, write_proto_ini)

OUT = os.path.join(str(MYRIAD_DIR), "eval", "refresh")
DEFAULT_TOPO = os.path.join(str(MYRIAD_DIR), "myriad_topologies", "csv",
                            "German_HV_net_n2312.min.ring.txt")

# Name -> the overrides that define it. Every run is eval_common otherwise, so a
# bar is the parameter in its name and nothing else.
BASE = {"pv": []}
CONTROL = ("pv_nosuppress", ["--**.suppressRedundantTransitions=false"])
COLOR = {"pv": H.PROTO_COLOR["pv"], CONTROL[0]: H.PROTO_COLOR.get("pv_norr", "#888")}


def quiet_len():
    return min(25.0, 0.25 * refresh_s())


def windows(nrefresh):
    """(label, t0, t1) per window, plus the horizon they need.

    A ring refreshes refreshInterval after its king was crowned, which happens
    during the COLLECT warmup, so the k-th refresh lands near warmup + k*R. Each
    window is centred on that point with half a refresh interval of guard on
    either side, which swallows refreshJitter (10% of R), updateJitter and the
    lap time, and the drift those accumulate over the run.

    `quiet` is the tail of `conv`, ending where the first refresh window begins:
    converged, nothing refreshing yet, so it is the hello floor every window is
    read against."""
    r, t0 = refresh_s(), warmup_s()
    first = t0 + 0.5 * r                       # where conv stops and r1 starts
    wins = [("conv", 0.0, first),
            ("quiet", max(0.0, first - quiet_len()), first)]
    wins += [(f"r{k}", t0 + (k - 0.5) * r, t0 + (k + 0.5) * r)
             for k in range(1, nrefresh + 1)]
    return wins, t0 + (nrefresh + 0.5) * r


def snapshot_s():
    """Snapshot period. Half the shortest window, so `quiet` always contains
    one: it is the narrowest, and a window with no snapshot cannot be checked
    for route loss at all."""
    return min(0.25 * refresh_s(), 0.5 * quiet_len())


def sim_cmd(name, ini, simt, resdir, dump):
    """One run. refreshFloodsUpdate is the point of the experiment, so it is set
    explicitly rather than left to the NED default other scripts turn off."""
    return ([BIN, "-u", "Cmdenv", ini, *overrides(sim_time_limit=f"{simt:g}s"),
             "--result-dir", resdir,
             "--**.refreshFloodsUpdate=true",
             f'--**.ribDumpFile="{dump}"',
             '--**.snapshotFormat="stats"',
             f"--**.snapshotInterval={snapshot_s():g}s"]
            + BASE.get(name, []))


def route_totals(dump, wins):
    """{label: routes held network-wide at the last snapshot in the window}.

    A refresh that costs nothing because its flood died looks the same in bytes
    as one that costs nothing because everything was suppressed. The route count
    tells the two apart: it has to hold flat across the refresh windows."""
    snaps = defaultdict(int)
    if not os.path.exists(dump):
        return {}
    for m in D.STATS_RE.finditer(open(dump).read()):
        present = int(m.group(6)) if m.group(6) else int(m.group(3))
        snaps[float(m.group(1))] += present
    out = {}
    for lab, t0, t1 in wins:
        inside = [t for t in snaps if t0 <= t < t1]
        if inside:
            out[lab] = snaps[max(inside)]
    return out


def bursts(resdir, limit=8):
    """Seconds carrying a fifth of the peak second or more. Windows are placed
    from refreshInterval rather than detected, so this is the check that they
    landed on the traffic: a burst on a window edge is a refresh being split
    across two windows."""
    times, vals = D.bytes_over_time(resdir, bin_s=1.0)
    peak = max(vals, default=0.0)
    if not peak:
        return []
    return [t for t, v in zip(times, vals) if v > 0.2 * peak][:limit]


def run_one(topo, ini, name, wins, simt, rep, run):
    tag = f"{D.stem(topo)}_{name}"
    resdir = rep_path(os.path.join(OUT, f"res_{tag}"), rep)
    dump = rep_path(os.path.join(OUT, f"{tag}.rib"), rep)
    if run:
        for stale in (dump, *glob.glob(resdir + "/*.sca"), *glob.glob(resdir + "/*.vec")):
            if os.path.exists(stale):
                os.remove(stale)
        run_sim(sim_cmd(name, ini, simt, resdir, dump), cwd=SIM_ROOT, rep=rep)
    return D.tx_windows(resdir, wins), route_totals(dump, wins), resdir


def report(res, routes, wins, names):
    """Per-window volume, and each refresh as a share of the formation cost."""
    div, unit = byte_unit([v[0] for vals in res.values() for v in vals])
    labs = [w[0] for w in wins]
    print(f"\n=== control-plane volume per window ({H.REPEATS} seed(s)) ===")
    print(f"{'window':>8}{'span':>16}" + "".join(
        f"{n + ' ' + unit:>16}{n + ' msgs':>16}{'vs conv':>10}" for n in names))
    for lab, t0, t1 in wins:
        row = f"{lab:>8}{f'[{t0:g},{t1:g})s':>16}"
        for n in names:
            vals = res.get((lab, n), [])
            if not vals:
                row += f"{'-':>16}{'-':>16}{'-':>10}"
                continue
            b = sum(v[0] for v in vals) / len(vals)
            m = sum(v[1] for v in vals) / len(vals)
            conv = res.get(("conv", n))
            ratio = b / (sum(v[0] for v in conv) / len(conv)) if conv else 0.0
            row += f"{b / div:>16,.3g}{m:>16,.0f}{ratio:>9.1%}"
        print(row)
    if routes:
        print(f"\n{'window':>8}" + "".join(f"{n + ' routes':>18}" for n in names))
        for lab in labs:
            row = f"{lab:>8}"
            for n in names:
                v = routes.get((lab, n), [])
                row += f"{sum(v) / len(v):>18,.0f}" if v else f"{'-':>18}"
            print(row)
    with open(os.path.join(OUT, "refresh.csv"), "w") as fh:
        fh.write("window,t0,t1,config,bytes,messages,routes\n")
        for lab, t0, t1 in wins:
            for n in names:
                for i, v in enumerate(res.get((lab, n), [])):
                    r = routes.get((lab, n), [])
                    fh.write(f"{lab},{t0:g},{t1:g},{n},{v[0]:.0f},{v[1]},"
                             f"{r[i] if i < len(r) else ''}\n")
    print(f"\ncsv -> {os.path.join(OUT, 'refresh.csv')}")


def _series(res, labs, names, idx, scale):
    """Per-config {name: ([mean per window], [half-range per window])}."""
    out = {}
    for n in names:
        vals = [res.get((lab, n), []) for lab in labs]
        out[n] = ([sum(v[idx] for v in vs) / len(vs) / scale if vs else 0.0
                   for vs in vals],
                  [(max(v[idx] for v in vs) - min(v[idx] for v in vs)) / 2 / scale
                   if len(vs) > 1 else 0.0 for vs in vals])
    return out


def plot(res, wins, names):
    """One figure: bytes as bars on the left axis, messages as markers on the
    right. Where two configs are compared the second is always the dearer one,
    so it is drawn as a faded extension of the first rather than beside it: the
    visible gap is then what the cheaper config saves."""
    try:
        plt = pyplot()
    except ImportError:
        print("\n(matplotlib not available — skipping plot)")
        return
    labs = [w[0] for w in wins if w[0] != "quiet"]
    div, unit = byte_unit([v[0] for vals in res.values() for v in vals])
    xs = list(range(len(labs)))
    fig, ax = plt.subplots(figsize=(1.9 * len(labs) + 4.0, 5.0))
    rax = ax.twinx()

    byt = _series(res, labs, names, 0, div)
    msg = _series(res, labs, names, 1, 1)
    lo, hi = names[0], (names[1] if len(names) > 1 else None)
    base, berr = byt[lo]
    colour = COLOR.get(lo, "tab:red")

    if hi:
        # Extension, not a second bar: it only reads as a saving if the two
        # share a baseline. A config that came out cheaper than `lo` would draw
        # a negative extension, so it is clamped and named in the table instead.
        top_v = [max(a, b) for a, b in zip(byt[hi][0], base)]
        ax.bar(xs, top_v, 0.5, color=colour, alpha=0.25, edgecolor="black",
               linewidth=0.4, linestyle=":", label=f"{hi} (bytes)")
    ax.bar(xs, base, 0.5, yerr=berr if any(berr) else None, capsize=3,
           color=colour, alpha=0.9, edgecolor="black", linewidth=0.4,
           label=f"{lo} (bytes)")

    top = 0.0
    for i, x in enumerate(xs):
        m = base[i]
        top = max(top, m + berr[i])
        share = f"\n{m / base[0]:.0%}" if base[0] and i else ""
        ax.annotate((f"{m:,.0f}" if m >= 100 else f"{m:,.3g}") + share,
                    (x, m + berr[i]), textcoords="offset points",
                    xytext=(0, 3), ha="center", fontsize=9)
        if hi and byt[hi][0][i] > m:
            v = byt[hi][0][i]
            top = max(top, v)
            ax.annotate(f"{v:,.0f}" if v >= 100 else f"{v:,.3g}", (x, v),
                        textcoords="offset points", xytext=(0, 3), ha="center",
                        fontsize=9, alpha=0.7)

    # Offset clear of the bars: a marker over a bar lands on its value label,
    # and the two axes are scaled independently so the collision is not
    # predictable from the numbers.
    mxs = [x + 0.36 for x in xs]
    # Thousands, so the axis carries its own numbers instead of a 1e6 offset
    # label floating over the title.
    rax.plot(mxs, [v / 1e3 for v in msg[lo][0]], "o-", color="black",
             markersize=7, label=f"{lo} (messages)")
    if hi:
        rax.plot(mxs, [v / 1e3 for v in msg[hi][0]], "o--", color="black",
                 markersize=7, markerfacecolor="white", label=f"{hi} (messages)")
    rax.set_ylabel("thousand messages per window")
    rax.set_ylim(0, max(max(msg[n][0]) for n in names) / 1e3 * 1.45)

    floor = res.get(("quiet", lo))
    if floor:
        qw = next(t1 - t0 for lab, t0, t1 in wins if lab == "quiet")
        ax.axhline(sum(v[0] for v in floor) / len(floor) / qw * refresh_s() / div,
                   color="grey", ls=":", label="hello floor (same window length)")
    ax.set_xticks(xs)
    ax.set_xticklabels(labs)
    ax.set_xlim(-0.55, len(labs) - 1 + 0.75)
    ax.set(ylabel=f"{unit} per window", xlabel="window",
           title=title("Formation against each refresh"))
    ax.set_ylim(top=top * 1.45)
    h1, l1 = ax.get_legend_handles_labels()
    h2, l2 = rax.get_legend_handles_labels()
    ax.legend(h1 + h2, l1 + l2, loc="upper right", fontsize=9)
    fig.tight_layout()
    out = os.path.join(OUT, "refresh.png")
    fig.savefig(out, dpi=200)
    plt.close(fig)
    print(f"plot -> {out}")


def topo_set(args):
    if not args.dir:
        return [args.topo]
    found = sorted(glob.glob(os.path.join(args.dir, "*.txt")))
    if args.match:
        found = [f for f in found if args.match in os.path.basename(f)]
    if not found:
        sys.exit(f"--dir {args.dir}: no *.txt topologies"
                 + (f" matching {args.match!r}" if args.match else ""))
    H.TOPO_LABEL = os.path.basename(args.dir.rstrip("/"))
    print(f"  [topologies] {len(found)}: " + ", ".join(D.stem(f) for f in found[:4])
          + (f", … (+{len(found) - 4})" if len(found) > 4 else ""))
    return found


def add_refresh_args(ap):
    """This eval's own flags. The topology and repeat flags come from
    add_topo_args, so --set, --refresh and the ring-basis options behave as they
    do everywhere else."""
    # The grid this runs on is not the harness default, and its rings are mined
    # rather than shipped, so both defaults move.
    ap.set_defaults(topo=DEFAULT_TOPO, basis="min")
    ap.add_argument("--dir", default=None, help="topology folder, every *.txt pooled")
    ap.add_argument("--match", default=None, help="substring filter on --dir")
    ap.add_argument("--refreshes", type=int, default=3, metavar="N",
                    help="how many refresh cycles to measure after convergence "
                         "(default 3). --refresh sets how long one cycle is")
    ap.add_argument("--control", action="store_true",
                    help="add a suppressRedundantTransitions=false bar, which is "
                         "what attributes the drop to the suppression")
    ap.add_argument("--margins", default=None, metavar="LOW,HIGH",
                    help="compare two ecmpCostMargin values instead (e.g. "
                         "0.1,0.25). The narrower band holds fewer co-best "
                         "siblings, so it propagates less; LOW is the solid bar "
                         "and HIGH its extension")
    ap.add_argument("--post", action="store_true",
                    help="re-parse the result dirs on disk, simulate nothing")
    ap.add_argument("--dry-run", action="store_true",
                    help="print the plan and the horizon, simulate nothing")


def main(args=None):
    if args is None:
        ap = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
        add_topo_args(ap)
        add_refresh_args(ap)
        args = ap.parse_args()
    apply_repeat_args(args)

    if args.control and args.margins:
        sys.exit("--control and --margins each define the second bar; pick one")
    if args.margins:
        lo, _, hi = args.margins.partition(",")
        if not hi:
            sys.exit("--margins takes two values, e.g. --margins 0.1,0.25")
        BASE.clear()
        for m in (lo, hi):
            BASE[f"margin {float(m):g}"] = [f"--**.ecmpCostMargin={float(m):g}"]
        COLOR.update({n: H.PROTO_COLOR["pv"] for n in BASE})
    elif args.control:
        BASE[CONTROL[0]] = CONTROL[1]
    names = list(BASE)
    wins, simt = windows(args.refreshes)
    topos = topo_set(args)

    global OUT
    OUT = os.path.join(OUT, (os.path.basename(args.dir.rstrip("/")) if args.dir
                             else D.stem(args.topo)) + (f"_{args.tag}" if args.tag else ""))
    os.makedirs(OUT, exist_ok=True)

    print(f"\n=== refresh cost ({simt:g}s horizon) ===")
    announce("refresh", opts=overrides(sim_time_limit=f"{simt:g}s"))
    print("  windows: " + ", ".join(f"{l} [{a:g},{b:g})s" for l, a, b in wins))
    print(f"  configs: {', '.join(names)}")
    if args.dry_run:
        print(f"\n{len(topos) * len(names) * H.REPEATS} runs of {simt:g}s "
              f"simulated time — nothing executed (--dry-run)")
        return

    if not os.path.exists(BIN):
        sys.exit(f"RoutingSim binary not found at {BIN} — build it first.")
    run_sims = not args.post
    inis = {}
    for topo in topos if run_sims else []:
        labels = ensure_ring_labels(topo, args.basis, args.max_ring_size,
                                    not args.no_echo_rings)
        inis[topo] = write_proto_ini(
            os.path.join(OUT, f"{D.stem(topo)}_pv.ini"), "pv", topo, labels,
            update_jitter=args.update_jitter)

    tasks = [(t, n, r) for t in topos for n in names for r in range(H.REPEATS)]
    res, routes, dirs = defaultdict(list), defaultdict(list), []
    with ThreadPoolExecutor(max_workers=H.JOBS or min(len(tasks), os.cpu_count())) as ex:
        futs = {ex.submit(run_one, t, inis.get(t), n, wins, simt, r, run_sims): (t, n, r)
                for t, n, r in tasks}
        for i, f in enumerate(as_completed(futs), 1):
            topo, name, rep = futs[f]
            win, rt, resdir = f.result()
            dirs.append((D.stem(topo), name, resdir))
            for lab, vals in win.items():
                res[(lab, name)].append(vals)
                if lab in rt:
                    routes[(lab, name)].append(rt[lab])
            print(f"  ✓ [{i}/{len(tasks)}] {D.stem(topo)} {name} seed {rep}", flush=True)

    if not any(res.values()):
        sys.exit("FAIL: no txBytes vectors parsed — opp_scavetool missing, or no "
                 "runs on disk for --post")
    # Windows are placed from refreshInterval, not detected, so say where the
    # traffic actually landed: a burst within 10s of an edge is a split refresh.
    edges = {round(b) for _, _, b in wins}
    for stem_, name, resdir in dirs[:1]:
        peaks = bursts(resdir)
        near = [t for t in peaks if any(abs(t - e) < 10 for e in edges)]
        print(f"\n  [alignment] {stem_} {name}: traffic at "
              + ", ".join(f"{t:g}s" for t in peaks[:8]))
        if near:
            print("  ! traffic within 10s of a window edge ("
                  + ", ".join(f"{t:g}s" for t in near)
                  + ") — a refresh is being split across two windows, or the "
                    "formation has not finished before the first one")
    # Every window after `quiet` should hold the same routes. Fewer in `quiet`
    # means the formation was still running when the first refresh landed, so
    # `conv` is not a convergence cost and the ratios below mean nothing.
    for n in names:
        q, last = routes.get(("quiet", n)), routes.get((wins[-1][0], n))
        if q and last and sum(q) / len(q) < 0.99 * sum(last) / len(last):
            print(f"  ! {n}: {sum(q) / len(q):,.0f} routes held at the end of "
                  f"`quiet` against {sum(last) / len(last):,.0f} at the end of "
                  f"the run — the network had not converged before the first "
                  f"refresh. Raise --refresh or lower --warmup.")
    report(res, routes, wins, names)
    plot(res, wins, names)


if __name__ == "__main__":
    main()
