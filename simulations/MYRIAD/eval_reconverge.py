#!/usr/bin/env python3
"""Reconvergence vs failure size: how long until every router has full routing
info again, as a function of how many nodes failed at once?

eval_recovery.py measures per-node recovery latency at a single fixed failure
fraction. This sweeps the fraction and, for each, reduces the per-router recovery
latencies to two numbers: the network-wide reconvergence time (MAX across all
affected routers = the last one to refill) and the MEAN. One curve per protocol:
GP with recovery-replay on vs LS.

A single failure wave per run: CatastrophicFailure fails `fraction` of nodes at
t=WARMUP and reboots them after DURATION; PERIOD > SIMTIME ⇒ no second wave. GP
and LS run the SAME failed set at each fraction (dynamics RNG isolated). The
recovery machinery (RecoveryNetwork, stats snapshots, recoveryReplayEnabled) is
the same one eval_recovery.py exercises.

    python3 simulations/MYRIAD/eval_reconverge.py [--topo PATH] [--basis {min,all}]
                                                  [--fractions 0.05,0.1,0.2,...]

Outputs a latency table + reconverge_vs_fraction.png under
simulations/MYRIAD/eval/reconverge/.
"""
import argparse
import glob
import os
import sys

from eval_harness import (BIN, apply_repeat_args, ensure_ring_labels, pm,
                          rep_path, repeat_mean, repeats_note, run_sim,
                          write_proto_ini, add_topo_args)
from eval_recovery import load, RECOVERY_NETWORK

HERE = os.path.dirname(os.path.abspath(__file__))
OUT = os.path.join(HERE, "eval", "reconverge")

WARMUP = "20s"       # let initial convergence settle (GP warmup 10s default) first
DURATION = "10s"     # downtime before reboot
PERIOD = "9999s"     # > SIMTIME ⇒ exactly one failure wave
SIMTIME = "100s"     # GP heals ~1 ring-hop/refresh: ~150s+ at low fractions, more higher
SNAP = "2s"
DEFAULT_FRACTIONS = [0.05, 0.1, 0.2, 0.3, 0.4]

# (name, proto, dump param, recoveryReplayEnabled)
CONFIGS = [
    ("gp", "gp", "ribDumpFile",    "true"),
    ("ls", "ls", "oracleDumpFile", None),
]


def _sec(s):
    return float(s.rstrip("s"))


def reconverge_latencies(series, target, wave_t, reboot_t):
    """Per affected router: seconds from reboot until its route count reaches and
    HOLDS `target[nid]` — the true per-node reachability, taken from the LS oracle's
    converged final counts (topology-agnostic; the same failed set restores the same
    reachability under either protocol). Using LS truth rather than each config's own
    final matters because GP heals only ~1 ring-hop per refresh cycle: at a short
    SIMTIME its own last-snapshot count can still be mid-reconvergence, which would
    make the node look "settled" at a wrong, low target and undercount the latency.
    'Affected' = count dipped below target at some t >= wave_t (post-wave, so the
    startup ramp isn't counted). Latency = first snapshot at/after which it stayed at
    or above target through the end.

    Returns (latencies, converged): converged is False if any affected node never
    reached target by the last snapshot ⇒ SIMTIME too short, the numbers undercount."""
    out = {}
    converged = True
    for nid, ser in series.items():
        tgt = target.get(nid)
        if tgt is None:
            continue
        ts = sorted(ser)
        if not any(t >= wave_t and ser[t] < tgt for t in ts):
            continue  # never dipped ⇒ untouched by this wave
        recovered = None
        for i, t in enumerate(ts):
            if t >= wave_t and all(ser[t2] >= tgt for t2 in ts[i:]):
                recovered = t
                break
        if recovered is None:                     # still below target at the end
            converged = False
            out[nid] = max(0.0, ts[-1] - reboot_t)  # record as ≥ this (undercount)
        else:
            out[nid] = max(0.0, recovered - reboot_t)
    return out, converged


def run(name, ini, dumppar, replay, fraction, rep=0):
    resdir = rep_path(os.path.join(OUT, f"res_{name}_{fraction}"), rep)
    dump = rep_path(os.path.join(OUT, f"{name}_{fraction}.rib"), rep)
    for stale in (dump, *glob.glob(resdir + "/*.sca")):
        if os.path.exists(stale):
            os.remove(stale)
    args = [BIN, "-u", "Cmdenv", ini, f"--sim-time-limit={SIMTIME}",
            f'--**.{dumppar}="{dump}"',
            f"--**.dynamics.fraction={fraction}",
            # CLI overrides eval_common's 100s (ini include wins over in-ini lines,
            # but --opts beat both): fast refresh so the refresh-bound recovery tail
            # (chains whose ring king also failed) heals within the window.
            "--**.refreshInterval=101s", "--**.refreshJitter=0s",
            # isolate the dynamics RNG so GP and LS see the SAME failed set
            # (run_sim pins the seed-set per repeat)
            "--num-rngs=2", "--**.dynamics.rng-0=1",
            "--result-dir", resdir]
    if replay is not None:
        args.append(f"--**.recoveryReplayEnabled={replay}")
    # a big wave can transiently break rings; tolerate it and skip empty points
    run_sim(args, allow_budget_expired=True, rep=rep)
    return dump if os.path.exists(dump) else None


def main(args=None):
    if args is None:
        p = argparse.ArgumentParser()
        add_topo_args(p)
        p.add_argument("--fractions", default=None,
                       help="comma-separated failure fractions, e.g. 0.1,0.3")
        args = p.parse_args()
        apply_repeat_args(args)

    fractions = ([float(x) for x in args.fractions.split(",")]
                 if getattr(args, "fractions", None) else DEFAULT_FRACTIONS)

    os.makedirs(OUT, exist_ok=True)
    if not os.path.exists(BIN):
        sys.exit(f"RoutingSim binary not found at {BIN} — build it first.")

    labels = ensure_ring_labels(args.topo, args.basis)
    extra = [
        f"**.dynamics.warmup = {WARMUP}",
        f"**.dynamics.period = {PERIOD}",
        f"**.dynamics.duration = {DURATION}",
        "**.dynamics.minDegree = 0",   # grid corners are degree 2; fail any node
        f"**.snapshotInterval = {SNAP}",
        '**.snapshotFormat = "stats"',
    ]
    inis = {
        proto: write_proto_ini(
            os.path.join(OUT, f"{proto}.ini"), proto, args.topo, labels,
            extra_lines=extra,
            update_jitter=args.update_jitter if proto == "gp" else None,
            warmup=args.warmup if proto == "gp" else None,
            network=RECOVERY_NETWORK)
        for proto in ("gp", "ls")
    }

    dur_s = float(DURATION.rstrip("s"))
    wave_t = _sec(WARMUP)
    reboot_t = wave_t + dur_s
    # results[name] = {fraction: (n_affected, mean, median, net_max)}
    results = {name: {} for name, *_ in CONFIGS}
    # One repeat = one fraction's LS+GP pair on the same seed: LS's converged
    # counts are the truth target for GP, so both must see the SAME failed set.
    def one(frac, rep):
        # LS first: it reconverges fast and correctly, so its converged per-node
        # counts are the truth target both configs must reach again (same failed
        # set ⇒ same restored reachability). See reconverge_latencies.
        series = {}
        for name, proto, dumppar, replay in CONFIGS:
            if rep == 0:
                print(f"  → {name} fraction={frac:.0%} …", flush=True)
            dump = run(name, inis[proto], dumppar, replay, frac, rep)
            if dump is None:
                print(f"  (skipped {name} @ {frac:.0%} rep {rep}: no dump)")
                continue
            series[name] = load(dump)
        if "ls" not in series:
            print(f"  (skipped @ {frac:.0%} rep {rep}: no LS target)")
            return {}
        target = {nid: ser[max(ser)] for nid, ser in series["ls"].items()}
        out = {}
        for name, *_ in CONFIGS:
            if name not in series:
                continue
            lat, converged = reconverge_latencies(series[name], target,
                                                   wave_t, reboot_t)
            vals = sorted(lat.values())
            if not vals:
                print(f"  (skipped {name} @ {frac:.0%} rep {rep}: no affected routers)")
                continue
            if not converged:
                print(f"  ⚠ {name} @ {frac:.0%} rep {rep}: some routers still below "
                      f"full reachability at {SIMTIME} — raise SIMTIME; numbers undercount")
            med = vals[len(vals) // 2]
            out[name] = (len(vals), sum(vals) / len(vals), med, max(vals))
        return out

    for frac in fractions:
        agg = repeat_mean(lambda rep, frac=frac: one(frac, rep))
        for name, r in agg.items():
            results[name][frac] = r
            print(f"  ✓ {name} @ {frac:.0%}: {r[0]:.0f} routers, "
                  f"mean={r[1]:.1f}{pm(r[1])}s, net(max)={r[3]:.0f}s")

    # --- table ---
    print(f"\n=== reconvergence vs failure fraction "
          f"(single wave, dur {DURATION}, {SIMTIME} sims){repeats_note()} ===")
    print(f"{'fraction':>9}{'config':>8}{'routers':>9}{'mean':>14}"
          f"{'median':>9}{'net(max)':>13}")
    for frac in fractions:
        for name, *_ in CONFIGS:
            r = results[name].get(frac)
            if r is None:
                print(f"{frac:>8.0%}{name:>8}{'-':>9}{'-':>14}{'-':>9}{'-':>13}")
            else:
                n, mean, med, net = r
                print(f"{frac:>8.0%}{name:>8}{n:>9.0f}"
                      f"{f'{mean:.1f}' + pm(mean) + 's':>14}"
                      f"{med:>8.0f}s{f'{net:.0f}' + pm(net, '.0f') + 's':>13}")
    print("\nnet(max) = time until the LAST affected router is full again "
          "(= whole-network reconvergence); mean = average across routers.")

    try:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
    except ImportError:
        print("\n(matplotlib not available — skipping plot)")
        return

    fig, ax = plt.subplots()
    for name, color in (("gp", "C0"), ("ls", "C2")):
        fr = sorted(results[name])
        if not fr:
            continue
        xs = [f * 100 for f in fr]
        ax.plot(xs, [results[name][f][3] for f in fr], "o-", color=color,
                label=f"{name.upper()} network-wide (max)")
        ax.plot(xs, [results[name][f][1] for f in fr], "s--", color=color,
                alpha=0.6, label=f"{name.upper()} mean")
    ax.set(xlabel="nodes failed (%)", ylabel="reconvergence time (s)",
           title="Reconvergence time vs failure size")
    ax.legend(fontsize=8); fig.tight_layout()
    path = os.path.join(OUT, "reconverge_vs_fraction.png")
    fig.savefig(path, dpi=120)
    print(f"\nplot -> {path}")


if __name__ == "__main__":
    main()
