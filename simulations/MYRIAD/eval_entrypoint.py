#!/usr/bin/env python3
"""Single entry point for all MYRIAD eval scripts.

Runs the oracle regression test first, then dispatches to the selected eval.

    python3 eval_entrypoint.py {convergence,costdrift,dynamics} [--topo PATH] [--basis {min,tree}]
    python3 eval_entrypoint.py scaling [--quick] [--only ...] [--basis {min,tree}] [--single TOPO] ...

Run conditions come from eval_common.ini; --refresh/--warmup/--snapshot/--set
change them globally and every script prints what deviates.

--repeat N runs every measurement point N times in parallel (--jobs, default
min(N, cpu count)) and reports mean±sd; run 0 streams its progress, the rest run
silent on their own result dirs.
"""
import argparse
import sys

from eval_harness import (add_repeat_args, add_topo_args, apply_repeat_args,
                          gate_up_to_date, mark_gate_passed)


def _oracle_gate(args):
    if gate_up_to_date():
        print("=== oracle regression gate: skipped (binary unchanged) ===\n")
        return
    import test_rib_oracle, eval_harness
    print("=== oracle regression gate ===")
    saved = eval_harness.EXTRA_OPTS
    eval_harness.EXTRA_OPTS = [o for o in saved if "snapshotInterval" not in o]
    try:
        test_rib_oracle.main(args)   # sys.exit on failure, so reaching here == pass
    finally:
        eval_harness.EXTRA_OPTS = saved
    mark_gate_passed()
    print()


def main():
    p = argparse.ArgumentParser(prog="eval_entrypoint",
                                description="Run MYRIAD evals (oracle gate runs first)")
    sub = p.add_subparsers(dest="cmd", required=True)

    for name in ("convergence", "costdrift", "dynamics"):
        sp = sub.add_parser(name)
        add_topo_args(sp)
        if name == "costdrift":
            sp.add_argument("--nospof", action="store_true",
                            help="also run PV with rankUseSpof=false, whose "
                                 "pure-cost chains should land on the oracle")
            sp.add_argument("--no-refresh-flood", action="store_true",
                            help="PV's routine refresh only re-verifies ring "
                                 "state, no reflood")
        if name == "dynamics":
            sp.add_argument("--only", choices=["churn", "link", "node"])
            sp.add_argument("--fractions", default=None,
                            help="node-wave failure fractions, e.g. 0.1,0.3")
            sp.add_argument("--churn-nodes", type=int, default=None,
                            help="nodes failing per churn period (default 1)")
            sp.add_argument("--churn-period", type=float, default=None,
                            help="seconds between churn failures (default 1)")
            sp.add_argument("--down-min", type=float, default=None,
                            help="minimum node downtime in seconds (default 2)")
            sp.add_argument("--down-jitter", type=float, default=None,
                            help="node downtime jitter in seconds (default 8)")
            sp.add_argument("--wave-spacing", type=float, default=None,
                            help="settle time between node waves (one failure "
                                 "fraction to the next); also the refresh "
                                 "interval of the node scenario")
            sp.add_argument("--failure-start", type=float, default=None,
                            help="warmup seconds before failures begin (churn "
                                 "start and first node wave) and the cut every "
                                 "post-warmup measurement uses (default 100)")
            sp.add_argument("--extra-warmup", type=float, default=0.0, metavar="S",
                            help="give the network S more seconds to converge "
                                 "before the first failure (adds to "
                                 "--failure-start; the run horizon follows)")
            sp.add_argument("--noreplay", action="store_true",
                            help="also run PV with recoveryReplayEnabled=false")
            sp.add_argument("--nospof", action="store_true",
                            help="also run PV with rankUseSpof=false, for a "
                                 "pure-cost stretch against LS")
            sp.add_argument("--no-refresh-flood", action="store_true",
                            help="PV's routine refresh only re-verifies ring "
                                 "state, no reflood (pv config only)")
            sp.add_argument("--protos", default=None,
                            help="exact config names to run (see --list-configs), "
                                 "default all; split across panes, then --post "
                                 "to combine")
            sp.add_argument("--list-configs", action="store_true",
                            help="print the config names the other flags select "
                                 "and exit (run_evals.sh SPLIT uses this)")
            sp.add_argument("--post", action="store_true",
                            help="re-parse prior runs' result dirs and draw the "
                                 "combined plots without simulating")
            sp.add_argument("--dir", default=None, metavar="PATH",
                            help="overview mode only: run every *.txt topology "
                                 "in this directory instead of --topo, so the "
                                 "boxes span topologies as well as RNG seeds")
            sp.add_argument("--tr-extremes", action="store_true",
                            help="with --dir: run only the lowest-_tr and "
                                 "highest-_tr topologies in it, every seed of each — "
                                 "the two ends of a transition sweep at a fraction of "
                                 "the sim cost")
            sp.add_argument("--overview", action="store_true",
                            help="traffic-only mode: messages and bytes per "
                                 "failure window (churn, each node-wave "
                                 "fraction), boxplotted over --repeat RNG seeds; "
                                 "no RIB snapshots, so no reconvergence output")

    import eval_optimizations
    op = sub.add_parser("optimizations")
    eval_optimizations.add_opt_args(op)
    add_repeat_args(op)

    sc = sub.add_parser("scaling")
    sc.add_argument("--quick", action="store_true")
    sc.add_argument("--only", choices=["square", "topos"])
    sc.add_argument("--no-batch", dest="nobatch", action="store_true")
    sc.add_argument("--convergence", action="store_true")
    sc.add_argument("--basis", choices=["min", "tree", "short", "all"], default="min")
    sc.add_argument("--single", metavar="TOPO")
    add_repeat_args(sc)

    # Run the oracle gate on its own and exit. run_evals.sh calls this once,
    # serially, before fanning out: the gate writes shared scratch under
    # eval/oracle/, so N evals starting at once would race that dir (interleaved
    # RIB dumps -> KeyError). One pre-run sets the binary-keyed stamp and every
    # parallel eval then skips the gate.
    g = sub.add_parser("gate")
    add_topo_args(g)

    args = p.parse_args()
    # costdrift reads only the final snapshot.
    if args.cmd == "costdrift" and not getattr(args, "snapshot", None):
        import eval_costdrift
        args.snapshot = eval_costdrift.SNAPSHOT
    apply_repeat_args(args)

    if args.cmd == "gate":
        _oracle_gate(args)
        return

    # Post mode re-parses on-disk results and --list-configs only prints names;
    # neither simulates, so the oracle gate (which does) would be pure waste.
    nosim = (args.cmd == "dynamics" and (getattr(args, "post", False)
                                         or getattr(args, "list_configs", False))
             or (args.cmd == "optimizations" and getattr(args, "dry_run", False)))
    # scaling and optimizations sweep whole topology families and carry no --topo
    # for the gate to run on; run_evals.sh stamps the gate once before fanning out.
    if args.cmd not in ("scaling", "optimizations") and not nosim:
        _oracle_gate(args)

    dispatch = {
        "convergence": "eval_convergence",
        "costdrift":   "eval_costdrift",
        "dynamics":    "eval_dynamics",
        "optimizations": "eval_optimizations",
        "scaling":     "eval_scaling",
    }
    mod = __import__(dispatch[args.cmd])
    mod.main(args)


if __name__ == "__main__":
    main()
