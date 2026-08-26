#!/usr/bin/env python3
"""Single entry point for all MYRIAD eval scripts.

Runs the oracle regression test first, then dispatches to the selected eval.

    python3 eval_entrypoint.py {convergence,costdrift,dynamics} [--topo PATH] [--basis {min,all}]
    python3 eval_entrypoint.py scaling [--quick] [--only ...] [--basis {min,all}] [--single TOPO] ...

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
    import test_rib_oracle
    print("=== oracle regression gate ===")
    test_rib_oracle.main(args)   # sys.exit on failure, so reaching here == pass
    mark_gate_passed()
    print()


def main():
    p = argparse.ArgumentParser(prog="eval_entrypoint",
                                description="Run MYRIAD evals (oracle gate runs first)")
    sub = p.add_subparsers(dest="cmd", required=True)

    for name in ("convergence", "costdrift", "dynamics"):
        sp = sub.add_parser(name)
        add_topo_args(sp)
        if name == "dynamics":
            sp.add_argument("--only", choices=["static", "link", "node"])
            sp.add_argument("--fractions", default=None,
                            help="node-wave failure fractions, e.g. 0.1,0.3")
            sp.add_argument("--down-min", type=float, default=None,
                            help="minimum node downtime in seconds (default 2)")
            sp.add_argument("--down-jitter", type=float, default=None,
                            help="node downtime jitter in seconds (default 8)")
            sp.add_argument("--wave-spacing", type=float, default=None,
                            help="seconds between waves; also the refresh "
                                 "interval of the node scenario (default 30)")
            sp.add_argument("--noreplay", action="store_true",
                            help="also run GP with recoveryReplayEnabled=false")
            sp.add_argument("--no-refresh-flood", action="store_true",
                            help="GP's routine refresh only re-verifies ring "
                                 "state, no reflood (gp config only)")

    sc = sub.add_parser("scaling")
    sc.add_argument("--quick", action="store_true")
    sc.add_argument("--only", choices=["square", "width", "ladder", "flower", "topos"])
    sc.add_argument("--no-batch", dest="nobatch", action="store_true")
    sc.add_argument("--convergence", action="store_true")
    sc.add_argument("--basis", choices=["min", "all"], default="min")
    sc.add_argument("--single", metavar="TOPO")
    add_repeat_args(sc)

    args = p.parse_args()
    apply_repeat_args(args)

    if args.cmd != "scaling":
        _oracle_gate(args)

    dispatch = {
        "convergence": "eval_convergence",
        "costdrift":   "eval_costdrift",
        "dynamics":    "eval_dynamics",
        "scaling":     "eval_scaling",
    }
    mod = __import__(dispatch[args.cmd])
    mod.main(args)


if __name__ == "__main__":
    main()
