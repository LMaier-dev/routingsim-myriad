#!/usr/bin/env python3
"""Single entry point for all MYRIAD eval scripts.

Runs the oracle regression test first, then dispatches to the selected eval.

    python3 eval_entrypoint.py {convergence,costdrift,overhead,recovery} [--topo PATH] [--basis {min,all}]
    python3 eval_entrypoint.py scaling [--quick] [--only ...] [--basis {min,all}] [--single TOPO] ...

--repeat N runs every measurement point N times in parallel (--jobs, default
min(N, cpu count)) and reports mean±sd; run 0 streams its progress, the rest run
silent on their own result dirs. Default --repeat 1 = the previous behaviour.
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

    for name in ("convergence", "costdrift", "overhead", "recovery", "reconverge"):
        sp = sub.add_parser(name)
        add_topo_args(sp)
        if name == "reconverge":
            sp.add_argument("--fractions", default=None,
                            help="comma-separated failure fractions, e.g. 0.1,0.3")

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
        "overhead":    "eval_overhead",
        "recovery":    "eval_recovery",
        "reconverge":  "eval_reconverge",
        "scaling":     "eval_scaling",
    }
    mod = __import__(dispatch[args.cmd])
    mod.main(args)


if __name__ == "__main__":
    main()
