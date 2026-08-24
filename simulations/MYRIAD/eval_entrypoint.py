#!/usr/bin/env python3
"""Single entry point for all MYRIAD eval scripts.

Runs the oracle regression test first, then dispatches to the selected eval.

    python3 eval_entrypoint.py {convergence,costdrift,overhead,recovery} [--topo PATH] [--basis {min,all}]
    python3 eval_entrypoint.py scaling [--quick] [--only ...] [--basis {min,all}] [--single TOPO] ...
"""
import argparse
import sys

from eval_harness import add_topo_args


def _oracle_gate(args):
    import test_rib_oracle
    print("=== oracle regression gate ===")
    test_rib_oracle.main(args)
    print()


def main():
    p = argparse.ArgumentParser(prog="eval_entrypoint",
                                description="Run MYRIAD evals (oracle gate runs first)")
    sub = p.add_subparsers(dest="cmd", required=True)

    for name in ("convergence", "costdrift", "overhead", "recovery"):
        sp = sub.add_parser(name)
        add_topo_args(sp)

    sc = sub.add_parser("scaling")
    sc.add_argument("--quick", action="store_true")
    sc.add_argument("--only", choices=["square", "width", "ladder", "flower", "topos"])
    sc.add_argument("--no-batch", dest="nobatch", action="store_true")
    sc.add_argument("--convergence", action="store_true")
    sc.add_argument("--basis", choices=["min", "all"], default="min")
    sc.add_argument("--single", metavar="TOPO")

    args = p.parse_args()

    if args.cmd != "scaling":
        _oracle_gate(args)

    dispatch = {
        "convergence": "eval_convergence",
        "costdrift":   "eval_costdrift",
        "overhead":    "eval_overhead",
        "recovery":    "eval_recovery",
        "scaling":     "eval_scaling",
    }
    mod = __import__(dispatch[args.cmd])
    mod.main(args)


if __name__ == "__main__":
    main()
