#!/usr/bin/env python3
"""Draw ringgrow's pipeline, one panel per pass, on an example small enough to
read node by node.

The companion to ringpipe_explain.py, and it reuses that script's drawing so the
two figures are directly comparable -- the point of ringgrow is that it reaches
the same kind of plan without ringpipe's clique cover and length solver.

Stages come out of build() itself via its `stages` hook rather than being
re-implemented here, so the picture cannot drift from what the generator does:

    sites -> pair order -> lay A rings -> merge tr pairs -> spof -> pad

    python3 myriad_topologies/ringgrow_explain.py [--rings 5] [--tr 7] ...

Writes gallery/ringgrow_pipeline.png.
"""
import argparse
import os
import sys

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import networkx as nx
import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)

import ringpipe as rp
import ringgrow as rg
from metrics import score
from ringpipe_explain import RING_COLORS, draw_rings, head, interfaces, plural


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--family", default="gabriel", choices=rp.SITE_FAMILIES)
    ap.add_argument("--rings", type=int, default=5, help="A: ring count, exact")
    ap.add_argument("--tr", type=int, default=7,
                    help="transitions, exact; at least A-1, at most A*(A-1)/2")
    ap.add_argument("--ring-size", type=int, default=6)
    ap.add_argument("--spof", type=float, default=0.35)
    ap.add_argument("--target-n", type=int, default=46)
    ap.add_argument("--seed", type=int, default=3)
    ap.add_argument("--out", default=os.path.join(rp.GALLERY, "ringgrow_pipeline.png"))
    a = ap.parse_args()

    st = {}
    g3, plan3, rpos = rg.build(rings=a.rings, seed=a.seed, ring_size=a.ring_size,
                               tr=a.tr, target_n=a.target_n, family=a.family,
                               spof=a.spof, stages=st)
    pts, pairs = st["pts"], st["pairs"]
    stage = {k: (rg._graph_of(st[k]), st[k]) for k in ("laid", "merged", "narrowed")}

    def note(g, plan):
        s = score(g, plan, check=False)
        return (f"{s['n']} routers, {s['rings']} rings, {s['pairs']} transitions, "
                f"fan-out {s['fan']}")

    # The site graph only decides WHICH pairs meet first; the count is --tr.
    sg = nx.Graph()
    sg.add_nodes_from(range(a.rings))
    for u, v in rp._raw_site_edges(a.family, pts, a.seed):
        sg.add_edge(u, v)
    spos = {i: tuple(pts[i]) for i in sg}

    fig, axes = plt.subplots(2, 4, figsize=(22, 11.5))
    ax = axes.ravel()

    # 1. sites
    nx.draw_networkx_edges(ax=ax[0], G=sg, pos=spos, edge_color="0.6")
    nx.draw_networkx_nodes(ax=ax[0], G=sg, pos=spos, node_size=420,
                           node_color=[RING_COLORS[i % len(RING_COLORS)] for i in sg])
    nx.draw_networkx_labels(ax=ax[0], G=sg, pos=spos, font_size=9, font_color="w")
    head(ax[0], f"1. place {a.rings} rings",
         "one dot per ring. The grey lines say which rings are near each other, "
         "which is all this graph is for: it decides the ORDER pairs are joined "
         "in, never how many.")

    # 2. the merge order. Numbered rather than grouped by tree/site/rest: the
    # numbers ARE the order, and re-deriving the groups here would duplicate
    # _pair_order and be free to disagree with it.
    nx.draw_networkx_edges(ax=ax[1], G=sg, pos=spos, edge_color="0.88")
    for k, (u, v) in enumerate(pairs):
        ax[1].annotate("", xy=spos[v], xytext=spos[u],
                       arrowprops=dict(arrowstyle="-", lw=2.0, color="tab:blue",
                                       alpha=0.8))
        ax[1].annotate(str(k + 1),
                       ((spos[u][0] + spos[v][0]) / 2, (spos[u][1] + spos[v][1]) / 2),
                       ha="center", va="center", fontsize=8, color="tab:blue",
                       bbox=dict(boxstyle="circle,pad=0.12", fc="white",
                                 ec="tab:blue", lw=0.8))
    nx.draw_networkx_nodes(ax=ax[1], G=sg, pos=spos, node_size=420,
                           node_color=[RING_COLORS[i % len(RING_COLORS)] for i in sg])
    nx.draw_networkx_labels(ax=ax[1], G=sg, pos=spos, font_size=9, font_color="w")
    head(ax[1], f"2. choose which {a.tr} pairs meet",
         "numbered in the order they are joined: the shortest tree first, so "
         "every ring is reachable at the smallest count the sweep can ask for, "
         "then the remaining near pairs, then anything left. Each pair is "
         "joined at most once, so a pair is a transition and the count is exact.")

    # 3-6. the router graph, pass by pass
    draw_rings(ax[2], *stage["laid"], rpos,
               f"3. lay {a.rings} separate rings",
               "no ring touches another yet. A ring gets two spare routers per "
               "transition it will carry, plus two kept at the end so the "
               "closing cable always joins spare to spare.\n"
               + note(*stage["laid"]))
    draw_rings(ax[3], *stage["merged"], rpos,
               f"4. join the {a.tr} pairs",
               "to join two rings, two spare routers of one are handed to the "
               "other: now both own them, and traffic can change ring there. "
               "That is one transition. The loops do not get longer -- the "
               "network gets SMALLER by two routers each time.\n"
               + note(*stage["merged"]))
    draw_rings(ax[4], *stage["narrowed"], rpos,
               "5. squeeze some meeting places to one router",
               "give one of the two routers back, so the pair now shares just "
               "one (red). They still meet -- lose that router and they cannot "
               "reach each other.\n" + note(*stage["narrowed"]),
               mark=[v for v in stage["narrowed"][0] if v not in stage["merged"][0]])
    draw_rings(ax[5], g3, plan3, rpos,
               f"6. add spare routers back to {a.target_n}",
               "spread evenly, so the loops stay about the same length. The "
               "network gets bigger; the number of transitions does not "
               "change.\n" + note(g3, plan3))

    # 7. the ring graph: what the plan looks like to the control plane
    ifc = interfaces(plan3)
    rgg = nx.Graph()
    rgg.add_nodes_from(range(len(plan3)))
    wid = {}
    for rs, vs in ifc.items():
        for i in sorted(rs):
            for j in sorted(rs):
                if i < j:
                    wid[(i, j)] = wid.get((i, j), 0) + len(vs)
    rgg.add_edges_from(wid)
    rpos_d = {k: rpos[k] for k in rgg}
    nx.draw_networkx_edges(ax=ax[6], G=rgg, pos=rpos_d, width=2.2,
                           edge_color=["tab:red" if wid[e] == 1 else "0.45"
                                       for e in rgg.edges()])
    nx.draw_networkx_nodes(ax=ax[6], G=rgg, pos=rpos_d, node_size=420,
                           node_color=[RING_COLORS[k % len(RING_COLORS)] for k in rgg])
    nx.draw_networkx_labels(ax=ax[6], G=rgg, pos=rpos_d, font_size=9, font_color="w")
    nx.draw_networkx_edge_labels(ax=ax[6], G=rgg, pos=rpos_d, font_size=7,
                                 edge_labels={e: str(w) for e, w in wid.items()})
    head(ax[6], f"7. what the routing sees: {rgg.number_of_edges()} transitions",
         "one dot per ring, one line per pair that meets -- and this is exactly "
         "the picture from panel 2, which is the point: the count was chosen up "
         "front, not solved for. The number on a line is how many routers the "
         "two rings share; red means one, so losing it cuts them apart.")

    # 8. the knobs, and the closed forms that make this generator worth having.
    # Predicted vs achieved rather than predicted alone: these identities are
    # the whole claim (no solver needed), so the panel should be falsifiable.
    ax[7].set_axis_off()
    fin = score(g3, plan3, check=False)
    n, tr_, s_ = fin["n"], fin["pairs"], fin["spof"]
    checks = [("transitions", tr_, a.tr),
              ("fan-out = 4*tr - 2*spof", fin["fan"], 4 * tr_ - 2 * s_),
              ("cables = n + tr", fin["m"], n + tr_),
              ("sum of ring lengths = n + 2*tr - spof",
               sum(len(r) for r in plan3), n + 2 * tr_ - s_)]
    ok = "\n".join(f"  {name:<38} {got:>6}  {'ok' if got == want else f'!= {want}'}"
                   for name, got, want in checks)
    ax[7].text(0.0, 1.0,
               "the settings used here\n\n"
               f"--rings {a.rings:<10} {a.rings} rings (panel 1)\n"
               f"--tr {a.tr:<13} {a.tr} pairs of them meet (panel 2)\n"
               f"--spof {a.spof:<11g} {s_} of the {tr_} squeezed to 1 router\n"
               f"--ring-size {a.ring_size:<6} every ring at least {a.ring_size} routers long\n"
               f"--target-n {a.target_n:<7} keep adding spare routers until "
               f"{a.target_n}\n\n"
               "ringgrow needs no solver: fix the ring count and the node\n"
               "count, and everything else follows in closed form. Measured\n"
               "against the identity, on this very plan:\n\n"
               f"{ok}\n\n"
               "so ring length and mean degree MOVE with tr, by exactly this\n"
               "much. That is the price of the x axis, not a confound -- which\n"
               "is what ringpipe's --hold-len solver was paying to avoid.\n\n"
               "the file name keeps the result:\n"
               f"  _A{fin['rings']} _tr{tr_} _fan{fin['fan']} _n{n}",
               transform=ax[7].transAxes, va="top", ha="left", fontsize=9.5,
               family="monospace")

    fig.suptitle(f"ringgrow.py --family {a.family} --rings {a.rings} --tr {a.tr} "
                 f"--spof {a.spof:g} --ring-size {a.ring_size} "
                 f"--target-n {a.target_n} --seed {a.seed}\n"
                 f"result: {n} routers in {fin['rings']} rings, "
                 f"{plural(tr_, 'transition')} between them", fontsize=11)
    fig.tight_layout(rect=(0, 0, 1, 0.94))
    os.makedirs(os.path.dirname(a.out), exist_ok=True)
    fig.savefig(a.out, dpi=120)
    print(f"wrote {a.out}")


if __name__ == "__main__":
    main()
