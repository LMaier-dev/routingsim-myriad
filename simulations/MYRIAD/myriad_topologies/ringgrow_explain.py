#!/usr/bin/env python3
"""Draw ringgrow's pipeline, one panel per pass, on an example small enough to
read node by node.

The companion to ringpipe_explain.py, and it reuses that script's drawing so the
two figures are directly comparable -- the point of ringgrow is that it reaches
the same kind of plan without ringpipe's clique cover and length solver.

Stages come out of build() itself via its `stages` hook rather than being
re-implemented here, so the picture cannot drift from what the generator does:

    sites -> pair order -> lay A rings -> merge tr pairs -> spof -> pad

The last pass is also the whole of --sweep-n: padding moves n without touching
the ring graph, so panel 6 is that sweep drawn once. Panel 8 spells it out
against panel 4's --sweep-tr.

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
from ringpipe_explain import (RING_COLORS, draw_rings, head, interfaces,
                              plural, spof_nodes)


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

    # Ring dots sit at their site positions, so three near-collinear sites hide
    # the outer pair's edge behind the middle dot. Panels 2 and 7 bow their edges.
    RAD = 0.12
    CURVE = f"arc3,rad={RAD}"

    fig, axes = plt.subplots(2, 4, figsize=(22, 13))
    ax = axes.ravel()

    # 1. sites
    nx.draw_networkx_edges(ax=ax[0], G=sg, pos=spos, edge_color="0.6")
    nx.draw_networkx_nodes(ax=ax[0], G=sg, pos=spos, node_size=560,
                           node_color=[RING_COLORS[i % len(RING_COLORS)] for i in sg])
    nx.draw_networkx_labels(ax=ax[0], G=sg, pos=spos, font_size=12, font_color="w")
    head(ax[0], f"1. place {a.rings} rings",
         "one dot per ring, which is what a ring is in the overlay: a node. "
         "Grey lines say which rings are near each other, and they fix only "
         "the ORDER pairs are joined in, never how many.")

    # 2. the merge order. Numbered rather than grouped by tree/site/rest: the
    # numbers ARE the order, and re-deriving the groups here would duplicate
    # _pair_order and be free to disagree with it.
    nx.draw_networkx_edges(ax=ax[1], G=sg, pos=spos, edge_color="0.88")
    for k, (u, v) in enumerate(pairs):
        ax[1].annotate("", xy=spos[v], xytext=spos[u],
                       arrowprops=dict(arrowstyle="-", lw=2.0, color="tab:blue",
                                       alpha=0.8, connectionstyle=CURVE))
        # On the curve, not the chord: three near-collinear sites would otherwise
        # put the outer pair's number on the middle ring's dot. arc3 bows to
        # mid + rad/2 * perp(v-u).
        (x0, y0), (x1, y1) = spos[u], spos[v]
        ax[1].annotate(str(k + 1),
                       ((x0 + x1) / 2 + RAD / 2 * (y1 - y0),
                        (y0 + y1) / 2 - RAD / 2 * (x1 - x0)),
                       ha="center", va="center", fontsize=8, color="tab:blue",
                       bbox=dict(boxstyle="circle,pad=0.12", fc="white",
                                 ec="tab:blue", lw=0.8))
    nx.draw_networkx_nodes(ax=ax[1], G=sg, pos=spos, node_size=560,
                           node_color=[RING_COLORS[i % len(RING_COLORS)] for i in sg])
    nx.draw_networkx_labels(ax=ax[1], G=sg, pos=spos, font_size=12, font_color="w")
    head(ax[1], f"2. choose which {a.tr} pairs meet",
         "numbered in join order: a spanning tree first, so every ring is "
         "reachable at the smallest count, then the nearest pairs. Each pair "
         "is joined once, so the count is exact.")

    # 3-6. the router graph, pass by pass
    draw_rings(ax[2], *stage["laid"], rpos,
               f"3. lay {a.rings} separate rings",
               "the underlay: routers and links. No ring touches another "
               "yet, and each holds two spare routers for every transition "
               "it will carry.\n" + note(*stage["laid"]))
    draw_rings(ax[3], *stage["merged"], rpos,
               f"4. join the {a.tr} pairs",
               "two routers of one ring are handed to the other, so both "
               "own them: they become transition routers and the pair now "
               "has one transition. Each join costs two routers.\n"
               + note(*stage["merged"]))
    draw_rings(ax[4], *stage["narrowed"], rpos,
               "5. squeeze some meeting places to one router",
               "one router is handed back, so the pair meets at a single "
               "transition router (red): a single point of failure, since "
               "losing it severs them.\n"
               + note(*stage["narrowed"]),
               mark=spof_nodes(stage["narrowed"][1]))
    draw_rings(ax[5], g3, plan3, rpos,
               f"6. add spare routers back to {a.target_n}",
               "spread evenly, so all rings grow alike. Member counts rise, "
               "the ring plan does not change: --sweep-n is this step "
               "alone.\n"
               + note(g3, plan3))

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
                           connectionstyle=CURVE, arrows=True, arrowstyle="-",
                           edge_color=["tab:red" if wid[e] == 1 else "0.45"
                                       for e in rgg.edges()])
    nx.draw_networkx_nodes(ax=ax[6], G=rgg, pos=rpos_d, node_size=560,
                           node_color=[RING_COLORS[k % len(RING_COLORS)] for k in rgg])
    nx.draw_networkx_labels(ax=ax[6], G=rgg, pos=rpos_d, font_size=12, font_color="w")
    nx.draw_networkx_edge_labels(ax=ax[6], G=rgg, pos=rpos_d, font_size=11,
                                 connectionstyle=CURVE,
                                 edge_labels={e: str(w) for e, w in wid.items()})
    head(ax[6], f"7. what the routing sees: {rgg.number_of_edges()} transitions",
         "the overlay: one node per ring, one link per transition, and "
         "exactly panel 2, because the count was chosen up front and never "
         "solved for. The label is the transition width, red means width 1.")

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
    ok = "\n".join(f"  {name:<28} {got:>5}  {'ok' if got == want else f'!= {want}'}"
                   for name, got, want in checks)
    ax[7].text(0.0, 1.0,
               "the settings used here\n\n"
               f"--rings {a.rings:<8} {a.rings} rings (panel 1)\n"
               f"--tr {a.tr:<11} pairs that meet (panel 2)\n"
               f"--spof {a.spof:<9g} {s_} of them squeezed to 1 router\n"
               f"--ring-size {a.ring_size:<4} shortest ring allowed\n"
               f"--target-n {a.target_n:<5} pad with spares until {a.target_n}\n\n"
               "no solver needed: fix the ring and node count and the rest\n"
               "follows in closed form. Checked on this plan:\n\n"
               f"{ok}\n\n"
               "member count and overlay degree therefore MOVE with tr, by\n"
               "exactly this much: the price of the x axis, not a confound.\n\n"
               "  --sweep-tr  n fixed, the ring graph IS the axis\n"
               "  --sweep-n   tr fixed, panel 6 alone: same ring graph at\n"
               "              every point, only the member counts rise",
               transform=ax[7].transAxes, va="top", ha="left", fontsize=11.5,
               family="monospace")

    fig.suptitle(f"ringgrow.py --family {a.family} --rings {a.rings} --tr {a.tr} "
                 f"--spof {a.spof:g} --ring-size {a.ring_size} "
                 f"--target-n {a.target_n} --seed {a.seed}\n"
                 f"result: {n} routers in {fin['rings']} rings, "
                 f"{plural(tr_, 'transition')} between them", fontsize=15)
    fig.tight_layout(rect=(0, 0, 1, 0.94))
    os.makedirs(os.path.dirname(a.out), exist_ok=True)
    fig.savefig(a.out, dpi=200)
    print(f"wrote {a.out}")


if __name__ == "__main__":
    main()
