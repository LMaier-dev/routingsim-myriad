#!/usr/bin/env python3
"""Draw ringpipe's pipeline, one panel per pass, on an example small enough to
read node by node.

Calls ringpipe's own internals in build()'s order rather than re-implementing
them, so the picture cannot drift from what the generator does:

    sites -> clique cover -> rings -> _prune_solo -> _spof_final -> _pad_uniform

    python3 myriad_topologies/ringpipe_explain.py [--rings 5] [--share 1] ...

Writes gallery/ringpipe_pipeline.png.
"""
import argparse
import math
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
from metrics import score
from topo_lib import _cap_degree

RING_COLORS = plt.get_cmap("tab10").colors


def plural(k, word):
    return f"{k} {word}" + ("" if k == 1 else "s")


def head(ax, title, note=""):
    """Panel title plus its wrapped explanation, both inside the axes' width —
    a long note as a figure-level text runs into the neighbouring panel. The
    figure is printed rotated and half a page wide, so the wrap is set by what
    stays legible at that size, not by how much fits."""
    import textwrap
    ax.set_title(title + ("\n" + "\n".join(textwrap.fill(ln, 50, break_on_hyphens=False)
                                           for ln in note.split("\n")) if note else ""),
                 fontsize=15 if not note else 13, linespacing=1.3)
    ax.set_axis_off()


def interfaces(plan):
    """{frozenset of rings: [nodes]} -- an interface is a set of nodes carrying
    the same rings, which is what _plan_cliques laid down as one clique."""
    memb = {}
    for k, r in enumerate(plan):
        for v in r:
            memb.setdefault(v, set()).add(k)
    out = {}
    for v, rs in memb.items():
        if len(rs) > 1:
            out.setdefault(frozenset(rs), []).append(v)
    return out


def spof_nodes(plan):
    """The single router a narrowed pair meets at: what panel 5 marks in red and
    what panel 7 draws as a red line. Derived from the finished plan, not by
    diffing node ids against the previous pass -- narrowing SPLITS a shared node
    and hands the new copy to one ring, so the id that appears is the solo half,
    not the point of failure, and _spof_final relabels besides."""
    memb = {}
    for k, r in enumerate(plan):
        for v in r:
            memb.setdefault(v, set()).add(k)
    shared = {}
    for v, rs in memb.items():
        for a in sorted(rs):
            for b in sorted(rs):
                if a < b:
                    shared.setdefault((a, b), []).append(v)
    return {vs[0] for vs in shared.values() if len(vs) == 1}


def layout(g, plan, rpos):
    """Node positions. Kamada-Kawai on the router graph itself: these examples
    are tiny, and it draws each ring as a visible loop with the interfaces
    pinched between them. Seeded from the per-site circles so a ring keeps
    roughly the place its site had, and panels stay comparable."""
    init, rad = {}, 0.30 / math.sqrt(max(len(plan), 1))
    for k, r in enumerate(plan):
        cx, cy = rpos[k]
        for j, v in enumerate(r):
            ang = 2 * math.pi * j / len(r)
            init.setdefault(v, []).append((cx + rad * math.cos(ang),
                                           cy + rad * math.sin(ang)))
    init = {v: np.mean(ps, axis=0) for v, ps in init.items()}
    kk = nx.kamada_kawai_layout(g, pos={v: tuple(p) for v, p in init.items()})
    # A spring pass on top: at the pruned floor there is barely any filler left
    # to hold the rings apart, and KK alone folds them into slivers.
    return nx.spring_layout(g, pos=kk, iterations=60, seed=0)


def draw_rings(ax, g, plan, rpos, title, note="", mark=(), label_ifaces=False):
    """One pipeline stage: the router graph, each ring in its own colour, shared
    nodes (interfaces) drawn large and dark, filler small and white."""
    pos = layout(g, plan, rpos)
    memb = {}
    for k, r in enumerate(plan):
        for v in r:
            memb.setdefault(v, set()).add(k)
    nx.draw_networkx_edges(ax=ax, G=g, pos=pos, edge_color="0.75", width=0.8)
    # An edge inside an interface belongs to EVERY ring meeting there, so one
    # colour per edge would hide all but the last drawn. Draw the rings that
    # carry it as parallel strands, offset along the edge's normal.
    carriers = {}
    for k, r in enumerate(plan):
        for j in range(len(r)):
            e = (min(r[j], r[(j + 1) % len(r)]), max(r[j], r[(j + 1) % len(r)]))
            carriers.setdefault(e, []).append(k)
    for (u, v), ks in carriers.items():
        (x0, y0), (x1, y1) = pos[u], pos[v]
        dx, dy = x1 - x0, y1 - y0
        L = math.hypot(dx, dy) or 1.0
        nx_, ny_ = -dy / L, dx / L          # unit normal to the edge
        for i, k in enumerate(ks):
            off = (i - (len(ks) - 1) / 2) * 0.016
            ax.plot([x0 + nx_ * off, x1 + nx_ * off],
                    [y0 + ny_ * off, y1 + ny_ * off], "-", lw=1.8, alpha=0.95,
                    color=RING_COLORS[k % len(RING_COLORS)], zorder=1)
    solo = [v for v in g if len(memb.get(v, ())) < 2]
    shared = [v for v in g if len(memb.get(v, ())) > 1]
    nx.draw_networkx_nodes(ax=ax, G=g, pos=pos, nodelist=solo, node_size=22,
                           node_color="white", edgecolors="0.6", linewidths=0.8)
    nx.draw_networkx_nodes(ax=ax, G=g, pos=pos, nodelist=shared, node_size=70,
                           node_color="0.15")
    if mark:
        nx.draw_networkx_nodes(ax=ax, G=g, pos=pos, nodelist=[v for v in mark if v in g],
                               node_size=150, node_color="none",
                               edgecolors="tab:red", linewidths=1.8)
    if label_ifaces:   # which rings meet here: the transitions this one carries
        for rs, vs in interfaces(plan).items():
            x = sum(pos[v][0] for v in vs) / len(vs)
            y = sum(pos[v][1] for v in vs) / len(vs)
            ax.annotate("rings " + ",".join(str(r) for r in sorted(rs)) + "\n"
                        + plural(len(rs) * (len(rs) - 1) // 2, "transition"),
                        (x, y), textcoords="offset points", xytext=(0, 14),
                        ha="center", fontsize=11, color="tab:blue")
    head(ax, title, note)


def stats_note(g, plan):
    s = score(g, plan, check=False)
    w = s["fan"] / (2 * s["pairs"]) if s["pairs"] else 0
    return (f"{s['n']} routers, {s['rings']} rings, {s['pairs']} transitions, "
            f"fan-out {s['fan']}")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--family", default="gabriel", choices=rp.SITE_FAMILIES)
    ap.add_argument("--rings", type=int, default=6, help="A: sites = rings")
    ap.add_argument("--ring-size", type=int, default=6)
    ap.add_argument("--share", type=float, default=1.0)
    ap.add_argument("--spof", type=float, default=0.35)
    ap.add_argument("--width-max", type=int, default=rp.WIDTH_MAX)
    ap.add_argument("--target-n", type=int, default=46)
    ap.add_argument("--prune-solo", type=float, default=1.0,
                    help="fraction of filler spliced out. build()'s default is "
                         "0 (no round trip at all); the sweeps pass 1.0, which "
                         "is what this picture shows")
    ap.add_argument("--seed", type=int, default=3)
    ap.add_argument("--out", default=os.path.join(rp.GALLERY, "ringpipe_pipeline.png"))
    a = ap.parse_args()

    # --- build()'s body, stopping at each pass ------------------------------
    # Same rng draws in the same order, so these ARE the intermediate states of
    # one build; a fresh rng for the cover panel repeats its draws exactly.
    rng = np.random.default_rng(a.seed)
    pts = rng.random((a.rings, 2))
    sg = nx.Graph()
    sg.add_nodes_from(range(a.rings))
    for u, v in rp._raw_site_edges(a.family, pts, a.seed):
        sg.add_edge(u, v, w=float(np.linalg.norm(pts[u] - pts[v])))
    sg = rp._hard_cap(_cap_degree(sg, a.ring_size // 2), a.ring_size // 2)

    m_max = 2 + int(round(a.share * (max(4, round(math.sqrt(a.rings))) - 2)))
    cliques = rp._clique_cover_geo(sg, pts, np.random.default_rng(a.seed), m_max)

    g0, plan0, rpos = rp._plan_cliques(sg, pts, rng, a.spof, a.share,
                                       a.ring_size, a.width_max)
    g1, plan1 = rp._prune_solo(g0.copy(), [list(r) for r in plan0],
                               a.prune_solo, rng)
    g2, plan2 = rp._spof_final(g1.copy(), [list(r) for r in plan1], a.spof, rng)
    g3, plan3 = rp._pad_uniform(g2.copy(), [list(r) for r in plan2], a.target_n, rng)

    fig, axes = plt.subplots(2, 4, figsize=(22, 13))
    ax = axes.ravel()

    # 1. sites
    spos = {i: tuple(pts[i]) for i in sg}
    nx.draw_networkx_edges(ax=ax[0], G=sg, pos=spos, edge_color="0.6")
    nx.draw_networkx_nodes(ax=ax[0], G=sg, pos=spos, node_size=560,
                           node_color=[RING_COLORS[i % len(RING_COLORS)] for i in sg])
    nx.draw_networkx_labels(ax=ax[0], G=sg, pos=spos, font_size=12, font_color="w")
    head(ax[0], f"1. place {a.rings} rings",
         "one dot per ring, which is what a ring is in the overlay: a node. "
         "Grey lines mark the rings close enough to meet.")

    # 2. clique cover: each clique becomes one interface.
    # Drawn as a hub with a spoke to each member site, NOT as coloured site
    # edges: a clique's pairs are not ring edges, and painting them in a ring's
    # colour reads as if they were (a 2-site clique became one lone line).
    site_of_ring = {k: min(sg.nodes(), key=lambda i: (pts[i][0] - p[0]) ** 2
                                                     + (pts[i][1] - p[1]) ** 2)
                    for k, p in enumerate(rpos)}
    width_of = {frozenset(site_of_ring[k] for k in rs): len(vs)
                for rs, vs in interfaces(plan0).items()}
    nx.draw_networkx_edges(ax=ax[1], G=sg, pos=spos, edge_color="0.85")
    for c in cliques:                     # a clique of m sites = m rings meeting
        cx = sum(spos[i][0] for i in c) / len(c)
        cy = sum(spos[i][1] for i in c) / len(c)
        for i in c:
            ax[1].plot(*zip((cx, cy), spos[i]), color="0.35", lw=1.4,
                       ls="--", zorder=1)
        ax[1].plot([cx], [cy], "s", color="0.15", ms=11, zorder=3)
        m = len(c)
        w = width_of.get(frozenset(c))
        ax[1].annotate(f"{plural(m, 'ring')}\n"
                       f"{plural(m * (m - 1) // 2, 'transition')}", (cx, cy),
                       textcoords="offset points", xytext=(0, 13), ha="center",
                       fontsize=11, color="tab:blue", zorder=4)
    nx.draw_networkx_nodes(ax=ax[1], G=sg, pos=spos, node_size=560,
                           node_color=[RING_COLORS[i % len(RING_COLORS)] for i in sg])
    nx.draw_networkx_labels(ax=ax[1], G=sg, pos=spos, font_size=12, font_color="w")
    head(ax[1], f"2. pick {len(cliques)} meeting places",
         f"each square is one place, the dashed lines say which rings meet "
         f"there. Every PAIR meeting is one transition, so m rings give "
         f"m(m-1)/2 of them.")

    # 3-6. the router graph, pass by pass
    draw_rings(ax[2], g0, plan0, rpos,
               "3. build the rings",
               f"the underlay: routers and links. A meeting place becomes "
               f"{a.width_max} transition routers, owned by every ring there "
               f"(black). The rest belong to one ring only (white).\n"
               + stats_note(g0, plan0), label_ifaces=True)
    draw_rings(ax[3], g1, plan1, rpos,
               "4. take the spare routers out",
               "a router in one ring only carries no transition, so cutting "
               "it out just shortens that ring. The floor of this plan.\n"
               + stats_note(g1, plan1))
    draw_rings(ax[4], g2, plan2, rpos,
               "5. squeeze some meeting places to one router",
               "the pair still meets, but at a single transition router "
               "(red): a single point of failure, since losing it severs "
               "them. Same transitions, less fan-out.\n"
               + stats_note(g2, plan2), mark=spof_nodes(plan2))
    draw_rings(ax[5], g3, plan3, rpos,
               f"6. add spare routers back to {a.target_n}",
               "spread evenly, so all rings grow alike. Member counts rise, "
               "the ring plan does not change.\n" + stats_note(g3, plan3))

    # 7. the ring graph: what the plan looks like to the control plane
    ifc = interfaces(plan3)
    rg = nx.Graph()
    rg.add_nodes_from(range(len(plan3)))
    wid = {}
    for rs, vs in ifc.items():
        for i in sorted(rs):
            for j in sorted(rs):
                if i < j:
                    wid[(i, j)] = wid.get((i, j), 0) + len(vs)
    rg.add_edges_from(wid)
    rgpos = {k: rpos[k] for k in rg}
    nx.draw_networkx_edges(ax=ax[6], G=rg, pos=rgpos, width=2.2,
                           edge_color=["tab:red" if wid[e] == 1 else "0.45"
                                       for e in rg.edges()])
    nx.draw_networkx_nodes(ax=ax[6], G=rg, pos=rgpos, node_size=560,
                           node_color=[RING_COLORS[k % len(RING_COLORS)] for k in rg])
    nx.draw_networkx_labels(ax=ax[6], G=rg, pos=rgpos, font_size=12, font_color="w")
    nx.draw_networkx_edge_labels(ax=ax[6], G=rg, pos=rgpos, font_size=11,
                                 edge_labels={e: str(w) for e, w in wid.items()})
    nred = sum(1 for e in rg.edges() if wid[e] == 1)
    head(ax[6], f"7. what the routing sees: {rg.number_of_edges()} transitions",
         f"the overlay, and all the routing keeps: one node per ring, one "
         f"link per transition. The label is the transition width, how many "
         f"routers the pair shares. Red means width 1.")

    # 8. which knob moves what
    ax[7].set_axis_off()
    fin = score(g3, plan3, check=False)
    biggest = max(len(c) for c in cliques)
    ax[7].text(0.0, 1.0,
               "the settings used here\n\n"
               f"--rings {a.rings:<10} {a.rings} rings (panel 1)\n"
               f"--share {a.share:<10g} up to {biggest} rings may meet in one place\n"
               f"--width-max {a.width_max:<6} transitions are {a.width_max} "
               f"routers wide\n"
               f"--spof {a.spof:<11g} {fin['spof']} of the {fin['pairs']} transitions "
               f"squeezed to 1 router\n"
               f"--ring-size {a.ring_size:<6} every ring at least {a.ring_size} routers long\n"
               f"--target-n {a.target_n:<7} keep adding spare routers until "
               f"{a.target_n}\n\n"
               "turn one up and you get\n\n"
               "  more rings        more transitions\n"
               "  more sharing      more transitions\n"
               "  wider transition  more fan-out\n"
               "  more spof         less fan-out\n"
               "  bigger n          longer rings\n\n"
               "only the first two move the transition count.\n\n"
               "the file name keeps the result:\n"
               f"  _A{fin['rings']} _tr{fin['pairs']} _fan{fin['fan']} _n{fin['n']}",
               transform=ax[7].transAxes, va="top", ha="left", fontsize=13,
               family="monospace")

    s = score(g3, plan3, check=False)
    fig.suptitle(f"ringpipe.py --family {a.family} --rings {a.rings} --share "
                 f"{a.share:g} --spof {a.spof:g} --width-max {a.width_max} "
                 f"--ring-size {a.ring_size} --target-n {a.target_n} "
                 f"--seed {a.seed}\n"
                 f"result: {s['n']} routers in {s['rings']} rings, "
                 f"{plural(s['pairs'], 'transition')} between them", fontsize=15)
    fig.tight_layout(rect=(0, 0, 1, 0.94))
    os.makedirs(os.path.dirname(a.out), exist_ok=True)
    fig.savefig(a.out, dpi=200)
    print(f"wrote {a.out}")


if __name__ == "__main__":
    main()
