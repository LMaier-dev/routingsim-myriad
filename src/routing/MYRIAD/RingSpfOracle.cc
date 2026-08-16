#include "routing/MYRIAD/RingSpfOracle.h"

#include <algorithm>
#include <fstream>
#include <limits>
#include <set>
#include <utility>

#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/dijkstra_shortest_paths.hpp>

namespace routingsim {
namespace ringspf {

namespace {

constexpr uint64_t INF = std::numeric_limits<uint64_t>::max();

// member_count^exp, saturating on overflow. Byte-for-byte the per-ring term of
// MyriadGP::pathCost, so ties/costs compare exactly against GP's uint64 cost.
uint64_t ringWeight(uint32_t members, long exp) {
    uint64_t term = 1;
    for (long k = 0; k < exp; ++k) {
        uint64_t div = members ? members : 1;
        if (term > INF / div) return INF;
        term *= members;
    }
    return term;
}

} // namespace

std::map<NodeId, std::map<RingId, RingRoute>>
shortestRingCosts(const std::map<NodeId, std::vector<RingId>>& perNodeRings, long exp) {
    // Last-value memo: within one snapshot all nodes call this consecutively with
    // the same (converged) membership, so caching the previous input/output
    // collapses ~N identical all-pairs Dijkstra passes to one. A mid-convergence
    // input that differs just misses and recomputes — always correct.
    static long lastExp = 0;
    static std::map<NodeId, std::vector<RingId>> lastRings;
    static std::map<NodeId, std::map<RingId, RingRoute>> lastOut;
    static bool haveLast = false;
    if (haveLast && lastExp == exp && lastRings == perNodeRings)
        return lastOut;

    // member_count per ring = number of nodes listing it.
    std::map<RingId, uint32_t> members;
    for (const auto& [node, rs] : perNodeRings)
        for (RingId r : rs) ++members[r];

    // Dense index for the graph vertices.
    std::vector<RingId> ringList;
    std::map<RingId, int> idx;
    for (const auto& [r, cnt] : members) { idx[r] = (int)ringList.size(); ringList.push_back(r); }
    const int R = (int)ringList.size();

    std::vector<uint64_t> w(R);
    for (int i = 0; i < R; ++i) w[i] = ringWeight(members[ringList[i]], exp);

    // Adjacency: rings sharing a member node. Directed pairs so each edge is
    // weighted by its destination ring — node weights are asymmetric, which a
    // single undirected edge weight can't hold.
    std::set<std::pair<int, int>> edges;
    for (const auto& [node, rs] : perNodeRings)
        for (size_t a = 0; a < rs.size(); ++a)
            for (size_t b = a + 1; b < rs.size(); ++b) {
                int ia = idx[rs[a]], ib = idx[rs[b]];
                edges.insert({ia, ib});
                edges.insert({ib, ia});
            }

    using namespace boost;
    using Graph = adjacency_list<vecS, vecS, directedS, no_property,
                                 property<edge_weight_t, uint64_t>>;
    Graph g(R);
    for (const auto& [u, v] : edges) add_edge(u, v, w[v], g);

    // ringDist[s][o] = optimal cost s->o (both endpoints included; Dijkstra sums
    // edge weights = every ring but the source, so add w[s] back). ringHops[s][o]
    // = ring count on that min-cost path, walked back through the predecessor map.
    std::vector<std::vector<uint64_t>> ringDist(R);
    std::vector<std::vector<uint32_t>> ringHops(R);
    for (int s = 0; s < R; ++s) {
        std::vector<uint64_t> d(R, INF);
        std::vector<int> pred(R);
        dijkstra_shortest_paths(g, vertex(s, g),
            distance_map(make_iterator_property_map(d.begin(), get(vertex_index, g)))
            .predecessor_map(make_iterator_property_map(pred.begin(), get(vertex_index, g))));
        ringDist[s].resize(R);
        ringHops[s].resize(R);
        for (int o = 0; o < R; ++o) {
            if (d[o] == INF) { ringDist[s][o] = INF; ringHops[s][o] = 0; continue; }
            ringDist[s][o] = w[s] + d[o];
            uint32_t h = 0;                  // ring count from o back to s, inclusive
            for (int v = o; ; v = pred[v]) {
                ++h;
                if (v == s) break;
                if (pred[v] == v) { h = 0; break; }  // defensive: broken chain
            }
            ringHops[s][o] = h;
        }
    }

    // Per node: best over its member rings (its possible graph entry points).
    // Ties broken by fewer hops, matching MyriadGP's cost-then-length rank.
    std::map<NodeId, std::map<RingId, RingRoute>> out;
    for (const auto& [node, rs] : perNodeRings) {
        auto& row = out[node];
        for (int o = 0; o < R; ++o) {
            RingRoute best{INF, 0};
            for (RingId mr : rs) {
                uint64_t c = ringDist[idx[mr]][o];
                uint32_t h = ringHops[idx[mr]][o];
                if (c < best.cost || (c == best.cost && h < best.hops))
                    best = {c, h};
            }
            if (best.cost != INF) row[ringList[o]] = best;
        }
    }

    lastExp = exp; lastRings = perNodeRings; lastOut = out; haveLast = true;
    return out;
}

void dumpOracle(const std::string& path, NodeId self,
                const std::map<NodeId, std::vector<RingId>>& perNodeRings, long exp,
                const std::string& tprefix, bool stats) {
    auto all = shortestRingCosts(perNodeRings, exp);
    auto it = all.find(self);
    if (it == all.end()) return;

    std::ofstream f(path, std::ios::app);
    if (stats) {
        uint64_t n = 0, sum = 0, mx = 0;
        for (const auto& [origin, route] : it->second) {
            ++n; sum += route.cost; if (route.cost > mx) mx = route.cost;
        }
        f << tprefix << self << " n=" << n
          << " cost_sum=" << sum << " cost_max=" << mx << "\n";
        return;
    }
    for (const auto& [origin, route] : it->second)
        f << tprefix << self << " origin=" << origin
          << " cost=" << route.cost << " hops=" << route.hops << "\n";
}

} // namespace ringspf
} // namespace routingsim
