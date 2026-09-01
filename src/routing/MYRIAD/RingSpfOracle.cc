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
uint64_t ringWeight(uint32_t members, long exp, long echoSize) {
    if (members == 2) members = (uint32_t) echoSize;
    uint64_t term = 1;
    for (long k = 0; k < exp; ++k) {
        uint64_t div = members ? members : 1;
        if (term > INF / div) return INF;
        term *= members;
    }
    return term;
}

using Graph = boost::adjacency_list<boost::vecS, boost::vecS, boost::directedS,
                                    boost::no_property,
                                    boost::property<boost::edge_weight_t, uint64_t>>;

// Ring graph built from one membership view, plus a memo of the single-source
// Dijkstra runs already done on it.
struct RingGraph {
    std::vector<RingId> ringList;
    std::map<RingId, int> idx;
    std::vector<uint64_t> w;      // ring weight = member_count^exp
    Graph g;
    // source ring index -> (cost to every ring, hop count to every ring)
    std::map<int, std::pair<std::vector<uint64_t>, std::vector<uint32_t>>> srcMemo;
};

RingGraph buildRingGraph(const std::map<NodeId, std::vector<RingId>>& perNodeRings, long exp,
                         long echoSize,
                         const std::map<NodeId, std::vector<RingId>>& weightRings) {
    RingGraph rg;

    // member_count per ring = number of nodes listing it. Counted over
    // weightRings (persistent membership) so a failure does not reprice the
    // graph — see the header. The vertex set still comes from perNodeRings:
    // only rings reachable in the live view are routable.
    std::map<RingId, uint32_t> members;
    for (const auto& [node, rs] : weightRings)
        for (RingId r : rs) ++members[r];

    std::set<RingId> liveRings;
    for (const auto& [node, rs] : perNodeRings)
        for (RingId r : rs) liveRings.insert(r);

    // Dense index for the graph vertices.
    for (RingId r : liveRings) {
        rg.idx[r] = (int) rg.ringList.size();
        rg.ringList.push_back(r);
    }
    const int R = (int) rg.ringList.size();

    rg.w.resize(R);
    for (int i = 0; i < R; ++i) {
        auto mit = members.find(rg.ringList[i]);
        // A ring absent from weightRings can only be one born after the weight
        // view was taken; fall back to its live count.
        uint32_t cnt = mit != members.end() ? mit->second : 0;
        if (!cnt)
            for (const auto& [node, rs] : perNodeRings)
                cnt += std::count(rs.begin(), rs.end(), rg.ringList[i]);
        rg.w[i] = ringWeight(cnt, exp, echoSize);
    }

    // Adjacency: rings sharing a member node. Directed pairs so each edge is
    // weighted by its destination ring — node weights are asymmetric, which a
    // single undirected edge weight can't hold.
    std::set<std::pair<int, int>> edges;
    for (const auto& [node, rs] : perNodeRings)
        for (size_t a = 0; a < rs.size(); ++a)
            for (size_t b = a + 1; b < rs.size(); ++b) {
                int ia = rg.idx[rs[a]], ib = rg.idx[rs[b]];
                edges.insert({ia, ib});
                edges.insert({ib, ia});
            }

    rg.g = Graph(R);
    for (const auto& [u, v] : edges) add_edge(u, v, rg.w[v], rg.g);
    return rg;
}

// Single-source Dijkstra from ring index `s`, memoized on the graph.
// cost[o] includes both endpoints: Dijkstra sums edge weights = every ring but
// the source, so w[s] is added back. hops[o] = ring count on that min-cost path,
// walked back through the predecessor map.
const std::pair<std::vector<uint64_t>, std::vector<uint32_t>>&
ringDijkstra(RingGraph& rg, int s) {
    auto mit = rg.srcMemo.find(s);
    if (mit != rg.srcMemo.end()) return mit->second;

    using namespace boost;
    const int R = (int) rg.ringList.size();
    std::vector<uint64_t> d(R, INF);
    std::vector<int> pred(R);
    dijkstra_shortest_paths(rg.g, vertex(s, rg.g),
        distance_map(make_iterator_property_map(d.begin(), get(vertex_index, rg.g)))
        .predecessor_map(make_iterator_property_map(pred.begin(), get(vertex_index, rg.g))));

    std::vector<uint64_t> cost(R);
    std::vector<uint32_t> hops(R);
    for (int o = 0; o < R; ++o) {
        if (d[o] == INF) { cost[o] = INF; hops[o] = 0; continue; }
        cost[o] = rg.w[s] + d[o];
        uint32_t h = 0;                  // ring count from o back to s, inclusive
        for (int v = o; ; v = pred[v]) {
            ++h;
            if (v == s) break;
            if (pred[v] == v) { h = 0; break; }  // defensive: broken chain
        }
        hops[o] = h;
    }
    return rg.srcMemo.emplace(s, std::make_pair(std::move(cost), std::move(hops)))
               .first->second;
}

} // namespace

std::map<RingId, RingRoute>
shortestRingCostsFor(const std::map<NodeId, std::vector<RingId>>& perNodeRings, long exp,
                     long echoSize, NodeId self,
                     const std::map<NodeId, std::vector<RingId>>* weightRings) {
    auto sit = perNodeRings.find(self);
    if (sit == perNodeRings.end()) return {};
    const auto& wr = weightRings ? *weightRings : perNodeRings;

    // Only `self`'s row is ever used, so Dijkstra runs from self's member rings
    // (its possible graph entry points) rather than from all R rings. Costs
    // O(|selfRings|) passes per call instead of O(R) — the difference between a
    // usable 1000-node dynamics snapshot and an unusable one.
    //
    // The graph and its per-source results are memoized on the membership view:
    // in a converged run every node passes an identical view, so all nodes share
    // one build and each ring is sourced at most once per snapshot. During churn
    // views differ per node, the memo misses, and correctness is unaffected.
    static long lastExp = 0, lastEcho = 0;
    static std::map<NodeId, std::vector<RingId>> lastRings, lastWeight;
    static RingGraph rg;
    static bool haveLast = false;
    if (!haveLast || lastExp != exp || lastEcho != echoSize || lastRings != perNodeRings
        || lastWeight != wr) {
        rg = buildRingGraph(perNodeRings, exp, echoSize, wr);
        lastExp = exp; lastEcho = echoSize; lastRings = perNodeRings; lastWeight = wr;
        haveLast = true;
    }

    const int R = (int) rg.ringList.size();
    // Best over self's member rings; ties broken by fewer hops, matching
    // MyriadGP's cost-then-length rank.
    std::vector<RingRoute> best(R, RingRoute{INF, 0});
    for (RingId mr : sit->second) {
        auto iit = rg.idx.find(mr);
        if (iit == rg.idx.end()) continue;
        const auto& [cost, hops] = ringDijkstra(rg, iit->second);
        for (int o = 0; o < R; ++o) {
            if (cost[o] < best[o].cost ||
                (cost[o] == best[o].cost && hops[o] < best[o].hops))
                best[o] = {cost[o], hops[o]};
        }
    }

    std::map<RingId, RingRoute> row;
    for (int o = 0; o < R; ++o)
        if (best[o].cost != INF) row[rg.ringList[o]] = best[o];
    return row;
}

void dumpOracle(const std::string& path, NodeId self,
                const std::map<NodeId, std::vector<RingId>>& perNodeRings, long exp,
                long echoSize, const std::string& tprefix, bool stats, long presentCount,
                const std::map<NodeId, std::vector<RingId>>* weightRings) {
    auto row = shortestRingCostsFor(perNodeRings, exp, echoSize, self, weightRings);
    if (row.empty()) return;

    std::ofstream f(path, std::ios::app);
    if (stats) {
        uint64_t n = 0, sum = 0, mx = 0;
        for (const auto& [origin, route] : row) {
            ++n; sum += route.cost; if (route.cost > mx) mx = route.cost;
        }
        // n/cost over the live topology feeds the stretch axis; present is the
        // route-refill count on the unfiltered (persistent-membership) basis,
        // comparable to a path-vector RIB that holds stale entries until purged.
        f << tprefix << self << " n=" << n
          << " cost_sum=" << sum << " cost_max=" << mx;
        if (presentCount >= 0) f << " present=" << presentCount;
        f << "\n";
        return;
    }
    for (const auto& [origin, route] : row)
        f << tprefix << self << " origin=" << origin
          << " cost=" << route.cost << " hops=" << route.hops << "\n";
}

} // namespace ringspf
} // namespace routingsim
