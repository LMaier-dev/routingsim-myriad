#ifndef __ROUTINGSIM_RINGSPFORACLE_H_
#define __ROUTINGSIM_RINGSPFORACLE_H_

// Independent optimality oracle for MyriadGP: pure STL + Boost, no protocol deps.
// Given every node's ring membership (a converged MyriadLS reads it from its LSDB
// RingsLsa), builds the ring graph and runs node-weighted Dijkstra for the optimal
// cost from each node to every origin ring. GP's converged cost must match.
//
// Ring graph: rings adjacent iff they share a node; ring weight = member_count^exp;
// path cost sums every ring on it, both endpoints — same as MyriadGP::pathCost.

#include <cstdint>
#include <map>
#include <string>
#include <vector>

namespace routingsim {
namespace ringspf {

using NodeId = uint32_t;
using RingId = uint32_t;

// Optimal route to an origin ring
struct RingRoute {
    uint64_t cost;
    uint32_t hops;
};

// perNodeRings: node id -> rings it belongs to
std::map<NodeId, std::map<RingId, RingRoute>>
shortestRingCosts(const std::map<NodeId, std::vector<RingId>>& perNodeRings, long exp);

void dumpOracle(const std::string& path, NodeId self,
                const std::map<NodeId, std::vector<RingId>>& perNodeRings, long exp,
                const std::string& tprefix = "", bool stats = false);

} // namespace ringspf
} // namespace routingsim

#endif
