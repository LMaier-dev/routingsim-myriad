#ifndef __ROUTINGSIM_RINGSPFORACLE_H_
#define __ROUTINGSIM_RINGSPFORACLE_H_

// Independent optimality oracle for MyriadPV: pure STL + Boost, no protocol deps.
// Given every node's ring membership (a converged MyriadLS reads it from its LSDB
// RingsLsa), builds the ring graph and runs node-weighted Dijkstra for the optimal
// cost from each node to every origin ring. PV's converged cost must match.
//
// Ring graph: rings adjacent iff they share a node; ring weight = member_count^exp
// (a count of 2 priced as echoSize, matching MyriadPvRib::sizeToCost);
// path cost sums every ring on it, both endpoints — same as MyriadPV::pathCost.
//
// Member counts come from `weightRings` (persistent membership), NOT from the
// live routing view: PV prices a chain by the ring sizes it learned and does not
// reprice when a member dies, so an oracle that shrank its weights on every
// failure would compare two different cost functions. Under churn that read as
// stretch swinging above and below 1 with no route having changed.

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

// Optimal cost/hops from `self` to every reachable origin ring.
// perNodeRings: node id -> rings it belongs to; defines the graph (which rings
//   exist and which are adjacent), so pass the LIVE view.
// weightRings: membership the ring weights are counted over. Pass the full
//   (unpruned) view so weights stay fixed as nodes fail; nullptr = weigh over
//   perNodeRings, the converged case where the two views coincide.
std::map<RingId, RingRoute>
shortestRingCostsFor(const std::map<NodeId, std::vector<RingId>>& perNodeRings, long exp,
                     long echoSize, NodeId self,
                     const std::map<NodeId, std::vector<RingId>>* weightRings = nullptr);

void dumpOracle(const std::string& path, NodeId self,
                const std::map<NodeId, std::vector<RingId>>& perNodeRings, long exp,
                long echoSize, const std::string& tprefix = "", bool stats = false,
                long presentCount = -1,
                const std::map<NodeId, std::vector<RingId>>* weightRings = nullptr);

} // namespace ringspf
} // namespace routingsim

#endif
