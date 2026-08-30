#include "NodeChurn.h"

namespace routingsim {

Define_Module(NodeChurn);

simtime_t NodeChurn::scheduleDynamics() {
	double period = par("period").doubleValue();
	double downMin = par("downMin").doubleValue();
	double downJitter = par("downJitter").doubleValue();
	size_t want = par("numNodes");
	size_t minDegree = par("minDegree");
	size_t maxDegree = par("maxDegree");
	if (maxDegree == 0) maxDegree = std::numeric_limits<int>::max();

	// Same split as EscalatingFailure: eligibility is live (an already-down node
	// can't fail again) but the degree filter reads the structural topology, so a
	// healthy node isn't excluded just because a neighbour happens to be down.
	std::vector<nodeid_t> activeIds;
	getTopology().getNodeIDs(activeIds);
	std::vector<nodeid_t> candidates;
	for (nodeid_t id : activeIds) {
		size_t deg = getTopology().getStructuralDegree(id);
		if (deg > minDegree && deg < maxDegree) candidates.push_back(id);
	}

	simtime_t delay = first ? par("warmup").doubleValue() : 0;
	first = false;

	size_t n = std::min(want, candidates.size());
	for (size_t i = 0; i < n; i++) {
		size_t pick = getRNG(0)->intRand(candidates.size());
		nodeid_t node = candidates[pick];
		candidates[pick] = candidates.back();
		candidates.pop_back();

		double down = downMin + getRNG(0)->doubleRand() * downJitter;
		scheduleNodeFailure(node, delay, down);
		// Recovery is measured from the reboot; a node churns repeatedly, so the
		// scalar is keyed by reboot time rather than by a round index.
		simtime_t back = simTime() + delay + down;
		recordScalar(("up_" + std::to_string(node) + "_" + std::to_string(back.raw())).c_str(),
		             back.dbl());
	}

	return simTime() + delay + period;
}

} //namespace
