#include "EscalatingFailure.h"

namespace routingsim {

Define_Module(EscalatingFailure);

simtime_t EscalatingFailure::scheduleDynamics() {
	if (waves.empty()) {
		omnetpp::cStringTokenizer tok(par("fractions").stringValue(), ",");
		while (tok.hasMoreTokens()) waves.push_back(atof(tok.nextToken()));
	}
	if (idx >= waves.size()) return SIMTIME_MAX;

	double warmup = par("warmup").doubleValue();
	double spacing = par("waveSpacing").doubleValue();
	double downMin = par("downMin").doubleValue();
	double downJitter = par("downJitter").doubleValue();
	size_t minDegree = par("minDegree");
	size_t maxDegree = par("maxDegree");
	if (maxDegree == 0) maxDegree = std::numeric_limits<int>::max();

	// We are called at t=0 for the first wave and at the wave time afterwards.
	simtime_t delay = (idx == 0) ? warmup : 0;

	// Eligibility is live, since an already-down node cannot fail again, but the
	// degree filter reads the structural topology, so a healthy node is not
	// excluded because a neighbour happens to be down.
	std::vector<nodeid_t> activeIds;
	getTopology().getNodeIDs(activeIds);
	std::vector<nodeid_t> candidates;
	for (nodeid_t id : activeIds) {
		size_t deg = getTopology().getStructuralDegree(id);
		if (deg > minDegree && deg < maxDegree) candidates.push_back(id);
	}

	size_t want = (double) getTopology().getActiveNumberNodes() * waves[idx];
	// Frozen before the loop, which shrinks candidates on every pick: a bound
	// recomputed from it would lower the target as the wave proceeds.
	size_t n = std::min(want, candidates.size());
	EV << "wave " << idx << ": failing " << n
	   << " nodes (" << waves[idx] * 100 << "%)" << endl;

	for (size_t i = 0; i < n; i++) {
		size_t pick = getRNG(0)->intRand(candidates.size());
		nodeid_t node = candidates[pick];
		candidates[pick] = candidates.back();
		candidates.pop_back();

		double down = downMin + getRNG(0)->doubleRand() * downJitter;
		scheduleNodeFailure(node, delay, down);
		// Recovery is measured from the reboot, which differs per node.
		recordScalar(("down_" + std::to_string(node) + "_" + std::to_string(idx)).c_str(),
		             (simTime() + delay + down).dbl());
	}

	if (++idx >= waves.size()) return SIMTIME_MAX;
	return warmup + idx * spacing;
}

} //namespace
