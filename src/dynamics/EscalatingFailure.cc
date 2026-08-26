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

	std::map<nodeid_t, unsigned long> candidates;
	getTopology().getNodeDegrees(candidates);
	for (auto it = candidates.begin(); it != candidates.end();) {
		if (it->second <= minDegree || it->second >= maxDegree) candidates.erase(it++);
		else ++it;
	}

	size_t want = (double) getTopology().getActiveNumberNodes() * waves[idx];
	EV << "wave " << idx << ": failing " << std::min(want, candidates.size())
	   << " nodes (" << waves[idx] * 100 << "%)" << endl;

	for (size_t i = 0; i < std::min(want, candidates.size()); i++) {
		auto it = candidates.begin();
		std::advance(it, getRNG(0)->intRand(candidates.size()));
		nodeid_t node = it->first;
		candidates.erase(it);

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
