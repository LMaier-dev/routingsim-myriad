//
// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU Lesser General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
// 
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU Lesser General Public License for more details.
// 
// You should have received a copy of the GNU Lesser General Public License
// along with this program.  If not, see http://www.gnu.org/licenses/.
// 

#include "VRObserver.h"
#include "NetworkManager.h"
#include "MessageStatistics.h"

namespace routingsim {

Define_Module(VRObserver);

using namespace boost;
using namespace std;

void VRObserver::initialize() {
	WATCH_VECTOR(entries);
	general = new MessageStatistics("general");
	anycast = new MessageStatistics("anycast");
	timer = new cMessage();
	scheduleAt(simTime() + 1.0, timer);
}

void VRObserver::finish() {
	general->finish();
	anycast->finish();
	entries.clear();
}

void VRObserver::handleMessage(cMessage *msg) {
	if ( msg != timer ) return;
	general->sample();
	anycast->sample();
	scheduleAt(simTime() + 1.0, timer);
}

void VRObserver::getNeighbors(unordered_set<vid_t>& set, vid_t id, size_t k) {
	if (entries.size() == 0)
		return;
	for (size_t i = 0; i < entries.size(); i++) {
		if (entries[i] != id)
			continue;
		set.insert(entries[i]);
		if (i>0) set.insert(entries[i-1]);
		if ((i+1)<(entries.size())) set.insert(entries[i+1]);
	/*
		for (size_t n = 1; n <= k; n++) {
			set.insert(entries[(n <= i) ? (i - n) : (entries.size() + i - n)]);
			set.insert(entries[(i + n) % entries.size()]);
		}
	*/
		break;
	}
}

void VRObserver::up(vid_t id, nodeid_t owner) {

	// anycast address?
	if (owner != undefined_node) {

		if (anycast_map.count(id) == 0)	anycast_entries.push_back(id);
		anycast_map.insert(make_pair(id, owner));

	// unicast address
	} else {
		// find entry whose id is greater or equal
		vector<vid_t>::iterator i = entries.begin();
		while (i != entries.end() && *i < id) i++;
		if (i!=entries.end() && *i == id) return;

		// not found, insert new entry
		if (i == entries.end())
			entries.push_back(id);
		else
			entries.insert(i, id);
	}
}

void VRObserver::down(vid_t id, nodeid_t owner) {

	// anycast
	if (owner != undefined_node) {
		typedef pair<anycastmap_t::iterator, anycastmap_t::iterator> range_pair;
		range_pair r = anycast_map.equal_range(id);

		for (anycastmap_t::iterator j = r.first; j != r.second; j++) {
			if (j->second != owner) continue;

			// erase from map
			anycast_map.erase(j);

			// clear from vector
			if (anycast_map.count(id) == 0) {
				for (size_t k = 0; k < anycast_entries.size(); k++) {
					if (anycast_entries[k] != id) continue;
					anycast_entries.erase(anycast_entries.begin() + k);
					break;
				}
			}
			return;
		}
	}

	// unicast
	else {
		for (size_t i = 0; i < entries.size(); i++)
			if (entries[i] == id) {
				entries.erase(entries.begin() + i);
				return;
			}
	}
}

vid_t VRObserver::random() const {
	return entries[intuniform(0, entries.size() - 1)];
}

vid_t VRObserver::random_anycast() const {
	if (anycast_entries.size()==0) return undefined_vid;
	return anycast_entries[intuniform(0, anycast_entries.size() - 1)];
}

static VRObserver* observer = NULL;

VRObserver& getVRObserver() {
	if (observer == NULL) {
		cModuleType* type = cModuleType::get(
				"routingsim.routing.evrr.VRObserver");
		cModule* mod = simulation.getContextModule();
		while (mod->getParentModule() != NULL)
			mod = mod->getParentModule();
		observer = dynamic_cast<VRObserver*>(type->create("vrobserver", mod));
	}
	return *observer;
}

// delivery ratio and stretch
void VRObserver::messageRegister(cMessage* msg, bool anycast_msg) {
	cModule* srcNode = simulation.getContextModule()->getParentModule();
	msg->setContextPointer(srcNode);
	if (anycast_msg) anycast->messageRegister(msg);
	else general->messageRegister(msg);
}

void VRObserver::messageDrop(cMessage* msg) {
	general->messageDrop(msg);
	anycast->messageDrop(msg);
}

void VRObserver::messageDelivered(cMessage* msg, vid_t dest, distance_t routingDist) {
	typedef pair<anycastmap_t::iterator, anycastmap_t::iterator> range_pair;

	// ignore unregistered messages
	if (!general->messageRegistered(msg) && !anycast->messageRegistered(msg))
		return;

	// get topology
	NetworkTopology& topology = getNetworkManager()->getTopology();

	// get source / destination node
	cModule* srcNode 	= static_cast<cModule*>(msg->getContextPointer());
	nodeid_t srcNodeID 	= check_and_cast<Node *>(srcNode)->getNodeId();
	cModule* dstNode 	= simulation.getContextModule()->getParentModule();
	nodeid_t dstNodeID = check_and_cast<Node *>(dstNode)->getNodeId();
	// calculate stretch
	distance_t realDistance = 0;
	if (getNetworkManager() != NULL) {
		// get real distance
		realDistance = (distance_t) topology.getDistance(srcNodeID, dstNodeID);

		// check any-cast distance
		if (anycast_map.count(dest) != 0) {
			range_pair rp = anycast_map.equal_range(dest);
			for (anycastmap_t::iterator i = rp.first; i != rp.second; i++) {
				distance_t d = (distance_t) topology.getDistance(srcNodeID,
						i->second);
				if (d < realDistance)
					realDistance = d;
			}
		}
	}

	// save
	anycast->messageDelivered(msg, routingDist, realDistance);
	general->messageDelivered(msg, routingDist, realDistance);
}

} //namespace
