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


#include <omnetpp.h>
#include "DiscoObserver.h"
#include "NetworkManager.h"


namespace routingsim {
static DiscoObserver *observer = NULL;

Define_Module(DiscoObserver);

DiscoObserver::DiscoObserver() {
	networkMgr = getNetworkManager();
	NumofLandmarkAnnouncements=0;
}

DiscoObserver::~DiscoObserver() {
	observer=NULL;
	entries.clear();
}

void DiscoObserver::initialize()
{
	WATCH_VECTOR(entries);
	WATCH(NumofLandmarkAnnouncements);
	map<nodeid_t, unsigned long> retDegreeMap;
	getNetworkManager()->getTopology().getNodeDegrees(retDegreeMap);
}


void DiscoObserver::finish()
{
	// write scalars
	recordLinkUsage();
	recordStatistics();
}



void DiscoObserver::handleMessage(cMessage *msg)
{
	//  currently not used
	delete msg;
}


void DiscoObserver::registerNode(Disco *DiscoModule) {
	/// if element is in list, do not add it again
	for (unsigned int i=0; i<entries.size(); i++) {
		if (entries[i] == DiscoModule->getId()) {
			return;
		}
	}
	entries.push_back(DiscoModule->getId());
}

void DiscoObserver::unregisterNode(Disco *DiscoModule) {
	for (unsigned int i=0; i<entries.size(); i++) {
		if (entries[i] == DiscoModule->getId()) {
			entries.erase(entries.begin()+i);
			return;
		}
	}
}


discoid_t &DiscoObserver::getRandomNodeAddress() const {
	Disco *selectedNode = dynamic_cast<Disco *>(getSimulation()->getModule(entries[intuniform(0,entries.size()-1)]));
	return selectedNode->myAddr;
}


// returns a randomly chosen set of nodes of size n
std::set<int> DiscoObserver::sampleNodes(int n) {
	if (n < 0 or (size_t) n > entries.size())
		n = entries.size();
	std::set<int> sample{};
	// use all nodes if n larger than 80% of all nodes (then sampling is not useful anymore)
	if (n > entries.size() * 0.8) {
		for (auto entry : entries)
			sample.insert(entry);
	}
	while (sample.size() < (size_t) n) {
		int nid = entries[intuniform(0,entries.size()-1)];
		sample.insert(nid);
	}
	while (sample.size() > (size_t) n)
		sample.erase(entries[intuniform(0,entries.size()-1)]);

	return sample;
}



discoid_t& DiscoObserver::getNodeAddress(nodeid_t id) const {
	for (unsigned int x=0; x<entries.size(); x++)
	{
		Disco *selectedNode = dynamic_cast<Disco *>(getSimulation()->getModule(entries[x]));
		if (selectedNode->node().getNodeId() == id) {
			return selectedNode->myAddr;
		}
	}
	EV << " Node ID not known, BUG, EXITING" << endl;
	exit(1);
}


float DiscoObserver::getDistance(discoid_t source, discoid_t destination) const {
	Node *sourceNode=NULL, *destinationNode=NULL;

	for(unsigned int i=0; i<entries.size(); i++) {
		Disco *selectedNode = dynamic_cast<Disco *>(getSimulation()->getModule(entries[i]));
		if (selectedNode->myAddr == source) {
			sourceNode = check_and_cast<Node *>(selectedNode->getParentModule());
		} else if (selectedNode->myAddr == destination) {
			destinationNode = check_and_cast<Node *>(selectedNode->getParentModule());
		}
	}

	if ( sourceNode== NULL || destinationNode == NULL) {
		EV_WARN << "Either source or destination node could not be mapped to a Node" << endl;
	}

	return networkMgr->getDistance(sourceNode->getNodeId(), destinationNode->getNodeId());
}

int DiscoObserver::getNetworkSize() const {
	return getNetworkManager()->getNetworkSize();
}

int DiscoObserver::getNumLandmarks() const {
	int sum=0;
	for (unsigned int i=0; i<entries.size(); i++) {
		Disco *currentNode = dynamic_cast<Disco *>(getSimulation()->getModule(entries[i]));
		if (currentNode->isLandmark)
			sum+=1;
	}
	return sum;
}

DiscoObserver *getDiscoObserver() {
	if (observer == NULL) {
		cModuleType* type = cModuleType::get("routingsim.routing.disco.DiscoObserver");
		cModule* mod = getSimulation()->getContextModule();
		while(mod->getParentModule()!=NULL) mod = mod->getParentModule();
		observer = dynamic_cast<DiscoObserver*>(type->create( "DiscoObserver",mod ));
		if (observer == NULL)
			EV_ERROR << "Initialization of DiscoObserver went wrong!" << endl;
	}

	return observer;
}


Disco* DiscoObserver::getDisco(int id) const
{
	Disco *currentNode = dynamic_cast<Disco *>(getSimulation()->getModule(id));
	ASSERT(currentNode != nullptr);
	return currentNode;
}


void DiscoObserver::recordLinkUsage() const {
	cHistogram linkUsageHist("linkUsage");
	//hist->setRange(rangeLower, rangeUpper);
	//hist->setNumBinsHint(numBins);
	linkUsageHist.setMode(cHistogram::MODE_INTEGERS);
	NetworkTopology& topo = getNetworkManager()->getTopology();

	for (nodeid_t nid=0; nid < entries.size(); nid++) {
		if (const Node *np= topo.getNodeModule(nid); np != nullptr) {
			// traverse all outgoing links to neighbors
			unsigned long long sum= 0;
			for (auto& neighbor : np->getLinkUsageMap()) {
				linkUsageHist.collect(neighbor.second);
				sum += neighbor.second;
			}
			//EV_TRACE << "Node " <<  nid << " linkusage sum:" << sum << endl;
		}
	}
	linkUsageHist.record();
}


void DiscoObserver::recordStatistics() {
	// record statistics for a sample set of nodes
	for (auto nid : sampleNodes(par("nodeSample").intValue())) {
		getDisco(nid)->recordScalars();
	}

	recordScalar("simTime", simTime());
}


int DiscoObserver::getNumofLandmarkAnnouncements() const
{
	return NumofLandmarkAnnouncements;
}

void DiscoObserver::setNumofLandmarkAnnouncements()
{
	NumofLandmarkAnnouncements++;
}
void DiscoObserver::setNumofLandmarkAnnouncements(int n)
{
	NumofLandmarkAnnouncements+=n;
}

} //namespace
