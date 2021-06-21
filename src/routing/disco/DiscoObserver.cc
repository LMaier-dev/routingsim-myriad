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
	NumofLandmarkAnnoucements=0;
}

DiscoObserver::~DiscoObserver() {
	observer=NULL;
	entries.clear();
}

void DiscoObserver::initialize()
{
	WATCH_VECTOR(entries);
	WATCH(NumofLandmarkAnnoucements);
	map<nodeid_t, unsigned long> retDegreeMap;
	getNetworkManager()->getTopology().getNodeDegrees(retDegreeMap);
}

void DiscoObserver::handleMessage(cMessage *msg)
{
	// TODO - Generated method body
}


void DiscoObserver::registerNode(Disco *DiscoModule) {
	/// if element is in list, do not add it again
	for(unsigned int i=0; i<entries.size(); i++) {
		if (entries[i] == DiscoModule->getId()) {
			return;
		}
	}
	entries.push_back(DiscoModule->getId());
}

void DiscoObserver::unregisterNode(Disco *DiscoModule) {
	for(unsigned int i=0; i<entries.size(); i++) {
		if (entries[i] == DiscoModule->getId()) {
			entries.erase(entries.begin()+i);
			return;
		}
	}
}

discoid_t &DiscoObserver::getRandomNodeAddress() {
	Disco *selectedNode = dynamic_cast<Disco *>(getSimulation()->getModule(entries[intuniform(0,entries.size()-1)]));
	return selectedNode->myAddr;
}

discoid_t &DiscoObserver::getNodeAddress(nodeid_t id ) {
	for(unsigned int x=0; x<entries.size(); x++)
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
	for(unsigned int i=0; i<entries.size(); i++) {
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



	//void DiscoObserver::sampleLaterPath(const OverlayKey& originKey, const kad_path& path) {}

int DiscoObserver::getNumofLandmarkAnnouncements() const
{
	return NumofLandmarkAnnoucements;
}

void DiscoObserver::setNumofLandmarkAnnouncements()
{
	NumofLandmarkAnnoucements++;
}
void DiscoObserver::setNumofLandmarkAnnouncements(int n)
{
	NumofLandmarkAnnoucements+=n;
}

} //namespace
