#include "Disco.h"


using namespace std;
using namespace boost;


/// Packet Types
const int landmarkAnnouncement = 100;
const int vicinityAnnouncement = 200;
const int dataPacket = 300;
const int BetterPathPkt = 350;

/// Timings
const int landmarkAnnouncementInterval = 10;
const int vicinityAnnouncementInterval = 15;
const int cooldownTime = 300;
const int sendDataPacketsStart = 60;

namespace routingsim {

Define_Module(Disco);


void Disco::sendLandmarkAnnouncement() {
	// Send Landmark Announcement to all neighbors
	NodePkt *np = new NodePkt();
	np->setType(landmarkAnnouncement);
	np->setName("Landmark Announcement");
	DiscoPkt *dp = new DiscoPkt();
	dp->setPathVectorArraySize(1);
	dp->setPathVector(0,localNodeId);
	np->encapsulate(dp);
	sendToNeighbors(np);

	// Statistics
	observer->setNumofLandmarkAnnouncements(node().getNeighbors().size());
	LandmarkAnnouncementsOriginated+=node().getNeighbors().size();
}

void Disco::sendVicinityAnnouncement() {
	// Send Vicinity Announcement to all neighbors
	NodePkt *np = new NodePkt();
	np->setType(vicinityAnnouncement);
	np->setName("Vicinity Announcement");
	DiscoPkt *dp = new DiscoPkt();
	dp->setPathVectorArraySize(1);
	dp->setPathVector(0,localNodeId);
	np->encapsulate(dp);
	sendToNeighbors(np);

	// Statistics
	VicinityAnnouncementsOriginated+=node().getNeighbors().size();
}


void Disco::onSelfMessage(cMessage* msg) {
	// simple timer realization
	NodePkt* np = static_cast< NodePkt* >(msg);
	if (np == nullptr) {
	       	EV_ERROR << "wrong type of self message" << endl;
		return;
	}

	if (np->isSelfMessage()) {
		// periodic landmark annoucnements
		if (np == landmarkAnnouncementTimer) {
			sendLandmarkAnnouncement();
			// sendVicinityAnnouncement();

			scheduleAt(simTime()+landmarkAnnouncementInterval, landmarkAnnouncementTimer);
		}

		// periodic vicinity announcements
		if (np == vicinityAnnouncementTimer) {
			sendVicinityAnnouncement();

			scheduleAt(simTime()+vicinityAnnouncementInterval, vicinityAnnouncementTimer);
		}

		if (np == cooldownTimer) {
			printLandmarks();
			printVicinity();
		}

		// perform initial landmark selection
		if (np == initialTimer) {
			EV_TRACE << "selfmessage: initialTimer" << endl;
			networkSize = observer->getNetworkSize();

			// if constructing from a topology and parameter
			// extractLandmarksFromTopologyFile is set, use settings from file
			if (par("extractLandmarksFromTopologyFile").boolValue()) {
				EV_INFO << "extracting Landmarks from file" << endl;
				if (par("landmark").boolValue()) {
					EV << "Setting node " << localNodeId.id << "as landmark from topology file annotations" << endl;
					isLandmark = true;
				}
				else {
					isLandmark = false;
				}
			} else {
				// default: randomize landmarks
				becomeLandmark();
			}

			if (isLandmark) {
				// landmarks are red
				node().setDisplayString("i=misc/node,red,20");
				generateAddr();

				// fire messages in 10 seconds
				landmarkAnnouncementTimer = new NodePkt();
				landmarkAnnouncementTimer->setName("landmarkAnnouncementTimer");
				scheduleAt(simTime()+uniform(0.5*landmarkAnnouncementInterval,1.5*landmarkAnnouncementInterval),
					   landmarkAnnouncementTimer);
			}

			vicinityAnnouncementTimer = new NodePkt();
			vicinityAnnouncementTimer->setName("vicinityAnnouncementTimer");
			scheduleAt(simTime()+uniform(0.5*vicinityAnnouncementInterval, 1.5*vicinityAnnouncementInterval),
				   vicinityAnnouncementTimer);
			//			}

			cooldownTimer = new NodePkt();
			cooldownTimer->setName("cooldownTimer");
			scheduleAt(simTime()+cooldownTime, cooldownTimer);

			// pick random nodes sending packets
			double p = uniform(0,1);
			if (p < probingNodeFraction) {
				sendDataPacketsTimer = new NodePkt();
				sendDataPacketsTimer->setName("sendDataPacketsTimer");
				scheduleAt(simTime()+sendDataPacketsStart+uniform(0.5*probeInterval,1.5*probeInterval), sendDataPacketsTimer);
			}

			cancelAndDelete(initialTimer);
			return;
		}

		if (np == sendDataPacketsTimer) {
			// send packets to random destinations
			//for (int i=0; i<(observer->getNetworkSize()*probePacketDestinationFraction); i++) {
			if (observer->getNetworkSize() > 1) { // cannot send packet to myself
				discoid_t addr = observer->getRandomNodeAddress();
				while (addr == myAddr) {
					addr = observer->getRandomNodeAddress();
				} // since network size > 1 will not loop forever
				sendPktToNode(addr);
			}
			scheduleAt(simTime()+uniform(0.5*probeInterval,1.5*probeInterval), sendDataPacketsTimer);
		}
		return;
	}
	else
		EV_ERROR << "no self message in OnSelfMessage()" << endl;
}
	
void Disco::onMessage(NodePkt *np) {

	switch (np->getType()) {
		case landmarkAnnouncement:
			handleLandmarkAnnouncement(np);
			break;

		case vicinityAnnouncement:
			handleVicinityPacket(np);
			break;

		case dataPacket:
			// we generate the packet and enqueue it
			// packet is deleted only if we ourselves are the final destination
			handleDataPacket(np);
			return;

		case BetterPathPkt:
			// we generate the packet and enqueue it, must not delete
			handleBetterPathPkt(np);
			return;

		default:
			EV_ERROR << "Unknown packet of type: " << np->getType() << endl;
			EV_ERROR << "arrived on gate " << np->getArrivalGate()->getFullName();
	}
	delete(np);
}


void Disco::printLandmarks() {
	EV << " Path to all Landmarks known to Node: " << localNodeId.origin << endl;
	for (const auto &l : landmarkSet)
	{
		EV << "Path to " << l.second.landmarkID.origin << " -> ";
		for (unsigned int y = 0; y < l.second.PathVector.size(); y++)
		{
			EV<< l.second.PathVector[y].origin << ";";
		}
		EV<< endl;
	}
	EV << endl;
}


void Disco::printVicinity() {
	EV << " Path to all Nodes in the Vicinity of Node: " << localNodeId.origin << endl;
	for(const auto &v : vicinitySet) {
		EV << "Path to " << v.second.landmarkID.origin << " -> ";
		for(unsigned int y = 0; y < v.second.PathVector.size(); y++) {
			EV << v.second.PathVector[y].origin << ";";
		}
		EV << endl;
	}
	if (myAddr.valid) {
		EV << " ID of nearest Landmark " << myAddr.landmark.id << endl;
		/// XXX: This is probably broken (starting from data structure / generateAddr()
		/// origin is set using landmark vector entries... const char *
		//EV << " name of nearest Landmark " << myAddr.landmark.origin << endl;
		EV << " name/id of nearest Landmark " << myAddr.landmark.id << endl;
	} else {
		EV << " No Landmark known yet" << endl;
	}
}


void Disco::printRoutingPath(StandardPkt* stdp) {
	EV << " Path from " << stdp->getSource().origin <<" to " << stdp->getDestination().origin << " via Landmark " << stdp->getVia_Landmark().origin << endl;
	for (unsigned int x= stdp->getRoutingPathArraySize() ; x>0 ; x--) {
		EV << stdp->getRoutingPath(x-1).origin << "; ";
	}
}


void Disco::handleLandmarkAnnouncement(NodePkt *np) {
	// Every node needs to know every landmark and a path to it

	LandmarkAnnouncementsReceived++;
	DiscoPkt *dp = dynamic_cast<DiscoPkt *>(np->getEncapsulatedPacket());
	Node_ID landmark_ID = dp->getPathVector(0);

	int hop_count = dp->getPathVectorArraySize();

	// Do we know this landmark?
	if (isKnownLandmark(landmark_ID)) {
		if (landmarkSet[landmark_ID].hops > hop_count) {
			// found better path -> update
			saveAddAndSendPackage(np, dp);
		} else {
			// path is not better -> ignore path & delete packet
			EV << "the path to this landmark is longer or equally long as the previous one -> ignore packet" << endl;
		}
	} else {
		// Landmark is not known -> save path to new landmark
		saveAddAndSendPackage(np, dp);
	}
}


void Disco::saveAddAndSendPackage(NodePkt *np, DiscoPkt *dp) {
	int hop_count = dp->getPathVectorArraySize();
	saveNewPathVector(np->getType(),dp);
	if (np->getType() == landmarkAnnouncement) {
		generateAddr();
		LandmarkAnnouncementsForwarded+=node().getNeighbors().size()-1;
		np->setName("Landmark Forward");
	} else if (np->getType() == vicinityAnnouncement) {
		VicinityAnnouncementsForwarded+=node().getNeighbors().size()-1;
		np->setName("Vicinity Forward");
	}
	dp->setPathVectorArraySize(hop_count+1);
	dp->setPathVector(hop_count, localNodeId);
	sendToOtherNeighbors(np);
}


void Disco::nodeStatistics() {
	//known landmarks, vicinity size, ...
	activeNeighbors.record(node().getNeighbors().size());
}


void Disco::vicinityStatistics() {
	vicinitySize.record(vicinitySet.size());
	double vicinityCosts=0;
	for (const auto &p : vicinitySet) {
		vicinityCosts+= p.second.hops;
	}
	vicinityCost.record(vicinityCosts);

	// landmark in vicinity?
}


void Disco::landmarkStatistics() {
	if (myAddr.valid and isKnownLandmark(myAddr.landmark)) {
		landmarkID.record(myAddr.landmark.id);
		landmarkDistance.record(landmarkSet[myAddr.landmark].PathVector.size());
	}
}


void Disco::messageStatistics() {
	statLandmarkAnnouncementsOriginated.record(LandmarkAnnouncementsOriginated);
	statLandmarkAnnouncementsReceived.record(LandmarkAnnouncementsReceived);
	statLandmarkAnnouncementsForwarded.record(LandmarkAnnouncementsForwarded);

	statVicinityAnnouncementsOriginated.record(VicinityAnnouncementsOriginated);
	statVicinityAnnouncementsReceived.record(VicinityAnnouncementsReceived);
	statVicinityAnnouncementsForwarded.record(VicinityAnnouncementsForwarded);
}


Node_ID Disco::getVicinityMaxEntry() const
{
	ASSERT(not vicinitySet.empty());
	// normally this will be only called if the vicinitySet has reached its max size
	int maxhops(-1);
	Node_ID maxentry;
	for (auto cit= vicinitySet.cbegin(); cit != vicinitySet.cend(); cit++)
	{
		if (cit->second.hops > maxhops) {
			maxentry= cit->first; // store key
			maxhops= cit->second.hops;
		}
	}
	return maxentry;
}


void Disco::handleVicinityPacket(NodePkt *np) {
	VicinityAnnouncementsReceived++;
	DiscoPkt *dp = dynamic_cast<DiscoPkt *>(np->getEncapsulatedPacket());
	Node_ID node(dp->getPathVector(0));
	int hop_count(dp->getPathVectorArraySize());
	EV << "Node ID " << node.id << " at distance " << hop_count << endl;

	if (NodeInVicinity(node)) {
		// Node is already known
		if (vicinitySet[node].hops > hop_count) {
			// found better path: update
			saveAddAndSendPackage(np,dp);
		} else {
			// path is not better -> ignore path
			EV << " Node already in the vicinity of this Node" << endl;
		}
	} else {
		// Node is not known
		// check if the vicinity limit is not yet reached -> save Path
		if (vicinitySet.size() < (sqrt(networkSize*(log10(networkSize)/log10(2))))) {
			saveAddAndSendPackage(np,dp);
		} else {
			// check if #hops to Node is smaller than the max
			Node_ID maxentry(getVicinityMaxEntry()); 
			if (vicinitySet[maxentry].hops > hop_count) {
				// erase the Node which is further away; save new
				vicinitySet.erase(maxentry);
				saveAddAndSendPackage(np,dp);
			} else {
				EV << " Node not in the vicinity of this Node" << endl;
			}
		}
	}
}


void Disco::saveNewPathVector(int type,DiscoPkt *dp) {
	// extract path from packet
	landmarkPath path;
	path.landmarkID = dp->getPathVector(0);
	path.hops = dp->getPathVectorArraySize();
	for (unsigned int i = 0; i < dp->getPathVectorArraySize(); i++) {
		path.PathVector.push_back(dp->getPathVector(i));
	}

	ASSERT(not path.PathVector.empty());
	// save path
	switch(type) {
		case landmarkAnnouncement:
			landmarkSet[path.landmarkID]= path;
			break;
		case vicinityAnnouncement:
			vicinitySet[path.landmarkID]= path;
			break;
		case BetterPathPkt:
			shortcutSet[path.landmarkID]= path;
			break;
	}
}


/// Determine whether the landmark is known to the local node
bool Disco::isKnownLandmark(const Node_ID landmark_ID) {
	return landmarkSet.find(landmark_ID) != landmarkSet.cend();
}


/// Determine whether a node is contained in the vicinity
bool Disco::NodeInVicinity(Node_ID node) {
	return vicinitySet.find(node) != vicinitySet.cend();
}


/// Determine whether a node is contained in the ShortCutVector
bool Disco::NodeInShortCutVector(Node_ID node) {
	return shortcutSet.find(node) != shortcutSet.cend();
}


/// Determine whether a node is contained in the Address-Vector of a StandardPkt
bool Disco::nodeInAddress(StandardPkt* stdp) {
	for (unsigned int x = stdp->getRoutingPathArraySize(); x>0; x--) {
		if (localNodeId.id == stdp->getRoutingPath(x-1).id) {
			return true;
		}
	}
	return false;
}


unsigned int Disco::getOwnAddrIdx(StandardPkt* stdp) {
	for (unsigned int x = stdp->getRoutingPathArraySize(); x>0; x--) {
		if (localNodeId.id == stdp->getRoutingPath(x-1).id) {
			return x-1;
		}
	}
	EV << __func__ << ": could not find Node in Address-Vector" << endl;
	EV_WARN << "could not find Node in Address-Vector" << endl;
	return -1;
}


unsigned int Disco::getAddrIdx(StandardPkt* stdp, Node_ID node) {
	for (unsigned int x = stdp->getRoutingPathArraySize(); x>0; x--) {
		if (node.id == stdp->getRoutingPath(x-1).id) {
			return x-1;
		}
	}
	EV << __func__ << ": could not find Node in Address-Vector" << endl;
	EV_WARN << "could not find Node in Address-Vector" << endl;
	return -1;
}

// Returns true if one of the Nodes in the Address-Path is in the Node's vicinity
// AND the vicinity-Path to this Node is shorter than the original one
// begin with Destination (the End of the Path) and continue towards the Landmark (the Beginning of the Path)
bool Disco::nodeInVector(StandardPkt* stdp) {
	for (unsigned int x=stdp->getRoutingPathArraySize(); x>0 ; x--) {
		if (NodeInVicinity(stdp->getRoutingPath(x-1))) {
			if (stdp->getPastLandmark()) {
				// Packet is already on the shortest route to the destination
				return false;
			}
			else {
				if (localNodeId.id == stdp->getVia_Landmark().id) {
					return false;
				}
				if (landmarkSet[stdp->getVia_Landmark()].PathVector.size()+x-1 > vicinitySet[stdp->getRoutingPath(x-1)].PathVector.size()) {
					return true;
				}
			}
		}
	}
	return false;
}


// int Disco::getNodeIdx(StandardPkt* stdp) {
// 	for (unsigned int x=stdp->getRoutingPathArraySize(); x>0 ; x--) {
// 		if (NodeInVicinity(stdp->getRoutingPath(x-1))) {
// 			return getVicinityIdx(stdp->getRoutingPath(x-1));
// 		}
// 	}
// 	EV_WARN << __func__ << ": could not find Node in Routing-Vector" << endl;
// 	return -1;
// }


// int Disco::getShortCutIdx(Node_ID Node) {
// 	for (unsigned int x= 0; x< shortcutSet.size(); x++) {
// 		if (shortcutSet[x].PathVector.back().id == Node.id) {
// 			return x;
// 		}
// 	}
// 	EV_WARN << __func__ << ": could not find Node in shortcut vector" << endl;
// 	return -1;
// }


// int Disco::getLandmarkIdx(Node_ID landmark) {
// 	for (unsigned int x= 0; x< landmarkSet.size(); x++) {
// 		if (landmarkSet[x].PathVector[0].id == landmark.id) {
// 			return x;
// 		}
// 	}
// 	EV_WARN << __func__ << ": could not find landmark in landmark vector" << endl;
// 	return -1;
// }


// int Disco::getVicinityIdx(Node_ID node) {
// 	for (unsigned int x= 0; x< vicinitySet.size(); x++) {
// 		if (vicinitySet[x].PathVector[0].id == node.id) {
// 			return x;
// 		}
// 	}
// 	EV_WARN << __func__ << ": could not find node " << node.id << " in vicinity vector" << endl;
// 	return -1;
// }

void Disco::sendToOtherNeighbors(NodePkt *np) {
	// we are the last node in the path, extract the node before us
	DiscoPkt *dp = dynamic_cast<DiscoPkt *>(np->getEncapsulatedPacket());
	Node_ID origin_Id = dp->getPathVector(dp->getPathVectorArraySize()-2);

	// send packet to all other neighbors
	neighbor_set = node().getNeighbors();
	for (unsigned int x = 0 ; x < neighbor_set.size() ;x++) {
		if (origin_Id.id != neighbor_set[x]) {
			observer->setNumofLandmarkAnnouncements();
			sendToNode(neighbor_set[x],np->dup());
		}
	}
}


Disco::~Disco() {
	// set to nullptr as otherwise the old value is used on rebuilding the network -> segfault
	observer = nullptr;

	write_in_file.close();

	if (isLandmark) {
		cancelAndDelete(landmarkAnnouncementTimer);
	}

	cancelAndDelete(vicinityAnnouncementTimer);
	cancelAndDelete(cooldownTimer);
	cancelAndDelete(sendDataPacketsTimer);
}


/// Initialize
void Disco::onInitialize()
{
	//clear File-Entries if the File already exists
	write_in_file.open("Landmark_ID.txt",ios::out|ios::trunc);
	write_in_file.close();

	///get Shortcut paramters
	to_Destination_Shortcutting = par("to_Destination_Shortcutting");
	no_Path_Knowledge_Shortcutting = par("no_Path_Knowledge_Shortcutting");
	up_Down_Stream_Shortcutting = par("up_Down_Stream_Shortcutting");
	probingNodeFraction = par("probingNodeFraction");
	probePacketDestinationFraction = par("probePacketDestinationFraction");
	probeInterval = (simtime_t) par("probeInterval").doubleValue();


	/// Set Timer
	landmarkAnnouncementTimer=nullptr;
	vicinityAnnouncementTimer=nullptr;
	cooldownTimer=nullptr;
	sendDataPacketsTimer=nullptr;

	// get Disco Observer, required for networkSize -> becomeLandmark
	observer = getDiscoObserver();
	observer->registerNode(this);

	// add sampler
	addStatisticSample(this);

	vicinitySize.setName("Vicinity Size");
	vicinityCost.setName("Vicinity Cost (Sum Distance)");

	landmarkDistance.setName("Distance to closest landmark");
	landmarkID.setName("ID of closest landmark");

	activeNeighbors.setName("Active Neighbors (==Degree)");

	statLandmarkAnnouncementsOriginated.setName("Number of LandmarkAnnouncements originated");
	statLandmarkAnnouncementsReceived.setName("Number of LandmarkAnnouncements received");
	statLandmarkAnnouncementsForwarded.setName("Number of LandmarkAnnouncements forwarded");

	statVicinityAnnouncementsOriginated.setName("Number of VicinityAnnouncements originated");
	statVicinityAnnouncementsReceived.setName("Number of VicinityAnnouncements received");
	statVicinityAnnouncementsForwarded.setName("Number of VicinityAnnouncements forwarded");

	LandmarkAnnouncementsOriginated=0;
	LandmarkAnnouncementsReceived=0;
	LandmarkAnnouncementsForwarded=0;

	VicinityAnnouncementsOriginated=0;
	VicinityAnnouncementsReceived=0;
	VicinityAnnouncementsForwarded=0;

	// set local Node ID/Name
	isLandmark = false;
	myAddr.valid = false;

	localNodeId.id = node().getNodeId();
	localNodeId.origin = node().getName();

	displayAddr();

	// registering packet type with Node
	node().bind(landmarkAnnouncement, this);
	node().bind(dataPacket, this);
	node().bind(vicinityAnnouncement, this);
	node().bind(BetterPathPkt, this);

	// schedule initial landmark selection and disco boot
	// cannot be done here as we need to be sure that the
	// NetworkManager has been initialized, even in the case
	// of a network given by a ned-topology
	initialTimer = new NodePkt();
	initialTimer->setName("initialTimer");
	scheduleAt(SIMTIME_ZERO, initialTimer);

	WATCH(landmarkSet);
	WATCH(vicinitySet);
	WATCH(shortcutSet);
}


/// called when a new node is connected
void Disco::onConnect(nodeid_t nid) {
	//EV << "connected to "<< nid << endl;
}


/// called when a node is disconnected
void Disco::onDisconnect( nodeid_t nid )
{
	//EV << "disconnected from "<< nid << endl;

}


void Disco::reset() {

}


void Disco::sampleStatistics( Statistics& stats ) {
	//vicinityStatistics();
	//landmarkStatistics();
	//XXX: massive overhead... do we need this? Granularity?
	//messageStatistics();
	//nodeStatistics();
}


bool Disco::addressInLandmarks(discoid_t addr) {
	return isKnownLandmark(addr.landmark);
}


void Disco::becomeLandmark() {
	double p = uniform(0,1);
	if (p < sqrt((log10(networkSize)/log10(2))/networkSize)) {
		EV << "becoming a landmark, id=" << localNodeId.id << endl;
		isLandmark = true;
		//write Landmark_ID in File
		write_in_file.open("Landmark_ID.txt",ios::out|ios::app);
		write_in_file << localNodeId.id << endl;
		write_in_file.close();
	} else {
		isLandmark = false;
		//EV << " p = "<< p << " I'm not a Landmark" << endl;
	}
}


void Disco::sendPktToNode(discoid_t addr) {
	Node_ID destination;

	EV << "Node "<< localNodeId.id <<": Sending Packet to " << addr << endl;

	// Destination is a Landmark
	if (addr.pathToNode.empty()) {
		// Check that sender is not also destination
		if (addr.landmark.id == localNodeId.id) {
			EV << "Not sending packet (destination == source)" << endl;
			return;
		}
		// set destination
		destination = addr.landmark;
	} else {
		// Check that sender is not also destination
		if (addr.pathToNode.back().id == localNodeId.id) {
			EV << "Not sending packet (destination == source)" << endl;
			return;
		}

		// set destination
		destination = addr.pathToNode.back();
	}

	// Generate new Pkt and set basic information
	NodePkt *np = new NodePkt();
	np->setType(dataPacket);
	np->setName("Data Packet");
	StandardPkt *stdp = new StandardPkt();
	Node_ID landmark = addr.landmark;
	stdp->setSource(localNodeId);
	stdp->setDestination(destination);
	stdp->setVia_Landmark(landmark);

	// register data packet at statistics module
	getStatistics().registerMessage(np);

	if (NodeInVicinity(destination) and to_Destination_Shortcutting) {
		// Node in Vicinity
		// Save direct Path in Packet (but in reverse order) and set pastLandmark true
		// TODO: send via direct, known path
		int m_size = vicinitySet[destination].PathVector.size();
		stdp->setRoutingPathArraySize(m_size);
		for (unsigned int x=m_size; x>0  ; x--) {
			stdp->setRoutingPath(m_size-x,vicinitySet[destination].PathVector[x-1]);
		}

		stdp->setPastLandmark(true);
		np->encapsulate(stdp);
		sendToNode(vicinitySet[destination].PathVector.back().id, np);
	}
	else if (NodeInShortCutVector(destination)) {
		// This is not the first Packet and there is a better Path to the Destination, take this better path
		// set PastLandmark true, so the following Nodes will route along the better Path
		EV << "Found Node in ShortCutVector -> Shortcut" << endl;
		stdp->setRoutingPathArraySize(shortcutSet[destination].PathVector.size());
		for (unsigned int x=0; x< shortcutSet[destination].PathVector.size() ; x++) {
			stdp->setRoutingPath(x,shortcutSet[destination].PathVector[x]);
		}
		stdp->setPastLandmark(true);
		np->encapsulate(stdp);
		sendToNode(shortcutSet[destination].PathVector[0].id,np);
	}
	else if (isKnownLandmark(landmark)) {
		// No shortcuts were found, follow the usual Protocol.
		// Save Address-Path and send Packet towards the Address-Landmark
		unsigned int addr_size(addr.pathToNode.size());
		stdp->setRoutingPathArraySize(addr_size);
		for (unsigned y = 0; y<addr_size; y++) {
			stdp->setRoutingPath(y,addr.pathToNode[y]);
		}
		stdp->setPastLandmark(false);
		np->encapsulate(stdp);
		sendToNode(landmarkSet[landmark].PathVector.back().id,np);
	}
	else if (addr.landmark.id == localNodeId.id) {
		// I am the Address-Landmark -> send Packet along the Address-Path
		// set pastLandmark true as an indicator that every Node has to follow the given Route to the Destination
		unsigned int addr_size = addr.pathToNode.size();
		stdp->setRoutingPathArraySize(addr_size);
		for (unsigned y = 0; y<addr_size; y++) {
			stdp->setRoutingPath(y,addr.pathToNode[y]);
		}
		stdp->setPastLandmark(true);
		np->encapsulate(stdp);
		sendToNode(addr.pathToNode[1].id,np);
	}
	else {
		// ERROR!! The Address-Landmark is unknown and there is no other known Path to the Destination
		EV_ERROR << "Landmark " << addr.landmark << " is unknown, dropping packet!" << endl;
		getStatistics().dropMessage(np, stdp->getDestination().id);
		delete(np);
		delete(stdp);
	}
}


void Disco::handleDataPacket(NodePkt *np) {
	StandardPkt *stdp = dynamic_cast<StandardPkt *>(np->getEncapsulatedPacket());

	/// local node is the packet's ultimate destination
	if (stdp->getDestination().id == localNodeId.id) {
		EV_TRACE << "Received Data Packet" << endl;
		stdp->setHops(stdp->getHops()+1);

		// notify statistics module
		getStatistics().deliveredMessage(np, stdp->getHops());

		findShortcut(stdp);

		if (no_Path_Knowledge_Shortcutting) {
			checkForShorterPath(stdp->getSource(),stdp->getHops());
		}

		// free the packet as it is not required anymore
		delete np;
		return;
	}


	if (to_Destination_Shortcutting and NodeInVicinity(stdp->getDestination())) {
		// Vicinity Shortcut enabled && Node in Vicinity
		// Route directly to destination, no Path-Change necessary,
		// because every Node will have the destination in its vicinity
		EV_TRACE << "Node " << localNodeId.id << ": Destination " << stdp->getDestination().id << " in Vicinity --> Shortcut" << endl;
		stdp->setHops(stdp->getHops()+1);
		EV << vicinitySet[stdp->getDestination()];
		sendToNode(vicinitySet[stdp->getDestination()].PathVector.back().id, np);
	}
	else
	if (up_Down_Stream_Shortcutting and nodeInVector(stdp)) {
		// Shortcut enabled && one Node along the Route is in Vicinity.
		// Route to this Node, change the Path: save Path to known Node as stored in the vicinity vector
		// and add the rest of the old Address-Path
		EV_TRACE << "Node: " << localNodeId.id << " Found Shortcut to Node " << stdp->getDestination().id << " along the Route" << endl;
		stdp->setHops(stdp->getHops()+1);
		ASSERT(stdp->getRoutingPathArraySize()>0);

		Node_ID lastnode(stdp->getRoutingPath(stdp->getRoutingPathArraySize()-1));
		unsigned int vicinity_path_size(0);
		unsigned int posInAddr(0);
		for (unsigned int x = stdp->getRoutingPathArraySize(); x > 0; x--) {
			lastnode= stdp->getRoutingPath(x-1);
			if (NodeInVicinity(lastnode)) {
				vicinity_path_size = vicinitySet[lastnode].PathVector.size();
				posInAddr = getAddrIdx(stdp,vicinitySet[lastnode].PathVector[0]);
				break;
			}
		}

		ASSERT(vicinity_path_size > 0);

		// calculate new Path-Size
		unsigned int new_size = vicinity_path_size + stdp->getRoutingPathArraySize()-posInAddr-1;

		//copy old Path XXX: MEMORY LEAK, trying dirty delete below after cstdp usage
		StandardPkt* cstdp = new StandardPkt();
		cstdp->setRoutingPathArraySize(stdp->getRoutingPathArraySize());
		for (unsigned int x = 0; x<cstdp->getRoutingPathArraySize(); x++) {
			cstdp->setRoutingPath(x,stdp->getRoutingPath(x));
		}

		// save new Path, beginning with the vicinity vector entries in reverse order...
		stdp->setRoutingPathArraySize(new_size);
		for (unsigned int x = 0 ; x<vicinity_path_size; x++) {
			stdp->setRoutingPath(x,vicinitySet[lastnode].PathVector[vicinity_path_size-1-x]);
		}
		//...followed by the rest of the old Address-Path
		for (unsigned int y = 0; y< new_size-vicinity_path_size; y++) {
			stdp->setRoutingPath(vicinity_path_size+y,cstdp->getRoutingPath(posInAddr+1+y));
		}
		delete cstdp;
		stdp->setPastLandmark(true);
		sendToNode(vicinitySet[lastnode].PathVector.back().id,np);
	}
	else if (localNodeId.id == stdp->getVia_Landmark().id) {
		// Node is the Address-Landmark to the destination Node
		// set pastLandmark true as an indicator that every Node has to follow the given Route to the Destination
		// and that this Route is already the shortest way to the destination
		EV_TRACE << "Node " << localNodeId.id <<"(Landmark) to Node " << stdp->getDestination().id << "from Node " << stdp->getSource().id << endl;
		stdp->setPastLandmark(true);
		stdp->setHops(stdp->getHops()+1);
		unsigned int posInVector = getOwnAddrIdx(stdp);
		EV_TRACE << "Routing Path (setting index to "<< stdp->getHops() <<")"<< endl;
		for (unsigned int i=0; i<stdp->getRoutingPathArraySize(); i++) {
			EV_TRACE << stdp->getRoutingPath(i) << " ";
		}
		EV_TRACE << endl;
		sendToNode(stdp->getRoutingPath(posInVector+1).id,np);
	}
	else if (stdp->getPastLandmark()) {
		// Follow the given Route to the destination
		stdp->setHops(stdp->getHops()+1);
		unsigned int posInVector = getOwnAddrIdx(stdp);
		EV << "Node " << localNodeId.id <<": "<< " to Node " << stdp->getDestination().id << " from Node " << stdp->getSource().id << endl;
		//EV << "Past Landmark, next hop: " << stdp->getRoutingPath(stdp->getHops()).id<< endl;
		sendToNode(stdp->getRoutingPath(posInVector+1).id,np);
	}

	// ERROR: Node doesn't know Address-Landmark -> Refresh Landmark announcements
	else if (!isKnownLandmark(stdp->getVia_Landmark()))
	{
		getStatistics().dropMessage(np, dynamic_cast<StandardPkt *>(np->getEncapsulatedPacket())->getDestination().id);
		delete(np);
		EV_ERROR << "Unknown Landmark!!-> refresh landmark announcements" << endl;
	} else {
		// Packet is still on its Way to the Address-Landmark, Route towards to Address-Landmark
		stdp->setHops(stdp->getHops()+1);
		sendToNode(landmarkSet[stdp->getVia_Landmark()].PathVector.back().id,np);
	}
}

void Disco::findShortcut(StandardPkt *stdp) {
	// Source is in the Vicinity of the Destination
	if (NodeInVicinity(stdp->getSource())) {
		// if path Destination->Source is shorter, send BetterPathPkt
		if (stdp->getHops()> vicinitySet[stdp->getSource()].hops) {
			NodePkt *np = new NodePkt();
			np->setType(BetterPathPkt);
			np->setName("I'm a BetterPathPkt (Vicinity)");
			DiscoPkt *dp = new DiscoPkt();
			unsigned int path_size = vicinitySet[stdp->getSource()].PathVector.size();
			dp->setPathVectorArraySize(path_size+1);

			for (unsigned int x=0; x<path_size; x++) {
				dp->setPathVector(x,vicinitySet[stdp->getSource()].PathVector[x]);
			}
			dp->setPathVector(path_size,localNodeId);

			np->encapsulate(dp);
			sendToNode(vicinitySet[stdp->getSource()].PathVector.back().id, np);
		}
	}
}


// no_Path_Knowledge_Shortcutting: The Destination checks if the Path Destination-to-Source is shorter
// if this is the case then it will send a BetterPathPkt to inform the Source that there is a shorter Path
void Disco::checkForShorterPath(Node_ID source, unsigned int hops) {
	discoid_t addr;
	addr = observer->getNodeAddress(source.id);
	for (auto lit= landmarkSet.cbegin(); lit != landmarkSet.cend(); lit++) {
		if (lit->second.landmarkID.id == addr.landmark.id) {
			// Sender is a Landmark
			if(addr.pathToNode.empty()) {
				unsigned int numofhops = lit->second.PathVector.size();
				if (numofhops < hops) {
					NodePkt *np = new NodePkt();
					np->setType(BetterPathPkt);
					np->setName("I'm a BetterPathPkt (NoPathKnowledge)");
					DiscoPkt *dp = new DiscoPkt();

					dp->setPathVectorArraySize(numofhops);
					for (unsigned int y=0; y<lit->second.PathVector.size(); y++) {
						dp->setPathVector(y, lit->second.PathVector[y]);
					}
					np->encapsulate(dp);
					sendToNode(lit->second.PathVector.back().id, np);
				}
				break;
			} else {
				// Not a Landmark
				unsigned int numofhops = lit->second.PathVector.size()+addr.pathToNode.size()-1;
				if (numofhops < hops) {
					NodePkt *np = new NodePkt();
					np->setType(BetterPathPkt);
					np->setName("I'm a BetterPathPkt (NoPathKnowledge)");
					DiscoPkt *dp = new DiscoPkt();

					dp->setPathVectorArraySize(numofhops);
					for (unsigned int y=addr.pathToNode.size();y>0; y--) {
						Node_ID id;
						id = addr.pathToNode[y-1];
						dp->setPathVector(addr.pathToNode.size()-y,id);
					}
					for (unsigned int y=1; y< lit->second.PathVector.size(); y++) {
						Node_ID id;
						id = lit->second.PathVector[y];
						dp->setPathVector(y+addr.pathToNode.size()-1,id);
					}

					np->encapsulate(dp);
					sendToNode(lit->second.PathVector.back().id,np);
				}
				break;
			}

		}
	}
}


void Disco::handleBetterPathPkt(NodePkt* np) {
	DiscoPkt *dp = dynamic_cast<DiscoPkt *>(np->getEncapsulatedPacket());

	if (dp->getPathVector(0).id == localNodeId.id) {
		// Packet has reached it destination
		EV_TRACE << "Better Path is saved" << endl;
		saveNewPathVector(np->getType(),dp);
		delete np;
		return;
	} else {
		// Not the destination, send packet along the route
		for (unsigned int x=0 ; x < dp->getPathVectorArraySize(); x++) {
			if (dp->getPathVector(x).id == localNodeId.id) {
				sendToNode(dp->getPathVector(x-1).id, np);
				return;
			}
		}
	}
}


void Disco::generateAddr() {
	if (isLandmark) {
		myAddr.landmark = localNodeId;
		myAddr.valid = true;
	} else {
		int min = numeric_limits<int>::max();

		// no landmarks --> no address
		if (landmarkSet.size() == 0) {
			EV_WARN << "address not determined yet, no landmarks known" << endl;
			myAddr.valid = false;
		} else {
			// determine address depending on next landmark
			auto litpos(landmarkSet.cbegin());
			for (auto lit= landmarkSet.cbegin() ; lit != landmarkSet.cend(); lit++) {
				if (min > lit->second.hops) {
					min= lit->second.hops;
					litpos= lit;
				}
			}
			myAddr.landmark = litpos->second.landmarkID;
			myAddr.pathToNode = litpos->second.PathVector;
			myAddr.pathToNode.push_back(localNodeId);
			myAddr.valid = true;
		}
	}
	displayAddr();
}


void Disco::displayAddr() {
	char s[256];
	if (myAddr.valid) {
		if (isLandmark) {
			sprintf(s,"%u",(unsigned)localNodeId.id);
		} else {
			std::ostringstream output;
			output << myAddr.landmark.id << ".";
			for (unsigned int i = 1; i < myAddr.pathToNode.size(); i++) {
				output << myAddr.pathToNode.at(i).id;
				if (i < myAddr.pathToNode.size() - 1) output << "-";
			}
			sprintf(s, "%s", output.str().c_str());
		}
	} else {
		sprintf(s, "(%u)", (unsigned)localNodeId.id);
	}

	node().getDisplayString().setTagArg("t", 0, s);
}

std::ostream& operator<<(std::ostream &os, std::unordered_map<Node_ID,landmarkPath> nodeSet)
{
	for (auto &n : nodeSet) {
		os << n.first << ": " << n.second << endl;
	}
	return os;
}



} //namespace
