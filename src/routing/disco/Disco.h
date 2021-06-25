/*
 * Disco.h
 * 
 * According to the Paper Singla et al.: "Scalable Routing on Flat Names"
 *
 *  Created on: Jun 23, 2011
 *      Author: Preis
 */

#ifndef DISCO_H_
#define DISCO_H_

#include <omnetpp.h>
#include "foundation.h"

#include "DiscoPkt_m.h"
#include "StandardPkt_m.h"
#include "DiscoObserver.h"
#include "StatisticSample.h"
#include <algorithm>
#include <vector>
#include <fstream>
#include <unordered_map>



#define CSTR(X,Y) \
		std::ostringstream __##X; __##X << Y; \
		const char* X = __##X.str().c_str()

#define BUBBLE(X,Y) CSTR(X,Y); node().bubble(X);

#define TOOLTIP(X,Y) CSTR(X,Y); node().getDisplayString().setTagArg("tt",0,X);

#define TEXT(X,Y) CSTR(X,Y); node().getDisplayString().setTagArg("t",0,X);

// Node_ID is declared in DiscoPkt.msg
namespace std {
	template<> class hash<routingsim::Node_ID>
	{
	public:
		size_t operator()(const routingsim::Node_ID& k) const;
	};

	inline
	std::size_t std::hash<routingsim::Node_ID>::operator()(const routingsim::Node_ID& k) const {
			using std::size_t;
			
			return k.id;
	};
}

namespace routingsim {

using namespace std;


class landmarkPath // find better name -> used not only for landmarks but also for the vicinity and shortcuts
{
public:
	vector <Node_ID> PathVector;

	Node_ID landmarkID; // not strictly necessary since ID is in vector at pos(0) anyway
	int hops; // not strictly necessary since equals to sizeOfVector

	friend ostream& operator<<(ostream &out, const landmarkPath &lp) {
		out << "Path Vector: <";
		for (unsigned int i=0; i<lp.PathVector.size(); i++) {
			out << " " << lp.PathVector[i].id;
		}
		out << " >";
		return out;
	}
};

/// Dummy definition, to integrate observer
class discoid_t {
	// path from landmark to node
public:
	Node_ID landmark;
	vector<Node_ID> pathToNode; /// TODO: check that landmark is not put (redundantly) into vector
	bool valid;

	bool operator==(const discoid_t &t) const { return this->landmark.id == t.landmark.id; }
	
	friend ostream &operator<<(ostream& out, const discoid_t &disco) {
		if ( !disco.pathToNode.empty() ) {
			out << "Destination: Node " << (disco.pathToNode.size() > 0 ? disco.pathToNode.back().id : -1) << " (non-landmark)" << endl;
			out << "Landmark: Node " << disco.landmark.id;
			out << " Path from Landmark to Node:";
			for(unsigned int i=0; i<disco.pathToNode.size(); i++) {
				out << " " << disco.pathToNode[i].id;
			}
		} else {
			out << "Destination: Node "<< disco.landmark.id << " (Landmark)";
		}

		return out;
	}
	discoid_t& operator=(const discoid_t &t) {

		if (this == &t) // protect against invalid self-assignment
			return *this;

		landmark = t.landmark;
		pathToNode = t.pathToNode;
		valid = t.valid;

		return *this;
	}
};

class DiscoObserver;

class Disco : public LegacyProtocol, public StatisticSample
{
	friend class DiscoObserver;
protected:
	/// Initialize
	void onInitialize();

	/// called when a new node is connected
	void onConnect( nodeid_t nid );

	/// called when a node is disconnected
	void onDisconnect( nodeid_t nid );

	/// on message
	void onMessage( NodePkt* pkt );

	void onSelfMessage(cMessage* msg);

	/// drop all acquired state
	void reset();

	/// sample statistics
	void sampleStatistics( Statistics& stats );


private:
	/// Timers
	NodePkt *landmarkAnnouncementTimer = nullptr;
	NodePkt *vicinityAnnouncementTimer = nullptr;
	NodePkt *cooldownTimer = nullptr;
	NodePkt *sendDataPacketsTimer = nullptr;
	NodePkt *initialTimer = nullptr;

	/// is the local node a landmark?
	bool isLandmark = false;

	/// Parameters to switch Shortcuts on/off
	bool to_Destination_Shortcutting;
	bool no_Path_Knowledge_Shortcutting;
	bool up_Down_Stream_Shortcutting;

	// Probe packet configuration
	double probingNodeFraction;
	double probePacketDestinationFraction;
	simtime_t probeInterval;

	int networkSize = 0;

	uint forwardedPackets = 0;
	uint droppedPackets = 0;
	uint sentPackets = 0;
	uint receivedPackets = 0;
	uint forwardedDataPackets = 0;
	uint droppedDataPackets = 0;
	uint sentDataPackets = 0;
	uint receivedDataPackets = 0;

	//bool knownLandmark;
	//bool knownNode;
	//int posInLandVector;
	//int posInViciniVector;
	;

	fstream write_in_file;
	Node_ID localNodeId;
	vector_set<nodeid_t> neighbor_set;
	discoid_t myAddr;
	DiscoObserver *observer;
	std::unordered_map<Node_ID,landmarkPath> landmarkSet;
	std::unordered_map<Node_ID,landmarkPath> vicinitySet;
	std::unordered_map<Node_ID,landmarkPath> shortcutSet;
	

	// statistics
	void recordScalars();

	void vicinityStatistics(Statistics &stats);
	void messageStatistics(Statistics &stats);
	void nodeStatistics(Statistics &stats);

	int LandmarkAnnouncementsOriginated;
	int LandmarkAnnouncementsReceived;
	int LandmarkAnnouncementsForwarded;

	int VicinityAnnouncementsOriginated;
	int VicinityAnnouncementsReceived;
	int VicinityAnnouncementsForwarded;

	~Disco();
	void becomeLandmark();
	void sendPktToNode(discoid_t pktAddr);
	void sendToOtherNeighbors(NodePkt *np);

	void handleLandmarkAnnouncement(NodePkt *np);
	void handleVicinityPacket(NodePkt *np);
	void handleDataPacket(NodePkt *np);
	void handleBetterPathPkt(NodePkt* np);

	void sendLandmarkAnnouncement();
	void sendVicinityAnnouncement();
	void saveNewPathVector(int type, DiscoPkt *dp);

	/// return the index of the worst (by hop distance) entry in the vicinity
	Node_ID getVicinityMaxEntry() const;

	/// return the index of the node in the vicinity vector
	unsigned int getOwnAddrIdx(StandardPkt* stdp);
	unsigned int getAddrIdx(StandardPkt* stdp,Node_ID node);

	bool nodeInVector(const StandardPkt* stdp);
	bool nodeInAddress(const StandardPkt* stdp) const;
	/// generate the node's disco address
	void generateAddr();
	/// Determine whether the landmark is known to the local node
	bool isKnownLandmark(Node_ID landmark_ID) const;
	/// Determine whether a node is contained in the vicinity
	bool NodeInVicinity(Node_ID node) const;
	/// Determine whether a node is contained in the vicinity
	bool NodeInShortCutVector(Node_ID node) const;

	bool addressInLandmarks(discoid_t addr) const;
	void printLandmarks();
	void printVicinity();
	void printRoutingPath(StandardPkt* stdp);
	void displayAddr();


	// Save Vector path; generate new Address; add own ID to package and Send it to the other Neighbors
	void saveAddAndSendPackage(NodePkt *np, DiscoPkt *dp);

	void findShortcut(StandardPkt *stdp);

	void checkForShorterPath(Node_ID source, unsigned int hops);


	vector<landmarkPath> stat;

};

inline
bool Disco::addressInLandmarks(discoid_t addr) const {
	return isKnownLandmark(addr.landmark);
}


std::ostream& operator<<(std::ostream &os, std::unordered_map<Node_ID,landmarkPath> nodeSet);

} //namespace


#endif /* DISCO_H_ */
