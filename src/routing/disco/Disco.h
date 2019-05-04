/*
 * Disco.h
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
#include <boost/foreach.hpp>

// #define foreach         BOOST_FOREACH



#define CSTR(X,Y) \
		std::ostringstream __##X; __##X << Y; \
		const char* X = __##X.str().c_str()

#define BUBBLE(X,Y) CSTR(X,Y); node().bubble(X);

#define TOOLTIP(X,Y) CSTR(X,Y); node().getDisplayString().setTagArg("tt",0,X);

#define TEXT(X,Y) CSTR(X,Y); node().getDisplayString().setTagArg("t",0,X);


namespace routingsim {

using namespace std;

	
class landmarkPath // find better name -> used not only for landmarks but also for the vicinity and shortcuts
{
public:
	vector <Node_ID> PathVector;

	Node_ID landmarkID; // nicht umbedingt nötig da wert in vektor an pos(0)
	int hops; //nicht umbedingt nötig da gleich sizeOfVector

	friend ostream& operator<<(ostream &out, const landmarkPath &lp) {
		out << "Path Vector:";
		for (unsigned int i=0; i<lp.PathVector.size(); i++) {
			out << " " << lp.PathVector[i].id;
		}
		out << endl;
		return out;
	}
};



/// nur dummy-definition, um den observer reinzubauen
/// Datentyp braucht Vergleichsoperator
class discoid_t {
	//path from landmark to node
public:
	Node_ID landmark;
	vector<Node_ID> pathToNode; /// TODO: check that landmark is not put (redundantly) into vector
	bool valid;

	bool operator==(discoid_t &t) {return this->landmark.id==t.landmark.id;}
	friend ostream &operator<<(ostream& out, const discoid_t &disco) {
		if ( !disco.pathToNode.empty() ) {
			out << "Destination: Node "<< disco.pathToNode.back().id <<" (non-landmark)"<< endl;
			out << "Landmark: Node "<< disco.landmark.id<<endl;
			out << "Path from Landmark to Node:";
			for(unsigned int i=0; i<disco.pathToNode.size(); i++) {
				out << " "<<disco.pathToNode[i].id;
			}
		} else {
			out << "Destination: Node "<< disco.landmark.id << " (Landmark)" << endl;
		}
		out << endl;
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

class Disco : public Protocol, public StatisticSample
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

	/// drop all acquired state
	void reset();

	/// sample statistics
	void sampleStatistics( Statistics& stats );


private:
	/// Timers
	NodePkt *landmarkAnnouncementTimer;
	NodePkt *vicinityAnnouncementTimer;
	NodePkt *cooldownTimer;
	NodePkt *sendDataPacketsTimer;
	NodePkt *initialTimer;

	/// is the local node a landmark?
	bool isLandmark;

	/// Parameters to switch Shortcuts on/off
	bool to_Destination_Shortcutting;
	bool no_Path_Knowledge_Shortcutting;
	bool up_Down_Stream_Shortcutting;

	// Probe packet configuration
	double probingNodeFraction;
	double probePacketDestinationFraction;


	int networkSize;
	//bool knownLandmark;
	//bool knownNode;
	//int posInLandVector;
	//int posInViciniVector;
	//int posOfMaxHops;
	fstream write_in_file;
	Node_ID localNodeId;
	vector_set<nodeid_t> neighbor_set;
	discoid_t myAddr;
	DiscoObserver *observer;
	vector <landmarkPath> landmarkVector;
	vector <landmarkPath> vicinityVector;
	vector <landmarkPath> shortcutVector;

	// statistics
	cOutVector vicinitySize; // number of nodes in vicinity
	cOutVector vicinityCost; // sum over hop-distance to all nodes in the vicinity
	cOutVector landmarkID; // id of the node's landmark
	cOutVector landmarkDistance; // distance to the node's landmark

	cOutVector activeNeighbors; //current node degree

	cOutVector statLandmarkAnnouncementsOriginated;
	cOutVector statLandmarkAnnouncementsReceived;
	cOutVector statLandmarkAnnouncementsForwarded;

	cOutVector statVicinityAnnouncementsOriginated;
	cOutVector statVicinityAnnouncementsReceived;
	cOutVector statVicinityAnnouncementsForwarded;

	void vicinityStatistics();
	void landmarkStatistics();
	void messageStatistics();
	void nodeStatistics();

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
	void saveNewPathVector(int type,DiscoPkt *dp);

	/// return the index of the worst (by hop distance) entry in the vicinity
	unsigned int getWorstVicinityEntryIdx();

	/// return the index of the landmark in the landmark vector
	unsigned int getLandmarkIdx(Node_ID landmark);

	/// return the index of the node in the vicinity vector
	unsigned int getVicinityIdx(Node_ID node);
	unsigned int getShortCutIdx(Node_ID node);
	unsigned int getNodeIdx(StandardPkt* stdp);
	unsigned int getOwnAddrIdx(StandardPkt* stdp);
	unsigned int getAddrIdx(StandardPkt* stdp,Node_ID node);

	bool nodeInVector(StandardPkt* stdp);
	bool nodeInAddress(StandardPkt* stdp);
	/// generate the node's disco address
	void generateAddr();
	/// Determine whether the landmark is known to the local node
	bool isKnownLandmark(Node_ID landmark_ID);
	/// Determine whether a node is contained in the vicinity
	bool NodeInVicinity(Node_ID node);
	/// Determine whether a node is contained in the vicinity
	bool NodeInShortCutVector(Node_ID node);

	bool addressInLandmarks(discoid_t addr);
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

} //namespace


#endif /* DISCO_H_ */
