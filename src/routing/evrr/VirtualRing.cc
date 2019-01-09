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

#include "VirtualRing.h"
#include "VRObserver.h"
#include "Statistics.h"

#include "VRouteUpdate.h"
#include "VRoutePkt_m.h"
#include "VRDataPkt_m.h"

#include <boost/tokenizer.hpp>
#include <boost/lexical_cast.hpp>
#include <boost/unordered_set.hpp>
#include <boost/unordered_map.hpp>
#include <boost/foreach.hpp>

// Virtual Ring Implementations
#include "VirtualRingEntry.h"
#include "VirtualRingQueue.h"
#include "VirtualRingRoutes.h"
#include "VirtualRingTable.h"
#include "VirtualRingAnycast.h"

#include "crng.h"

#define CSTR(X,Y) \
	std::ostringstream __##X; __##X << Y; \
	const char* X = __##X.str().c_str()

#define BUBBLE(X,Y) CSTR(X,Y); node().bubble(X);

#define TOOLTIP(X,Y) CSTR(X,Y); node().getDisplayString().setTagArg("tt",0,X);

#define TEXT(X,Y) CSTR(X,Y); node().getDisplayString().setTagArg("tt",0,X);

namespace routingsim {

const int vroute_update 		= 0x445;
const int vrdata_payload 		= 0x446;

using namespace boost;
using namespace std;

class VirtualRing::Table {
private:
	update_queue* queue_;
	route_maintenance* dsdv_;
	ring_table* ring_;
	anycast_table* anycast_;

public:
	Table(cRNG* rng) {
		queue_ = new update_queue();
		dsdv_ = new route_maintenance(*queue_);
		ring_ = new ring_table(*dsdv_, rng);
		anycast_ = new anycast_table(*ring_);
	}

	~Table() {
		delete ring_;
		delete dsdv_;
		delete queue_;
		delete anycast_;
	}

	update_queue& queue() {
		return *queue_;
	}

	route_maintenance& dsdv() {
		return *dsdv_;
	}

	ring_table& ring() {
		return *ring_;
	}

	anycast_table& anycast() {
		return *anycast_;
	}

	bool process( nodeid_t from, const vroute_update_t& route ) {
		bool processed = false;
		if (!processed) processed |= dsdv().process(from,route);
		if (!processed) processed |= ring().process(from,route);
		if (!processed) processed |= anycast().process(from,route);
		return processed;
	}
};




//============================================================================
//	VIRTUAL RING MODULE
//============================================================================

// module definition
Define_Module(VirtualRing);

static boost::unordered_set<vid_t> ids;
static boost::unordered_map<vid_t, vid_t> anycast_map;


vid_t VirtualRing::get_random_id() {
	vid_t id;
	do { id = (intuniform(0,65535) << 16) ^ (intuniform(0,65535)); }
	while (ids.count(id)!=0);
	ids.insert(id);
	return id;
}

vid_t VirtualRing::get_anycast_id( vid_t id ) {
	if (anycast_map.count(id)!=0) return anycast_map[id];
	vid_t nid = get_random_id();
	anycast_map.insert(make_pair(id,nid));
	return nid;
}

/// initialize
void VirtualRing::onInitialize() {

	// initialize routing table
	table = new Table(getRNG(0));

	// set evrr parameters
	roundTime 					= par( "roundTime" );
	immediateResponse 			= par( "immediateResponse");
	useRouteDescentCriterion 	= par( "useRouteDescentCriterion");
	consistencyCheck    		= par( "consistencyCheck");
	timeout						= par( "timeout" );
	sendMessageDelay			= par( "sendMessageDelay" );

	// set ring parameters
	table->ring().set_use_linearization(par( "useLinearization" ).boolValue());
	table->ring().set_discovery_alpha(par( "parallelDiscoveries").intValue());
	table->ring().set_delay_threshold(par("delayThreshold").intValue());
	table->anycast().set_delay_threshold(par("delayThreshold").intValue());

	// set node id
	table->dsdv().set_nodeid( node().getNodeId() );

	// register anycast addresses
	if (par("useAnycast").boolValue()) {
		string anycast = par("anycast");
		escaped_list_separator<char> sep("\\","\t ","\"'");
		typedef boost::tokenizer<escaped_list_separator<char> > tokenizer_t;
		tokenizer_t tok(anycast,sep);
		for (tokenizer_t::iterator it = tok.begin(); it != tok.end(); it++) {
			vid_t id = lexical_cast<uint32_t>(*it);
			table->anycast().add_id(get_anycast_id(id));
		}
	}

	// register ring id
	vid_t id = get_random_id();
	table->ring().set_id( id  );

	// set node id
	//table->dsdv()				= node().getNodeId();

	// get ring observer
	observer = &getVRObserver();

	// initialize statistics
	controlTrafficIn 		= 0;
	controlTrafficOut 		= 0;
	controlTrafficInLast 	= 0;
	controlTrafficOutLast	= 0;
	controlMessagesIn		= 0;
	controlMessagesInLast	= 0;
	controlMessagesOut		= 0;
	controlMessagesOutLast  = 0;
	routeUpdatesSent 		= 0;
	routeUpdatesReceived 	= 0;
	tableVNeighborChanges 	= 0;
	tableChanges 			= 0;
	seqChanges				= 0;

	consistencyCheckCount 	= -(consistencyCheck/2);

	connected				= false;

	// add sampler
	addStatisticSample(this);

	// bind module
	node().bind(vroute_update, this);
	node().bind(vrdata_payload, this);

	// demo visualization
	char str[30];
	sprintf(str,"%d",id);
	node().getDisplayString().setTagArg("t",0,str);
	node().setName("");

	// start timer and main-loop
	timer = new NodePkt();
	double startupDelay = (double)intuniform(10,1000)/1000.0;
	scheduleAt(simTime() + roundTime + startupDelay, timer);
}

/// called when a new neighbor is connected
void VirtualRing::onConnect(nodeid_t nid) {

	// inform rtTable about new neighbor
	table->dsdv().add_neighbor(nid);

	// the node is connected when it has at least one neighbor
	if (!connected) {
		for (size_t i=0; i<table->ring().size(); i++) {
			const entry_t& e = table->ring()[i];
			assert(e.id != undefined_vid);
			if (e.own()) observer->up(e.id);
		}
		foreach( const entry_t* e, table->anycast().get_entries())
			if (e->own()) observer->up(e->id, node().getNodeId() );
	}
	connected = true;
}

/// called when a node is disconnected
void VirtualRing::onDisconnect(nodeid_t nid) {

	// remove invalid next hops from routing table
	table->dsdv().remove_neighbor(nid);

	// disable node
	if (node().getNeighbors().size()==0 && connected) {
		connected = false;
		for (size_t i=0; i<table->ring().size(); i++) {
			const entry_t& e = table->ring()[i];
			if (e.own()) observer->down(e.id);
		}
		foreach( const entry_t* e, table->anycast().get_entries())
			if (e->own()) observer->down(e->id, node().getNodeId() );
	}
}

/// on message
void VirtualRing::onMessage(NodePkt* pkt) {
	// handle timer and re-schedule
	if (pkt == timer) {
		onTimer();
		scheduleAt(simTime() + roundTime, timer);
		return;
	}

	// process packet from neighbors
	switch (pkt->getType()) {

	// process routes from neighbors
	case vroute_update: {
		VRoutePkt* vrp =
				dynamic_cast<VRoutePkt*> (pkt->getEncapsulatedPacket());
		routeUpdatesReceived += vrp->getUpdatesArraySize();
		controlTrafficIn += vrp->getByteLength();
		controlMessagesIn++;
		for (size_t i = 0; i < vrp->getUpdatesArraySize(); i++)
			table->process(pkt->getHop().from, vrp->getUpdates(i));
		delete pkt;
		if (immediateResponse) flushBacklog();
		break;
	}

	// route payload & notifications
	case vrdata_payload: {
		route(pkt);
		break;
	}

	}
}

void VirtualRing::flushBacklog() {
	const vector_set<nodeid_t>& neighbors = node().getNeighbors();

	for (size_t i=0; i<neighbors.size(); i++) {
		VRoutePkt* pkt = table->queue().get_updates( neighbors[i] );
		if (pkt == NULL) continue;
		if (pkt->getUpdatesArraySize()==0) {
			delete pkt;
			continue;
		}
		routeUpdatesSent += pkt->getUpdatesArraySize();
		controlTrafficOut += pkt->getByteLength();
		controlMessagesOut++;
		NodePkt* npkt = new NodePkt();
		npkt->setType( vroute_update );
		npkt->encapsulate(pkt);
		sendToNode( neighbors[i], npkt );
	}
}

void VirtualRing::route(NodePkt* pkt ) {
	VRDataPkt* vdp = dynamic_cast<VRDataPkt*> (pkt->getEncapsulatedPacket());

	/// get next hop table entry
	const entry_t* entry  = table->ring().next_hop(vdp->getDst());
	const entry_t* anycast_entry = table->anycast().next_hop(vdp->getDst());

	// prefer anycast entry if available
	if (anycast_entry != NULL) entry = anycast_entry;

	/// if next-hop not found (empty table) -> drop packet
	if (entry == NULL) {
		getVRObserver().messageDrop(pkt);
		delete pkt;
		return;
	}

	// increase hop-count
	int ttl = vdp->getTtl() - 1;
	vdp->setTtl(ttl);
	vdp->setHopcount( vdp->getHopcount() + 1 );

	/// route TTL zero or distance increasing? -> drop packet
	vid_t dist = distance(entry->id, vdp->getDst());
	if (ttl == 0 || (dist>vdp->getBest() && useRouteDescentCriterion)) {
		getVRObserver().messageDrop(pkt);
		delete pkt;
		return;
	}
	vdp->setBest( dist );

	/// reached destination-> account packet
	if (entry->own() && vdp->getDst() == entry->id ) {
		EV << "Node " << node().getNodeId()	<< ": received packet from " << vdp->getSrc() << endl;
		getVRObserver().messageDelivered(pkt, vdp->getDst(), vdp->getHopcount()-1);
		delete pkt;
		return;
	}

	/// packet is for myself, but I'm not the destination-> drop message!
	if (entry->own() && vdp->getDst() != entry->id ) {
		getVRObserver().messageDrop(pkt);
		delete pkt;
		return;
	}

	/// route packet to next node
	sendToNode(entry->next_hop, pkt);
}

/// sends a probe message
void VirtualRing::sendProbeMessage( bool anycast ) {
	if (!connected) return;

	// get random source / destination address
	vid_t src,dst;
	if (!anycast) {
		src = table->ring().random_id();
		do {dst=observer->random();} while(src==dst);
	} else {
		src = table->ring().random_id();
		do {dst=observer->random_anycast();} while(src==dst);
	}

	// send probe message
	NodePkt* probePkt = new NodePkt();
	probePkt->setType(vrdata_payload);
	VRDataPkt* dataPkt = new VRDataPkt();
	dataPkt->setSrc(src);
	dataPkt->setDst(dst);
	dataPkt->setBest( ~(vid_t)0 );
	dataPkt->setTtl(128);
	dataPkt->setHopcount(0);
	probePkt->encapsulate(dataPkt);
	getVRObserver().messageRegister(probePkt, anycast);
	route(probePkt);
}

// on timer message
void VirtualRing::onTimer() {

	// consistency check
	consistencyCheckCount++;
	if (consistencyCheckCount > (int)consistencyCheck && consistencyCheck!=0) {
		consistencyCheckCount = 0;
		table->dsdv().trigger_consistency_check();
	}

	// update DSDV routing-table
	table->dsdv().maintenance(timeout);

	// update virtual ring routing-table
	table->ring().maintenance();

	// update anycast announcements
	table->anycast().maintenance();

	// update stats
	if ( table->dsdv().table_altered() ) tableChanges++;
	if ( table->dsdv().seqno_changed() ) seqChanges++;
	if ( table->ring().vneighbors_changed() ) tableVNeighborChanges++;

	// reset flags
	table->ring().reset();
	table->dsdv().reset();

	flushBacklog();

	// start sending probe messages
	if (simTime()>sendMessageDelay) {
		sendProbeMessage(false);
		sendProbeMessage(true);
	}
}


void VirtualRing::reset() {

}

//---------------------------------------------------------- stats and vis ---

// convenience: record value per neighbor
#define STAT_PER_NEIGHBOR( NAME, VARNAME ) \
	stats.value( NAME, VARNAME ); \
	if (node().getNeighbors().size()!=0) \
		stats.value( NAME"PerNeighbor", \
		(double)VARNAME/(double)node().getNeighbors().size());

void VirtualRing::sampleStatistics( Statistics& stats ) {
	// check correct neighborhood
	boost::unordered_set<vid_t> *neighbors = table->ring().get_vneighbors();
	boost::unordered_set<vid_t> set;
	for (size_t i = 0; i < table->ring().size(); i++) {
		if (!table->ring()[i].own()) continue;
		observer->getNeighbors(set, table->ring()[i].id, 1);
	}
	bool neighbors_valid = (neighbors != NULL && set == *neighbors) || !connected;
	delete neighbors;

	// update visualization
	visualize(neighbors_valid);

	// flags
	connected = node().getNeighbors().size()!=0;
	stats.value( "generalConnected", 			(connected==true ? 1 : 0) );
	stats.value( "routingRingAccuracy", 		(neighbors_valid==true ? 1 : 0) );
	stats.value( "routingVNeighborChanges", 	tableVNeighborChanges );
	stats.value( "routingTableChanges", 		tableChanges );
	stats.value( "routingTableSeqChanges", 		seqChanges );

	// record routing table size
	int rtSize = table->dsdv().size();
	STAT_PER_NEIGHBOR("routingTableSize", rtSize);
	rtSize = table->dsdv().size( entry_t::RING );
	STAT_PER_NEIGHBOR("routingTableSizeRing", rtSize);
	rtSize = table->dsdv().size( entry_t::ANYCAST );
	STAT_PER_NEIGHBOR("routingTableSizeAnycast", rtSize);

	// add stats
	STAT_PER_NEIGHBOR("controlMessagesIn", 		controlMessagesIn);
	STAT_PER_NEIGHBOR("controlMessagesOut", 	controlMessagesOut);
	STAT_PER_NEIGHBOR("controlMessagesTotal", 	(controlMessagesOut+controlMessagesIn));
	STAT_PER_NEIGHBOR("controlMessagesDeltaIn",	(controlMessagesIn-controlMessagesInLast));
	STAT_PER_NEIGHBOR("controlMessagesDeltaOut", (controlMessagesOut-controlMessagesOutLast));
	STAT_PER_NEIGHBOR("controlMessagesDeltaTotal", 	(controlMessagesOut+controlMessagesIn-controlMessagesInLast-controlMessagesOutLast));
	STAT_PER_NEIGHBOR("controlTrafficIn", 		controlTrafficIn);
	STAT_PER_NEIGHBOR("controlTrafficOut", 		controlTrafficOut);
	STAT_PER_NEIGHBOR("controlTrafficTotal", 	(controlTrafficOut+controlTrafficIn));
	STAT_PER_NEIGHBOR("controlBandwidthIn", 	(controlTrafficIn-controlTrafficInLast) );
	STAT_PER_NEIGHBOR("controlBandwidthOut", 	(controlTrafficOut-controlTrafficOutLast));
	STAT_PER_NEIGHBOR("controlBandwidthTotal", 	(controlTrafficOut+controlTrafficIn-controlTrafficInLast-controlTrafficOutLast));
	STAT_PER_NEIGHBOR("controlUpdatesIn", 		routeUpdatesReceived);
	STAT_PER_NEIGHBOR("controlUpdatesOut", 		routeUpdatesSent);
	STAT_PER_NEIGHBOR("controlUpdatesTotal", 	(routeUpdatesSent+routeUpdatesReceived));

	controlMessagesInLast = controlMessagesIn;
	controlMessagesOutLast = controlMessagesOut;
	controlTrafficInLast = controlTrafficIn;
	controlTrafficOutLast = controlTrafficOut;
}


void VirtualRing::visualize( bool neighbors_valid ) {
	if (!getEnvir()->isGUI()) return;

	// output table to visualization
	ostringstream str;
	for (size_t i = 0; i < table->ring().size(); i++) {
		if (i != 0)
		str << ",";
		if (table->ring()[i].own()) str << "[";
		//		if (table->get(i).neighbor) str << "(";
		str << table->ring()[i].id;
		if (table->ring()[i].delegated) str << "*";
		//	str << "(" << table->get(i).dist << ")";
		// 	str << "/" << table->left_right(i);
		//if (table->ownid_of_vneighbor_by_index(i)!=-1)
		//	str << "/" << table->get(table->ownid_of_vneighbor_by_index(i)).id;
		if (table->ring()[i].own()) str << "]";
		//	if (table->get(i).neighbor) str << ")";
	}
	TEXT(s, str.str() );

	if (!connected) {
		node().getDisplayString().setTagArg("i", 1, "#ff0000");
	} else
	if (neighbors_valid)
		node().getDisplayString().setTagArg("i", 1, "#00ff00");
	else
		node().getDisplayString().setTagArg("i", 1, "");
}

} //namespace



