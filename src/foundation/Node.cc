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

#include "Node.h"

namespace routingsim {

Define_Module(Node);

static nodeid_t gnodeid=1;

void Node::initialize() {
	WATCH( id );
	WATCH_VECTOR( neighbor_set );

	if (par("id").longValue() == -1) {
		this->id = gnodeid;
		gnodeid++;
	} else
		this->id = par("id").longValue();

	minDelay = par("nodeMinDelay").longValue();
	maxDelay = par("nodeMaxDelay").longValue();

	initNeighbors();
}

void Node::doBuildInside() {
	// create node modules
	const char* nodeModules = par("protocols");
	cModuleType* type = cModuleType::get(nodeModules);
	Protocol* module =
			dynamic_cast<Protocol*>(type->create( "module0", this ));
	module->finalizeParameters();

	// connect gates
	cGate* in = NULL, *out = NULL;
	getOrCreateFirstUnconnectedGatePair("toProtocols", false, true, in, out );
	module->gate("toNode$o")->connectTo(in);
	out->connectTo( module->gate("toNode$i") );

	// create internals, and schedule it
	module->buildInside();
	module->scheduleStart(simTime());
}

/// Binds a module to the given packet type
void Node::bind( int type, Protocol* mod ) {
	assert( bindings.count(type) == 0);
	bindings.insert( make_pair(type,mod) );
}

/// Unbind the module
void Node::unbind( int type, Protocol* mod ) {
	assert( bindings[type]==mod );
	bindings.erase(type);
}

void Node::handleMessage(cMessage *msg) {
	NodePkt* np = check_and_cast<NodePkt*>(msg);

	// packet from Protocol? yes-> delegate!
	if (np->arrivedOn("toProtocols$i")) {
		np->getHop().from = this->id;
		if (np->getHop().to == neighbor_cast)
			sendToNeighbors(np);
		else
			sendToNode(np->getHop().to, np);
		return;
	}

	// interpret local package
	switch (np->getType()) {

	// hello
	case PAYLOAD_HELLO:
		onHello( np );
		delete np;
		return;

		// goodbye
	case PAYLOAD_GOODBYE:
		onGoodbye( np );
		delete np;
		return;

		// invalid
	case PAYLOAD_INVALID:
		break;
	}

	// send to binding
	if (bindings.count(np->getType())!=0) {
		cGate* g = bindings[np->getType()]->gate("toNode$i")->getPathStartGate();
		send(np,g);
	} else {
		EV << "Binding to " << np->getType() << " unknown ";
		delete np;
	}
}


void Node::sendToNode( nodeid_t nid, NodePkt* np ) {
	// node not a neighbor ->
	if (neighbor_map.count(nid)==0) {
		EV << "Node "<<getNodeId()<<": Node " << nid << " unknown" << endl;
		EV << "Neighbors are: " << endl;
		boost::unordered_map<nodeid_t, neighbor_t>::iterator it;
		for(it=neighbor_map.begin(); it!= neighbor_map.end(); it++) {
			EV << (int)it->second.id << endl;
		}
		delete np;
		return;
	}

	// set hop
	neighbor_t& n = neighbor_map[nid];
	np->setHop( hop_t(id, nid) );

	// if gate is disabled by networkManager, do not send
	if (gateIsDisabled(n.out)) {
		delete np;
		return;
	}

	// send packet with delay
	simtime_t delay = ((double)intuniform(minDelay,maxDelay))/1000.0;
	sendDelayed(np,delay, n.out);
}

void Node::sendToModules( NodePkt* np ) {
	int outGateBaseId = gateBaseId("toProtocols$o");
	int n = gate(outGateBaseId)->getVectorSize();
	for (int i=0; i<n; i++) {
		NodePkt* pkt = i==n-1 ? np : np->dup();
		send(pkt, outGateBaseId+i);
	}
}

void Node::sendToNeighbors( NodePkt* np ) {
	size_t n = neighbor_set.size();

	// if all neighbors are gone, free packet and return
	if (n == 0) {
		delete np;
		return;
	}
	for (size_t i=0; i<n; i++) {
		sendToNode(neighbor_set[i], i!=n-1 ? np->dup() : np );
	}
}

void Node::sendNotify(NodePkt *packet, cGate *dg) {
	Enter_Method_Silent("Node::sendNotify");
	take(packet);
	//cout << "sendNotify from " << id << "to " << dynamic_cast<Node *>(dg->getNextGate()->getOwnerModule())->id << endl;
	send(packet, dg);
}

void Node::initNeighbors() {
	NodePkt* np = new NodePkt();
	np->setType(PAYLOAD_HELLO);
	np->setHop( hop_t(id, neighbor_cast) );
	int outGateBaseId = gateBaseId("toNeighbors$o");
	int n = gate(outGateBaseId)->getVectorSize();
	for (int i=0; i<n; i++) {
		NodePkt* pkt = i==n-1 ? np : np->dup();
		send(pkt, outGateBaseId+i);
	}
}

void Node::onHello( NodePkt* pkt ) {
	nodeid_t nid = pkt->getHop().from;

	// add new neighbor data

	if (neighbor_map.count(nid) == 0) {
		neighbor_map.insert( make_pair( nid, neighbor_t() ) );
		neighbor_set.push_back( nid );
	};
	neighbor_t& nr = neighbor_map[nid];

	// set record
	nr.activity = simTime();
	nr.id  = nid;
	nr.out = gate("toNeighbors$o", pkt->getArrivalGate()->getIndex() );

	// announce at modules
	sendToModules(pkt->dup());
}

void Node::onGoodbye( NodePkt* pkt ) {
	nodeid_t nid = pkt->getHop().from;
	if (neighbor_map.count(nid) == 0) return;

	// delete record
	neighbor_map.erase(nid);
	neighbor_set.erase(nid);

	// announce at neighbors
	sendToModules(pkt->dup());
}

bool Node::disableLink(cModule *mod, const char *gatename) {
	if (!hasGate(gatename))
		return false;

	for(int i=0; i<gateSize(gatename); i++) {
		if (mod == gate(gatename, i)->getNextGate()->getOwnerModule()) {
			disableGate(gate(gatename, i));
			cDisplayString ds = gate(gatename,i)->getDisplayString();
			ds.insertTag("ls");
			ds.setTagArg("ls",0, "red" );
			ds.setTagArg("ls",2,"d");
			gate(gatename,i)->setDisplayString(ds);
			return true;
		}
	}
	return false;
}

bool Node::enableLink(cModule *mod, const char *gatename) {
	if (!hasGate(gatename)) {
		opp_warning("Node: %d: Gate %s does not exist", getNodeId(), gatename);
		return false;
	}

	for(int i=0; i<gateSize(gatename); i++) {
		if (mod == gate(gatename, i)->getNextGate()->getOwnerModule()) {
			enableGate(gate(gatename, i));
			cDisplayString ds = gate(gatename,i)->getDisplayString();
			ds.insertTag("ls");
			ds.setTagArg("ls",0, "black" );
			ds.setTagArg("ls",2,"solid");
			gate(gatename,i)->setDisplayString(ds);
			return true;
		}
	}
	opp_warning("Node %d: Could not find supposedly neighboring module with id: %d", getNodeId(), mod->getId());
	return false;

}

void Node::disableGate(cGate *dg) {
	vector<cGate *>::iterator cit;

	// verify that gate is not already disabled
	for(cit=disabledLinks.begin(); cit < disabledLinks.end(); cit++) {
		if (*cit == dg)
			return;
	}

	//create disconnect packet before filter is set
	NodePkt *disableLinkPacket = new NodePkt();
	disableLinkPacket->setType(PAYLOAD_GOODBYE);
	hop_t disconnectNode;
	disconnectNode.from=id;
	disconnectNode.to=undefined_node;
	disableLinkPacket->setHop( disconnectNode );
	sendNotify(disableLinkPacket, dg);

	//add gate to filter list
	//ev << "disabling cGate * " << dg << endl;
	disabledLinks.push_back(dg);
	return;
}

void Node::enableGate(cGate *dg) {
	vector<cGate *>::iterator cit;

	//remove gate from filter list
	for(cit=disabledLinks.begin(); cit < disabledLinks.end(); cit++) {
		if (*cit == dg) {
			//ev << " enabling cGate * " << dg << endl;
			disabledLinks.erase(cit);
		}
	}

	// notify node that it is connected now
	NodePkt *enableLinkPacket = new NodePkt();
	enableLinkPacket->setType(PAYLOAD_HELLO);
	hop_t connectNode;
	connectNode.from=id;
	connectNode.to=undefined_node;
	enableLinkPacket->setHop( connectNode );
	sendNotify(enableLinkPacket, dg);

	return;
}

bool Node::gateIsDisabled(cGate *g) {
	vector<cGate *>::iterator cit;
	for(cit=disabledLinks.begin(); cit < disabledLinks.end(); cit++) {
		if (*cit == g) {
			//ev << "found gate in disabledLinks" << endl;
			return true;
		}
	}

	// gate not found in disabledLinks
	return false;
}

} //namespace
