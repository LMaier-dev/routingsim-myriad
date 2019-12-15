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
#include "NetworkManager.h"

namespace routingsim {

Define_Module(Node);

/// if the network is created from a ned file, the variable gnodeid is incremented
/// by each new Node instance. To allow for reproducible IDs for NED files when the
/// network is rebuilt, we reset gnodeid to 1 in the destructor of Node.
/// (ATM dynamics in the network do not remove but only disable nodes. Therefore, this
/// is no problem)
static nodeid_t gnodeid=0;

Node* Node::getNode(cModule *mod) {
	if (!mod)
		return nullptr;
	
	cModule *submod = mod->getSubmodule("node");
	if (submod) {
		// we have a node submodule
		return check_and_cast<Node *>(submod);
	}
	if (mod->hasGate("toNode$o")) {
		// we have a protocol, so we follow the toNode gate
		return getOtherNode(mod->gate("toNode$o"));
	}
	// last we try to cast it
	return check_and_cast<Node *>(mod);
}

Node* Node::getOtherNode(cGate *gate) {
	ASSERT(gate->getType() == cGate::OUTPUT);
	return check_and_cast<Node *>(gate->getPathEndGate()->getOwnerModule());
}

void Node::initialize() {
	// we need to be sure that the network manager is also initialized for topologies
	// read from RoutingSim-unaware ned files
	// make sure that the network manager instance is generated at first node initialization
	getNetworkManager();

	WATCH( id );
	WATCH_VECTOR( neighbor_set );

	if (par("id").intValue() == -1) {
		this->id = gnodeid;
		gnodeid++;
	} else
		this->id = par("id").intValue();

	minDelay = par("nodeMinDelay").intValue();
	maxDelay = par("nodeMaxDelay").intValue();

	initNeighbors();
}

Node::~Node() { gnodeid = 1; }

void Node::doBuildInside() {
	// create node modules
	const char* nodeModules = par("protocols");
	cModuleType* protocolType = cModuleType::get(nodeModules);
	cModule* protocol = protocolType->create( "protocol0", this->getParentModule() );
	protocol->finalizeParameters();

	// connect gates
	cGate* in = NULL, *out = NULL;
	getOrCreateFirstUnconnectedGatePair("toProtocols", false, true, in, out );
	protocol->gate("toNode$o")->connectTo(in);
	out->connectTo( protocol->gate("toNode$i") );

	// We do not need to buildInside or scheduleStart, this is done
	// automatically since protocol is not a submodule.
}

/// Binds a module to the given packet type
void Node::bind( int type, cModule* mod ) {
	ASSERT( bindings.count(type) == 0);
	cGate* gate = mod->gate("toNode$i")->getPathStartGate();
	ASSERT( gate->getOwnerModule() == this );
	ASSERT( strcmp( gate->getName(), "toProtocols$o" ) == 0);
	bindings.insert( std::make_pair(type, gate->getIndex()) );
}

/// Unbind the module
void Node::unbind( int type, cModule* mod ) {
	assert( bindings[type]==mod->gate("toNode$i")->getPathStartGate()->getIndex() );
	bindings.erase(type);
}

///
void Node::setColor( const char *color ) {
	getDisplayString().setTagArg("i", 1, color);
	getParentModule()->getDisplayString().setTagArg("i", 1, color);
}

void Node::handleMessage(cMessage *msg) {
#ifdef NDEBUG
	NodePkt* np = reinterpret_cast<NodePkt*>(msg);
#else
	NodePkt* np = check_and_cast<NodePkt*>(msg);
#endif

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
		return;

		// goodbye
	case PAYLOAD_GOODBYE:
		onGoodbye( np );
		return;

		// invalid
	case PAYLOAD_INVALID:
		break;
	}

	// send to binding
	if (bindings.count(np->getType())!=0) {
		int gateIndex = bindings[np->getType()];
		send(np, "toProtocols$o", gateIndex);
	} else {
		EV_WARN << "Binding to packet type " << np->getType() << " unknown -- did you register your module to this packet type by calling bind()?";
		delete np;
	}
}


void Node::sendToNode( nodeid_t nid, NodePkt* np ) {
	// node not a neighbor ->
	if (neighbor_map.count(nid)==0) {
		EV_WARN << "sendToNode() @Node "<<getNodeId()<<": Destination node " << nid << " unknown, @msg id=" << np->getId() << endl;
		EV_TRACE << "Neighbors are: {";
		for (const auto& it : neighbor_map) {
			EV_TRACE << ' ' << (int)it.second.id;
		}
		EV_TRACE << " }" << endl;
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
	// schedule not before earlier sent message!
	if (simTime() < n.lastsent)
		delay+= n.lastsent-simTime();
	
	n.lastsent= simTime()+delay;
	sendDelayed(np, delay, n.out);
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
	//cout << "sendNotify from " << id << "to " << getOtherNode(dg)->id << endl;
	send(packet, dg);
}

void Node::initNeighbors() {
	NodePkt* np = new NodePkt();
	np->setType(PAYLOAD_HELLO);
	if (cSimulation::getActiveEnvir()->isGUI())
		np->setName("Link Up");
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
		neighbor_map.insert( std::make_pair( nid, neighbor_t() ) );
		neighbor_set.push_back( nid );
	};
	neighbor_t& nr = neighbor_map[nid];

	// set record
	nr.activity = simTime();
	nr.id  = nid;
	nr.out = gate("toNeighbors$o", pkt->getArrivalGate()->getIndex() );

	// announce at modules
	sendToModules(pkt);
}

void Node::onGoodbye( NodePkt* pkt ) {
	nodeid_t nid = pkt->getHop().from;
	if (neighbor_map.count(nid) == 0) return;

	// delete record
	neighbor_map.erase(nid);
	neighbor_set.erase(nid);

	// announce at neighbors
	sendToModules(pkt);
}

bool Node::disableLink(Node *mod, const char *gatename) {
	if (!hasGate(gatename))
		return false;

	for(int i=0; i<gateSize(gatename); i++) {
		if (mod == getOtherNode(gate(gatename, i))) {
			disableGate(gate(gatename, i));
			if (cSimulation::getActiveEnvir()->isGUI()) {
				cDisplayString ds = gate(gatename,i)->getDisplayString();
				ds.insertTag("ls");
				ds.setTagArg("ls",0, "red" );
				ds.setTagArg("ls", 1, 3);
				ds.setTagArg("ls",2,"s");
				gate(gatename,i)->setDisplayString(ds);
			}
			return true;
		}
	}
	return false;
}

bool Node::enableLink(Node *mod, const char *gatename) {
	if (!hasGate(gatename)) {
		EV_WARN << "Node " << getNodeId() << ": Gate " << gatename << " does not exist" << endl;
		return false;
	}

	for(int i=0; i<gateSize(gatename); i++) {
		if (mod == getOtherNode(gate(gatename, i))) {
			enableGate(gate(gatename, i));
			if (cSimulation::getActiveEnvir()->isGUI()) {
				cDisplayString ds = gate(gatename,i)->getDisplayString();
				ds.insertTag("ls");
				ds.setTagArg("ls",0, "black" );
				ds.setTagArg("ls",1, 1);
				ds.setTagArg("ls",2,"solid");
				gate(gatename,i)->setDisplayString(ds);
			}
			return true;
		}
	}
	EV_WARN << "Node " << getNodeId() << ": Could not find supposedly neighboring module with id: " << mod->getId() << endl;
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
	disableLinkPacket->setName("Link Down");
	hop_t disconnectNode;
	disconnectNode.from=id;
	disconnectNode.to=undefined_node;
	disableLinkPacket->setHop( disconnectNode );
	sendNotify(disableLinkPacket, dg);

	//add gate to filter list
	//EV << "disabling cGate * " << dg << endl;
	disabledLinks.push_back(dg);
	return;
}

void Node::enableGate(cGate *dg) {
	vector<cGate *>::iterator cit;

	//remove gate from filter list
	for(cit=disabledLinks.begin(); cit < disabledLinks.end(); cit++) {
		if (*cit == dg) {
			//EV << " enabling cGate * " << dg << endl;
			disabledLinks.erase(cit);
		}
	}

	// notify node that it is connected now
	NodePkt *enableLinkPacket = new NodePkt();
	enableLinkPacket->setType(PAYLOAD_HELLO);
	enableLinkPacket->setName("Link Up");
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
			//EV << "found gate in disabledLinks" << endl;
			return true;
		}
	}

	// gate not found in disabledLinks
	return false;
}

} //namespace
