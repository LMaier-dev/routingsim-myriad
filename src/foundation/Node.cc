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
#include "NodePkt_m.h"
#include "simtime.h"
#include "cstringtokenizer.h"
#include <version>
#if __cpp_lib_format
#include <format> // supported since g++-13 only
#else
#include <sstream>
#endif

namespace routingsim {

Define_Module(Node);

/// if the network is created from a ned file, the variable gnodeid is
/// incremented by each new Node instance. To allow for reproducible IDs for NED
/// files when the network is rebuilt, we reset gnodeid to 1 in the destructor
/// of Node. (ATM dynamics in the network do not remove but only disable nodes.
/// Therefore, this is no problem)
static nodeid_t gnodeid = 0;

Node *Node::getNode(cModule *mod) {
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


// get node at other end of link where the gate leads to
Node *Node::getOtherNode(cGate *gate) {
  ASSERT(gate->getType() == cGate::OUTPUT);
  return check_and_cast<Node *>(gate->getPathEndGate()->getOwnerModule());
}


void Node::initialize(int stage) {
  // we need to be sure that the network manager is also initialized for
  // topologies read from RoutingSim-unaware ned files make sure that the
  // network manager instance is generated at first node initialization
  if (stage == 0)
  {
	  getNetworkManager();
	  WATCH(id);
	  WATCH_VECTOR(neighbor_set);

          if (par("id").intValue() == -1) {
		  this->id = gnodeid;
		  gnodeid++;
	  }
	  else
		  this->id = par("id").intValue();

          minNodeControlPlaneDelay = par("nodeMinControlPlaneDelay").intValue(); // nodeCPMinDelay is given in ns
	  maxNodeControlPlaneDelay = par("nodeMaxControlPlaneDelay").intValue(); // nodeCPMaxDelay is given in ns
          minNodeDataPlaneDelay = par("nodeMinDataPlaneDelay").intValue(); // nodeDPMinDelay is given in ns
	  maxNodeDataPlaneDelay = par("nodeMaxDataPlaneDelay").intValue(); // nodeDPMaxDelay is given in ns

          initNeighbors();
  }
  else {
	  // create the protocol module dynamically
      auto nodeModules = cStringTokenizer(par("protocols")).asVector();
      auto i = 0;
      for (auto& module: nodeModules) {
	    cModuleType* protocolType = cModuleType::get(module.c_str());
#if __cpp_lib_format
	    cModule* protocol = protocolType->create( std::format("protocol{}", i).c_str(), this->getParentModule() );
#else
	    std::stringstream protocolstr("protocol");
	    protocolstr << i;
	    cModule* protocol = protocolType->create(protocolstr.str().c_str(),this->getParentModule());
#endif
            protocol->finalizeParameters();

          // connect gates
	    cGate* in = NULL, *out = NULL;
	    getOrCreateFirstUnconnectedGatePair("toProtocols", false, true, in, out );
	    protocol->gate("toNode$o")->connectTo(in);
	    out->connectTo( protocol->gate("toNode$i") );

        i++;
      }
  } // end else
}

Node::~Node() { gnodeid = 1; }

void Node::doBuildInside() {
  // We do not need to buildInside or scheduleStart, this is done
  // automatically since protocol is not a submodule.
}

/// Binds a module to the given packet type
void Node::bind(int type, cModule *mod) {
  ASSERT(bindings.count(type) == 0);
  cGate *gate = mod->gate("toNode$i")->getPathStartGate();
  ASSERT(gate->getOwnerModule() == this);
  ASSERT(strcmp(gate->getName(), "toProtocols$o") == 0);
  bindings.insert(std::make_pair(type, gate->getIndex()));
}

/// Unbind the module
void Node::unbind(int type, cModule *mod) {
  assert(bindings[type] ==
         mod->gate("toNode$i")->getPathStartGate()->getIndex());
  bindings.erase(type);
}

///
void Node::setColor(const char *color) {
  getDisplayString().setTagArg("i", 1, color);
  getParentModule()->getDisplayString().setTagArg("i", 1, color);
}

void Node::handleMessage(cMessage *msg) {
#ifdef NDEBUG
  NodePkt *np = reinterpret_cast<NodePkt *>(msg);
#else
  NodePkt *np = check_and_cast<NodePkt *>(msg);
#endif

  // packet from Protocol and not to ourself? yes-> delegate!
  if (np->arrivedOn("toProtocols$i") and np->getHop().to != this->id) {
    np->getHopForUpdate().from = this->id;

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
    onHello(np);
    return;

    // goodbye
  case PAYLOAD_GOODBYE:
    onGoodbye(np);
    return;

    // invalid
  case PAYLOAD_INVALID:
    break;
  }


  // send to binding
  if (bindings.count(np->getType()) != 0) {
    int gateIndex = bindings[np->getType()];
    send(np, "toProtocols$o", gateIndex);

  } else {
    EV_WARN << "Binding to packet type " << np->getType()
            << " unknown -- did you register your module to this packet type "
               "by calling bind()?";
    delete np;
  }
}

void Node::sendToNode(nodeid_t nid, NodePkt *np) {
  // node not a neighbor -> drop packet
  if (neighbor_map.contains(nid) == false) {
    EV_WARN << "sendToNode() @Node " << getNodeId() << ": Destination node "
            << nid << " unknown, @msg id=" << np->getId() << endl;
#ifndef NDEBUG
    EV_TRACE << "Neighbors are: {";
    for (const auto &it : neighbor_map) {
      EV_TRACE << ' ' << (int)it.second.id;
    }
    EV_TRACE << " }" << endl;
#endif
    delete np;
    return;
  }

  // set hop
  neighbor_t &n = neighbor_map[nid];
  np->setHop(hop_t(id, nid));

  // if gate is disabled by networkManager, do not send
  if (gateIsDisabled(n.out) || !n.out->isConnected()) {
    delete np;
    return;
  }

  // send packet with randomized node processing delay (use ns precision)
  simtime_t node_processing_delay = SIMTIME_ZERO;
  if (maxNodeControlPlaneDelay > 0 and (np->getAttributes() & NODEPKT_ATTR_CONTROL_PLANE_PROCESSING_DELAY))
          node_processing_delay = SimTime(intuniform(minNodeControlPlaneDelay, maxNodeControlPlaneDelay), SIMTIME_NS);
  // add data plane processing delay if requested
  if (maxNodeDataPlaneDelay > 0 and (np->getAttributes() & NODEPKT_ATTR_DATA_PLANE_PROCESSING_DELAY))
	  node_processing_delay += SimTime(intuniform(minNodeDataPlaneDelay, maxNodeDataPlaneDelay), SIMTIME_NS);

  // schedule not before earlier sent message to avoid message reordering!
  if (simTime() + node_processing_delay < n.lastsent) {
    node_processing_delay = n.lastsent - simTime();
  }

  // schedule not before transmission of other message on channel
  cChannel *txChannel = n.out->findTransmissionChannel();
  if (txChannel) {
    simtime_t txFinishTime = txChannel->getTransmissionFinishTime();
    if (simTime() + node_processing_delay < txFinishTime) {
      node_processing_delay = txFinishTime - simTime();
    }
  }

  if (not(np->getAttributes() & NODEPKT_ATTR_DO_NOT_COUNT_ON_LINK)) {
	  linkusage_map[nid].msgcount++;
          linkusage_map[nid].bytecount += np->getByteLength();
  }
  // EV_TRACE << "sendToNode() @Node " << getNodeId() << ": Destination node "
  //          << nid << " msg:" << np->getName() << " " << n.out->getNextGate()->getFullPath() << endl;
  sendDelayed(np, node_processing_delay, n.out);
  if (txChannel) {
    n.lastsent = simTime() + node_processing_delay + txChannel->calculateDuration(np);
  }
  else {
    n.lastsent = simTime() + node_processing_delay;
  }
}


void Node::sendToModules(NodePkt *np) {
  int outGateBaseId = gateBaseId("toProtocols$o");
  int n = gate(outGateBaseId)->getVectorSize();
  for (int i = 0; i < n; i++) {
    NodePkt *pkt = i == n - 1 ? np : np->dup();
    send(pkt, outGateBaseId + i);
  }
  if (n == 0) delete np;
}

void Node::sendToNeighbors(NodePkt *np) {
  size_t n = neighbor_set.size();
  for (size_t i = 0; i < n; i++) {
    sendToNode(neighbor_set[i], i != n - 1 ? np->dup() : np);
  }
  // if all neighbors are gone, free packet and return
  if (n == 0) {
    delete np;
  }
}


void Node::sendNotify(NodePkt *packet, cGate *dg) {
  Enter_Method_Silent("Node::sendNotify");
  take(packet);
  // EV_TRACE << "SendNotify: " << "@node "
  //          << id << " -> " << getOtherNode(dg)->id
  // 	   << " from outgate " << dg->getFullName() << " to ingate "
  //          << dg->getPathEndGate()->getFullName() << endl;
  // this packet contains the incoming gate for the link as parameter
  packet->setDestGateId(dg->getPathEndGate()->getIndex());
  sendDirect(packet, getOtherNode(dg), "directIn");
}


void Node::initNeighbors() {
  if (!hasGate("toNeighbors$o", 0)) return;
  NodePkt *np = new NodePkt();
  np->setType(PAYLOAD_HELLO);
  if (cSimulation::getActiveEnvir()->isGUI())
    np->setName("Link Up");
  np->setHop(hop_t(id, neighbor_cast));
  int outGateBaseId = gateBaseId("toNeighbors$o");
  int n = gate(outGateBaseId)->getVectorSize();
  for (int i = 0; i < n; i++) {
    NodePkt *pkt = i == n - 1 ? np : np->dup();
    send(pkt, outGateBaseId + i);
  }
}


void Node::onHello(NodePkt *pkt) {
  nodeid_t nid = pkt->getHop().from;

  // add new neighbor data
  if (neighbor_map.contains(nid) == false) {
    neighbor_map.insert(std::make_pair(nid, neighbor_t()));
    neighbor_set.push_back(nid);
  };
  neighbor_t &nr = neighbor_map[nid];

  // set record
  nr.activity = simTime();
  nr.lastsent = 0;
  nr.id = nid;
  if (pkt->getDestGateId() == -1) {
    nr.out = gate("toNeighbors$o", pkt->getArrivalGate()->getIndex());
    // EV_TRACE << "onHello: Node " << id << " received hello from " << nid
    //  	<< " on gate " << pkt->getArrivalGate()->getIndex() << endl;
  } else { // outgate is given explicitly in NodePkt
    nr.out = gate("toNeighbors$o", pkt->getDestGateId());
    // EV_TRACE << "onHello: Node " << id << " received hello from " << nid
    //  	<< " on directIn gate, outgate " << pkt->getDestGateId() << endl;
  }

  // announce at modules
  sendToModules(pkt);
}

void Node::onGoodbye(NodePkt *pkt) {
  nodeid_t nid = pkt->getHop().from;
  if (neighbor_map.count(nid) == 0)
    return;

  // delete record
  neighbor_map.erase(nid);
  neighbor_set.erase(nid);

  // announce at neighbors
  sendToModules(pkt);
}


bool Node::disableLink(const Node *othernode_mod, const char *gatename) {
  if (!hasGate(gatename))
    return false;

  for (int i = 0; i < gateSize(gatename); i++) {
    if (othernode_mod == getOtherNode(gate(gatename, i))) {
      disableGate(gate(gatename, i));
      if (cSimulation::getActiveEnvir()->isGUI()) {
        cDisplayString ds = gate(gatename, i)->getDisplayString();
        ds.insertTag("ls");
        ds.setTagArg("ls", 0, "red");
        ds.setTagArg("ls", 1, 3);
        ds.setTagArg("ls", 2, "s");
        gate(gatename, i)->setDisplayString(ds);
      }
      return true;
    }
  }
  return false;
}


bool Node::enableLink(const Node *othernode_mod, const char *gatename) {
  if (!hasGate(gatename)) {
    EV_WARN << "Node " << getNodeId() << ": Gate " << gatename
            << " does not exist" << endl;
    return false;
  }

  // traverse outgoing gates to neighbors and look for other node's module
  for (int i = 0; i < gateSize(gatename); i++) {
    if (othernode_mod == getOtherNode(gate(gatename, i))) {
      // enableGate sends a HELLO to the neighbor to indicate that the link is up
      enableGate(gate(gatename, i));
      if (cSimulation::getActiveEnvir()->isGUI()) {
        cDisplayString ds = gate(gatename, i)->getDisplayString();
        ds.insertTag("ls");
        ds.setTagArg("ls", 0, "black");
        ds.setTagArg("ls", 1, 1);
        ds.setTagArg("ls", 2, "solid");
        gate(gatename, i)->setDisplayString(ds);
      }
      return true;
    }
  }
  EV_WARN << "Node " << getNodeId()
          << ": Could not find supposedly neighboring module with id: "
          << othernode_mod->getId() << endl;
  return false;
}


// disables gate and notifies
void
Node::disableGate(cGate *dg) {
  // verify that gate is not already disabled
  if (gateIsDisabled(dg))
    return;

  // create disconnect packet before filter is set
  NodePkt *disableLinkPacket = new NodePkt();
  disableLinkPacket->setType(PAYLOAD_GOODBYE);
  disableLinkPacket->setName("Link Down");
  hop_t disconnectNode;
  disconnectNode.from = id;
  disconnectNode.to = undefined_node;
  disableLinkPacket->setHop(disconnectNode);
  sendNotify(disableLinkPacket, dg);

  // add gate to filter list
  // EV << "disabling cGate * " << dg << endl;
  disabledLinks.push_back(dg);
}


void
Node::enableGate(cGate *dg) {
  // remove gate from filter list
  if (vector<cGate *>::iterator cit =
          std::find(disabledLinks.begin(), disabledLinks.end(), dg);
      cit != disabledLinks.end()) {
    disabledLinks.erase(cit);
    // EV << " enabling cGate * " << dg << endl;
  }

  // notify node that it is connected now
  NodePkt *enableLinkPacket = new NodePkt();
  enableLinkPacket->setType(PAYLOAD_HELLO);
  enableLinkPacket->setName("Link Up");
  hop_t connectNode;
  connectNode.from = id;
  connectNode.to = undefined_node;
  enableLinkPacket->setHop(connectNode);
  sendNotify(enableLinkPacket, dg);

  return;
}


bool
Node::gateIsDisabled(const cGate *g) const {
  if (std::find(disabledLinks.begin(), disabledLinks.end(), g) !=
      disabledLinks.end()) {
    return true;
    // EV << "found gate in disabledLinks" << endl;
  }

  return false; // gate not found in disabledLinks
}

bool Node::setLinkDatarate(Node *mod, const char *gatename, double datarate) {
  cGate *gate = findGateByName(mod, gatename);
  if (gate) {
    cChannel *channel = gate->getTransmissionChannel();
    if (!channel) {
      EV_WARN << "Node " << getNodeId() << ": Could not find channel for gate "
              << gatename << endl;
      return false;
    }
    cDatarateChannel *datarateChannel =
        dynamic_cast<cDatarateChannel *>(channel);
    if (!datarateChannel) {
      EV_WARN << "Node " << getNodeId()
              << ": Channel is not a datarate channel for gate " << gatename
              << endl;
      return false;
    }
    datarateChannel->setDatarate(datarate);
    return true;
  }
  return false;
}

cGate *Node::findGateByName(Node *mod, const char *gatename) {
  for (int i = 0; i < gateSize(gatename); i++) {
    if (mod == getOtherNode(gate(gatename, i))) {
      return gate(gatename, i);
    }
  }
  return nullptr;
}

} // namespace routingsim
