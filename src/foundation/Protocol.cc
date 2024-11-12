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

#include "Protocol.h"
#include "NodePkt_m.h"
#include "ProtocolPacketQueue.h"
#include "checkandcast.h"
#include "clog.h"
#include "csimulation.h"
#include <memory>

namespace routingsim {

//Define_Module(Protocol);
Register_Abstract_Class(Protocol);

void Protocol::initialize(int stage) {

	// Protocol entity is created at initialization stage 1
	if (stage == 1) {
		parent = Node::getOtherNode(gate("toNode$o"));
		ASSERT(parent != nullptr);
		this->onInitialize();
	}
}


Protocol::~Protocol() {
	// cancel any timer msgs that serve the packet queues
	for (auto it = protocolQueues.begin(); it != protocolQueues.end(); it++) {

		if (auto ppq= it->second.get(); ppq and ppq->isScheduled()) {
			cancelEvent(ppq->getTimerMsg());
			ppq->setTimerMsg(nullptr);
		}

	}
}


void Protocol::handleMessage(cMessage *msg) {

	// self message is either internal ProtocolTimerMsg or other protocol specific
	if (msg->isSelfMessage()) {
		if (auto pt = dynamic_cast<ProtocolTimerMsg *>(msg); pt) {
			// if this is a protocol timer message: get next message from the queue
			auto ppq= reinterpret_cast<ProtocolPacketQueue*>(pt->getContextPointer());
                        ASSERT(ppq != nullptr);
                        auto [ts, np] = ppq->popPacket();
			//EV_TRACE << "Packet was enqueued @" << ts << ", waited " << simTime()-ts << "s " << endl;
                        if (np) { // got next packet to serve
                                // call function in protocol
                                onMessage(std::move(np));
				auto finishingTime = simTime() + ppq->getProcessingTime();
                                ppq->setFinishingTime(finishingTime);
				//EV_TRACE << "packet processing will be finished at " << finishingTime << endl;
                                if (ppq->empty()) { // queue empty now, will not schedule ProtocolTimerMsg
					ppq->setScheduled(false);
				}
				else {
					// calculate processing time for this message and start timer for popping next packet
					scheduleAt(finishingTime, pt);
                                        ppq->setScheduled(true);
				}
                        }
                        else { // queue is empty now
				ASSERT(ppq->empty());
                                // unset scheduled state, self message is kept
				ppq->setScheduled(false);
                        }
		}
		else { // other self message, just propagate
                        onSelfMessage(std::unique_ptr<cMessage>(msg));
		}
		return;
	} // endif if selfMsg

#ifdef NDEBUG
	NodePkt* np = reinterpret_cast<NodePkt*>(msg);
#else
	NodePkt* np = check_and_cast<NodePkt*>(msg);
#endif
	switch( np->getType() ) {
	case PAYLOAD_HELLO:
		this->onConnect(np->getHop().from);
		delete np;
		return;
	case PAYLOAD_GOODBYE:
		this->onDisconnect(np->getHop().from);
		delete np;
		return;
	default:
		break;
        }

	if (protocolQueues.empty()) { // direct callback if processing delay is not modelled
		onMessage(std::unique_ptr<NodePkt>(np));
		return;
	}

        // check whether NodePkt needs to be enqueued for protocol
        if (auto it = protocolQueues.find(np->getType());
            it != protocolQueues.cend()) {
		// add packet to queue if necessary
                if (auto ppq = it->second.get(); ppq) {

			if (ppq->isBusy()) { // waiting packets present or packet currently being served
				// add this packet to the queue
				auto added = ppq->addPacket(std::unique_ptr<NodePkt>(np));
                                if (not added) // if discarded just return
					return;

                                if (not ppq->isScheduled()) { // no timer scheduled yet
					auto timermsg = ppq->getTimerMsg();
					if (timermsg == nullptr) { // otherwise reuse existing timer msg
						// need to create new timer message
                                                ASSERT(ppq->getTimerMsg() == nullptr);
						timermsg= new ProtocolTimerMsg(0, ppq);
                                                ppq->setTimerMsg(timermsg);
                                        }
					scheduleAt(ppq->getFinishingTime(),timermsg);
					ppq->setScheduled(true);
                                }
                        }
                        else { // queue is empty and no packet being served, so start serving immediately
				// set finishing time of this packet
				auto finishingTime = simTime() + ppq->getProcessingTime();
                                ppq->setFinishingTime(finishingTime);
				//EV_TRACE << "packet processing will be finished at " << finishingTime << endl;
				// start serving this packet immediately
                                onMessage(std::unique_ptr<NodePkt>(np));
				return;
			}
                }
        }
	else {
		onMessage(std::unique_ptr<NodePkt>(np));
	}
}


/// Bind module to node to receive messages of the given type
void Protocol::bind(int type, std::unique_ptr<ProtocolPacketQueue> pq) {
        if (pq) {
		protocolQueues[type]= std::move(pq);
	}
	node().bind(type, this);
}


/// Return ProtocolPacketQueue for given type
const ProtocolPacketQueue *
Protocol::getBoundProtocolPacketQueue(int type) const {
  if (auto it = protocolQueues.find(type); it != protocolQueues.cend()) {
	  return it->second ? it->second.get() : nullptr;
  }
  else {
	  return nullptr;
  }
}


/// Unbind module to node to receive messages of the given type no more
void Protocol::unbind(int type) {
	node().unbind(type, this);
	protocolQueues.erase(type);
}


/// sends a packet
void Protocol::sendToNode( nodeid_t nid, NodePkt* pkt, simtime_t delay) {
	pkt->getHopForUpdate().to = nid;

	if (delay == 0)
		cSimpleModule::send(pkt, "toNode$o");
	else
		cSimpleModule::sendDelayed(pkt, delay, "toNode$o");
}

/// sends a packet
void Protocol::sendLocal(NodePkt* pkt, simtime_t delay) {
	pkt->getHopForUpdate().to = node().getNodeId();
	pkt->getHopForUpdate().from = node().getNodeId();

	if (delay == 0)
		cSimpleModule::send(pkt, "toNode$o");
	else
		cSimpleModule::sendDelayed(pkt, delay, "toNode$o");
}


/// sends a packet
void Protocol::sendToNeighbors( NodePkt* pkt) {
	pkt->getHopForUpdate().to = neighbor_cast;
	cSimpleModule::send(pkt, "toNode$o");
}

} // namespace
