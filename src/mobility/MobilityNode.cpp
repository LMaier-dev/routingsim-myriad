//
// Created by s_schaut on 26.03.24.
//

#include "MobilityNode.h"

namespace routingsim {

MobilityNode::MobilityNode() {

}

void routingsim::MobilityNode::handleMessage(cMessage *msg) {
  Node::handleMessage(msg);
}
void routingsim::MobilityNode::initialize(int stage) { Node::initialize(); }

} // namespace routingsim