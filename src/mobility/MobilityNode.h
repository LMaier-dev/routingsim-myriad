//
// Created by s_schaut on 26.03.24.
//

#ifndef ROUTINGSIM_MOBILITYNODE_H
#define ROUTINGSIM_MOBILITYNODE_H

#include "../foundation/Node.h"
#include "MobilityBase.h"
#include "AntennaBase.h"

namespace routingsim {

class MobilityNode : public Node {

private:
  MobilityBase *mobility_model;
  AntennaBase *antenna_model;

protected:
  void handleMessage(cMessage *msg) override;

public:
  MobilityNode();
  void initialize(int stage) override;
};

} // namespace routingsim

#endif // ROUTINGSIM_MOBILITYNODE_H
