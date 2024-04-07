
#ifndef ROUTINGSIM_MOBILITYNODE_H
#define ROUTINGSIM_MOBILITYNODE_H

#include "../foundation/Node.h"
#include "MobilityBase.h"
#include "AntennaBase.h"
#include "MobilityTopology.h"

namespace routingsim {

class MobilityNode : public Node {
private:
  MobilityBase *mobility_model;
  AntennaBase *antenna_model;

protected:
  friend class MobilityTopology;
  void handleMessage(cMessage *msg) override;

public:
  MobilityNode();
  void initialize(int stage) override;
  MobilityBase *getMobilityModel() { return mobility_model; };
  AntennaBase *getAntennaModel() { return antenna_model; };
};

} // namespace routingsim

#endif // ROUTINGSIM_MOBILITYNODE_H
