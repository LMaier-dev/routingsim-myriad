
//
// This class inherits from the NetworkTypology class and is responsible for the creation and management of mobile nodes and their edges.
// Like NetworkTypology, MobilityTopology is only a simple class and therefore not a OMNeT++ module.
// The advantage of this independence is that the overhead is reduced, as MobilityTopology is more of an auxiliary class for NetworkManager.
// To create the topology, the NetworkManager calls the createTopology() method of this class.
// It reads the number of nodes to be created, creates a corresponding number of MobilityNodeWrapper modules and passes them to the NetworkManager.
// The resulting topology is also saved in the boost-graph library used by NetworkTopology so that the routing algorithms based on it can be executed correctly.
// It is possible to manipulate the topology using the two methods createLink(nodeid\_t, nodeid\_t), deleteLink(nodeid\_t, nodeid\_t) and setDatarate(nodeid\_t, nodeid\_t, double).
// The topology updates are also saved in the \emph{boost-graph}.
// These methods are called by MobilityDynamics to update the connections between the nodes.
//

#ifndef ROUTINGSIM_MOBILITYMANAGER_H
#define ROUTINGSIM_MOBILITYMANAGER_H

#include "MobilityNode.h"
#include "../foundation/NetworkTopology.h"

namespace routingsim {

class MobilityTopology: public NetworkTopology {

public:
  MobilityTopology(){ mobilityNodeMods= new std::vector<cModule *>(); };
  inline ~MobilityTopology();
  void initialize();
  /**
   * @param rng The random number generator to use for mobility.
   */
  explicit MobilityTopology(cModule *networkManager);

  // set the given datarate in byte per second on the link between the given nodes
  void setDatarate(MobilityNode *fromNode, MobilityNode *toNode, double datarate);

  // creates a link between two nodes
  void createLink(MobilityNode *fromNode, MobilityNode *toNode);

  // deletes a link between two nodes
  void deleteLink(MobilityNode *fromNode, MobilityNode *toNode);

  // Get the output gate from fromNode, that connects to toNode
  cGate *getGateBetweenNodes(MobilityNode *fromNode, MobilityNode *toNode);

  // Get the datarate in byte per second on the link between the given nodes
  double getDatarate(MobilityNode *fromNode, MobilityNode *toNode);

  // Get all MobilityNodeWrappers
  std::vector<cModule*> getNodeMods() const { return *mobilityNodeMods; }

  // initial start the topology creation
  void createTopology();

private:
  cModule *networkManager= nullptr;
  std::vector<cModule*> *mobilityNodeMods= nullptr;
  int numNodes = 0;
};


inline
MobilityTopology::~MobilityTopology() {
    // for (auto nodeMod : *mobilityNodeMods) {
    //   nodeMod->deleteModule();
    // }
    delete mobilityNodeMods;
    // delete BGLtopology;
}

} // namespace routingsim

#endif // ROUTINGSIM_MOBILITYMANAGER_H
