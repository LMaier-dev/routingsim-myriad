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

#ifndef __ROUTINGSIM_NETWORKDYNAMICS_H_
#define __ROUTINGSIM_NETWORKDYNAMICS_H_

#include <omnetpp.h>
#include <iostream>

// forward declarations
namespace routingsim {
class NetworkDynamics;
}

#include "NetworkTopology.h"
#include "NetworkManager.h"

namespace routingsim {

/**
 * This class generates dynamics in the network topology.
 * Node Failures handle automatically if we are dealing with
 * directed or undirected graphs and deactivates all attached links.
 *
 * Edge Failures need to explicitly state whether a link to be removed
 * is directed or not in the dynamicsInfo passed to the networkManager
 *
 * @author Christoph Werle <werle@kit.edu>
 */
class NetworkDynamics : public cSimpleModule {
  friend class NetworkManager;

private:
  cMessage *timer;
  NetworkTopology *topology;
  NetworkManager *manager;
  virtual void handleMessage(cMessage *msg);
  virtual void initialize(int stage);
  virtual void finish();
  virtual int numInitStages() const;

protected:
  /**
   * Initialize module.
   */
  virtual void initialize();

  /**
   * Sub-classes need to implement this function to schedule dynamic events.
   * Must return the sim time when it should be called again.
   */
  virtual simtime_t scheduleDynamics() = 0;

  /// Schedules a node failure at time occurence with given duration
  /// Message is sent to networkManager, which processes the event
  /// in the handleMessage()-method.
  /// Effect:
  /// In the topology management, the node as well as all attached links
  /// (in both directions) are disabled
  /// The node will not be able to send/receive messages any more and is
  /// sent a reset, which enables the node to (voluntarily) forget some state
  void scheduleNodeFailure(nodeid_t node, simtime_t occurence,
                           simtime_t duration);

  /// schedules a (unidirectional) link failure of the link connecting
  /// (from/to). if the "full" link is supposed to fail, set bidir to true.
  void scheduleLinkFailure(nodeid_t from, nodeid_t to, simtime_t occurence,
                           simtime_t duration, bool bidir = true);

public:
  inline NetworkTopology &getTopology() { return *topology; }
  inline NetworkManager &getManager() { return *manager; }
  virtual ~NetworkDynamics();
};

/**
 * Dynamic information handled by the network manager
 */
class DynamicsInfo {
public:
  // dynamic types
  enum type_t {
    nodeFailure,
    linkFailure,
    nodeRecovery,
    linkRecovery,
    undefined = 0xff
  };
  type_t type;

  // affected nodes
  nodeid_t source, destination;

  // Does a link failure between nodes a and b disable both directed links
  // from a->b and the reverse direction b->a
  bool bidirectionalLinkFailure;

  // occurence and duration
  simtime_t occurrence, duration;

  DynamicsInfo()
      : type(undefined), source(undefined_node), destination(undefined_node),
        bidirectionalLinkFailure(true), occurrence(0), duration(0) {}
  DynamicsInfo(const DynamicsInfo &other)
      : type(other.type), source(other.source), destination(other.destination),
        bidirectionalLinkFailure(other.bidirectionalLinkFailure),
        occurrence(other.occurrence), duration(other.duration) {}
};

} // namespace routingsim

#endif
