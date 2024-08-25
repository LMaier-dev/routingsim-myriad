
//
// This module inherits from the Node module and provides the functionality of a mobile node.
// During initialisation, it creates a movement model and an antenna model that belong to itself.
// The movement of the node is controlled by the movement model, while the connections are determined by the antenna model.
// The movement model is realised by the MobilityBase module and the antenna model by the AntennaBase module.
// The scenario parameters determine which specific movement and antenna models are used.
// In addition, MobilityNode overrides the refreshDisplay() method, which visualises position and link updates of the nodes in the graphical user interface.
// Communication between the nodes takes place via the inherited Node module.
//

#ifndef ROUTINGSIM_MOBILITYNODE_H
#define ROUTINGSIM_MOBILITYNODE_H

#include "../foundation/Node.h"
#include "AntennaBase.h"
#include "IsotropicAntenna.h"
#include "MobilityBase.h"
#include "RandomWaypointMobility.h"

namespace routingsim {

    class MobilityNode : public Node {
    private:
        MobilityBase *mobility_model;
        AntennaBase *antenna_model;

    protected:
        friend class MobilityTopology;

    public:
        // initialises a mobility and antenna model for this MobilityNode
        void doBuildInside() override;
#
        // visualises movement and wireless events in the GUI
        void refreshDisplay() const override;
        MobilityBase *getMobilityModel() { return mobility_model; };
        AntennaBase *getAntennaModel() { return antenna_model; };
    };

}// namespace routingsim

#endif// ROUTINGSIM_MOBILITYNODE_H
