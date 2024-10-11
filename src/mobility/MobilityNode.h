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

#include "Node.h"
#include "AntennaBase.h"
#include "MobilityBase.h"


namespace routingsim {

class MobilityNode : public Node {
    private:
        MobilityBase *mobility_model;
        AntennaBase *antenna_model;

    protected:
        friend class MobilityTopology;

    public:
	// initialises a mobility and antenna model for this MobilityNode
	virtual void initialize(int stage) override;
	virtual int numInitStages() const override { return 2; };
        void doBuildInside() override;
        void preDelete(cComponent *root) override {
          mobility_model = nullptr;
	  antenna_model = nullptr;
	};

        // visualises movement and wireless events in the GUI
        void refreshDisplay() const override;
        MobilityBase *getMobilityModel() { return mobility_model; };
        AntennaBase *getAntennaModel() { return antenna_model; };
};

}// namespace routingsim

#endif// ROUTINGSIM_MOBILITYNODE_H
