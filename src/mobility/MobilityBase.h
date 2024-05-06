//
// This class extends the Environment class and adds mobility features.
// It is responsible for managing the mobility of the nodes in the simulation.
// The class defines the basic mobility features of a node, such as its position, velocity, and heading.
// Furthermore, it provides methods to update the position of the node and reflect the node if it is outside the environment.
// Nevertheless, it is an abstract class and the move method must be implemented by subclasses.
// Basic mobility features are implemented in this class, such as updating the position of the node and reflecting the node if it is outside the environment.
// Subclasses must implement the move method to define the mobility model of the node.
//

#ifndef ROUTINGSIM_MOBILITYBASE_H_
#define ROUTINGSIM_MOBILITYBASE_H_

#include <cmath>
#include "Environment.h"

enum { MOBILITY_UPDATE };

namespace routingsim {
    class MobilityBase : public Environment {
    private:
        cMessage *updatePositionMessage = new cMessage("mobility_update", MOBILITY_UPDATE);
        static int reflect(double, double, double &, double &);
        void sendUpdatePositionMessage();

    protected:
        // The interval to update the position of the node.
        double updateInterval;

        // The heading of the node.
        Rad2D heading;

        // The target position of the node.
        // The node moves towards this position with the velocity.
        Coord2D targetPosition;

        // The position of the last update of the node.
        Coord2D lastPosition;

        // The velocity of the node.
        Coord2D velocity = Coord2D::ZERO;

        // Initialize the position of the node.
        void initializePosition();

        // Initialize the heading of the node.
        void initializeHeading();

        // Handle the messages received.
        // The method schedules a self message every updateIntervall to update the position of the node.
        // If the message is not a self message, an error is thrown.
        void handleMessage(cMessage *msg) override;

        // Handle the self messages to update the position of the node.
        void handleSelfMessage(cMessage *pMessage);

        // Update the position of the node using the move method and handle the node if it is outside the environment.
        void updatePosition();

        // Move the node to the next position based on the mobility model.
        // This method must be implemented by subclasses to define the mobility model of the node.
        virtual void move() = 0;

        // handle the node if it is outside the environment to put it back inside.
        // The default behavior is to call the reflectIfOutside method, but subclasses can override this method to implement different behaviors.
        void handleIfOutside();

        // reflect the node if it is outside the environment
        void reflectIfOutside();

    public:
        // Sets initial position and heading of the node.
        void initialize() override;

        // Returns the position of node
        Coord2D getPosition() { return lastPosition; }

        double getUpdateInterval() const;
        static simsignal_t mobilityStateChangedSignal;
    };

};
#endif //ROUTINGSIM_MOBILITYBASE_H
