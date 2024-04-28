//
// This class implements the Random Waypoint mobility model.
// The node moves with a random speed and direction until it reaches a random target position in the environment.
// After reaching the target position, the node waits for a random waitTime before moving to the next target position.
//

#ifndef ROUTINGSIM_RANDOMWALK_H
#define ROUTINGSIM_RANDOMWALK_H

#include "MobilityBase.h"

namespace routingsim {

    class RandomWaypointMobility : public MobilityBase {
    private:
        simtime_t lastUpdate;
        simtime_t nextChange;
        double timeToNextPeriod = 0.0;
        bool waitNextPeriod = false;
        double speed = 0.0;

        void setTargetSpeed();
        void setTargetVelocity();
        void setTargetPosition();
        void setTargetNextPeriod();
    protected:
        // The minimum speed of the node in m/s.
        double minSpeed = 0.0;

        // The maximum speed of the node in m/s.
        double maxSpeed = 0.0;

        // The constant wait time of the node in seconds.
        double waitTime = 0.0;

        // Set the next target position and velocity of the node.
        // The velocity is calculated based on the new speed of the node.
        // The speed is generated using the uniform distribution between minSpeed and maxSpeed.
        void setNextTarget();

        // Move the node to the targetPosition and update the lastPosition of the node.
        void move() override;
      public:
        // Initialize the mobility model.
        void initialize() override;
    };
}

#endif //ROUTINGSIM_RANDOMWALK_H
