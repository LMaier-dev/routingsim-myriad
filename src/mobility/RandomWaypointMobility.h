
//
// This module extends the MobilityBase module with the Random Waypoint mobility model.
// It overwrites the move() method to define the movement of the node.
// The node moves at a random speed and direction until it reaches a random target position in the environment.
// To do this, it sets the target position targetPosition to a random position in the physical environment at the beginning of a movement period.
// The move() method checks with each call whether the expected arrival time has been reached or exceeded.
// If not, it updates the position lastPosition of the node according to the speed and direction.
// If it does, it sets the position of the node lastPosition to the target position targetPosition, to not overshoot targets.
// The pause time of the node begins afterward.
// For this purpose, the targetPosition is set to the current position,
// the velocity to zero and the nextChange to the current simulation time plus the constant pause time waitTime.
// If the node has exceeded its target position and is reset as a result, the time in which the node moved too much is subtracted from the pause time.
// After the wait time, the movement period of the node is ended and a new one begins as described at the beginning.
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

      // Calculates the simtime needed, to reach targetPosition from lastPosition.
      // This duration is needed to prevent overshooting the target
      void setTargetNextPeriod();

      // Randomly pick a speed value
      void setTargetSpeed();

      // Randomly pick a new targetPosition
      void setTargetPosition();

      // determine velocity vector with speed and targetPosition
      void setTargetVelocity();

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

        // Returns the duration in s until this node leaves the given rectangle
        // Or the time this node reaches its current target
        // It can be assumed that the rectangle encloses the current position
        double getLeaveDuration(Coord2D a, Coord2D b);
    };
}

#endif //ROUTINGSIM_RANDOMWALK_H
