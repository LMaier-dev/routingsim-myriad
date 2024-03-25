//
// Created by s_schaut on 22.03.24.
//

#include "RandomWaypointMobility.h"
#include "MobilityBase.cpp"

namespace routingsim {

    Define_Module(RandomWaypointMobility);

    void RandomWaypointMobility::initialize(int stage) {
        setNextTarget();
        MobilityBase::initialize(stage);
    }

    void RandomWaypointMobility::move() {
        simtime_t now = simTime();
        if (now >= nextChange) {
            lastPosition = targetPosition;
            EV_INFO << "reached current target position = " << lastPosition.toString() << endl;
            setNextTarget();
            EV_INFO << "new target position = " << targetPosition.toString() << ", next change = " << nextChange << endl;
        }
        else if (now > lastUpdate) {
            ASSERT(nextChange == -1 || now < nextChange);
            lastPosition += velocity * (now - lastUpdate).dbl();
            lastUpdate = now;
        }
    }

    void RandomWaypointMobility::setNextTarget() {
        setTargetPosition();
        setTargetSpeed();
        setTargetVelocity();
        setTargetNextPeriod();
        waitNextPeriod = !waitNextPeriod;
    }

    void RandomWaypointMobility::setTargetPosition() {
        if (waitNextPeriod) {
            targetPosition = lastPosition;
        } else {
            targetPosition = Coord2D(uniform(borderMin.x, borderMax.x), uniform(borderMin.y, borderMax.y));
        }
    }

    void RandomWaypointMobility::setTargetVelocity() {
        if (waitNextPeriod) {
            velocity = Coord2D::ZERO;
            timeToNextPeriod = waitTime;
        } else {
            Coord2D direction = targetPosition - lastPosition;
            double distance = direction.distance_squared();
            timeToNextPeriod = distance / timeToNextPeriod;
            velocity = direction / timeToNextPeriod;
        }
    }

    void RandomWaypointMobility::setTargetSpeed() {
        speed = uniform(minSpeed, maxSpeed);
    }

    void RandomWaypointMobility::setTargetNextPeriod() {
        nextChange = simTime() + timeToNextPeriod;
    }

};

