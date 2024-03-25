//
// Created by s_schaut on 22.03.24.
//

#ifndef ROUTINGSIM_RANDOMWALK_H
#define ROUTINGSIM_RANDOMWALK_H

#include <omnetpp.h>
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
        const double minSpeed = par("minSpeed");
        const double maxSpeed = par("maxSpeed");
        const double waitTime = par("waitTime");
        void initialize(int stage) override;
        void setNextTarget();
        void move() override;
    };
}

#endif //ROUTINGSIM_RANDOMWALK_H
