//
// Created by s_schaut on 22.03.24.
//

#ifndef ROUTINGSIM_MOBILITYBASE_H_
#define ROUTINGSIM_MOBILITYBASE_H_

#include <omnetpp.h>
#include <cmath>
#include "../foundation/BaseTypes.h"
#include "Environment.h"

namespace routingsim {
    class MobilityBase : public Environment {
    private:
        static int reflect(double, double, double &, double &);

    protected:
        double updateInterval;
        Rad2D heading;
        Coord2D targetPosition;
        Coord2D lastPosition;
        Coord2D velocity = Coord2D::ZERO;

        void initialize(int) override;

        void initializePosition();

        void initializeHeading();

        void handleMessage(cMessage *msg) override;

        void handleSelfMessage(cMessage *pMessage);

        void updatePosition();

        virtual void move() = 0;

        void handleIfOutside();

        void reflectIfOutside();

    public:
        double getUpdateInterval() const;
    };

};
#endif //ROUTINGSIM_MOBILITYBASE_H
