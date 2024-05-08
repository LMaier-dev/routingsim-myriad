
#ifndef ROUTINGSIM_ANTENNABASE_H
#define ROUTINGSIM_ANTENNABASE_H

#include "../foundation/BaseTypes.h"

namespace routingsim {

    class AntennaBase : public cSimpleModule {
    private:
        double calcRange();

    protected:
        DBm signalPower;
        DBm systemNoise;
        MHz bandwidth;
        double range;

    public:
        void initialize() override;
        DBm getSignalPower() const { return signalPower; }
        MHz getBandwidth() const { return bandwidth; }
        double getRange() const { return range; }

        // Calculate pathloss of the signal power at a given distance
        virtual DBm pathloss(double distance) = 0;

        virtual double distance_from_pathloss(DBm pathloass) = 0;

        // Calculate achievable datarate at a given distance
        virtual double calcDatarate(double distance) = 0;
    };

}// namespace routingsim

#endif// ROUTINGSIM_ANTENNABASE_H
