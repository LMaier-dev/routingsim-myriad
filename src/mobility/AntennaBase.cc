
#include "AntennaBase.h"

namespace routingsim {

    Register_Abstract_Class(AntennaBase);

    void AntennaBase::initialize() {
        this->signalPower = DBm(uniform(par("minP").doubleValue(), par("maxP").doubleValue()));
        this->systemNoise = DBm(par("systemNoise").doubleValue());
        this->bandwidth = MHz(uniform(par("minB").doubleValue(), par("maxB").doubleValue()));
        this->range = calcRange();
    }

    double AntennaBase::calcRange() {
        DBm maxPathloss = signalPower + DBm(-90);
        return this->distance_from_pathloss(maxPathloss);
    }
}// namespace routingsim
