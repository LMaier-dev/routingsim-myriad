
#include "AntennaBase.h"

namespace routingsim {

Register_Abstract_Class(AntennaBase);

void AntennaBase::initialize() {
  this->signalPower = uniform(par("minP").doubleValue(), par("maxP").doubleValue());
  this->bandwidth = uniform(par("minB").doubleValue(), par("maxB").doubleValue());
  this->range = calcRange();
  this->systemNoise = par("systemNoise").doubleValue();
}

double AntennaBase::calcRange() {
  double maxPathloss = signalPower - 90;//TODO: dbm als eigener Datentyp!;
  return this->distance_from_pathloss(maxPathloss);
}
} // namespace routingsim
