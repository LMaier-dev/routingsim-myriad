
#include "AntennaBase.h"

namespace routingsim {

Register_Abstract_Class(AntennaBase);

void AntennaBase::initialize() {
  this->signalPower = uniform(par("minP").doubleValue(), par("maxP").doubleValue());
  this->bandwidth = uniform(par("minB").doubleValue(), par("maxB").doubleValue());
}
} // namespace routingsim
