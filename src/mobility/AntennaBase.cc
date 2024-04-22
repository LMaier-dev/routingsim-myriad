
#include "AntennaBase.h"

namespace routingsim {

Register_Abstract_Class(AntennaBase);

AntennaBase::AntennaBase() {
  this->radius = uniform(par("minR").doubleValue(), par("maxR").doubleValue());
  this->maxBandwidth = par("bandwidthMobilityMbps");
}
} // namespace routingsim
