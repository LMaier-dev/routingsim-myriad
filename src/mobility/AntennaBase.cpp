//
// Created by s_schaut on 05.04.24.
//

#include "AntennaBase.h"

namespace routingsim {

Register_Abstract_Class(AntennaBase);

AntennaBase::AntennaBase() {
  this->radius.setRadius(uniform(par("minR"), par("maxR")));
  this->maxBandwidth = par("bandwidthMobilityMbps");
}
} // namespace routingsim