//
// Created by s_schaut on 05.04.24.
//

#include "IsotropicAntenna.h"
#include <math.h>

namespace routingsim {

Define_Simple_Module(IsotropicAntenna);

double AntennaBase::pathloss(double distance) {
  if (distance <= 1) {
    return maxBandwidth;
  } else {
    return maxBandwidth  / pow(distance, 2);
  }
}

} // namespace routingsim
