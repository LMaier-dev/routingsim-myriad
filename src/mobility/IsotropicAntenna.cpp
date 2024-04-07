
#include "IsotropicAntenna.h"
#include <cmath>

namespace routingsim {

Define_Module(IsotropicAntenna);

double IsotropicAntenna::pathloss(double distance) {
  const double maxBandwidth = this->getMaxBandwidth();
  if (distance <= 1) {
    return maxBandwidth;
  } else {
    return maxBandwidth / pow(distance, 2);
  }
}

} // namespace routingsim
