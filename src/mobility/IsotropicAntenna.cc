
#include "IsotropicAntenna.h"

namespace routingsim {

Define_Module(IsotropicAntenna);

// Free-Space-Path-Loss
DB IsotropicAntenna::pathloss(double distance) {
  return DB(20 * log10(distance) + 20 * log10(frequency.toHz()) +
            10 * log10(4 * PI / LIGHTSPEED));
}

double IsotropicAntenna::distance_from_pathloss(DB pathloss) {
  EV_DEBUG << "distance_from_pathloss: " << pathloss.getValue() << endl;
  return pow(10, (pathloss.getValue() - 20 * log10(frequency.toHz()) - 10 * log10(4 * PI / LIGHTSPEED)) / 20);
}

double IsotropicAntenna::calcDatarate(double distance) {
  return frequency.toHz() *
         log2((DB(1) + (sendGain - pathloss(distance)) / systemNoise)
                  .getValue());
}

} // namespace routingsim
