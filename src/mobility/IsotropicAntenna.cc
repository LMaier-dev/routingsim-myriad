
#include "IsotropicAntenna.h"

namespace routingsim {

Define_Module(IsotropicAntenna);

// Free-Space-Path-Loss
DB IsotropicAntenna::pathloss(double distance) {
  return DB(20 * log10(distance) + 20 * log10(frequency.toHz()) +
            10 * log10(4 * PI / LIGHTSPEED));
}

double IsotropicAntenna::distance_from_pathloss(DB pathloss) {
  return pow(10, (pathloss.getValue() - 20 * log10(frequency.toHz()) - 10 * log10(4 * PI / LIGHTSPEED)) / 20);
}

double IsotropicAntenna::calcDatarate(double distance) {
  DB PL = pathloss(distance);
  double signalToNoiceRatio = (DB(1) + (signalPower - PL) / systemNoise).toDouble();
  return bandwidth.toHz() * log2(signalToNoiceRatio);
}

} // namespace routingsim
