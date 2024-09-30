#include "IsotropicAntenna.h"
#include <numbers>

namespace routingsim {

Define_Module(IsotropicAntenna);

constexpr double LIGHTSPEED = 299792458;

// Free-Space-Path-Loss
DB IsotropicAntenna::pathloss(double distance) {
  return DB(20 * log10(distance) + 20 * log10(frequency.toHz()) +
            10 * log10(4 * std::numbers::pi / LIGHTSPEED));
}


double IsotropicAntenna::distance_from_pathloss(DB pathloss) {
  return pow(10, (pathloss.getValue() - 20 * log10(frequency.toHz()) - 10 * log10(4 * std::numbers::pi / LIGHTSPEED)) / 20.0);
}


double IsotropicAntenna::calcDatarate(double distance) {
  DB PL = pathloss(distance);
  double signalToNoiseRatio = (DB(1) + (signalPower - PL) / systemNoise).toDouble();
  return bandwidth.toHz() * log2(signalToNoiseRatio);
}


} // namespace routingsim
