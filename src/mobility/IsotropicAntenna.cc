#include "IsotropicAntenna.h"
#include <numbers>

namespace routingsim {

Define_Module(IsotropicAntenna);

constexpr double LIGHTSPEED = 299792458;

// Free-Space-Path-Loss
DB IsotropicAntenna::pathloss(double distance) {
  return DB(20 * log10(distance) + 20 * log10(frequency.toHz()) +
            20 * log10(4 * std::numbers::pi / LIGHTSPEED));
}


double IsotropicAntenna::distance_from_pathloss(DB pathloss) {
  return pow(10, (pathloss.getValue() / 20.0) - log10(4 * std::numbers::pi * frequency.toHz() / LIGHTSPEED));
}


double IsotropicAntenna::calcDatarate(double distance) {
  DB PL = pathloss(distance);
  double signalToNoiseRatio = (DB(1) + (signalPower - PL) / systemNoise).toDouble();
  return bandwidth.toHz() * log2(signalToNoiseRatio);
}


} // namespace routingsim
