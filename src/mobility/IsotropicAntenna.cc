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
  if (datarate != -1) {
    EV_INFO << "Overwriting datarate with config value " << datarate << endl;
    return datarate;
  }

  DB PL = pathloss(distance);
  DB S = signalPower;
  double N = systemNoise.toDouble();
  // SNR = 1 + (S - PL) / N
  double signalToNoiseRatio = (S - PL).toDouble() / N;
  double datarate = bandwidth.toHz() * log2( 1 + signalToNoiseRatio);

  return datarate;
}


} // namespace routingsim
