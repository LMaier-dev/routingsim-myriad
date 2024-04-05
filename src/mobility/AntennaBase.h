//
// Created by s_schaut on 05.04.24.
//

#ifndef ROUTINGSIM_ANTENNABASE_H
#define ROUTINGSIM_ANTENNABASE_H

#include "BaseTypes.h"

namespace routingsim {

class AntennaBase {
private:
  Rad2D radius;
  double maxBandwidth;

public:
  AntennaBase();
  Rad2D getRadius() { return radius; }
  double getMaxBandwidth() { return maxBandwidth; }

  // Calculate the bandwidth of the antenna at a given distance
  virtual double pathloss(double distance) = 0;
};

} // namespace routingsim

#endif // ROUTINGSIM_ANTENNABASE_H
