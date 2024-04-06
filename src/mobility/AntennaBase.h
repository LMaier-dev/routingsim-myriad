//
// Created by s_schaut on 05.04.24.
//

#ifndef ROUTINGSIM_ANTENNABASE_H
#define ROUTINGSIM_ANTENNABASE_H

#include "BaseTypes.h"

namespace routingsim {

class AntennaBase: public cSimpleModule {
private:
  double radius;
  double maxBandwidth;

public:
  AntennaBase();
  double getRadius() { return radius; }
  double getMaxBandwidth() const { return maxBandwidth; }

  // Calculate the bandwidth of the antenna at a given distance
  virtual double pathloss(double distance) = 0;
};

} // namespace routingsim

#endif // ROUTINGSIM_ANTENNABASE_H
