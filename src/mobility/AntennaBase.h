
#ifndef ROUTINGSIM_ANTENNABASE_H
#define ROUTINGSIM_ANTENNABASE_H

#include "../foundation/BaseTypes.h"

namespace routingsim {

class AntennaBase: public cSimpleModule {
private:
  double calcRange();

protected:
  double signalPower;
  double bandwidth;
  double range;
  double systemNoise;

public:
  void initialize() override;
  double getSignalPower() const { return signalPower; }
  double getBandwidth() const { return bandwidth; }
  double getRange() const { return range; }

  // Calculate pathloss of the signal power at a given distance
  virtual double pathloss(double distance) = 0;

  virtual double distance_from_pathloss(double pathloass) = 0;

  // Calculate achievable datarate at a given distance
  virtual double calcDatarate(double distance) = 0;

};

} // namespace routingsim

#endif // ROUTINGSIM_ANTENNABASE_H
