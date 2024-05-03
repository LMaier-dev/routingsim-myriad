
#ifndef ROUTINGSIM_ANTENNABASE_H
#define ROUTINGSIM_ANTENNABASE_H

#include "../foundation/BaseTypes.h"

namespace routingsim {

class AntennaBase: public cSimpleModule {
private:
  double signalPower;
  double bandwidth;

public:
  void initialize() override;
  double getSignalPower() const { return signalPower; }
  double getBandwidth() const { return bandwidth; }
  double getRange() const { return 300.0; }

  // Calculate the bandwidth of the antenna at a given distance
  virtual double pathloss(double distance) = 0;
};

} // namespace routingsim

#endif // ROUTINGSIM_ANTENNABASE_H
