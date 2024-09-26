
//
// This module is responsible for the antenna model.
// It defines the basic properties of an antenna, which are determined by the scenario parameters when the module is initialised.
// Since the \emph{AntennaBase} module is abstract, concrete methods and values must be implemented by a specific antenna model.
// These determine the calculation of the path loss through air, the maximum transmission range of the node and the available data rate depending on the distance.
// MobilityDynamics uses these methods to update the data rates of the connections between the nodes.
//

#ifndef ROUTINGSIM_ANTENNABASE_H
#define ROUTINGSIM_ANTENNABASE_H


constexpr double LOWESTSIGNALSTRENGTH = 90;

#include "../foundation/BaseTypes.h"

namespace routingsim {

class AntennaBase : public cSimpleModule {
private:
  double calcRange();

protected:
  DB signalPower = DB(0);
  DB sendGain = DB(0);
  DB recvGain = DB(0);
  DB systemNoise = DB(0);
  MHz frequency = MHz(0);
  MHz bandwidth = MHz(0);
  double range = 0;

public:
  void initialize() override;
  DB getSignalPower() const { return signalPower; }
  DB getSendGain() const { return sendGain; }
  DB getRecvGain() const { return recvGain; }
  MHz getFrequency() const { return frequency; }
  double getRange() const { return range; }

  // Calculate pathloss of the signal power at a given distance
  virtual DB pathloss(double distance) = 0;

  // Calculate the maximum transmission range to reach the given pathloss
  virtual double distance_from_pathloss(DB pathloss) = 0;

  // Calculate achievable datarate at a given distance
  virtual double calcDatarate(double distance) = 0;
};


inline
double AntennaBase::calcRange() {
  DB maxPathloss = signalPower + DB(LOWESTSIGNALSTRENGTH);
  return distance_from_pathloss(maxPathloss);
}

} // namespace routingsim

#endif // ROUTINGSIM_ANTENNABASE_H
