
#ifndef ROUTINGSIM_ANTENNABASE_H
#define ROUTINGSIM_ANTENNABASE_H

#ifndef LOWESTSIGNALSTRENGTH
#define LOWESTSIGNALSTRENGTH        90
#endif

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

  virtual double distance_from_pathloss(DB pathloass) = 0;

  // Calculate achievable datarate at a given distance
  virtual double calcDatarate(double distance) = 0;
};

} // namespace routingsim

#endif // ROUTINGSIM_ANTENNABASE_H
