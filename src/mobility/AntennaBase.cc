
#include "AntennaBase.h"

namespace routingsim {

Register_Abstract_Class(AntennaBase);

void AntennaBase::initialize() {
  this->signalPower = DB(uniform(par("minSignalPower").doubleValue(),
                                 par("maxSignalPower").doubleValue()));
  this->sendGain = DB(uniform(par("minSendGain").doubleValue(),
                              par("maxSendGain").doubleValue()));
  this->recvGain = DB(uniform(par("minRecvGain").doubleValue(),
                              par("maxRecvGain").doubleValue()));
  this->systemNoise = DB(par("systemNoise").doubleValue());
  this->frequency =
      MHz(uniform(par("minFreq").doubleValue(), par("maxFreq").doubleValue()));
  this->range = calcRange();
}

double AntennaBase::calcRange() {
  DB maxPathloss = signalPower + DB(LOWESTSIGNALSTRENGTH);
  return this->distance_from_pathloss(maxPathloss);
}
} // namespace routingsim
