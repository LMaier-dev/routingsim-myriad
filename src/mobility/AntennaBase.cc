
#include "AntennaBase.h"

namespace routingsim {

Register_Abstract_Class(AntennaBase);

void AntennaBase::initialize() {
  this->signalPower = DB(uniform(par("minSignalPower").doubleValue(),
                                 par("maxSignalPower").doubleValue(), 1));
  this->sendGain = DB(uniform(par("minSendGain").doubleValue(),
                              par("maxSendGain").doubleValue(), 1));
  this->recvGain = DB(uniform(par("minRecvGain").doubleValue(),
                              par("maxRecvGain").doubleValue(), 1));
  this->systemNoise = DB(par("systemNoise").doubleValue());
  this->frequency =
      MHz(uniform(par("minFreq").doubleValue(), par("maxFreq").doubleValue(), 1));
  this->range = calcRange();
}

double AntennaBase::calcRange() {
  DB maxPathloss = signalPower + DB(LOWESTSIGNALSTRENGTH);
  return this->distance_from_pathloss(maxPathloss);
}
} // namespace routingsim
