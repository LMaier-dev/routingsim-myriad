
#include "AntennaBase.h"

namespace routingsim {

Register_Abstract_Class(AntennaBase);

void AntennaBase::initialize() {
  EV_DEBUG << "AntennaBase::initialize()" << endl;
  signalPower = DB(uniform(par("minSignalPower").doubleValue(),
                           par("maxSignalPower").doubleValue(), 1));
  sendGain = DB(uniform(par("minSendGain").doubleValue(),
                        par("maxSendGain").doubleValue(), 1));
  recvGain = DB(uniform(par("minRecvGain").doubleValue(),
                        par("maxRecvGain").doubleValue(), 1));
  systemNoise = DB(par("systemNoise").doubleValue());
  frequency = MHz(uniform(par("minFreq").doubleValue(), par("maxFreq").doubleValue(), 1));
  bandwidth = MHz(par("bandwidth").doubleValue());
  datarate = par("averageDatarate").doubleValue();
  range = calcRange();
}


} // namespace routingsim
