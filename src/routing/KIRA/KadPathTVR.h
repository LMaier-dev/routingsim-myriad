#ifndef KADPATHTVR_H_
#define KADPATHTVR_H_

#include "KadPath.h"

namespace routingsim {

class KadPathTVR : public KadPath {
				simtime_t beginValidity = SIMTIME_ZERO;
				simtime_t endValidity = SIMTIME_ZERO;
 public:
				simtime_t getBeginValidity() const { return beginValidity; }
				simtime_t getEndValidity() const { return endValidity; }
				void setBeginValidity(simtime_t beginval) { beginValidity= beginval; }
				void setEndValidity(simtime_t endval) { endValidity= endval; }
};

} // end namespace

#endif // KADPATHTVR_H_
