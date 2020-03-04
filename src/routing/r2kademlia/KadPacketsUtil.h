#ifndef __ROUTINGSIM_KadPacketsUtil_H_
#define __ROUTINGSIM_KadPacketsUtil_H_

#include <omnetpp.h>

#include "KadPackets_m.h"

namespace routingsim {

template <class T>
void addNotViaLink(T& packet, const link_t& failedlink) {
	// check whether link is already present
	EV_TRACE << "entered addNotViaLink(), list size=" << packet.getFailedLinkArraySize() << endl;
	unsigned int i= 0;
	while(i < packet.getFailedLinkArraySize())
	{
		if ( (packet.getFailedLink(i) == failedlink) or (packet.getFailedLink(i) == std::make_pair(failedlink.second, failedlink.first)) ) {
			break;
		}
		i++;
	}
	EV_TRACE << "addNotViaLink(): i=" << i << endl;

	if (i == packet.getFailedLinkArraySize()) { // not found
		// add failed link
		packet.setFailedLinkArraySize(i+1); // since i == packet.getFailedLinkArraySize()
		packet.setFailedLink(packet.getFailedLinkArraySize()-1, failedlink);
		if (i==0)
			packet.setNotVia(true);
	}
	EV_TRACE << "Left addNotViaLink(), list size=" << packet.getFailedLinkArraySize() << endl;
}

} // end namespace

#endif
