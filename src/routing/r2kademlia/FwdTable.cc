#include "FwdTable.h"

namespace routingsim {


void FwdTablePathID::addEntry(KadContact_p contact)
{
	if (contact == nullptr)
		return;

	const kad_path& currentpath(contact->getPath());
	
	if ( currentpath.size()==0 )
	{ // next hop is our physical neighbor
		PathID incomingPathID(getPathID(R2Kademlia::getOverlayKey(), contact->getKey()));
		
		FwdTableEntry fwdentry;
		
		fwdentry.outgoingPathID= getPathID(currentpath);
		fwdentry.nextHopNode= contact->getKey();
		fwdentry.fwdState= State::valid;
		fwdentry.fwdAction= Action::fwdNextKadId;
	}
	else
	{
		PathID incomingPathID(getPathID(R2Kademlia::getOverlayKey(), currentpath));
		
		FwdTableEntry fwdentry;
		if (currentpath.size()==1)
		{
			fwdentry.outgoingPathID= getPathID(currentpath);
			fwdentry.nextHopNode= currentpath[0];
			fwdentry.fwdState= State::valid;
			fwdentry.fwdAction= Action::fwdNextPathID;
		}
		else
		{
			fwdentry.outgoingPathID= getPathID(currentpath);
			fwdentry.nextHopNode= currentpath[0];
			fwdentry.fwdState= State::valid;
			fwdentry.fwdAction= Action::fwdNextPathID;
		}
		
		PIDFwdTable[incomingPathID]= fwdentry;
	}
}
	
} // end namespace
