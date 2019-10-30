#include "FwdTable.h"
#include "R2Kademlia.h"

namespace routingsim {


void FwdTablePathID::addEntry(KadContact_p contact)
{
	if (contact == nullptr)
		return;

	// Routingtable entry on node 4: Dest 5: < 3 2 >

	kad_path currentpath(contact->getPath());
	// add destination node ID to path (will never be empty then)
	currentpath.push_back(contact->getKey());

	PathID incomingPathID(getPathID(*ownNodeID, currentpath));
		
	FwdTableEntry fwdentry;
	
	fwdentry.outgoingPathID= getPathID(currentpath);
	fwdentry.nextHopNode= currentpath[0];
	fwdentry.fwdState= FwdTableEntry::State::valid;
	fwdentry.fwdAction=
		(currentpath.size()==1) // next hop is our physical neighbor
		? FwdTableEntry::Action::fwdNextKadId : FwdTableEntry::Action::fwdNextPathID;
		
	PIDFwdTable[incomingPathID]= fwdentry;
}
	
} // end namespace
