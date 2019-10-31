#include "FwdTable.h"
#include "R2Kademlia.h"

namespace routingsim {

std::string FwdTablePathID::str() const
{
	std::ostringstream out;

	for (auto& pidfwdtableentry : PIDFwdTable)
	{
		const FwdTableEntry&  fwdentry= pidfwdtableentry.second;

		out << "in:[" << pidfwdtableentry.first << "] (";
		switch (fwdentry.fwdState) {
			case FwdTableEntry::State::undefined:
				out << 'U';
			case FwdTableEntry::State::valid:
				out << 'V';
			case FwdTableEntry::State::obsolete:
				out << 'O';
			case FwdTableEntry::State::invalid:
				out << 'I';
			default:
				out << '?';
		} // end switch;

	        out << ") A:";
		switch (fwdentry.fwdAction) {
			case FwdTableEntry::Action::undefined:
				out << 'U';
			case FwdTableEntry::Action::fwdNextPathID:
				out << 'P';
			case FwdTableEntry::Action::fwdNextKadId:
				out << 'N';
			case FwdTableEntry::Action::fwdlocal:
				out << 'L';
			default:
				out << '?';
		} // end switch;

		out << " out: " << fwdentry.outgoingPathID
		    << "nh: " << fwdentry.nextHopNode;
	}

	return out.str();

}



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
