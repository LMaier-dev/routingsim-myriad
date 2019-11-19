#include "FwdTable.h"
#include "R2Kademlia.h"

namespace routingsim {

std::string FwdTablePathID::str() const
{
	std::ostringstream out;

	for (auto& pidfwdtableentry : PIDFwdTable)
	{
		const FwdTableEntry&  fwdentry= pidfwdtableentry.second;

		out << "[" << pidfwdtableentry.first << "=>" << fwdentry.outgoingPathID << "] (";
		switch (fwdentry.fwdState) {
			case FwdTableEntry::State::undefined:
				out << 'U';
				break;
			case FwdTableEntry::State::valid:
				out << 'V';
				break;
			case FwdTableEntry::State::obsolete:
				out << 'O';
				break;
			case FwdTableEntry::State::invalid:
				out << 'I';
				break;
			default:
				out << '?';
		} // end switch;

	        out << ':';
		switch (fwdentry.fwdAction) {
			case FwdTableEntry::Action::undefined:
				out << 'U';
				break;
			case FwdTableEntry::Action::fwdNextPathID:
				out << 'P';
				break;
			case FwdTableEntry::Action::fwdNextKadId:
				out << 'N';
				break;
			case FwdTableEntry::Action::fwdlocal:
				out << 'L';
				break;
			default:
				out << '?';
		} // end switch;

		out << ") nh <" << fwdentry.nextHopNode.shortStr() << ">; ";
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
	NIDPIDTable[contact->getKey()]= incomingPathID;
}



void FwdTablePathID::modEntry(KadContact_p contact)
{
       	if (contact == nullptr)
		return;

	PathID priorPathID= PathID::UndefinedPathID;
	auto pidentry= NIDPIDTable.find(contact->getKey());
	// Lookup old entry if it exists
	if ( pidentry != NIDPIDTable.end() )
	{
		// Routingtable entry on node 4: Dest 5: < 3 2 >
		priorPathID= pidentry->second;
		if ( PIDFwdTable.find(priorPathID) == PIDFwdTable.end() )
		{
			EV_ERROR << "FwdTablePathID::modEntry earlier PathID not found" << endl;
		}
		else
		{ // set old entry to obsolete state
			FwdTableEntry& priorfwdentry(PIDFwdTable[pidentry->second]);
			priorfwdentry.fwdState= FwdTableEntry::State::obsolete;
		}
		
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
		fwdentry.prevPathID= priorPathID;
		PIDFwdTable[incomingPathID]= fwdentry;
		NIDPIDTable[contact->getKey()]= incomingPathID;

	}
	else
	{
		EV_WARN << "FwdTablePathID::modEntry() no prior entry found" << endl;
	}
	
}

} // end namespace
