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
		if (fwdentry.isUndefined()) out << 'U';
		if (fwdentry.isValid()) out << 'V';
		if (fwdentry.isObsolete()) out << 'O';
		if (fwdentry.isInvalid()) out << 'I';
		if (fwdentry.isForeign()) out << 'F';
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



void FwdTablePathID::addEntry(KadContact_p contact, bool foreignPID)
{
       	if (contact == nullptr)
		return;

	// Routingtable entry on node 4: Dest 5: < 3 2 >

	// note: this path may be empty for physical neighbors
	kad_path currentpath(contact->getPath());
	// add destination node ID to end of the path (will never be empty then)
	currentpath.push_back(contact->getKey());

	// prepend own nodeID for incoming PathID
	PathID incomingPathID(getPathID(*ownNodeID, currentpath));
		
	FwdTableEntry fwdentry;
	
	// Check for previous existing entry for this contact
	auto it= NIDPIDTable.find(contact->getKey());
	if (it != NIDPIDTable.end())
	{
		EV_WARN << "addEntry(): PID entry for contact " << contact->getKey().shortStr() << " existed already with incoming PID " << it->second << endl;
		
		if (it->second == incomingPathID)
		{
			EV_WARN << "addEntry(): same PID entry for contact already existed - ignored" << endl;
			return;
		}
		
		// do the right thing now and call modEntry
		modEntry(contact);
		return;
	}

	fwdentry.outgoingPathID= getPathID(currentpath);
	fwdentry.nextHopNode= currentpath[0];
	fwdentry.setValid();
	if (foreignPID)
		fwdentry.setForeign();
	fwdentry.fwdAction=
		(currentpath.size()==1) // next hop is our physical neighbor
		? FwdTableEntry::Action::fwdNextKadId : FwdTableEntry::Action::fwdNextPathID;
		
	PIDFwdTable[incomingPathID]= fwdentry;
	NIDPIDTable[contact->getKey()]= incomingPathID;

	EV_TRACE << "addEntry(): inPID: " << incomingPathID << " outPID: "
		 << fwdentry.outgoingPathID << " path: " ;
	
	if (currentpath.size() == 0)
		EV_TRACE << "<->" << endl;
	else {
		EV_TRACE << "< ";
		for (auto key : currentpath)
			EV_TRACE << key.shortStr() << ' ';
		EV_TRACE << '>' << endl;
		
	}
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
		kad_path currentpath(contact->getPath());
		// add destination node ID to path (will never be empty then)
		EV_TRACE << "modEntry(): Dest NodeID " << contact->getKey().shortStr();
		if (currentpath.size() == 0)
			EV_TRACE << "<->" << endl;
		else {
			EV_TRACE << "< ";
			for (auto key : currentpath)
				EV_TRACE << key.shortStr() << ' ';
			EV_TRACE << '>' << endl;
		}

		currentpath.push_back(contact->getKey());
		// calculate incomingPathID
		PathID incomingPathID(getPathID(*ownNodeID, currentpath));

		// Routingtable entry on node 4: Dest 5: < 3 2 >
		priorPathID= pidentry->second;
		if ( PIDFwdTable.find(priorPathID) == PIDFwdTable.end() )
		{
			EV_ERROR << "FwdTablePathID::modEntry earlier PathID not found" << endl;
		}
		else if (priorPathID == incomingPathID)
		{ // same PathID already existed, just return
			EV_WARN << "modEntry(): same PID entry for contact already existed" << endl;
			return;
		}
		else
		{ // set old entry to obsolete state
			FwdTableEntry& priorfwdentry(PIDFwdTable[pidentry->second]);
			priorfwdentry.setObsolete();
			// delete the entry before priorPathID (i.e., the one before the penultimate entry)
			//TODO make this configurable in depth
			if (priorfwdentry.prevPathID != PathID::UndefinedPathID)
			{
				auto it= PIDFwdTable.find(priorfwdentry.prevPathID);
				if (it != PIDFwdTable.end())
				{
					if (it->second.isObsolete())
					{ // remove entry before priorPathID
						PIDFwdTable.erase(it);
					}
					else
						EV_WARN << "PID entry before prior penultimate entry not obsolete";
				}
			}

		}

		
		FwdTableEntry fwdentry;
		
		fwdentry.outgoingPathID= getPathID(currentpath);
		fwdentry.nextHopNode= currentpath[0];
		fwdentry.setValid();
		fwdentry.fwdAction=
			(currentpath.size()==1) // next hop is our physical neighbor
			? FwdTableEntry::Action::fwdNextKadId : FwdTableEntry::Action::fwdNextPathID;
		fwdentry.prevPathID= priorPathID;
		PIDFwdTable[incomingPathID]= fwdentry;
		NIDPIDTable[contact->getKey()]= incomingPathID;

		EV_TRACE << "modEntry(): inPID:" << incomingPathID << " outPID:"
			 << fwdentry.outgoingPathID << " path: " ;
	
		if (currentpath.size() == 0)
			EV_TRACE << "<->" << endl;
		else {
			EV_TRACE << "< ";
			for (auto key : currentpath)
				EV_TRACE << key.shortStr() << ' ';
			EV_TRACE << '>' << endl;
		}
	}
	else
	{
		EV_WARN << "FwdTablePathID::modEntry() no prior entry found" << endl;
	}
	
}



void FwdTablePathID::delPIDEntry(PathID pid)
{
	if (pid == PathID::UndefinedPathID)
		return;
	
	auto it= PIDFwdTable.find(pid);
	if ( it != PIDFwdTable.end() )
	{ // entry exists
		// delete all related PID entries (prior entries)
		if (it->second.prevPathID != PathID::UndefinedPathID)
		{ // recurse here
			delPIDEntry(it->second.prevPathID);
			it->second.prevPathID= PathID::UndefinedPathID;
		}
		// delete this PID Entry
		PIDFwdTable.erase(it);
	}
	else
		EV_ERROR << "FwdTablePathID::delEntry PathID " << pid << " not found" << endl;
}



void FwdTablePathID::delEntry(KadContact_p contact)
{
       	if (contact == nullptr)
		return;

	auto pidentry= NIDPIDTable.find(contact->getKey());
	// Lookup entry
	if ( pidentry != NIDPIDTable.end() )
	{
		// delete all PID entries related to this contact
		PathID currentPathID= pidentry->second;
		delPIDEntry(currentPathID);
		// delete anchor entry in NIDPIDTable
		NIDPIDTable.erase(pidentry);
	}
	else
	{
		EV_WARN << "delEntry(): no entry for " << contact->getKey().shortStr() << " found" << endl;
	}
	
}

} // end namespace
