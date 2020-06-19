#include "FwdTable.h"
#include "R2Kademlia.h"

namespace routingsim {

// this counts only valid and active entries (=non obsolete)
size_type FwdTablePathID::getNumValidEntries() const
{
	unsigned long sum= 0;
	for (auto& pidfwdtableentry : PIDFwdTable)
	{
		const FwdTableEntry&  fwdentry= pidfwdtableentry.second;

		if (fwdentry.isValid() and not(fwdentry.isObsolete()))
			sum++;
	}
	return sum;

}


// this counts only foreign valid and active entries (=non obsolete)
size_type FwdTablePathID::getNumForeignEntries() const
{
	unsigned long sum= 0;
	for (auto& pidfwdtableentry : PIDFwdTable)
	{
		const FwdTableEntry&  fwdentry= pidfwdtableentry.second;

		if (fwdentry.isValid() and not(fwdentry.isObsolete()) and fwdentry.isForeign())
			sum++;
	}
	return sum;
}


	
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
		if (fwdentry.isLocal()) out << 'L';
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

		out << ") nh <" << fwdentry.nextHopNode.shortStr() << ">(" << fwdentry.getPathLength() << "); ";
	}

	return out.str();

}


// contact is sometimes a temporary variable, i.e., the contact may not be part of the routing table yet
void FwdTablePathID::addEntry(KadContact_p contact, bool isForeignPID)
{
       	if (contact == nullptr)
		return;

	// Routingtable entry on node 4: Dest 5: < 3 2 >
	//EV_TRACE << "addEntry(): called for contact " << *contact << " with path ";

	// note: this path may be empty for physical neighbors
	// Incoming PathIDs are required for physical neighbors, outgoing PathIDs not
	kad_path currentpath(contact->getPath());
	// add destination node ID to end of the path (will never be empty then)
	currentpath.push_back(contact->getKey());

	//EV_TRACE << currentpath << endl;

	// prepend own nodeID for incoming PathID
	PathID incomingPathID(getPathID(*ownNodeID, currentpath));

	// check for already existing PID
	if ( auto pidentry= PIDFwdTable.find(incomingPathID); pidentry != PIDFwdTable.end() )
	{
		//EV_TRACE << "addEntry(): inPID entry " << incomingPathID << " exists already" << endl;
		// set foreign flag
		if (isForeignPID)
			pidentry->second.setForeign();
		return; // already there, fine
	}

	FwdTableEntry fwdentry;
	// Check for previous existing entry for this contact

	if ( auto it= NIDPIDTable.find(contact->getKey()); it != NIDPIDTable.end() )
	{
		//EV_TRACE << "addEntry(): PID entry for contact " << contact->getKey().shortStr() << " existed already with incoming PID " << it->second << endl;
		if (it->second == incomingPathID)
		{
			//EV_TRACE << "addEntry(): same PID entry for contact already existed - ignored" << endl;
			return;
		}

		// do the right thing now and call modEntry
		// will probably add an additional entry
		modEntry(contact, isForeignPID);
		return;
	}

	fwdentry.outgoingPathID= (currentpath.size() > 1) ? getPathID(currentpath) : PathID::UndefinedPathID;
	fwdentry.setPathLength(currentpath.size());
	fwdentry.nextHopNode= currentpath[0];
	fwdentry.setValid();
	if (isForeignPID)
		fwdentry.setForeign();
	else
		fwdentry.setLocal();

	fwdentry.fwdAction=
		(currentpath.size()==1) // next hop is our physical neighbor
		? FwdTableEntry::Action::fwdNextKadId : FwdTableEntry::Action::fwdNextPathID;
	
	PIDFwdTable[incomingPathID]= fwdentry;
	NIDPIDTable[contact->getKey()]= incomingPathID;

	EV_TRACE << "addEntry(): inPID: " << incomingPathID << " outPID: "
		 << fwdentry.outgoingPathID << " path: " << currentpath << endl;
}


// returns false if entry is not accepted (e.g., path is longer)
bool FwdTablePathID::modEntry(KadContact_p contact, bool isForeignPID)
{
       	if (contact == nullptr)
		return false;

	PathID priorPathID= PathID::UndefinedPathID;
	bool additionalEntry= false;

	// Lookup old entry if it exists
	if ( auto pidentry= NIDPIDTable.find(contact->getKey()); pidentry != NIDPIDTable.end() )
	{
		kad_path newpath(contact->getPath());

		EV_TRACE << "modEntry(): Dest NodeID " << contact->getKey().shortStr() << newpath << endl;

		// add destination node ID to path (will never be empty then)
		newpath.push_back(contact->getKey());
		// calculate incomingPathID
		PathID incomingPathID(getPathID(*ownNodeID, newpath));

		// Routingtable entry on node 4: Dest 5: < 3 2 >
		priorPathID= pidentry->second;
		if ( PIDFwdTable.find(priorPathID) == PIDFwdTable.end() )
		{
			EV_ERROR << "FwdTablePathID::modEntry earlier PathID not found" << endl;
			return false;

		}
		else if (priorPathID == incomingPathID)
		{ // same PathID already existed, just return
			EV_WARN << "modEntry(): same PID entry for contact already existed" << endl;
			return false;
		}
		else
		{ // set old entry to obsolete state if local update
			FwdTableEntry& priorfwdentry(PIDFwdTable[pidentry->second]);

			// only accept better PIDs
			if (newpath.size() > priorfwdentry.getPathLength())
			{
				EV_TRACE << "new path not better: len=" << newpath.size()
					 << " prior len=" << priorfwdentry.getPathLength() << " newPID: " << getPathID(newpath) << endl;
				return false;
			}
			else
			if (newpath.size() == priorfwdentry.getPathLength())
			{
				EV_TRACE << "new path has equal length: len=" << newpath.size()
					 << " prior len=" << priorfwdentry.getPathLength() << " newPID: " << getPathID(newpath) << endl;
				additionalEntry= true;
			}

			if (not additionalEntry)
			{
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
							EV_TRACE << "deleting PID entry " << priorfwdentry.prevPathID << endl;
							PIDFwdTable.erase(it);
						}
						else
							EV_WARN << "PID entry before prior penultimate entry not obsolete";
					}
				}
			}
		}

		FwdTableEntry fwdentry;
		// PID already exists, just re-enable mapping
		if (auto it= PIDFwdTable.find(incomingPathID); it != PIDFwdTable.end())
		{
			EV_TRACE << "modEntry(): inPID " << incomingPathID << " already existed" << endl;
			fwdentry= it->second;
		}
		else
		{ // create new entry in FwdTable

			fwdentry.outgoingPathID= getPathID(newpath);
			fwdentry.nextHopNode= newpath[0];
			fwdentry.setValid();
			if (isForeignPID)
				fwdentry.setForeign();
			fwdentry.setPathLength(newpath.size());
			fwdentry.fwdAction=
				(newpath.size()==1) // next hop is our physical neighbor
				? FwdTableEntry::Action::fwdNextKadId : FwdTableEntry::Action::fwdNextPathID;
			fwdentry.prevPathID= priorPathID;
			PIDFwdTable[incomingPathID]= fwdentry;
		}

		// activate new mapping for replacements
		if (not additionalEntry)
			NIDPIDTable[contact->getKey()]= incomingPathID;

		EV_TRACE << "modEntry(): inPID:" << incomingPathID << " outPID:"
			 << fwdentry.outgoingPathID
			 << (additionalEntry ? " additional entry" : " replacement")
			 << " path: " << newpath << endl;

		return true;
	}
	else
	{
		EV_WARN << "FwdTablePathID::modEntry() no prior entry found" << endl;
		return false;
	}

	return false;
}



void FwdTablePathID::delPIDEntry(PathID pid)
{
	if (pid == PathID::UndefinedPathID)
		return;

	if ( auto it= PIDFwdTable.find(pid); it != PIDFwdTable.end() )
	{ // entry exists
		// delete all related PID entries (prior entries)
		if (it->second.prevPathID != PathID::UndefinedPathID)
		{ // recurse here
			delPIDEntry(it->second.prevPathID);
			it->second.prevPathID= PathID::UndefinedPathID;
		}

		if ( not it->second.isLocal() and not it->second.isForeign() ) {
			EV_TRACE << "FwdTablePathID::delEntry PathID " << pid << " deleted." << endl;
			// delete this PID Entry
			PIDFwdTable.erase(it);
		}
		else
			EV_TRACE << "FwdTablePathID::delEntry PathID " << pid << " not yet deleted." << endl;
	}
	else
		EV_ERROR << "FwdTablePathID::delEntry PathID " << pid << " not found" << endl;
}


void FwdTablePathID::delEntry(KadContact_p contact)
{
       	if (contact == nullptr)
		return;

	// Lookup entry
	if ( auto pidentry= NIDPIDTable.find(contact->getKey()); pidentry != NIDPIDTable.end() )
	{
		// delete all PID entries related to this contact
		PathID currentPathID= pidentry->second;

		// get fwdentry and remove local flag for related PIDs because the local contact gets deleted
		for (auto fwdentry_it= PIDFwdTable.find(currentPathID);
		     fwdentry_it != PIDFwdTable.end();
		     fwdentry_it= PIDFwdTable.find(fwdentry_it->second.prevPathID)) {
			fwdentry_it->second.unsetLocal();
			if (fwdentry_it->second.prevPathID == PathID::UndefinedPathID)
				break;
		}

		// now try to delete the PID entries
		delPIDEntry(currentPathID);
		// delete anchor entry in NIDPIDTable
		NIDPIDTable.erase(pidentry);
		EV_WARN << "delEntry(): NIDPID entry for " << contact->getKey().shortStr() << " deleted" << endl;

	}
	else
	{
		EV_WARN << "delEntry(): no entry for " << contact->getKey().shortStr() << " found" << endl;
	}
}

} // end namespace
