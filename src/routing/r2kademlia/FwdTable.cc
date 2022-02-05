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


// this counts how many of the precalculated entries exist (first return value) and have been used (2nd return value)
std::tuple<size_type, size_type> FwdTablePathID::getNumUsedPrecalcEntries() const
{
	unsigned long total_sum(0);
	unsigned long used_sum(0);

	for (auto& pidfwdtableentry : PIDFwdTable)
	{
		const FwdTableEntry&  fwdentry= pidfwdtableentry.second;

		if (fwdentry.isPrecalc()) {
			total_sum++;
			if (fwdentry.getUsageCount()>0) {
				used_sum++;
			}
		}
	}
	return {total_sum,used_sum};
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



// checks whether path to contact already exists as PID
bool FwdTablePathID::existsEntry(KadContact_p contact)
{
       	if (contact == nullptr)
		return false;

	// Routingtable entry on node 4: Dest 5: < 3 2 >
	//EV_TRACE << "existsEntry(): called for contact " << *contact << " with path ";

	// note: this path may be empty for physical neighbors
	// Incoming PathIDs are required for physical neighbors, outgoing PathIDs not
	kad_path currentpath(contact->getPath());
	// add destination node ID to end of the path (will never be empty then)
	currentpath.push_back(contact->getKey());

	//EV_TRACE << currentpath << endl;

	// prepend own nodeID for incoming PathID
	PathID incomingPathID(getPathID(*ownNodeID, currentpath));

	// check for already existing PID
	return PIDFwdTable.find(incomingPathID) != PIDFwdTable.end();
}


// contact is sometimes a temporary variable, i.e., the contact may not be part of the routing table yet
// precalc is set for contacts within the discovery range
// @returns incoming PathID if entry was added or already present, otherwise (already existed etc.) PathID::undefined
PathID
FwdTablePathID::addEntry(KadContact_p contact, bool isForeignPID, bool precalc)
{
       	if (contact == nullptr)
		return PathID::UndefinedPathID;

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
		if (precalc)
			pidentry->second.setPrecalc();

		return incomingPathID; // already there, fine
	}

	FwdTableEntry fwdentry;

	fwdentry.outgoingPathID= (currentpath.size() > 1) ? getPathID(currentpath) : PathID::UndefinedPathID;
	fwdentry.setPathLength(currentpath.size());
	fwdentry.nextHopNode= currentpath[0];
	fwdentry.setValid();
	if (isForeignPID)
		fwdentry.setForeign();
	else
		fwdentry.setLocal();

	if (precalc)
		fwdentry.setPrecalc();

	fwdentry.fwdAction=
		(currentpath.size()==1) // next hop is our physical neighbor
		? FwdTableEntry::Action::fwdNextKadId : FwdTableEntry::Action::fwdNextPathID;

	PIDFwdTable[incomingPathID]= fwdentry;

	EV_TRACE << "addEntry(): inPID: " << incomingPathID << " outPID: "
		 << fwdentry.outgoingPathID << " spath: " << currentpath << endl;

	return incomingPathID;
}


// this invalidates the PID entry that corresponds to the current path
void FwdTablePathID::invalidateEntry(KadContact_p contact)
{
	if (contact == nullptr)
		return;

	kad_path currentpath(contact->getPath());
	// do not invalidate any paths in vicinity
	if (currentpath.size() < nodeVicinityGraph::vicinityDiscoveryRadius)
		return;
	// add destination node ID to end of the path (will never be empty then)
	currentpath.push_back(contact->getKey());

	// prepend own nodeID for incoming PathID
	PathID incomingPathID(getPathID(*ownNodeID, currentpath));

	if (auto it= PIDFwdTable.find(incomingPathID); it != PIDFwdTable.end()) {
		FwdTableEntry& fwdentry= it->second;
		fwdentry.setInvalid();
	}
	else {
		EV_ERROR << "no FwdTable entry found for PID " << incomingPathID << endl;
	}
}


// returns false if entry is not accepted (e.g., path is longer)
bool FwdTablePathID::modEntry(KadContact_p contact, bool isForeignPID)
{
       	if (contact == nullptr)
		return false;

	PathID priorPathID= PathID::UndefinedPathID;
	bool additionalEntry= false;
	kad_path newpath(contact->getPath());
	// add destination node ID to path (will never be empty then)
	newpath.push_back(contact->getKey());
	// calculate incomingPathID
	PathID incomingPathID(getPathID(*ownNodeID, newpath));

	// Lookup old entry if it exists
	// (which should be mostly the case, however in case a PathSetupReq is ongoing it may be missing)
	auto pidentry= NIDPIDTable.find(contact->getKey());
	if ( pidentry != NIDPIDTable.end() )
	{
		EV_TRACE << "modEntry(): Dest NodeID " << contact->getKey().shortStr() << newpath << endl;

		// Routingtable entry on node 4: Dest 5: < 3 2 >
		priorPathID= pidentry->second;
		if ( PIDFwdTable.find(priorPathID) == PIDFwdTable.end() )
		{
			EV_ERROR << "FwdTablePathID::modEntry earlier PathID not found" << endl;
			return false;

		}
		else if (priorPathID == incomingPathID)
		{ // same PathID already existed, just return
			EV_TRACE << "modEntry(): same PID entry " << incomingPathID << " for contact already existed" << endl;
			if (auto it= PIDFwdTable.find(incomingPathID); it != PIDFwdTable.end())
			{
				auto& fwdentry= it->second;
				fwdentry.setValid();
			}

			return false;
		}
		else
		{ // set old entry to obsolete state if local update
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
						//EV_TRACE << "deleting PID entry " << priorfwdentry.prevPathID << endl;
						//if (not it->second.isPrecalc() and not it->second.isForeign())
						//	PIDFwdTable.erase(it);
					}
					else
						EV_WARN << "PID entry before prior penultimate entry not obsolete";
				}
			}
		}
	}

	FwdTableEntry fwdentry;
	// if PID already exists, just re-enable mapping and set to valid
	if (auto it= PIDFwdTable.find(incomingPathID); it != PIDFwdTable.end())
	{
		EV_TRACE << "modEntry(): inPID " << incomingPathID << " already existed" << endl;
		fwdentry= it->second;
		fwdentry.setValid();
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

	// activate new mapping for replacements (or new entries)
	// only if path setup is not required
	NIDPIDTable[contact->getKey()]= incomingPathID;

	EV_TRACE << "modEntry(): inPID:" << incomingPathID << " outPID:"
		 << fwdentry.outgoingPathID
		 << (additionalEntry ? " additional entry" : ((pidentry != NIDPIDTable.end()) ? " replacement" : " new"))
		 << " spath: " << newpath << endl;

	return true;
}



void FwdTablePathID::delPIDEntry(PathID pid)
{
	if (pid == PathID::UndefinedPathID)
		return;

	if ( auto it= PIDFwdTable.find(pid); it != PIDFwdTable.end() )
	{ // entry exists
		// detect any cycles: stop recursion here
		if (it->second.isDeleted())
			return;

		// delete all related PID entries (prior entries)
		if (it->second.prevPathID != PathID::UndefinedPathID)
		{ // recurse here
			// mark for deletion
			it->second.setDeleted();
			delPIDEntry(it->second.prevPathID);
			it->second.prevPathID= PathID::UndefinedPathID;
		}

		if ( not it->second.isLocal() and not it->second.isForeign() and not it->second.isPrecalc()) {
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
		// delete all PID entries related to this contact except when they were precalculated
		if (PathID currentPathID= pidentry->second; currentPathID != PathID::UndefinedPathID) {
			std::unordered_set<PathID> alreadySeenPIDSet;
			// get fwdentry and remove local flag for related PIDs because the local contact gets deleted
			for (auto fwdentry_it= PIDFwdTable.find(currentPathID);
			     fwdentry_it != PIDFwdTable.end() and fwdentry_it->first != PathID::UndefinedPathID;
			     fwdentry_it= PIDFwdTable.find(fwdentry_it->second.prevPathID)) {
				// if the next condition hold, we got a cycle
				if (alreadySeenPIDSet.find(fwdentry_it->first) != alreadySeenPIDSet.end()) {
					EV_TRACE << "CYCLE detected, fwdentry: " << fwdentry_it->first
						 << endl;
					break;
				}

				alreadySeenPIDSet.insert(fwdentry_it->first);

				fwdentry_it->second.unsetLocal();
				if (fwdentry_it->second.prevPathID == PathID::UndefinedPathID)
					break;
			}

			// now try to delete the PID entries
			delPIDEntry(currentPathID);
		}

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
