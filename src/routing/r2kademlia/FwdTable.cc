#include "FwdTable.h"
#include "R2Kademlia.h"
#include "PathIDUtils.h"

namespace routingsim {

std::string FwdTableEntry::str() const {

	std::ostringstream out;
	out << outgoingPathID << " (";
		if (isUndefined()) out << 'U';
		if (isValid()) out << 'V';
		if (isObsolete()) out << 'O';
		if (isInvalid()) out << 'I';
		if (isForeign()) out << 'F';
		if (isLocal()) out << 'L';
		if (isPrecalc()) out << 'P';
	        out << ':';

		switch (fwdAction) {
			case FwdTableEntry::Action::undefined:
				out << 'U';
				break;
			case FwdTableEntry::Action::fwdNextPathID:
				out << 'P';
				break;
			case FwdTableEntry::Action::fwdNextNodeId:
				out << 'N';
				break;
			case FwdTableEntry::Action::fwdlocal:
				out << 'L';
				break;
			default:
				out << '?';
		} // end switch;
		out << ":" << getReferenceCount();
		out << ") nh <" << nextHopNode.shortStr() << ">(" << getPathLength() << ")";

		return out.str();

}


// this counts only valid and active entries (=non obsolete)
size_type FwdTablePathID::getNumValidEntries() const
{
	unsigned long sum= 0;
	for (auto& pidfwdtableentry : PIDFwdTable)
	{
		const FwdTableEntry& fwdentry= pidfwdtableentry.second;

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
		const FwdTableEntry& fwdentry= pidfwdtableentry.second;
		// it may be the case that foreign entries are also precalculated or locally required, we do not count them as foreign then
		if (fwdentry.isForeign() and fwdentry.isValid() and  not(fwdentry.isPrecalc() or fwdentry.isLocal() or fwdentry.isObsolete()))
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
		const FwdTableEntry& fwdentry= pidfwdtableentry.second;

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

	out << "NIDPIDTable (" << NIDPIDTable.size() << "): ";
	for (const auto& nidpidentry : NIDPIDTable)
	{
		out << "{ " << nidpidentry.first.shortStr() << "=> " << nidpidentry.second << ' ';
		if (const auto it= PIDFwdTable.find(nidpidentry.second); it != PIDFwdTable.end()) {
			out << it->second.str() << " } ";
		}
		else
			out << "<missing PID> } ";
	}

	auto [ptotal,pused]= getNumUsedPrecalcEntries();
	out << "PIDFwdTable (F=" << getNumForeignEntries() << ",P=" << pused << '/' << ptotal << ",V=" << getNumValidEntries() << "): ";
	for (const auto& pidfwdtableentry : PIDFwdTable)
	{
		const FwdTableEntry& fwdentry= pidfwdtableentry.second;

		out << "[" << pidfwdtableentry.first << "=>" << fwdentry.str() << "] ";
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


// adds/updates a mapping from incomingPID -> outgoingPID, calculated from path_to_add
// path_to_add should contain all subsequent hops, e.g. , <h1 h2 h3 h4 h5> -> outgoing PathID = H(h1|h2|h3|h4|h5)
// precalc is set for contacts within the vicinity discovery range
// @returns true if entry was newly added/created, otherwise false (for existing entry)
// the incomingPathID is returned
std::pair<bool, PathID>
FwdTablePathID::addEntry(const kad_path& path_to_add, bool isForeignPID, bool precalc)
{
	ASSERT(not path_to_add.empty());

	// in case the path is empty, it makes no sense to add anything
	// this should never happen actually
	if (path_to_add.empty())
		return std::make_pair(false, PathID::UndefinedPathID);

	// Routingtable entry on node 4: Dest 5: < 3 2 >
	//EV_TRACE << "addEntry(): called for path " << path_to_add << endl;
	//ASSERT((not precalc and (currentpath.size()>nodeVicinityGraph::vicinityDiscoveryRadius))
	//       or (precalc and currentpath.size()<=nodeVicinityGraph::vicinityDiscoveryRadius));
	// prepend own nodeID for incoming PathID
	PathID incomingPathID(getPathID(*ownNodeID, path_to_add));

	// check for already existing PID
	if ( auto pidentry= PIDFwdTable.find(incomingPathID); pidentry != PIDFwdTable.end() )
	{
		// if it was marked for deletion before, reset the flag as it is used now
		pidentry->second.unsetDeleted();
		pidentry->second.setValid();

		//EV_TRACE << "addEntry(): inPID entry " << incomingPathID << " exists already" << endl;
		// set foreign or local flag
		if (isForeignPID)
			pidentry->second.setForeign();
		else {
			pidentry->second.incrementReferenceCount();
			pidentry->second.setLocal();
		}

		if (precalc)
			pidentry->second.setPrecalc();

		return std::make_pair(false, incomingPathID); // already there, fine
	}

	// create new PID entry
	FwdTableEntry fwdentry;

	fwdentry.outgoingPathID= (path_to_add.size() > 1) ? getPathID(path_to_add) : PathID::UndefinedPathID;
	fwdentry.setPathLength(path_to_add.size());
	fwdentry.nextHopNode= path_to_add.at(0);
	fwdentry.setValid();
	if (isForeignPID)
		fwdentry.setForeign();
	else {
	        fwdentry.incrementReferenceCount();
		fwdentry.setLocal();
	}

	if (precalc)
		fwdentry.setPrecalc();

	fwdentry.fwdAction=
		(path_to_add.size()==1) // next hop is our physical neighbor
		? FwdTableEntry::Action::fwdNextNodeId : FwdTableEntry::Action::fwdNextPathID;

	PIDFwdTable[incomingPathID]= fwdentry;

	// tpath: path including the target
	EV_TRACE << "addEntry(): inPID: " << incomingPathID << " outPID: "
		 << fwdentry.outgoingPathID << " tpath: " << path_to_add << " " << fwdentry.str() << endl;

	return std::make_pair(true, incomingPathID);
}


// this invalidates the PID entry that corresponds to the current path (i.e., path is known to be broken)
void FwdTablePathID::invalidateEntry(KadContact_p contact)
{
	if (contact == nullptr)
		return;

	// do not invalidate any paths in vicinity as they are precalculated
	if (contact->getPathLength() < nodeVicinityGraph::vicinityDiscoveryRadius)
		return;

	// we have to check whether the PID part is actually affected by the broken link
	kad_path currentpath(contact->getPath());
	if (contact->getPathLength() <= nodeVicinityGraph::vicinityDiscoveryRadius) {
	// add destination node ID to end of the path (will never be empty then)
		currentpath.push_back(contact->getKey());
	}
	else {
		if (contact->getPathLength() <= 2*nodeVicinityGraph::vicinityDiscoveryRadius) {
			// for paths with 4 or 5 hops: stop at 3rd hop
			currentpath.resize(nodeVicinityGraph::vicinityDiscoveryRadius+1);
		}
		else {
			// stop three hops before destination
			currentpath.resize(contact->getPathLength()-2);
		}
	}

	// check if broken link is within this path
	// TODO!!
	
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


// returns false if entry is not accepted
// NOTE: newpath is the complete path (w/o source and target) to the contact, not just the first segment
// Note that modEntry() may also be called while a path setup is still pending, i.e.,
// there is no NIDPID and PID entry yet and a shorter path may have been found meanwhile
bool FwdTablePathID::modEntry(const OverlayKey& contact_key, const kad_path& newpath, bool isForeignPID)
{
	PathID priorPathID= PathID::UndefinedPathID;
	bool additionalEntry= false;
	auto fullpath(newpath); // getSegmentPathIDs needs also the destination node ID
	fullpath.push_back(contact_key);
	// calculate incomingPathID
	auto [incomingPathID,outgoingPathID,lastsegmentPathID]= getSegmentPathIDs(*ownNodeID, fullpath);

	// Lookup old entry if it exists
	// (which should be mostly the case, however in case a PathSetupReq is ongoing or was not necessary, it may be missing)
	auto pidentry= NIDPIDTable.find(contact_key);
	if ( pidentry != NIDPIDTable.end() )
	{
		EV_TRACE << "modEntry(): Dest NodeID " << contact_key.shortStr() << " new PID path " << newpath << endl;

		// Routingtable entry on node 4: Dest 5: < 3 2 >
		priorPathID= pidentry->second;
		if ( PIDFwdTable.find(priorPathID) == PIDFwdTable.end() )
		{
			EV_ERROR << "FwdTablePathID::modEntry earlier PathID " << priorPathID << " not found" << endl;
			return false;
		}
		else if (priorPathID == incomingPathID)
		{ // same PathID already existed, just return
			EV_TRACE << "modEntry(): same PID entry " << incomingPathID << " for contact already existed" << endl;
			if (auto it= PIDFwdTable.find(incomingPathID); it != PIDFwdTable.end())
			{
				it->second.setValid();
				it->second.unsetObsolete();
				if (isForeignPID)
					it->second.setForeign();
				else {
					it->second.incrementReferenceCount();
					it->second.setLocal();
				}
			}

			return false;
		}
		else
		{ // set old entry to obsolete state if local update
			FwdTableEntry& priorfwdentry(PIDFwdTable[pidentry->second]);

			// mark it obsolete unless it was precalculated
			if (not priorfwdentry.isPrecalc()) {
				priorfwdentry.setObsolete();
				// not referenced anymore
				if (priorfwdentry.isLocal()) {
					if (priorfwdentry.decrementReferenceCount() == 0)
						priorfwdentry.unsetLocal();
				}
			}
			// delete the entry before priorPathID (i.e., the one before the penultimate entry)
			//TODO make this configurable in depth
			if (priorfwdentry.prevPathID != PathID::UndefinedPathID)
			{
				auto it= PIDFwdTable.find(priorfwdentry.prevPathID);
				if (it != PIDFwdTable.end())
				{
					// remove entry before priorPathID if still obsolete and not in use otherwise
					// it may still have the local flag
					if (it->second.isObsolete() and not it->second.isPrecalc() and not it->second.isForeign() and it->second.getReferenceCount()==0)
					{
						EV_TRACE << "deleting PID entry " << priorfwdentry.prevPathID << endl;
						PIDFwdTable.erase(it);
					}
				}
			}
		}
	}

	FwdTableEntry fwdentry;
	// if PID already exists, just re-enable mapping and set to valid or modify flags
	if (auto it= PIDFwdTable.find(incomingPathID); it != PIDFwdTable.end())
	{
		EV_TRACE << "modEntry(): inPID " << incomingPathID << " already existed" << endl;
		it->second.setValid();
		if (isForeignPID)
			it->second.setForeign();
		else {
			it->second.setLocal();
			it->second.incrementReferenceCount();
		}

		// only used for debugging output afterwards
		fwdentry= it->second;
	}
	else
	{ // create new entry in FwdTable
		// this may happen in case a PathSetup is still pending
		fwdentry.outgoingPathID= outgoingPathID;
		fwdentry.nextHopNode= fullpath.at(0); // fullpath will never be empty
		fwdentry.setValid();
		if (isForeignPID)
			fwdentry.setForeign();
		else {
			fwdentry.setLocal();
			fwdentry.incrementReferenceCount();
		}

		// use length of first segment only
		fwdentry.setPathLength(fullpath.size() <= (nodeVicinityGraph::vicinityDiscoveryRadius+1) ? fullpath.size() :
				       (fullpath.size() > 2*nodeVicinityGraph::vicinityDiscoveryRadius+1 ?
					fullpath.size()-(nodeVicinityGraph::vicinityDiscoveryRadius+1) : fullpath.size()-nodeVicinityGraph::vicinityDiscoveryRadius));
		fwdentry.fwdAction=
			(fullpath.size()==1) // next hop is our physical neighbor
			? FwdTableEntry::Action::fwdNextNodeId : FwdTableEntry::Action::fwdNextPathID;
		fwdentry.prevPathID= priorPathID;
		PIDFwdTable[incomingPathID]= fwdentry;
	}

	// activate new mapping for replacements (or new entries)
	// only if path setup is not required
	setNIDtoPIDmapping(contact_key, incomingPathID, lastsegmentPathID);

	EV_TRACE << "modEntry(): inPID:" << incomingPathID << " outPID:"
		 << fwdentry.outgoingPathID << " 2nd PID:" << lastsegmentPathID
		 << (additionalEntry ? " additional entry" : ((pidentry != NIDPIDTable.end()) ? " replacement" : " new"))
		 << " path: " << newpath << endl;

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
			// mark this entry for deletion
			it->second.setDeleted();
			delPIDEntry(it->second.prevPathID);
			it->second.prevPathID= PathID::UndefinedPathID;
		}

		// local and precalculated PID entries will not be deleted here,
		// foreign entries should be deleted by PathTeardown messages by unsetting their foreign bit first
		if ( not it->second.isLocal() and not it->second.isForeign() and not it->second.isPrecalc() and it->second.getReferenceCount()==0) {
			it->second.setDeleted();
			EV_TRACE << "FwdTablePathID::delPIDEntry PathID " << pid << " marked for deletion." << endl;
			// actually delete this PID Entry
			// suspended until garbageCollect()
		}
		else
			EV_TRACE << "FwdTablePathID::delPIDEntry PathID " << pid << " not yet deleted." << endl;
	}
}


void FwdTablePathID::delForeignPIDEntry(const PathID pid)
{
	if (pid == PathID::UndefinedPathID)
		return;

	if ( auto it= PIDFwdTable.find(pid); it != PIDFwdTable.end() )
	{ // entry exists -> unset foreign flag (may be restored by next refresh)
		it->second.unsetForeign();
		// try to mark entry for deletion (in case it is not local or precalculated)
		delPIDEntry(pid);
	}
}


void FwdTablePathID::delEntry(KadContact_p contact)
{
       	if (contact == nullptr)
		return;

	// Lookup NID -> PID entry
	if ( auto pidentry= NIDPIDTable.find(contact->getKey()); pidentry != NIDPIDTable.end() )
	{
		// delete all PID entries related to this contact except when they were precalculated
		if (PathID currentPathID= pidentry->second; currentPathID != PathID::UndefinedPathID) {
			std::unordered_set<PathID> alreadySeenPIDSet;
			// get fwdentry and remove local flag for related PIDs because the local contact gets deleted
			for (auto fwdentry_it= PIDFwdTable.find(currentPathID);
			     fwdentry_it != PIDFwdTable.end() and fwdentry_it->first != PathID::UndefinedPathID;
			     fwdentry_it= PIDFwdTable.find(fwdentry_it->second.prevPathID)) {
				// if the next condition holds, we got a cycle
				if (alreadySeenPIDSet.find(fwdentry_it->first) != alreadySeenPIDSet.end()) {
					EV_TRACE << "CYCLE detected, fwdentry: " << fwdentry_it->first
						 << endl;
					break;
				}

				alreadySeenPIDSet.insert(fwdentry_it->first);

				if (fwdentry_it->second.isLocal()) {
					if (fwdentry_it->second.decrementReferenceCount() == 0)
						fwdentry_it->second.unsetLocal();
				}

				if (fwdentry_it->second.prevPathID == PathID::UndefinedPathID)
					break;
			}

			// now try to delete the PID entries
			delPIDEntry(currentPathID);
		}

		// delete anchor entry in NIDPIDTable
		NIDPIDTable.erase(pidentry);
		NIDPIDTableLastSeg.erase(contact->getKey());
		EV_TRACE << "delEntry(): NIDPID entry for " << contact->getKey().shortStr() << " deleted" << endl;

	}
	else // maybe only the PID exists but no NID -> PID mapping (e.g., in case of PathSetupReq)
	{
		kad_path newpath(contact->getPath());
		// add destination node ID to path (will never be empty then)
		newpath.push_back(contact->getKey());
		PathID incomingPathID(getPathID(*ownNodeID, newpath));
		// try to find PID entry and unset the foreign flag
		delForeignPIDEntry(incomingPathID);
	}
}


void FwdTablePathID::garbageCollect(const unsigned long fwdentry_refresh_timeout_ms)
{
	auto it= PIDFwdTable.begin();
	while ( it != PIDFwdTable.end() )
	{
		// not used entries are marked for deletion and also have lost their foreign bit
		if (it->second.isDeleted() and not it->second.isLocal() and not it->second.isPrecalc()
		    and it->second.getAge() > fwdentry_refresh_timeout_ms) {
			EV_TRACE << "garbageCollect(): actually deleting PID " << it->first << ": " << it->second.str() << endl;
			it= PIDFwdTable.erase(it);
		}
		else {
			it++;
		}
	} // end while
}

} // end namespace
