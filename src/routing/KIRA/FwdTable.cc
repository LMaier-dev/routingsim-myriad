#include "FwdTable.h"
#include "PathIDUtils.h"
#include "NodeVicinityGraph.h"

namespace routingsim {

std::string FwdTableEntry::str() const {

	std::ostringstream out;
	out << outgoingPathID << " (";
		if (isUndefined()) out << '?';
		if (isValid()) out << 'V';
		if (isObsolete()) out << 'O';
		if (isInvalid()) out << 'I';
		if (isForeign()) out << 'F';
		if (isLocal()) out << 'L';
		if (isPrecalc()) out << 'P';
		if (isUsed()) out << 'U';
	        out << ':';

		switch (fwdAction) {
			case FwdTableEntry::Action::undefined:
				out << '_';
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
	out << "PIDFwdTable (F=" << getNumForeignEntries() << ",P=" << pused << '/' << ptotal << ",VL=" << getNumValidLocalEntries() << ",V=" << getNumValidEntries() << "): ";
	for (const auto& pidfwdtableentry : PIDFwdTable)
	{
		const FwdTableEntry &fwdentry = pidfwdtableentry.second;
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
// isLocal should only be set if we have a local contact with that path
// precalc is set for contacts within the vicinity discovery range
// @returns true if entry was newly added/created, otherwise false (for existing entry)
// the incomingPathID is returned
std::pair<bool, PathID>
FwdTablePathID::addEntry(const kad_path& path_to_add, bool isLocal, bool isForeignPID, bool precalc)
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
                pidentry->second.unsetObsolete();
                pidentry->second.setUsed();

		// set foreign or local flag
		if (isForeignPID)
			pidentry->second.setForeign();
		else {
			if (isLocal) { // only if we have a local contact
				pidentry->second.incrementReferenceCount();
                                pidentry->second.setLocal();
			}
		}

		if (precalc)
			pidentry->second.setPrecalc();

		EV_TRACE << "addEntry(): inPID entry " << incomingPathID << " exists already. " << pidentry->second.str() << endl;

		updates++;
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
		if (isLocal) { // only if we have a local contact
			fwdentry.incrementReferenceCount();
                        fwdentry.setLocal();
		}
	}

	if (precalc)
		fwdentry.setPrecalc();

	fwdentry.fwdAction=
		(path_to_add.size()==1) // next hop is our physical neighbor
		? FwdTableEntry::Action::fwdNextNodeId : FwdTableEntry::Action::fwdNextPathID;

	PIDFwdTable[incomingPathID]= fwdentry;

	// tpath: path including the target
	// EV_TRACE << "addEntry(): inPID: " << incomingPathID << " outPID: "
	// 	 << fwdentry.outgoingPathID << " tpath: " << path_to_add << " " << fwdentry.str() << endl;

	return std::make_pair(true, incomingPathID);
}



void
FwdTablePathID::delPrecalcEntry(const kad_path &path_to_remove) {
	if (path_to_remove.empty())
		return;
	PathID incomingPathID(getPathID(*ownNodeID, path_to_remove));
	// get PID
        if (auto pidentry = PIDFwdTable.find(incomingPathID);
            pidentry != PIDFwdTable.end()) {
		pidentry->second.unsetPrecalc();
		// PIDEntry may still be used by others, so let the regular routine decide about deletion
                delPIDEntry(incomingPathID);
        }

}


// this invalidates the PID entry that corresponds to the current path (i.e., path is known to be broken)
void FwdTablePathID::invalidateEntry(KadContact_p contact)
{
	if (contact == nullptr)
		return;

	// simply delete the NID to PID mapping
	delEntry(contact);

	// we have to check whether the PID part is actually affected by the broken link
	// kad_path currentpath(contact->getPath());
	// if (contact->getPathLength() <= nodeVicinityGraph::vicinityDiscoveryRadius) {
	// 	// add destination node ID to end of the path (will never be empty then)
	// 	currentpath.push_back(contact->getKey());
	// }
	// else {
	// 	if (contact->getPathLength() <= 2*nodeVicinityGraph::vicinityDiscoveryRadius) {
	// 		// for paths with 4 or 5 hops: stop at 3rd hop
	// 		currentpath.resize(nodeVicinityGraph::vicinityDiscoveryRadius+1);
	// 	}
	// 	else {
	// 		// stop three hops before destination
	// 		currentpath.resize(contact->getPathLength()-2);
	// 	}
	// }

	// // invalidateEntry is called by on ContactInvalidation
	// // need to determine whether the failure is in the first segment
	// // (this will affect probably also other contacts)

	// // prepend own nodeID for incoming PathID
	// PathID incomingPathID(getPathID(*ownNodeID, currentpath));

	// if (auto it= PIDFwdTable.find(incomingPathID); it != PIDFwdTable.end()) {
	// 	FwdTableEntry& fwdentry= it->second;
	// 	fwdentry.setInvalid();
	// }
	// else {
	// 	EV_ERROR << "no FwdTable entry found for PID " << incomingPathID << endl;
	// }
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

		//EV_TRACE << "modEntry(): Dest NodeID " << contact_key.shortStr() << " new PID path " << newpath << endl;
		updates++;
		// we probably want to keep the old PID in case there are still packets inflight
		priorPathID= pidentry->second;
		if ( PIDFwdTable.find(priorPathID) == PIDFwdTable.end() )
		{
			// this should not happen as some PID should have existed
			EV_ERROR << "FwdTablePathID::modEntry earlier PathID " << priorPathID << " not found" << endl;

			if (not isForeignPID and newpath.size() <= 2*nodeVicinityGraph::vicinityDiscoveryRadius)
				setNIDtoPIDmapping(contact_key, incomingPathID, lastsegmentPathID);

			return false;
		}
		else {
			if (priorPathID != incomingPathID)
			{ // this is a different PID than before, but maybe it exists already (will be handled below)
                          // set old PID entry to obsolete state if local update and not referenced anymore
				FwdTableEntry& priorfwdentry(PIDFwdTable[pidentry->second]);

				// mark it obsolete (may be still in use by foreign nodes)
				// if entry is local, we need to decrement the reference counter
				if (priorfwdentry.isLocal()) {
					if (priorfwdentry.decrementReferenceCount() == 0) {
						// last reference not a local entry anymore
						priorfwdentry.unsetLocal();
						if (not priorfwdentry.isPrecalc())
							priorfwdentry.setObsolete(); // obsolete means: no longer locally used (implies referenceCounter == 0)
						// note: entry may still be precalculated or required by foreign nodes
					}
				}
			}
			// else: priorPathID == incomingPathID
			// this will also be handled correctly below
		} // end else
	}

	FwdTableEntry fwdentry;
	// if PID already exists, just re-enable mapping and set to valid or modify flags
	if (auto it= PIDFwdTable.find(incomingPathID); it != PIDFwdTable.end())
	{
		//EV_TRACE << "modEntry(): inPID " << incomingPathID << " already existed" << endl;
		updates++;
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
		updates++;
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
	if (not isForeignPID)
		setNIDtoPIDmapping(contact_key, incomingPathID, lastsegmentPathID);

	EV_TRACE << "modEntry(): inPID:" << incomingPathID << " outPID:"
		 << fwdentry.outgoingPathID << " 2nd PID:" << lastsegmentPathID
		 << (additionalEntry ? " additional entry" : ((pidentry != NIDPIDTable.end()) ? " replacement" : " new"))
		 << " path: " << newpath << " Fwdentry:" << fwdentry.str() << endl;

	return true;
}


void FwdTablePathID::delPIDEntry(PathID pid)
{
	if (pid == PathID::UndefinedPathID)
		return;

	if ( auto it= PIDFwdTable.find(pid); it != PIDFwdTable.end() )
	{ // entry exists
		//EV_TRACE << "FwdTablePathID::delPIDEntry called for " << pid << " " << it->second.str() << endl;
		// delete all related PID entries (prior entries)

		// local and precalculated PID entries will not be deleted here,
		// foreign entries should be deleted by PathTeardown messages by unsetting their foreign bit first
		if ( not it->second.isLocal() and not it->second.isForeign() and not it->second.isPrecalc() and it->second.getReferenceCount()==0) {
			it->second.setDeleted();
			EV_TRACE << "FwdTablePathID::delPIDEntry PathID " << pid << " marked for deletion " << it->second.str() << endl;
			// entry cannot be deleted here, because there may be other foreign nodes
			// that require it still. Need to wait for garbage collection.
			return;
		}
		else { // Precalc or Foreign or still locally used
			EV_TRACE << "FwdTablePathID::delPIDEntry PathID " << pid << " not yet deleted." << it->second.str() << endl;
		}

		EV_TRACE << "FwdTablePathID::delPIDEntry finished for " << pid << " " << it->second.str() << endl;
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


// delete a local NID->PID entry and the corresponding PID
// currently called from onContactRemoval()
void FwdTablePathID::delEntry(KadContact_p contact)
{
       	if (contact == nullptr)
		return;

	// Lookup NID -> PID entry
	if ( auto pidentry= NIDPIDTable.find(contact->getKey()); pidentry != NIDPIDTable.end() )
	{
		// delete all PID entries related to this contact
                if (PathID currentPathID = pidentry->second; currentPathID != PathID::UndefinedPathID) {
			if (auto pid_it = PIDFwdTable.find(pidentry->second); pid_it != PIDFwdTable.end()) {
				FwdTableEntry& priorfwdentry= pid_it->second;
				if (priorfwdentry.isLocal()) {
					if (priorfwdentry.decrementReferenceCount() == 0) {
						// last reference not a local entry anymore
                                                priorfwdentry.unsetLocal();
						if (not priorfwdentry.isPrecalc())
							priorfwdentry.setObsolete(); // obsolete means: no longer locally used (implies referenceCounter == 0)
						// note: entry may still be precalculated or required by foreign nodes
					}
				}
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
	updates++;
}


void
FwdTablePathID::garbageCollect(std::unordered_map<PathID, std::unordered_set<OverlayKey>>& notifyFirstSegmentSources,
			       std::unordered_map<OverlayKey, std::pair<OverlayKey, PathID>>& notifyFirstSegmentSourcesReversePath,
			       const unsigned long fwdentry_refresh_timeout_ms)
{
	auto pwdtable_it= PIDFwdTable.begin();
	while ( pwdtable_it != PIDFwdTable.end() )
	{
		// not used entries are marked for deletion and also have lost their foreign bit
		if (not pwdtable_it->second.isLocal() and not pwdtable_it->second.isPrecalc()
		    and pwdtable_it->second.getAge() > fwdentry_refresh_timeout_ms) {
			EV_TRACE << "garbageCollect(): actually deleting PID "
				 << pwdtable_it->first << ": " << pwdtable_it->second.str() << endl;

                        if (auto nfss_it = notifyFirstSegmentSources.find(pwdtable_it->first);
                            nfss_it != notifyFirstSegmentSources.end()) {
				// need to iterate over all 1st segment source nodes in this entry
				for (auto srcnode : nfss_it->second) {
					notifyFirstSegmentSourcesReversePath.erase(srcnode);
				} // end for

			}
			// remove all 1st segment source nodes for this PID
			notifyFirstSegmentSources.erase(pwdtable_it->first);
			// remove PID
			pwdtable_it = PIDFwdTable.erase(pwdtable_it);
			updates++;
		}
		else {
			pwdtable_it++;
		}
	} // end while
}

} // end namespace
