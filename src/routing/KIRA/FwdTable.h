/******************************************************************
 * Forwarding Table Functionality
 *
 ******************************************************************/
#ifndef __ROUTINGSIM_FWDTABLE_H_
#define __ROUTINGSIM_FWDTABLE_H_

#include <BaseTypes.h>
#include <unordered_map>
#include <tuple>
#include <functional>
#include <unordered_set>

#include "KadContact.h"
#include "PathID.h"

namespace routingsim {

class FwdTableEntry {
private:
	uint8_t fwdState = State::undefined;
	uint16_t pathlen = 0; // default: number of hops to destination
	unsigned long usageCount = 0;
	unsigned long referenceCount = 0;
	simtime_t last_touched = SIMTIME_ZERO; // age for soft state refresh in ms
	bool used = false;
public:
	PathID outgoingPathID = PathID::UndefinedPathID;
	OverlayKey nextHopNode = OverlayKey::UNSPECIFIED_KEY;
	PathID prevPathID = PathID::UndefinedPathID;

	enum State : uint8_t {
		undefined     = 0,
		valid         = 1, // path intact so far
		local         = 2, // locally wanted (contact or PN)
		foreign       = 4, // PID wanted by KadPathSetup
		obsolete      = 8, // if overwritten by a newer entry
		precalc       = 16, // typically all paths in the vicinity discovery radius
		deleted       = 32  // if explicitly marked for deletion
	};

	// forwarding actions
	enum class Action {
		undefined     = 0,
		fwdNextPathID = 1, // forward with new outgoing PathID
		fwdNextNodeId = 2, // forward to next NodeID (must be direct neighbor)
		fwdlocal      = 3
	} fwdAction = Action::undefined;

	void refresh() { last_touched= simTime(); }
	unsigned long getAge() const { return (simTime() - last_touched).inUnit(SIMTIME_MS); }
	simtime_t getLastTouched() const { return last_touched; }

	void unsetObsolete() { fwdState &= ~State::obsolete; refresh(); }
	void setObsolete() { fwdState |= State::obsolete; refresh(); }
	bool isObsolete() const { return fwdState & State::obsolete; }

	void setInvalid() { fwdState &= ~State::valid; refresh(); }
	bool isInvalid() const { return (fwdState & State::valid) == 0; }

	void setValid() { fwdState |= State::valid; refresh(); }
	bool isValid() const { return fwdState & State::valid; }

	void unsetForeign() { fwdState &= ~State::foreign; refresh(); };
	void setForeign() { if (not isPrecalc()) fwdState |= State::foreign; refresh(); };
	bool isForeign() const { return fwdState & State::foreign; };

	void unsetLocal() { fwdState &= ~State::local; refresh(); };
	void setLocal() { fwdState |= State::local; refresh(); };
	bool isLocal() const { return fwdState & State::local; };

	void unsetPrecalc() { fwdState &= ~State::precalc; refresh(); };
	void setPrecalc() { fwdState |= State::precalc; fwdState &= ~State::foreign; refresh(); };
	bool isPrecalc() const { return fwdState & State::precalc; };

	void unsetDeleted() { fwdState &= ~State::deleted; refresh(); };
	void setDeleted() { fwdState |= State::deleted; refresh(); };
	bool isDeleted() const { return fwdState & State::deleted; };

	bool isUndefined() const { return fwdState == State::undefined; }

	void setPathLength(uint16_t plen) { pathlen= plen; }
	uint16_t getPathLength() const { return pathlen; }

	void incrementUsageCount() { usageCount++; }
	unsigned long getUsageCount() const { return usageCount; }

	unsigned long incrementReferenceCount() {
		//EV_TRACE << "incrementReferenceCount(): " << str() << endl;
		return ++referenceCount; }
	unsigned long decrementReferenceCount() { 
		if (referenceCount > 0) {
			//EV_TRACE << "decrementReferenceCount(): " << str() << endl;

			if (referenceCount == 1)
				EV_TRACE << "Reference Count to ZERO: " << str() << endl;
			return --referenceCount;
		}
		else {
			EV_WARN << "Reference Count Violation: " << str() << endl;
			ASSERT(referenceCount > 0);
			return referenceCount;}
	}
	unsigned long getReferenceCount() const { return referenceCount; }

	bool isUsed() const { return used; }
	void setUsed() { used= true; }

	std::string str() const;
};


class FwdTablePathID : public cObject {
public:
	// This stores incoming PathID -> PathID/NextHop mappings plus state and action
	std::unordered_map<PathID,FwdTableEntry> PIDFwdTable;
	// This stores NodeID to incoming PathID mapping
	// (outgoing PID for the first segment can be determined by lookup in PIDFwdTable)
	std::unordered_map<OverlayKey,PathID> NIDPIDTable;
	// This stores NodeID to second segment PathID (which will start at a different node)
	std::unordered_map<OverlayKey,PathID> NIDPIDTableLastSeg;
	// cannot use constructor to set ID, because OnInitialize generates the Node ID
	void setOwnNodeID(const OverlayKey* myownNodeID) { ownNodeID= myownNodeID; };
	// returns true if exactly this PID exists for path to contact
	bool existsEntry(KadContact_p contact);

	inline void setNIDtoPIDmapping(const OverlayKey& nid, const PathID incomingPathID, const PathID LastSegmentPathID= PathID::UndefinedPathID);
	inline void eraseNIDtoPIDmapping(const OverlayKey& nid);
	std::pair<bool, PathID> addPrecalcEntry(const kad_path& path_to_add) { return addEntry(path_to_add, false, false, true); }
	std::pair<bool, PathID> addForeignEntry(const kad_path& path_to_add) { return addEntry(path_to_add, false, true, false); }
	std::pair<bool, PathID> addLocalEntry(const kad_path& path_to_add, bool precalc) { return addEntry(path_to_add, true, false, precalc); }
	void delPrecalcEntry(const kad_path &path_to_remove);

	// modify PID for path to contact
	bool modEntry(const OverlayKey& contact_key, const kad_path& newpath, bool foreignPID);
	void delEntry(KadContact_p contact);
	void invalidateEntry(KadContact_p contact);

	void delPIDEntry(PathID pid);
	void delForeignPIDEntry(PathID pid);

	// garbage collect, actually delete unnecessary entries
        void garbageCollect(std::unordered_map<PathID, std::unordered_set<OverlayKey>> &notifyFirstSegmentSources,
			    std::unordered_map<OverlayKey, std::pair<OverlayKey, PathID>> &notifyFirstSegmentSourcesReversePath,
			    const unsigned long fwdentry_refresh_timeout_ms);

	// for gathering statistics
	size_type getNumEntries() const { return PIDFwdTable.size()+NIDPIDTable.size()+NIDPIDTableLastSeg.size(); };
	inline size_type getNumSelectedEntries(std::function<bool(const FwdTableEntry& fwdentry)> selectorFn) const;
	size_type getNumValidEntries() const;
	size_type getNumValidLocalEntries() const;
	size_type getNumForeignEntries() const;
	std::tuple<size_type, size_type> getNumUsedPrecalcEntries() const;
	size_type getNumUpdates() const { return updates; };

	// from cObject()
	virtual std::string str() const;
protected:
	// try to add PID for path to contact (needs to be included in path_to_add), returns incomingPID or undefined on failure
	std::pair<bool, PathID> addEntry(const kad_path& path_to_add, bool isLocal, bool foreignPID, bool precalc);

private:
	const OverlayKey *ownNodeID = nullptr;
	size_type updates = 0;
};


// map an NID to an (incoming) PID, note that several NIDs may be mapped to the same PID,
// because the rest of the path to the viacontact may diverge then 
inline
void
FwdTablePathID::setNIDtoPIDmapping(const OverlayKey& nid, const PathID incomingPathID, const PathID LastSegmentPathID) {
	if (auto it = NIDPIDTable.find(nid); it != NIDPIDTable.end()) {
		// entry exists already, delete old PID
                delPIDEntry(it->second);
		// replace with new one
                it->second= incomingPathID;
        }
	else {
		NIDPIDTable.insert({nid,incomingPathID});
	}
	// update last segment
	if (LastSegmentPathID != PathID::UndefinedPathID)
		NIDPIDTableLastSeg[nid]= LastSegmentPathID;
	else
		NIDPIDTableLastSeg.erase(nid);
}


inline
void
FwdTablePathID::eraseNIDtoPIDmapping(const OverlayKey& nid) {
	//EV_TRACE << "erasing NID to PID mappings for " << nid.shortStr() << endl;
	NIDPIDTable.erase(nid);
	NIDPIDTableLastSeg.erase(nid);
}


// this counts the number of forwarding table lines
// i.e., one for the PID-PID Table (PIDFwdTable) and one for the NID-PID Table
// (NIDPIDTable) (even if a second segment is necessary, i.e., NIDPIDTableLastSeg is ignored)
inline size_type FwdTablePathID::getNumSelectedEntries(std::function<bool(const FwdTableEntry& fwdentry)> selectorFn) const
{
	unsigned long sum= 0;
	for (auto& pidfwdtableentry : PIDFwdTable)
	{
		if ( selectorFn(pidfwdtableentry.second) == true )
			sum++;
        }
        // add local entries
        for (auto &nidpidtableentry : NIDPIDTable) {
	  // first segments
          if (auto it = PIDFwdTable.find(nidpidtableentry.second);
              it != PIDFwdTable.cend()) {
	    // count this entry if status is ok for selector function
	    if ( selectorFn(it->second) == true ) {
              sum++;
	    }
	  }
	} // end for

	return sum;
}


inline
size_type FwdTablePathID::getNumValidEntries() const
{
	// this counts only valid and active entries (=non obsolete)
	return getNumSelectedEntries([](const FwdTableEntry& fwdentry)->bool{ return fwdentry.isValid() and not(fwdentry.isObsolete()); });
};


inline
size_type FwdTablePathID::getNumValidLocalEntries() const
{
	return getNumSelectedEntries([](const FwdTableEntry& fwdentry)->bool{ return fwdentry.isValid() and not(fwdentry.isObsolete()) and fwdentry.isLocal(); });
};


inline
size_type FwdTablePathID::getNumForeignEntries() const {
	// this counts only foreign valid and active entries (=non obsolete)
	// it may be the case that foreign entries are also precalculated or locally required, we do not count them as foreign then
	return getNumSelectedEntries([](const FwdTableEntry& fwdentry)->bool{ return fwdentry.isForeign() and fwdentry.isValid() and  not(fwdentry.isPrecalc() or fwdentry.isLocal() or fwdentry.isObsolete()); });
};

} // end namespace

#endif // __ROUTINGSIM_FwdTable_H_
