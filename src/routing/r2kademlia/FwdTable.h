/******************************************************************
 * Forwarding Table Functionality
 *
 ******************************************************************/
#ifndef __ROUTINGSIM_FWDTABLE_H_
#define __ROUTINGSIM_FWDTABLE_H_

#include "foundation.h"
#include <BaseTypes.h>
#include <unordered_map>

#include "KadContact.h"
#include "PathID.h"

namespace routingsim {

	class FwdTableEntry {
	private:
		uint8_t fwdState = State::undefined;
                uint16_t pathlen = 0; // default: number of hops to destination
	public:
		PathID outgoingPathID = PathID::UndefinedPathID;
		OverlayKey nextHopNode = OverlayKey::UNSPECIFIED_KEY;
		PathID prevPathID = PathID::UndefinedPathID;

		enum State : uint8_t {
			undefined     = 0,
			valid         = 1,
			foreign       = 2,
			obsolete      = 4,
			invalid       = 8
		};

		enum class Action {
		     undefined     = 0,
		     fwdNextPathID = 1,
		     fwdNextKadId  = 2,
		     fwdlocal      = 3
		} fwdAction = Action::undefined;

		void setObsolete() { setInvalid(); fwdState |= State::obsolete; }
		bool isObsolete() const { return fwdState & State::obsolete; }

		void setInvalid() { fwdState &= ~State::valid; fwdState |= State::invalid; }
		bool isInvalid() const { return fwdState & State::invalid; }

		void setValid() { fwdState &= ~State::invalid; fwdState |= State::valid; }
		bool isValid() const { return fwdState & State::valid; }

		void setForeign() { fwdState = fwdState | State::foreign; };
		bool isForeign() const { return fwdState & State::foreign; };

		bool isUndefined() const { return fwdState == State::undefined; }


		void setPathLength(uint16_t plen) { pathlen= plen; }
		uint16_t getPathLength() const { return pathlen; }
	};


	class FwdTableNodeID {
	public:
		std::unordered_map<OverlayKey,FwdTableEntry> NIDFwdTable;
	};


	class FwdTablePathID : public cObject {
	public:
		// This stores incoming PathID -> PathID/NextHop mappings plus state and action
		std::unordered_map<PathID,FwdTableEntry> PIDFwdTable;
		// This stores NodeID to incoming PathID mapping
		// (outgoing PID can be determined by lookup in PIDFwdTable)
		std::unordered_map<OverlayKey,PathID> NIDPIDTable;
		// cannot use constructor to set ID, because OnInitialize generates the Node ID
	        void setOwnNodeID(const OverlayKey* myownNodeID) { ownNodeID= myownNodeID; };
		void addEntry(KadContact_p contact, bool foreignPID);
		bool modEntry(KadContact_p contact, bool foreignPID);
		void delEntry(KadContact_p contact);
		void delPIDEntry(PathID pid);

		// for gathering statistics
		size_type getNumEntries() const { return PIDFwdTable.size(); };
		size_type getNumValidEntries() const;
		size_type getNumForeignEntries() const;

		// from cObject()
		virtual std::string str() const;

	private:
		const OverlayKey* ownNodeID = nullptr;
	};
	
} // end namespace

#endif // __ROUTINGSIM_FwdTable_H_
