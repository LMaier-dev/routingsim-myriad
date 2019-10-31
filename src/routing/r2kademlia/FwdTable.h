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
	public:
		PathID outgoingPathID;
		OverlayKey nextHopNode;
		
		enum class State {
			undefined     = 0,
			valid         = 1,
			obsolete      = 2,
			invalid       = 3
		} fwdState = State::undefined;
		
		enum class Action {
		     undefined     = 0,
		     fwdNextPathID = 1,
		     fwdNextKadId  = 2,
		     fwdlocal      = 3
		} fwdAction = Action::undefined;
	};
	

	class FwdTableNodeID {
	public:
		std::unordered_map<OverlayKey,FwdTableEntry> NIDFwdTable;
	};


	class FwdTablePathID : public cObject {
	public:
		std::unordered_map<PathID,FwdTableEntry> PIDFwdTable;
		// cannot use constructor to set ID, because OnInitialize generates the Node ID
	        void setOwnNodeID(const OverlayKey* myownNodeID) { ownNodeID= myownNodeID; };
		void addEntry(KadContact_p contact);

		// from cObject()
		virtual std::string str() const;

	private:
		const OverlayKey* ownNodeID = nullptr;
	};
	
} // end namespace

#endif // __ROUTINGSIM_FwdTable_H_
