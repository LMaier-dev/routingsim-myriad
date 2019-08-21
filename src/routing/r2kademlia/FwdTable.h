/******************************************************************
 * Forwarding Table Functionality
 *
 ******************************************************************/
#ifndef __ROUTINGSIM_FWDTABLE_H_
#define __ROUTINGSIM_FWDTABLE_H_

#include "foundation.h"
#include <BaseTypes.h>
#include <PathID.h>

#include <unordered_map>

namespace routingsim {

	// outgoing path ID and next hop
	using std::pair<PathID,nodeid_t> = FwdTableEntry;

	class FwdTableNodeID {
	public:
		std::unordered_map<OverlayKey,FwdTableEntry> NIDFwdTable;
	};


	clase FwdTablePathID {
	public:
		std::unordered_map<PathID,FwdTableEntry>;
	}
} // end namespace

#endif // __ROUTINGSIM_FwdTable_H_
