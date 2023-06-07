#ifndef __ROUTINGSIM_KadPath_H_
#define __ROUTINGSIM_KadPath_H_

#include "OverlayKey.h"
#include <sstream>

namespace routingsim {

using kad_path = std::vector<OverlayKey>;
using link_t = std::pair<OverlayKey,OverlayKey> ;


/// searches for undirected link in constructed path: <source + path + target>, returns true if found, false if not found
/// some paths exclude source and target nodes, e.g., those stored for KadContacts
bool containsUndirectedLink(const OverlayKey& source, const OverlayKey& target, const kad_path& path, const link_t link);

bool containsUndirectedLink(const kad_path& path, const OverlayKey& firstNodeKey, const OverlayKey& secondNodeKey);

inline
bool are_equal_r(const kad_path& first_path, const kad_path& second_path) {
	auto it_second= second_path.crbegin();
	for (auto it_first= first_path.crbegin(); it_first != first_path.crend(); it_first++) {
		if ( *it_first != *it_second )
			return false;
		it_second++;
	}
	return true;
};

// calculates an XXH3 hash of path and destkey
OverlayKey HashSum(const OverlayKey& seed, const kad_path& path, const OverlayKey& destkey);

std::ostream& operator<<(std::ostream& os, const kad_path& c);

std::ostream& operator<<(std::ostream& os, const link_t& l);
}

#endif // __ROUTINGSIM_KadPath_H_
