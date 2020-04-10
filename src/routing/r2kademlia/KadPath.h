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

std::ostream& operator<<(std::ostream& os, const kad_path& c);
}

#endif // __ROUTINGSIM_KadPath_H_
