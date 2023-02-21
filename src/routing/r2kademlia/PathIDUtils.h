#ifndef __ROUTINGSIM_R2KademliaPathUtils_H_
#define __ROUTINGSIM_R2KademliaPathUtils_H_

#include "PathID.h"

namespace routingsim {

std::tuple<PathID, PathID, PathID> getSegmentPathIDs(const OverlayKey ownNodeID, const kad_path& path);

PathID getPathID(const kad_path& path);

PathID getPathID(const OverlayKey& prependKey, const kad_path& path);

}

#endif
