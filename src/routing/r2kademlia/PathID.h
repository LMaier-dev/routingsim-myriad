/******************************************************************
 * PathID type for Forwarding
 ******************************************************************/
#ifndef __ROUTINGSIM_PathID_H_
#define __ROUTINGSIM_PathID_H_

#include "OverlayKey.h"
#include "KadPath.h"

namespace routingsim {

constexpr uint64_t UndefinedPathID_u64= 0x0ULL;
// just a random seed for the XXhash
constexpr uint64_t PathIDseed= 0xfe246b05927bd2d7ULL;

class PathID {

public:
	static const PathID UndefinedPathID;
	
	uint64_t pathId;
	
        PathID(uint64_t pid= UndefinedPathID_u64) : pathId(pid) {};
	PathID(const kad_path& path);
	bool isUndefined() const { return pathId == UndefinedPathID_u64; };
        operator uint64_t() const { return pathId; };
	//bool operator==(const PathID& other) const { return pathId == other.pathId; }

};

std::ostream& operator<<(std::ostream& os, const PathID& pid);

} // end namespace


namespace std {
        template<> class hash<routingsim::PathID>
        {
        public:
                size_t operator()(const routingsim::PathID& k) const;
        };
}

inline std::size_t std::hash<routingsim::PathID>::operator()(const routingsim::PathID& p) const {
	return p.pathId;
}



#endif // __ROUTINGSIM_PathID_H_
