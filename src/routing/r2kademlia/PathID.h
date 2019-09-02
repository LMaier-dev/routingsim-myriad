/******************************************************************
 * PathID type for Forwarding
 ******************************************************************/
#ifndef __ROUTINGSIM_PathID_H_
#define __ROUTINGSIM_PathID_H_

#include "OverlayKey.h"

namespace routingsim {

using kad_path = std::vector<OverlayKey>;
//using uint64_t =  unsigned long long;

constexpr uint64_t UndefinedPathID= 0x0ULL;


class PathID {

public:

	uint64_t pathId;
	
        PathID(uint64_t pid= UndefinedPathID) : pathId(pid) {};
	PathID(const kad_path& path);
	bool isUndefined() const { return pathId == UndefinedPathID; };
	uint64_t operator()() const { return pathId; };

};


std::ostream& operator<<(std::ostream& os, const PathID& pid);


PathID getPathID(const kad_path& path);

PathID getPathID(const OverlayKey& prependKey, const kad_path& path);


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
