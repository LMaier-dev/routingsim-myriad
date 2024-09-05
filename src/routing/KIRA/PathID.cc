#include "PathID.h"
#include <iomanip>

namespace routingsim {

const PathID PathID::UndefinedPathID= PathID();
char *PathID::tmppathIDbuf= nullptr;
unsigned long PathID::tmppathIDbufSize= 0;

PathID::PathID(const kad_path& path)
{
    if (path.size() > 0)
    {
	    constexpr auto keylen(OverlayKey::getLength() / 8 + ((OverlayKey::getLength() % 8) != 0 ? 1 : 0));
	    // if temporary buffer too small, reallocate
	    if (PathID::tmppathIDbufSize < path.size()*keylen) {
		    delete[] PathID::tmppathIDbuf;
		    PathID::tmppathIDbufSize= path.size()*keylen;
		    PathID::tmppathIDbuf= new char[PathID::tmppathIDbufSize];
	    }

	    size_t offset= 0;
	    for (auto &key : path) {
		    memcpy(PathID::tmppathIDbuf+offset, key.getRawKey(), keylen);
		    offset+= keylen;
	    }
	    XXH128_hash_t result= XXH3_128bits_withSeed(PathID::tmppathIDbuf, offset, PathIDseed);
	    pathId= result.high64 ^ result.low64;
    }
}


std::ostream& operator<<(std::ostream& os, const PathID& pid)
{
	std::ios_base::fmtflags f( os.flags() );  // save flags state
	os << std::hex << std::setfill('0') << std::setw(16) << pid.pathId;
	os.flags( f );  // restore flags state
	return os;
}

	
} // end namespace
