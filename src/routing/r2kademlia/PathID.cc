#include "PathID.h"
#include "SHA1.h"

namespace routingsim {

const PathID PathID::UndefinedPathID= PathID();

PathID::PathID(const kad_path& path)
{
    if (path.size() > 0)
    {
	    uint8_t temp[20]; // SHA1 result
	    CSHA1 sha1;
	    
	    sha1.Reset();
	    // hash all keys of the given path
	    const auto keylen(OverlayKey::getLength() / 8 + ((OverlayKey::getLength() % 8) != 0 ? 1 : 0));
	    for (const auto& key : path)
	    {
		    sha1.Update(key.getRawKey(), keylen);
	    }
	    sha1.Final();
	    sha1.GetHash(temp);

	    // we only need 64 bits for PathId
	    memcpy((uint8_t*) &pathId, temp, sizeof(pathId));
    }
}


std::ostream& operator<<(std::ostream& os, const PathID& pid)
{
	std::ios_base::fmtflags f( os.flags() );  // save flags state
	os << std::hex << pid.pathId;
	os.flags( f );  // restore flags state
	return os;
}


PathID getPathID(const kad_path& path)
{
    PathID returnPathID;
    if (path.size() > 0)
    {
	    uint8_t temp[20]; // SHA1 result
	    CSHA1 sha1;
	    // optimization: assumption is that all keys have the same length
	    const auto keylen(OverlayKey::getLength() / 8 + ((OverlayKey::getLength() % 8) != 0 ? 1 : 0));
	    
	    sha1.Reset();
	    // hash all keys of the given path
	    for (const auto& key : path)
	    {
		    sha1.Update(key.getRawKey(), keylen);
	    }
	    sha1.Final();
	    sha1.GetHash(temp);
	    
	    // we only need 64 bits for PathId
	    memcpy((uint8_t*) &returnPathID.pathId, temp, sizeof(returnPathID.pathId));
    }

    return returnPathID;
}


PathID getPathID(const OverlayKey& prependKey, const kad_path& path)
{
    PathID returnPathID;
    if (path.size() > 0)
    {
	    uint8_t temp[20]; // SHA1 result
	    CSHA1 sha1;
	    const auto keylen(OverlayKey::getLength() / 8 + ((OverlayKey::getLength() % 8) != 0 ? 1 : 0));
	    
	    sha1.Reset();
	    sha1.Update(prependKey.getRawKey(), keylen);
	    // hash all keys of the given path
	    for (const auto& key : path)
	    {
		    sha1.Update(key.getRawKey(), keylen);
	    }
	    sha1.Final();
	    sha1.GetHash(temp);
	    
	    // we only need 64 bits for PathId
	    memcpy((uint8_t*) &returnPathID.pathId, temp, sizeof(returnPathID.pathId));
    }

    return returnPathID;
}

	
} // end namespace
