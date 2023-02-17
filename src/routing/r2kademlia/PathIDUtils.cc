#include "PathIDUtils.h"
#include "NodeVicinityGraph.h"
#include "SHA1.h"

namespace routingsim {

// the given path will be split into two segments and this function
// will return the incoming PIDs for each segment
// it is assumed that the path is includes the destination (excludes the source)
std::tuple<PathID, PathID>
getSegmentPathIDs(const OverlayKey ownNodeID, const kad_path& path)
{
	// note: path length = number of nodes = hops + 1
	// if path is <= 3 hops, we have no second segment
	if (path.size() <= nodeVicinityGraph::vicinityDiscoveryRadius+1)
		return {getPathID(ownNodeID, path),PathID::UndefinedPathID};
	else { // we have two segments
		kad_path firstpathsegment(path);
		
		// for paths with length 4 or 5 hops (length 5 or 6) the second segment is only 2 hops long (length 3)
		auto secondpathsegment_begin_index= path.size() - nodeVicinityGraph::vicinityDiscoveryRadius;
		// for longer paths, the last segment is one hop longer
		if (path.size() > (2*nodeVicinityGraph::vicinityDiscoveryRadius+1))
			secondpathsegment_begin_index--;

		// copy the rest into second segment
		kad_path secondpathsegment(path.size()-secondpathsegment_begin_index);
		for (unsigned long i= 0; i < secondpathsegment.size(); i++)
			secondpathsegment.at(i)= firstpathsegment.at(secondpathsegment_begin_index+i);
		// now cut first segment
		firstpathsegment.resize(secondpathsegment_begin_index+1);

		EV_TRACE << "Whole path: " << path << " First segment: " << firstpathsegment << " Second segment: " << secondpathsegment << endl;

		return {getPathID(ownNodeID, firstpathsegment),getPathID(secondpathsegment)};
	}
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
