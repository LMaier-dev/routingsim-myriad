#include "PathIDUtils.h"
#include "NodeVicinityGraph.h"

namespace routingsim {

// the given path (must include the destination ID) will be split into two segments and this function
// returns the incoming PID and outgoing PID for the first segment and the incoming PID for the second segment
// it is assumed that the path includes the destination (but excludes the source)
std::tuple<PathID, PathID, PathID>
getSegmentPathIDs(const OverlayKey ownNodeID, const kad_path& path)
{
	// note: path length = number of nodes = hops + 1
	// if path is <= 3 hops, we have no second segment
	if (path.size() <= nodeVicinityGraph::vicinityDiscoveryRadius+1)
		return {getPathID(ownNodeID, path), getPathID(path), PathID::UndefinedPathID};
	else { // we have two segments
		kad_path firstpathsegment(path);
		
		// for paths with length 4 or 5 hops the second segment is only 2 hops long (length 3)
		auto secondpathsegment_begin_index= path.size() - nodeVicinityGraph::vicinityDiscoveryRadius - 1;
		// for longer paths, the last segment is one hop longer
		if (path.size() > 2*nodeVicinityGraph::vicinityDiscoveryRadius+1)
			secondpathsegment_begin_index--;

		// copy the rest into second segment
		kad_path secondpathsegment(path.size()-secondpathsegment_begin_index);
		for (unsigned long i= 0; i < secondpathsegment.size(); i++)
			secondpathsegment.at(i)= firstpathsegment.at(secondpathsegment_begin_index+i);
		// now cut first segment
		firstpathsegment.resize(secondpathsegment_begin_index+1);

		//EV_TRACE << "Whole path: " << path << " First segment: " << firstpathsegment << " Second segment: " << secondpathsegment << endl;

		return {getPathID(ownNodeID, firstpathsegment), getPathID(firstpathsegment), getPathID(secondpathsegment)};
	}
}

PathID getPathID(const OverlayKey& prependKey, const kad_path& path)
{
	if (path.size() > 0) {
		constexpr auto keylen(OverlayKey::getLength() / 8 + ((OverlayKey::getLength() % 8) != 0 ? 1 : 0));
		// if temporary buffer too small, reallocate
		if (PathID::tmppathIDbufSize < (path.size()+1)*keylen) {
			delete[] PathID::tmppathIDbuf;
			PathID::tmppathIDbufSize= (path.size()+1)*keylen;
			PathID::tmppathIDbuf= new char[PathID::tmppathIDbufSize];
		}

		size_t offset= 0;
		memcpy(PathID::tmppathIDbuf, prependKey.getRawKey(), keylen);
		offset+= keylen;
		for (auto &key : path) {
			memcpy(PathID::tmppathIDbuf+offset, key.getRawKey(), keylen);
			offset+= keylen;
		}
		XXH128_hash_t result= XXH3_128bits_withSeed(PathID::tmppathIDbuf, offset, PathIDseed);

		return {result.high64 ^ result.low64};
	}
	else
		return {};
}


} // end namespace
