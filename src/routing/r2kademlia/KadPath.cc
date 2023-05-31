#include "KadPath.h"
#include "xxh3.h"

namespace routingsim {

std::ostream& operator<<(std::ostream& os, const link_t& l)
{
	os << '<' << l.first.shortStr() << ',' << l.second.shortStr() << '>';
	return os;
}


// print a path using abbreviated key ids
std::ostream& operator<<(std::ostream& os, const kad_path& p)
{
	os << '<';
	if (p.size() == 0)
		 os << '-';
	else {
		for (const auto &key : p)
			os << ' ' << key.shortStr();
		os << ' ';
	}
	os << '>';
	return os;
}


// some paths exclude source and target nodes, e.g., those stored for KadContacts
bool containsUndirectedLink(const OverlayKey& source, const OverlayKey& target, const kad_path& path, const link_t link)
{
	if (path.size() == 0) { // empty path
		return (link.first == source and link.second == target) or (link.first == target and link.second == source);
	}
	else
	{
		// check for first link (PN)
		if ( (link.first == source and link.second == path[0]) or (link.first == path[0] and link.second == source) )
			return true;
	        else
			return (target == link.first and path[path.size()-1] == link.second) // pen-ultimate hop is neighbor node of failed contact => this link failed
				or
				(target == link.second and path[path.size()-1] == link.first) // pen-ultimate hop is the failed node leading to the neighboring node => this link failed
				or
				containsUndirectedLink(path, link.first, link.second);
	}
}


/// searches for undirected link in given path, returns true if found, false if not found
bool containsUndirectedLink(const kad_path& path, const OverlayKey& firstNodeKey, const OverlayKey& secondNodeKey)
{
	if (path.size() >= 2) // only for paths that contain links
	{ // now loop over all nodes contained in the path
		for (unsigned int i= 0; i != path.size(); i++)
		{
			// need at least one more node in the path for link testing
			if ((i+1) < path.size())
			{
				if ( path[i+1] == secondNodeKey ) {
					if (path[i] == firstNodeKey)
						return true;
				}
				else
				if ( path[i+1] == firstNodeKey ) {
					if (path[i] == secondNodeKey)
						return true;
				}
				else // the node i+1 cannot be part of the link, advance one more
					i++; // so now i < path.size() still holds
			}
			else
				return false;
		} // end for
		// if here: for loop is finished, but not found so far
		return false;
	}
	else
	{
		return false;
	}
}


// calculates XXH3 hashsum of path and destkey
// the hashsum is returned as OverlayKey to be able to calculate the XOR distance
OverlayKey HashSum(const OverlayKey& seed, const kad_path& path, const OverlayKey& destkey)
{
	XXH3_state_t* state= XXH3_createState();
	XXH3_128bits_reset_withSeed(state, seed.toUint64_t());
	// hash all keys of the given path
	const auto keylen(OverlayKey::getLength() / 8 + ((OverlayKey::getLength() % 8) != 0 ? 1 : 0));
	for (const auto& key : path)
	{
		XXH3_128bits_update(state, key.getRawKey(), keylen);
	}

	XXH3_128bits_update(state, destkey.getRawKey(), keylen);
	XXH128_hash_t result = XXH3_128bits_digest(state);
	XXH3_freeState(state);
	return OverlayKey(result.high64, result.low64);
}

}
