#include "KadPath.h"

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

// print a path using abbreviated key ids and marked overlay hops
std::ostream &operator<<(std::ostream &os,
                         std::pair<const kad_path &, const std::vector<bool>&> path_and_active) {
	auto [p,a]= path_and_active;
	os << '<';
	if (p.size() == 0)
		 os << '-';
        else {
		ASSERT(p.size() == a.size());
		auto a_it= a.cbegin();
                for (const auto &key : p) {
			if (a_it != a.cend()) {
				os << (*a_it ? " *" : " ") << key.shortStr();
				a_it++;
			}
		}
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


/// Counts the number of common links between this path and the
/// passed otherPath.
/// Example:
/// <A, B, C, D> and <A, C, D, E, F> returns 2, as the link to A
/// and the link C->D are in both paths.
uint KadPath::commonLinks(const KadPath& otherPath) const
{
    if (length() == 0 or otherPath.length() == 0) return 0;

    uint cl = 0;
    // for each node in path A
    for (unsigned int i = 0; i < length() - 1; i++)
    {
        // for each node in path B
        for (unsigned int j = 0; j < otherPath.length() - 1; j++)
        {
            // see if it and the next nodes in both paths are identical
            // -> same link
            if ( path[i] == otherPath[j] ) {
                if (path[i+1] == otherPath[j+1])
                    ++cl;
            }
            else
            if ( path[i+1] == otherPath[j] ) {
                // or if they match crossed over
                if (path[i] == otherPath[j+1] )
                    ++cl;
            }
        }
    }

    // check if both paths use the same link to our PN
    if (length() > 0 and otherPath.length() > 0) {
        if (path[0] == otherPath[0]) {
            ++cl;
        }
    }

    // check if both paths use the same last hop to the contact
    if (length() > 1 or otherPath.length() > 1) {
        uint pos = length() > 1 ? length()-1 : 0;
        uint posOther = otherPath.length() > 1 ? otherPath.length()-1 : 0;
        if (path[pos] == otherPath[posOther]) {
            ++cl;
        }
    }

    return cl;
}


/// Returns number of common links AND common nodes
/// between this and the other path.
/// Example:
/// <A, B, C, D> and <A, C, D, E, F> returns 5, as the link to A
/// and the link C->D are in both paths and both paths contain nodes
/// A, C and D.
uint KadPath::commonLinksAndNodes(const KadPath& otherPath) const
{
    uint cl = commonLinks(otherPath);
    uint cn = 0;
    for (unsigned int i = 0; i < length(); i++)
    {
        for (unsigned int j = 0; j < otherPath.length(); j++)
        {
            if (path[i] == otherPath[j]) {
                ++cn;
            }
        }
    }

    return cl + cn;
}


char *tmpHashSumBuf= nullptr;
unsigned long tmpHashSumBufSize= 0;

// calculates XXH3 hashsum of path and destkey
// the hashsum is returned as OverlayKey to be able to calculate the XOR distance
OverlayKey HashSum(const OverlayKey& seed, const kad_path& path, const OverlayKey& destkey)
{
  constexpr auto keylen(OverlayKey::getLength() / 8 + ((OverlayKey::getLength() % 8) != 0 ? 1 : 0));
  if (tmpHashSumBufSize < (path.size()+1)*keylen) {
	  delete[] tmpHashSumBuf;
	  tmpHashSumBufSize= (path.size()+1)*keylen;
	  tmpHashSumBuf= new char[tmpHashSumBufSize];
  }

  size_t offset= 0;
  for (auto &key : path) {
    memcpy(tmpHashSumBuf+offset, key.getRawKey(), keylen);
    offset+= keylen;
  }
  memcpy(tmpHashSumBuf+offset, destkey.getRawKey(), keylen);
  offset+= keylen;
  XXH128_hash_t result= XXH3_128bits_withSeed(tmpHashSumBuf, offset, seed.toUint64_t());

  return OverlayKey(result.high64, result.low64);
}

}
