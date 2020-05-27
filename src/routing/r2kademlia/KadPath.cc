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
	if (path.size() >= 2) // only for path that contain links
	{
		const OverlayKey* prev_node= &path[0];
		// now loop over all nodes contained in the path
		for (unsigned int i= 0; i != path.size(); i++)
		{
			if (path[i] == firstNodeKey)
			{ // one end of the link detected
				if (*prev_node == secondNodeKey)
				{ // previous node is the other end of the link
					return true;
				}
				else
				{ // check if next hop is other end of the link
					if ((i+1) < path.size())
					{ // next hop available
						if (path[i+1] == secondNodeKey) // next hop is second node
							return true;
					}
					else
					{ // current node is last hop -> no link found
						return false;
					}
				}
			}
			// current node will be prev_node in next iteration
			prev_node= &path[i];
		} // end for
		// if here: for loop is finished, but not found so far
		return false;
	}
	else
	{
		return false;
	}
}

}
