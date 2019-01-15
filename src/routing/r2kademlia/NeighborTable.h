//
// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU Lesser General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
// 
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU Lesser General Public License for more details.
// 
// You should have received a copy of the GNU Lesser General Public License
// along with this program.  If not, see http://www.gnu.org/licenses/.
// 

#ifndef __ROUTINGSIM_NeighborTable_H_
#define __ROUTINGSIM_NeighborTable_H_


#include "foundation.h"
#include "OverlayKey.h"
#include <vector>
#include <utility>

namespace routingsim {
using inner_table = std::vector<std::pair<OverlayKey, nodeid_t>>;
/**
 * A table for mapping overlay keys of neigbors to node ids an back.
 * Compare to arp table.
 * @author Jakob Thomas
 */
class NeighborTable
{
public:
	/// adds or updates the table with the given pair
	void addNeighbor(nodeid_t nodeId, const OverlayKey& key);
	/// removes the entry with the given node id if present
	void removeNeighbor(nodeid_t nodeId);
	/// returns a vector of all known neigbors
	std::vector<OverlayKey> getNeighbors();
	/// returns the node id of the neighbor with the given key.
	/// @returns unspecified_node if the key is not found
	nodeid_t getNodeId(const OverlayKey& key);
	/// returns true if the key belongs to a known neigbor
	inline bool isNeighbor(const OverlayKey& key) { return getNodeId(key) != undefined_node; }
	/// clears the table
	void clear();

	/// Common stdc++ console output method
	friend std::ostream& operator<<(std::ostream& os, const NeighborTable& c);
private:
	/// returns an iterator pointing to the entry with the given node id
	inner_table::iterator getIter(nodeid_t nodeId);
	/// returns an iterator pointing to the entry with the given overlay key
	inner_table::iterator getIter(const OverlayKey& key);
private:
	/// storage for the overlay key, node id pairs
	inner_table table;
};

} //namespace

#endif // __ROUTINGSIM_NeighborTable_H_
