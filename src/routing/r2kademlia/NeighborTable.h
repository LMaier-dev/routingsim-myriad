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
#include <algorithm>

namespace routingsim {
using inner_table = std::map<OverlayKey, nodeid_t>;
/**
 * A table for mapping overlay keys of neigbors to nodeIDs.
 * Similar to ARP table that maps IP addresses to MAC addresses.
 * @author Jakob Thomas
 */
class NeighborTable
{
public:
	/// adds or updates the table with the given pair
	/// @returns true if add/updated, false if this entry existed already
	bool addNeighbor(const OverlayKey& key, nodeid_t nodeId);
	/// removes the entry with the given node id if present
	void removeNeighbor(const OverlayKey& key) { table.erase(key); };
	/// returns a reference to the table, e.g., for reading the entries
	const inner_table& getTable() const { return table; };
	/// returns the node id of the neighbor with the given key.
	/// @returns undefined_node if the key is not found
	inline nodeid_t getNodeId(const OverlayKey& key) const;
	/// returns true if the key belongs to a known neighbor
	bool isNeighbor(const OverlayKey& key) const { return getNodeId(key) != undefined_node; }
	/// clears the table
	void clear() { table.clear(); }
	/// returns the number of saved neighbors
	size_t size() const { return table.size(); }

	/// Common stdc++ console output method
	friend std::ostream& operator<<(std::ostream& os, const NeighborTable& c);
private:
	/// returns an iterator pointing to the entry with the given overlay key
	inner_table::iterator getIter(const OverlayKey& key) {	return table.find(key); }
	/// returns an iterator pointing to the entry with the given overlay key
	inner_table::const_iterator getIter(const OverlayKey& key) const { return table.find(key); };
private:
	/// storage for the overlay key, node id pairs
	inner_table table;
};


inline
bool NeighborTable::addNeighbor(const OverlayKey& key, nodeid_t nodeId){
	auto returnval= table.insert({key,nodeId});

	return returnval.second;
}

inline
nodeid_t NeighborTable::getNodeId(const OverlayKey& key) const {
	if (auto it = getIter(key); it != table.end())
		return it->second;
	else
		return undefined_node;
}


} //namespace

#endif // __ROUTINGSIM_NeighborTable_H_
