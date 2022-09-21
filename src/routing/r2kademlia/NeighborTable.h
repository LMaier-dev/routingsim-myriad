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
using inner_table = std::vector<std::pair<OverlayKey, nodeid_t>>;
/**
 * A table for mapping overlay keys of neigbors to nodeIDs and back.
 * Similar to ARP table that maps IP addresses to MAC addresses.
 * @author Jakob Thomas
 */
class NeighborTable
{
public:
	/// adds or updates the table with the given pair
	/// @returns true if add/updated, false if this entry existed already
	bool addNeighbor(nodeid_t nodeId, const OverlayKey& key);
	/// removes the entry with the given node id if present
	void removeNeighbor(nodeid_t nodeId);
	/// returns a vector of all known neighbors
	std::vector<OverlayKey> getNeighbors() const;
	/// returns the node id of the neighbor with the given key.
	/// @returns undefined_node if the key is not found
	nodeid_t getNodeId(const OverlayKey& key) const;
	/// returns the overlay key of the given neighbor node id if known, else unspecified
	const OverlayKey& getOverlayKey(nodeid_t nodeId) const;
	/// returns true if the key belongs to a known neighbor
	inline bool isNeighbor(const OverlayKey& key) const { return getNodeId(key) != undefined_node; }
	/// returns true if the node id belongs to a known neighbor
	inline bool isNeighbor(nodeid_t nodeId) const{ return !getOverlayKey(nodeId).isUnspecified(); }
	/// clears the table
	void clear();
	/// returns the number of saved neighbors
	size_t size() const { return table.size(); }

	/// Common stdc++ console output method
	friend std::ostream& operator<<(std::ostream& os, const NeighborTable& c);
private:
	/// returns an iterator pointing to the entry with the given node id
	inner_table::iterator getIter(nodeid_t nodeId);
	/// returns an iterator pointing to the entry with the given overlay key
	inner_table::iterator getIter(const OverlayKey& key);
	/// returns an iterator pointing to the entry with the given node id
	inner_table::const_iterator getIter(nodeid_t nodeId) const;
	/// returns an iterator pointing to the entry with the given overlay key
	inner_table::const_iterator getIter(const OverlayKey& key) const;
private:
	/// storage for the overlay key, node id pairs
	inner_table table;
};

inline
inner_table::iterator NeighborTable::getIter(nodeid_t nodeId) {
	return std::find_if(table.begin(),table.end(),
			    [=](const auto& elem){ return std::get<nodeid_t>(elem) == nodeId; });
}


inline
inner_table::iterator NeighborTable::getIter(const OverlayKey& key) {
	return std::find_if(table.begin(),table.end(),
			    [&](const auto& elem){ return std::get<OverlayKey>(elem) == key; });
}


inline
inner_table::const_iterator NeighborTable::getIter(nodeid_t nodeId) const {
	return std::find_if(table.begin(),table.end(),
			    [=](const auto& elem){ return std::get<nodeid_t>(elem) == nodeId; });
}


inline
inner_table::const_iterator NeighborTable::getIter(const OverlayKey& key) const {
	return std::find_if(table.begin(),table.end(),
			    [&](const auto& elem){ return std::get<OverlayKey>(elem) == key; });
}


} //namespace

#endif // __ROUTINGSIM_NeighborTable_H_
