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

#include "NeighborTable.h"

namespace routingsim {

void NeighborTable::addNeighbor(nodeid_t nodeId, const OverlayKey& key){
	//TODO assert nodeId != undefined_node;
	auto itId = getIter(nodeId);
	auto itKey = getIter(key);
	if (itId == itKey){
		if(itId == table.end())
			table.emplace_back(key, nodeId);
		// else identical pair exists
	}
	else if (itKey != table.end()){
		std::get<nodeid_t>(*itKey) = nodeId;
		if (itId != table.end()) // both id and key existed, we erase the id.
			table.erase(itId);
	}
	else if (itId != table.end()){
		std::get<OverlayKey>(*itId) = key;
	}
}

void NeighborTable::removeNeighbor(nodeid_t nodeId){
	auto it = getIter(nodeId);
	if (it != table.end())
		table.erase(it);
}

std::vector<OverlayKey> NeighborTable::getNeighbors() const
{
	std::vector<OverlayKey> result{};
	result.reserve(table.size());
	for (auto& pair : table) {
		result.push_back(std::get<OverlayKey>(pair));
	}
	return result;
}


nodeid_t NeighborTable::getNodeId(const OverlayKey& key) const {
	auto it = getIter(key);
	if (it != table.end())
		return std::get<nodeid_t>(*it);
	else
		return undefined_node;
}

const OverlayKey& NeighborTable::getOverlayKey(nodeid_t nodeId) const {
	auto it = getIter(nodeId);
	if (it != table.end())
		return std::get<OverlayKey>(*it);
	else
		return OverlayKey::UNSPECIFIED_KEY;
}

void NeighborTable::clear()
{
	table.clear();
}


std::ostream& operator<<(std::ostream& os, const NeighborTable& c) {
	os << "(" << c.table.size() << ") ";
	for (auto pair : c.table)
		os << pair.first.shortStr() << "->" << pair.second << "  ";
	return os;
}



} //namespace
