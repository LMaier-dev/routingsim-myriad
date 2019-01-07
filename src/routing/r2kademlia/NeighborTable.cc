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
nodeid_t NeighborTable::getNodeId(const OverlayKey& key) {
	auto it = getIter(key);
	if (it != table.end())
		return std::get<nodeid_t>(*it);
	else
		return undefined_node;
}

void NeighborTable::clear()
{
	table.clear();
}

inner_table::iterator NeighborTable::getIter(nodeid_t nodeId){
	for(auto it=table.begin(); it != table.end(); it++){
		if(std::get<nodeid_t>(*it) == nodeId)
			return it;
	}
	return table.end();
}
inner_table::iterator NeighborTable::getIter(const OverlayKey& key){
	for(auto it=table.begin(); it != table.end(); it++){
		if(std::get<OverlayKey>(*it) == key)
			return it;
	}
	return table.end();
}

} //namespace
