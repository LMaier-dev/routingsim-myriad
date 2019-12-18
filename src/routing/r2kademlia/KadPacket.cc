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

#include "KadPacket.h"

namespace routingsim {
Register_Class(KadPacket);

void KadPacket::setSource(const OverlayKey & source) {
	if(route_arraysize == 0)
		appendHop(source);
	else
		setRoute(0, source);
};

void KadPacket::appendRoute(const kad_path& path) {
	auto old_size = getRouteArraySize();
	setRouteArraySize(old_size + path.size());
	setActiveArraySize(old_size + path.size());
	for(size_t i = 0; i < path.size(); i++) {
		setRoute(old_size + i, path[i]);
		setActive(old_size + i, false);
	}
}

// path should also contain target node
void KadPacket::modRoute(const kad_path& path) {
	auto tail_size = getRouteArraySize()-progress-1; // progress is index starting from 0
	ASSERT(tail_size >= path.size());

	size_t newsize= progress + 1 + path.size();
	ASSERT(newsize <= getRouteArraySize());
	for(size_t i = 0; i < path.size(); i++) {
		setRoute(progress + 1 + i, path[i]);
		setActive(progress + 1 + i, false);
	}
	setRouteArraySize(newsize);
	setActiveArraySize(newsize);
}

void KadPacket::appendHop(const OverlayKey& key) {
	auto index = getRouteArraySize();
	setRouteArraySize(index+1);
	setActiveArraySize(index+1);
	setRoute(index, key);
	setActive(index, true);
}

void KadPacket::appendRoute(const KadContact& contact) {
	ASSERT(getRouteArraySize() == getActiveArraySize());
	auto old_size = getRouteArraySize();
	setRouteArraySize(old_size + contact.getPathLength() + 1);
	setActiveArraySize(getRouteArraySize());
	for(size_t i = 0; i < contact.getPathLength(); i++) {
		setRoute(old_size + i, contact.getPath()[i]);
		setActive(old_size + i, false);
	}
	setRoute(getRouteArraySize() - 1, contact.getKey());
	setActive(getRouteArraySize() - 1, true);
}

kad_path KadPacket::getSrcPath(uint from, uint to) {
	// index bounds are checked in getHop 
	if(from > to)
		throw cRuntimeError("from > to"); // TODO
	kad_path result{};
	result.reserve(to - from);
	for(; from < to; from++)
		result.emplace_back(getHop(from));
	return result;
}

void KadPacket::setReturnPathFrom(KadPacket& other) {
	// we include the current hop
	ASSERT(route_arraysize == 0);
	setRouteArraySize(other.progress + 1);
	for (uint i= 0; i <= other.progress; i++)
		setRoute(i, other.getRoute(other.progress - i));
}


void KadPacket::setSimplifiedReturnPathFrom(KadPacket& other) {
	// we include the current hop
	ASSERT(route_arraysize == 0);
	uint index= 0;
	for (uint i= 0; i <= other.progress; i++)
	{
		const OverlayKey& current= other.getRoute(other.progress - i);
		bool skip= false;
		uint j= 0;
		while (j < other.progress - i)
		{
			if (other.getRoute(j) == current)
			{
				skip= true;
				break;
			}
			j++;
		}
		// j is now either other.progress - i or the leftmost index of the same key
		setRoute(index, other.getRoute(j));
		index++;
		// if we skipped elements we can forward i accordingly
		if (skip)
			i= other.progress - j;
	}  // end for
	setRouteArraySize(index);
}


Register_Class(KadDataPacket);

	
} // namespace

