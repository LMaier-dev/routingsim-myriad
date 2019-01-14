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
	for(size_t i=0; i<path.size(); i++)
		setRoute(old_size + i, path[i]);
}
void KadPacket::appendHop(const OverlayKey& key) {
	auto index = getRouteArraySize();
	setRouteArraySize(index+1);
	setRoute(index, key);
}

kad_path KadPacket::getPath(uint from, uint to) {
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
	for(uint i=0; i <= other.progress; i++)
		setRoute(i, other.getRoute(other.progress - i));
}

} // namespace

