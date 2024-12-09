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
#include "routing/KIRA/KadPacket_m.h"
#include <cstdint>
#include <sys/types.h>
#include <unordered_map>

namespace routingsim {
Register_Class(KadPacket);

std::unordered_map<uint8_t, const char *const> PDUNames = {
    {KAD_ULNHello, "ULNHello"},
    {KAD_ULNDiscReq, "ULNDiscReq"},
    {KAD_ULNDiscRsp, "ULNDiscRsp"},
    {KAD_FindNodeReq, "FindNodeReq"},
    {KAD_FindNodeRsp, "FindNodeRsp"},
    {KAD_QueryRouteReq, "QueryRouteReq"},
    {KAD_QueryRouteRsp, "QueryRouteRsp"},
    {KAD_UpdateRouteReq, "UpdateRouteReq"},
    {KAD_Error, "Error"},
    {KAD_RevPathFlood, "RevPathFlood"},
    {KAD_LinkFailureNotify, "LinkFailureNotify"},
    {KAD_ProbeReq, "ProbeReq"},
    {KAD_ProbeRsp, "ProbeRsp"},
    {KAD_PathSetupReq, "KAD_PathSetupReq"},
    {KAD_PathSetupRsp, "PathSetupRsp"},
    {KAD_PathTeardownReq, "PathTeardownReq"},
    {KAD_Data, "Data"},
    {KAD_Flood, "Flood"},
    {count,"NotAMsg-Count"}
};


void KadPacket::setSource(const OverlayKey & source) {
	if (route_arraysize == 0)
		appendHop(source);
	else
		setRoute(0, source);
};


void KadPacket::appendRoute(const kad_path& path) {
	auto old_size = getRouteArraySize();
	setRouteArraySize(old_size + path.size());
	setActiveArraySize(old_size + path.size());
	for (size_t i = 0; i < path.size(); i++) {
		setRoute(old_size + i, path[i]);
		setActive(old_size + i, false);
	}
	addBitLength(OverlayKey::getLength()*path.size());
}


// appends path and target to source route
// target is assumed to be an overlay hop
void KadPacket::appendRoute(const kad_path& path, const OverlayKey& target) {
	auto old_size = getRouteArraySize();
	setRouteArraySize(old_size + path.size()+1);
	setActiveArraySize(old_size + path.size()+1);
	for (size_t i = 0; i < path.size(); i++) {
		setRoute(old_size + i, path[i]);
		setActive(old_size + i, false);
	}
	setRoute(old_size + path.size(), target);
	setActive(old_size + path.size(), true);

	addBitLength(OverlayKey::getLength()*(path.size()+1));
}


// replaces rest of path with newpath (starting from progress + 1)
// path should also contain target node
void KadPacket::modRoute(const kad_path& newpath) {
	auto old_size = getRouteArraySize();
	// progress is an index so the size of the current path up to this node is progress + 1
	size_t newsize= progress + 1 + newpath.size();
	if (newsize > old_size) // need to resize
	{ // this creates an enlarged copy beforehand
		setRouteArraySize(newsize);
		setActiveArraySize(newsize);
		addBitLength(OverlayKey::getLength()*(newsize-old_size));
	}

	for (size_t i = 0; i < newpath.size(); i++) {
		setRoute(progress + 1 + i, newpath[i]);
		setActive(progress + 1 + i, false);
	}

	// last hop is next overlay node, so set active flag
	setActive(newsize - 1, true);

	if ( newsize < old_size )
	{
		setRouteArraySize(newsize);
		setActiveArraySize(newsize);
		addBitLength(OverlayKey::getLength()*(newsize-old_size));
	}
}


void KadPacket::appendHop(const OverlayKey& key) {
	auto index = getRouteArraySize();
	setRouteArraySize(index+1);
	setActiveArraySize(index+1);
	setRoute(index, key);
	setActive(index, true);
	addBitLength(OverlayKey::getLength());
}


void KadPacket::appendRoute(const KadContact& contact) {
	ASSERT(getRouteArraySize() == getActiveArraySize());
	auto old_size = getRouteArraySize();
	setRouteArraySize(old_size + contact.getPathLength() + 1);
	setActiveArraySize(getRouteArraySize());
	for (size_t i = 0; i < contact.getPathLength(); i++) {
		setRoute(old_size + i, contact.getPath()[i]);
		setActive(old_size + i, false);
	}
	setRoute(getRouteArraySize() - 1, contact.getKey());
	setActive(getRouteArraySize() - 1, true);
	addBitLength(OverlayKey::getLength()*(contact.getPathLength() + 1));
}


// returns source routing path from "from" until "to", but excluding "to"
kad_path KadPacket::getSrcPath(uint from, uint to) const {
	// index bounds are checked in getHop 
	if(from > to)
		throw cRuntimeError("from > to"); // TODO
	kad_path result{};
	result.reserve(to - from);
	for (; from < to; from++)
		result.emplace_back(getHop(from));
	return result;
}


void KadPacket::setReturnPathFrom(const KadPacket& other) {
	// we include the current hop
	ASSERT(route_arraysize == 0);
	setRouteArraySize(other.progress + 1);
	setActiveArraySize(getRouteArraySize());

	for (uint i= 0; i <= other.progress; i++) {
		setRoute(i, other.getRoute(other.progress - i));
		setActive(i, false);
	}
	addBitLength(OverlayKey::getLength()*(other.progress + 1));
}


// returns true if path was shortened
bool KadPacket::setSimplifiedReturnPathFrom(const KadPacket& other) {
	// we include the current hop
	ASSERT(route_arraysize == 0);
	kad_path resultpath;
	uint otherPathLen= other.progress + 1;
	resultpath.reserve(otherPathLen); // result may be shorter

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
		resultpath.push_back(current);
		// j is now either other.progress - i or the leftmost index of the same key
		index++;
		// if we skipped elements we can forward i accordingly
		if (skip)
			i= other.progress - j;
	}  // end for

	setRouteArraySize(index);
	setActiveArraySize(index);
	for (uint i= 0; i < index; i++)
		setRoute(i, resultpath[i]);

	addBitLength(OverlayKey::getLength()*index);

	return index < otherPathLen;
}


std::vector<bool>
KadPacket::getActiveVector() const {
  std::vector<bool> active_vector(getActiveArraySize());
  for (size_t i = 0; i < getActiveArraySize(); i++)
    active_vector[i] = getActive(i);
  return active_vector;
}


// failedlinkage should always be set to 0 by nodes that are one of the failedLink nodes,
// i.e., they have current information that the link is currently down
void KadPacket::addNotViaLink(const link_t& failedlink, age_t failedlinkAge) {
	ASSERT(not failedlink.first.isUnspecified() and not failedlink.second.isUnspecified());
	ASSERT(failedlink.first != failedlink.second);

	// check whether link is already present
	unsigned int i= 0;
	while(i < getFailedLinkArraySize())
	{
		if ( (getFailedLink(i) == failedlink) or (getFailedLink(i) == link_t(failedlink.second, failedlink.first)) ) {
			// check whether we need to update the age value
			if (getFailedLinkAge(i) > failedlinkAge)
				setFailedLinkAge(i, failedlinkAge);
			break;
		}
		i++;
	}

	if (i == getFailedLinkArraySize()) { // not found
		// add failed link
		setFailedLinkArraySize(i+1); // since i == getFailedLinkArraySize()
		setFailedLink(i, failedlink);
		setFailedLinkAgeArraySize(i+1);
		setFailedLinkAge(i, failedlinkAge);

		if (i==0) // indicate NotVia for the first added link
			setNotVia(true);

		addBitLength(OverlayKey::getLength()*2+sizeof(age_t)*8); // two keys + age
	}
}

Register_Class(KadDataPacket);

	
} // namespace

