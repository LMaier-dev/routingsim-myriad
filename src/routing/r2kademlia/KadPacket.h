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


#ifndef __ROUTINGSIM_KadPacket_H_
#define __ROUTINGSIM_KadPacket_H_

#include <omnetpp.h>
#include "KadPacket_m.h"
#include "KadContact.h"

namespace routingsim {

class KadPacket : public KadPacket_Base
{
public:
	// member functions needed for minimal implementation
	KadPacket(const char *name=nullptr, short kind=0) : KadPacket_Base(name, kind) {}
	KadPacket(const KadPacket& other) : KadPacket_Base(other) {}
	~KadPacket() {};
	KadPacket& operator=(const KadPacket& other)
		{KadPacket_Base::operator=(other); return *this;}
	virtual KadPacket *dup() const override {return new KadPacket(*this);}

	// ==== other functions ====
	// --- utility foo
	/// sets the progress index to the specified int
	virtual void setProgress(uint16_t progress) override
		{ ASSERT(progress < route_arraysize); this->progress = progress; }
	/// sets the first key in the source route to the specified key.
	/// If the source route is empty, this initializes it.
	void setSource(const OverlayKey & source);
	/// appends the specified path to the source route
	void appendRoute(kad_path& path);
	/// appends the specified key to the source route
	void appendHop(const OverlayKey& key);
	/// appends the key of the specified contact to the source route
	inline void appendHop(KadContact& contact) { appendHop(contact.getKey()); }
	/// appends the key of the specified contact to the source route
	inline void appendHop(KadContact_p contact) { appendHop(*contact); }
	/// returns the source route as a kad_path (std::vector) from the given position
	/// until the given "to" position (excluding "to")
	kad_path getPath(uint from, uint to);
	/// returns the source route as a kad_path (std::vector) from the given position
	inline kad_path getPath(uint from = 0) { return getPath(from, route_arraysize); }
	/// sets the path of this packet to the reverse path of the other packet
	/// taking into acount the progress of the other packet
	void setReturnPathFrom(KadPacket& other);
	
	
	// --- inline utility foo
	/// returns the index of the next hop
	inline uint getNextHopIndex() const { return progress + 1;}
	/// incremets the progress counter, advances it to the next hop
	inline void advanceProgress() { ASSERT(progress < route_arraysize); progress++; }
	/// returns the specified part of the source route
	inline const OverlayKey& getHop(uint index) const { return getRoute(index); }
	/// returns the key of the next hop
	inline const OverlayKey& getNextHop() const { return getHop(getNextHopIndex()); }
	/// returns the current hop this packet should be at
	inline const OverlayKey& getCurrentHop() const { return getHop(progress); }
	/// returns the last element in the source route
	inline const OverlayKey& getTarget() const  { return getHop(route_arraysize - 1); }
	/// returns the first element in the source route
	inline const OverlayKey& getSource() const { return getHop(0); }
	/// returns true if the packet still has hops to take
	inline bool hasRemainingHops() const { return getNextHopIndex() < route_arraysize; }
	/// returns the remaining source route path starting with the next hop
	inline kad_path getRemainingPath() { return getPath(getNextHopIndex()); }
	/// returns the previous source route path until the current hop
	inline kad_path getPreviousPath() { return getPath(0, getNextHopIndex()); }
};
} // namespace
#endif // __ROUTINGSIM_KadPacket_H_
