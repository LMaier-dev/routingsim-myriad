
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

#ifndef __ROUTINGSIM_KadContact_H_
#define __ROUTINGSIM_KadContact_H_

#include <omnetpp.h>
#include <memory>
#include <vector>
#include <algorithm>
#include "KadPath.h"

/**
 * @file KadContact.h
 * @author Jakob Thomas
 */

namespace routingsim {

// type definitions

using size_type = std::size_t;
using stateseqno_t = uint16_t;
constexpr stateseqno_t invalidSeqNo= 0xffff;

class RediscoveryState {
public:
	RediscoveryState(const link_t& failedlink) : failedLinkList({failedlink}) {};
	std::list<link_t> failedLinkList; // the currently failed links
	std::list<OverlayKey> viaContacts; // these nodes should be considered for rediscovery actions for this contact
};


class KadContact
{
public:
	enum class State : uint8_t {
		undefined     = 0,
		valid         = 1,
		rediscovering = 2,
		invalid       = 3
	};

	enum class PathProperty : uint8_t {
		undefined     = 0,
		direct        = 1, // path was directly learned, i.e., traversed
		indirect      = 2, // path was indirectly learned
		invalid       = 3  // path is invalid
	};

	KadContact(const KadContact& o) : key(o.key), lastseen(o.lastseen), path(o.path), seqno(o.seqno),
					  state(o.state), pathproperty(o.pathproperty),
					  physicalNeighbor(o.physicalNeighbor),
					  rds_p(o.rds_p == nullptr ? nullptr :
						std::make_unique<RediscoveryState>(*o.rds_p)) {};
        KadContact(KadContact&& k) : key(std::move(k.key)), lastseen(std::move(k.lastseen)), path(std::move(k.path)),
				     seqno(k.seqno), state(k.state), pathproperty(k.pathproperty),
				     physicalNeighbor(k.physicalNeighbor),
				     rds_p(std::move(k.rds_p)) {};
        explicit KadContact(const OverlayKey& key): key(key), lastseen(SIMTIME_ZERO), path(), state(State::undefined), rds_p(nullptr) {};
        KadContact(const OverlayKey& key, kad_path opath, stateseqno_t seqno= invalidSeqNo) :
		key(key), lastseen(SIMTIME_ZERO), path(std::move(opath)), seqno(seqno),
		state(State::valid), physicalNeighbor(path.empty() == true), rds_p(nullptr) // simply path.empty() doesn't work correctly!
		{};
        KadContact& operator=(const KadContact &other); // copy assignment
	KadContact& operator=(KadContact &&other); // move assignment
	/// returns the current state of the contact
	State getState() const { return state; }
	bool isInvalid() const { return (state != State::valid); }
	bool isValid() const { return (state == State::valid) ; }
	bool isRediscovering() const { return (state == State::rediscovering) ; }

	/// this will be kept being true, even if the link goes down
	bool isPhysicalNeighbor() const { return physicalNeighbor; }

	/// set contact state
	inline void setInvalid(const link_t& failed_link);
	void setValid() { state= State::valid; rds_p.reset(nullptr); }
	void setRediscovering() { ASSERT(state == State::invalid); state= State::rediscovering; }
	void setState(State newstate) { state= newstate; }

	/// return age of this entry
	unsigned long getAge() const { return (simTime() - lastseen).inUnit(SIMTIME_MS); }
	simtime_t getLastSeen() const { return lastseen; }
	void updateLastSeen() { lastseen= simTime(); }
	void updateLastSeenTo(const simtime_t &new_time_stamp) { lastseen= new_time_stamp; }

	const RediscoveryState& getRDS() const { ASSERT(rds_p != nullptr); return *rds_p; }
	RediscoveryState& getRDS() { ASSERT(rds_p != nullptr); return *rds_p; }
	void addRediscoverVia(const OverlayKey& nodeid) { ASSERT(rds_p != nullptr); rds_p->viaContacts.push_back(nodeid); };
	void setPhysicalNeighbor() { physicalNeighbor= true; }
	
	/// returns the key of this contact
	const OverlayKey& getKey() const { return key; }
	/// compares the given overlay key to the contacts overlay key
	bool hasKey(const OverlayKey& other) const { return other == key; }
	/// returns the length of the path (here: number of hops-1)
	/// i.e., the _intermediate hops_ between me and this contact
	size_t getPathLength() const { return path.size(); }
	/// sets the path to reach this contact (will implicitly set status to valid)
	void setPath(kad_path path) { this->path = std::move(path); setValid(); }
	const kad_path& getPath() const { return path; }
	PathProperty getPathProperty() const { return pathproperty; } 
	void setPathProperty(PathProperty newprop) { pathproperty= newprop; }
	stateseqno_t getStateSeqNo() const { return seqno; }
	void setStateSeqNo(stateseqno_t newseqno) {
		//if (not (seqno == invalidSeqNo or (seqno != invalidSeqNo and newseqno >= seqno)))
		//	EV_ERROR << "Contact: " << key.shortStr() << " setPathSeqNo: old=" << seqno << " new=" << newseqno << endl;
		ASSERT(newseqno != invalidSeqNo); ASSERT(seqno == invalidSeqNo or (seqno != invalidSeqNo and newseqno >= seqno));
		seqno= newseqno; }
	bool isContainedinPath(const OverlayKey& myOverlayKey, const link_t& link) const;
	/// Common stdc++ console output method
	friend std::ostream& operator<<(std::ostream& os, const KadContact& c);

	static const KadContact undefinedContact;
private:
	OverlayKey key;	   /// the OverlayKey of this contact
	simtime_t lastseen; /// time of last direct contact (could stem from other node, too)
	/// the following path does neither contain the source (myself) nor the target node
	kad_path path;	   /// the best known source routing path to reach this contact
	stateseqno_t seqno= invalidSeqNo; /// sequence number for the path state (announced by origin)
	State state= State::undefined; /// the state of the contact
	PathProperty pathproperty= PathProperty::undefined; /// the path property
	bool physicalNeighbor= false; /// true if we have a direct physical link to this neighbor
	std::unique_ptr<RediscoveryState> rds_p= nullptr;
};

// contact type definitions
using KadContact_p = const KadContact*;
using KadContactList = std::vector<KadContact_p>;


// compare two contacts against given key
struct KadContactCompare {
	const OverlayKey& key;
	KadContactCompare(const OverlayKey& key) : key(key) {}
	bool operator()(KadContact_p& a, KadContact_p& b) const {
		return (a->getKey() ^ key) < (b->getKey() ^ key);
	}
};

/********************* inline methods *********************/


inline KadContact& KadContact::operator=(const KadContact& other)
{
	if (this != &other) {
		key= other.key;
		lastseen = other.lastseen;
		path= other.path;
		seqno= other.seqno;
		state= other.state;
		pathproperty= other.pathproperty;
		physicalNeighbor= other.physicalNeighbor;
	}
	return *this;
}


// move assignment
inline KadContact& KadContact::operator=(KadContact&& other)
{
	if (this != &other) {
		key= other.key;
		lastseen = other.lastseen;
		path= other.path;
		state= other.state;
		seqno= other.seqno;
		pathproperty= other.pathproperty;
		physicalNeighbor= other.physicalNeighbor;
		other.path.clear();
		other.seqno= invalidSeqNo;
		other.state= State::undefined;
	}
	return *this;
}


inline void KadContact::setInvalid(const link_t& failed_link)
{
	if (state == State::valid) {
		state= State::invalid;
		rds_p= std::make_unique<RediscoveryState>(failed_link);
	}
	else // already invalid or rediscovering
	{
		ASSERT(rds_p != nullptr);
		if (rds_p != nullptr) {
			if (std::find_if(rds_p->failedLinkList.begin(),
					 rds_p->failedLinkList.end(),
					 [&](const auto& link) { return link == failed_link or (link.first == failed_link.second and link.second == failed_link.first); })
			    == rds_p->failedLinkList.end())
			{ // link not present yet, so add it to the list
				EV_TRACE << "Adding another failed link " << failed_link << " to RDS for contact " << key.shortStr() << endl;
				rds_p->failedLinkList.push_back(failed_link);
			}
		}
	}
}

}; // namespace

#endif // __ROUTINGSIM_KadContact_H_
