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
#include "stateseqno.h"

/**
 * @file KadContact.h
 * @author Jakob Thomas
 */

namespace routingsim {

// type definitions

using size_type = std::size_t;
	//using stateseqno_t = uint16_t;
using age_t = uint32_t;

constexpr unsigned short int maxRediscoveryRetries= 6;

// see R2Kademlia.h

class R2Kademlia;


class RediscoveryState {
public:
	enum class rediscoverQ_type : uint8_t {
		undefined     = 0,
		urgent        = 1,
		regular       = 2,
		slow          = 3
	};
	RediscoveryState(const std::pair<link_t, simtime_t>& failedlink) : failedLinkList({failedlink}) {};
	std::list<std::pair<link_t, simtime_t>> failedLinkList; // the currently failed links
	std::list<OverlayKey> viaContacts; // these nodes should be considered for rediscovery actions for this contact
	unsigned short int retryCounter= 0;
	rediscoverQ_type rediscoverQtype= rediscoverQ_type::undefined;
};


class KadContact
{
public:
	enum class State : uint8_t {
		undefined     = 0,
		valid         = 1,
		rediscovering = 2,
		invalid       = 3,
		dead          = 4
	};

	KadContact(const KadContact& o) : key(o.key), lastseen(o.lastseen), lastPathRefresh(o.lastPathRefresh), path(o.path), pathHashSum(o.pathHashSum),
					  seqno(o.seqno), state(o.state),
					  physicalNeighbor(o.physicalNeighbor),
					  nodeDegree(o.nodeDegree),
					  rds_p(o.rds_p == nullptr ? nullptr :
						std::make_unique<RediscoveryState>(*o.rds_p)) {};
        KadContact(KadContact&& k) : key(std::move(k.key)), lastseen(std::move(k.lastseen)), lastPathRefresh(std::move(k.lastPathRefresh)), path(std::move(k.path)),
				     pathHashSum(std::move(k.pathHashSum)),
				     seqno(k.seqno), state(k.state),
				     physicalNeighbor(k.physicalNeighbor),
				     nodeDegree(k.nodeDegree),
				     rds_p(std::move(k.rds_p)) {};
        explicit KadContact(const OverlayKey& key): key(key), lastseen(SIMTIME_ZERO), lastPathRefresh(SIMTIME_ZERO), path(), state(State::undefined), nodeDegree(0), rds_p(nullptr) {};
        KadContact(const OverlayKey& key, kad_path opath, stateseqno_t seqno= stateseqno_t::invalidSeqNo) :
		key(key), lastseen(SIMTIME_ZERO), lastPathRefresh(SIMTIME_ZERO), path(std::move(opath)), pathHashSum(HashSum(path,key)), seqno(seqno),
		state(State::valid), physicalNeighbor(path.empty() == true), nodeDegree(0), rds_p(nullptr) // simply path.empty() doesn't work correctly!
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
	inline bool setInvalid(const link_t& failed_link, age_t failed_link_age);
	void setValid() { state= State::valid; rds_p.reset(nullptr); }
	void setRediscovering() { ASSERT(state == State::invalid); state= State::rediscovering; }
	void setState(State newstate) { state= newstate; }

	/// return age of this entry
	unsigned long getAge() const { return (simTime() - lastseen).inUnit(SIMTIME_MS); }
	simtime_t getLastSeen() const { return lastseen; }
	void updateLastSeen() { lastseen= simTime(); }
	void updateLastSeenTo(const simtime_t &new_time_stamp) { lastseen= new_time_stamp; }

	unsigned long getPathAge() const { return (simTime() - lastPathRefresh).inUnit(SIMTIME_MS); }
	simtime_t getLastPathRefresh() const { return lastPathRefresh; }
	void updateLastPathRefresh() { lastPathRefresh = simTime(); }

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
	/// sets new path (originally checked for repair of failed links, too)
	void updatePath(const OverlayKey& myOverlayKey, kad_path newpath) { setPath(newpath, HashSum(newpath,key)); }
	/// sets the path to reach this contact (will implicitly set status to valid)
	void setPath(kad_path path, OverlayKey&& HashSum) { this->path = std::move(path); setValid(); pathHashSum= HashSum; }
	const kad_path& getPath() const { return path; }
	const OverlayKey& getPathHashSum() const { return pathHashSum; }
	stateseqno_t getStateSeqNo() const { return seqno; }
	void setStateSeqNo(stateseqno_t newseqno) {
		if (not (seqno.isInvalid() or (not seqno.isInvalid() and newseqno >= seqno)))
			EV_ERROR << "Contact: " << key.shortStr() << " setPathSeqNo: old=" << seqno << " new=" << newseqno << endl;
		ASSERT(not newseqno.isInvalid()); ASSERT(seqno.isInvalid() or (not seqno.isInvalid() and newseqno >= seqno));
		seqno= newseqno; }
	stateseqno_t getSyncPNSeqNo() const { return syncPNSeqno; }
	void setSyncPNSeqNo(stateseqno_t newseqno) {
		if (not (syncPNSeqno.isInvalid() or (not syncPNSeqno.isInvalid() and newseqno >= syncPNSeqno)))
			EV_ERROR << "Contact: " << key.shortStr() << " setSyncPNSeqNo: old=" << syncPNSeqno << " new=" << newseqno << endl;
		ASSERT(not newseqno.isInvalid()); ASSERT(syncPNSeqno.isInvalid() or (not syncPNSeqno.isInvalid() and newseqno >= syncPNSeqno));
		syncPNSeqno= newseqno; }

	bool isContainedinPath(const OverlayKey& myOverlayKey, const link_t& link) const;
	uint32_t getNodeDegree() const { return nodeDegree; };
	void setNodeDegree(uint32_t newnodeDegree) { nodeDegree= newnodeDegree; };
	uint32_t getConnectivity() const { return nodeDegree; };

	/// Common stdc++ console output method
	friend std::ostream& operator<<(std::ostream& os, const KadContact& c);

	static const KadContact undefinedContact;
private:
	OverlayKey key;	   /// the OverlayKey of this contact
	simtime_t lastseen; /// time of last direct contact or path update (could stem from other node, too)
	simtime_t lastPathRefresh; /// time of last path refresh (setupRsp / propeRsp)
	/// the following path does neither contain the source (myself) nor the target node
	kad_path path;	   /// the best known source routing path to reach this contact
	OverlayKey pathHashSum; /// the hash sum of the path
	stateseqno_t seqno= stateseqno_t::invalidSeqNo; /// sequence number for the path state (announced by origin)
	stateseqno_t syncPNSeqno= stateseqno_t::invalidSeqNo; /// sequence number for syncing with PN
	State state= State::undefined; /// the state of the contact
	bool physicalNeighbor= false; /// true if we have a direct physical link to this neighbor
	uint32_t nodeDegree = 0; /// degree of contact (node attribute connectivity)
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
		lastPathRefresh = other.lastPathRefresh;
		path= other.path;
		pathHashSum= other.pathHashSum;
		seqno= other.seqno;
		state= other.state;
		physicalNeighbor= other.physicalNeighbor;
		nodeDegree = other.nodeDegree;
	}
	return *this;
}


// move assignment
inline KadContact& KadContact::operator=(KadContact&& other)
{
	if (this != &other) {
		key= other.key;
		lastseen = other.lastseen;
		lastPathRefresh = other.lastPathRefresh;
		path= other.path;
		pathHashSum= other.pathHashSum;
		state= other.state;
		seqno= other.seqno;
		nodeDegree = other.nodeDegree;
		physicalNeighbor= other.physicalNeighbor;
		other.path.clear();
		other.seqno= stateseqno_t::invalidSeqNo;
		other.state= State::undefined;
		other.nodeDegree= 0;
	}
	return *this;
}

// returns true in case it is initially set invalid
inline bool KadContact::setInvalid(const link_t& failed_link, age_t failed_link_age)
{
	if (state == State::valid) {
		state= State::invalid;
		rds_p= std::make_unique<RediscoveryState>(std::make_pair(failed_link, simTime() - SimTime(failed_link_age,SIMTIME_MS)));
		return true;
	}
	else // already invalid or rediscovering
	{
		ASSERT(rds_p != nullptr);
		if (rds_p != nullptr) {
			if (std::find_if(rds_p->failedLinkList.begin(),
					 rds_p->failedLinkList.end(),
					 [&](const auto& link) { return link.first == failed_link or (link.first.first == failed_link.second and link.first.second == failed_link.first); })
			    == rds_p->failedLinkList.end())
			{ // link not present yet, so add it to the list
				EV_TRACE << "Adding another failed link " << failed_link << " to RDS for contact " << key.shortStr() << endl;
				rds_p->failedLinkList.push_back(std::make_pair(failed_link,simTime() - SimTime(failed_link_age,SIMTIME_MS)));
			}
		}
	}

	return false;
}


inline
bool KadContact::isContainedinPath(const OverlayKey& myOverlayKey, const link_t& link) const
{
	return containsUndirectedLink(myOverlayKey, getKey(), path, link);
}

}; // namespace

#endif // __ROUTINGSIM_KadContact_H_
