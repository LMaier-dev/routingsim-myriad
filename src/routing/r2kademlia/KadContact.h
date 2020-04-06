
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
#include "KadPath.h"

/**
 * @file KadContact.h
 * @author Jakob Thomas
 */

namespace routingsim {

// type definitions

using size_type = std::size_t;
using pathseqno_t = uint16_t;
constexpr pathseqno_t invalidSeqNo= 0xffff;

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

	KadContact(const KadContact&) = default;
        KadContact(KadContact&& k) : key(std::move(k.key)), path(std::move(k.path)), state(k.state), physicalNeighbor(k.physicalNeighbor) {};
        explicit KadContact(const OverlayKey& key): key(key), path(), state(State::undefined) {};
        KadContact(const OverlayKey& key, kad_path opath) :
	                  key(key), path(std::move(opath)),
			  state(State::valid), physicalNeighbor(path.empty() == true) // simply path.empty() doesn't work correctly!
			  {};
        KadContact& operator=(const KadContact &other); // copy assignment
	KadContact& operator=(KadContact &&other); // move assignement
	/// returns the current state of the contact
	State getState() const { return state; }
	bool isInvalid() const { return (state != State::valid); }
	bool isValid() const { return (state == State::valid) ; }
	bool isRediscovering() const { return (state == State::rediscovering) ; }

	/// this will be kept being true, even if the link goes down
	bool isPhysicalNeighbor() const { return physicalNeighbor; }

	/// set contact state
	void setInvalid() { state= State::invalid; }
	void setValid() { state= State::valid; }
	void setRediscovering() { state= State::rediscovering; }
	void setState(State newstate) { state= newstate; }


	void setPhysicalNeighbor() { physicalNeighbor= true; }
	
	/// returns the key of this contact
	const OverlayKey& getKey() const { return key; }
	/// compares the given overlay key to the contacts overlay key
	bool hasKey(const OverlayKey& other) const { return other == key; }
	/// returns the length of the path
	/// i.e. the intermediate hops between me and this contact
	size_t getPathLength() const { return path.size(); }
	/// sets the path to reach this contact (will implicitly set status to valid)
	void setPath(kad_path path) { this->path = std::move(path); setValid(); }
	const kad_path& getPath() const { return path; }
	PathProperty getPathProperty() const { return pathproperty; } 
	void setPathProperty(PathProperty newprop) { pathproperty= newprop; }
	pathseqno_t getPathSeqNo() const { return seqno; }
	void setPathSeqNo(pathseqno_t newseqno) {
		if (not (seqno == invalidSeqNo or (seqno != invalidSeqNo and newseqno >= seqno)))
			EV_ERROR << "Contact: " << key.shortStr() << " setPathSeqNo: old=" << seqno << " new=" << newseqno << endl;
		ASSERT(newseqno != invalidSeqNo); ASSERT(seqno == invalidSeqNo or (seqno != invalidSeqNo and newseqno >= seqno));
		seqno= newseqno; }

	/// Common stdc++ console output method
	friend std::ostream& operator<<(std::ostream& os, const KadContact& c);

	static const KadContact undefinedContact;
private:
	OverlayKey key;	   /// the OverlayKey of this contact
	kad_path path;	   /// the best known source routing path to reach this contact
	pathseqno_t seqno= invalidSeqNo; /// sequence number for the path state (announced by origin)
	State state= State::undefined; /// the state of the contact
	PathProperty pathproperty= PathProperty::undefined; /// the path property
	bool physicalNeighbor= false; /// true if we have a direct physical link to this neighbor
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
		path= other.path;
		state= other.state;
		physicalNeighbor= other.physicalNeighbor;
	}
	return *this;
}


inline KadContact& KadContact::operator=(KadContact&& other)
{
	if (this != &other) {
		key= other.key;
		path= other.path;
		state= other.state;
		physicalNeighbor= other.physicalNeighbor;
		other.path.clear();
		other.state= State::undefined;
	}
	return *this;
}


}; // namespace
#endif // __ROUTINGSIM_KadContact_H_
