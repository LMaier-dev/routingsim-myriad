
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


class KadContact
{
public:
	enum class State {
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
	inline State getState() const { return state; }
	inline bool isInvalid() const { return (state != State::valid); }
	inline bool isValid() const { return (state == State::valid) ; }
	inline bool isRediscovering() const { return (state == State::rediscovering) ; }

	/// this will be kept being true, even if the link goes down
	inline bool isPhysicalNeighbor() const { return physicalNeighbor; }

	/// set contact state
	inline void setInvalid() { state= State::invalid; }
	inline void setValid() { state= State::valid; }
	inline void setRediscovering() { state= State::rediscovering; }
	inline void setState(State newstate) { state= newstate; }


	inline void setPhysicalNeighbor() { physicalNeighbor= true; }
	
	/// returns the key of this contact
	inline const OverlayKey& getKey() const { return key; }
	/// compares the given overlay key to the contacts overlay key
	inline bool hasKey(const OverlayKey& other) const { return other == key; }
	/// returns the length of the path
	/// i.e. the intermediate hops between me and this contact
	inline size_t getPathLength() const { return path.size(); }
	/// sets the path to reach this contact (will implicitly set status to valid)
	inline void setPath(kad_path path) { this->path = std::move(path); setValid(); }
	inline const kad_path& getPath() const { return path; }
	inline PathProperty getPathProperty() const { return pathproperty; } 
	inline void setPathProperty(PathProperty newprop) { pathproperty= newprop; } 

	/// Common stdc++ console output method
	friend std::ostream& operator<<(std::ostream& os, const KadContact& c);

	static const KadContact undefinedContact;
private:
	OverlayKey key;	/// the OverlayKey of this contact
	kad_path path;		/// the best known source routing path to reach this contact
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
