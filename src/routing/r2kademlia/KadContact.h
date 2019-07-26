
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
#include "OverlayKey.h"

/**
 * @file KadContact.h
 * @author Jakob Thomas
 */

namespace routingsim {

// type definitions
using kad_path = std::vector<OverlayKey>;
using size_type = std::size_t;

class KadContact
{
public:
	enum State {
		undefined     = 0,
		valid         = 1,
		rediscovering = 2,
		invalid       = 3
	};

	KadContact(const KadContact&) = delete;
        KadContact(const OverlayKey& key): key(key), path(), state(undefined) {};
        KadContact(const OverlayKey& key, kad_path opath) :
	                  key(key), path(std::move(opath)),
			  state(valid), physicalNeighbor(path.empty() == true) // simply path.empty() doesn't work correctly!
			  {};
	/// returns the current state of the contact
	inline State getState() const { return state; }
	inline bool isInvalid() const { return (state != valid); }
	inline bool isValid() const { return (state == valid) ; }
	inline bool isRediscovering() const { return (state == rediscovering) ; }

	/// this will be kept being true, even if the link goes down
	inline bool isPhysicalNeighbor() const { return physicalNeighbor; }

	/// set contact state
	inline void setInvalid() { state= invalid; }
	inline void setValid() { state= valid; }
	inline void setRediscovering() { state= rediscovering; }

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

	/// Common stdc++ console output method
	friend std::ostream& operator<<(std::ostream& os, const KadContact& c);
private:
	const OverlayKey key;	/// the OverlayKey of this contact
	kad_path path;		/// the best known source routing path to reach this contact
	State state;            /// the state of the contact
	bool physicalNeighbor= false; /// true if we have a direct physical link to this neighbor
};

}; // namespace
#endif // __ROUTINGSIM_KadContact_H_
