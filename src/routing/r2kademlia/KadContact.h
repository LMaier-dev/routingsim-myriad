
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
	KadContact(const OverlayKey& key): key(key), path(){};
	KadContact(const OverlayKey& key, kad_path path): key(key), path(path) {};
	/// returns the key of this contact
	inline const OverlayKey& getKey() const { return key; }
	/// compares the given overlay key to the contacts overlay key
	inline bool hasKey(const OverlayKey& other) const { return other == key; }
	/// marks the contact as stale (or not)
	inline void setStale(bool stale=true) { this->stale=stale; }
	/// returns if this contact was marked as stale
	inline bool isStale() const { return stale; }
	/// sets the last seen time to now
	inline void seen() { lastSeen = simTime(); }
	inline simtime_t getLastSeen() const { return lastSeen; }
private:
	const OverlayKey key;	/// the OverlayKey of this contact
	simtime_t lastSeen;		/// the last time this contact was seen
	kad_path path;			/// the best known source routing path to reach this contact
	bool stale;				/// true if the contact is not answering
};
// contact type definitions
using KadContact_p = std::shared_ptr<KadContact>;
using KadContactList = std::vector<KadContact_p>;

// contact list helper functions:
/**
 * Returns an iterator at the contacts position.
 * Returns list->end() if the contact is not found.
 */
std::vector<KadContact_p>::iterator findContactInList(KadContactList& list, const OverlayKey& key);
/** tries to remove the given contact from the list. returns true if found.*/
bool removeContactFromList(KadContactList& list, const OverlayKey& key);

}; // namespace
#endif // __ROUTINGSIM_KadContact_H_
