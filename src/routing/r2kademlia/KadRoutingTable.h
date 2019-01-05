
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

#ifndef __ROUTINGSIM_KadRoutingTalbe_H_
#define __ROUTINGSIM_KadRoutingTalbe_H_

#include <omnetpp.h>
#include <memory>
#include <vector>
#include "OverlayKey.h"
/**
 * @file KadRoutingTable.h
 * @author Jakob Thomas
 */

namespace routingsim {

// type definitions
using kad_path = std::vector<OverlayKey>;
using size_type = std::size_t;

class KadContact
{
public:
	KadContact(const OverlayKey key): key(key) {};
	OverlayKey getKey() const { return key; }
	/// compares the given key to the contacts key
	bool hasKey(const OverlayKey& other) const { return other == key; }
	void setStale(bool stale_) {stale = stale_};
	bool isStale() const { return stale; }
	/// sets the last seen time to now
	void seen() { lastSeen = simTime(); }
	simtime_t getLastSeen() const { return lastSeen; }
private:
	const OverlayKey key;	/// the OverlayKey of this contact
	simtime_t lastSeen;		/// the last time this contact was seen
	kad_path path;			/// the best known source routing path to reach this contact
	bool stale;				/// true if the contact is not answering
};
// contact type definitions
using KadContact_p = std::shared_ptr<KadContact>;
using KadContactList = std::vector<KadContact_p>;
using KadContactList_p = std::unique_ptr<KadContactList>;

// contact list helper functions:
/**
 * Returns an iterator at the contacts position.
 * Returns list->end() if the contact is not found.
 */
auto findContactInList(KadContactList_p& list, const OverlayKey& key){
	auto it = list->begin();
	for (;!(*it)->hasKey(key) && it !=  list->end(); it++);
	return it;
}
/** tries to remove the given contact from the list. returns true if found.*/
bool removeContactFromList(KadContactList_p& list, const OverlayKey& key);

class KadBucket
{
public:
	KadContactList_p contacts;
	KadContactList_p refillContacts;
	KadContact_p getContact(const OverlayKey& key);
	bool removeContact(const OverlayKey& key) { return removeContactFromList(contacts, key);}
	bool removeContactFromReplacements(const OverlayKey& key) { return removeContactFromList(refillContacts, key);}
	bool empty() { return contacts->empty();}
};

/**
 */
class KadRoutingTable
{
public:
	bool addContact(KadContact_p contact);
	bool removeContact(const OverlayKey& key);
	bool removeContact(KadContact_p contact) { return removeContact(contact->getKey()); }
	KadContact_p getContact(const OverlayKey& key);
	KadContactList_p getClosestContacts(const OverlayKey& key, size_type n);
	
protected:
	/** returns the index of the bucket corresponding to the given overlay key. */
	size_type getBucketIndex(const OverlayKey& key);

	/** returns the index of the bucket corresponding to the given contact */
	size_type getBucketIndex(KadContact_p contact) { return getBucketIndex(contact->getKey());}

	/** returns the bucket corresponding to the given overlay key */
	KadBucket* getBucket(const OverlayKey& key);

	/** returns the index of the bucket that is the first on the level of the given bucket */
	size_type getFirstOnBucketLevel(size_type index) { return index - (index % ((1 << b) - 1));}

	// ------ policy foo -----
	/** called when a new contact is discovered */
	void onDiscovery(KadContact_p) {};
	/** called when we hear about a node id */
	void onRumor(const OverlayKey& key) {};
	/** called when there is space in a bucket and the bucket is not empty */
	void onBucketSpace(KadBucket* bucket) {};
	/** called when last contact is removed from a bucket*/
	void onBucketEmpty(KadBucket* bucket) {};
	/**  called when a bucket needs to be refreshed */
	void onBucketRefresh(KadBucket* bucket) {};
	/** called when a contact could be replaced by a new one */
	void onBucketFull(KadContact_p contact, KadBucket* bucket) {};

private:
	/** acceleration factor, number of bits used for accelerated lookup */
	const size_type b=1;
	/** the key of this overlay node */
	const OverlayKey myKey=OverlayKey::ZERO;
	size_type numBuckets;
	std::vector<KadBucket> buckets;

	//------ private inline helper functions -------
	/** returns a random access iterator situated on the bucket with the given index */
	auto inline getBucketIterator(size_type index=0){
		auto it=buckets.begin();
		std::advance(it, index);
		return it;
	}

	/** copies all contacts from one contact list to the end of the other contact list*/
	void inline copyContacts(KadContactList_p& source, KadContactList_p& target){
		std::copy(source->begin(), source->end(), std::back_inserter(*target));
	}
};

} //namespace

#endif // __ROUTINGSIM_KadRoutingTalbe_H_
