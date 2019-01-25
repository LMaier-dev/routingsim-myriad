
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
#include "KadContact.h"
/**
 * @file KadRoutingTable.h
 * @author Jakob Thomas
 */

namespace routingsim {
class KadRoutingTable;

class KadBucket
{
	friend KadRoutingTable;
public:
	/** returns the contact if found, nullptr otherwise */
	KadContact_p getContact(const OverlayKey& key);
	/** tries to remove the contact, returns true if found */
	inline bool removeContact(const OverlayKey& key) { return removeContactFromList(contacts, key); }
	/** tries to remove the contact from replacement list, returns true if found */
	inline bool removeContactFromReplacements(const OverlayKey& key) { return removeContactFromList(contactReplacements, key); }
	/** returns true if the contact list is empty */
	inline bool empty() { return contacts.empty(); }
	/** returns the size of the contact list */
	inline size_type size() { return contacts.size(); }
	inline size_type replacementSize() { return contactReplacements.size(); }
	void replaceContact(KadContact_p oldContact, KadContact_p newContact);
	/// Common stdc++ console output method
	friend std::ostream& operator<<(std::ostream& os, const KadBucket& b) {
		return os << b.contacts.size() << " contacts, " << b.contactReplacements.size() << " replacements";
	}
	KadContactList contacts;
	KadContactList contactReplacements;
};

/**
 * A class with callback functions that are called then the routing table
 * encounters things.
 */
class KadPolicy {
public:
	/** called when there is space in a bucket and the bucket is not empty */
	virtual void onBucketSpace(KadBucket& bucket) = 0;
	/** called when last contact is removed from a bucket*/
	virtual void onBucketEmpty(KadBucket& bucket) = 0;
	/**  called when a bucket needs to be refreshed */
	virtual void onBucketRefresh(KadBucket& bucket) = 0;
	/** called when a contact could be replaced by a new one */
	virtual void onBucketFull(KadBucket& bucket, KadContact_p contact) = 0;
};

/**
 * Models a kademlia routing table that stores contacts in buckets depending
 * on the overlay key of the node.
 */
class KadRoutingTable : public cObject
{
	friend KadBucket;
public:
	KadRoutingTable() {};
	void init(const OverlayKey& myKey, KadPolicy* policy, size_type accelerationFactor=1, size_type bucketSize=10);
	/**
	 * Adds the contact to the replacement cache and calls a policy function
	 * to trigger adding the contact to the bucket.
	 */
	void addCandidate(KadContact_p contact);
	/**
	 * Adds the contact to the routing table.
	 * If the corresponding bucket is full, the contact is handled by the
	 * onBucketFull policy.
	 */
	void addContact(KadContact_p contact);

	/**
	 * Removes the contact with the givne key from the routing table.
	 * @returns true if the contact was removed, false if it was not present
	 */
	bool removeContact(const OverlayKey& key);
	/** Removes the contact from the routing table */
	inline bool removeContact(KadContact_p contact) {
		ASSERT(getContact(contact->getKey()) == contact);
		return removeContact(contact->getKey());
	}
	/**
	 * Returns the contact with the specified key.
	 * Returns a nullptr if the contact was not found.
	 */
	KadContact_p getContact(const OverlayKey& key);
	/**
	 * Returns a list of n contacts that are closest to the given key if possible.
	 * If there are not enough contacts found, the list is shorter.
	 */
	KadContactList getClosestContacts(const OverlayKey& key, size_type n);
	/// returns the number of contacts in buckets
	size_type getNumContacts();
	/// returns the number of contact replacements in buckets
	size_type getNumReplacements();
	/// returns the number of buckets currenty held
	inline size_type getNumBuckets() { return buckets.size(); }
	/// returns true if the key is known, i.e. there is a contact present
	inline bool isKnown(const OverlayKey& key) { return getContact(key).get() != nullptr; }

	/// returns the distance between the two keys
	inline OverlayKey distance(const OverlayKey& a, const OverlayKey& b) { return a ^ b; }
	/// returns the distance of the given key to my key
	inline OverlayKey distance(const OverlayKey& key) { return key ^ myKey; }
	
protected:
	/** @name bucket access */
	//@{
	/** returns the index of the bucket corresponding to the given overlay key. */
	size_type getBucketIndex(const OverlayKey& key);
	/** splits the deepest bucket */
	void splitLastBucket();
	/** returns the index of the bucket corresponding to the given contact */
	inline size_type getBucketIndex(KadContact_p contact) { return getBucketIndex(contact->getKey());}

	/** returns the bucket corresponding to the given overlay key */
	KadBucket& getBucket(const OverlayKey& key);
	/** returns the bucket corresponding to the given contact */
	inline KadBucket& getBucket(KadContact_p contact) { return getBucket(contact->getKey()); }
	/** returns true if we could split this bucket */
	inline bool isSplittable(KadBucket& bucket) { return &bucket == &buckets.back() && buckets.size() < getMaxNumBuckets(); }

	/** returns the width of a level in the accelerated lookup tree */
	inline size_type getLevelWidth() { return (1 << accelerationFactor) - 1; }
	
	/** returns the theoretica number of buckets that we could have */
	inline size_type getMaxNumBuckets() { return myKey.getLength()/accelerationFactor * getLevelWidth(); }

	/** returns the index of the bucket that is the first on the level of the given bucket */
	inline size_type getFirstOnBucketLevel(size_type index) { return index - (index % getLevelWidth()); }
	//@}

private:
	/**
	 * number of bits used for accelerated lookup
	 * called "b" in research papers.
	 */
	size_type accelerationFactor;
	/** the key of this overlay node */
	OverlayKey myKey = OverlayKey::UNSPECIFIED_KEY;
	/** max size of the buckets, called "k" in research papers (k-Buckets) */
	size_type bucketSize;
	std::vector<KadBucket> buckets;
	KadPolicy* policy = nullptr;

	//------ private inline helper functions -------
	/** returns a random access iterator situated on the bucket with the given index */
	auto inline getBucketIterator(size_type index=0) {
		auto it=buckets.begin();
		std::advance(it, index);
		return it;
	}

	/** copies all contacts from one contact list to the end of the other contact list*/
	void inline copyContacts(KadContactList& source, KadContactList target) {
		std::copy(source.begin(), source.end(), std::back_inserter(target));
	}
};

} //namespace

#endif // __ROUTINGSIM_KadRoutingTalbe_H_
