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
#include <cmath>
#include <algorithm>
#include "KadContact.h"
/**
 * @file KadRoutingTable.h
 * @author Jakob Thomas
 */

namespace routingsim {
class KadRoutingTable;

// contact type definitions
using KadContact_p = KadContact*;
using KadContactList = std::vector<KadContact_p>;

class KadBucket
{
	friend KadRoutingTable;
public:
	/** returns the contact if found, nullptr otherwise */
	KadContact_p getContact(const OverlayKey& key);
	/** tries to remove the contact, returns true if found */
	inline bool removeContact(const OverlayKey& key);
	/** returns true if the contact list is empty */
	inline bool empty() { return contacts.empty(); }
	/** returns the size of the contact list */
	inline size_type size() { return contacts.size(); }
	/// Common stdc++ console output method
	friend std::ostream& operator<<(std::ostream& os, const KadBucket& b) {
		return os << b.contacts.size() << " contacts";
	}
protected:
	/**
	 * Returns an iterator at the contacts position.
	 * Returns contacts->end() if the contact is not found.
	 */
	std::vector<std::unique_ptr<KadContact>>::iterator findContact(const OverlayKey& key);
	std::vector<std::unique_ptr<KadContact>> contacts;
};

/**
 * A class with callback functions that are called then the routing table
 * encounters things.
 */
class KadPolicy {
public:
	/** called when last contact is removed from a bucket*/
	virtual void onBucketEmpty(KadBucket& bucket) = 0;
	/**  called when a bucket needs to be refreshed */
	virtual void onBucketRefresh(KadBucket& bucket) = 0;
	virtual void onNewContact(KadContact_p contact) = 0;
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
	void init(const OverlayKey& myKey, KadPolicy* policy, size_type accelerationFactor, size_type bucketSize, bool proximityNeighborSelection);
	/**
	 * add a potentially new path, adds a contactz or updates an existing one
	 */
	void addPath(const OverlayKey& key, kad_path path);
	/**
	 * add a potentially new path, adds a contact or updates an existing one
	 */
	void addPath(const OverlayKey& key, const kad_path& path, const kad_path& via);
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
	/// returns the number of buckets currenty held
	inline size_type getNumBuckets() { return buckets.size(); }
	/// the position of the bit that defines the lowest bucket
	inline size_type getDepth() { return std::min<size_type>(ceil(getNumBuckets() / getLevelWidth()) * accelerationFactor, myKey.getLength()); }
	/// calculates the average path length of the known contacts
	double getAveragePathLength();
	/// calculates the avarage stretch of the known contacts
	double getAveragePathStretch();
	/// returns the maximum size of a bucket, i.e. the size after which a bucket is split
	inline size_type getSplitSize() { return bucketSize; }
	/// returns true if the key is known, i.e. there is a contact present
	inline bool isKnown(const OverlayKey& key) { return getContact(key) != nullptr; }

	/// returns the distance between the two keys
	inline OverlayKey distance(const OverlayKey& a, const OverlayKey& b) { return a ^ b; }
	/// returns the distance of the given key to my key
	inline OverlayKey distance(const OverlayKey& key) { return key ^ myKey; }

protected:
	virtual void handleNewContact(const OverlayKey& key, kad_path path);
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

	/** returns the theoretical number of buckets that we could have */
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
	bool proximityNeighborSelection;

	//------ private helper functions -------
	/** copies all contacts from one contact list to the end of the other contact list*/
	void inline copyContacts(std::vector<std::unique_ptr<KadContact>>& source, KadContactList& target) {
		target.reserve(target.size() + source.size());
		for(auto it = source.begin(); it != source.end(); it++)
			target.push_back(it->get());
	}
	/**  */
	kad_path concatViaPath(const kad_path& viaPath, const kad_path& path);
};

} //namespace

#endif // __ROUTINGSIM_KadRoutingTalbe_H_
