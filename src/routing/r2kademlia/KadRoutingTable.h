
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
	KadContact_p getContact(const OverlayKey& key);
	bool removeContact(const OverlayKey& key) { return removeContactFromList(contacts, key);}
	bool removeContactFromReplacements(const OverlayKey& key) { return removeContactFromList(refillContacts, key);}
	bool empty() { return contacts->empty(); }
	size_type size() { return contacts->size(); }
protected:
	KadContactList_p contacts;
	KadContactList_p refillContacts;
};

/**
 */
class KadRoutingTable
{
	friend KadBucket;
public:
	KadRoutingTable(const OverlayKey& myKey, size_type b=1, size_type bucketSize=20);
	bool addContact(KadContact_p contact);
	bool removeContact(const OverlayKey& key);
	bool removeContact(KadContact_p contact) { return removeContact(contact->getKey()); }
	KadContact_p getContact(const OverlayKey& key);
	KadContactList_p getClosestContacts(const OverlayKey& key, size_type n);
	
protected:
	/** returns the index of the bucket corresponding to the given overlay key. */
	size_type getBucketIndex(const OverlayKey& key);

	/** returns the index of the bucket corresponding to the given contact */
	inline size_type getBucketIndex(KadContact_p contact) { return getBucketIndex(contact->getKey());}

	/** returns the bucket corresponding to the given overlay key */
	KadBucket* getBucket(const OverlayKey& key);
	/** returns the bucket corresponding to the given contact */
	inline KadBucket* getBucket(KadContact_p contact) { return getBucket(contact->getKey());}

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
	void onBucketFull(KadBucket* bucket, KadContact_p contact);

private:
	/** acceleration factor, number of bits used for accelerated lookup */
	const size_type b;
	/** the key of this overlay node */
	const OverlayKey myKey;
	/** max size of the buckets, called "k" in research papers (k-Buckets) */
	const size_type bucketSize;
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
