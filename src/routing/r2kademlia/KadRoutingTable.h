
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
	OverlayKey key;
	kad_path path;
	bool stale;
	OverlayKey getOverlayKey() { return key; }
	bool hasKey(const OverlayKey& other) { return other == key; }
};
using KadContact_p = std::shared_ptr<KadContact>;
using KadContactList = std::vector<KadContact_p>;

class KadBucket
{
public:
	std::unique_ptr<KadContactList> contacts;
	std::unique_ptr<KadContactList> refillContacts;
	KadContact_p getContact(const OverlayKey& key);
};
using KadBucket_p = std::shared_ptr<KadBucket>;

/**
 */
class KadRoutingTable
{
public:
	bool addContact(KadContact_p contact);
	bool removeContact(const OverlayKey& key);
	bool removeContact(KadContact_p contact);
	KadContact_p getContact(const OverlayKey& key);
	std::unique_ptr<KadContactList> getClosestContacts(OverlayKey key, int n);
	
protected:
	/** returns the index of the bucket corresponding to the given overlay key. */
	size_type getBucketIndex(const OverlayKey& key);

	/** returns the index of the bucket corresponding to the given contact */
	size_type getBucketIndex(KadContact_p contact) { return getBucketIndex(contact->getOverlayKey());}

	/** returns the bucket corresponding to the given overlay key */
	KadBucket* getBucket(const OverlayKey& key);

	/** returns the index of the bucket that is the first on the level of the given bucket */
	size_type getFirstOnBucketLevel(size_type index) { return index - (index % ((1 << b) - 1));}

	// policy foo
	/** called when a new contact is discovered */
	void onDiscovery(KadContact_p);
	/** called when we hear about a node id */
	void onRumor(const OverlayKey& key);
	/** called when there is space in a bucket */
	void onBucketSpace(KadBucket_p bucket);
	/**  called when a bucket needs to be refreshed */
	void onBucketRefresh(KadBucket_p bucket);
	/** called when a contact could be replaced by a new one */
	void onReplacementPossibility(KadContact_p contact, KadBucket_p bucket);

private:
	/** acceleration factor, number of bits used for accelerated lookup */
	const size_type b=1;
	/** the key of this overlay node */
	const OverlayKey myKey=OverlayKey::ZERO;
	size_type numBuckets;
	std::vector<KadBucket> buckets;
};

} //namespace

#endif // __ROUTINGSIM_KadRoutingTalbe_H_
