//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// as published by the Free Software Foundation; either version 2
// of the License, or (at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with this program; if not, write to the Free Software
// Foundation, Inc., 59 Temple Place - Suite 330, Boston, MA  02111-1307, USA.

/**
 * @file KadRoutingTable.cc
 * @author Jakob Thomas
 */

#include "KadRoutingTable.h"

namespace routingsim {
KadRoutingTable::KadRoutingTable(const OverlayKey& myKey, size_type b, size_type bucketSize):
	b(b), myKey(myKey), bucketSize(bucketSize)
{
	size_type numBuckets = myKey.getLength()/b * ((1 << b) - 1);
	buckets.reserve(numBuckets);
	for(size_type i=0; i<numBuckets; i++){
		buckets.emplace_back();
	}
}

size_type KadRoutingTable::getBucketIndex(const OverlayKey& key)
{
	// taken from OverSim Kademlia.cc
	// Copyright (C) 2006 Institut fuer Telematik, Universitaet Karlsruhe (TH)
	// licenced GPLv2 or later
	// calculate XOR distance
	OverlayKey delta = key ^ myKey;
	// find first subinteger that is not zero...
	int i;
	for (i = key.getLength() - b; i >= 0 && delta.getBitRange(i, b) == 0;
		i -= b);
	if (i < 0)
		return -1;
	return (i / b) * ((1 << b) - 1) + (delta.getBitRange(i, b) - 1);
}

KadContactList_p KadRoutingTable::getClosestContacts(const OverlayKey& key, size_type n)
{
	KadContactList_p result = std::make_unique<KadContactList>();
	// find correct bucket and copy contents
	size_type index = getBucketIndex(key);
	auto bucket = getBucketIterator(index);
	copyContacts(*(bucket->contacts), *result);
	if(result->size() < n){
		// not enough contacts in first bucket
		// now we scan the level of the accelerated lookup
		// and then we go down the tree
		size_type firstOnLevelIndex = getFirstOnBucketLevel(index);
		auto otherBucket = getBucketIterator(firstOnLevelIndex);
		while(result->size() < n && otherBucket != buckets.end()){
			if(otherBucket == bucket)
				continue;
			copyContacts(*(otherBucket->contacts), *result);
			otherBucket++;
		}
		otherBucket = getBucketIterator(firstOnLevelIndex);
		// if we still do not have enough contacts, we go up the tree
		while(result->size() < n && otherBucket != buckets.begin()){
			otherBucket--;
			copyContacts(*(otherBucket->contacts), *result);
		}
	}
	//TODO if whe have found move than n contacts, we need to remove some...
	return result;
}

bool KadRoutingTable::addContact(KadContact_p contact)
{
	KadBucket* bucket = getBucket(contact);
	if (bucket->size() == bucketSize){
		onBucketFull(bucket, contact);
		return false;
	}
	else {
		bucket->contacts->push_back(contact);
		return true;
	}
}

bool KadRoutingTable::removeContact(const OverlayKey& key)
{
	KadBucket* bucket = getBucket(key);
	bool result = bucket->removeContact(key);
	if (bucket->empty())
		onBucketEmpty(bucket);
	if (result)
		onBucketSpace(bucket);
	else
		bucket->removeContactFromReplacements(key);
	return result;
}

KadBucket* KadRoutingTable::getBucket(const OverlayKey& key)
{
	size_type index = getBucketIndex(key);
	return &buckets.at(index);
}

KadContact_p KadRoutingTable::getContact(const OverlayKey& key)
{
	KadBucket* bucket = getBucket(key);
	return bucket->getContact(key);
}

void KadRoutingTable::onBucketFull(KadBucket* bucket, KadContact_p contact)
{
	if (bucket->refillContacts->size() < bucketSize){
		bucket->refillContacts->push_back(contact);
	}
}

//----------- KadBucket --------------
KadContact_p KadBucket::getContact(const OverlayKey& key)
{

	auto it = findContactInList(contacts, key);
	if (it != contacts->end())
		return *it;
	it = findContactInList(refillContacts, key);
	if (it != refillContacts->end())
		return *it;
	return KadContact_p();
}

} // namespace
