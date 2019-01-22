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
#include <algorithm>
#include "BaseTypes.h"

namespace routingsim {
void KadRoutingTable::init(const OverlayKey& myKey, KadPolicy* policy, size_type accelerationFactor, size_type bucketSize)
{
	ASSERT(this->myKey.isUnspecified());
	ASSERT(not myKey.isUnspecified());
	this->myKey = myKey;
	this->accelerationFactor = accelerationFactor;
	this->bucketSize = bucketSize;
	this->policy = policy;
	size_type numBuckets = myKey.getLength()/accelerationFactor * ((1 << accelerationFactor) - 1);
	ASSERT(buckets.size() == 0);
	buckets.reserve(numBuckets);
	for(size_type i=0; i<numBuckets; i++){
		buckets.emplace_back();
	}
	WATCH_VECTOR(buckets);
}

size_type KadRoutingTable::getBucketIndex(const OverlayKey& key)
{
	// based on OverSim Kademlia.cc
	// Copyright (C) 2006 Institut fuer Telematik, Universitaet Karlsruhe (TH)
	// licenced GPLv2 or later
	// calculate XOR distance
	OverlayKey delta = key ^ myKey;
	// find first subinteger that is not zero...
	int i;
	for (i = key.getLength() - accelerationFactor; i >= 0 && delta.getBitRange(i, accelerationFactor) == 0;
		i -= accelerationFactor);
	if (i < 0)
		throw cRuntimeError("You cannot get the bucket index of your own key!");
	// i is now the index of the LSB of the first non zero subinteger
	// we reverse the bucket order. now the bigges bucket is on the left.
	// for accelerationFactor = 1 this is just (key.getLength() - i)
	size_type index = (key.getLength()/accelerationFactor - i) * getLevelWidth() - delta.getBitRange(i, accelerationFactor) + 1;
	return min(buckets.size()-1, index);
}

KadContactList KadRoutingTable::getClosestContacts(const OverlayKey& key, size_type n)
{
	KadContactList result{};
	// find correct bucket and copy contents
	size_type index = getBucketIndex(key);
	auto bucket = getBucketIterator(index);
	copyContacts(bucket->contacts, result);
	if(result.size() < n){
		// not enough contacts in first bucket
		// now we scan the level of the accelerated lookup
		// and then we go down the tree
		size_type firstOnLevelIndex = getFirstOnBucketLevel(index);
		auto otherBucket = getBucketIterator(firstOnLevelIndex);
		while(result.size() < n && otherBucket != buckets.end()) {
			if(otherBucket == bucket)
				continue;
			copyContacts(otherBucket->contacts, result);
			otherBucket++;
		}
		otherBucket = getBucketIterator(firstOnLevelIndex);
		// if we still do not have enough contacts, we go up the tree
		while(result.size() < n && otherBucket != buckets.begin()) {
			otherBucket--;
			copyContacts(otherBucket->contacts, result);
		}
	}
	// now we sort the keys
	struct KadContactCompare{
		const OverlayKey& key;
		KadContactCompare(const OverlayKey& key) : key(key) {}
		bool operator()(KadContact_p& a, KadContact_p& b) const {
			return (a->getKey() ^ key) < (b->getKey() ^ key);
		}
    };
	std::sort(result.begin(), result.end(), KadContactCompare(key));
	// if we have found move than n contacts, we need to remove some
	if (result.size() > n) {
		result.resize(n);
	}
	result.shrink_to_fit();
	return result;
}

void KadRoutingTable::addCandidate(KadContact_p contact)
{
	EV << "adding candidate " << *contact << endl;
	// contact must not be present
	if(getContact(contact->getKey())) {
		KadContact_p other = getContact(contact->getKey());
		EV << "contact with key allready present" << contact << endl;
		ASSERT(contact == other);
		return;
	}
	KadBucket& bucket = getBucket(contact);
	if (bucket.contactReplacements.size() == bucketSize) {
		ASSERT(bucket.size() == bucketSize);
		return; // the bucket is completely full!
	}
	else {
		bucket.contactReplacements.push_back(contact);
		if (bucket.size() < bucketSize)
			policy->onBucketSpace(bucket);
	}
}

void KadRoutingTable::addContact(KadContact_p contact)
{
	EV << "adding contact" << *contact << endl;
	// contact must not be present
	// TODO could it be in the replacements?
	ASSERT(not getContact(contact->getKey()));
	KadBucket& bucket = getBucket(contact);
	if (bucket.size() == bucketSize)
		policy->onBucketFull(bucket, contact);
	else
		bucket.contacts.push_back(contact);
}


bool KadRoutingTable::removeContact(const OverlayKey& key)
{
	KadBucket& bucket = getBucket(key);
	bool result = bucket.removeContact(key);
	if (bucket.empty())
		policy->onBucketEmpty(bucket);
	if (result)
		policy->onBucketSpace(bucket);
	else
		bucket.removeContactFromReplacements(key);
	return result;
}

KadBucket& KadRoutingTable::getBucket(const OverlayKey& key)
{
	size_type index = getBucketIndex(key);
	return buckets.at(index);
}

KadContact_p KadRoutingTable::getContact(const OverlayKey& key)
{
	return getBucket(key).getContact(key);
}

//----------- KadBucket --------------
KadContact_p KadBucket::getContact(const OverlayKey& key)
{

	auto it = findContactInList(contacts, key);
	if (it != contacts.end())
		return *it;
	it = findContactInList(contactReplacements, key);
	if (it != contactReplacements.end())
		return *it;
	return KadContact_p();
}

void KadBucket::replaceContact(KadContact_p oldContact, KadContact_p newContact)
{
	auto oldIt = findContactInList(contacts, oldContact->getKey());
	ASSERT(oldContact == *oldIt);
	ASSERT(findContactInList(contacts, newContact->getKey()) == contacts.end());
	ASSERT(findContactInList(contactReplacements, newContact->getKey()) == contactReplacements.end());
	*oldIt = std::move(newContact);
	// TODO add old contact to contact replacements?
}

} // namespace
