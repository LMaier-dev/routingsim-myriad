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
	size_type numBuckets = getLevelWidth() + 1;
	ASSERT(buckets.size() == 0);
	buckets.reserve(getMaxNumBuckets());
	for (size_type i = 0; i < numBuckets; i++) {
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
	// for accelerationFactor = 1 this is just (key.getLength() - i - 1)
	size_type index = (key.getLength() - i - 1)/accelerationFactor * getLevelWidth() - delta.getBitRange(i, accelerationFactor) + 1;
	ASSERT(index <= getMaxNumBuckets());
	return std::min(buckets.size()-1, index);
}

void KadRoutingTable::addPath(const OverlayKey& key, kad_path path)
{
	if (key == myKey) return;
	KadContact_p contact = getContact(key);
	if (contact) {
		// contact is known
		if (contact->getPathLength() > path.size()) {
			contact->setPath(std::move(path));
			//TODO onImprovement?
			return;
		}
		// else new path is not interesting
	}
	else // contact is new
		handleNewContact(key, std::move(path));
}

void KadRoutingTable::addPath(const OverlayKey& key, const kad_path& path, const OverlayKey& via)
{
	if (key == myKey) return;
	KadContact_p viaContact = getContact(via);
	if (!viaContact) {
		EV_ERROR << "Via discovery failed, via contact not known." << endl;
		return;
	}
	KadContact_p contact = getContact(key);
	if (contact) {
		// contact is known
		if (contact->getPathLength() > path.size() + 1 + viaContact->getPathLength()) {
			contact->setPath(concatViaPath(*viaContact, path));
			//TODO onImprovement?
			return;
		}
		// else new path is not interesting
	}
	else // contact is new
		handleNewContact(key, concatViaPath(*viaContact, path));
}
void KadRoutingTable::handleNewContact(const OverlayKey& key, kad_path path)
{
	KadContact_p contact = make_shared<KadContact>(key, std::move(path));
	KadBucket& bucket = getBucket(key);
	ASSERT(!bucket.getContact(key));
	if(bucket.size() < bucketSize) {
		bucket.contacts.push_back(std::move(contact));
		return;
	}
	else {
		// TODO replacement
	}
}

void KadRoutingTable::splitLastBucket() {
	EV << "splitting bucket" << endl;
	ASSERT(buckets.size() < getMaxNumBuckets());
	KadBucket oldBucket{};
	std::swap(oldBucket, buckets.back());
	ASSERT(oldBucket.size() >= bucketSize);
	// create new buckets
	size_type newSize = buckets.size() + getLevelWidth();
	if (newSize > getMaxNumBuckets()) {
		ASSERT(newSize == getMaxNumBuckets()-1);
		newSize--;
		// this happens when we create the last level
		// then we do not need a to create a last bucket to catch contacts that
		// need to be resorted later
		EV << "reached full routing table depth" << endl;
	}
	buckets.reserve(newSize);
	while(buckets.size() < newSize)
		buckets.emplace_back();
	// sort contacts into new buckets
	for(KadContact_p contact : oldBucket.contacts)
		getBucket(contact).contacts.push_back(contact);
	// TODO optimization: use std::move for contacts
}

KadContactList KadRoutingTable::getClosestContacts(const OverlayKey& key, size_type n)
{
	KadContactList result{};
	// find correct bucket and copy contents
	size_type index = getBucketIndex(key);
	auto bucket = buckets.begin() + index;
	copyContacts(bucket->contacts, result);
	if(result.size() < n){
		// not enough contacts in first bucket
		// now we scan the level of the accelerated lookup
		// and then we go down the tree
		size_type firstOnLevelIndex = getFirstOnBucketLevel(index);
		auto otherBucket = buckets.begin() + firstOnLevelIndex;
		while(result.size() < n && otherBucket != buckets.end()) {
			if (otherBucket != bucket)
				copyContacts(otherBucket->contacts, result);
			otherBucket++;
		}
		otherBucket = buckets.begin() + firstOnLevelIndex;
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

size_type KadRoutingTable::getNumContacts() {
	size_type sum = 0;
	for (KadBucket& b : buckets)
		sum += b.size();
	return sum;
}


bool KadRoutingTable::removeContact(const OverlayKey& key)
{
	KadBucket& bucket = getBucket(key);
	bool result = bucket.removeContact(key);
	if (!result)
		EV << "failed to remove contact " << key << endl;
	if (bucket.empty())
		policy->onBucketEmpty(bucket);
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

kad_path KadRoutingTable::concatViaPath(KadContact& viaContact, const kad_path& path) {
	kad_path newPath{};
	newPath.reserve(viaContact.getPathLength() + 1 + path.size());
	std::copy(viaContact.getPath().begin(), viaContact.getPath().end(), std::back_inserter(newPath));
	newPath.push_back(viaContact.getKey());
	std::copy(path.begin(), path.end(), std::back_inserter(newPath));
	return newPath;
}

//----------- KadBucket --------------
KadContact_p KadBucket::getContact(const OverlayKey& key)
{
	auto it = findContact(key);
	if (it != contacts.end())
		return *it;
	return KadContact_p();
}

void KadBucket::replaceContact(KadContact_p oldContact, KadContact_p newContact)
{
	auto oldIt = findContact(oldContact->getKey());
	ASSERT(oldContact == *oldIt);
	ASSERT(findContact(newContact->getKey()) == contacts.end());
	*oldIt = std::move(newContact);
}

bool KadBucket::removeContact(const OverlayKey& key)
{
	auto it = findContact(key);
	if (it != contacts.end()){
		contacts.erase(it);
		return true;
	}
	else
		return false;
}

std::vector<KadContact_p>::iterator KadBucket::findContact(const OverlayKey& key)
{
	auto it = contacts.begin();
	for (; it !=  contacts.end() && !(*it)->hasKey(key); it++);
	return it;
}

} // namespace
