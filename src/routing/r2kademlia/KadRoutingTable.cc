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
#include "KadObserver.h"
#include <algorithm>
#include "BaseTypes.h"

namespace routingsim {

bool containsUndirectedLink(const kad_path& path, const OverlayKey& firstNodeKey, const OverlayKey& secondNodeKey);

	
void KadRoutingTable::init(const OverlayKey& myKey, KadPolicy* policy,
		size_type accelerationFactor, size_type bucketSize,
		bool proximityNeighborSelection, bool pathSimplification)
{
	ASSERT(this->myKey.isUnspecified());
	ASSERT(not myKey.isUnspecified());
	this->myKey = myKey;
	this->accelerationFactor = accelerationFactor;
	this->bucketSize = bucketSize;
	this->policy = policy;
	this->proximityNeighborSelection = proximityNeighborSelection;
	this->pathSimplification = pathSimplification;
	ASSERT(buckets.size() == 0);
	//TODO optimization: implement adaptively
	buckets.reserve(5);
	buckets.emplace_back();
	maxNumBuckets = myKey.getLength()/accelerationFactor * getLevelWidth();
	// if the length is not divisible by the acceleration factor,
	// we add a possible bucket for the remainder on the end
	if ((myKey.getLength() % accelerationFactor) != 0)
			maxNumBuckets++;
	WATCH_VECTOR(buckets);
}


std::string KadRoutingTable::str() const
{
	std::ostringstream out;

	unsigned int i= 0;
	for (auto& b : buckets)
	{
		out << "B[" << i << "]:{";
		for (const auto& contact : b.contacts) {
			out << ' ' << *contact;
		}
		out << "} ";
		i++;
	}
	
	return out.str();
}

	
size_type KadRoutingTable::getBucketIndex(const OverlayKey& key) const
{
	OverlayKey delta = key ^ myKey;
	uint prefixLen = delta.sharedPrefixLength(0, accelerationFactor);

	// i is now the index of the LSB of the first non zero sub integer in delta
	int i = delta.getLength() - (prefixLen+1)*accelerationFactor;
	if (i < 0) // this is my key
		return buckets.size() - 1;
	ASSERT(delta.getBitRange(i, accelerationFactor) != 0);
	ASSERT(i + accelerationFactor >= delta.getLength() or
			delta.getBitRange(i + accelerationFactor, accelerationFactor) == 0);

	// bucket 0 is the farthest one from my id.
	// the prefixLen denotes depth of the level
	// levelId = prefixLen
	// levelBaseIndex =  levelId * levelWidth
	// levelOffset = levelWidth - subint
	// index = levelBaseIndex + levelOffset
	// => index = (prefixLen+1) * levelWidth - subint
	size_type index = (prefixLen+1)*getLevelWidth() - delta.getBitRange(i, accelerationFactor);
	ASSERT(index <= getMaxNumBuckets());
	// old calculation formula, just for checking...
	// ASSERT(index == (key.getLength() - i)/accelerationFactor * getLevelWidth()- delta.getBitRange(i, accelerationFactor));
	return std::min(buckets.size()-1, index);
}


/// @returns true in case path was added
bool KadRoutingTable::addPath(const OverlayKey& key, kad_path path)
{
	if (key == myKey) return false;
	
	// we must be careful: some neighbor may tell me paths that lead via me to my neighbors
	// we simply discard this information as it cannot be more current than mine

	const bool pathWasNotEmpty(path.size() > 0);
	// find my key -> this is a poisonous path via me, simply ignore this
	if (std::find(path.begin(), path.end(), myKey) != path.end())
	{
		EV_TRACE << "not adding poisonous path to " << key.shortStr() << " leading via me (" << myKey.shortStr() << "): ";
		for (const auto& node : path)
			EV_TRACE << node.shortStr() << ' ';
		EV_TRACE << endl;
		return false;
	}

	/*
	path.erase(path.begin(), std::find(path.rbegin(), path.rend(), myKey).base());
	// find target key
	path.erase(std::find(path.begin(), path.end(), key), path.end());
	
	if (pathWasNotEmpty && path.empty()==true) {
		// path is now empty -> someone sends path information for a PN that I should know
		EV_TRACE << "Path about my PN " << key.shortStr() << " (leading via me) ignored" << endl;
		return;
	}
	*/

	if (pathSimplification) {
		// find other cycles/duplicates
		simplifyPath(path);
	}
	
	ASSERT(pathWasNotEmpty==false || (pathWasNotEmpty && path.empty()==false));
	
	KadContact *contact = getContactInternal(key);
	if (contact) {
		// contact is known

		// there may be the rare case when we learned the new contact from someone else
		// before we heard the Hello
		if (!contact->isPhysicalNeighbor() && path.empty()==true)
		{
			contact->setPhysicalNeighbor();
			EV_TRACE << "Contact existed already, but seems to be a PN now: " << *contact << endl;
		}

		// sanity check: do not use paths that lead via a valid neighbor as first hop
		if (path.size() > 0) {
			// first hop must be found in neighbortable
			if (not neighbortable.isNeighbor(path[0])) {
				EV_ERROR << "Tried to use add path with invalid physical neighbor "
					 << path[0] << " as next hop - ignoring";
				return false;
			}
		}
		
		// if contact was marked invalid, update path and status:
		// since this information is hopefully more current, path should be valid (again)
		// it does not make sense to check for equalness of the paths: it may be the same,
		// but re-enabled
		// or: we learn a shorter path -> update anyways
		if (contact->isInvalid() || contact->getPathLength() > path.size()) {
			contact->setPath(std::move(path)); // will set contact automatically to valid
			updateCount++;
			
			// do something on improved path according to policy
			policy->onImprovement(*contact);

			return true;
		}
		// else contact exists, but new path is not interesting
		
		return false;
	}
	else // contact is new
	{
		return handleNewContact(key, std::move(path));
	}
}


/// @returns true in case path was added
bool KadRoutingTable::addPath(const OverlayKey& key, const kad_path& path, const kad_path& viaPath)
{
	if (key == myKey)
		return false;

	if (pathSimplification) {
		return addPath(key, concatViaPath(viaPath, path));
	}

	KadContact *contact = getContactInternal(key);
	if (contact) { // contact is known

		// there may be the rare case when we learned the new contact from someone else
		// before we heard the Hello
		if (!contact->isPhysicalNeighbor() && concatViaPath(viaPath, path).empty())
		{
			contact->setPhysicalNeighbor();
			EV_TRACE << "Contact existed already, but seems to be a PN now: " << *contact << endl;
		}
		
		// if contact was marked invalid, update path and status:
		// since this information is hopefully more current, path should be valid (again)
		// it does not make sense to check for equalness of the paths: it may be the same,
		// but re-enabled
		// or: we learn a shorter path -> update anyways
		if (contact->isInvalid() || contact->getPathLength() > path.size() + viaPath.size()) {
			updateCount++;
			contact->setPath(concatViaPath(viaPath, path)); // will set contact automatically to valid
			
			// do something on improved path according to policy
			policy->onImprovement(*contact);
			
			return true;
		}
		// else contact exists, but new path is not interesting
		EV_TRACE << "Contact known, but new path not added " << *contact << endl;

		return false;
	}
	else // contact is new
	{
		return handleNewContact(key, concatViaPath(viaPath, path));
	}
}

	
void KadRoutingTable::simplifyPath(kad_path& path)
{
	if (path.size() < 2) return;
	for (auto right = path.end()-1; right != path.begin(); right--) {
		auto left = std::find(path.begin(), right, *right);
		if (left != right) {
			ASSERT(*left == *right);
			path.erase(left, right);
			return;
		}
	}
}


// @returns true if contact was inserted into routing table, false if not
bool KadRoutingTable::handleNewContact(const OverlayKey& key, kad_path path)
{
	ASSERT(key != myKey);
	KadBucket& bucket = getBucket(key);
	ASSERT(!bucket.getContact(key));
	if (isSplittable(bucket)) {
		splitLastBucket();
		// we recurse to get the new bucket after the split
		return handleNewContact(key, std::move(path));
	}

	auto contact = std::make_unique<KadContact>(key, std::move(path));
	if (bucket.size() < bucketSize)
	{
		policy->onNewContact(contact.get());
		bucket.contacts.push_back(std::move(contact));
		
		return true;
	}
	else {
		if (not proximityNeighborSelection) {
			EV_INFO << "dropping new contact " << *contact << endl;
			return false;
		}
		// contact replacement
		auto& contacts = bucket.contacts;
		auto result = contacts.end();
		size_type maxLen = contact->getPathLength();
		for (auto it = contacts.begin(); it!=contacts.end(); it++) {
			if( (*it)->getPathLength() > maxLen) {
				maxLen = (*it)->getPathLength();
				result = it;
			}
		}
		if (result != contacts.end()) {
			replacementCount++;
			EV_DEBUG << "replacing contact, new: " << *contact << " old: " << **result << endl;
			*result = std::move(contact);
			policy->onNewContact(result->get());
			
			return true;
		}
		else {
			EV_TRACE << "dropping new contact " << *contact << endl;
			return false;
		}
	}
	
	// just as safety measure in case we fall through here
	return false;
}

void KadRoutingTable::splitLastBucket()
{
	EV_DEBUG << "splitting bucket" << endl;
	ASSERT(isSplittable(buckets.back()));
	ASSERT(buckets.size() < getMaxNumBuckets());
	KadBucket oldBucket{};
	std::swap(oldBucket, buckets.back());
	ASSERT(oldBucket.size() >= bucketSize);
	// create new buckets
	size_type newSize = std::min(buckets.size() + getLevelWidth(), getMaxNumBuckets());
	if (newSize == getMaxNumBuckets()) {
		EV_INFO << "reached full routing table depth" << endl;
	}
	buckets.resize(newSize);
	// sort contacts into new buckets
	for (auto& contact : oldBucket.contacts)
		getBucket(contact.get()).contacts.push_back(std::move(contact));
}


/** copies all contacts from one contact list to the end of the other contact list */
inline void copyContacts(const std::vector<std::unique_ptr<KadContact>>& source, KadContactList& target) {
	target.reserve(target.size() + source.size());
	for (auto it = source.begin(); it != source.end(); it++)
		target.push_back(it->get());
}


/** copies all valid contacts from one contact list to the end of the other contact list */
inline void copyValidContacts(const std::vector<std::unique_ptr<KadContact>>& source, KadContactList& target) {
	target.reserve(target.size() + source.size());
	for (auto it = source.begin(); it != source.end(); it++)
	{
		if (it->get()->isValid())
			target.push_back(it->get());
	}
}


/** 
 * returns at most n contacts closes to key
 * ContactList is sorted according to XOR metric with respect to key, i.e., the first returned contact
 * is the closest to key
 */

KadContactList KadRoutingTable::getClosestContacts(const OverlayKey& key, size_type n) const
{
	KadContactList result{};
	result.reserve(n);
	size_type index = getBucketIndex(key);
	auto bucket = buckets.begin() + index;
	if (bucket->size() >= n)
		copyValidContacts(bucket->contacts, result);
	else {
		// not enough contacts in first bucket
		// now we scan the level of the accelerated lookup
		// and then we go down the tree
		size_type firstOnLevelIndex = getFirstOnBucketLevel(index);
		uint width = getLevelWidth();
		auto level = buckets.begin() + firstOnLevelIndex;
		for (; level < buckets.end() and result.size() < n; level+=width) {
			if (level == buckets.end()-1)
				copyValidContacts(level->contacts, result);
			else // always insert the whole level
				for (uint i = 0; i < width and level+i < buckets.end(); i++)
					copyValidContacts(level[i].contacts, result);
		}
		// if we still do not have enough contacts, we go up the tree
		auto otherBucket = buckets.begin() + firstOnLevelIndex;
		while(result.size() < n && otherBucket != buckets.begin()) {
			otherBucket--;
			copyValidContacts(otherBucket->contacts, result);
		}
	}
	// now we sort the contacts by their XOR metric
	struct KadContactCompare {
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


KadContactList KadRoutingTable::getAllContacts() const
{
	KadContactList result{};
	result.reserve(bucketSize * buckets.size());
	for (auto& bucket : buckets)
		copyContacts(bucket.contacts, result);
	return result;
}


KadContactList KadRoutingTable::getAllValidContacts() const
{
	KadContactList result{};
	result.reserve(bucketSize * buckets.size());
	for (auto& bucket : buckets)
		copyValidContacts(bucket.contacts, result);
	return result;
}


size_type KadRoutingTable::getNumContacts() const
{
	size_type sum = 0;
	for (const KadBucket& b : buckets)
		sum += b.size();
	return sum;
}


/** invalidate all contacts with paths that contain a failed link otherNodeKey <-> failedNodeKey */
void KadRoutingTable::invalidateContacts(const OverlayKey& failedNodeKey, const OverlayKey& otherNodeKey)
{
	EV_TRACE << "invalidateContacts(" << failedNodeKey << ")" << endl;
	// iterate over all buckets and contacts
	for (KadBucket& b : buckets)
		for (auto& contact : b.contacts) {
			// if we find a failed link in the path, mark contact invalid
			const auto& path= contact->getPath();
			if ((contact->getKey() == failedNodeKey
			     && path.size() > 0
			     && path[path.size()-1] == otherNodeKey
			    ) // pen-ultimate hop is neighbor node of failed contact => this link failed
			    ||
			    (contact->getKey() == otherNodeKey
			     && path.size() > 0
			     && path[path.size()-1] == failedNodeKey
			    )
			    || // pen-ultimate hop is the failed node leading to the neighboring node => this link failed
			    containsUndirectedLink(contact->getPath(), failedNodeKey, otherNodeKey))
			{ // link somewhere in the path
				contact->setInvalid();
				EV_TRACE << "Set Contact to invalid due to failed link <"
					 << failedNodeKey.toString().substr(0, 6) << "," << otherNodeKey.toString().substr(0, 6) << ">:"  << *contact << endl;
			}
		} // end for inner loop over all contacts
}


/** invalidate all contacts with paths that contain the physical neighbor failedNodeKey as next hop */
void KadRoutingTable::invalidateContactsPN(const OverlayKey& failedNodeKey)
{
	EV_TRACE << "invalidateContactsPN(" << failedNodeKey << ")" << endl;
	// iterate over all buckets and contacts
	for (KadBucket& b : buckets)
		for (auto& contact : b.contacts) {
			// we found a failed link in the path, mark contact invalid
			const auto& path= contact->getPath();
			if (path.size() >= 1)
			{
				if (path[0] == failedNodeKey) // failed node is first hop
				{
					contact->setInvalid();
					EV_TRACE << "Set Contact to invalid due to PN first hop failure: "  << *contact << endl;
					// will schedule this for rediscovery later
					policy->scheduleRediscovery(contact->getKey(), failedNodeKey, myKey);
				}
			}
		} // end for inner loop over all contacts
}


double KadRoutingTable::getAveragePathLength() const
{
	uint count = 0;
	double sum = 0;
	for (const KadBucket& b : buckets)
		for (auto& contact : b.contacts) {
			sum += contact->getPathLength();
			count++;
		}
	return count == 0 ? 0.0 : sum / count;
}


double KadRoutingTable::getAveragePathStretch() const
{
	uint count = 0;
	double sum = 0;
	for (const KadBucket& b : buckets)
		for (auto& contact : b.contacts) {
			if (getKadObserver().getPhysicalDistance(myKey, contact->getKey()) > 0)
			    sum += (contact->getPathLength()+1) / getKadObserver().getPhysicalDistance(myKey, contact->getKey());
			else
				EV_WARN << "Physical Distance = " << getKadObserver().getPhysicalDistance(myKey, contact->getKey())
					<< ": " << myKey << " to " << contact->getKey().toString().substr(0, 6) << endl;
			count++;
		}
	return count == 0 ? 0.0 : sum / count;
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


KadContact_p KadRoutingTable::getContact(const OverlayKey& key) const
{
	return getBucket(key).getContact(key);
}


kad_path KadRoutingTable::concatViaPath(const kad_path& viaPath, const kad_path& path) const
{
	kad_path newPath{};
	auto left = viaPath.end();
	auto right = pathSimplification ? path.rbegin() : path.rend();
	for (;right != path.rend() and left == viaPath.end(); right++) {
		left = std::find(viaPath.begin(), viaPath.end(), *right);
	}
	newPath.reserve((left - viaPath.begin()) + (path.end() - right.base()));
	std::copy(viaPath.begin(), left, std::back_inserter(newPath));
	std::copy(right.base(), path.end(), std::back_inserter(newPath));

	return newPath;
}

//----------- KadBucket --------------
KadContact *KadBucket::getContact(const OverlayKey& key)
{
	auto it = findContact(key);
	if (it != contacts.end())
		return it->get();
	return nullptr;
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

std::vector<std::unique_ptr<KadContact>>::iterator KadBucket::findContact(const OverlayKey& key)
{
	auto it = contacts.begin();
	for (; it !=  contacts.end() && !(*it)->hasKey(key); it++);
	return it;
}

// ---------------- other class independent methods ----------------

/// searches for undirected link in given path, returns true if found, false if not found
bool containsUndirectedLink(const kad_path& path, const OverlayKey& firstNodeKey, const OverlayKey& secondNodeKey)
{
	if (path.size() >= 2) // only for path that contain links
	{
		const OverlayKey* prev_node= &path[0];
		// now loop over all nodes contained in the path
		for (unsigned int i= 0; i != path.size(); i++)
		{
			if (path[i] == firstNodeKey) 
			{ // one end of the link detected
				if (*prev_node == secondNodeKey)
				{ // previous node is the other end of the link
					return true;
				}
				else
				{ // check if next hop is other end of the link
					if ((i+1) < path.size())
					{ // next hop available
						if (path[i+1] == secondNodeKey) // next hop is second node
							return true;
					}
					else
					{ // current node is last hop -> no link found
						return false;
					}
				}
			}
			// current node will be prev_node in next iteration
			prev_node= &path[i];
		} // end for
		// if here: for loop is finished, but not found so far
		return false;
	}
	else
	{
		return false;
	}
}

} // namespace
