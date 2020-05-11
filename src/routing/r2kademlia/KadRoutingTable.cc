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

namespace ph = std::placeholders;
	
void KadRoutingTable::init(const OverlayKey& myKey, KadPolicy* policy,
			   size_type accelerationFactor, size_type bucketSize,
			   bool proximityNeighborSelection, bool pathSimplification, uint8_t discoveryRadius)
{
	ASSERT(this->myKey.isUnspecified());
	ASSERT(not myKey.isUnspecified());
	this->myKey = myKey;
	this->accelerationFactor = accelerationFactor;
	this->bucketSize = bucketSize;
	this->policy = policy;
	this->proximityNeighborSelection = proximityNeighborSelection;
	this->pathSimplification = pathSimplification;
	this->discoveryRadius= discoveryRadius;
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
		out << "}+{";
		for (const auto& econtact : b.PNContacts) {
			out << ' ' << *econtact;
		}
		out << '}';		
		i++;
	}

	return out.str();
}

	
size_type KadRoutingTable::getBucketIndex(const OverlayKey& key) const
{
	OverlayKey delta = key ^ myKey;
	// this returns the prefixlength as number of matching digits
	// (each digit ranges from 0 to 2^accelerationfactor - 1
	uint prefixLen = delta.sharedPrefixLength(0, accelerationFactor);

	// bitindex is now the index of the LSB of the first non-zero digit in delta
	int bitindex = delta.getLength() - (prefixLen+1)*accelerationFactor;
	if (bitindex < 0) // this is my key
		return buckets.size() - 1;

	// bool debuginfo(false);
	// if (delta.getBitRange(i, accelerationFactor) == 0)
	// {
	// 	EV_WARN << "delta.getBitRange(i, accelerationFactor) == 0, key: "
	// 		<< key << " nodekey: " << myKey << " delta: " << delta << endl;
	// 	debuginfo= true;
	// }
	// bitindex is the LSB of the first non-zero digit, so digit must not be zero
	ASSERT(delta.getBitRange(bitindex, accelerationFactor) != 0);

	// if (not (bitindex + accelerationFactor >= delta.getLength() or
	// 	 delta.getBitRange(bitindex + accelerationFactor, accelerationFactor) == 0))
	// {
	// 	EV_WARN << "bitindex + accelerationFactor < delta.getLength() and delta.getBitRange(bitindex + accelerationFactor, accelerationFactor) != 0), key: " << key << " nodekey: " << myKey << " delta: " << delta << endl;
	// 	debuginfo= true;
	// }
	ASSERT(bitindex + accelerationFactor >= delta.getLength() or
		delta.getBitRange(bitindex + accelerationFactor, accelerationFactor) == 0);

	// on each level we have levelWidth buckets:
	// levelWidth = 2^accelerationfactor - 1
	// the prefixLen denotes depth of the level
	// levelId = prefixLen
	// bucket 0 is the farthest one from my id.
	// levelBaseIndex =  levelId * levelWidth
	// levelOffset = levelWidth - digit
	// index = levelBaseIndex + levelOffset
	// => index = (prefixLen+1) * levelWidth - digit
	size_type index = (prefixLen+1)*getLevelWidth() - delta.getBitRange(bitindex, accelerationFactor);
	ASSERT(index <= getMaxNumBuckets());

	// if (debuginfo)
	// {
	// 	EV_TRACE << "getBucketIndex(" << key << ") len/acc=" << delta.getLength() << "/" << accelerationFactor
	// 		 << " bitindex=" << bitindex
	// 		 << " delta=" << delta << " plen=" << prefixLen
	// 		 << " subint=" << (unsigned int) delta.getBitRange(bitindex, accelerationFactor)
	// 		 << " index=" << index << " maxindex=" << buckets.size()-1 << endl;
	// }

	// old calculation formula, just for checking...
	// ASSERT(index == (key.getLength() - bitindex)/accelerationFactor * getLevelWidth()- delta.getBitRange(i, accelerationFactor));
	return std::min(buckets.size()-1, index);
}


/// @returns true in case path was added
bool KadRoutingTable::addPath(const OverlayKey& key, kad_path path, pathseqno_t pathseqno, KadContact::PathProperty pathprop)
{
	if (key == myKey) return false;

	// we must be careful: some neighbor may tell me paths that lead via me to my neighbors
	// we simply discard this information as it cannot be more current than mine

	const bool pathWasNotEmpty(path.size() > 0);
	// find my key -> this is a poisonous path via me, simply ignore this
	if (std::find(path.begin(), path.end(), myKey) != path.end())
	{
		//EV_TRACE << "not adding poisonous path to " << key.shortStr() << " leading via me (" << myKey.shortStr() << "): ";
		//for (const auto& node : path)
		//	EV_TRACE << node.shortStr() << ' ';
		//EV_TRACE << endl;
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

	bool path_was_simplified= false;
	if (pathSimplification) {
		// find other cycles/duplicates
		path_was_simplified= simplifyPath(path);
	}

	// try to shorten the path using local routing table
	if (path.size() > 0 and shortenPath(path))
		path_was_simplified= true;

	// sanity check: do not use paths that lead via an invalid neighbor as first hop
	if (path.size() > 0 and not neighbortable.isNeighbor(path[0]))
	{ // first hop must be found in neighbortable
		//EV_WARN << "Tried to use add path with invalid physical neighbor "
		//	<< path[0] << " as next hop - ignoring" << endl;
		return false;
	}

	if (path_was_simplified) // override the given argument if path was simplified here
		pathprop= KadContact::PathProperty::indirect;

	// sanity check: either path was empty before or it was not and is not empty after simplification
	ASSERT(pathWasNotEmpty==false or (pathWasNotEmpty and path.empty()==false));

	KadContact *contact = getContactInternal(key);
	if (contact) {
		// contact is known

		// there may be the rare case when we learned the new contact from someone else
		// before we heard the Hello
		if (!contact->isPhysicalNeighbor() and path.empty()==true)
		{
			contact->setPhysicalNeighbor();
			contact->setPathProperty(KadContact::PathProperty::direct);

			// move over to PNcontacts			
			KadBucket& bucket = getBucket(key);
			auto it = bucket.findContact(key);
			if (it != bucket.contacts.end()) {
				bucket.PNContacts.push_back(std::move(*it));
				bucket.contacts.erase(it);
			}
			else
				EV_ERROR << "Contact existed already, but not found" << endl;
			
			//EV_TRACE << "Contact existed already, but seems to be a PN now: " << *contact << endl;
		}
		
		// update path if:
		// always when sequence number is actually higher
		// contact is invalid: only if we have newer information (higher sequence number)
		//                     or the same seq# (the path may have changed though and reenabling the contact is important)
		// or: we learn a shorter path (seq# no must be same or newer) -> update anyways

		if (bool seqno_is_ok(contact->getPathSeqNo() <= pathseqno);
		    (contact->isInvalid() and seqno_is_ok)
		    or
		    (seqno_is_ok and path.size() < contact->getPathLength())
		    or
		    (contact->getPathSeqNo() < pathseqno) )
		{
			// in case the path had a failure, check whether the supposed new path actually avoids the broken link
			if (contact->isInvalid() and containsUndirectedLink(myKey, contact->getKey(), path, contact->getRDS().failedLink))
			{ // provided path does not repair broken link
				return false;
			}
			contact->setPath(std::move(path)); // will set contact automatically to valid
			contact->setPathSeqNo(pathseqno);

			contact->setPathProperty(pathprop);
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
		return handleNewContact(key, std::move(path), pathseqno, pathprop);
	}
}


/// @returns true in case path was added
bool KadRoutingTable::addPath(const OverlayKey& key, const kad_path& path, pathseqno_t pathseqno, const kad_path& viaPath)
{
	if (key == myKey)
		return false;

	// if viaPath.size()+path.size()+1 hop (for contact) <= discoveryRadius we can assume that PathID is available there
	KadContact::PathProperty pathPropHint= (viaPath.size()+path.size() < discoveryRadius) ?
		                               KadContact::PathProperty::direct :
		                               KadContact::PathProperty::indirect;

	if (pathSimplification) { // concatenated paths are direct if learned from neighborhood within discovery radius
		return addPath(key, concatViaPath(viaPath, path), pathseqno, pathPropHint);
	}

	KadContact *contact = getContactInternal(key);
	if (contact) { // contact is known

		// there may be the rare case when we learned the new contact from someone else
		// before we heard the Hello
		if (!contact->isPhysicalNeighbor() and concatViaPath(viaPath, path).empty())
		{
			contact->setPhysicalNeighbor();
			
			// move over to PNcontacts
			
			KadBucket& bucket = getBucket(key);
			auto it = bucket.findContact(key);
			if (it != bucket.contacts.end()) {
				bucket.PNContacts.push_back(std::move(*it));
				bucket.contacts.erase(it);
			}
			else
				EV_ERROR << "Contact existed already, but not found" << endl;
			
			//EV_TRACE << "Contact existed already, but seems to be a PN now: " << *contact << endl;
		}
		
		// if contact was marked invalid, update path and status if seqno is not older and path repairs failure
		// or: seqno is not older and we learn a shorter path -> update anyways
		if (bool seqno_is_ok(contact->getPathSeqNo() <= pathseqno);
		    (contact->isInvalid() and seqno_is_ok)
		    or
		    (seqno_is_ok and (path.size() + viaPath.size()) < contact->getPathLength()) )
		{
			if (contact->isInvalid() and containsUndirectedLink(myKey, contact->getKey(), concatViaPath(viaPath, path), contact->getRDS().failedLink))
			{ // provided path does not repair broken link
				return false;
			}
			contact->setPath(concatViaPath(viaPath, path)); // will set contact automatically to valid
			contact->setPathSeqNo(pathseqno);

			contact->setPathProperty(pathPropHint);
			updateCount++;

			// do something on improved path according to policy
			policy->onImprovement(*contact);

			return true;
		}
		// else contact exists, but new path is not interesting
		//EV_TRACE << "Contact known, but new path not added " << *contact << endl;

		return false;
	}
	else // contact is new
	{ // concatenated paths are direct if within discovery radius
		return handleNewContact(key, concatViaPath(viaPath, path), pathseqno, pathPropHint);
	}
}


/// @returns true if path was simplified, false if not
bool KadRoutingTable::simplifyPath(kad_path& path) const
{
	if (path.size() < 2)
		return false;
	
	for (auto right = path.end()-1; right != path.begin(); right--) {
		auto left = std::find(path.begin(), right, *right);
		if (left != right) {
			ASSERT(*left == *right);
			path.erase(left, right);
			return true;
		}
	}
	return false;
}


/// uses own routing table to find shorter paths
/// @returns true if path was shortened, false if not
bool KadRoutingTable::shortenPath(kad_path& path) const
{
	if (path.size() <= 1) // this could only be optimized in case the contact is a physical neighbor anyway
		return false;

	unsigned int currentlength= path.size();
	std::list<OverlayKey> residualpath;
	KadContact_p knowncontactp= nullptr;
	// check for each pathelement from the far end whether the known path is shorter
	for (auto pathit = path.crbegin(); pathit != path.crend(); pathit++) {
		if ( (knowncontactp= getContact(*pathit)) != nullptr ) {
			// caveat: contact must be valid and the resulting path must be actually shorter
			if ( knowncontactp->isValid() and knowncontactp->getPathLength()+1 < currentlength )
			{
				// we know a shorter replacement path, use it
				path= knowncontactp->getPath();
				// contact itself is not included in path
				path.push_back(knowncontactp->getKey());
				// append residual path
				path.insert(path.end(), residualpath.begin(), residualpath.end());

				return true;
			}
		}
		residualpath.push_front(*pathit);
		currentlength--;
	}
	return false;
}


// @returns true if contact was inserted into routing table, false if not
bool KadRoutingTable::handleNewContact(const OverlayKey& key, kad_path path, pathseqno_t pathseqno, KadContact::PathProperty pathprop)
{
	ASSERT(key != myKey); // never insert myself
	// get responsible bucket for contact
	KadBucket& bucket = getBucket(key);
	ASSERT(!bucket.getContact(key)); // contact should be new
	// if contact would be inserted into the deepest bucket (new ID-wise neighbor):
	// expand routing table by splitting the bucket and increase the depth:
	// this will ensure that all ID-wise neighbors will be learned and kept
	if (isSplittable(bucket)) {
		splitLastBucket();
		// we recurse to get the new bucket after the split
		return handleNewContact(key, std::move(path), pathseqno, pathprop);
	}

	bool isPhysicalNeighbor(path.empty() == true);
	auto contact = std::make_unique<KadContact>(key, std::move(path));
	contact->setPathProperty(pathprop); // use hint from caller
	contact->setPathSeqNo(pathseqno);
	
	// physical neighbors get always inserted as contact
	if (isPhysicalNeighbor)
	{
		if (pathprop != KadContact::PathProperty::direct)
			EV_WARN << "handleNewContact(): inconsistent path property, should be direct, but is " << (uint8_t) pathprop << endl;
		contact->setPathProperty(KadContact::PathProperty::direct);
		policy->onNewContact(contact.get()); // this should be safe even when contact is not yet added to bucket
		bucket.PNContacts.push_back(std::move(contact));

		return true;
	}

	// from here on the contact is not a physical Neighbor:
	// add if space left or perform contact replacement
	if (bucket.contacts.size() < bucketSize)
	{ // still room in contacts left
		policy->onNewContact(contact.get());
		bucket.contacts.push_back(std::move(contact));

		return true;
	}
	else { // no space in bucket for inserting the new contact: replace existing one?
		if (not proximityNeighborSelection) {
			//EV_DEBUG << "dropping new contact " << *contact << endl;
			return false;
		}
		// contact replacement if contact is better than existing ones
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
			policy->onContactRemoval(result->get());
			*result = std::move(contact);
			policy->onNewContact(result->get());

			return true;
		}
		else {
			//EV_DEBUG << "dropping new contact " << *contact << endl;
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
	
	// sort PNcontacts into new buckets
	for (auto& contact : oldBucket.PNContacts)
		getBucket(contact.get()).PNContacts.push_back(std::move(contact));
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
 * returns at most n contacts closest to key
 * ContactList is sorted according to XOR metric with respect to key, i.e., the first returned contact
 * is the closest to key
 */

KadContactList KadRoutingTable::getClosestContacts(const OverlayKey& key, size_type n, bool validcontactsonly) const
{
	ASSERT(not key.isUnspecified()); // calling this with unspecified key is considered useless and an error
	KadContactList result{};
	result.reserve(n);

	std::function<void(const std::vector<std::unique_ptr<KadContact>>& source, KadContactList& target)> copyContactsFn =
		validcontactsonly ? copyValidContacts : copyContacts;

	size_type index = getBucketIndex(key);
	auto bucket = buckets.begin() + index;
	if (bucket->size() >= n)
	{
		copyContactsFn(bucket->contacts, result);
		copyContactsFn(bucket->PNContacts, result);
	}
	else {
		// not enough contacts in first bucket
		// now we scan the level of the accelerated lookup
		// and then we go down the tree
		size_type firstOnLevelIndex = getFirstOnBucketLevel(index);
		uint width = getLevelWidth();

		for (auto level = buckets.begin() + firstOnLevelIndex;
		     level < buckets.end() and result.size() < n;
		     level+=width) {
			if (level == buckets.end()-1)
			{
				copyContactsFn(level->contacts, result);
				copyContactsFn(level->PNContacts, result);
			}
			else // always insert the whole level
				for (uint i = 0; i < width and level+i < buckets.end(); i++)
				{
					copyContactsFn(level[i].contacts, result);
					copyContactsFn(level[i].PNContacts, result);
				}
		}
		// if we still do not have enough contacts, we go up the tree
		auto otherBucket = buckets.begin() + firstOnLevelIndex;
		while(result.size() < n and otherBucket != buckets.begin()) {
			otherBucket--;
			copyContactsFn(otherBucket->contacts, result);
			copyContactsFn(otherBucket->PNContacts, result);
		}
	}
	// now we sort the contacts by their XOR metric
	std::sort(result.begin(), result.end(), KadContactCompare(key));
	// if we have found more than n contacts, we need to remove some
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
	{
		copyContacts(bucket.contacts, result);
		copyContacts(bucket.PNContacts, result);
	}
	return result;
}


KadContactList KadRoutingTable::getAllValidContacts() const
{
	KadContactList result{};
	result.reserve(bucketSize * buckets.size());
	for (auto& bucket : buckets)
	{
		copyValidContacts(bucket.contacts, result);
		copyValidContacts(bucket.PNContacts, result);
	}

	return result;
}


size_type KadRoutingTable::getNumContacts() const
{
	size_type sum = 0;
	for (const KadBucket& b : buckets)
		sum += b.size();
	return sum;
}


size_type KadRoutingTable::getNumIndirectPaths() const
{
	size_type sum = 0;
	for (const KadBucket& b : buckets)
		for (const auto& contact : b.contacts) {
			if (contact->getPathProperty() == KadContact::PathProperty::indirect)
				sum++;
		}
	// no need to check for PNcontacts: they are always direct...
	return sum;
}


void KadRoutingTable::invalidateContact(KadContact& contact, const link_t& failed_link)
{
	// if we find a failed link in the path, mark contact invalid
	const auto& path= contact.getPath();
	if (path.size() > 0)
	{
		const auto &[failedNodeKey, otherNodeKey]= failed_link;
		if (contact.isContainedinPath(myKey, failed_link))
		{ // link somewhere in the path
			// TODO: handle the case when there is another failed link in an already failed path
			if (contact.isValid()) {

				contact.setInvalid(failed_link);

				EV_TRACE << "Set Contact to invalid due to failed link <"
					 << failedNodeKey.shortStr() << "," << otherNodeKey.shortStr() << ">:"  << contact << endl;
				// if contact is important neighbor: send rediscovery immediately
				// otherwise schedule normal rediscovery
				policy->scheduleRediscovery(contact.getKey(), failedNodeKey, otherNodeKey, isInLastBucket(contact.getKey()));
			}
		}
	}
}


/** invalidate all contacts with paths that contain a failed link otherNodeKey <-> failedNodeKey */
void KadRoutingTable::invalidateContacts(const link_t& failedlink)
{
	//EV_TRACE << "invalidateContacts(" << failedNodeKey << ")" << endl;
	// iterate over all buckets and contacts (even PNcontacts, because they may use redirected paths)
	for (KadBucket& b : buckets) {
		for (auto& contact : b.contacts) {
			invalidateContact(*contact, failedlink);
		} // end for inner loop over all contacts

		for (auto& contact : b.PNContacts) {
			invalidateContact(*contact, failedlink);
		} // end for inner loop over all PN contacts
	} // end for over all buckets
}


/** invalidate all contacts with paths that contain the physical neighbor failedNodeKey as next hop */
void KadRoutingTable::invalidateContactsPN(const OverlayKey& failedNodeKey)
{
	//EV_TRACE << "invalidateContactsPN(" << failedNodeKey << ")" << endl;
	// iterate over all buckets and contacts
	for (KadBucket& b : buckets) {
		for (auto& contact : b.contacts) {
			// we found a failed link in the path, mark contact invalid
			const auto& path= contact->getPath();
			if (path.size() >= 1)
			{
				if (path[0] == failedNodeKey) // failed node is first hop
				{
					contact->setInvalid({failedNodeKey, myKey});
					EV_TRACE << "Set Contact to invalid due to PN first hop failure: "  << *contact << endl;
					// will schedule this for rediscovery later
					policy->scheduleRediscovery(contact->getKey(), failedNodeKey, myKey, isInLastBucket(contact->getKey()));
				}
			}
		} // end for inner loop over all contacts

		for (auto& contact : b.PNContacts) {
			// we found a failed link in the path, mark contact invalid
			const auto& path= contact->getPath();
			if (path.size() >= 1)
			{
				if (path[0] == failedNodeKey) // failed node is first hop
				{
					contact->setInvalid({failedNodeKey, myKey});
					EV_TRACE << "Set Contact to invalid due to PN first hop failure: "  << *contact << endl;
					// will schedule this for rediscovery later
					policy->scheduleRediscovery(contact->getKey(), failedNodeKey, myKey, isInLastBucket(contact->getKey()));
				}
			}
		} // end for inner loop over all contacts
	} // end for over all buckets
}


double KadRoutingTable::getAveragePathLength() const
{
	uint count = 0;
	double sum = 0;
	for (const KadBucket& b : buckets) {
		for (auto& contact : b.contacts) {
			sum += contact->getPathLength();
			count++;
		}
		sum += b.PNContacts.size();
		count += b.PNContacts.size();
	}
	return count == 0 ? 0.0 : sum / count;
}


double KadRoutingTable::getAveragePathStretch() const
{
	uint count = 0;
	double sum = 0;
	for (const KadBucket& b : buckets) {
		for (auto& contact : b.contacts) {
			if (getKadObserver().getPhysicalDistance(myKey, contact->getKey()) > 0)
				sum += (contact->getPathLength()+1) / getKadObserver().getPhysicalDistance(myKey, contact->getKey());
			else
				EV_WARN << "Physical Distance = " << getKadObserver().getPhysicalDistance(myKey, contact->getKey())
					<< ": " << myKey << " to " << contact->getKey().shortStr() << endl;
			count++;
		}

		sum += b.PNContacts.size();
		count += b.PNContacts.size();
	}
	return count == 0 ? 0.0 : sum / count;
}


bool KadRoutingTable::removeContact(const OverlayKey& key)
{
	// function will be called *before* contact is actually removed
	policy->onContactRemoval(getContact(key));
	
	KadBucket& bucket = getBucket(key);
	bool result = bucket.removeContact(key);
	if (!result) {
		EV_ERROR << "failed to remove contact " << key << endl;
	}
	
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

/// @returns the contact either from ordinary contacts or from PN contacts
KadContact *KadBucket::getContact(const OverlayKey& key)
{
	auto it = findContact(key);
	if (it != contacts.end() )
		return it->get();
	
	it = findPNContact(key);
	if (it != PNContacts.end() )
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
	{ // not found in contacts yet, try PNContacts
		it = findPNContact(key);
		if (it != PNContacts.end()){
			PNContacts.erase(it);
			return true;
		}
	}
	// if we arrive here, nothing found
	return false;
}

// only iterates over non PN contacts, see findPNContact for other case
std::vector<std::unique_ptr<KadContact>>::iterator KadBucket::findContact(const OverlayKey& key)
{
	auto it = contacts.begin();
	for (; it !=  contacts.end() and !(*it)->hasKey(key); it++);

	return it;
}


std::vector<std::unique_ptr<KadContact>>::iterator KadBucket::findPNContact(const OverlayKey& key)
{
	auto it = PNContacts.begin();
	for (; it !=  PNContacts.end() and !(*it)->hasKey(key); it++);
	return it;
}


// ---------------- other class independent methods ----------------


} // namespace
