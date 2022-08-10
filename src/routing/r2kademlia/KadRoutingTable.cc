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
 * @author Jakob Thomas, Roland Bless
 */

#include "KadRoutingTable.h"
#include "KadObserver.h"
#include <algorithm>
#include "BaseTypes.h"

namespace routingsim {

namespace ph = std::placeholders;

constexpr unsigned int exploreMinRadius = 10;

void KadRoutingTable::init(const OverlayKey& myKey, KadPolicy* policy,
			   size_type accelerationFactor, size_type bucketSize,
			   bool proximityNeighborSelection,
			   bool enableSinglePathDiversity,
			   uint8_t discoveryRadius)
{
	ASSERT(this->myKey.isUnspecified());
	ASSERT(not myKey.isUnspecified());
	this->myKey = myKey;
	this->accelerationFactor = accelerationFactor;
	this->bucketSize = bucketSize;
	this->policy = policy;
	this->proximityNeighborSelection = proximityNeighborSelection;
	this->enableSinglePathDiversity = enableSinglePathDiversity;

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
bool KadRoutingTable::addPath(const OverlayKey& key, kad_path path, stateseqno_t stateseqno, age_t pathinfo_age)
{
	if (key == myKey) return false;

	// we must be careful: some neighbor may tell me paths that lead via me to my neighbors
	// we simply discard this information as it cannot be more current than mine

	const bool pathWasNotEmpty(path.size() > 0);
	// find my key -> this is a poisonous path via me, simply ignore this
	if (pathWasNotEmpty and std::find(path.begin(), path.end(), myKey) != path.end())
	{
		//EV_TRACE << "not adding poisonous path to " << key.shortStr() << " leading via me (" << myKey.shortStr() << "): ";
		//for (const auto& node : path)
		//	EV_TRACE << node.shortStr() << ' ';
		//EV_TRACE << endl;
		return false;
	}

	// find other cycles/duplicates (note: path may even be empty afterwards)
	simplifyPath(path, &key);

	// try to shorten the path using local routing table
	if (path.size() > 0)
		shortenPath(path);

	// sanity check: do not use paths that lead via an invalid neighbor as first hop
	if (path.size() > 0 and not neighbortable.isNeighbor(path[0]))
	{ // first hop must be found in neighbortable
		//EV_WARN << "Tried to use add path with invalid physical neighbor "
		//	<< path[0] << " as next hop - ignoring" << endl;
		return false;
	}

	KadContact *contact = getContactInternal(key);
	if (contact == nullptr) { // contact is new
		// if contact is not known yet but should be a PN, we wait until PNHello
		// might be the case if QueryRouteReq arrives before PNDiscReq or PNHello
		if (path.size() == 0 and not neighbortable.isNeighbor(key))
			return false;

		return handleNewContact(key, std::move(path), stateseqno, pathinfo_age);
	}

	// contact is known

	// there may be the rare case when we learned the new contact from someone else
	// before we heard the Hello
	if (!contact->isPhysicalNeighbor() and path.empty()==true)
	{
		contact->setPhysicalNeighbor();
		contact->updateLastSeen();

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
	// 1) when sequence number is higher (except when contact is valid and new path is longer, then we recheck current path)
	// 2) contact is invalid: only if we have newer information (higher sequence number)
	//                        or the same seq# (the path may have changed though and reenabling the contact is important)
	//                        however, path information must be more recent if same seq#
	// 3) we learn a shorter path (seq# must be same or newer) -> update anyways

	bool newer_seqno(contact->getStateSeqNo() < stateseqno); // contact has newer state for its physical neighbors
	bool same_seqno(contact->getStateSeqNo() == stateseqno); // same contact state
	bool shorter_path(path.size() < contact->getPathLength()); // new path is shorter than old path
	bool equivalent_path(path.size() == contact->getPathLength()); // path of same length (or distance metric)
	bool path_is_eligible(enableSinglePathDiversity ? shorter_path or equivalent_path : shorter_path);

	if ( not newer_seqno and not (same_seqno and (contact->isInvalid() or path_is_eligible)) ) {
		//  contact exists, but new path is not interesting
		return false;
	}

	// from here on the following condition holds:
	// newer_seqno or (same_seqno and (contact->isInvalid() or path_is_eligible))

	bool recheckPhysicalNeighbors(false);
	// special case for non updated sequence number
	if (same_seqno) {
		// check for age information if seq no is equal
		if (contact->getAge() < pathinfo_age) {
			EV_TRACE << "ignoring older contact info: mine " << contact->getAge() << " other " << pathinfo_age << endl;
			return false;
		}

		// in case the path had a failure, check whether the supposed new path actually avoids all broken links
		// note that this check will fail in case the path was repaired,
		// but then pathinfo_age must be newer
		if (contact->isInvalid())
		{
			for (const auto& failedlink : contact->getRDS().failedLinkList) {
				if (static_cast<unsigned long>((simTime() - failedlink.second).inUnit(SIMTIME_MS)) < pathinfo_age
				    and containsUndirectedLink(myKey, contact->getKey(), path, failedlink.first)) {
					// only if pathinfo_age is not newer and
					// provided new path does not repair broken link
					return false;
				}
			} // end for
		}
	}
	else // must be newer_seqno due to surrounding if
	{ // contact has new state?
		// sometimes updates may be received that carry a newer sequence number
		// from physical neighbors; this indicates that we do not have the latest state about its neighbors
		// if PN link is still up, we ignore this for now and initiate a resynch
		// we may have received a PNDiscReq from that PN, in this case no resync necessary (path is <-> then)
		if ( contact->isPhysicalNeighbor() and
		     neighbortable.isNeighbor(contact->getKey()) and
		     (contact->getPathLength() == 0) and
		     path.size() > 0
		   ) {
			// try to resynchronize with physical neighbor instead of blindly accepting a longer route now
			if (stateseqno > contact->getSyncPNSeqNo()) {
				EV_TRACE << "resync required for " << contact->getKey().shortStr() << " new: "
					 << stateseqno << " old: " << contact->getStateSeqNo() << endl;
				policy->reSyncPN(*contact, stateseqno);
				return false;
			}
		}

		// if contact is within discovery radius, ask again for its neighbors since something has changed
		if ( not contact->isPhysicalNeighbor() and (contact->getPathLength()+1 < nodeVicinityGraph::vicinityDiscoveryRadius or path.size()+1 < nodeVicinityGraph::vicinityDiscoveryRadius) ) {
			// we "defer" that until new path has been set (see below)
			// otherwise addNode() in sendGetPhysicalNeighbors will not update correctly
			recheckPhysicalNeighbors= true;
		}

		// if contact is valid and path is longer, we should recheck the current path
		// because it might be bad timing and we temporarily accept a worse choice due to
		// the higher sequence number. Therefore, we simply trigger a path recheck for
		// the current shorter path. The answer will overwrite the worse choice again
		if ( not contact->isInvalid() and not shorter_path and not equivalent_path ) {
			policy->onPathCheck(*contact);
			return false; // returning from here would reject the longer route
		}
	} // end else newer_seqno

	// for valid paths:
	// only update paths of equal length if HashSum of new path is closer to own ID
	// this should allow for better path diversity without route flapping
	bool pathimproved(newer_seqno or contact->isInvalid() or shorter_path);
	bool identical_path(false);
	OverlayKey pathHashSum;
	if (contact->isValid()) {
		if (path.size() == 0) {
			if (equivalent_path) {
				if (not newer_seqno) {
					// this is an identical path, ignore
					return false;
				}
				else { // just higher seq no, not necessary to send updates etc.
					pathimproved= false;
					identical_path= true;
				}
			}
			// in case of path improvement or prior contact invalidity or newer seq no
			// we need to update this contact
		}
		else // path.size() > 0
		{ // it is not a direct neighbor
			if (equivalent_path) {
				// this is the condition where we check for updating with a diverse path

				// the are_equal_r comparison operator starts at the end
				// if paths diverge, they will do this more likely at their end
				identical_path= are_equal_r(path, contact->getPath());
				// path is actually improved in case we
				// learn a shorter path (not here) or path was invalid before

				// if it is exactly the same path => not interesting,
				// except if sequence number was increased
				// (may be ok in case of recovered link, but path is valid now)
				if (identical_path and not newer_seqno)
					return false;
				// we have a different path of same length
				pathimproved= false;

				// check for eligibility of new path with same length
				// now check for XOR distance to own ID of Path Hash
				pathHashSum= HashSum(path, key);
				// pathSet.insert(pathHashSum);
				if ((pathHashSum ^ myKey) >= (contact->getPathHashSum() ^  myKey)) {
					EV_TRACE << "for contact " << key.shortStr() << " discarding path of same length: " << path << endl;
					return false;
				}
				else {
					EV_TRACE << "for contact " << key.shortStr()
						 << " replacing path of same length with: " << path << endl;
				}
			} // end if equivalent_path (otherwise path is shorter)
		} // end else if path.size() > 0
	} // endif contact is valid

	EV_TRACE << "for contact " << key.shortStr() << " seq#" << contact->getStateSeqNo()
		 << " updating old path " << contact->getPath() << " with new " << path
		 << " seq#"  << stateseqno << " identical: " << (identical_path ? 'T' : 'F') << endl;

	if (not identical_path) { // avoid to overwrite identical paths
		// valid and identical paths were checked above already

		// must calculate PathHashSum unless this has been done already
		if (pathHashSum.isUnspecified())
			pathHashSum= HashSum(path, key);

		contact->setPath(std::move(path), std::move(pathHashSum)); // will set contact automatically to valid
	}
	else { // new path is identical with old path
		// need to set this to valid state again
		contact->setValid();
	}

	// update sequence number
	contact->setStateSeqNo(stateseqno);
	// update lastseen to that of the originator
	contact->updateLastSeenTo(simTime()-SimTime(pathinfo_age,SIMTIME_MS));

	updateCount++;

	// recheck neighbors of that contact because something has changed
	if (recheckPhysicalNeighbors)
		policy->recheckPhysicalNeighbors(*contact);

	// do something on improved path according to policy
	// (only if path has been improved actually)
	if (pathimproved) {
		policy->onImprovement(*contact);

		// try to improve other paths with newly improved contact path
		updatePaths(*contact);
	}
	else {
		policy->onPathChange(*contact);
	}

	return true;

}


/// @returns true in case path was added

bool KadRoutingTable::addPath(const OverlayKey& key, const kad_path& path, stateseqno_t stateseqno, age_t pathinfo_age, const kad_path& viaPath)
{
	if (key == myKey)
		return false;

	// concatenated paths are direct if learned from neighborhood within discovery radius
	return addPath(key, concatViaPath(viaPath, path), stateseqno, pathinfo_age);
}


/// @returns true if path was simplified, false if not
bool KadRoutingTable::simplifyPath(kad_path& path, const OverlayKey* destination) const
{
	if (path.size() < 2) // nothing to optimize here
		return false;

	if (destination != nullptr)
		path.push_back(*destination);

	bool modified(false);
	for (auto right = path.end()-1; right != path.begin(); right--) {
		// find element starting from the left
		auto left = std::find(path.begin(), right, *right);
		if (left != right) { // element found, erase sequence
			// erase returns new location of the element that followed
			// the last element erased by the function call
			right= path.erase(left, right);
			modified= true;
			// special case if only one element left
			if (right == path.begin()) {
				// remove destination again
				if (destination != nullptr) {
					path.pop_back();
				}
				return modified;
			}
		}
	}

	// remove destination again
	if (destination != nullptr) {
		path.pop_back();
	}
	return modified;
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
bool KadRoutingTable::handleNewContact(const OverlayKey& key, kad_path path, stateseqno_t stateseqno, age_t pathinfo_age)
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
		return handleNewContact(key, std::move(path), stateseqno, pathinfo_age);
	}

	bool isPhysicalNeighbor(path.empty() == true);
	auto contact = std::make_unique<KadContact>(key, std::move(path));
	contact->setStateSeqNo(stateseqno);
	// TODO should be corrected by RTT
	contact->updateLastSeenTo(simTime()-SimTime(pathinfo_age,SIMTIME_MS));

	// physical neighbors get always inserted as contact
	if (isPhysicalNeighbor)
	{
		contact->updateLastSeen();
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
			//EV_DEBUG << "bucket full - dropping new contact " << *contact << endl;
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


/** copies all selected contacts from one contact list to the end of the other contact list */
inline void copySelectedContacts(const std::vector<std::unique_ptr<KadContact>>& source, KadContactList& target,
				 std::function<bool(const KadContact& contact)> selectorFn) {
	target.reserve(target.size() + source.size());
	for (auto it = source.begin(); it != source.end(); it++) {
		if ( it->get() and selectorFn(*(it->get())) )
			target.push_back(it->get());
	}
}

// simply selects all contacts (valid and invalid) to copy
auto copyAllContacts = std::bind( &copySelectedContacts,ph::_1,ph::_2, [](const KadContact& contact) -> bool { return true; } );
// selects all valid contacts to copy
auto copyValidContacts = std::bind( &copySelectedContacts,ph::_1,ph::_2, [](const KadContact& contact) -> bool { return contact.isValid(); } );


/** 
 * returns at most n contacts closest to key
 * ContactList is sorted according to XOR metric with respect to key, i.e.,
 * the first returned contact is the closest to key.
 * if the notvialist is not empty, it will be considered for validcontactsonly==true
 */
KadContactList KadRoutingTable::getClosestContacts(const OverlayKey& key, size_type n, bool validcontactsonly, const std::list<std::pair<link_t,age_t>>& notvialist) const
{
	ASSERT(not key.isUnspecified()); // calling this with unspecified key is considered useless and an error
	KadContactList result{};
	result.reserve(n);

	// the following function excludes contacts that are affected by a notvia link list
	auto selectorValidNotViaSafeFn = [&](const KadContact& contact) -> bool {
					     if (contact.isValid()) {
						     for ( const auto& failedLink : notvialist) {
							     if (contact.getAge() > failedLink.second and
								 containsUndirectedLink(myKey,contact.getKey(),contact.getPath(),failedLink.first))
							     {
								     EV_TRACE << "Contact " << contact << " matches not via " << failedLink.first << endl;
								     return false;
							     }
						     };
						     return true;
					     }
					     return false;
				     };

	// selects all valid and "not via safe" contacts, i.e., contacts are excluded if occuring in the notvialist
	auto copyValidNotViaSafeContacts = std::bind( &copySelectedContacts,ph::_1,ph::_2, selectorValidNotViaSafeFn );

	std::function<void(const std::vector<std::unique_ptr<KadContact>>& source, KadContactList& target)> copyContactsFn(copyAllContacts);
	if (validcontactsonly) {
		if ( notvialist.empty() )
			copyContactsFn = copyValidContacts;
		else
			copyContactsFn = copyValidNotViaSafeContacts;
	}

	// that's the bucket where we start
	size_type index = getBucketIndex(key);
	auto startbucket = buckets.cbegin() + index;

	//EV_TRACE << "getClosestContacts() bucket index: " << index << endl;

	// if bucket is large enough, try to copy all contacts
	// however, size() does not consider contact validity
	// so we need to check after this for result.size()
	if (startbucket->size() >= n)
	{
		copyContactsFn(startbucket->contacts, result);
		copyContactsFn(startbucket->PNContacts, result);
	}
	else
	{ // this bucket has not been considered, so do not skip in next routine
		startbucket= buckets.cend();
	}

	// not enough contacts yet, try to gather more, but do not add those from startbucket again
	if (result.size() < n) {
		// not enough (valid) contacts in first bucket
		// now we scan the level of the accelerated lookup
		// and then we go down the tree
		size_type firstOnLevelIndex = getFirstOnBucketLevel(index);
		uint width = getLevelWidth();

		//EV_TRACE << "getClosestContacts() first on bucket level: " << firstOnLevelIndex << " width: " << width << endl;

		for (auto level = buckets.cbegin() + firstOnLevelIndex;
		     level < buckets.cend() and result.size() < n;
		     level+=width)
		{
			//EV_TRACE << "getClosestContacts() level index: " << buckets.cend() - level << endl;
			// if it is the very last bucket try to copy content
			if (level == buckets.cend()-1 and level != startbucket) {
				copyContactsFn(level->contacts, result);
				copyContactsFn(level->PNContacts, result);
			}
			else { // always insert the whole level
				for (uint i = 0; i < width and level+i < buckets.cend(); i++)
				{
					if (level+i != startbucket) {
						copyContactsFn(level[i].contacts, result);
						copyContactsFn(level[i].PNContacts, result);
					}
				}
			}
		}

		// if we still do not have enough contacts, we go up the tree
		auto otherBucket = buckets.cbegin() + firstOnLevelIndex;
		while (result.size() < n and otherBucket != buckets.cbegin()) {
			otherBucket--;
			if (otherBucket != startbucket) {
				copyContactsFn(otherBucket->contacts, result);
				copyContactsFn(otherBucket->PNContacts, result);
			}
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
		copyAllContacts(bucket.contacts, result);
		copyAllContacts(bucket.PNContacts, result);
	}
	return result;
}


KadContactList KadRoutingTable::getAllValidContacts(bool noPN) const
{
	KadContactList result{};
	result.reserve(bucketSize * buckets.size());
	for (auto& bucket : buckets)
	{
		copyValidContacts(bucket.contacts, result);
		if (noPN == false)
			copyValidContacts(bucket.PNContacts, result);
	}

	return result;
}


KadContact_p KadRoutingTable::getRandomContact() const
{
	size_type numContacts(getNumContactsWithoutPNs());

	constexpr unsigned int max_retries= 3U;
	unsigned int randomContactIndex(intuniform(getEnvir()->getRNG(0), 0, numContacts > 0 ? numContacts-1 : 0));
	EV_TRACE << "getRandomContact() random contact index = " << randomContactIndex << endl;
	unsigned int index(0);
	for (unsigned int retry= 0; retry < max_retries; retry++)
	{
		for (const auto& bucket : buckets)
		{
			if ((index + bucket.sizeWithoutPNContacts()) > randomContactIndex) { // the contact must be in the current bucket
				auto contact_p= bucket.contacts[randomContactIndex-index].get();
				if (contact_p != nullptr and contact_p->isValid()) {
					return contact_p;
				}
			}
			else {
				index += bucket.sizeWithoutPNContacts();
			}
		} // end for
	}

	return nullptr;
}


// returns the two oldest entries from each bucket, deepest contacts first and oldest before newer (per bucket)
// only non PN contacts will be returned
// However, contacts that have recent information <1s will not be selected
void KadRoutingTable::getOldestValidContacts(std::list<OverlayKey>& result) const
{
	// do not consider contacts younger than R2Kademlia::PathInfoAgeThreshold

	for (auto& bucket : buckets)
	{
		KadContact_p oldest(nullptr);
		KadContact_p secondoldest(nullptr);
		for (auto it = bucket.contacts.begin(); it != bucket.contacts.end(); it++) {
			if ( *it != nullptr and it->get()!= nullptr and it->get()->getAge() > R2Kademlia::PathInfoAgeThreshold ) {
				if (oldest != nullptr) {
					// older entry? then replace oldest
					if (oldest->getAge() < it->get()->getAge()) {
						secondoldest= oldest; oldest= it->get();
					}
					else if (secondoldest != nullptr and secondoldest->getAge() < it->get()->getAge()) {
						// replace second oldest
						secondoldest= it->get();
					}
				}
				else { // initialize
					oldest= it->get();
				}
			}
		}

		if (oldest != nullptr) {
			if (secondoldest!= nullptr) {
				result.push_front(secondoldest->getKey());
				//EV_TRACE << "Contacts to refresh: "
				//	 << oldest->getKey().shortStr() << " (age: " << oldest->getAge() << ") "
				//	 << secondoldest->getKey().shortStr() << " (age: " << secondoldest->getAge() << ")" << endl;
			}
			//else {
				//EV_TRACE << "Contact to refresh: "
				//	 << oldest->getKey().shortStr() << " (age: " << oldest->getAge() << ")" << endl;
			//}
			// push older entry first
			result.push_front(oldest->getKey());
		}
	} // end for iterate over all buckets
}


size_type KadRoutingTable::getNumContacts() const
{
	size_type sum = 0;
	for (const KadBucket& b : buckets)
		sum += b.size();
	return sum;
}


size_type KadRoutingTable::getNumContactsWithoutPNs() const
{
	size_type sum = 0;
	for (const KadBucket& b : buckets)
		sum += b.sizeWithoutPNContacts();
	return sum;
}


void KadRoutingTable::updatePaths(const KadContact& improvedContact)
{
	// iterate over all buckets and contacts (not PNcontacts, because this does not make any sense)
	for (KadBucket& b : buckets) {
		for (auto& contact : b.contacts) {
			// check for existence of improvedContact in path and replace path
			// (either shorter or more current)
			if (auto& path= contact->getPath(); path.size() > 0 and contact->isValid()) {
				bool found(false);
				for (auto revpathit= path.rbegin(); revpathit!= path.rend() and !found; revpathit++) {
					if (*revpathit == improvedContact.getKey()) {
						// special case when found contact is the neighbor and nothing changed
						if (path.size() == 1 and improvedContact.getPath().empty()) {
								continue;
						}
						// construct new path
						kad_path newpath(improvedContact.getPath());
						// +1 because improvedContact is missing in getPathLength
						newpath.reserve(improvedContact.getPathLength()+1+(revpathit-path.rbegin()));
						newpath.push_back(improvedContact.getKey());
						for (auto fwdpathit= path.begin() + (path.rend() - revpathit); fwdpathit != path.end(); fwdpathit++) {
							newpath.push_back(*fwdpathit);
						}
						// set new path (must be more current than old one)
						// note: this will set the contact to be valid
						contact->updatePath( myKey, newpath );
						// call onImprovement(contact) also for updating PID
						policy->onImprovement(*contact);
						found=true; // break the loop
					}
				} // end for: iterating backwards over path
			}
		} // end for inner loop over all contacts
	} // end for over all buckets
}


void KadRoutingTable::invalidateContact(KadContact& contact, const link_t& failedLink, age_t failedLinkAge)
{
	// if we find a failed link in the path, contact should schedule a rediscovery
	const auto& path= contact.getPath();
	if (path.size() > 0)
	{
		if (contact.getAge() > failedLinkAge and contact.isContainedinPath(myKey, failedLink))
		{ // link somewhere in the path

			if (contact.isValid()) {

				if (contact.setInvalid(failedLink, failedLinkAge)) {
					policy->onContactInvalidation(&contact);
				}
				contact.updateLastSeen();

				EV_TRACE << "Set Contact to invalid due to failed link " << failedLink
					 << " (age: " << failedLinkAge << ") " << contact << endl;
				// if contact is important neighbor: send rediscovery immediately
				// otherwise schedule slow rediscovery
				if (isInLastBucket(contact.getKey()))
					policy->scheduleUrgentRediscovery(contact.getKey());
				else
					policy->scheduleSlowRediscovery(contact.getKey());
			}
			else
			if (contact.isInvalid()) { // contact already invalid or rediscovering
				// add potentially further failed link
				contact.setInvalid(failedLink, failedLinkAge);
				contact.updateLastSeen();
			}
		}
	}
}


void KadRoutingTable::checkPathToContact(KadContact& contact, const link_t& failedLink)
{
	// if we find a failed link in the path, send a ProbeReq to check for path validity
	// if the path has actually failed, we'll get back a segment failure and rediscovery will start
	const auto& path= contact.getPath();
	if (path.size() > 0)
	{
		if (contact.isContainedinPath(myKey, failedLink))
		{ // link somewhere in the path

			if (contact.isValid()) {
				EV_TRACE << "Will probe path to " << contact << " due to potentially failed link " << failedLink << ":" << endl;
				policy->onPathCheck(contact);
			}
		}
	}
}


/** check all contacts with paths that contain a failed link **/
void KadRoutingTable::checkContacts(const link_t& failedlink)
{
	// iterate over all buckets and contacts (even PNcontacts, because they may use redirected paths)
	for (KadBucket& b : buckets) {
		for (auto& contact : b.contacts) {
			if (contact->isValid())
				checkPathToContact(*contact, failedlink);
		} // end for inner loop over all contacts

		for (auto& contact : b.PNContacts) {
			if (contact->isValid())
				checkPathToContact(*contact, failedlink);
		} // end for inner loop over all PN contacts
	} // end for over all buckets
}


/** invalidate all contacts with paths that contain a failed link otherNodeKey <-> failedNodeKey */
void KadRoutingTable::invalidateContacts(const link_t& failedLink, age_t failedLinkAge)
{
	//EV_TRACE << "invalidateContacts(" << failedNodeKey << ")" << endl;
	// iterate over all buckets and contacts (even PNcontacts, because they may use redirected paths)
	for (KadBucket& b : buckets) {
		for (auto& contact : b.contacts) {
			invalidateContact(*contact, failedLink, failedLinkAge);
		} // end for inner loop over all contacts

		for (auto& contact : b.PNContacts) {
			invalidateContact(*contact, failedLink, failedLinkAge);
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
			const auto& path= contact->getPath();
			if (path.size() >= 1)
			{
				// if we find a failed link in the path, mark contact invalid
				// here it must be the first hop that failed
				if (path[0] == failedNodeKey) // failed node is first hop
				{
					if (contact->isValid()) {
						contact->setInvalid({failedNodeKey, myKey},0);
						EV_TRACE << "Set Contact to invalid due to PN first hop failure: "  << *contact << endl;
						// will schedule this for regular rediscovery later
						if (isInLastBucket(contact->getKey()))
							policy->scheduleUrgentRediscovery(contact->getKey());
						else
							policy->scheduleRegularRediscovery(contact->getKey());
					}
					else
					if (contact->isInvalid()) { // contact already invalid or rediscovering
						// add potentially further failed link
						contact->setInvalid({failedNodeKey, myKey},0);
					}
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
					if (contact->isValid()) {
						contact->setInvalid({failedNodeKey, myKey},0);
						EV_TRACE << "Set Contact to invalid due to PN first hop failure: "  << *contact << endl;
						// will schedule this for rediscovery later
						if (isInLastBucket(contact->getKey()))
							policy->scheduleUrgentRediscovery(contact->getKey());
						else
							policy->scheduleRegularRediscovery(contact->getKey());
					}
					else
					if (contact->isInvalid()) { // contact already invalid or rediscovering
						// add potentially further failed link
						contact->setInvalid({failedNodeKey, myKey}, 0);
					}
				}
			}
		} // end for inner loop over all contacts
	} // end for over all buckets
}


// Note that this Path Length reflects the number hops/link
// whereas contact.getPathLength() excludes the destination node
double KadRoutingTable::getAveragePathLength() const
{
	uint count = 0;
	double sum = 0;
	for (const KadBucket& b : buckets) {
		for (auto& contact : b.contacts) {
			// stored paths exclude the destination node
			sum += contact->getPathLength()+1;
			count++;
		}
		// need to also iterate over each PNContact:
		// it could be the case that we currently use a detour due to link failure
		for (auto& contact : b.PNContacts) {
			// stored paths exclude the destination node
			sum += contact->getPathLength()+1;
			count++;
		}
	}
	return count == 0 ? 0.0 : sum / count;
}


double KadRoutingTable::getAveragePathStretch() const
{
	uint count = 0;
	double sum = 0;
	for (const KadBucket& b : buckets) {
		for (auto& contact : b.contacts) {
			if (uint phy_distance(getKadObserver().getPhysicalDistance(myKey, contact->getKey())); phy_distance > 0) {
				if (getNetworkManager()->getFailedNodes().size() > 0) {
					// in case of failed links or nodes: check path validity of contact path
				}

				sum += (contact->getPathLength()+1) / phy_distance;
				count++;
			}
			else
				EV_WARN << "Physical Distance = " << phy_distance
					<< ": " << myKey << " to " << contact->getKey().shortStr() << endl;
		}

		for (auto& contact : b.PNContacts) {
			// stored paths exclude the destination node
			if (uint phy_distance(getKadObserver().getPhysicalDistance(myKey, contact->getKey())); phy_distance > 0) {
				if (getNetworkManager()->getFailedNodes().size() > 0) {
					// in case of failed links or nodes: check path validity of contact path
				}

				sum += (contact->getPathLength()+1) / phy_distance;
				count++;
			}
			else
				EV_WARN << "Physical Distance = " << phy_distance
					<< ": " << myKey << " to " << contact->getKey().shortStr() << endl;
		}
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
	auto left = viaPath.cend();
	auto right = path.crbegin();
	for (;right != path.crend() and left == viaPath.cend(); right++) {
		left = std::find(viaPath.cbegin(), viaPath.cend(), *right);
	}
	newPath.reserve((left - viaPath.cbegin()) + (path.cend() - right.base()));
	std::copy(viaPath.cbegin(), left, std::back_inserter(newPath));
	std::copy(right.base(), path.cend(), std::back_inserter(newPath));

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
