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

#ifndef __ROUTINGSIM_KadRoutingTable_H_
#define __ROUTINGSIM_KadRoutingTable_H_

#include <omnetpp.h>
#include <memory>
#include <vector>
#include <cmath>
#include <algorithm>
#include <utility>
//#include <unordered_set> // only required for pathSet

#include "KadContact.h"
/**
 * @file KadRoutingTable.h
 * @author Jakob Thomas
 */

namespace routingsim {
class KadRoutingTable;
class NeighborTable;

typedef uint32_t age_t;

// this class implements a single R²/Kad Bucket that distinguishes between normal contacts and physical neighbor contacts
class KadBucket
{
public:
	using BucketElement = std::unique_ptr<KadContact>;
	using BucketType = std::vector<BucketElement>;

	KadContact_p getContact(const OverlayKey& key) const;
	inline void insertContact(BucketType& bucket, BucketElement contactp);
	void insertContact(BucketElement contactp) { insertContact(contacts, std::move(contactp)); } ;
	void insertPNContact(BucketElement contactp) { insertContact(PNContacts,std::move(contactp)); } ;
	const BucketType& getContacts() const { return contacts; }
	BucketType& getContactsWrite() { return contacts; }
	const BucketType& getPNContacts() const { return PNContacts; }
	BucketType& getPNContactsWrite() { return PNContacts; }
	/** tries to remove the contact, returns true if found */
	inline bool removeContact(const OverlayKey& key);
	inline auto removeContact(BucketType::iterator it) { return contacts.erase(it); }
	inline auto removePNContact(BucketType::iterator it) { return PNContacts.erase(it); }
	/** returns true if the contact list is empty */
	inline bool empty() { return contacts.empty() and PNContacts.empty(); }
	/** returns the size of the contact list including PNcontacts */
	inline size_type size() const { return contacts.size() + PNContacts.size(); }
	/** returns the size of the contact list without PNcontacts */
	inline size_type sizeWithoutPNContacts() const { return contacts.size(); }
	/// Common stdc++ console output method
	friend std::ostream& operator<<(std::ostream& os, const KadBucket& b) {
		return os << b.contacts.size()+ b.PNContacts.size() << " contacts (PN: " << b.PNContacts.size() << ")";
	}
	/**
	 * Returns an iterator at the contact's position.
	 * Returns contacts->end() if the contact is not found.
	 */
	BucketType::iterator findContactWrite(const OverlayKey& key) { auto res= std::lower_bound(contacts.begin(),contacts.end(),key,[](const BucketElement& lhs, const OverlayKey& key) -> bool { return lhs.get()->getKey() < key; }); if (res != contacts.end()) { return (res->get()->getKey() != key) ? contacts.end() : res; } else { return res; } }
	BucketType::iterator findPNContactWrite(const OverlayKey& key) { auto res= std::lower_bound(PNContacts.begin(),PNContacts.end(),key,[](const BucketElement& lhs, const OverlayKey& key) -> bool { return lhs.get()->getKey() < key; }); if (res != PNContacts.end()) { return (res->get()->getKey() != key) ? PNContacts.end() : res; } else { return res; } }

	BucketType::const_iterator findContact(const OverlayKey& key) const { auto res= std::lower_bound(contacts.cbegin(),contacts.cend(),key,[](const BucketElement& lhs, const OverlayKey& key) -> bool { return lhs.get()->getKey() < key; }); if (res != contacts.cend()) { return (res->get()->getKey() != key) ? contacts.cend() : res; } else { return res; } }
	BucketType::const_iterator findPNContact(const OverlayKey& key) const { auto res= std::lower_bound(PNContacts.cbegin(),PNContacts.cend(),key,[](const BucketElement& lhs, const OverlayKey& key) -> bool { return lhs.get()->getKey() < key; }); if (res != PNContacts.cend()) { return (res->get()->getKey() != key) ? PNContacts.cend() : res; } else { return res; } }

private:

	// regular Kademlia bucket
	BucketType contacts;
	// extends the bucket by _all_ physical neighbors,
	// but do not fit into the regular bucket anymore
	// this is not strictly necessary, but saves overlay hops
	BucketType PNContacts;
};


/// inserts a KadContact into a bucket (kept sorted order)
inline void KadBucket::insertContact(BucketType& bucket, BucketElement contactp) {
	//ASSERT(is_sorted(bucket.cbegin(),bucket.cend(),[](const BucketElement& lhs, const BucketElement& rhs) -> bool { return lhs.get()->getKey() < rhs.get()->getKey(); }));

	// Note that this performs a binary search, but the insertion itself may cause O(n) move operations inside the vector
	bucket.insert(std::upper_bound(bucket.begin(), bucket.end(), contactp,
		      [](const BucketElement& lhs, const BucketElement& rhs) -> bool { return lhs->getKey() < rhs->getKey(); }), std::move(contactp));

	//ASSERT(is_sorted(bucket.cbegin(),bucket.cend(),[](const BucketElement& lhs, const BucketElement& rhs) -> bool { return lhs.get()->getKey() < rhs.get()->getKey(); }));
}


/**
 * A class with callback functions that are called when the routing table
 * encounters things.
 */
class KadPolicy {
public:
	/** called when last contact is removed from a bucket */
	virtual void onBucketEmpty(KadBucket& bucket) = 0;
	/** called when a new contact is created in the routing table */
	virtual void onNewContact(KadContact_p contact) = 0;
	/** called when path has changed, but is not improved **/
	virtual void onPathChange(const KadContact& contact) = 0;
	/** called when a better path has been learned for a contact */
	virtual void onImprovement(const KadContact& contact) = 0;
	/** called when a contact is about to being removed */
	virtual void onContactRemoval(KadContact_p contact) = 0;
	/** called when a contact is invalidated */
	virtual void onContactInvalidation(KadContact_p contact) = 0;
	/** called when physical neighbors may be out of synch with their sequence numbers **/
	virtual void reSyncPN(const KadContact& contact, stateseqno_t expected_seqno) = 0;
	/** called when physical neighbors in discoveryradius may be out of synch with their sequence numbers **/
	virtual void recheckPhysicalNeighbors(const KadContact& contact) = 0;
	/** called when a rediscovery needs to be performed for a contact */
	virtual void schedulePNRediscovery(const OverlayKey& contact) = 0;
	virtual void scheduleUrgentRediscovery(const OverlayKey& contact) = 0;
	virtual void scheduleRegularRediscovery(const OverlayKey& contact) = 0;
	virtual void scheduleSlowRediscovery(const OverlayKey& contact) = 0;
	/** called when the path of a contact is probably broken and needs to be checked */
	virtual void onPathCheck(const KadContact& contact) = 0;
};

/**
 * Models a Kademlia routing table that stores contacts in buckets depending
 * on the overlay key of the node.
 */
class KadRoutingTable : public cObject
{
	friend KadBucket;
public:
 KadRoutingTable(const NeighborTable& ro_neighbortable) : neighbortable(ro_neighbortable) {};
	void init(const OverlayKey& myKey,
		  KadPolicy* policy,
		  size_type accelerationFactor, size_type bucketSize,
		  bool proximityNeighborSelection,
		  bool enableSinglePathDiversity,
		  uint8_t discoveryRadius= 1);

	void clear() { buckets.clear(); myKey= OverlayKey::UNSPECIFIED_KEY; };
	
	// from cObject()
	virtual std::string str() const;

	/**
	 * add a potentially new path, adds a contacts or updates an existing one
	 * @returns true if path was added
	 */
	bool addPath(const OverlayKey& key, kad_path path, stateseqno_t pathseqno, age_t pathinfo_age, uint32_t nodeDegree);
	/**
	 * add a potentially new path, adds a contact or updates an existing one
	 * @returns true if path was added
	 */
	bool addPath(const OverlayKey& key, const kad_path& path, stateseqno_t pathseqno, age_t pathinfo_age, uint32_t nodeDegree, const kad_path& via);

	/**
	 * add a physical neighbor as contact or updates an existing one
	 */
	void addPNContact(const OverlayKey& key, stateseqno_t stateseqno, uint32_t nodeDegree);

	/**
	 * Removes the contact with the given key from the routing table.
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
	inline KadContact_p getContact(const OverlayKey& key) const;
	inline KadContact* getContactMutable(const OverlayKey& key);

	// set contact invalid, returns true on success, false if contact not found
	bool setContactInvalid(const OverlayKey& key) {
		if (auto contactp= getContactInternal(key))
		{
			contactp->setInvalid({myKey, contactp->getKey()},0);
			return true;
		}
		else
			return false;
	}

	/**
	 * Returns a list of n (valid) contacts that are closest to the given key if possible.
	 * If there are not enough contacts found, the list is shorter.
	 */
	KadContactList getClosestContacts(const OverlayKey& key, size_type n, bool validcontactsonly, const std::list<std::pair<link_t,age_t>>& notvialist= {}) const;
	/// returns a list of all known contacts
	KadContactList getAllContacts() const;
	/// returns a list of all known valid contacts (if noPN true, PNs are not returned)
	KadContactList getAllValidContacts(bool noPN) const;
	/// returns a list of two oldest valid contacts per bucket
	void getOldestValidContacts(std::list<OverlayKey>& result) const;
	/// returns a list of contacts that need a path refresh
	void getContactsforPathRefresh(std::list<OverlayKey>& result) const;
	/// get list of failed PNs
	std::list<OverlayKey> getFailedPNs() const;

	/// returns the number of contacts in buckets
	size_type getNumContacts() const;
	/// returns the number of contacts in buckets without physical neighbors
	size_type getNumContactsWithoutPNs() const;

	/// returns the number of found equivalent paths
	///size_type getNumFoundEquivPaths() const { return pathSet.size(); };

	/// returns a pointer random valid contact
	KadContact_p getRandomContact() const;

	/// returns path if failedLink is contained in path to contact (for debugging)
	kad_path checkPathValidityToContact(KadContact& contact, const link_t& failedLink) const;
	/// diagnostic output of paths with failedlink
	void getBrokenPaths(const link_t& failedlink) const;

	/// if failed link is in contact's path send a ProbeReq to check for path validity
	void checkPathToContact(KadContact& contact, const link_t& failed_link);
	/// do not invalidate valid contacts, but test path validity
	void checkContacts(const link_t& failedlink);

	/// invalidate all contacts with paths that contain a failed link otherNodeKey <-> failedNodeKey
	void invalidateContacts(const link_t& failedLink, age_t failedLinkAge);
	/// invalidate all contacts with paths that contain the physical neighbor failedNodeKey as next hop, returns number of affected contacts
	unsigned short invalidateContactsPN(const OverlayKey& failedNodeKey);
	/// invalidate individual contact if its path contains the failed_link
	void invalidateContact(KadContact& contact, const link_t& failed_link, age_t failedLinkAge);

	/// returns the number of buckets currently held
	inline size_type getNumBuckets() const { return buckets.size(); }
	/// the position of the bit that defines the lowest bucket
	inline size_type getDepth() const { return std::min<size_type>(ceil(getNumBuckets() / getLevelWidth()) * accelerationFactor, myKey.getLength()); }
	/// calculates the average path length of the known contacts (w/o physical contacts!)
	double getAveragePathLength() const;
	/// calculates the average stretch of the known contacts (w/o physical contacts!)
	double getAveragePathStretch() const;
	/// returns how many times paths have been updated
	inline uint getUpdateCount() const { return updateCount; }
	/// returns how many times contacts have been replaced
	inline uint getReplacementCount() const { return replacementCount; }
	/// returns the maximum size of a bucket, i.e. the size after which a bucket is split
	inline size_type getSplitSize() const { return bucketSize; }
	/// returns true if the key is known, i.e. there is a contact present
	inline bool isKnown(const OverlayKey& key) const { return getContact(key) != nullptr; }
	/// returns true if key is in address range of last bucket (it does not have to be present there)
	inline bool isInLastBucket(const OverlayKey& key) const { return getBucketIndex(key) == buckets.size()-1; }
	/// returns true if key is in last bucket and not present
	inline bool isInLastBucketAndNew(const OverlayKey& key) const { size_t index(getBucketIndex(key));
		return  index == buckets.size()-1 and buckets.at(index).getContact(key) == nullptr; }

	/// returns the distance between the two keys
	inline OverlayKey distance(const OverlayKey& a, const OverlayKey& b) const { return a ^ b; }
	/// returns the distance of the given key to my key
	inline OverlayKey distance(const OverlayKey& key) const { return key ^ myKey; }

	// update all contact paths that are affected by improved contact path
	void updatePaths(const KadContact& improvedContact);
	/// tries to find shortcuts using own routing table entries
	bool shortenPath(kad_path& path) const;
	/// concatenates the two paths
	kad_path concatViaPath(const kad_path& viaPath, const kad_path& path) const;
	/// removes duplicates and loops from a path
	bool simplifyPath(kad_path& path, const OverlayKey* destination) const;
        inline size_t getBucketCount() const { return buckets.size(); }
        KadContactList getValidContactsFromBucket(size_t index) const;

	/** returns the width of a level in the accelerated lookup tree */
	inline size_type getAccelerationFactor() const { return accelerationFactor; }

protected:
	/// adds a new contact, called by addPath, returns true if contact was added
	virtual bool handleNewContact(const OverlayKey& key, kad_path path, stateseqno_t pathseqno, age_t pathinfo_age, uint32_t nodeDegree);
	/** @name bucket access */
	//@{
	/** returns the index of the bucket corresponding to the given overlay key. */
	size_type getBucketIndex(const OverlayKey& key) const;
	/** splits the deepest bucket */
	void splitLastBucket();
	/** returns the index of the bucket corresponding to the given contact */
	inline size_type getBucketIndex(KadContact_p contact) const { return getBucketIndex(contact->getKey());}

	/** returns the bucket corresponding to the given overlay key */
	inline KadBucket& getBucket(const OverlayKey& key) { return buckets.at(getBucketIndex(key)); }
	inline const KadBucket& getBucket(const OverlayKey& key) const { return buckets.at(getBucketIndex(key)); }
	/** returns the bucket corresponding to the given contact */
	inline KadBucket& getBucket(KadContact_p contact) { return getBucket(contact->getKey()); }
	/** returns true if we could split this bucket */
	inline bool isSplittable(KadBucket& bucket) const { return bucket.size() >= bucketSize and &bucket == &buckets.back() and buckets.size() < getMaxNumBuckets(); }

	/** returns the width of a level in the accelerated lookup tree */
	inline size_type getLevelWidth() const { return (1 << accelerationFactor) - 1; }

	/** returns the theoretical number of buckets that we could have */
	inline size_type getMaxNumBuckets() const { return maxNumBuckets; }

	/** returns the index of the bucket that is the first on the level of the given bucket */
	inline size_type getFirstOnBucketLevel(size_type index) const { return index - (index % getLevelWidth()); }
	//@}

	KadContact *getContactInternal(const OverlayKey& key) { return const_cast<KadContact*>(getContact(key)); }

private:

	const NeighborTable& neighbortable; // only for read-only access
	/**
	 * number of bits used for accelerated lookup
	 * called "b" in research papers.
	 */
	size_type accelerationFactor;
	size_type maxNumBuckets;
	/** the key of this overlay node */
	OverlayKey myKey = OverlayKey::UNSPECIFIED_KEY;
	/** max size of the buckets, called "k" in research papers (k-Buckets) */
	size_type bucketSize;
	std::vector<KadBucket> buckets;
	KadPolicy* policy = nullptr;
	/// turns on Proximity Neighbor Selection
	bool proximityNeighborSelection;
	/// counts the number of contact replacements
	uint replacementCount = 0;
	/// counts the number of improvements made to contact paths
	uint updateCount = 0;
	/// discoveryRadius (from R2Kademlia class)
	uint8_t discoveryRadius = 1;
	/// enable special extension that also selects paths of equal length to be "better"
	bool enableSinglePathDiversity = false;

	///std::unordered_set<OverlayKey> pathSet; // only used for statistics
};



inline
KadContact_p KadRoutingTable::getContact(const OverlayKey& key) const
{
	return getBucket(key).getContact(key);
}

inline
KadContact* KadRoutingTable::getContactMutable(const OverlayKey& key)
{
	return const_cast<KadContact*>(getBucket(key).getContact(key));
}


} // end namespace



#endif // __ROUTINGSIM_KadRoutingTalbe_H_
