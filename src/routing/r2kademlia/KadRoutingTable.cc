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

//----------- KadBucket --------------
KadContact_p KadBucket::getContact(const OverlayKey& key)
{
	for (auto it = contacts->begin(); it != contacts->end(); it++){
		if((*it)->hasKey(key))
			return *it;
	}
	for (auto it = refillContacts->begin(); it != refillContacts->end(); it++){
		if((*it)->hasKey(key))
			return *it;
	}
	return KadContact_p();
}

}
