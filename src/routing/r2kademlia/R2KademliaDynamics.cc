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

#include "R2Kademlia.h"
/**
 * @file R2KademliaDynamics.cc
 * @see R2Kademlia.h for class documentation
 *
 * This file contains dynamics related methods such as rediscovery and handling UpdateRouteReqs
 */

namespace routingsim {
// for std::bind with callback functions
namespace ph = std::placeholders;


// UPDATEROUTEREQ
void R2Kademlia::handlePacket(std::unique_ptr<KadUpdateRouteReq> packet) {
	EV_TRACE << "received KadUpdateRouteReq from " << packet->getSource().shortStr() << endl;

	// check for malformed packet
	ASSERT(packet->getContactArraySize() == packet->getPathArraySize());
	ASSERT(packet->getContactArraySize() == packet->getActionArraySize());

	receivedRouteUpdates++;

	// path to origin of this KadUpdateRouteReq (excluding myself)
	kad_path source_path= packet->reversePath(packet->getSrcPath(0, packet->getRouteArraySize()-1));
	/*
	EV_TRACE << "path to origin: <";
	for (const OverlayKey& k : source_path)
		EV_TRACE << k.shortStr() << ' ';
	EV_TRACE << '>' << endl;
	*/

	bool propagateUpdate(false);
	// this is route update information only, no answer required
	unsigned int numContacts= packet->getContactArraySize();
	for (unsigned int i= 0; i < numContacts; i++)
	{
		// get contact and corresponding path info
		const auto& contactkey= packet->getContact(i);
		const auto& path= packet->getPath(i);
		auto action= packet->getAction(i);

		if (contactkey == myOverlayKey) {
			// someone is telling me news about myself -> ignore
			EV_TRACE << "ignoring update about me" << endl;
			continue;
		}

		/* 
		EV_TRACE << "reported path to contact " << contactkey.shortStr() << " <";
		for (const auto& k : path)
			EV_TRACE << k.shortStr() << ' ';
		EV_TRACE << '>' << endl;
		*/
		switch (action) {
			case KAD_Announce:
				EV_TRACE << "UpdateReq Announce for contact " << contactkey.shortStr() << endl;
			        routingTable.addPath(contactkey, path, source_path);
				break;

			case KAD_Withdraw:
				EV_TRACE << "UpdateReq Withdraw for contact " << contactkey.shortStr() << endl;
				//TODO implement this
				break;

			case KAD_Change:
				EV_TRACE << "UpdateReq Change for contact " << contactkey.shortStr() << endl;
			        routingTable.addPath(contactkey, path, source_path);
				break;

			case KAD_Unreachable:
				// unreachable node, path shows the failed link
				ASSERT(path.size() == 1);

				EV_TRACE << "Unreachable contact: " << contactkey.shortStr() << ", link: < "
					 << packet->getPath(0).front().shortStr() << ',' << contactkey.shortStr() << " >" << endl;
				// now invalidate all contacts with this link
				routingTable.invalidateContacts(contactkey, packet->getPath(0).front());
				propagateUpdate= true;
				break;

			default:
				EV_ERROR << "unknown action type: " << action << endl;
				break;
		}
	}

	// forward UpdateReq closer towards destination
	if (propagateUpdate)
		propagateUpdateReq(std::move(packet));

}

	
void R2Kademlia::scheduleUpdate(const KadContact& affectedContact, KadRouteUpdateAction action)
{

	// we collect all update information for neighbors
	// send UpdateRouteReq to ID-wise closest Nodes
	size_t contactsLen = std::max((size_t)findNodeNumber, routingTable.getSplitSize());
	KadContactList closeContacts(routingTable.getClosestContacts(affectedContact.getKey(), contactsLen));

	const uint8_t maxNumberOfNotifiedContacts= 5;
	uint8_t numberOfNotifiedContacts = 0;
	if (closeContacts.size() > 0)
	{ // if there are contacts left to notify
		for (const auto& notifyContact : closeContacts)
		{ // each ID-wise close contact gets an UpdateRouteReq from us
			// if we reached the maximum fan out, exit the loop
			if (numberOfNotifiedContacts == maxNumberOfNotifiedContacts)
				break;
			
			//EV_TRACE << "scheduleUpdate for " << affectedContact.getKey().shortStr() << " (" << action
			//	 << ") notify:" << notifyContact->getKey().shortStr() ;
			// except when it is the contact itself
			if ( affectedContact.getKey() == notifyContact->getKey() ) {
				//EV_TRACE << " is the same - skipped." << endl;
				continue;
			}
			// or it is a physical neighbor of the affected node (here the last hop in the path)
			if ( affectedContact.getPathLength() > 0
			     and
			     affectedContact.getPath()[affectedContact.getPathLength()-1] == notifyContact->getKey() ) {
				//EV_TRACE << " is the last hop - skipped." << endl;
				
				continue;
			}

			// create new entry or append update
			if ( auto entry= updateQueue.find(notifyContact->getKey()); entry == updateQueue.end() ) {
				// need to create new entry
				// in case of KAD_Unreachable we want to report also the failed link
				updateQueue[notifyContact->getKey()]= { std::make_pair((action == KAD_Unreachable) ?
										       KadContact(affectedContact.getKey(), { myOverlayKey }) :
										       affectedContact, action) };
			}
			else
			{ // entry for contact exists, so append
				// in case of KAD_Unreachable we want to report also the failed link
				auto newelem= std::make_pair((action == KAD_Unreachable) ?
							     KadContact(affectedContact.getKey(), { myOverlayKey }) : affectedContact, action);
				// entry exists, so we need to append this update to the list
				auto existelem= std::find_if(entry->second.rbegin(), entry->second.rend(),
							     [&](const auto& p1) { return p1.first.getKey() == newelem.first.getKey(); });
				if (existelem != entry->second.rend()) { // if exists: replace with newer info
					*existelem= newelem;
				}
				else { // not found, append
					entry->second.emplace_back(newelem);
				}
			}
			//EV_TRACE << " done." << endl;
			numberOfNotifiedContacts++;
		} // end for each contact
			
	} // endif

	if (!updateScheduled)
	{
		if (action == KAD_Unreachable)
			addTimer(updateWaitTimeFast, 0, std::bind(&R2Kademlia::sendUrgentUpdates, this));
		else
			addTimer(updateWaitTime, 0, std::bind(&R2Kademlia::sendUpdates, this));

		updateScheduled= true;
	}
}


void R2Kademlia::propagateUpdateReq(std::unique_ptr<KadUpdateRouteReq> packet)
{
	OverlayKey affectedContactKey;
	unsigned int numContacts= packet->getContactArraySize();
	for (unsigned int i= 0; i < numContacts; i++)
	{
		// get contact and corresponding path info
		affectedContactKey= packet->getContact(i);
		auto action= packet->getAction(i);

		if (affectedContactKey == myOverlayKey) {
			continue;
		}

		if (action == KAD_Unreachable) {
			break;
		}
	}
	size_t contactsLen = std::max((size_t)findNodeNumber, routingTable.getSplitSize());
	KadContactList closeContacts(routingTable.getClosestContacts(affectedContactKey, contactsLen));
	const KadContact& contact = chooseNextHop(closeContacts, affectedContactKey, myOverlayKey);
	// check whether that contact is closer to the target than we
	if ( (contact.getKey() ^ myOverlayKey) > (contact.getKey() ^ affectedContactKey) )
	{
		packet->setSource(myOverlayKey);
		packet->setHopLimit(hopLimit);
		// this will reset the route
		packet->setRouteArraySize(0);
		packet->setActiveArraySize(0);
		packet->appendRoute(contact);
	}
}


/// send all updates with KAD_Unreachable only
void R2Kademlia::sendUrgentUpdates()
{
	// the queue (map) will be traversed
	// std::unordered_map< OverlayKey, UpdateList_t > updateQueue;
	for ( auto notifyContact_it= updateQueue.begin();
	      notifyContact_it != updateQueue.end();
	      notifyContact_it= updateQueue.erase(notifyContact_it) )
	{
		KadContact_p notify_target= routingTable.getContact(notifyContact_it->first);
		if (notify_target and notify_target->isValid())
		{ // only notify contacts that we have a path to (i.e., that are valid)
			// Extract urgent updates first
			auto& updatelist(notifyContact_it->second);
			UpdateList_t urgentupdatelist;

			for (auto affectedcontact_it= updatelist.begin(); affectedcontact_it != updatelist.end(); /* see loop */ )
			{ // affectedcontactentry is std::pair<KadContact, KadRouteUpdateAction>
				if (affectedcontact_it->second == KAD_Unreachable) {
					urgentupdatelist.push_back(*affectedcontact_it);
					affectedcontact_it= updatelist.erase(affectedcontact_it);
				}
				else
					affectedcontact_it++;
			}

			if (not urgentupdatelist.empty()) {
				// UpdateRouteReqs are not expecting an answer, so no RPC call
				auto packet = makePacket<KadUpdateRouteReq>(notify_target, "UpdateRoute");

				// write down all routing notifications for a specific contact
				packet->setContactArraySize(urgentupdatelist.size());
				packet->setPathArraySize(urgentupdatelist.size());
				packet->setActionArraySize(urgentupdatelist.size());

				EV_TRACE << "UrgentUpdateRoute packet to " << notifyContact_it->first.shortStr() << " w/ ";

				// now construct the routing information by the information from urgentupdatelist
				std::size_t i= 0;
				for (const auto& [affectedContact,action] : urgentupdatelist)
				{ // affectedcontactentry is std::pair<KadContact, KadRouteUpdateAction>
					packet->setContact(i, affectedContact.getKey());
					packet->setPath(i, affectedContact.getPath());
					packet->setAction(i, action);
					EV_TRACE << affectedContact << "; ";
					i++;
				}

				EV_TRACE << endl;
				// send UpdateRouteReq w/ failed link
				sendPacket(std::move(packet));
				sentRouteUpdates++;
			} // end if something to send
		} // if contact is valid
	}

	// non urgent entries still there
	if (updateQueue.size() > 0) {
		addTimer(updateWaitTime - updateWaitTimeFast, 0, std::bind(&R2Kademlia::sendUpdates, this));
		updateScheduled= true;
	}
	else {
		// the updateQueue map should is now empty again
		// reset the updateScheduled variable
		updateScheduled= false;
	}
}


/// send all updates
void R2Kademlia::sendUpdates()
{
	// the queue (map) will be traversed
	// std::unordered_map< OverlayKey, UpdateList_t > updateQueue;
	for ( auto notifyContact_it= updateQueue.begin();
	      notifyContact_it != updateQueue.end();
	      notifyContact_it= updateQueue.erase(notifyContact_it) )
	{
		KadContact_p notify_target= routingTable.getContact(notifyContact_it->first);
		if (notify_target and notify_target->isValid())
		{ // only notify contacts that we have a path to (i.e., that are valid)
			// UpdateRouteReqs are not expecting an answer, so no RPC call
			auto packet = makePacket<KadUpdateRouteReq>(notify_target, "UpdateRoute");
			//EV_TRACE << "UpdateRoute packet to " << notifyContact_it->first.shortStr() << " w/ ";
			const auto& updatelist(notifyContact_it->second);
			// write down all routing notifications for a specific contact
			packet->setContactArraySize(updatelist.size());
			packet->setPathArraySize(updatelist.size());
			packet->setActionArraySize(updatelist.size());
			
			// now construct the routing information by the information from updatelist
			std::size_t i= 0;
			for (const auto& [affectedContact,action] : updatelist)
			{ // affectedcontactentry is std::pair<KadContact, KadRouteUpdateAction>
				packet->setContact(i, affectedContact.getKey());
				packet->setPath(i, affectedContact.getPath());
				packet->setAction(i, action);
				//EV_TRACE << affectedContact << "; ";
				i++;
			}
			//EV_TRACE << endl;
			// send UpdateRouteReq w/ failed link
			sendPacket(std::move(packet));
			sentRouteUpdates++;
		}
	}

	// the updateQueue map should is now empty again
	// reset the updateScheduled variable
	updateScheduled= false;
}



// send a find node request excluding paths that include the failed link
void R2Kademlia::scheduleRediscovery(const OverlayKey& contactkey, const OverlayKey& notvia_unreachable_node, const OverlayKey& notvia_failednode_neighbor)
{
	KadContact_p rt_contact= routingTable.getContact(contactkey);
	if (rt_contact and rt_contact->isRediscovering())
	{
		// do not schedule for rediscovery if there is something still in progress
		EV_WARN << "Rediscovery not scheduled for contact " << contactkey.shortStr()
			<< " because it is already in rediscovering state" << endl;
		return;
	}
	
	EV_TRACE << "Rediscovery scheduled for contact " << contactkey.shortStr() << " <"
		<< notvia_unreachable_node.shortStr() << ' '
		<< notvia_failednode_neighbor.shortStr() << '>' << endl;

	auto rediscoverContact=
		std::make_unique<const KadContact>(contactkey, std::vector<OverlayKey>{notvia_unreachable_node, notvia_failednode_neighbor});
	
	rediscoverQueue.push_back(std::move(rediscoverContact));

	if (!rediscoveryScheduled)
	{
		addTimer(rediscoveryWaitTime, 0, std::bind(&R2Kademlia::rediscoverContacts, this));
		rediscoveryScheduled= true;
	}
}


void R2Kademlia::rediscoverContacts() {
	// traverse list of scheduled contacts for rediscovery
	// recheck status and set to rediscover
	EV_TRACE << "rediscoverContacts called, Queued: " << rediscoverQueue.size() << endl;
	
	while (!rediscoverQueue.empty())
	{
		auto& contact= rediscoverQueue.front();
		// check for current status of scheduled contact
		auto rt_contact= routingTable.getContact(contact->getKey());
		if (rt_contact) { // so contact still exists
			// contact may have been recovered meanwhile
			// do not rediscover contact which is already Rediscovering
			if (rt_contact->isInvalid() and not rt_contact->isRediscovering())
			{
				// set contact state to rediscovering
				const_cast<KadContact*>(rt_contact)->setRediscovering();
			        const kad_path& failedLink(contact->getPath());
				ASSERT(failedLink.size() == 2);
				sendFindNode(rt_contact->getKey(), true, &failedLink[0], &failedLink[1]);
			}
		}
		rediscoverQueue.pop_front();
	}

	// next call of this procedure will be scheduled on demand
	rediscoveryScheduled= false;
}


} // end namespace
