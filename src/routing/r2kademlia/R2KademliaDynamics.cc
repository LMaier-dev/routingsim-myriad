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
	EV_TRACE << "received KadUpdateRouteReq from " << packet->getSource().shortStr() << " final dest: " << packet->getDstNodeID().shortStr() << endl;

	// check for malformed packet
	ASSERT(packet->getContactArraySize() == packet->getPathArraySize());
	ASSERT(packet->getContactArraySize() == packet->getActionArraySize());

	receivedRouteUpdates++;
	receivedPackets++;

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
				EV_TRACE << "UpdateRouteReq Announce for contact " << contactkey.shortStr() << endl;
			        routingTable.addPath(contactkey, path, packet->getSeqNo(i), packet->getContactAge(i), source_path);
				
				if (updatePropagateStrategy == 2 ||
						updatePropagateStrategy == 4 ||
						updatePropagateStrategy == 5) propagateUpdate = true;
				break;

			case KAD_Withdraw:
				EV_TRACE << "UpdateRouteReq Withdraw for contact " << contactkey.shortStr() << endl;
				//TODO implement this
				break;

			case KAD_Change:
				EV_TRACE << "UpdateRouteReq Change for contact " << contactkey.shortStr() << endl;
			        routingTable.addPath(contactkey, path, packet->getSeqNo(i), packet->getContactAge(i), source_path);

				if (updatePropagateStrategy == 3 ||
						updatePropagateStrategy == 4 ||
						updatePropagateStrategy == 5) propagateUpdate = true;
				break;

			case KAD_Unreachable:
				// unreachable node, path shows the failed link
				ASSERT(path.size() == 1);
				ASSERT(path[0] != contactkey);

				EV_TRACE << "UpdateRouteReq UNREACHABLE contact: " << contactkey.shortStr() << ", link: < "
					 << path[0].shortStr() << ',' << contactkey.shortStr() << " >" << endl;

				if (path[0] != myOverlayKey) {
					// now invalidate all contacts with this link
					routingTable.invalidateContacts({contactkey, path[0]}, 0);
				}
				else { // if I'm part of the failed link, it's a PN link, so ignore this
					EV_TRACE << "ignored news about me or my PN" << endl;
				}

				if (updatePropagateStrategy == 1 ||
						updatePropagateStrategy == 5) propagateUpdate = true;
				break;

			default:
				EV_ERROR << "unknown action type: " << action << endl;
				break;
		} // end switch
	} // end for

	// forward UpdateReq closer towards destination
	if (propagateUpdate)
		propagateUpdateReq(std::move(packet));
}


void R2Kademlia::scheduleUpdate(const KadContact& affectedContact, KadRouteUpdateAction action)
{
	// do not schedule updates for myself
	if (affectedContact.getKey() == myOverlayKey)
		return;

	// we collect all update information for neighbors
	// send UpdateRouteReq to ID-wise closest Nodes
	size_t contactsLen = std::max((size_t)findNodeNumber, routingTable.getSplitSize());
	OverlayKey notifyKey;
	switch (updateNotifyStrategy) {
		case 1:
			notifyKey = myOverlayKey;
			break;
		case 2:
			notifyKey = affectedContact.getKey();
			break;
		default:
			throw cRuntimeError("unknown updateNotify strategy %i, values from 1 to 2 are allowed");
	}
	//KadContactList closeContacts(routingTable.getClosestContacts(affectedContact.getKey(), contactsLen));
	KadContactList closeContacts(routingTable.getClosestContacts(notifyKey, contactsLen, false));

	uint8_t numberOfNotifiedContacts = 0;
	if (closeContacts.size() > 0)
	{ // if there are contacts left to notify
		for (const auto& notifyContact : closeContacts)
		{ // each ID-wise close contact gets an UpdateRouteReq from us
			// if we reached the maximum fan out, exit the loop
			if (numberOfNotifiedContacts == updateMaxNotification)
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
				updateQueue[notifyContact->getKey()]= { { (action == KAD_Unreachable) ?
									  KadContact(affectedContact.getKey(), { myOverlayKey }, affectedContact.getStateSeqNo()) :
									  affectedContact, action } };
			}
			else
			{ // entry for to be notified contact exists, so append
				// in case of KAD_Unreachable we want to report also the failed link
				std::pair<KadContact, KadRouteUpdateAction> newelem=
					{ (action == KAD_Unreachable) ?
					  KadContact(affectedContact.getKey(), { myOverlayKey }, affectedContact.getStateSeqNo()) :
					  affectedContact,
					  action };
				// entry exists, so we need to append this update to the list (or replace an existing action)
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

	if (action == KAD_Unreachable) {
		if (not urgentUpdateScheduled) {
			addTimer(updateWaitTimeFast, 0, std::bind(&R2Kademlia::sendUrgentUpdates, this));
			urgentUpdateScheduled= true;
		}
	}
	else
	if (not updateScheduled)
	{
		addTimer(updateWaitTime, 0, std::bind(&R2Kademlia::sendUpdates, this));
		updateScheduled= true;
	}
}


// inspects a RouteUpdateReq message for Unreachability information
void R2Kademlia::evalUpdateReq(const KadUpdateRouteReq* packet)
{
	unsigned int numContacts= packet->getContactArraySize();
	for (unsigned int i= 0; i < numContacts; i++)
	{
		// get contact and corresponding path info
		const auto& contactkey= packet->getContact(i);
		const auto& path= packet->getPath(i);
		auto action= packet->getAction(i);

		if (contactkey == myOverlayKey) {
			// someone is telling me news about myself -> ignore
			continue;
		}

		switch (action) {

			case KAD_Unreachable:
				// unreachable node, path shows the failed link
				ASSERT(path.size() == 1);
				ASSERT(path[0] != contactkey);

				EV_TRACE << "Overheard unreachable contact: " << contactkey.shortStr() << ", link: < "
					 << path[0].shortStr() << ',' << contactkey.shortStr() << " >" << endl;
				if (path[0] != myOverlayKey) {
					// now invalidate all contacts with this link
					routingTable.invalidateContacts({contactkey, path[0]}, 0);
				}
				else { // if I'm part of the failed link, it's a PN link, so ignore this
					EV_TRACE << "ignored news about me" << endl;
				}
				break;

			default:
				break;
		} // end switch
	} // end for
}


// this functions tries to send an UpdateReq closer to the destination (ID-wise) neighborhood
void R2Kademlia::propagateUpdateReq(std::unique_ptr<KadUpdateRouteReq> packet)
{
	OverlayKey destContactKey= packet->getDstNodeID();
	unsigned int numContacts= packet->getContactArraySize();

	size_t contactsLen = std::max((size_t)findNodeNumber, routingTable.getSplitSize());
	std::list<std::pair<link_t,age_t>> notvialist;
	if (packet->getNotVia())
	{
		for (uint i= 0; i < packet->getFailedLinkArraySize(); i++) {
			notvialist.push_back(make_pair(packet->getFailedLink(i),packet->getFailedLinkAge(i)));
		}
	}
	KadContactList closeContacts(routingTable.getClosestContacts(destContactKey, contactsLen, true, notvialist));

	if (closeContacts.empty()) {
		EV_TRACE << "propagateUpdateReq(): Update forwarding ends. No contact found" << endl;
		return;
	}

	const KadContact& contact = chooseNextHop(closeContacts, destContactKey);
	// check whether that contact is closer to the target than we are (but do not send back to source)
	if ( contact.isValid() and (contact.getKey() ^ myOverlayKey) > (contact.getKey() ^ destContactKey) and not packet->isOverlayHopInPath(contact.getKey()) )
	{
		packet->appendRoute(contact);
		EV_TRACE << "propagateUpdateReq(): final dest: " << destContactKey.shortStr() << ", next dest: " << contact << endl;
	        resendPacket(std::move(packet));
	}
	else
	{
		EV_TRACE << "propagateUpdateReq(): Update forwarding ends, final dest: " << destContactKey.shortStr() << ", next dest: " << contact << endl;
	}
	// if no chance of getting closer to the destination: UpdateReq will be deleted here
}


/// send all updates with KAD_Unreachable only
void R2Kademlia::sendUrgentUpdates()
{
	// the queue (map) will be traversed
	// std::unordered_map< OverlayKey, UpdateList_t > updateQueue;
	EV_TRACE << "sendUrgentUpdates() enter. Queued Updates:" << updateQueue.size() << endl;

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
					// only queue this if contact still exists and state seq fits
					if (KadContact_p affcontactnow= routingTable.getContact(affectedcontact_it->first.getKey());
					    affcontactnow != nullptr and affcontactnow->getStateSeqNo() == affectedcontact_it->first.getStateSeqNo())
						urgentupdatelist.push_back(*affectedcontact_it);
					else
						EV_TRACE << "sendUrgentUpdates() - dismiss obsolete " << affectedcontact_it->first << endl;

					affectedcontact_it= updatelist.erase(affectedcontact_it);
				}
				else
					affectedcontact_it++;
			}
			EV_TRACE << "sendUrgentUpdates() - found urgent updates:" << urgentupdatelist.size() << endl;

			if (not urgentupdatelist.empty()) {
				// UpdateRouteReqs are not expecting an answer, so no RPC call
				auto packet = makePacket<KadUpdateRouteReq>(notify_target, "UpdateRouteReqURG");
				packet->setDstNodeID(notify_target->getKey());
				// write down all routing notifications for a specific contact
				packet->setContactArraySize(urgentupdatelist.size());
				packet->setPathArraySize(urgentupdatelist.size());
				packet->setActionArraySize(urgentupdatelist.size());
				packet->setSeqNoArraySize(urgentupdatelist.size());

				//EV_TRACE << "UrgentUpdateRoute packet to " << notifyContact_it->first.shortStr() << " w/ ";

				// now construct the routing information by the information from urgentupdatelist
				std::size_t i= 0;
				for (const auto& [affectedContact,action] : urgentupdatelist)
				{ // affectedcontactentry is std::pair<KadContact, KadRouteUpdateAction>
					packet->setContact(i, affectedContact.getKey());
					packet->setPath(i, affectedContact.getPath());
					packet->setSeqNo(i, affectedContact.getStateSeqNo());
					packet->setAction(i, action);
					//EV_TRACE << affectedContact << "; ";
					i++;
				}
				//EV_TRACE << endl;
				if ( i > 0 ) {
					// send UpdateRouteReq w/ failed link
					sendPacket(std::move(packet));
					sentRouteUpdates++;
				}
			} // end if something to send
		} // if contact is valid
	}

	// non urgent entries still there
	if (updateQueue.size() > 0) {
		EV_TRACE << "left non-urgent updates: " << updateQueue.size() << endl;
	}

	urgentUpdateScheduled= false;
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
			auto packet = makePacket<KadUpdateRouteReq>(notify_target, "UpdateRouteReq");
			packet->setDstNodeID(notify_target->getKey());
			//EV_TRACE << "UpdateRoute packet to " << notifyContact_it->first.shortStr() << " w/ ";
			auto& updatelist(notifyContact_it->second);
			// sanitize list first, i.e., remove obsolete entries
			for (auto affcontact_it= updatelist.begin(); affcontact_it != updatelist.end(); /* see loop */)
			{
				if (KadContact_p affcontactnow= routingTable.getContact(affcontact_it->first.getKey()); affcontactnow != nullptr)
				{
					if (affcontactnow->getStateSeqNo() > affcontact_it->first.getStateSeqNo())
					{ // current state for contact is newer, there should be another update for this
						//EV_TRACE << "(obsolete " << affcontact_it->first
						//	 << " #" << affcontactnow->getStateSeqNo()
						//	 << " > #" << affcontact_it->first.getStateSeqNo() << ") ";

						affcontact_it= updatelist.erase(affcontact_it);
					}
					else
						affcontact_it++;
				}
				else // contact not known anymore, probably replaced already
				{
					//EV_TRACE << "(removed " << affcontact_it->first.getKey().shortStr() << ") ";

					affcontact_it= updatelist.erase(affcontact_it);
				}
			}
			// write down all routing notifications for a specific contact
			packet->setContactArraySize(updatelist.size());
			packet->setPathArraySize(updatelist.size());
			packet->setActionArraySize(updatelist.size());
			packet->setSeqNoArraySize(updatelist.size());
			packet->setContactAgeArraySize(updatelist.size());
			
			// now construct the routing information by the information from updatelist
			std::size_t i= 0;
			for (const auto& [affectedContact,action] : updatelist)
			{ // affectedcontactentry is std::pair<KadContact, KadRouteUpdateAction>
				// check first if state of contact has been updated meanwhile
				// (skip this update then, it is obsolete)
				packet->setContact(i, affectedContact.getKey());
				packet->setPath(i, affectedContact.getPath());
				packet->setSeqNo(i, affectedContact.getStateSeqNo());
				packet->setContactAge(i, affectedContact.getAge());
				packet->setAction(i, action);
				//EV_TRACE << affectedContact << "; ";
				i++;
			}
			//EV_TRACE << endl;
			// send UpdateRouteReq w/ failed link
			if ( i > 0 ) {
				sendPacket(std::move(packet));
				sentRouteUpdates++;
			};
		}
	}

	// the updateQueue map should is now empty again
	// reset the updateScheduled variable
	updateScheduled= false;
}



// send a find node request excluding paths that include the failed links (stored in RDS)
void R2Kademlia::scheduleRediscovery(const OverlayKey& contactkey, bool immediate)
{
	KadContact_p rt_contact= routingTable.getContact(contactkey);
	if (rt_contact and rt_contact->isRediscovering() and not rt_contact->getRDS().viaContacts.empty())
	{
		// do not schedule for rediscovery if there is something still in progress
		EV_WARN << "Rediscovery not scheduled for contact " << contactkey.shortStr()
			<< " because it is already in rediscovering state" << endl;
		return;
	}

	EV_TRACE << (immediate ? "Urgent " : "") << "Rediscovery scheduled for contact " << contactkey.shortStr() << endl;

	if (immediate) {
	        urgentRediscoverQueue.push(contactkey);
	}
	else {
		rediscoverQueue.push(contactkey);
	}


	if (not urgentRediscoveryScheduled and not urgentRediscoverQueue.empty()) {
		addTimer(uniform(rediscoveryUrgentWaitTime/2,rediscoveryUrgentWaitTime*1.5), 0, std::bind(&R2Kademlia::rediscoverContacts, this));
		urgentRediscoveryScheduled= true;
	}

	if (not rediscoveryScheduled and not rediscoverQueue.empty()) {
		addTimer(uniform(rediscoveryWaitTime/2,rediscoveryWaitTime*1.5), 0, std::bind(&R2Kademlia::rediscoverContacts, this));
		rediscoveryScheduled= true;
	}
}


void R2Kademlia::rediscoverContacts() {
	// traverse list of scheduled contacts for rediscovery
	// the list is sorted by ID-wise vicinity, because contacts in lower buckets are more important
	// recheck status and set to rediscover
	EV_TRACE << "rediscoverContacts called, urgent queued: " << urgentRediscoverQueue.size() << " normal queued: " << rediscoverQueue.size() << endl;
	rediscoverQueueClass& rediscoverQ= urgentRediscoverQueue.empty() ? rediscoverQueue : urgentRediscoverQueue;
	unsigned short sent(0U);
	constexpr unsigned short max_rediscovery_msg_batch= 5U;
	while (!rediscoverQ.empty() and sent < max_rediscovery_msg_batch)
	{
		auto& contactkey= rediscoverQ.top();
		// check for current status of scheduled contact
		KadContact* rt_contact= const_cast<KadContact*>(routingTable.getContact(contactkey));
		if (rt_contact) { // so contact still exists
			// contact may have been recovered meanwhile
			// do not rediscover contact which is already Rediscovering (unless in retry)
			// if viaContacts is empty, refill when retrying
			if (rt_contact->isInvalid() or (rt_contact->isRediscovering() and rt_contact->getRDS().viaContacts.empty()))
			{
				// set contact state to rediscovering
				if (not rt_contact->isRediscovering())
					rt_contact->setRediscovering();

				// it is only useful to store node ids, because later on, those contacts may have been gone already
				// collect still valid closest contacts to the failed contact
				auto contacts = routingTable.getClosestContacts(contactkey, rediscoveryMaxNotification, true);
				//auto myclosecontacts = routingTable.getClosestContacts(myOverlayKey, findNodeNumber, true);
				//auto myclosecontacts_it= myclosecontacts.begin();
				for (auto c : contacts) {
					if (c != nullptr) {
						rt_contact->addRediscoverVia(c->getKey());
						//if (myclosecontacts_it != myclosecontacts.end() and *myclosecontacts_it != nullptr) {
						//	rt_contact->addRediscoverVia((*myclosecontacts_it)->getKey());
						//	myclosecontacts_it++;
						//}
					}
				}

				// while (myclosecontacts_it != myclosecontacts.end()) {
				// 	if (*myclosecontacts_it != nullptr) {
				// 		rt_contact->addRediscoverVia((*myclosecontacts_it)->getKey());
				// 	}
				// 	myclosecontacts_it++;
				// }

				EV_TRACE << "rediscoverContacts(): number of via contacts: " << contacts.size() << endl;
				if (rt_contact->getRDS().retryCounter == 0 and contacts.size() > 0) { // immediate rediscovery
					sendRediscovery(rt_contact->getKey(), rediscoveryFanOutFactor);
					sent++;
				}
				else {
					// schedule rediscovery for later (max backoffFactor arbitrarily set to 1024)
					unsigned long backoffFactor(1UL << (rt_contact->getRDS().retryCounter < 10 ? rt_contact->getRDS().retryCounter : 10));
					unsigned long period_in_ms= 100*backoffFactor;
					unsigned long scatter = uniform(0, period_in_ms);

					EV_TRACE << "rediscoverContacts(): will sendRediscovery in " << period_in_ms/2+scatter << "ms, retry:" << rt_contact->getRDS().retryCounter << endl;
					addTimer(SimTime(period_in_ms/2+scatter,SIMTIME_MS), 0, std::bind(&R2Kademlia::sendRediscovery, this, rt_contact->getKey(), rediscoveryFanOutFactor));
				}

			}
		}
		rediscoverQ.pop();
	} // end while

	// next call of this procedure will be scheduled on demand
	if (urgentRediscoverQueue.empty()) {
		urgentRediscoveryScheduled= false;
	}
	else {
		// do not use rediscoveryUrgentWaitTime
		// to dampen the rediscovery rate
		addTimer(uniform(rediscoveryWaitTime/4,rediscoveryWaitTime*3/4), 0, std::bind(&R2Kademlia::rediscoverContacts, this));
		urgentRediscoveryScheduled= true;
		return;
	}

	if (rediscoverQueue.empty()) {
		rediscoveryScheduled= false;
	}
	else {
		addTimer(uniform(rediscoveryWaitTime/2,rediscoveryWaitTime*1.5), 0, std::bind(&R2Kademlia::rediscoverContacts, this));
		rediscoveryScheduled= true;
	}
}


// send a FindNodeReq message for rediscovery
void R2Kademlia::sendRediscovery(OverlayKey failedcontactkey, uint8_t maxFanOut) {

	KadContact* failedcontact= const_cast<KadContact*>(routingTable.getContact(failedcontactkey));
	if (failedcontact == nullptr or not failedcontact->isRediscovering())
		return;
	// contact still exists
	EV_TRACE << "sendRediscovery w/ " << failedcontact->getKey().shortStr() << endl;

	if (auto& contacts= failedcontact->getRDS().viaContacts; contacts.size() > 0)
	{
		// use several contacts to send multiple Rediscovery requests in parallel
		for (auto fanout= 0; fanout < maxFanOut and contacts.size() > 0; fanout++)
		{
			// must check for existence and validity of contact
			if (auto contact= routingTable.getContact(contacts.front()); contact != nullptr and contact->isValid())
			{
				if (contact->getKey() == myOverlayKey)
				{
					EV_ERROR << "I'm the next hop for the destination. Will not send anything." << endl;
					return;
				}
				auto packet = makePacket<KadFindNodeReq>(contact, "FindNode");
				packet->setDstNodeID(failedcontact->getKey());
				packet->setExact(true);
				packet->setRediscovery(true);
				for (auto& failedLink : failedcontact->getRDS().failedLinkList)
					packet->addNotViaLink(failedLink.first, (simTime() - failedLink.second).inUnit(SIMTIME_MS));
				sentRediscovery++;
				EV_TRACE << "Rediscovery sending ... msg id=" << packet->getId() << " via " << contact->getKey().shortStr() << " fanout: " << fanout << endl;
				EV_TRACE << "sending KadFindNodeReq" << endl;
				sendRpcCall(
					std::move(packet),
					std::bind(&R2Kademlia::handleFindNodeResponse, this, ph::_1, ph::_2),
					std::bind(&R2Kademlia::handleFindNodeResponse, this, ph::_1, ph::_2),
					std::bind(&R2Kademlia::handleRPCTimeOut, this, ph::_1, ph::_2)
					);
			}
			// remove the used element from the list
			contacts.pop_front();
		}
	}
	else
	{ // no (more) contacts to choose from (initially only possible for isolated nodes)
		EV_WARN << "no (more) via contacts found, cannot send rediscovery" << endl;
		// this may be triggered by an answer arriving later than the first one that already hit this condition here
		// so check whether contact was already scheduled
		if (std::find(urgentRediscoverQueue.begin(),urgentRediscoverQueue.end(),failedcontactkey) != urgentRediscoverQueue.end()
		    or
		    std::find(rediscoverQueue.begin(),rediscoverQueue.end(),failedcontactkey) != rediscoverQueue.end()) {
			EV_TRACE << "already scheduled for rediscovery" << endl;
			return;
		}

		// try several rounds of rediscovery for contacts
		if (failedcontact->getRDS().retryCounter < maxRediscoveryRetries) {
			failedcontact->getRDS().retryCounter++;
			EV_TRACE << "will retry rediscovery for contact " << failedcontact->getKey().shortStr() << " retry: " << failedcontact->getRDS().retryCounter << endl;
			scheduleRediscovery(failedcontact->getKey(), routingTable.isInLastBucket(failedcontact->getKey()));
		}
		else {
			// do not remove physical contacts
			if (failedcontact->isInvalid() and failedcontact->isPhysicalNeighbor() == false) {
				EV_TRACE << "rediscovery finally failed, removing contact " << failedcontact->getKey().shortStr() << endl;
				routingTable.removeContact(failedcontact);
			}
		}
	}
}

} // end namespace
