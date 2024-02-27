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
#include <unordered_set>
#include <utility>
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
	receivedVolume+= packet->getByteLength();

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
		auto nodeDegree= packet->getNodeDegree(i);

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
			        routingTable.addPath(contactkey, path, packet->getSeqNo(i), packet->getPathInfoAge(i), nodeDegree, source_path);
				
				if (updatePropagateStrategy == 2 or
				    updatePropagateStrategy == 4 or
				    updatePropagateStrategy == 5) propagateUpdate = true;
				break;

			case KAD_Withdraw:
				EV_TRACE << "UpdateRouteReq Withdraw for contact " << contactkey.shortStr() << endl;
				//TODO implement this
				// Withdraw should be used in case a node performs an ordered leave
				break;

			case KAD_Change:
				EV_TRACE << "UpdateRouteReq Change for contact " << contactkey.shortStr() << endl;
			        routingTable.addPath(contactkey, path, packet->getSeqNo(i), packet->getPathInfoAge(i), nodeDegree, source_path);

				if (updatePropagateStrategy == 3 or
				    updatePropagateStrategy == 4 or
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

				if (updatePropagateStrategy == 1 or
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


// schedule an UpdateRouteReq to Neighbors about affectedContact (Announce, Withdraw, Changed, Unreachable)
void R2Kademlia::scheduleUpdate(const KadContact& affectedContact, KadRouteUpdateAction action)
{
	// do not schedule updates for myself
	if (affectedContact.getKey() == myOverlayKey)
		return;

	// we collect all update information for neighbors
	// send UpdateRouteReq to ID-wise closest Nodes (even if they are currently not reachable/valid)
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
			throw cRuntimeError("unknown updateNotify strategy %u, values from 1 to 2 are allowed", updateNotifyStrategy);
	}
	// note that validcontactonly == false,
	// i.e., we also use unreachable contacts as destination and try to forward
	// it via the overlay routing strategy as close as possible to them
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
			// the updateQueue contains the contacts to which the updates are sent
			// and for each entry there is a list of affected contacts and actions std::list< std::pair<KadContact, KadRouteUpdateAction> >
			if ( auto entry= updateQueue.find(notifyContact->getKey()); entry == updateQueue.end() ) {
				// need to create new entry
				// in case of KAD_Unreachable we want to report also the failed link
				updateQueue[notifyContact->getKey()]= { { (action == KAD_Unreachable) ?
						KadContact(myOverlayKey, affectedContact.getKey(), { myOverlayKey }, affectedContact.getStateSeqNo()) :
									  affectedContact, action } };
			}
			else
			{ // entry for to be notified contact exists, so append
				// in case of KAD_Unreachable we want to report also the failed link
				std::pair<KadContact, KadRouteUpdateAction> newelem=
					{ (action == KAD_Unreachable) ?
					  KadContact(myOverlayKey, affectedContact.getKey(), { myOverlayKey }, affectedContact.getStateSeqNo()) :
					  affectedContact,
					  action };
				// entry exists, so we need to append this update to the list (or replace an existing action)
				auto existelem= std::find_if(entry->second.rbegin(), entry->second.rend(),
							     [&](const auto& p1) { return p1.first.getKey() == newelem.first.getKey(); });
				if (existelem != entry->second.rend()) {
                                        // if exists: replace with newer info in case of a KAD_Change or KAD_Announce
					// if it is a KAD_Unreachable prepend it
					if (existelem->second == KAD_Change or existelem->second == KAD_Announce)
						*existelem= newelem;
					else {
						existelem++; // required, because of .base() returning the next element
						// in the worst case the iterator is now at .rend()
						// insert before the found entry
						entry->second.insert(existelem.base(),newelem);
					}
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
			addRandomizedTimer(updateWaitTimeFast, 0, std::bind(&R2Kademlia::sendUrgentUpdates, this));
			urgentUpdateScheduled= true;
		}
	}
	else
	if (not updateScheduled)
	{
		addRandomizedTimer(updateWaitTime, 0, std::bind(&R2Kademlia::sendUpdates, this));
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
	auto destinationID(packet->getDstNodeID());
	forwardCloser(std::move(packet), destinationID);
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
				// only queue this if contact still exists and state seq fits
				if (KadContact_p affcontactnow= routingTable.getContact(affectedcontact_it->first.getKey());
				    affcontactnow != nullptr
				    and (affcontactnow->isValid() or (affectedcontact_it->second == KAD_Unreachable))
				    and affcontactnow->getStateSeqNo() == affectedcontact_it->first.getStateSeqNo()
				    and affcontactnow->getLastSeen() == affectedcontact_it->first.getLastSeen() )
					urgentupdatelist.push_back(*affectedcontact_it);
				else
					EV_TRACE << "sendUrgentUpdates() - dismiss obsolete " << affectedcontact_it->first << endl;

				affectedcontact_it= updatelist.erase(affectedcontact_it);
			}
			EV_TRACE << "sendUrgentUpdates() - found urgent updates:" << urgentupdatelist.size() << endl;

			if (not urgentupdatelist.empty()) {
				// UpdateRouteReqs are not expecting an answer, so no RPC call
				auto packet = makePacket<KadUpdateRouteReq>(notify_target, "UpdateRouteReqURG");
				packet->setDstNodeID(notify_target->getKey());
				// write down all routing notifications for a specific contact
				packet->setContactArraySize(urgentupdatelist.size());
				packet->setPathArraySize(urgentupdatelist.size());
				packet->setSeqNoArraySize(urgentupdatelist.size());
				packet->setPathInfoAgeArraySize(urgentupdatelist.size());
				packet->setNodeDegreeArraySize(urgentupdatelist.size());
				packet->setActionArraySize(urgentupdatelist.size());

				EV_TRACE << "UrgentUpdateRoute packet to " << notifyContact_it->first.shortStr() << " w/ ";

				// now construct the routing information by the information from urgentupdatelist
				std::size_t i= 0;
				for (const auto& [affectedContact,action] : urgentupdatelist)
				{ // affectedcontactentry is std::pair<KadContact, KadRouteUpdateAction>
					packet->setContact(i, affectedContact.getKey());
					packet->setPath(i, affectedContact.getPath());
					packet->setSeqNo(i, affectedContact.getStateSeqNo());
					packet->setPathInfoAge(i, affectedContact.getPathAge());
					packet->setNodeDegree(i, affectedContact.getNodeDegree());
					packet->setAction(i, action);
					EV_TRACE << ' ' <<
						((action == KAD_Announce) ? 'A' :
						 ((action == KAD_Withdraw) ? 'W' :
						  ((action == KAD_Change) ? 'C' :
						   ((action == KAD_Unreachable) ? 'U' : '?'))))
						 << ' ' << affectedContact << "; ";
					i++;
				}
				EV_TRACE << endl;
				if ( i > 0 ) {
					// send UpdateRouteReq w/ failed link
					sendPacket(std::move(packet));
					sentRouteUpdates++;
				}
			} // end if something to send
		} // if contact to be notified is valid
		else {
			EV_TRACE << "skipped invalid notify contact " << notifyContact_it->first.shortStr() << endl;
		}
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
	EV_TRACE << "sendUpdates() enter. Queued Updates:" << updateQueue.size() << endl;

	// the queue (map) will be traversed
	// std::unordered_map< OverlayKey, UpdateList_t > updateQueue;
	for ( auto notifyContact_it= updateQueue.begin();
	      notifyContact_it != updateQueue.end();
	      notifyContact_it= updateQueue.erase(notifyContact_it) )
	{
		KadContact_p notify_target= routingTable.getContact(notifyContact_it->first);
		if (notify_target and notify_target->isValid())
		{ // only notify contacts that we have a validated path to (i.e., that are valid)
			// UpdateRouteReqs are not expecting an answer, so no RPC call
			auto packet = makePacket<KadUpdateRouteReq>(notify_target, "UpdateRouteReq");
			packet->setDstNodeID(notify_target->getKey());
			EV_TRACE << "UpdateRoute packet to " << notifyContact_it->first.shortStr() << " w/ ";
			auto& updatelist(notifyContact_it->second);
			// sanitize list first, i.e., remove obsolete entries
			for (auto affcontact_it= updatelist.begin(); affcontact_it != updatelist.end(); /* see loop */)
			{
				if (KadContact_p affcontactnow= routingTable.getContact(affcontact_it->first.getKey()); affcontactnow != nullptr)
				{
					if ( (affcontactnow->isNotValid() and (affcontact_it->second != KAD_Unreachable))
					     or
					     (affcontactnow->getStateSeqNo() > affcontact_it->first.getStateSeqNo()
					      or
					      affcontactnow->getLastSeen() > affcontact_it->first.getLastSeen()) )
					{ // current state for contact is newer, there should be another update for this
						EV_TRACE << "(obsolete " << affcontact_it->first
							 << ((affcontactnow->getStateSeqNo() > affcontact_it->first.getStateSeqNo()) ? " seq)" : ")")
							 << ' ';

						affcontact_it= updatelist.erase(affcontact_it);
					}
					else {
						//ASSERT( affcontact_it->second != KAD_Unreachable ? (affcontactnow->isValid()) : true );
						affcontact_it++;
					}
				}
				else // contact not known anymore, probably replaced already
				{
					EV_TRACE << "(removed " << affcontact_it->first.getKey().shortStr() << ") ";

					affcontact_it= updatelist.erase(affcontact_it);
				}
			}
			// write down all routing notifications for a specific contact
			packet->setContactArraySize(updatelist.size());
			packet->setPathArraySize(updatelist.size());
			packet->setActionArraySize(updatelist.size());
			packet->setSeqNoArraySize(updatelist.size());
			packet->setPathInfoAgeArraySize(updatelist.size());
			packet->setNodeDegreeArraySize(updatelist.size());

			// now construct the routing information by the information from updatelist
			std::size_t i= 0;
			for (const auto& [affectedContact,action] : updatelist)
			{ // affectedcontactentry is std::pair<KadContact, KadRouteUpdateAction>
				// check first if state of contact has been updated meanwhile
				// (skip this update then, it is obsolete)
				packet->setContact(i, affectedContact.getKey());
				packet->setPath(i, affectedContact.getPath());
				packet->setSeqNo(i, affectedContact.getStateSeqNo());
				packet->setPathInfoAge(i, affectedContact.getAge());
				packet->setNodeDegree(i, affectedContact.getNodeDegree());
				packet->setAction(i, action);
				EV_TRACE << ' ' <<
					((action == KAD_Announce) ? 'A' :
					 ((action == KAD_Withdraw) ? 'W' :
					  ((action == KAD_Change) ? 'C' :
					   ((action == KAD_Unreachable) ? 'U' : '?'))))
					 << ' ' << affectedContact << "; ";
				i++;
			}
			EV_TRACE << endl;
			// send UpdateRouteReq w/ failed link
			if ( i > 0 ) {
				sendPacket(std::move(packet));
				sentRouteUpdates++;
			};
		} // if contact to be notified is valid
		else {
			EV_TRACE << "skipped invalid notify contact " << notifyContact_it->first.shortStr() << endl;
		}
	}

	// the updateQueue map should is now empty again
	// reset the updateScheduled variable
	updateScheduled= false;
}



// schedule a find node request excluding paths that include the failed links (stored in RDS)
void R2Kademlia::scheduleRediscovery(const OverlayKey& contactkey, rediscoverQueueClass& rediscoverQ, simtime_t rediscoveryWaitTime)
{
	KadContact_p rt_contact= routingTable.getContact(contactkey);
	if (rt_contact and rt_contact->isRediscovering() and not rt_contact->getRDS().viaContacts.empty())
	{
		// do not schedule for rediscovery if there is something still in progress
		EV_WARN << "Rediscovery not scheduled for contact " << contactkey.shortStr()
			<< " because it is already in rediscovering state" << endl;
		return;
	}

	EV_TRACE << ((&rediscoverQ == &PNRediscoverQueue) ? "PN" :
		     ((&rediscoverQ == &slowRediscoverQueue) ? "Slow" :
		     ((&rediscoverQ == &regularRediscoverQueue) ? "Regular" :
		      ((&rediscoverQ == &urgentRediscoverQueue) ? "Urgent" : "Unknown"))))
	         << " Rediscovery scheduled for contact " << contactkey.shortStr() << endl;

	rediscoverQ.push(contactkey);
	// remember where it was queued in for next retry round
	RediscoveryState::rediscoverQ_type rediscoverQ_type((&rediscoverQ == &PNRediscoverQueue) ? RediscoveryState::rediscoverQ_type::physicalneighbors :
							    ((&rediscoverQ == &urgentRediscoverQueue) ? RediscoveryState::rediscoverQ_type::urgent :
							    ((&rediscoverQ == &regularRediscoverQueue) ? RediscoveryState::rediscoverQ_type::regular : RediscoveryState::rediscoverQ_type::slow)));

	const_cast<KadContact*>(rt_contact)->getRDS().rediscoverQtype= rediscoverQ_type;

	// start timer for calling rediscoverContacts unless it has been started already
	if (not rediscoverQ.isScheduled() and not rediscoverQ.empty()) {
		//EV_TRACE << "addRandomizedTimer(): rediscoveryWaitTime =" << rediscoveryWaitTime << endl;
		addRandomizedTimer(uniform(rediscoveryWaitTime/2,rediscoveryWaitTime*1.5), 0, std::bind(&R2Kademlia::rediscoverContacts, this, rediscoverQ_type, rediscoveryWaitTime));
		rediscoverQ.setScheduled();
	}
}


void R2Kademlia::rediscoverContacts(RediscoveryState::rediscoverQ_type qtype, simtime_t rediscoveryWaitTime) {
	// traverse list of scheduled contacts for rediscovery
	// the list is sorted by ID-wise vicinity, because contacts in lower buckets are more important
	// recheck status and set to rediscover
	rediscoverQueueClass& rediscoverQ( (qtype==RediscoveryState::rediscoverQ_type::physicalneighbors ? PNRediscoverQueue :
					    (qtype==RediscoveryState::rediscoverQ_type::urgent ? urgentRediscoverQueue :
					     (qtype==RediscoveryState::rediscoverQ_type::regular ? regularRediscoverQueue :
					      slowRediscoverQueue))) );

	EV_TRACE << ((&rediscoverQ == &PNRediscoverQueue) ? "PN" :
		     ((&rediscoverQ == &slowRediscoverQueue) ? "slow" :
		     ((&rediscoverQ == &regularRediscoverQueue) ? "regular" :
		      ((&rediscoverQ == &urgentRediscoverQueue) ? "urgent" : "unknown"))))
	         << " rediscoverContacts called, queued: " << rediscoverQ.size() << endl;
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
				if (rt_contact->isPhysicalNeighbor() and rt_contact->getNodeDegree()==1) {
					// PN has lost its only link to us -> not useful for rediscovering
					rediscoverQ.pop();
					continue;
				}

				// set contact state to rediscovering
				if (not rt_contact->isRediscovering())
					rt_contact->setRediscovering();

				// it is only useful to store node ids, because later on, those contacts may have been gone already
				// collect still valid closest contacts to the failed contact
				auto contacts = routingTable.getClosestContacts(contactkey, rediscoveryMaxNotification, true);
				for (auto c : contacts) {
					if (c != nullptr) {
						rt_contact->addRediscoverVia(c->getKey());
					}
				}

				EV_TRACE << "rediscoverContacts(): number of via contacts: " << contacts.size() << endl;
				// on first rediscovery attempt: immediate rediscovery, retries: after waiting for a backoff time
				if (rt_contact->getRDS().retryCounter == 0 and contacts.size() > 0) {
					sendRediscovery(rt_contact->getKey(), rediscoveryFanOutFactor, rediscoveryWaitTime);
					sent++;
				}
				else {
					// repeated rediscovery attempt: add backoff waiting time
					// schedule rediscovery for later (max backoffFactor arbitrarily set to 1024)
					unsigned long backoffFactor(1UL << (rt_contact->getRDS().retryCounter < 10 ? rt_contact->getRDS().retryCounter : 10));
					unsigned long period_in_ms= 100*backoffFactor;
					unsigned long scatter = uniform(0, period_in_ms);

					EV_TRACE << "rediscoverContacts(): will sendRediscovery in " << period_in_ms/2+scatter << "ms, retry:" << rt_contact->getRDS().retryCounter << endl;
					addRandomizedTimer(SimTime(period_in_ms/2+scatter,SIMTIME_MS), 0, std::bind(&R2Kademlia::sendRediscovery, this, rt_contact->getKey(), rediscoveryFanOutFactor, rediscoveryWaitTime));
					sent++;
				}
			}
		}
		// sendRediscovery for contact already called (or will be called soon),
		// so remove top element from queue
		rediscoverQ.pop();
	} // end while

	// reschedule next call to rediscover next batch of failed contacts
	// if rediscoveryQ is now empty, next call of this procedure will be scheduled on demand
	if (not rediscoverQ.empty()) {
		if (qtype==RediscoveryState::rediscoverQ_type::urgent or qtype==RediscoveryState::rediscoverQ_type::physicalneighbors) {
			addRandomizedTimer(uniform(rediscoveryWaitTime/4,rediscoveryWaitTime*3/4), 0, std::bind(&R2Kademlia::rediscoverContacts, this, qtype, rediscoveryWaitTime));
		}
		else {
			addRandomizedTimer(uniform(rediscoveryWaitTime/2,rediscoveryWaitTime*1.5), 0, std::bind(&R2Kademlia::rediscoverContacts, this, qtype, rediscoveryWaitTime));
		}
		rediscoverQ.setScheduled();
	}
	else { // queue is now empty, so reset scheduled flag
		rediscoverQ.clearScheduled();
	}
}


// send a FindNodeReq message for rediscovery of a given contact
// sends a find node call for rediscovery, possibly using "not via" failed link,
// maxFanOut determines how many messages are sent in parallel via different contacts
// last two parameters required for scheduling retries
void R2Kademlia::sendRediscovery(OverlayKey failedcontactkey, uint8_t maxFanOut, simtime_t rediscoveryWaitTime) {
	//EV_TRACE << "sendRediscovery called for " << failedcontactkey.shortStr() << endl;

	KadContact* failedcontact= const_cast<KadContact*>(routingTable.getContact(failedcontactkey));
	// do nothing if contact has been deleted or has been rediscovered successfully meanwhile
	if (failedcontact == nullptr or not failedcontact->isRediscovering())
		return;
	// contact still exists
	// get the correct queue object
	RediscoveryState::rediscoverQ_type qtype(failedcontact->getRDS().rediscoverQtype);
	rediscoverQueueClass& rediscoverQ( (qtype==RediscoveryState::rediscoverQ_type::physicalneighbors ? PNRediscoverQueue :
					    (qtype==RediscoveryState::rediscoverQ_type::urgent ? urgentRediscoverQueue :
					    (qtype==RediscoveryState::rediscoverQ_type::regular ? regularRediscoverQueue :
					     slowRediscoverQueue))) );

	EV_TRACE << "sendRediscovery for " << failedcontact->getKey().shortStr()
		 << " (" << ((&rediscoverQ == &PNRediscoverQueue) ? "PN" :
			     ((&rediscoverQ == &slowRediscoverQueue) ? "slow" :
			     ((&rediscoverQ == &regularRediscoverQueue) ? "regular" :
			      ((&rediscoverQ == &urgentRediscoverQueue) ? "urgent" : "unknown"))))
		 << ')' << endl;

	if (auto& via_contacts= failedcontact->getRDS().viaContacts; via_contacts.size() > 0)
	{
		// use several contacts to send multiple Rediscovery requests in parallel
		for (auto fanout= 0; fanout < maxFanOut and via_contacts.size() > 0; fanout++)
		{
			// must check for existence and validity of contact
			if (auto contact= routingTable.getContact(via_contacts.front()); contact != nullptr and contact->isValid())
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
			// remove the used via contact from the list
			via_contacts.pop_front();
		}
	}
	else
	{ // no (more) contacts to choose from (initially only possible for isolated nodes)

		EV_WARN << "no (more) via contacts found, cannot send rediscovery" << endl;
		// this may be triggered by an answer arriving later than the first one that already hit this condition here
		// so check whether contact was already scheduled
		if (std::find(rediscoverQ.begin(),rediscoverQ.end(),failedcontactkey) != rediscoverQ.end()) {
			if (rediscoverQ.isScheduled()) {
				EV_TRACE << "already rescheduled for rediscovery" << endl;
				return;
			}
		}

		// try several rounds of rediscovery for contacts
		if (failedcontact->getRDS().retryCounter < maxRediscoveryRetries) {
			failedcontact->getRDS().retryCounter++;
			EV_TRACE << "will retry rediscovery for contact " << failedcontact->getKey().shortStr() << " retry: " << failedcontact->getRDS().retryCounter << endl;
			// via contacts will be refilled in rediscoverContacts
			scheduleRediscovery(failedcontact->getKey(), rediscoverQ, rediscoveryWaitTime);
		}
		else {
			// do not remove physical contacts
			if (failedcontact->isInvalid() and failedcontact->isPhysicalNeighbor() == false) {
				EV_TRACE << "rediscovery finally failed, removing contact " << failedcontact->getKey().shortStr() << endl;
				failedcontact->setState(KadContact::State::dead);
				routingTable.removeContact(failedcontact);
			}
		}
	}
}

///
/// LinkFailureNotifications
///

void
R2Kademlia::scheduleLinkFailureNotification() {

	if ( not linkFailureNotificationScheduled ) {
		addRandomizedTimer(uniform(SimTime(notifyLinkFailureWaitTime/2,SIMTIME_MS),
					   SimTime(notifyLinkFailureWaitTime*1.5,SIMTIME_MS)), 0, std::bind(&R2Kademlia::sendLocalLinkFailureNotifications, this));
		linkFailureNotificationScheduled= true;
	}
}


void
R2Kademlia::sendLinkFailureNotifications(const NotViaList_t& notvialist, const std::unordered_set<PathID>& notifyPIDList, OverlayKey excludeNode)
{
	EV_TRACE << "sendLinkFailureNotifications, notifyPIDList " << notifyPIDList.size() << " entries, { ";
	for (auto pid : notifyPIDList)
		EV_TRACE << pid << ' ';
	EV_TRACE << '}' << endl;
	if (notifyPIDList.empty()) {
		linkFailureNotificationScheduled= false;
		return;
	}


	// iterate over all active/intact direct underlay neighbors
	for (auto ntcit= neighborTable.getTable().cbegin(); ntcit!=neighborTable.getTable().cend(); ntcit++) {

		if (ntcit->first == excludeNode) // exclude this neighbor (usually the source of a previous flooding message)
			continue;

		const auto ifitmapcit(incomingIFtoPIDmap.find(ntcit->first));
		if ( ifitmapcit == incomingIFtoPIDmap.cend() )
			continue;

		// only send PIDs to this neighbor that are requested by this neighbor
		std::list<PathID> filteredPIDList;
		for (auto pidcit = notifyPIDList.cbegin(); pidcit != notifyPIDList.cend(); pidcit++) {
			if ( ifitmapcit->second.find(*pidcit) != ifitmapcit->second.cend() )
				filteredPIDList.push_back(*pidcit);
		}

		if (filteredPIDList.empty()) {
			EV_TRACE << "skipped neighbor " << ntcit->first.shortStr() << " in KadRevPathFlood" << endl;
			continue;
		}

		auto packet = makePacket<KadRevPathFlood>(routingTable.getContact(ntcit->first), "LinkFailureNotificationFlood");
		packet->setPIDListArraySize(filteredPIDList.size());

		auto pidcit(filteredPIDList.cbegin());
		for (unsigned long i= 0; i < filteredPIDList.size(); i++) {
			packet->setPIDList(i,*pidcit);
			pidcit++;
		}

		packet->setNotVia(true);
		packet->setFailedLinkArraySize(notvialist.size());
		packet->setFailedLinkAgeArraySize(notvialist.size());
		unsigned long i= 0UL;
		for ( auto& notvialinkinfo : notvialist ) {
			// add the failed links
			packet->setFailedLink(i,notvialinkinfo.first);
			packet->setFailedLinkAge(i,notvialinkinfo.second);
			i++;
		}

		sendPacket(std::move(packet));
	} // end for every underlay neighbor

	// wait for PN discovery to complete before sending these out
	addOneShotTimer(uniform(3*PNRediscoveryWaitTime/2,9*PNRediscoveryWaitTime/2),
			std::bind(&R2Kademlia::doNotifyFirstSegmentSources, this, notifyPIDList, notvialist));

	linkFailureNotificationScheduled= false;
}


// find out which PathIDs are affected by the link failure and send notifications
// note that this function may be called multiple times for each failed link
// therefore it is useful to schedule the sending of the notification
void
R2Kademlia::notifyAffectedPathUsers(const OverlayKey& nexthop_key) {

	EV_TRACE << "notifyAffectedPathUsers()" << endl;
	// traverse the complete PIDFwdTable and check for incoming PIDs that use the outgoing link
	for (auto it= fwdTablePathID.PIDFwdTable.cbegin(); it != fwdTablePathID.PIDFwdTable.cend(); it++) {
		if (it->second.nextHopNode == nexthop_key and
			it->second.isUsed() > 0) { // for debugging only
			// TODO should be replaced by .getAge() < 2*PathRefreshThreshold)
			notifyPIDs.insert(it->first);
		}
	}
	EV_DEBUG << "found " << notifyPIDs.size() << " affected PathIDs (total)" << endl;
	EV_TRACE << "notifyPIDs " << notifyPIDs.size() << " entries, { ";
	for (auto pid : notifyPIDs)
		EV_TRACE << pid << ' ';
	EV_TRACE << '}' << endl;
	// schedule the sending of notifications if not already scheduled
	scheduleLinkFailureNotification();
}


// this is a non-RPC flooding message, forwarded hop-by-hop in the underlay
void R2Kademlia::handlePacket(std::unique_ptr<KadRevPathFlood> packet) {
	// NotVia information has been processed before this method is called
	// Therefore, only the PIDs have to be looked up and it has to be figured out
	// which PIDs need to be propagated further "upstream"
	EV_TRACE << "received KadRevPathFlood from " << packet->getSource().shortStr() << endl;
	std::unordered_set<PathID> upstreamPIDs;
	auto numPIDs= packet->getPIDListArraySize();
	EV_TRACE << "PIDList: { ";
	for (unsigned long i= 0; i < numPIDs; i++) {
		EV_TRACE << packet->getPIDList(i) << ' ';
	}
	EV_TRACE << '}' << endl;

	// iterate over all forwarding table entries to find any upstream PIDs
	for (auto it= fwdTablePathID.PIDFwdTable.cbegin(); it != fwdTablePathID.PIDFwdTable.cend(); it++) {
		if (it->second.outgoingPathID != PathID::UndefinedPathID) { // only if outgoing PID is used
			for (unsigned long i= 0; i < numPIDs; i++) {
				// only consider actually used PathIDs: they need periodic refreshes
				if (it->second.outgoingPathID == packet->getPIDList(i)
				    and it->second.isUsed()) { // for debugging only
					// TODO should be replaced by .getAge() < 2*PathRefreshThreshold
					upstreamPIDs.insert(it->first);
				}
			} // end for inner loop
		}
	}

	// just copy the notvia list for passing it on
	NotViaList_t notvialist;
	if (packet->getNotVia())
	{
		for (uint i= 0; i < packet->getFailedLinkArraySize(); i++) {
			notvialist.push_back(std::make_pair(packet->getFailedLink(i), packet->getFailedLinkAge(i)));
		}
	}

	if (not upstreamPIDs.empty()) {
		EV_TRACE << "sendLinkFailureNotifications for upstream PIDs { ";
		for (const auto& pid : upstreamPIDs)
			EV_TRACE << pid << ' ';
		EV_TRACE << '}' << endl;

		// exclude the packet source for further flood messages
		sendLinkFailureNotifications(notvialist, upstreamPIDs, packet->getSource());
	}
	else {
		EV_TRACE << "not distributing linkFailureNotification any further" << endl;
	}

	// now check for nodes (sources of the first segment) to be notified
	addOneShotTimer(uniform(3*PNRediscoveryWaitTime/2,9*PNRediscoveryWaitTime/2),
			std::bind(&R2Kademlia::doNotifyFirstSegmentSources, this, upstreamPIDs, notvialist));
}


void
R2Kademlia::doNotifyFirstSegmentSources(const std::unordered_set<PathID>& upstreamPIDs, const NotViaList_t& notvialist)
{
	// structure to collect all nodes to be notified (there may be many duplicates)
	std::unordered_set<OverlayKey> nodestonotify;
	// now check for nodes (sources of the first segment) to be notified
	for (const auto& incomingPID : upstreamPIDs) {
		if (const auto it= notifyFirstSegmentSources.find(incomingPID);
		    it != notifyFirstSegmentSources.cend()) {
			EV_TRACE << "check for notifyFirstSegmentSources: PID " << it->first << " #sources=" << it->second.size() << endl;

			// entry for PID found, so notify all first segment sources with an update
			for (const auto &notify_target : it->second) {
				nodestonotify.insert(notify_target);
			} // end for all sources to notify
		} // end if there is an entry for this PID
	} // end for all PIDs to check

	for (const auto &notify_target : nodestonotify) {
		size_t contactsLen = std::max((size_t)findNodeNumber, routingTable.getSplitSize());
		KadContactList closeContacts(routingTable.getClosestContacts(notify_target, contactsLen, true, notvialist));
		if (not closeContacts.empty()) {
			const KadContact& next_contact = chooseNextHop(closeContacts, notify_target);
			// check whether that contact is closer to the target than we are (but do not send back to source)
			if ( next_contact.isValid() and (next_contact.getKey() ^ myOverlayKey) > (next_contact.getKey() ^ notify_target) )
			{
				auto notify_packet = makePacket<KadLinkFailureNotify>(next_contact, "LinkFailureNotifyDirect");
				notify_packet->setDstNodeID(notify_target);
				// copy notvia list
				notify_packet->setFailedLinkArraySize(notvialist.size());
				notify_packet->setFailedLinkAgeArraySize(notvialist.size());
				auto it(notvialist.cbegin());
				for (uint i= 0; i < notvialist.size(); i++) {
					notify_packet->setFailedLink(i,it->first);
					notify_packet->setFailedLinkAge(i, it->second);
					it++;
				}
				//ASSERT(notvialist.size() > 0);
				notify_packet->setNotVia(notvialist.size() > 0);

				EV_TRACE << "Sending LinkFailureNotify to " << notify_target.shortStr() << endl;
				sendPacket(std::move(notify_packet));
			}
		}
	} // end for all nodes to be notified
}


// non-RPC notification message that is forwarded via the normal overlay routing
// no need for reliability of this message
void R2Kademlia::handlePacket(std::unique_ptr<KadLinkFailureNotify> packet) {

	// when this method is called either we are an overlay hop or the final recipient
	OverlayKey& target = packet->getDstNodeID(); // get the destination ID

	EV_TRACE << "KadLinkFailureNotify to " << target.shortStr() << " (id=" << packet->getId() << ")" << endl;
	if (target == myOverlayKey) { // notification is for this node and ends here
		EV_TRACE << "KadLinkFailureNotify reached final destination " << myOverlayKey.shortStr() << endl;
		// nothing to do, just drop the packet
		return;
	}

	// forward via overlay routing
	size_t contactsLen = std::max((size_t)findNodeNumber, routingTable.getSplitSize());
	// construct notvia list for filtering contacts
	NotViaList_t notvialist;
	if (packet->getNotVia())
	{
		for (uint i= 0; i < packet->getFailedLinkArraySize(); i++) {
			notvialist.push_back(std::make_pair(packet->getFailedLink(i), packet->getFailedLinkAge(i)));
		}
	}
	// ContactList is sorted according to XOR metric with respect to key
	// the first contact is ID-wise closest to query key, however, it may not be the one with the shortest path
	KadContactList contacts = routingTable.getClosestContacts(target, contactsLen, true, notvialist);

	// remove source from the list of potential next hops to avoid loop or exclude it from answer
	if (!contacts.empty()) {
		// EV_TRACE << "Found contacts: ";
		// for (const auto& thiscontact : contacts)
		// {
		//  	EV_TRACE << thiscontact->getKey().shortStr() << " ";
		// }
		// EV_TRACE << endl;

		for (auto it= contacts.begin(); it != contacts.end(); it++) {
			if (*it != nullptr and (*it)->getKey() == packet->getSource()) {
				contacts.erase(it);
				EV_TRACE << "removed " << packet->getSource() << " from found contacts" << endl;
				break;
			}
		}
	}

	if (contacts.empty() or contacts.front()->getKey().isUnspecified())
	{
		EV_ERROR << "no contacts left, dead end." << endl;

		return;
	}

	// precondition here: !contacts.empty(), i.e., we have a contact as potential next hop
	// if next contact is closer to destination than us, it's ok unless we route back to the source
	if ((contacts.front()->getKey() ^ target) < (myOverlayKey ^ target)
	    and contacts.front()->getPathLength() < packet->getHopLimit()) {

		// we have found a contact that is closer than us to target (query)
		// => forward request
		// note: chooseNextHop may choose the next hop with the shortest path (proximity routing)
		const KadContact& contact = chooseNextHop(contacts, target);
		EV_TRACE << "KadLinkFailureNotify destined for: " << packet->getDstNodeID().shortStr() << " next overlay hop is " << contact << endl;
		// note: the log_2 will be smaller for a longer common prefix
		ASSERT((contact.getKey() ^ target).log_2() <= (myOverlayKey ^ target).log_2());
		if((contact.getKey() ^ target).log_2() > (myOverlayKey ^ target).log_2()) {
			EV_WARN << "no prefix progress made" << endl;
		}
		ASSERT((contact.getKey() ^ target) < (myOverlayKey ^ target));
		packet->appendRoute(contact);
		ASSERT(packet->getRouteArraySize() - packet->getProgress() <= packet->getHopLimit());
		resendPacket(std::move(packet));
		return;
	}
	else {
		// otherwise it's ok to report what we know
		EV_TRACE << "Cannot get closer to the target, best neighbor now: " << contacts.front()->getKey() << endl;
		EV_TRACE << "Distance for contact=" << (contacts.front()->getKey() ^ target).log_2()
			 << " own distance=" << (myOverlayKey ^ target).log_2() << " own ID: " << myOverlayKey.shortStr() << endl;
		return;
	} // end else
}


} // end namespace
