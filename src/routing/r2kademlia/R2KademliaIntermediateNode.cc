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

namespace routingsim {

// returns true if forwarding should stop here
bool
R2Kademlia::intermediateNodeProcessing(const KadPacket& packet) {

	// special actions while forwarding packets
	switch (packet.getType()) {

	case KAD_PathSetupReq:
	{
		// as intermediate node I need to check to path ID installation
		// returns true if path was installed: forward to next hop only if
		// hops left > vicinityDiscoveryRadius+1

		bool return_answer(false);
		if ( checkForPathIDInstallation(packet) ) {
			// PID was locally installed

			EV_TRACE << "KadPathSetupReq: PID for spath "
				 << packet.getSrcPath(packet.getProgress(), (packet.hopsLeft() == (nodeVicinityGraph::vicinityDiscoveryRadius+1)) ? packet.getRouteArraySize() : packet.getRouteArraySize()-nodeVicinityGraph::vicinityDiscoveryRadius-1)
				 << " installed " << endl;
		}
		else { // send back response if path ID already present/installed
			EV_TRACE << "KadPathSetupReq: PID for spath " << packet.getSrcPath(packet.getProgress(), (packet.hopsLeft() == (nodeVicinityGraph::vicinityDiscoveryRadius+1)) ? packet.getRouteArraySize() : packet.getRouteArraySize()-nodeVicinityGraph::vicinityDiscoveryRadius-1)
			     << " already present" << endl;

			if (packet.hopsLeft() <= (nodeVicinityGraph::vicinityDiscoveryRadius+1))
				return_answer= true;
			// we have to forward until three hops before the destination at least
			// to make sure that the path is installed there
		}

	        // always send response if hops to go <= vicinityDiscoveryRadius+1
		// no need to install PID further downstream due to precomputation
		if (packet.hopsLeft() <= nodeVicinityGraph::vicinityDiscoveryRadius+1)
			return_answer= true;

		if (return_answer) {
			// check for path improvements since this returns early here
			if (pathSnooping) {
				kad_path path_of_interest(packet.reversePath(packet.getSrcPath(1, packet.getProgress())));
				if (path_of_interest.size() > 0)
				{
					routingTable.addPath(packet.getSource(), path_of_interest, packet.getStateSeqNo(), 0, packet.getSrcNodeDegree());
				}
			}
			const KadPathSetupReq* setupreq = static_cast<const KadPathSetupReq *>(&packet);
			sendPacket(makeAnswerKadPathSetupRsp(*setupreq));
			return true; // this will delete the original packet
		}
		forwardedPathSetup++;
	}
	break;

	case KAD_PathTeardownReq:
	{
		// as intermediate node I need to check to path ID teardown
		checkForPathTeardown(packet);

		if (packet.getRouteArraySize()-packet.getProgress() <= nodeVicinityGraph::vicinityDiscoveryRadius+2) {
			EV_TRACE << "KadPathTeardown: terminated here, the rest is precomputed" << endl;
			return true;  // this will delete the original packet
		}

		forwardedPathTeardown++;
	}
	break;

	case KAD_PathSetupRsp:
	{
		EV_TRACE << "forwarding PathSetupRsp, origin spath: "
			 << packet.reversePath(packet.getSrcPath()) << endl;
	}
	break;

	case KAD_ProbeReq: // this is for path probing and refreshes of forwarding entries
	{
		const KadProbeReq* probereq= dynamic_cast<const KadProbeReq*>(&packet);
		// usually the following is not necessary, but sometimes
		// PathSetup was scheduled but not performed and a newer seq of a longer
		// path start a path check, resulting in the ProbeReq
		if (usePathIDs and probereq->getPathCheck()) {
			if (checkForPathIDInstallation(packet))
				EV_TRACE << "POST Installed PathID by ProbeReq PathCheck" << endl;
		}
	}
	break;

	case KAD_UpdateRouteReq: // check for unreachable info
		evalUpdateReq(static_cast<const KadUpdateRouteReq*>(&packet));
		break;

	case KAD_QueryRouteRsp: // check for interesting contacts
		evalQueryRouteRsp(static_cast<const KadQueryRouteRsp*>(&packet));
		break;

	case KAD_Error: // check for contact invalidation due to SegmentFailure
		if (auto errorpkt= static_cast<const KadError*>(&packet);
		    errorpkt != nullptr and (errorpkt->getErrorType() == KAD_SegmentFailure))
		{
			KadPacket* original = check_and_cast<KadPacket *>(errorpkt->getEncapsulatedPacket());
			// find out the failed link
			auto reportingNode(errorpkt->getSource());
			const auto originalPath(original->getSrcPath());
			// use progress in original path to find the correct link
			// just finding the reporting node in the source path isn't safe due to cycles in forwarding path!
			short unsigned int index(original->getProgress());
			// the following should be true, otherwise we are getting
			// an error from some off path node
			ASSERT(originalPath[index] == reportingNode and index+1U < originalPath.size());
			// the next hop is the one that failed
			if (index+1U < originalPath.size()) {
				auto failedNextHop = originalPath[index+1];
				link_t failedlink(reportingNode, failedNextHop);
				EV_INFO << "Check for invalid contacts because of failed link " << failedlink << " in error packet" << endl;
				// this segment or node error is current and should invalidate all affected contacts
				// a rediscovery will be triggered automatically if necessary
				routingTable.invalidateContacts(failedlink, (simTime()-errorpkt->getCreationTime()).inUnit(SIMTIME_MS));
			}
		}
		break;
		
	} // end switch

	if (pathSnooping) {
		// aka PATH OVERHEARING
		//TODO optimization use isPathInteresting
		
		// one can only trust the so far traversed path to work and exist
		// this is especially important for rediscovery messages: they may use still somewhat obsolete path information
		// also: avoid learning about supposed physical Neighbors
		kad_path path_of_interest(packet.reversePath(packet.getSrcPath(1, packet.getProgress())));
		if (path_of_interest.size() > 0)
		{
			routingTable.addPath(packet.getSource(), path_of_interest, packet.getStateSeqNo(), 0, packet.getSrcNodeDegree());
		}

		// NOTE: the rest of the source routing path should not be used for improvement
		// we will probably get the path back in the response message
	}

	return false;
}

} // end namespace
