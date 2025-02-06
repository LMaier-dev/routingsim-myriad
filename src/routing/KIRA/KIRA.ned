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

package routingsim.routing.KIRA;

//
// 
//
simple R2Kademlia extends routingsim.foundation.Protocol
{
    parameters:
		@class(R2Kademlia);
		@display("i=misc/node");
		// for modeling Control Plane processing delay (disabled if maxKIRAControlPlaneProcessingDelay == 0)
		// suggest default average processing delay is 250µs which corresponds to 4000 msgs/s
		int minKIRAControlPlaneProcessingDelay = default(0ns)  @unit(ns);
		//int maxKIRAControlPlaneProcessingDelay = default(500000ns)  @unit(ns);
		int maxKIRAControlPlaneProcessingDelay = default(0ns)  @unit(ns);
		int minKIRADataPlaneProcessingDelay = default(0ns)  @unit(ns);
		int maxKIRADataPlaneProcessingDelay = default(0ns)  @unit(ns);
		int KIRAControlPlaneProcessingQueueDelayLimit = default(1000ms) @unit(ms);
		/// number of bits used for accelerated lookup called "b" in research papers.
		int accelerationFactor = default(1);
		/// max size of the buckets, called "k" in research papers (k-Buckets), for KIRA recommended k=40
		int bucketSize = default(40);
		/// number of contacts returned by find node response, called "k" in research papers
		int findNodeNumber = default(bucketSize);
		/// radius of physical neighborhood discovery (for populating the routing table)
		/// (note that the vicinity discovery radius for PathID precalculation is fixed at 2)
		int discoveryRadius = default(3);
		bool proximityRouting = default(true);
		bool proximityNeighborSelection = default(true);
		/// note: default hopLimit may be too small for topologies with large diameters (safe to increase)
		int hopLimit = default(256);
		/// if activated the node collects path information from passing packets
		bool pathSnooping = default(true);
		bool sendDataPackets = default(true);
		bool usePathIDs = default(true);
		bool useLinkLayerIndication = default(true);
		// enable special extension that also selects paths of equal length to be deterministically "better"
		// see also exploreInterval
		bool enableSinglePathDiversity = default(false);
		// the following is used for discovering diverse paths, turned-off when 0s
		double exploreInterval = default(0s) @unit(s);
		double probeInterval = default(2s) @unit(s);
		/// strategy for probing randomly
		string probeStrategy @enum(none,randomID,existingID,bucketExpand,randomSuffix,suffixExpand,randomBucket,randomIDandBucketMixed) = default("randomIDandBucketMixed");
		// minHelloInterval
		double minHelloInterval = default(200ms) @unit(ms);
		// maxHelloInterval
		double maxHelloInterval = default(30000ms) @unit(ms);
		/// waiting time until PN rediscovery is scheduled
		double PNRediscoveryWaitTime = default(100ms) @unit(ms);
		/// waiting time until urgent rediscovery is scheduled
		double urgentRediscoveryWaitTime = default(300ms) @unit(ms);
		/// waiting time until rediscovery is scheduled
		double regularRediscoveryWaitTime = default(1000ms) @unit(ms);
		/// waiting time until rediscovery is scheduled
		double slowRediscoveryWaitTime = default(2000ms) @unit(ms);
		/// waiting time until scheduled urgent routing updates are sent
		double updateHoldTimeFast = default(250ms) @unit(ms);
		/// waiting time until scheduled routing updates are sent
		double updateHoldTime = default(500ms) @unit(ms);
		/// hold time until path setup messages are sent
		double pathSetupHoldTime = default(100ms) @unit(ms);

		/// maxFanOut for rediscovery
		int rediscoveryFanOutFactor = default(2);
		/// rediscoveryMaxNotification
		int rediscoveryMaxNotification = default(findNodeNumber);
		/// updateMaxNotification
		int updateMaxNotification = default(4);
		/// updateNotifyStrategy 1=close to own key, 2=close to affected key
		int updateNotifyStrategy = default(1);
		/// updatePropagateStrategy 0=none, 1=KAD_Unreachable, 2=KAD_Announce, 3=KAD_Change, 4 =KAD_Announce+KAD_Change, 5=all
		int updatePropagateStrategy = default(1);
		/// sendLinkFailureNotifications (none = 0, floodonly= 1, flood_and_directPID= 2, full= 3)
		int linkFailureNotifications = default(0);
		/// if failureFocussedProbing true: send reachability probes to affected nodes during failures (unless isolated),
		/// otherwise choose nodes randomly
		bool failureFocussedProbing = default(true);
		/// add additional randomly selected contacts from various buckets in responses (usually 2 per bucket)
		bool addGratuitousContacts = default(true);

		/// Numer of tries when searching a destination for a probe packet
		int maxReachablityTries = default(10);

		/// in which interval a node sends a floodPacket (0s is disabled)
		double floodInterval = default(0s) @unit(s);
		/// max stride factor for flooding packets
		int floodStrideFactor = default(0);
		/// whether to count trailing non-full buckets as part of the responsibility range
		bool floodCompleteBucketOpt = default(true);

		/// send flood packet on link failure
		bool floodOnFailure = default(false);

		// statistics
		/// if true records average path lengths and stretch in routing tables (can be costly)
		bool detailedRoutingTableStats = default(true);
		/// if true records per KIRA message type statistics over all nodes
		bool detailedKIRAMessageStats = default(false);
}
