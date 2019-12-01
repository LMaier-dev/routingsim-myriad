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

package routingsim.routing.r2kademlia;

//
// 
//
simple R2Kademlia extends routingsim.foundation.Protocol
{
    parameters:
		@class(R2Kademlia);
		@display("i=misc/node");
		/// number of bits used for accelerated lookup called "b" in research papers.
		int accelerationFactor = default(1);
		/// max size of the buckets, called "k" in research papers (k-Buckets)
		int bucketSize = default(20);
		/// number of contacts returned by find node response, called "k" in research papers
		int findNodeNumber = default(bucketSize);
		/// radius of physical neighborhood discovery
		int discoveryRadius = default(2);
		bool proximityRouting = default(true);
		bool proximityNeighborSelection = default(true);
		int hopLimit = default(256);
		/// if activated the node collects path information from passing packets
		bool pathSnooping = default(true);
		bool sendDataPackets = default(true);
		bool pathSimplification = default(true);
		double probeInterval = default(2s) @unit(s);
		/// 0=none, 1=random, 2=exiting, 3=bucket_expand, 4=random_suffix, 5=suffix_expand, 6=random_bucket
		int probeStrategy @enum(ProbeStrategy) = default(3);
		/// waiting time until rediscovery is scheduled
		double rediscoveryWaitTime = default(200ms) @unit(ms);
		/// waiting time until routing updates are scheduled
		double updateWaitTime = default(100ms) @unit(ms);

		// statistics foo
		/// if true records avarage path lengths and stretch in routing tables.
		bool detailedRoutingTableStats = default(true);
}
