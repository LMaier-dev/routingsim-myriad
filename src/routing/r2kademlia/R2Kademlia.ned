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
		/// radius of physical neighborhood discovery
		int discoveryRadius = default(4);
		/// number of contacts returned by find node response, called "k" in research papers
		int findNodeNumber = default(20);
		bool proximityRouting = default(true);
		bool proximityNeighborSelection = default(true);
		int hopLimit = default(1000);
		bool collectPathsFromPassingPackets = default(true);
}
