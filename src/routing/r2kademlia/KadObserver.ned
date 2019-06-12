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
simple KadObserver
{
    parameters:
        /// if true, histograms and averages are reset each sampling interval
        bool resetAfterSample = default(true);
        /// sets the time when message recording should start
        double messageSampleTime = default(200s) @unit(s);
        /// sets the max number of message delivery infos to be recorded
        int messageDeliverySample = default(2000);
        /// sets the max number of message dropped infos to be recorded
        int messageDroppedSample = default(500);
        /// number of nodes for node scalar sampling
        int nodeSample = default(-1);
        /// number of nodes for routing table scalar sampling
        int routingTableSample = default(-1);
}
