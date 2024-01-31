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

package routingsim.foundation;

simple Node {
parameters:
    @display("i=block/join");

    /// node id, -1 = choose own id
    int id = default(-1);

    /// protocols to the node uses
    string protocols;

    // minimum and maximum delay in milliseconds
    int nodeMinDelay @unit(us) = default(0us);
    int nodeMaxDelay @unit(us) = default(500us);

gates:
    input directIn @directIn;
    inout toProtocols[];
    inout toNeighbors[];
}
