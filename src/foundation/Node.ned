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

    // minimum and maximum processing delay for control packets in nanoseconds
    int nodeMinControlPlaneDelay @unit(ns) = default(0ns);
    int nodeMaxControlPlaneDelay @unit(ns) = default(500000ns);
    // minimum and maximum processing delay for data packets in nanoseconds
    int nodeMinDataPlaneDelay @unit(ns) = default(0ns);
    int nodeMaxDataPlaneDelay @unit(ns) = default(0ns);

gates:
    input directIn @directIn;
    inout toProtocols[];
    inout toNeighbors[];
}
