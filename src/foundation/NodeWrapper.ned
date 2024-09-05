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

module NodeWrapper {
parameters:
	@display("i=block/join");
	/// node id, -1 = choose own id
	int nodeId = default(-1);

gates:
	inout toNeighbors[];

submodules:
	// this will be dynamically created by NetworkTopology
	node: Node {
             //@dynamic(true);
             id = parent.nodeId;
	};

	// this will be dynamically created by NetworkTopology
	protocol0 : Protocol {
		  @dynamic(true);
	};

connections:
	for i = 0..sizeof(toNeighbors)-1 {
		toNeighbors[i] <--> node.toNeighbors++;
	}
}
