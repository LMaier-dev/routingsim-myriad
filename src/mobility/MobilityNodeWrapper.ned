
package routingsim.mobility;

module MobilityNodeWrapper {
parameters:
	@display("i=block/join");
	/// node id, -1 = choose own id
	int nodeId = default(-1);

gates:
	inout toNeighbors[];

submodules:
	node: MobilityNode {
		id = nodeId;
	};

connections:
	for i = 0..sizeof(toNeighbors)-1 {
		toNeighbors[i] <--> node.toNeighbors++;
	}
}
