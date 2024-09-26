
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
		id = parent.nodeId;
	};

	mobility_model : MobilityBase {
		  @dynamic(true);
	}

	antenna_model : AntennaBase {
		  @dynamic(true);
	}

connections:
	for i = 0..sizeof(toNeighbors)-1 {
		toNeighbors[i] <--> node.toNeighbors++;
	}
}
