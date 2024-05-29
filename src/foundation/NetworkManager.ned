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

import routingsim.mobility.MobilityNode;


/// The network manager comprises a network topology and a connection to 
/// dynamics module.
/// 
/// @author Christoph Werle <werle@kit.edu>
/// @author Sebastian Mies <mies@kit.edu>
///
simple NetworkManager
{
    parameters:
        @display("i=block/network2;is=l");

		//TODO this is not used!
        /// names of the modules inside a node
        string protocols;

	    /// filename of the network topology 
        string topologyFile = default("");

        /// determine the file format of topologyFile
        string topologyFileType @enum(SELF_DEFINED,DIMES) = default("SELF_DEFINED");

        /// extract the largest stronlgy connected component from the graph read?
        bool useOnlyLargestConnectedComponent = default(true);

        /// make any given link bidirectional?
        bool makeLinksBidirectional = default(true);

        /// allow id-remapping (if ids are too high, map them to lower, unused ids)
        /// if ids are larger than std::vector.max_size(), an exception is thrown
        /// may be required for DIMES ASEdges
        bool allowNodeIdRemapping = default(false);

        /// enable compression of the node id space; currently the node ids are used
        /// directly as indices to our bgl graph data structures; therefore high node
        /// ids result in increased memory consumption.
        /// if compression is enabled, the max node ids are sequentially merged into
        /// the lowest available ids
        bool compressNodeIDs = default(false);

        /// filename of the shortest-path pre-calculations
        string spathFile = default("");

        /// determine the way shortest paths are determine; we want an alternative to
        /// calculating the full distance matrix for large graphs, when only sampling
        /// is used.
        string shortestPathCalculationMode @enum(fullDistanceMatrix,onDemand) = default("fullDistanceMatrix");

        /// when dynamics gate is not connected, create a dynamics model by name
        string dynamicsModel @enum("None","CatastrophicFailure","LifetimeChurn","LinkFailure","IncrementalFailure") = default("LinkFailure");

        string topologyStatistics @enum(None,Light,Heavy) = default("Light");

        /// limits bandwidth of the links in network
        bool limitBandwidthLinks = default(true);

        /// bandwidth of the links in network measured in bits per second (only active if limitBandwidthLinks is true)
        double bandwidthLinksBps = default(1e12);

        /// add mobility nodes to the network
        bool mobility = default(false);

        /// number of mobility nodes to add to the network
        int mobilityNodes = default(100);

    gates:
        /// input gate for topoplogy dynamics
        input dynamics @loose;

        /// input gate for mobility dynamics
        input mobilityDynamics @loose;
}

