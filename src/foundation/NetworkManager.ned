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

/// The network manager comprises a network topology and a connection to 
/// dynamics module.
/// 
/// @author Christoph Werle <werle@kit.edu>
/// @author Sebastian Mies <mies@kit.edu>
///
simple NetworkManager {
	parameters:
	    @display("i=block/network2;is=l");

        /// names of the modules inside a node
        string protocols;
	    
	    /// filename of the network topology 
        string topologyFile = default("");
        
        /// filename of the shortest-path pre-calculations
        string spathFile = default("");
        
        /// when dynamics gate is not connected, create a dynamics model by name
        string dynamicsModel @enum(None, CatastrophicFailure, LifetimeChurn) = 
        	default("CatastrophicFailure");
    gates:
        /// input gate for topoplogy dynamics
        input dynamics @loose;
}
