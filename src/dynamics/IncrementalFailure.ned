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

package routingsim.dynamics;

import routingsim.foundation.NetworkDynamics;

/// This module implements an incremental failure model to, e.g., 
/// track the performance of a routing protocol in a consistently
/// degrading environment. 
/// Can be configured to fail links, nodes, or both.
///
/// Default behaviour is to fail 1% of all nodes each 200s after warmup of 100s
///
/// @author Christoph Werle <werle@kit.edu>
///
simple IncrementalFailure extends NetworkDynamics {
parameters:
    @class(IncrementalFailure);
    
    /// have NodeFraction nodes fail each period
	bool NodeFailure = default(true);
	
	/// fail LinkFraction links each period
	bool LinkFailure = default(false);
	    			
    /// fraction of nodes that fail at the beginning of each period
   	double NodeFraction = default(0.01);
   	
    /// fraction of links that fail at the beginning of each period
   	double LinkFraction = default(0.01);
   	
   	/// warmup phase after which failures will begin to occur
   	double warmup @unit(s) = default(100s);
   		
   	/// periodic incremental occurence of failures starting after warmup
    double period @unit(s) = default(200s);		
}
