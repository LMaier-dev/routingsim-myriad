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

/// This module implements a catastrophic failure dynamics model.
///
/// @author Christoph Werle <werle@kit.edu>
/// @author Sebastian Mies <mies@kit.edu>
///
simple CatastrophicFailure extends NetworkDynamics {
parameters:
    @class(CatastrophicFailure);
    
    /// min. degree of nodes that fail, default are bigger routers (>=3)
    int minDegree = default(3);
    			 
    /// max. degree of nodes that fail, 0=infinite
    int maxDegree = default(0);
    			
    /// fraction of nodes that fail
   	double fraction = default(0.05);
   	
   	/// warmup phase
   	double warmup @unit(s) = default(100s);
   		
   	/// periodic occurence of failure
    double period @unit(s) = default(200s);
    	
    /// duration of failure on period start
    double duration @unit(s) = default(100s);		
}
