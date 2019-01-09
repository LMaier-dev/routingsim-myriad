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

/// This module implements a distributed churn dynamics model.
/// It supports several distribution functions, such as weibull, pareto,
/// and truncnormal.
///
/// @author Sebastian Mies <mies@kit.edu>
///
simple LifetimeChurn extends NetworkDynamics {
parameters:
    @class(LifetimeChurn);

    /// min. degree of nodes that fail, default are bigger routers (>=3)
    int minDegree = default(3);
    			 
    /// max. degree of nodes that fail, 0=infinite
    int maxDegree = default(0);
    
    /// distribution function
    string distribution @enum("Weibull", "ParetoShifted","TruncNormal") = 
    	default("Weibull");
 
    /// mean node lifetime in seconds, default 
	///double meanLifetime @unit(s) = default(10000s); 
   	double meanLifetime @unit(s) = default(10000s); 
   	
   	/// parameter for the distribution function
    double distParameter = default(0.5); 
}
