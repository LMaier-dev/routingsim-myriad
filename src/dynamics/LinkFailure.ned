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


/// Fail a fixed amount of links.
/// Most other models consider only relative amounts of failures
simple LinkFailure extends NetworkDynamics {
parameters:
    @class(LinkFailure);

    /// number of links that should be failed
    int numLinksFail = 3;

    /// fraction of links that fail at the beginning of each interval
   double linkFraction = default(0.01);


    /// periodic occurence of failure
    double period @unit(s) = default(200s);
    	
    /// duration of failure after period start
    double duration @unit(s) = default(100s);
    
    /// Kind of links to fail
    /// None: Deactivates failures
    /// highestDegree: selects the numLinksFail nodes with the highest Degree
    /// degreeRange: selects numLinksFail links connected to a node with degree in the range [minDegree, maxDegree]
    /// random: random selection of nodes
    /// randomAllButOne: random selection of node, removes all links but one if possible, but only if graph stays connected
    string linkSelection @enum("none", "highestDegree", "degreeRange", "random", "randomAllButOne") = default("none");

	/// string with specific edges that should fail
	/// format: x1:y1;x2:y2 edges separated with ';', nodes of edges separated with ':'
	string specificLinks;
    
    int minDegree = 4;
    int maxDegree = 10;
}
