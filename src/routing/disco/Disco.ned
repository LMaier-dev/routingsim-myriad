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
// along with this program.  If not, see http://www.gnu.org/licenses/
// 

package routingsim.routing.disco;


//
// TODO auto-generated module
//
//import routingsim.foundation.Node;

simple Disco extends routingsim.foundation.Protocol {
parameters:
 	@class(Disco);
 	
 	bool to_Destination_Shortcutting = default(true);
 	bool no_Path_Knowledge_Shortcutting = default(true);
 	bool up_Down_Stream_Shortcutting = default(true);
 	
 	bool landmark = default(false);
 	bool extractLandmarksFromTopologyFile = default(true);
 	
 	// probe data packet configuration
	double probingNodeFraction = default(1);
	double probePacketDestinationFraction = default(1); 	
}
