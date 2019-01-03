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

package routingsim.routing.evrr;

//
// TODO auto-generated module
//
simple VirtualRing extends routingsim.foundation.Protocol {
parameters:
    @class(VirtualRing);
    @display("i=abstract/router");
    
    /// length of a round
    double roundTime @unit(s) = default(1s);

    /// rounds until the routes are checked for consistency
    /// 0rounds = perform no consistency check 
    int consistencyCheck @unit(rounds) = default(60rounds);
    
    /// rounds until unused routing table or blacklist entries are removed
    /// should be greater than the number of rounds of the consistency check
    int timeout @unit(rounds) = default(60rounds);
    
    /// flag, whether eVRR sends updates immediately (for faster convergence)
    bool immediateResponse = default(true);
    
    /// flag, whether each hop needs get closer to the destination
    bool useRouteDescentCriterion = default(false);
    
    /// use linearization of stabilization
    bool useLinearization = default(false);
    
    /// number of parallel discovery messages
    int parallelDiscoveries = default(4);
    
    /// threshold until discoveries and anycast is delayed linearly
    int delayThreshold = default(5);
    
    /// use anycast
    bool useAnycast = default(false);
    
    /// anycast identifiers -- a space separated string of numbers
    string anycast = default("");
    
    /// Delay until probe messages are sent
    int sendMessageDelay @unit(s) = default(1s);
    

/// ----- not supported yet!

	/// virtual identifiers -- a space separated string of numbers
	/// "random" = select one identifier uniformly at random
    string vid = default("random");

   	/// number of virtual ring neighbors
    int vneighbors = default(1);
   
    /// try to adapt the network topology to the virtual ring
    bool useShortcuts = default(false);
}
