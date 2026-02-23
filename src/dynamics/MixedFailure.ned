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

// The MixedFailure Model enables simulation of node and link failures in parallel.
// The period in which failures occur, the duration of failures and a jitter can be
// defined independently for link and node failures.
simple MixedFailure extends NetworkDynamics {
parameters:
    @class(MixedFailure);
    
    /// warmup phase (failures start after warmup)
    double warmup @unit(s) = default(100s);
    
    /// whether nodes should fail
	bool nodeFailure = default(true);
	/// fraction of nodes that fail at the beginning of each interval
    double nodeFraction = default(0.01);
    /// periodic occurrence of node failures
    double nodeFailPeriod @unit(s) = default(320s);
    /// duration of each node failure
    double nodeFailDuration @unit(s) = default(480s);
    /// jitter +/- for nodeFailDuration
    double nodeFailDurationJitter @unit(s) = default(0);
    
    
    /// whether links should fail
	bool linkFailure = default(true);
	/// fraction of links that fail at the beginning of each interval
    double linkFraction = default(0.01);
    /// periodic occurrence of link failures
    double linkFailPeriod @unit(s) = default(30s);
    /// duration of each link failure
    double linkFailDuration @unit(s) = default(60s);
    /// jitter +/- for linkFailDuration
    double linkFailDurationJitter @unit(s) = default(0);
}
