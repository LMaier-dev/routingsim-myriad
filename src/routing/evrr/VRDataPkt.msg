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
cplusplus {{
#include "foundation.h"
}}

namespace routingsim;

class noncobject nodeid_t;

// virtual ring data packet
packet VRDataPkt {
    // time to live
    int ttl;
    
    // hop count
    int hopcount;
    
    // best distance (zero in the end)
    nodeid_t best;
    
    // source address
    nodeid_t src;
    
    // destination address
    nodeid_t dst;
}