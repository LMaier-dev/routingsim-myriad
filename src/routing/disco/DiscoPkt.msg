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

cplusplus {{
/// definition of a route
class Node_ID {
public:
	nodeid_t id;
	const char *origin= nullptr;
	bool operator==(const Node_ID &cmp) const {
		return(id==cmp.id);
	}
};
inline std::ostream& operator<<(std::ostream& os, const Node_ID& u ) {
    os << " id=" << u.id << " origin=" << (u.origin != nullptr ? u.origin : "NULL") ;
    return os;
}
}}


//
// example message format
//
class noncobject Node_ID;

/// if landmarkAnnouncement: landmark in position 0
/// if vicinityAnnouncement: announcing node in position 0
packet DiscoPkt {
     Node_ID pathVector[];
}