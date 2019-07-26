//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// as published by the Free Software Foundation; either version 2
// of the License, or (at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with this program; if not, write to the Free Software
// Foundation, Inc., 59 Temple Place - Suite 330, Boston, MA  02111-1307, USA.

/**
 * @file KadContact.cc
 * @author Jakob Thomas
 */

#include "KadContact.h"

namespace routingsim {
std::ostream& operator<<(std::ostream& os, const KadContact& c) {
	os << c.getKey().shortStr();
	// state is next V = valid, I = invalid, U = undefined
	os << ":(" << (c.getState() == KadContact::State::valid ? 'V' :
		       ((c.getState() == KadContact::State::invalid) ? 'I' :
			((c.getState() == KadContact::State::rediscovering) ? 'R' : 'U')));
	
	if ( c.isPhysicalNeighbor() ) {
		os << "/P";
	}
	os << ")";
	if (c.getPath().size() == 0)
		return os << "<->";
	else {
		os << "< ";
		for (auto key : c.getPath())
			os << key.shortStr() << ' ';
		os << '>';
	}
	return os;
}

} // namespace
