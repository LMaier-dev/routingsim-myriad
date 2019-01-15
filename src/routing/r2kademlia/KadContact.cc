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

std::vector<KadContact_p>::iterator findContactInList(KadContactList& list, const OverlayKey& key){
	auto it = list.begin();
	for (; it !=  list.end() && !(*it)->hasKey(key); it++);
	return it;
}

bool removeContactFromList(KadContactList& list, const OverlayKey& key)
{
	auto it = findContactInList(list, key);
	if (it != list.end()){
		list.erase(it);
		return true;
	}
	else
		return false;
}

} // namespace
