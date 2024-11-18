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
@property[castFunction](type=bool; usage=class; desc="If false: Do not specialize the fromAnyPtr<T>(any_ptr) function for this class. Useful for preventing compile errors if the function already exists, e.g. in hand-written form, or generated for another type (think aliased typedefs).");


cplusplus {{
#include "BaseTypes.h"
#include "NodePkt_m.h"

namespace routingsim {
// new "types"


} // end namespace

}}

// the following will be defined in namespace routingsim
namespace routingsim;

enum KIRALSRPDUType {
	KIRALSR_PNHello = 		0x01;
}

// a KIRALSR control packet
packet KIRALSRPacket {
	@customize(true);
	uint8_t type @enum(KIRALSRPDUType);
}
