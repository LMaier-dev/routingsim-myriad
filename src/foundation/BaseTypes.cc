
#include "BaseTypes.h"

namespace routingsim {

std::ostream& operator<<(std::ostream& os, const hop_t& h) {
	return os << "[" << "from=" << h.from << ",to=" << h.to << ",dist=" << h.distance << "]";
}

}
