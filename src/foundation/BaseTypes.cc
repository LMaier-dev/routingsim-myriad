
#include "BaseTypes.h"

namespace routingsim {

std::ostream& operator<<(std::ostream& os, const hop_t& h) {
	return os << "[" << "from=" << h.from << ",to=" << h.to << ",edge="
			<< h.edge << ",dist=" << h.distance << "]";
}

std::ostream& operator<<(std::ostream& os, const path_t& h) {
	for (path_t::const_iterator i = h.begin(); i!=h.end(); i++) {
		os << "path(";
		if (i!=h.begin()) os << ",";
		os << *i;
		os << ")";
	}
	return os;
}

}
