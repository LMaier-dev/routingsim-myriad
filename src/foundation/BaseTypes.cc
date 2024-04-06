
#include "BaseTypes.h"

namespace routingsim {

    std::ostream &operator<<(std::ostream &os, const hop_t &h) {
        return os << "[" << "from=" << h.from << ",to=" << h.to << ",dist=" << h.distance << "]";
    }

    const Coord2D Coord2D::ZERO = Coord2D(0.0, 0.0);
    const Coord2D Coord2D::ONE = Coord2D(1.0, 1.0);
    const Rad2D Rad2D::ZERO = Rad2D(0.0);

}
