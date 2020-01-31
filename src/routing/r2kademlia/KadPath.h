#ifndef __ROUTINGSIM_KadPath_H_
#define __ROUTINGSIM_KadPath_H_

#include "OverlayKey.h"
#include <sstream>

namespace routingsim {

using kad_path = std::vector<OverlayKey>;

std::ostream& operator<<(std::ostream& os, const kad_path& c);
}

#endif // __ROUTINGSIM_KadPath_H_
