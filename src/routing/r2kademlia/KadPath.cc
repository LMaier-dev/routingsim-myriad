#include "KadPath.h"

namespace routingsim {

// print a path using abbreviated key ids
std::ostream& operator<<(std::ostream& os, const kad_path& p)
{
	os << '<';
	if (p.size() == 0)
		 os << '-';
	else {
		for (const auto &key : p)
			os << ' ' << key.shortStr();
		os << ' ';
	}
	os << '>';
	return os;

}

}
