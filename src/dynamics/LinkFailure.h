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

#ifndef LINKFAILURE_H_
#define LINKFAILURE_H_
#include "NetworkDynamics.h"
#include <list>
#include <utility>

namespace routingsim {

class LinkFailure: public NetworkDynamics {
private:

public:
	LinkFailure();
	virtual ~LinkFailure();
	virtual void initialize();
	virtual simtime_t scheduleDynamics();

};

} /* namespace routingsim */
#endif /* LINKFAILURE_H_ */
