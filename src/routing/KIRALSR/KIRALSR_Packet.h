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


#ifndef __ROUTINGSIM_KIRALSR_Packet_H_
#define __ROUTINGSIM_KIRALSR_Packet_H_

#include <omnetpp.h>
#include "KIRALSR_Packet_m.h"

namespace routingsim {


class KIRALSRPacket : public KIRALSRPacket_Base
{
  private:
    void copy(const KIRALSRPacket& other) { type = other.type; return; }

  public:
    KIRALSRPacket(const char *name=nullptr, short kind=0) : KIRALSRPacket_Base(name,kind) {}
    KIRALSRPacket(const KIRALSRPacket& other) : KIRALSRPacket_Base(other) {copy(other);}
    KIRALSRPacket& operator=(const KIRALSRPacket& other) {if (this==&other) return *this; KIRALSRPacket_Base::operator=(other); copy(other); return *this;}
    virtual KIRALSRPacket *dup() const override {return new KIRALSRPacket(*this);}

    virtual uint8_t getType() const override { return type; }
    virtual void setType(uint8_t type) override { this->type = type; }
};
}
#endif 
