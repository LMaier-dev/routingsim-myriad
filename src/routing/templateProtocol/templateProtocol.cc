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

#include "templateProtocol.h"

namespace routingsim {

Define_Module(TemplateProtocol);

void TemplateProtocol::onInitialize() {
	EV << "TemplateProtocol::onInitialize()" << endl;
}

void TemplateProtocol::onConnect( nodeid_t nid ) {
	EV << "Now I am connected to " << nid << endl;
}

void TemplateProtocol::onDisconnect( nodeid_t nid ) {
	cout << "Now I am disconnected from " << nid << endl;
}

void TemplateProtocol::onMessage( NodePkt* pkt ) {
	EV << "received msg" << endl;
	switch (pkt->getType()) {
	default:
		EV << "unhandled packet: " << pkt->getType() << endl;
	}
	delete pkt;
}

void TemplateProtocol::reset() {
	EV << "resetting state of TemplateProtocol" << endl;
}

} //namespace
