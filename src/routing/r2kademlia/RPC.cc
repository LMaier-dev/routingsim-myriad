
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

#include "RPC.h"

namespace routingsim {

nonce_t RPCComponent::registerState(RPCState_p rpcState)
{
	nonce_t nonce = nonceCounter++;
	ASSERT(nonceCounter < std::numeric_limits<nonce_t>::max());
	ASSERT(states.find(nonce) == states.end());
	rpcState->nonce = nonce;
	rpcState->parent = this;
	states[nonce] = std::move(rpcState);
	return nonce;
}

void RPCComponent::handleAnswer(nonce_t nonce, cPacket_p answer)
{
	auto it = states.find(nonce);
	if(it == states.end()) {
		onUnknownNonce(nonce, std::move(answer));
		return;
	}
	RPCState_p state = std::move(it->second);
	states.erase(it);
	state->handleAnswer(std::move(answer));
}

}
