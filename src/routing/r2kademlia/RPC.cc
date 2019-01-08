
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

template<typename Answer> nonce_t RPCComponent<Answer>::registerState(RPCState_p rpcState)
{
	nonce_t nonce = nonceCounter++;
	ASSERT(nonceCounter < std::numeric_limits<nonce_t>::max());
	ASSERT(states.find(nonce) == states.end());
	rpcState->nonce = nonce;
	rpcState->parent = this;
	states[nonce] = rpcState;
	return nonce;
}

template<typename Answer> void RPCComponent<Answer>::handleAnswer(nonce_t nonce, Answer answer)
{
	auto it = states.find(nonce);
	if(it == states.end()) {
		onUnknownNonce(nonce);
		return;
	}
	RPCState_p state = it->second;
	states.erase(it);
	state->onAnswer(answer);
}

}
