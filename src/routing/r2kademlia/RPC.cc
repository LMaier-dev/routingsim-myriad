
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
#include "KadPacket.h"

namespace routingsim {
void RPCProtocol::sendRpcCall(packet_p rpcCall, rpc_callback_t callback)
{
	sendRpcCall(std::move(rpcCall), callback, callback, callback);
}

void RPCProtocol::sendRpcCall(packet_p rpcCall, rpc_callback_t success_callback,
	rpc_callback_t error_callback,  rpc_callback_t timeout_callback)
{
	nonce_t nonce = rpcCall->getTreeId();
	ASSERT(states.find(nonce) == states.end());
	rpcCall->setNonce(nonce);
	// create state
	RPCState rpcState;
	rpcState.nonce = nonce;
	rpcState.success_callback = success_callback;
	rpcState.error_callback = error_callback;
	rpcState.timeout_callback = timeout_callback;
	rpcState.original_packet = std::unique_ptr<packet_t>(rpcCall->dup());
	// register
	states[nonce] = std::move(rpcState);
	// send out
	sendPacket(std::move(rpcCall));
}

void RPCProtocol::handleAnswer(packet_p answer)
{
	nonce_t nonce = answer->getNonce();
	auto it = states.find(nonce);
	if(it == states.end()) {
		EV_WARN << "recieved answer packet with unknown nonce" << endl;
		return;
	}
	RPCState& rpcState = it->second;
	// TODO check for error
	rpcState.status = RPC_SUCCESS;
	bool result = rpcState.success_callback(std::move(answer), rpcState);
	// erase at last, because we give around references to it!
	if (result)
		states.erase(it);
}

} // namespace
