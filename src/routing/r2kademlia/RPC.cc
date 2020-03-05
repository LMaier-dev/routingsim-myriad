
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
	// only schedule timer message in case we want retransmissions
	if (rpcState.maxRetransmissions > 0)
	{
		RPCtimeoutMsg* rpc_timeout_msg= new RPCtimeoutMsg(nonce);
		rpcState.timeoutMsg= rpc_timeout_msg; // just a reference for cancelling
		// start timer for RPC timeout
		scheduleAt(simTime() + SimTime(rpcState.timeout_ms,SIMTIME_MS), rpc_timeout_msg);
	}
	// register
	states[nonce] = std::move(rpcState);
	// send out
	sendPacket(std::move(rpcCall));
}


void RPCProtocol::handleAnswer(packet_p answer)
{
	nonce_t nonce = answer->getNonce();
	auto it = states.find(nonce);
	if (it == states.end()) {
		EV_ERROR << "received answer packet with unknown nonce " << answer->getNonce() << endl;
		return;
	}

	bool result;
	RPCState& rpcState = it->second;

	// cancel RPC timeout timer
	if (rpcState.timeoutMsg != nullptr)
		cancelAndDelete(rpcState.timeoutMsg);

	if (answer->getType() == KAD_Error) {
		rpcState.status = RPC_ERROR;
		result = rpcState.error_callback(std::move(answer), rpcState);
	}
	else {
		ASSERT(answer->getType() == rpcState.original_packet->getType() + 1);
		rpcState.status = RPC_SUCCESS;
		result = rpcState.success_callback(std::move(answer), rpcState);
	}
	// if result returns true, the RPC is finally done
	// erase the RPC state (should also destroy the original packet)
	if (result)
		states.erase(it);
}


void RPCProtocol::handleTimeOut(nonce_t nonce)
{
	auto it= states.find(nonce);
	if (it == states.end())
	{
		EV_ERROR << "Timeout RPC for " << nonce << " not found" << endl;
		return;
	}

	RPCState& rpcstate(it->second);
	// set state to TIMEOUT
	rpcstate.status= RPC_TIMEOUT;
	// it depends on the retransmission policy whether we still need this state
	if (rpcstate.timeout_callback) {
		if (rpcstate.timeout_callback(nullptr,rpcstate))
		{
			// RPC now done, so delete its state
			states.erase(it);
		}
	}
	else { // no callback to use, nevertheless delete pending RPC state
		EV_ERROR << "no timeout callback defined - deleting RPC state" << endl;
		states.erase(it);
	}
}


void RPCProtocol::finish() {

	//EV_TRACE << "RPC: # of states left: " << states.size() << endl;
	// for (const auto& state : states)
	// {
	// 	EV_TRACE << "Nonce: " << state.first << " RPCState " << state.second.status
	// 		 << " Packet: " << (state.second.original_packet ? state.second.original_packet->getName() : "NULL")
	// 		 << " id=" << (state.second.original_packet ? state.second.original_packet->getId() : 0) << endl;
	// }
}
	
} // namespace
