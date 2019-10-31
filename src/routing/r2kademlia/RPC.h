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

#ifndef __ROUTINGSIM_RPC_H_
#define __ROUTINGSIM_RPC_H_

#include <omnetpp.h>
#include <memory>
#include <unordered_map>
#include <functional>
#include "Protocol.h"

namespace routingsim {


typedef unsigned int nonce_t;
//KadPacket;
using packet_t = class KadPacket;
using packet_p = std::unique_ptr<KadPacket>;

/**
 * A basic implementation of a component that sets up rpc calls.
 */
class RPCProtocol : public Protocol {

protected:
	enum RPC_STATUS { RPC_WAITING, RPC_TIMEOUT, RPC_ERROR, RPC_SUCCESS};
	/// represents the state of an rpc call, gets called
	struct RPCState;
	/// a function that handles some rpc state change, returns true if rpc is done
	using rpc_callback_t = std::function<bool(packet_p, RPCState&)>;
	struct RPCState {
	   public:	
		nonce_t nonce;
		packet_p original_packet;
		uint8_t retransmissions = 0;
		uint8_t maxRetransmissions = 0;
		unsigned int timeout_ms = 500; // timeout in ms
		RPC_STATUS status = RPC_WAITING;
		rpc_callback_t success_callback;
		rpc_callback_t error_callback;
		rpc_callback_t timeout_callback;
		cMessage* timeoutMsg = nullptr;
	};

	
	class RPCtimeoutMsg : public cMessage
	{
	  public:
	        RPCtimeoutMsg(nonce_t nonce) : cMessage("RPCTimerMsg"), nonce(nonce) {};
		nonce_t nonce;

	};
	
protected:
	/** @name functions called by the subclass */
	//@{
	/// dispatches the answer to the correct RPCState
	void handleAnswer(packet_p answer);

	/// uses a timeout callback function for the corresponding RPC
	void handleTimeOut(nonce_t nonce);
	
	/**
	 * registers the given packet, assigns a nonce and sends it out.
	 *
	 * @success_callback is called with the answer when it arrives
	 * @error_callback is called with the error packet when it arrives
	 * @timeout_callback is called when a timeout is reached
	 */
	
	void sendRpcCall(packet_p rpcCall, rpc_callback_t success_callback,
		 rpc_callback_t error_callback,  rpc_callback_t timeout_callback);
	/**
	 * registers the given packet, assigns a nonce and sends it out
	 * callback is called on answer, error or timeout
	 */
	void sendRpcCall(packet_p rpcCall, rpc_callback_t callback);
	//@}

	/** @name functions implemented by the subclass */
	//@{
	/// used for sending packets
	virtual void sendPacket(packet_p packet) = 0;
	//@}
public:	
        void finish();

private:
	/// maps nonces to rpc states, used to dispatch answers
	std::unordered_map<nonce_t, RPCState> states;
};

} // namespace
#endif /*__ROUTINGSIM_RPC_H_*/
