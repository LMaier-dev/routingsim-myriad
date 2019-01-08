
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
#include <map>
#include <atomic>
#include "Protocol.h"

namespace routingsim {
typedef unsigned int nonce_t;

/**
 * A basic implementation of a component that sets up rpc calls.
 * TODO retransmittions and timeouts
 */
template<typename Answer>
class RPCComponent : public Protocol {
protected:
	using Answer_p = std::unique_ptr<Answer>;
	/// represents the state of an rpc call, gets called
	class RPCState {
	public:
		nonce_t nonce;
		RPCComponent* parent;
		inline const cComponent* getThisPtr() const { return parent->getThisPtr(); };
		virtual void retransmit() {};
		virtual void handleTimeout() {};
		virtual void handleAnswer(Answer_p answer) = 0;
	};
	using RPCState_p = std::unique_ptr<RPCState>;
protected:
	/// dispatches the answer to the correct RPCState
	void handleAnswer(nonce_t nonce, Answer answer);
	/// registers the given state and returns the assigned nonce
	nonce_t registerState(RPCState_p rpcState);
	/// gets called when handleAnswer encounters an onknown nonce
	virtual void onUnknownNonce(nonce_t nonce, Answer_p answer) = 0;
private:
	/// maps nonces to rpc states, used to dispatch answers
	std::map<nonce_t, RPCState_p> states;
	/// counter to ensure nonce uniqueness
	std::atomic_int nonceCounter;
};

} // namespace
#endif /*__ROUTINGSIM_RPC_H_*/
