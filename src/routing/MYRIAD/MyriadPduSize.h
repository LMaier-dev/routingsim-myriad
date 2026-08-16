#ifndef __ROUTINGSIM_MYRIADPDUSIZE_H_
#define __ROUTINGSIM_MYRIADPDUSIZE_H_

#include <cstdint>

// Modelled on-wire field widths (bytes) for MYRIAD control PDUs, shared by the
// path-vector (MyriadGp) and link-state (MyriadLs) protocols and the common
// Hello. Widths match the declared .msg field types so a PDU's setByteLength()
// reflects a realistic control-plane payload. Node::sendToNode already tallies
// getByteLength() per link, so these feed the bytes_total scalar directly.
//
// ponytail: L2/L3 framing is intentionally NOT modelled — it is identical per
// packet for both protocols (and message count is reported separately), so it
// cancels in the GP-vs-LS comparison. Add a fixed FRAME constant here only if an
// absolute link-utilisation figure (not relative overhead) is ever required.

namespace routingsim {
namespace msgsize {

constexpr int64_t NODEID       = 2;   // nodeid_t = unsigned short
constexpr int64_t U32          = 4;
constexpr int64_t U16          = 2;
constexpr int64_t SIMTIME      = 8;   // simtime_t on the wire
constexpr int64_t TYPE_TAG     = 1;   // PDU / LSA type enum

constexpr int64_t MYRIADLS_HDR = TYPE_TAG + NODEID;             // 3:  type + router_id_src
constexpr int64_t MYRIADGP_HDR = TYPE_TAG + NODEID + U32 + U32; // 11: + seq_no + current_ring
constexpr int64_t LSA_HDR      = TYPE_TAG + U16 + U32 + NODEID; // 9:  type + seq + age + origin
constexpr int64_t RR           = U32 + NODEID;                  // 6:  subnet + owner
constexpr int64_t TRANSITION   = U32 + 1;                       // 5:  ring + isRedundant
constexpr int64_t LSREQ        = NODEID + TYPE_TAG;             // 3:  origin + type

} // namespace msgsize
} // namespace routingsim

#endif
