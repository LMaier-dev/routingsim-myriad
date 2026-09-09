#ifndef __ROUTINGSIM_MYRIADPDUSIZE_H_
#define __ROUTINGSIM_MYRIADPDUSIZE_H_

#include <cstdint>

// Modelled on-wire field widths (bytes) for MYRIAD control PDUs, shared by the
// path-vector (MyriadGp) and link-state (MyriadLs) protocols and the common
// Hello.

namespace routingsim {
namespace msgsize {

// Machine widths.
constexpr int64_t NODEID       = 2;   // nodeid_t = unsigned short
constexpr int64_t U32          = 4;
constexpr int64_t U64          = 8;
constexpr int64_t U16          = 2;
constexpr int64_t U8           = 1;
constexpr int64_t BOOL         = 1;
constexpr int64_t TYPE_TAG     = 1;   // PDU / LSA type enum

// Semantic widths — change these, not the machine widths above, and change them
// for BOTH protocols at once.
constexpr int64_t RING     = U16;  // ring id: 65,535 rings, far past what the RIB fits
constexpr int64_t SEQ      = U16;  // origin generation; LS LSAs already used 16 bits
constexpr int64_t SPOF     = U8;   // spofs on a chain <= its length (max 54 measured)
constexpr int64_t COST     = U32;  // sum of memberCount^e over the chain
constexpr int64_t RSIZE    = U16;  // ring member_count / origin counts
constexpr int64_t INTERVAL = U16;  // a timer in seconds, as OSPF sends them

// Per-packet L2/L3 framing: Ethernet II header (6 dst + 6 src + 2 ethertype) and
// FCS, plus an IPv4 header.
constexpr int64_t ETH_HDR  = 14;
constexpr int64_t ETH_FCS  = 4;
constexpr int64_t IPV4_HDR = 20;
constexpr int64_t FRAME    = ETH_HDR + ETH_FCS + IPV4_HDR;   // 38

constexpr int64_t MYRIADLS_HDR = TYPE_TAG + NODEID;             // 3:  type + router_id_src
constexpr int64_t MYRIADGP_HDR = TYPE_TAG + NODEID + SEQ + RING + U8; // 8: + seq_no + current_ring + hops
constexpr int64_t LSA_HDR      = TYPE_TAG + SEQ + NODEID;       // 5:  type + seq + origin
constexpr int64_t RR           = U32 + NODEID;                  // 6:  subnet + owner
constexpr int64_t TRANSITION   = RING + BOOL;                   // 3:  ring + isRedundant
constexpr int64_t LSREQ        = NODEID + TYPE_TAG;             // 3:  origin + type

} // namespace msgsize
} // namespace routingsim

#endif
