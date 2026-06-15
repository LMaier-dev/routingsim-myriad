@property[castFunction](type=bool; usage=class; desc="If false: Do not specialize the fromAnyPtr<T>(any_ptr) function for this class. Useful for preventing compile errors if the function already exists, e.g. in hand-written form, or generated for another type (think aliased typedefs).");

import foundation.NodePkt;

cplusplus {{
#include "BaseTypes.h"
#include "NodePkt_m.h"

namespace routingsim {
// new "types"

} // end namespace

}}

// the following will be defined in namespace routingsim
namespace routingsim;

// MyriadGP — path-vector variant. Hello packets are shared with MyriadLS
// (see MyriadLsPkt.msg / MyriadHello.{h,cc}). The PDUs below are the
// path-vector–specific ones (UPDATE / WITHDRAW / ACK) and use a Gp-prefixed
// class hierarchy so they don't collide with MyriadLsPkt_m's symbols when
// both modules are linked into the same binary.

enum MyriadGpPDUType {
    MYRIADGP_HELLO = 0;       // reuses LS MyriadHello on the wire
    MYRIADGP_UPDATE = 1;      // advertise one or more path-vector entries
    MYRIADGP_WITHDRAW = 2;    // retract previously-advertised destinations
    MYRIADGP_ACK = 3;         // ack an UPDATE/WITHDRAW
}

// directly from Kira (mirrors MyriadLsPkt.msg)
class nodeid_t {
      @cppType(routingsim::nodeid_t);
      @opaque;
      @subclassable(false);
      @supportsPtr(false);
      @editable(false);
      @castFunction(false);
      @byValue;
      @existingClass;
      @toValue((omnetpp::intval_t)($));
      @fromValue(omnetpp::checked_int_cast<unsigned short>($.intValue()));
}

packet MyriadGpHeader {
    MyriadGpPDUType type;
    nodeid_t router_id_src;
}

// One path-vector entry: destination plus the AS_PATH-style hop list of
// router ids the announcement traversed (used for loop detection).
// cObject so dup() is virtual and the @owned array can deep-copy entries.
class MyriadGpPath extends cObject {
    nodeid_t dest;
    uint32_t seq_no;       // monotonic per (origin, dest); newer wins
    nodeid_t path[];       // ordered list, origin first
}

packet MyriadGpUpdate extends MyriadGpHeader {
    type = MYRIADGP_UPDATE;
    MyriadGpPath *paths[] @owned;
}

packet MyriadGpWithdraw extends MyriadGpHeader {
    type = MYRIADGP_WITHDRAW;
    nodeid_t dests[];
    uint32_t seq_no;       // matches the UPDATE that originated the path
}

packet MyriadGpAck extends MyriadGpHeader {
    type = MYRIADGP_ACK;
    uint32_t seq_no;       // ack covers everything up to and including this
}
