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

enum MyriadPDUType {
    MYRIAD_HELLO = 0;
    MYRIAD_DBD = 1;
    MYRIAD_LSR = 2;
    MYRIAD_LSU = 3;
    MYRIAD_ACK = 4;
}

enum MyriadLSAType {
    LSA_ADJACENCIES = 0; // list of neighbors the origin is adjacent to
    LSA_SUBNETS = 1; // connected subnets to the origin
    LSA_RINGS = 2; // origin is member of rings
}

// directly from Kira
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

packet MyriadHeader {
    MyriadPDUType type;
    nodeid_t router_id_src;
}

packet MyriadHello extends MyriadHeader {
    type = MYRIAD_HELLO;
    simtime_t helloInterval;
    simtime_t deadInterval;
    nodeid_t knownNeighbors[];
}

packet MyriadDBD extends MyriadHeader {
    type = MYRIAD_DBD;
    uint16_t seq_no; // of the exchange (not of the LSA) masters seq_no wins
    bool init; // set on first dbd packet only
    bool master; // Both set this initally, the one with the higher id wins and will keep this
    bool more; // indicates to receiving node that there will be more DBDs
    LsaHeader ls_dbds[];
}

// extends cObject so dup() is virtual: lets MyriadLSU's `@owned` array deep-copy
// the right derived LSA, and lets the LSDB store unique_ptr<LsaHeader> polymorphically.
class LsaHeader extends cObject {
    MyriadLSAType type;
    uint16_t seq_no;
    uint32_t age; // flush if too old TODO set max-age protocol parameter
    nodeid_t lsa_origin;
}

packet MyriadLSR extends MyriadHeader {
    type = MYRIAD_LSR;
    LsaHeader requested[];
}

// TODO check if this pointer approach works for the polymorphic LSAs
packet MyriadLSU extends MyriadHeader {
    type = MYRIAD_LSU;
    LsaHeader *lsas[] @owned;
}

packet MyriadAck extends MyriadHeader {
    type = MYRIAD_ACK;
    LsaHeader acked[];
}

class AdjacenciesLsa extends LsaHeader{
    type = LSA_ADJACENCIES;
    nodeid_t adjacencies[];
}

class SubnetsLsa extends LsaHeader{
    type = LSA_SUBNETS;
    uint64_t subnets[];
}

class RingsLsa extends LsaHeader{
    type = LSA_RINGS;
    uint64_t rings[]; // TODO decide on how to store rings
}
