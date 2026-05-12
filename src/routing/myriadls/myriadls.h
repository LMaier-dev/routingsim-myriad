#ifndef __ROUTINGSIM_MYRIADLS_H_
#define __ROUTINGSIM_MYRIADLS_H_

#include "foundation.h"

namespace routingsim {

class MyriadLS : public Protocol
{
protected:
    void onInitialize() override;
    void onConnect(nodeid_t nid) override;
    void onDisconnect(nodeid_t nid) override;
    void onMessage(NodePkt_p pkt) override;
    void reset() override;

private:
    nodeid_t myId;
    std::string rings;
};

} // namespace routingsim

#endif