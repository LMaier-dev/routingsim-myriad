#include "routing/MYRIAD/MyriadHelper.h"
#include <cctype>

namespace routingsim {

MyriadLogLevel MyriadHelper::parseLogLevel(const char* s) {
    std::string str(s ? s : "");
    for (auto& c : str) c = (char) std::toupper((unsigned char) c);
    if (str == "DEBUG") return MLOG_LEVEL_DEBUG;
    if (str == "INFO")  return MLOG_LEVEL_INFO;
    if (str == "WARN" || str == "WARNING") return MLOG_LEVEL_WARN;
    if (str == "ERROR") return MLOG_LEVEL_ERROR;
    return MLOG_LEVEL_DEBUG;
}

const char* MyriadHelper::levelColor(const char* level) {
    if (!std::strncmp(level, "DEBUG", 5)) return "\033[2;37m"; // dim grey
    if (!std::strncmp(level, "INFO",  4)) return "\033[36m";   // cyan
    if (!std::strncmp(level, "WARN",  4)) return "\033[33m";   // yellow
    if (!std::strncmp(level, "ERROR", 5)) return "\033[31m";   // red
    return "";
}

std::string MyriadHelper::logPrefix(const char* level) const {
    std::ostringstream o;
    if (colorLog) o << levelColor(level);
    o << "[t=" << omnetpp::simTime()
      << " n=" << (int) myId
      << " " << level << "]";
    if (colorLog) o << "\033[0m";
    o << " ";
    return o.str();
}

std::string MyriadHelper::logPrefix(const char* level, nodeid_t nbr) const {
    NeighborState state = getNeighborState(nbr);
    std::ostringstream o;
    if (colorLog) o << levelColor(level);
    o << "[t=" << omnetpp::simTime()
      << " n=" << (int) myId
      << " nbr=" << (int) nbr << "/" << nstateName(state)
      << "]";
    if (colorLog) o << "\033[0m";
    o << " ";
    return o.str();
}

PendingTx* MyriadHelper::sendReliable(nodeid_t to, cPacket* pkt, simtime_t interval) {
    auto* h = new PendingTx{pkt, to, interval, nullptr};

    // First send: dup() so we keep the template for retransmits
    auto np = std::make_unique<NodePkt>(pkt->getName());
    np->setType(packetType);
    np->encapsulate(pkt->dup());
    sendToNode(to, std::move(np));

    h->timer = addOneShotTimer(interval, [this, h](short) {
        retxFire(h);
        return false;
    });
    return h;
}

void MyriadHelper::retxFire(PendingTx* h) {
    auto np = std::make_unique<NodePkt>(h->pkt->getName());
    np->setType(packetType);
    np->encapsulate(h->pkt->dup());
    sendToNode(h->dest, std::move(np));

    h->timer = addOneShotTimer(h->interval, [this, h](short) {
        retxFire(h);
        return false;
    });
}

void MyriadHelper::cancelReliable(PendingTx*& h) {
    if (!h) return;
    if (h->timer) cancelAndDelete(h->timer);
    delete h->pkt;
    delete h;
    h = nullptr;
}

} // namespace routingsim
