#ifndef __ROUTINGSIM_MYRIADHELPER_H_
#define __ROUTINGSIM_MYRIADHELPER_H_

#include "foundation.h"
#include "routing/KIRA/TimerMsg.h"
#include "routing/MYRIAD/MyriadPduSize.h"
#include <algorithm>
#include <cstring>
#include <sstream>
#include <string>
#include <map>
#include <vector>

namespace routingsim {

enum NeighborState {
    NSTATE_DOWN,
    NSTATE_INIT,
    NSTATE_TWOWAY,
    NSTATE_FULL,
};

inline std::string formatChain(const std::vector<uint32_t>& chain) {
    std::string s = "[";
    for (size_t i = 0; i < chain.size(); ++i) {
        if (i) s += ',';
        s += std::to_string(chain[i]);
    }
    s += ']';
    return s;
}

inline const char* nstateName(NeighborState s) {
    switch (s) {
        case NSTATE_DOWN:   return "DOWN";
        case NSTATE_INIT:   return "INIT";
        case NSTATE_TWOWAY: return "TWOWAY";
        case NSTATE_FULL:   return "FULL";
    }
    return "?";
}

enum MyriadLogLevel {
    MLOG_LEVEL_DEBUG = 0,
    MLOG_LEVEL_INFO  = 1,
    MLOG_LEVEL_WARN  = 2,
    MLOG_LEVEL_ERROR = 3,
};

// Tracks an in-flight reliable transmission so it can be retransmitted or cancelled.
struct PendingTx {
    cPacket* pkt;        // template; dup()'d on each (re)send
    nodeid_t dest;
    simtime_t interval;
    TimerMsg* timer;
};

class MyriadHelper : public Protocol {
    public:
        ~MyriadHelper() override { delete txBytesVec; }

    protected:
        nodeid_t myId;
        MyriadLogLevel logLevel = MLOG_LEVEL_DEBUG;
        bool colorLog = false;
        // Subclass must set this (e.g., in onInitialize) before calling sendReliable.
        // Used as the NodePkt type when the helper wraps reliable payloads.
        int packetType = -1;

        // Per-message-type send counters and byte volume.
        std::map<std::string, unsigned long> sentCounts;
        std::map<std::string, unsigned long long> sentBytes;
        std::map<std::string, unsigned long long> sentMaxBytes;  // largest single PDU per type
        unsigned long long sentMaxBytesOverall = 0;              // largest single PDU, any type
        // Per-PDU byte samples over time (OMNeT++ vector, .vec output): sum across
        // all nodes' txBytes in a time window for network-wide overhead-over-time,
        // e.g. to see it spike during a failure wave.
        cOutVector* txBytesVec = nullptr;
        void countSend(const std::string& type, unsigned long long bytes = 0) {
            bytes += msgsize::FRAME;
            sentCounts[type]++;
            sentBytes[type] += bytes;
            auto& m = sentMaxBytes[type];
            if (bytes > m) m = bytes;
            if (bytes > sentMaxBytesOverall) sentMaxBytesOverall = bytes;
            if (txBytesVec) txBytesVec->record((double) bytes);
        }
        static std::string sendTypeLabel(const char* pktName);
        void recordSendScalars() {
            unsigned long total = 0;
            for (auto& [type, n] : sentCounts) {
                recordScalar(("sent_" + type).c_str(), n);
                total += n;
            }
            recordScalar("sent_total", total);
            // Per-type byte volume
            for (auto& [type, b] : sentBytes)
                recordScalar(("bytes_" + type).c_str(), b);
            unsigned long long totalBytes = 0;
            std::vector<unsigned long long> perLink;
            for (auto& [nbr, st] : node().getLinkUsageMap()) {
                totalBytes += st.bytecount;
                perLink.push_back(st.bytecount);
            }
            recordScalar("bytes_total", totalBytes);
            // Per LINK, not per node
            if (!perLink.empty()) {
                auto mid = perLink.begin() + perLink.size() / 2;
                std::nth_element(perLink.begin(), mid, perLink.end());
                recordScalar("linkbytes_med", (double) *mid);
                recordScalar("linkbytes_max",
                             (double) *std::max_element(perLink.begin(), perLink.end()));
            }
            // Largest single PDU observed, per type and overall.
            for (auto& [type, b] : sentMaxBytes)
                recordScalar(("bytes_max_" + type).c_str(), b);
            recordScalar("bytes_max", sentMaxBytesOverall);
        }

        void sendToNode(nodeid_t nid, NodePkt_p pkt, simtime_t delay = 0) {
            pkt->setByteLength(pkt->getByteLength() + msgsize::FRAME);
            Protocol::sendToNode(nid, std::move(pkt), delay);
        }

        void onInitialize() override {
            myId = node().getNodeId();
            logLevel = parseLogLevel(par("logLevel").stringValue());
            colorLog = par("colorLog").boolValue();
            txBytesVec = new cOutVector("txBytes");
        }

        // Default dispatch for TimerMsg callbacks (used by addOneShotTimer and
        // the reliable-tx retx timer). Subclasses with non-TimerMsg self-messages
        // should override and delegate here for the timer case.
        void onSelfMessage(cMessage_p msg) override {
            auto* timer = dynamic_cast<TimerMsg*>(msg.get());
            if (timer) {
                auto* cb = static_cast<TimerMsg::cbfunc_t*>(timer->getContextPointer());
                (*cb)(timer->getKind());
            }
        }

        // Subclasses provide neighbor-state lookup so log prefixes can include it.
        virtual NeighborState getNeighborState(nodeid_t nbr) const = 0;

        bool logEnabled(MyriadLogLevel lvl) const { return lvl >= logLevel; }

        std::string logPrefix(const char* level) const;
        std::string logPrefix(const char* level, nodeid_t nbr) const;

        static MyriadLogLevel parseLogLevel(const char* s);
        static const char* levelColor(const char* level);
        
        TimerMsg* addOneShotTimer(simtime_t delay, TimerMsg::cbfunc_t callback) {
            auto* msg = new TimerMsg(-1, callback);
            scheduleAt(simTime() + delay, msg);
            return msg;
        }

        // Reliable-tx helpers. The helper takes ownership of `pkt` and dup()'s it
        // for every (re)send. Cancel via cancelReliable(handle).
        PendingTx* sendReliable(nodeid_t to, cPacket* pkt, simtime_t interval);
        void cancelReliable(PendingTx*& handle);

    private:
        void retxFire(PendingTx* h);
};

} // namespace routingsim

// Streaming log macros. Use in member functions of a MyriadHelper subclass.
//   MLOG_DEBUG()      << "rings: " << r << "\n";
//   MLOG_N_WARN(from) << "unexpected DBD\n";
// Trailing newline is the caller's responsibility (matches existing EV << style).
#define MLOG_DEBUG() \
    for (bool _mlog_once = this->logEnabled(::routingsim::MLOG_LEVEL_DEBUG); _mlog_once; _mlog_once = false) \
        EV << this->logPrefix("DEBUG")

#define MLOG_INFO() \
    for (bool _mlog_once = this->logEnabled(::routingsim::MLOG_LEVEL_INFO); _mlog_once; _mlog_once = false) \
        EV << this->logPrefix("INFO")

#define MLOG_WARN() \
    for (bool _mlog_once = this->logEnabled(::routingsim::MLOG_LEVEL_WARN); _mlog_once; _mlog_once = false) \
        EV << this->logPrefix("WARN")

#define MLOG_ERROR() \
    for (bool _mlog_once = this->logEnabled(::routingsim::MLOG_LEVEL_ERROR); _mlog_once; _mlog_once = false) \
        EV << this->logPrefix("ERROR")

#define MLOG_N_DEBUG(n) \
    for (bool _mlog_once = this->logEnabled(::routingsim::MLOG_LEVEL_DEBUG); _mlog_once; _mlog_once = false) \
        EV << this->logPrefix("DEBUG", (n))

#define MLOG_N_INFO(n) \
    for (bool _mlog_once = this->logEnabled(::routingsim::MLOG_LEVEL_INFO); _mlog_once; _mlog_once = false) \
        EV << this->logPrefix("INFO", (n))

#define MLOG_N_WARN(n) \
    for (bool _mlog_once = this->logEnabled(::routingsim::MLOG_LEVEL_WARN); _mlog_once; _mlog_once = false) \
        EV << this->logPrefix("WARN", (n))

#define MLOG_N_ERROR(n) \
    for (bool _mlog_once = this->logEnabled(::routingsim::MLOG_LEVEL_ERROR); _mlog_once; _mlog_once = false) \
        EV << this->logPrefix("ERROR", (n))

#endif
