#ifndef __ROUTINGSIM_MYRIADHELPER_H_
#define __ROUTINGSIM_MYRIADHELPER_H_

#include "foundation.h"
#include "routing/KIRA/TimerMsg.h"
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
    protected:
        nodeid_t myId;
        MyriadLogLevel logLevel = MLOG_LEVEL_DEBUG;
        bool colorLog = false;
        // Subclass must set this (e.g., in onInitialize) before calling sendReliable.
        // Used as the NodePkt type when the helper wraps reliable payloads.
        int packetType = -1;

        // Per-message-type send counters
        std::map<std::string, unsigned long> sentCounts;
        void countSend(const std::string& type) { sentCounts[type]++; }
        static std::string sendTypeLabel(const char* pktName);
        void recordSendScalars() {
            unsigned long total = 0;
            for (auto& [type, n] : sentCounts) {
                recordScalar(("sent_" + type).c_str(), n);
                total += n;
            }
            recordScalar("sent_total", total);
            unsigned long long totalBytes = 0;
            for (auto& [nbr, st] : node().getLinkUsageMap()) totalBytes += st.bytecount;
            recordScalar("bytes_total", totalBytes);
        }

        void onInitialize() override {
            myId = node().getNodeId();
            logLevel = parseLogLevel(par("logLevel").stringValue());
            colorLog = par("colorLog").boolValue();
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
        
        static std::string formatChain(const std::vector<uint32_t>& chain) {
            std::string s = "[";
            for (size_t i = 0; i < chain.size(); ++i) {
                if (i) s += ',';
                s += std::to_string(chain[i]);
            }
            s += ']';
            return s;
        }

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
