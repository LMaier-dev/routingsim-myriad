#ifndef __ROUTINGSIM_MYRIADHELLO_H_
#define __ROUTINGSIM_MYRIADHELLO_H_

#include "routing/MYRIAD/MyriadHelper.h"
#include "routing/MYRIAD/MyriadLsPkt_m.h"
#include <sstream>
#include <unordered_map>
#include <vector>

namespace routingsim {

    // Common neighbor state every hello-driven protocol needs. Subclasses
    // extend this with their own per-neighbor bookkeeping and pass the 
    // derived type to MyriadHelloProto<N>.
    struct HelloNeighbor {
        nodeid_t id;
        NeighborState state = NSTATE_DOWN;
        TimerMsg* deadTimer = nullptr;
        std::vector<uint32_t> rings;
        // Intersection of `rings` and the local node's selfRings
        std::vector<uint32_t> sharedRings;
    };

    template <class N>
    class MyriadHelloProto : public MyriadHelper {
        protected:
            std::unordered_map<nodeid_t, N> neighbors;

            // Rings this node belongs to (integer ring ids).
            // Populated by initSelfRings()
            std::vector<uint32_t> selfRings;
            std::string selfRingsLabel;

            simtime_t helloInterval;
            simtime_t deadInterval;
            simtime_t rxmtInterval;
            simtime_t helloJitter;

            void onInitialize() override {
                MyriadHelper::onInitialize();
                helloInterval = par("helloInterval");
                deadInterval = par("deadInterval");
                rxmtInterval = par("rxmtInterval");
                helloJitter = par("helloJitter");
                initSelfRings();
            }

            void onConnect(nodeid_t nid) override {
                N& nbr = neighbors[nid];
                nbr.id = nid;
                nbr.state = NSTATE_DOWN;
            }

            void onDisconnect(nodeid_t nid) override {
                handleDead(nid);
            }

            NeighborState getNeighborState(nodeid_t nbr) const override {
                auto it = neighbors.find(nbr);
                return it == neighbors.end() ? NSTATE_DOWN : it->second.state;
            }

            // Hooks for subclasses to react to adjacency transitions.
            virtual void onTwoWayReached(N& /*nbr*/) {}
            virtual void onNeighborDead(N& /*nbr*/, bool /*wasFull*/) {}

            void initSelfRings() {
                // Ring membership from the ringLabel param, e.g. "1000000,999999".
                // Set per node via **.NodeN.*.ringLabel
                selfRingsLabel = hasPar("ringLabel") ? par("ringLabel").stdstringValue() : "";
                selfRings.clear();
                std::stringstream ss(selfRingsLabel);
                std::string tok;
                while (std::getline(ss, tok, ',')) {
                    size_t b = tok.find_first_not_of(" \t");
                    if (b == std::string::npos) continue;
                    selfRings.push_back(static_cast<uint32_t>(std::stoul(tok.substr(b))));
                }
                MLOG_INFO() << "rings: " << selfRingsLabel << "\n";
            }

            void scheduleHello(simtime_t delay) {
                // Add jitter so nodes don't all hello on the same tick
                helloTimer = addOneShotTimer(delay + uniform(0, helloJitter), [this](short) {
                    helloTimer = nullptr;
                    sendHello();
                    return false;
                });
            }

            void sendHello() {
                auto hello = std::make_unique<MyriadHello>("Myriad_Hello");
                hello->setRouter_id_src(myId);
                hello->setHelloInterval(helloInterval);
                hello->setDeadInterval(deadInterval);

                hello->setRingsArraySize(selfRings.size());
                for (size_t i = 0; i < selfRings.size(); ++i) {
                    hello->setRings(i, selfRings[i]);
                }

                // Only advertise neighbors we've actually heard a hello from.
                std::vector<nodeid_t> heard;
                heard.reserve(neighbors.size());
                for (auto& [nid, nbr] : neighbors) {
                    if (nbr.state >= NSTATE_INIT) heard.push_back(nid);
                }
                hello->setKnownNeighborsArraySize(heard.size());
                for (size_t i = 0; i < heard.size(); ++i) {
                    hello->setKnownNeighbors(i, heard[i]);
                }

                // TODO set hello->setByteLength
                for (auto& [nid, nbr] : neighbors) {
                    auto np = std::make_unique<NodePkt>("Myriad_Hello_Pkt");
                    np->setType(packetType);
                    np->setAttributes(np->getAttributes() | NODEPKT_ATTR_CONTROL_PLANE_PROCESSING_DELAY);
                    np->encapsulate(hello->dup());
                    sendToNode(nid, std::move(np));
                }
                scheduleHello(helloInterval);
            }

            void handleHello(MyriadHello* hello, nodeid_t from) {
                N& nbr = neighbors[from];
                nbr.id = from;
                resetDead(from);

                size_t nr = hello->getRingsArraySize();
                nbr.rings.assign(nr, 0);
                for (size_t i = 0; i < nr; ++i) nbr.rings[i] = hello->getRings(i);

                nbr.sharedRings.clear();
                for (uint32_t r : nbr.rings) {
                    for (uint32_t sr : selfRings) {
                        if (r == sr) { nbr.sharedRings.push_back(r); break; }
                    }
                }

                if (nbr.state == NSTATE_DOWN) {
                    nbr.state = NSTATE_INIT;
                    MLOG_N_INFO(from) << "received first hello (INIT)";
                    if (!nbr.rings.empty()) {
                        EV << " rings=[";
                        for (size_t i = 0; i < nbr.rings.size(); ++i)
                            EV << (i ? "," : "") << nbr.rings[i];
                        EV << "]";
                    }
                    EV << "\n";
                }

                if (nbr.state >= NSTATE_TWOWAY) return;

                int size = hello->getKnownNeighborsArraySize();
                for (int i = 0; i < size; i++) {
                    if (hello->getKnownNeighbors(i) == myId) {
                        nbr.state = NSTATE_TWOWAY;
                        MLOG_N_INFO(from) << "now in TWO-WAY state\n";
                        onTwoWayReached(nbr);
                        break;
                    }
                }
            }

            void resetDead(nodeid_t nid) {
                auto it = neighbors.find(nid);
                if (it == neighbors.end()) return;
                cancelAndDelete(it->second.deadTimer); // nullptr ok
                it->second.deadTimer = addOneShotTimer(deadInterval, [this, nid](short) {
                    handleDead(nid);
                    return false;
                });
            }

            void handleDead(nodeid_t nid) {
                auto it = neighbors.find(nid);
                if (it == neighbors.end()) return;
                N& nbr = it->second;
                bool wasFull = (nbr.state == NSTATE_FULL);
                MLOG_N_WARN(nid) << "neighbor timed out (DEAD)\n";
                nbr.deadTimer = nullptr;
                nbr.state = NSTATE_DOWN;
                onNeighborDead(nbr, wasFull);
            }

            void cancelHelloTimer() {
                if (helloTimer) {
                    cancelAndDelete(helloTimer);
                    helloTimer = nullptr;
                }
            }

        private:
            // Pending hello self-message (cancelled on reset).
            TimerMsg* helloTimer = nullptr;
    };

} // namespace routingsim

#endif
