#ifndef __ROUTINGSIM_MYRIADGPRIB_H_
#define __ROUTINGSIM_MYRIADGPRIB_H_

#include "routing/MYRIAD/MyriadGpPkt_m.h"
#include <array>
#include <cstdint>
#include <iosfwd>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

namespace routingsim {

    // A learned path to an origin ring. chain is prepend-order:
    // chain[0] = latest ring, chain.back() = origin.
    struct RibEntry {
        std::vector<uint32_t> chain;
        uint32_t chain_cost = 0;           // sum of memberCount^ringCostExponent over chain
        uint32_t seq_no = 0;
        uint32_t spof_count = 0;
    };

    // Everything a router keeps per origin ring it knows a route to. Two-frame
    // model: frames[cur] is curSeq's generation, frames[cur^1] the previous one
    // kept for make-before-break; a flip on a newer seq clears the new current.
    // soleFront holds the freshest chain per front ring — replay coverage, never
    // forwarded, not counted against a frame's maxEcmpPaths.
    struct RibRing {
        std::vector<MyriadGpRr> rr; // (subnet, owner) pairs; refreshed on a flip
        uint32_t curSeq = 0;
        std::array<std::vector<RibEntry>, 2> frames;
        uint8_t cur = 0;
        std::unordered_map<uint32_t /*front ring*/, RibEntry> soleFront;
        RibEntry preferred;         // cached winner across both frames + soleFront
        bool hasPreferred = false;
    };

    // dup = the exact chain was already held at >= seq. propagate = the chain
    // won its generation and should be transitioned inter-ring (subject to the
    // caller's loop guard).
    struct StoreResult {
        bool dup = false;
        bool propagate = false;
        size_t cobest = 0;      // current frame size at the decision point, for logging
    };

    // Origin-scoped route state and the whole rank/store/select cascade. No
    // protocol, timer or packet state — MyriadGP owns those.
    class MyriadGpRib {
        public:
            void configure(long costExponent, long echoSize, bool useSpof, long maxEcmp,
                           double ecmpMargin = 0.0, bool diverseEvict = true) {
                ringCostExponent = costExponent;
                echoRingSize = echoSize;
                rankUseSpof = useSpof;
                maxEcmpPaths = maxEcmp;
                ecmpCostMargin = ecmpMargin;
                ecmpDiverseEviction = diverseEvict;
            }
            void clear() { rings.clear(); degrades.clear(); }

            uint32_t sizeToCost(uint32_t s) const;
            uint32_t curSeq(uint32_t ring) const;
            const std::vector<MyriadGpRr>& rr(uint32_t ring) const;
            void setRr(uint32_t ring, std::vector<MyriadGpRr> v) { rings[ring].rr = std::move(v); }
            // Ring size changed under us: everything learned about it is suspect.
            void invalidate(uint32_t ring);
            // Adopt generation `originSeq` for a ring I originate on and install
            // my trivial [ring] chain — else the king has no entry to its own
            // ring and backfills a looped-back detour.
            void installSelfChain(uint32_t ring, uint32_t originSeq, uint32_t cost);

            StoreResult store(uint32_t origin, const std::vector<uint32_t>& fullChain,
                              uint32_t fullCost, uint32_t seq, uint32_t spof,
                              const std::vector<MyriadGpRr>* originRrTrailer);

            // Cancel if injecting `cand` into `frontRing` would teach it nothing
            bool ringAlreadyHas(uint32_t origin, uint32_t frontRing,
                                const RibEntry& cand) const;

            // A ring's lap broke: open or re-beacon its degrade. Returns (seq, beacon)
            // to put on the wire.
            std::pair<uint32_t, uint32_t> markDegraded(uint32_t ring);
            // false = stale or duplicate, so the caller must not re-flood.
            bool applyDegrade(uint32_t ring, uint32_t seq, uint32_t beacon);
            // Ring has an open degrade: its laps are not closing.
            bool isDegraded(uint32_t ring) const { return degrades.count(ring) != 0; }

            // Pointers borrow from the RIB; invalid after any mutating call.
            struct ReplayChain {
                const std::vector<uint32_t>* chain;
                uint32_t cost;
                uint32_t spof;
                const std::vector<MyriadGpRr>* rr;   // the origin ring's rr
            };
            // My best route to every origin I know; see MyriadGP::replayInto
            // for how each chain is rebased onto the ring it is replayed into.
            std::vector<ReplayChain> preferredChains() const;
            // Origins I currently hold a route to. Used as a RIB-richness test:
            // a just-recovered node knows only its own rings, a converged one
            // knows the network.
            size_t knownOrigins() const;

            // Bytes of RIB state this router holds, at the wire field widths of
            // MyriadPduSize.h so it is commensurable with MyriadLS::lsdbBytes().
            uint64_t stateBytes() const;

            // How good my RIB is as a replay source: breadth plus the totals
            // over the per-origin preferred chains, so a lap can rank members
            // that know equally much by how well they know it.
            struct ReplayQuality {
                uint32_t origins = 0;
                uint32_t spofTotal = 0;
                uint64_t costTotal = 0;   // sums over all origins: stays 64-bit
            };
            ReplayQuality replayQuality() const;

            void dumpTo(std::ostream& f, const std::string& tprefix, bool stats, int nodeId) const;

        private:
            std::unordered_map<uint32_t /*origin ring*/, RibRing> rings;
            // Degrade state per culprit ring, kept here (not per-chain) so RIB-In
            // purges don't lose it; chains re-mark from this.
            std::unordered_map<uint32_t /*culprit ring*/,
                               std::pair<uint32_t /*seq_no*/, uint32_t /*beacon_no*/>> degrades;
            // Path cost = sum of member_count^ringCostExponent over the chain.
            long ringCostExponent = 1;
            // Substituted for a member_count of 2; see MyriadGp.ned.
            long echoRingSize = 2;
            // false = rank purely by cost (oracle mode); see MyriadGp.ned.
            bool rankUseSpof = true;
            long maxEcmpPaths = 1;
            // See MyriadGp.ned. 0 = exact-equality co-best.
            double ecmpCostMargin = 0.0;
            bool ecmpDiverseEviction = true;
            bool withinMargin(uint32_t a, uint32_t b) const;
            // spof, then out-of-margin cheaper; flat for in-band equal-spof pairs
            // so an in-band arrival widens the ECMP set instead of replacing it.
            bool strictlyBetter(const RibEntry& a, const RibEntry& b) const;
            // Total order: spof → cost → length → chain IDs, for choosing within
            // a co-best set where strictlyBetter reports a tie.
            bool betterExact(const RibEntry& a, const RibEntry& b) const;
            // Acceptance: equal spof and mutually within the cost margin.
            bool coBest(const RibEntry& a, const RibEntry& b) const;
            // Admit cand to frames[cur]; returns whether it entered.
            bool admitToFrame(RibRing& rs, const RibEntry& cand);
            // Record cand as its front ring's freshest chain. Never forwards.
            void updateSoleFront(RibRing& rs, const RibEntry& cand);
            // Winner across both frames + soleFront, cached into rs.preferred:
            // not-degraded → spof → cost → fresh(seq==curSeq) → chain IDs.
            void selectPreferred(RibRing& rs);
            void reselectAll();
            // A chain is degraded iff any ring on it has an active degrade.
            bool chainDegraded(const std::vector<uint32_t>& chain) const;
            // A newer generation of `ring` proves it healed → drop its degrade.
            void clearDegradeIfRecovered(uint32_t ring);
    };

} // namespace routingsim

#endif
