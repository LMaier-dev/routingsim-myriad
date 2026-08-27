#ifndef __ROUTINGSIM_MYRIADGPRIB_H_
#define __ROUTINGSIM_MYRIADGPRIB_H_

#include "routing/MYRIAD/MyriadGpPkt_m.h"
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
        uint64_t chain_cost = 0;           // sum of memberCount^ringCostExponent over chain
        uint32_t seq_no = 0;
        uint32_t spof_count = 0;
    };

    // Everything a router keeps per origin ring it knows a route to.
    struct RibRing {
        std::vector<MyriadGpRr> rr; // (subnet, owner) pairs; refreshed when curSeq advances
        uint32_t curSeq = 0;        // highest seq_no seen for this origin ring
        // Learned chains for this origin. Usually one generation (all co-best).
        // Make-before-break may also keep PREVIOUS-generation chains (seq_no ==
        // curSeq-1) that beat the current one, so the data plane keeps a good
        // redundant route while reconverging.
        // At most two adjacent generations coexist.
        std::vector<RibEntry> ribIn;
        size_t preferred = SIZE_MAX; // index into ribIn; SIZE_MAX = none
    };

    // dup = the exact chain was already held at >= seq. propagate = the chain
    // won its generation and should be transitioned inter-ring (subject to the
    // caller's loop guard).
    struct StoreResult {
        bool dup = false;
        bool propagate = false;
        size_t cobest = 0;      // ribIn size at the decision point, for logging
    };

    // Origin-scoped route state and the whole rank/store/select cascade. No
    // protocol, timer or packet state — MyriadGP owns those.
    class MyriadGpRib {
        public:
            void configure(long costExponent, bool useSpof, long maxEcmp) {
                ringCostExponent = costExponent;
                rankUseSpof = useSpof;
                maxEcmpPaths = maxEcmp;
            }
            void clear() { rings.clear(); degrades.clear(); }

            uint64_t sizeToCost(uint32_t s) const;
            uint32_t curSeq(uint32_t ring) const;
            const std::vector<MyriadGpRr>& rr(uint32_t ring) const;
            void setRr(uint32_t ring, std::vector<MyriadGpRr> v) { rings[ring].rr = std::move(v); }
            // Ring size changed under us: everything learned about it is suspect.
            void invalidate(uint32_t ring);
            // Adopt generation `originSeq` for a ring I originate on and install
            // my trivial [ring] chain — else the king has no entry to its own
            // ring and backfills a looped-back detour.
            void installSelfChain(uint32_t ring, uint32_t originSeq, uint64_t cost);

            StoreResult store(uint32_t origin, const std::vector<uint32_t>& fullChain,
                              uint64_t fullCost, uint32_t seq, uint32_t spof,
                              const std::vector<MyriadGpRr>* originRrTrailer);

            // A ring's lap broke: open or re-beacon its degrade. Returns (seq, beacon)
            // to put on the wire.
            std::pair<uint32_t, uint32_t> markDegraded(uint32_t ring);
            // false = stale or duplicate, so the caller must not re-flood.
            bool applyDegrade(uint32_t ring, uint32_t seq, uint32_t beacon);

            // Pointers borrow from the RIB; invalid after any mutating call.
            struct ReplayChain {
                const std::vector<uint32_t>* chain;
                uint64_t cost;
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

            void dumpTo(std::ostream& f, const std::string& tprefix, bool stats, int nodeId) const;

        private:
            std::unordered_map<uint32_t /*origin ring*/, RibRing> rings;
            // Degrade state per culprit ring, kept here (not per-chain) so RIB-In
            // purges don't lose it; chains re-mark from this.
            std::unordered_map<uint32_t /*culprit ring*/,
                               std::pair<uint32_t /*seq_no*/, uint32_t /*beacon_no*/>> degrades;
            // Path cost = sum of member_count^ringCostExponent over the chain.
            // Integer exponent (a>=1) keeps costs in uint64 so co-best ties are
            // exact (no float ULP mismatch), which ECMP needs.
            long ringCostExponent = 1;
            // false = rank purely by cost (oracle mode); see MyriadGp.ned.
            bool rankUseSpof = true;
            long maxEcmpPaths = 1;

            // Rank order: spof, then cost, then chain length. Used by both the
            // store cascade and selectPreferred.
            bool strictlyBetter(const RibEntry& a, const RibEntry& b) const;
            // Co-best: equal on (spof, cost, chain-length). Chain order is not in
            // the key, so genuine ties are retained for ECMP.
            bool coBest(const RibEntry& a, const RibEntry& b) const;
            bool frontRepresented(const RibRing& rs, uint32_t front, size_t except) const;
            // New-best erase that keeps each front-ring's sole current-gen.
            void dropCurGenKeepingFronts(RibRing& rs, uint32_t candFront);
            // Trim ribIn to maxEcmpPaths, keeping the most ring-diverse subset;
            // operates on the current generation only. Returns whether candChain
            // survived.
            bool capEcmp(RibRing& rs, const std::vector<uint32_t>& candChain);
            // Best across all of ribIn (strictlyBetter, then lex-min chain),
            // ignoring seq — so preferred may lag curSeq (see ribIn in RibRing).
            void selectPreferred(RibRing& rs);
            void reselectAll();
            // A chain is degraded iff any ring on it has an active degrade.
            bool chainDegraded(const std::vector<uint32_t>& chain) const;
            // A newer generation of `ring` proves it healed → drop its degrade.
            void clearDegradeIfRecovered(uint32_t ring);
    };

} // namespace routingsim

#endif
