#include "routing/MYRIAD/MyriadGpRib.h"
#include "routing/MYRIAD/MyriadHelper.h"
#include "routing/MYRIAD/MyriadPduSize.h"
#include <algorithm>
#include <limits>
#include <ostream>
#include <set>

namespace routingsim {

uint32_t MyriadGpRib::sizeToCost(uint32_t s) const {
    if (s == 2) s = (uint32_t) echoRingSize;   // see MyriadGp.ned
    // s^ringCostExponent with u32 saturation: a high exponent on a big ring still
    // overflows, and saturating keeps it costly rather than wrapping to 0.
    uint32_t term = 1;
    for (long k = 0; k < ringCostExponent; ++k) {
        if (term > std::numeric_limits<uint32_t>::max() / (s ? s : 1))
            return std::numeric_limits<uint32_t>::max();
        term *= s;
    }
    return term;
}

uint32_t MyriadGpRib::curSeq(uint32_t ring) const {
    auto it = rings.find(ring);
    return it == rings.end() ? 0 : it->second.curSeq;
}

const std::vector<MyriadGpRr>& MyriadGpRib::rr(uint32_t ring) const {
    static const std::vector<MyriadGpRr> none;
    auto it = rings.find(ring);
    return it == rings.end() ? none : it->second.rr;
}

void MyriadGpRib::invalidate(uint32_t ring) {
    RibRing& rs = rings[ring];
    rs.frames[0].clear();
    rs.frames[1].clear();
    rs.soleFront.clear();
    rs.hasPreferred = false;
}

void MyriadGpRib::installSelfChain(uint32_t ring, uint32_t originSeq, uint32_t cost) {
    RibRing& rs = rings[ring];
    if (originSeq > rs.curSeq) {            // flip: fresh generation of my own ring
        rs.curSeq = originSeq;
        rs.cur ^= 1;
        rs.frames[rs.cur].clear();
    }
    clearDegradeIfRecovered(ring);
    RibEntry self{{ring}, cost, originSeq, 0};
    auto& cf = rs.frames[rs.cur];
    for (auto& e : cf) if (e.chain == self.chain) return;
    cf.push_back(self);
    selectPreferred(rs);
}

bool MyriadGpRib::withinMargin(uint32_t a, uint32_t b) const {
    if (ecmpCostMargin <= 0) return a <= b;
    // double, since b may be sizeToCost's uint64 saturation where b*(1+m) wraps.
    double lim = (double)b * (1.0 + ecmpCostMargin);
    double cap = (double)std::numeric_limits<uint32_t>::max();
    return (double)a <= (lim > cap ? cap : lim);
}

bool MyriadGpRib::strictlyBetter(const RibEntry& a, const RibEntry& b) const {
    if (rankUseSpof && a.spof_count != b.spof_count) return a.spof_count < b.spof_count;
    if (!withinMargin(a.chain_cost, b.chain_cost)) return false;
    if (!withinMargin(b.chain_cost, a.chain_cost)) return a.chain_cost < b.chain_cost;
    // Mutually in-margin: siblings, so neither wins — else store()'s new-best
    // branch erases them. At margin 0 in-margin means equal cost; length breaks it.
    return ecmpCostMargin > 0 ? false : a.chain.size() < b.chain.size();
}

bool MyriadGpRib::betterExact(const RibEntry& a, const RibEntry& b) const {
    if (rankUseSpof && a.spof_count != b.spof_count) return a.spof_count < b.spof_count;
    if (a.chain_cost != b.chain_cost) return a.chain_cost < b.chain_cost;
    if (a.chain.size() != b.chain.size()) return a.chain.size() < b.chain.size();
    return a.chain < b.chain;   // total order, so preference is deterministic
}

bool MyriadGpRib::coBest(const RibEntry& a, const RibEntry& b) const {
    return (!rankUseSpof || a.spof_count == b.spof_count)
        // With a margin, length is already strictlyBetter's tiebreak.
        && (ecmpCostMargin > 0 || a.chain.size() == b.chain.size())
        && withinMargin(a.chain_cost, b.chain_cost)
        && withinMargin(b.chain_cost, a.chain_cost);
}

// Admit cand to frames[cur]; returns whether to propagate. The margin is
// pure acceptance: an in-band arrival widens the frame, an out-of-margin cheaper
// one replaces everything it beats. On a fullframe the evicted entry is the least  
// ring-diverse vs the best chain (ecmpDiverseEviction) OR the costliest.
bool MyriadGpRib::admitToFrame(RibRing& rs, const RibEntry& cand) {
    auto& cf = rs.frames[rs.cur];

    // Index, not pointer: erase/push_back below invalidate references into cf.
    size_t bestIdx = SIZE_MAX;
    for (size_t i = 0; i < cf.size(); ++i)
        if (bestIdx == SIZE_MAX || betterExact(cf[i], cf[bestIdx])) bestIdx = i;
    if (bestIdx == SIZE_MAX) { cf.push_back(cand); return true; }

    if (strictlyBetter(cand, cf[bestIdx])) {    // new best: drop everything worse
        // The self chain [origin] survives: it is this node's own membership, not
        // a learned route, and re-learning the ring via a neighbour costs a hop.
        cf.erase(std::remove_if(cf.begin(), cf.end(),
            [&](const RibEntry& e) {
                return e.chain.size() > 1 && cand.chain_cost < e.chain_cost; }), cf.end());
        cf.push_back(cand);
        return true;
    }
    if (!coBest(cand, cf[bestIdx])) return false;   // out of margin, inferior
    if ((long)cf.size() < maxEcmpPaths) { cf.push_back(cand); return true; }

    // Full frame: pick the eviction target over (frame \ best) ∪ {cand}.
    const std::vector<uint32_t> bc = cf[bestIdx].chain;
    auto intersect = [&](const std::vector<uint32_t>& c) {
        std::set<uint32_t> s(bc.begin(), bc.end());
        size_t n = 0; for (uint32_t r : c) n += s.count(r); return n;
    };
    // A cand cheaper than best is always admitted, so exempt it from being the
    // target; else it competes on the eviction key.
    bool candSafe = betterExact(cand, cf[bestIdx]);
    size_t worst = SIZE_MAX, worstKey = 0; uint64_t worstCost = 0;
    for (size_t i = 0; i < cf.size(); ++i) {
        if (i == bestIdx) continue;
        size_t key = ecmpDiverseEviction ? intersect(cf[i].chain) : 0;
        bool wins = worst == SIZE_MAX;
        if (!wins && ecmpDiverseEviction) wins = key > worstKey
            || (key == worstKey && cf[i].chain_cost > worstCost);
        else if (!wins) wins = betterExact(cf[worst], cf[i]);   // costlier => target
        if (wins) { worst = i; worstKey = key; worstCost = cf[i].chain_cost; }
    }
    if (worst == SIZE_MAX) {        // frame is just {best}: nothing to evict but it
        // A strictly cheaper chain must still take the slot.
        if (!candSafe || cf[bestIdx].chain.size() == 1) return false;
        cf[bestIdx] = cand;
        return true;
    }
    if (!candSafe) {                             // cand only enters if it beats the worst
        bool candWorse = ecmpDiverseEviction
            ? (intersect(cand.chain) > worstKey
               || (intersect(cand.chain) == worstKey && cand.chain_cost >= cf[worst].chain_cost))
            : betterExact(cf[worst], cand);
        if (candWorse) return false;
    }
    cf.erase(cf.begin() + worst);
    cf.push_back(cand);
    return true;
}

// Record cand as its front ring's best recent chain (replay coverage, degrade fallback
// and suppression, never forwarded). A better chain always wins; a worse one only
// replaces a slot two generations old, so a path that broke drops out within one cycle.
bool MyriadGpRib::updateSoleFront(RibRing& rs, const RibEntry& cand) {
    if (cand.chain.empty()) return false;
    auto it = rs.soleFront.find(cand.chain.front());
    if (it == rs.soleFront.end()
        || (it->second.chain == cand.chain && cand.seq_no >= it->second.seq_no)  // same path: re-quote or newer generation
        || betterExact(cand, it->second)                                          // better, any generation
        || cand.seq_no >= it->second.seq_no + 2) {                                // slot two generations stale
        rs.soleFront[cand.chain.front()] = cand;
        return true;
    }
    return false;
}

bool MyriadGpRib::ringAlreadyHas(uint32_t origin, uint32_t frontRing,
                                 const RibEntry& cand) const {
    auto ri = rings.find(origin);
    if (ri == rings.end()) return false;
    // The front-ring slot holds the best chain that ring recently carried, so a
    // candidate it beats would teach the ring nothing new.
    auto si = ri->second.soleFront.find(frontRing);
    if (si == ri->second.soleFront.end()) return false;   // ring never quoted it
    const RibEntry& have = si->second;
    // No generation check: the slot keeps the best of the last two generations
    // (updateSoleFront), and an entry two generations old is replaced by anything,
    // so a suppression that a broken path made wrong costs at most one refresh cycle.
    // A degraded chain is what the fallback needs an alternative to: never suppress against it.
    if (chainDegraded(have.chain)) return false;
    // Suppress only what the ring would reject outright. ECMP is admitted.
    return !strictlyBetter(cand, have) && !coBest(cand, have);
}

void MyriadGpRib::selectPreferred(RibRing& rs) {
    // Rank BOTH frames together by cost (make-before-break): while the current
    // generation is still converging, last generation's completed chains are
    // valid and may be cheaper, so they must stay eligible until the new gen
    // produces something as good. Non-degraded beats degraded first; then spof,
    // cost, chain IDs.
    const RibEntry* best = nullptr;
    auto consider = [&](const RibEntry& e) {
        if (!best) { best = &e; return; }
        bool de = chainDegraded(e.chain), db = chainDegraded(best->chain);
        if (de != db) { if (!de) best = &e; return; }
        if (rankUseSpof && e.spof_count != best->spof_count) { if (e.spof_count < best->spof_count) best = &e; return; }
        if (e.chain_cost != best->chain_cost) { if (e.chain_cost < best->chain_cost) best = &e; return; }
        if (e.chain < best->chain) best = &e;
    };
    for (auto& e : rs.frames[rs.cur]) consider(e);
    for (auto& e : rs.frames[rs.cur ^ 1]) consider(e);
    size_t ranked = rs.frames[rs.cur].size() + rs.frames[rs.cur ^ 1].size();
    // Degrade fallback: the frames only hold the best chains, which may all cross
    // the degraded ring. The freshest chain per front ring can leave around it.
    if (best && chainDegraded(best->chain)) {
        for (auto& [front, e] : rs.soleFront) consider(e);
        ranked += rs.soleFront.size();
    }
    rs.hasPreferred = best != nullptr;
    if (best) rs.preferred = *best;
    // One op per candidate ranked.
    if (onCalc) onCalc(ranked);
}

void MyriadGpRib::reselectAll() {
    for (auto& [ring, rs] : rings) selectPreferred(rs);
}

bool MyriadGpRib::chainDegraded(const std::vector<uint32_t>& chain) const {
    for (uint32_t r : chain) if (degrades.count(r)) return true;
    return false;
}

void MyriadGpRib::clearDegrade(uint32_t ring) {
    if (degrades.erase(ring)) reselectAll();
}

void MyriadGpRib::clearDegradeIfRecovered(uint32_t ring) {
    auto it = degrades.find(ring);
    if (it != degrades.end() && it->second.first < rings[ring].curSeq) {
        degrades.erase(it);
        reselectAll();
    }
}

std::pair<uint32_t, uint32_t> MyriadGpRib::markDegraded(uint32_t ring) {
    uint32_t seq = rings[ring].curSeq;
    bool isNew = !degrades.count(ring);
    auto& d = degrades[ring];               // culprit ring -> (seq, beacon)
    if (d.first < seq) d = {seq, 0};        // new generation
    else d.second += 1;                     // re-beacon the current one
    if (isNew) reselectAll();
    return d;
}

bool MyriadGpRib::applyDegrade(uint32_t ring, uint32_t seq, uint32_t beacon) {
    // Stale: the ring has already re-originated past this generation (recovered).
    auto rit = rings.find(ring);
    if (rit != rings.end() && seq < rit->second.curSeq) return false;
    // Dedup: act only on a strictly newer (seq, beacon) — else the flood loops.
    auto it = degrades.find(ring);
    if (it != degrades.end()
        && (seq < it->second.first
            || (seq == it->second.first && beacon <= it->second.second)))
        return false;
    bool isNew = it == degrades.end();
    degrades[ring] = {seq, beacon};
    if (isNew) reselectAll();
    return true;
}

// originRrTrailer is the rr to adopt on a seq-advance (the packet's own rr).
StoreResult MyriadGpRib::store(uint32_t origin, const std::vector<uint32_t>& fullChain,
                               uint32_t fullCost, uint32_t seq, uint32_t spof,
                               const std::vector<MyriadGpRr>* originRrTrailer) {
    StoreResult res;
    RibRing& originRs = rings[origin];

    RibEntry cand{fullChain, fullCost, seq, spof};

    // A real update (seq>0) from an older generation is stale. A replay (seq==0)
    // carries a live member's current chain rather than a generation, so it is
    // adopted purely on "I hold nothing myself".
    if (seq == 0 ? !originRs.frames[originRs.cur].empty()
                 : seq < originRs.curSeq) return res;

    // Whether anything selectPreferred RANKS actually moved. It ranks both
    // frames, and soleFront only while a degrade is open; rr changes never count.
    bool framesChanged = false;

    if (seq > originRs.curSeq) {              // flip
        framesChanged = true;
        originRs.curSeq = seq;
        clearDegradeIfRecovered(origin);
        if (originRrTrailer) originRs.rr = *originRrTrailer;
        originRs.cur ^= 1;
        originRs.frames[originRs.cur].clear();   // previous current stays as old
        // Bound soleFront to the same two generations the frames span.
        for (auto it = originRs.soleFront.begin(); it != originRs.soleFront.end(); )
            it = seq - it->second.seq_no >= 2 ? originRs.soleFront.erase(it) : std::next(it);
    } else if (originRs.rr.empty() && originRrTrailer && !originRrTrailer->empty()) {
        originRs.rr = *originRrTrailer;        // recovered node backfill at seq==curSeq
    }

    // Same chain re-arriving: keep the cheaper quote. An early lap can price a
    // ring at UNKNOWN_RING_SIZE, so only a not-cheaper repeat is a true dup.
    for (auto& e : originRs.frames[originRs.cur])
        if (e.chain == fullChain) {
            if (e.chain_cost <= fullCost) { res.dup = true; return res; }
            e = cand;
            selectPreferred(originRs);
            res.propagate = true;
            res.cobest = originRs.frames[originRs.cur].size();
            return res;
        }

    // Same chain lingering in the old frame is a stale quote of this path, not a
    // make-before-break backup; drop it so this generation's cost is the one used.
    auto& of = originRs.frames[originRs.cur ^ 1];
    size_t ofWas = of.size();
    of.erase(std::remove_if(of.begin(), of.end(),
        [&](const RibEntry& e) { return e.chain == fullChain; }), of.end());
    if (of.size() != ofWas) framesChanged = true;

    if (admitToFrame(originRs, cand)) { res.propagate = true; framesChanged = true; }
    // soleFront is ranked only while a degrade is open, so otherwise a rejected
    // candidate leaves the winner where it was.
    if (updateSoleFront(originRs, cand) && !degrades.empty()) framesChanged = true;
    if (framesChanged) selectPreferred(originRs);
    res.cobest = originRs.frames[originRs.cur].size();
    return res;
}

size_t MyriadGpRib::knownOrigins() const {
    size_t n = 0;
    for (auto& [origin, rs] : rings) if (rs.hasPreferred) ++n;
    return n;
}

uint64_t MyriadGpRib::stateBytes() const {
    // A chain is the path vector itself; the rest of RibEntry is its scalars.
    auto entry = [](const RibEntry& e) {
        return e.chain.size() * msgsize::RING    // chain
             + msgsize::COST                     // chain_cost
             + msgsize::SEQ + msgsize::SPOF;     // seq_no + spof_count
    };
    uint64_t b = 0;
    for (const auto& [origin, r] : rings) {
        b += msgsize::RING + msgsize::SEQ;       // origin ring key + curSeq
        b += r.rr.size() * msgsize::RR;
        for (const auto& frame : r.frames)
            for (const auto& e : frame) b += entry(e);
        for (const auto& [front, e] : r.soleFront) b += msgsize::RING + entry(e);
        if (r.hasPreferred) b += entry(r.preferred);
    }
    // degrades is per culprit ring, bounded by the ring count and tiny next to
    // the chains; counted for completeness rather than because it moves the number.
    b += degrades.size() * (msgsize::RING + 2 * msgsize::SEQ);
    return b;
}

MyriadGpRib::ReplayQuality MyriadGpRib::replayQuality() const {
    ReplayQuality q;
    for (auto& [origin, rs] : rings) {
        if (!rs.hasPreferred) continue;
        ++q.origins;
        q.spofTotal += rs.preferred.spof_count;
        // Saturating: a chain_cost can already be the max sentinel.
        uint64_t c = rs.preferred.chain_cost;
        q.costTotal = (c > std::numeric_limits<uint64_t>::max() - q.costTotal)
                    ? std::numeric_limits<uint64_t>::max() : q.costTotal + c;
    }
    return q;
}

std::vector<MyriadGpRib::ReplayChain> MyriadGpRib::preferredChains() const {
    std::vector<ReplayChain> out;
    // The per-origin preferred plus every soleFront slot: replay must reach every
    // (origin, front ring), and front coverage lives in soleFront.
    for (auto& [origin, ors] : rings) {
        if (!ors.hasPreferred) continue;
        auto emit = [&](const RibEntry& e) {
            if (!e.chain.empty())
                out.push_back({&e.chain, e.chain_cost, e.spof_count, &rr(origin)});
        };
        emit(ors.preferred);
        for (auto& [front, e] : ors.soleFront)
            if (e.chain != ors.preferred.chain) emit(e);
    }
    return out;
}

void MyriadGpRib::dumpTo(std::ostream& f, const std::string& tprefix, bool stats, int nodeId) const {
    std::vector<uint32_t> originIds;
    for (auto& [rid, rs] : rings)
        if (rs.hasPreferred) originIds.push_back(rid);
    std::sort(originIds.begin(), originIds.end());

    if (stats) {
        uint64_t n = 0, sum = 0, mx = 0;
        for (uint32_t rid : originIds) {
            uint64_t c = rings.at(rid).preferred.chain_cost;
            ++n; sum += c; if (c > mx) mx = c;
        }
        f << tprefix << nodeId << " n=" << n
          << " cost_sum=" << sum << " cost_max=" << mx << "\n";
        return;
    }

    // One line per origin with the primary chain.
    for (uint32_t rid : originIds) {
        const RibRing& rs = rings.at(rid);
        const RibEntry& e = rs.preferred;
        size_t ecmp = rs.frames[rs.cur].size();
        f << tprefix << nodeId << " origin=" << rid
          << " len=" << e.chain.size()
          << " spof=" << e.spof_count
          << " seq=" << e.seq_no
          << " cost=" << e.chain_cost
          << " ecmp=" << ecmp
          << " chain=" << formatChain(e.chain) << "\n";
    }
}

} // namespace routingsim
