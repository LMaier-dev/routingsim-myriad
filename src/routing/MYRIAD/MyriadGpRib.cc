#include "routing/MYRIAD/MyriadGpRib.h"
#include "routing/MYRIAD/MyriadHelper.h"
#include <algorithm>
#include <limits>
#include <map>
#include <ostream>
#include <set>

namespace routingsim {

uint64_t MyriadGpRib::sizeToCost(uint32_t s) const {
    // s^ringCostExponent with u64 saturation; UNKNOWN_RING_SIZE at e>=4 overflows,
    // saturating to max keeps it costly rather than wrapping to 0.
    uint64_t term = 1;
    for (long k = 0; k < ringCostExponent; ++k) {
        if (term > std::numeric_limits<uint64_t>::max() / (s ? s : 1))
            return std::numeric_limits<uint64_t>::max();
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
    rs.ribIn.clear();
    rs.preferred = SIZE_MAX;
}

void MyriadGpRib::installSelfChain(uint32_t ring, uint32_t originSeq, uint64_t cost) {
    RibRing& rs = rings[ring];
    if (originSeq > rs.curSeq) { rs.curSeq = originSeq; rs.ribIn.clear(); }
    clearDegradeIfRecovered(ring);
    RibEntry self{{ring}, cost, originSeq, 0};
    for (auto& e : rs.ribIn) if (e.chain == self.chain) return;
    rs.ribIn.push_back(self);
    selectPreferred(rs);
}

bool MyriadGpRib::withinMargin(uint64_t a, uint64_t b) const {
    if (ecmpCostMargin <= 0) return a <= b;
    // double, since b may be sizeToCost's uint64 saturation where b*(1+m) wraps.
    double lim = (double)b * (1.0 + ecmpCostMargin);
    double cap = (double)std::numeric_limits<uint64_t>::max();
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

// Sole front-ring guard: is `front` held by any CURRENT-GEN chain in rs.ribIn
// other than index `except`? Used to keep the last chain entering an origin via
// a given ring, so some member can still replay that (origin, front-ring) to a
// recovered ring-mate under best-only storage. (except=SIZE_MAX excludes none.)
bool MyriadGpRib::frontRepresented(const RibRing& rs, uint32_t front, size_t except) const {
    for (size_t i = 0; i < rs.ribIn.size(); ++i) {
        if (i == except) continue;
        const RibEntry& e = rs.ribIn[i];
        if (e.seq_no == rs.curSeq && !e.chain.empty() && e.chain.front() == front)
            return true;
    }
    return false;
}

// New-best erase with the sole front-ring guard: drop the now-inferior
// current-gen entries, but keep the single best current-gen chain of every
// front-ring != candFront (candFront is covered by the incoming best). Older
// holdovers (seq_no != curSeq) are untouched — make-before-break owns those.
void MyriadGpRib::dropCurGenKeepingFronts(RibRing& rs, uint32_t candFront) {
    std::map<uint32_t /*front*/, size_t /*best idx*/> bestPerFront;
    for (size_t i = 0; i < rs.ribIn.size(); ++i) {
        const RibEntry& e = rs.ribIn[i];
        if (e.seq_no != rs.curSeq || e.chain.empty() || e.chain.front() == candFront)
            continue;
        auto it = bestPerFront.find(e.chain.front());
        if (it == bestPerFront.end() || betterExact(e, rs.ribIn[it->second]))
            bestPerFront[e.chain.front()] = i;   // cheapest survivor per front ring
    }
    std::set<size_t> keep;
    for (auto& [f, idx] : bestPerFront) keep.insert(idx);
    std::vector<RibEntry> out;
    for (size_t i = 0; i < rs.ribIn.size(); ++i) {
        if (rs.ribIn[i].seq_no == rs.curSeq && !keep.count(i)) continue;  // now-inferior
        out.push_back(std::move(rs.ribIn[i]));
    }
    rs.ribIn.swap(out);
}

bool MyriadGpRib::capEcmp(RibRing& rs, const std::vector<uint32_t>& candChain) {
    // Cap the current generation (seq_no==curSeq) to maxEcmpPaths; retained older
    // holdovers aren't ECMP siblings, so skip them. Which one goes is
    // ecmpDiverseEviction: the member leaving the largest ring-union behind
    // (default — the set is a failover set), else simply the costliest. K is tiny.
    auto isCur = [&](size_t i) { return rs.ribIn[i].seq_no == rs.curSeq; };
    auto curCount = [&]() {
        size_t n = 0;
        for (size_t i = 0; i < rs.ribIn.size(); ++i) if (isCur(i)) ++n;
        return n;
    };
    auto unionSansIndex = [&](size_t skip) {
        std::set<uint32_t> u;
        for (size_t i = 0; i < rs.ribIn.size(); ++i) {
            if (i == skip || !isCur(i)) continue;
            for (uint32_t r : rs.ribIn[i].chain) u.insert(r);
        }
        return u.size();
    };
    auto droppable = [&](size_t i) {
        return isCur(i) && (rs.ribIn[i].chain.empty()
                            || frontRepresented(rs, rs.ribIn[i].chain.front(), i));
    };
    while ((long)curCount() > maxEcmpPaths) {
        size_t candIdx = SIZE_MAX;   // last current-gen entry (just appended)
        for (size_t i = rs.ribIn.size(); i-- > 0; ) if (isCur(i)) { candIdx = i; break; }
        // Seeded with the candidate so it loses every tie: an incumbent is only
        // displaced by a strictly better score, not by arrival order.
        size_t drop = SIZE_MAX, bestUnion = 0;
        if (droppable(candIdx)) { drop = candIdx; bestUnion = unionSansIndex(candIdx); }
        for (size_t i = 0; i < candIdx; ++i) {
            if (!droppable(i)) continue;
            bool wins = drop == SIZE_MAX;
            if (!wins && ecmpDiverseEviction) wins = unionSansIndex(i) > bestUnion;
            else if (!wins) wins = betterExact(rs.ribIn[drop], rs.ribIn[i]);
            if (wins) { drop = i; bestUnion = unionSansIndex(i); }
        }
        if (drop == SIZE_MAX) break;   // every over-cap entry is a sole front rep
        rs.ribIn.erase(rs.ribIn.begin() + drop);
    }
    for (auto& e : rs.ribIn) if (e.chain == candChain) return true;
    return false;
}

void MyriadGpRib::selectPreferred(RibRing& rs) {
    // Best across ALL of ribIn (may span two generations): a degraded chain (one
    // transiting a ring under an active DEGRADE) loses to any non-degraded one,
    // overriding spof/cost; within a class, betterExact. The margin governs which
    // chains are KEPT, never which of them is used — preferring anything but the
    // cheapest would pay the whole margin as stretch on the data plane.
    rs.preferred = SIZE_MAX;
    for (size_t i = 0; i < rs.ribIn.size(); ++i) {
        if (rs.preferred == SIZE_MAX) { rs.preferred = i; continue; }
        bool di = chainDegraded(rs.ribIn[i].chain);
        bool dp = chainDegraded(rs.ribIn[rs.preferred].chain);
        if (di != dp) { if (!di) rs.preferred = i; continue; }
        if (betterExact(rs.ribIn[i], rs.ribIn[rs.preferred]))
            rs.preferred = i;
    }
}

void MyriadGpRib::reselectAll() {
    for (auto& [ring, rs] : rings) selectPreferred(rs);
}

bool MyriadGpRib::chainDegraded(const std::vector<uint32_t>& chain) const {
    for (uint32_t r : chain) if (degrades.count(r)) return true;
    return false;
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
    auto& d = degrades[ring];               // culprit ring -> (seq, beacon)
    if (d.first < seq) d = {seq, 0};        // new generation
    else d.second += 1;                     // re-beacon the current one
    reselectAll();
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
    degrades[ring] = {seq, beacon};
    reselectAll();
    return true;
}

// originRrTrailer is the rr to adopt on a seq-advance (the packet's own rr).
StoreResult MyriadGpRib::store(uint32_t origin, const std::vector<uint32_t>& fullChain,
                               uint64_t fullCost, uint32_t seq, uint32_t spof,
                               const std::vector<MyriadGpRr>* originRrTrailer) {
    StoreResult res;
    RibRing& originRs = rings[origin];

    // Replay (seq==0) repopulates stateless nodes only: a node that already knows
    // this origin ignores it (the caller still forwards the lap to the recovered
    // node). Without this, live members treat it as a stale chain and re-store it.
    if (seq == 0 && !originRs.ribIn.empty()) return res;

    RibEntry cand{fullChain, fullCost, seq, spof};

    // Drop entries >=2 generations behind (see the ribIn comment in RibRing).
    auto purgeStale = [&]() {
        auto& v = originRs.ribIn;
        v.erase(std::remove_if(v.begin(), v.end(),
            [&](const RibEntry& e) { return originRs.curSeq - e.seq_no >= 2; }),
            v.end());
    };

    // Seq-advance: adopt the new generation, purge stale, but keep old-gen chains
    // strictly better than this candidate (make-before-break); drop the rest.
    // Same-chain entries are always replaced — cost can increase (echo-path gives
    // a low transient count; full COLLECT corrects it upward), so a stale cheaper
    // same-chain entry is wrong data, not a genuine backup path.
    if (seq > originRs.curSeq) {
        originRs.curSeq = seq;
        clearDegradeIfRecovered(origin);
        if (originRrTrailer) originRs.rr = *originRrTrailer;
        purgeStale();
        auto& v = originRs.ribIn;
        v.erase(std::remove_if(v.begin(), v.end(),
            [&](const RibEntry& e) { return e.chain == cand.chain || !strictlyBetter(e, cand); }),
            v.end());
    }

    // Older-than-current chain that isn't strictly better: stale, ignore it.
    if (seq < originRs.curSeq) {
        bool useful = originRs.ribIn.empty();
        for (auto& e : originRs.ribIn)
            if (strictlyBetter(cand, e)) { useful = true; break; }
        if (!useful) return res;
    }

    // Recovered node: a seq=0 replay carries no seq advance, so adopt the
    // origin's rr here to repopulate empty state.
    if (originRs.rr.empty() && originRrTrailer && !originRrTrailer->empty())
        originRs.rr = *originRrTrailer;

    // Exact chain already held at >= seq: transition already happened.
    for (auto& e : originRs.ribIn)
        if (e.chain == fullChain && e.seq_no >= seq) { res.dup = true; return res; }

    res.propagate = true;

    // Rank against the current generation only, so retained holdovers can't
    // make a valid new-gen chain look inferior (it still must propagate).
    // betterExact, not strictlyBetter: the incumbents are mutually in-margin, so
    // strictlyBetter is flat among them and would leave curBest at whichever
    // entry happens to sit first — making the branch below depend on insertion
    // order rather than cost.
    RibEntry* curBest = nullptr;
    for (auto& e : originRs.ribIn)
        if (e.seq_no == originRs.curSeq
            && (!curBest || betterExact(e, *curBest))) curBest = &e;

    if (!curBest) {
        originRs.ribIn.push_back(cand);                      // first of this gen
    } else if (strictlyBetter(cand, *curBest)) {
        // New best: drop now-inferior current-gen entries and keep holdovers,
        // but still keep sole front-ring guard so the king can still replay
        // every (origin, front-ring).
        dropCurGenKeepingFronts(originRs, fullChain.front());
        originRs.ribIn.push_back(cand);
    } else if (coBest(cand, *curBest)) {
        originRs.ribIn.push_back(cand);
        if ((long)originRs.ribIn.size() > maxEcmpPaths)
            res.propagate = capEcmp(originRs, fullChain);
    } else {
        // Inferior — not propagated. But if no other current-gen chain enters
        // this origin via fullChain.front(), keep it (sole front-ring guard).
        if (!frontRepresented(originRs, fullChain.front(), SIZE_MAX)) {
            originRs.ribIn.push_back(cand);
            selectPreferred(originRs);
        }
        res.propagate = false;
    }

    if (res.propagate) selectPreferred(originRs);
    res.cobest = originRs.ribIn.size();
    return res;
}

size_t MyriadGpRib::knownOrigins() const {
    size_t n = 0;
    for (auto& [origin, rs] : rings) if (rs.preferred != SIZE_MAX) ++n;
    return n;
}

std::vector<MyriadGpRib::ReplayChain> MyriadGpRib::preferredChains() const {
    std::vector<ReplayChain> out;
    out.reserve(rings.size());
    for (auto& [origin, ors] : rings) {
        if (ors.preferred == SIZE_MAX) continue;
        const RibEntry& e = ors.ribIn[ors.preferred];
        if (e.chain.empty()) continue;
        out.push_back({&e.chain, e.chain_cost, e.spof_count, &rr(origin)});
    }
    return out;
}

void MyriadGpRib::dumpTo(std::ostream& f, const std::string& tprefix, bool stats, int nodeId) const {
    std::vector<uint32_t> originIds;
    for (auto& [rid, rs] : rings)
        if (rs.preferred != SIZE_MAX) originIds.push_back(rid);
    std::sort(originIds.begin(), originIds.end());

    if (stats) {
        uint64_t n = 0, sum = 0, mx = 0;
        for (uint32_t rid : originIds) {
            uint64_t c = rings.at(rid).ribIn[rings.at(rid).preferred].chain_cost;
            ++n; sum += c; if (c > mx) mx = c;
        }
        f << tprefix << nodeId << " n=" << n
          << " cost_sum=" << sum << " cost_max=" << mx << "\n";
        return;
    }

    // One line per origin with the primary chain.
    for (uint32_t rid : originIds) {
        const RibRing& rs = rings.at(rid);
        const RibEntry& e = rs.ribIn[rs.preferred];
        size_t ecmp = 0;
        for (auto& c : rs.ribIn) if (c.seq_no == rs.curSeq) ++ecmp;
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
