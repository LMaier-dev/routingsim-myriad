#ifndef __ROUTINGSIM_NodeVicinityGraph_H_
#define __ROUTINGSIM_NodeVicinityGraph_H_
#include <unordered_map>

#include "OverlayKey.h"
#include "stateseqno.h"

namespace routingsim {

class nodeVicinityNode {
	public:
		const OverlayKey* nodeKey = nullptr; // pointer to NodeID for this node (typically the key in the graph structure)
		unsigned short distance = 0; // (shortest) distance from root node in hops to (this) node
		stateseqno_t stateseqno = stateseqno_t::invalidSeqNo; // last seen state seq no when communicated with this node
		unsigned long parents = 0UL;
		OverlayKey twoHopULNXorSum;
		std::vector<const nodeVicinityNode*> children; // (sorted) list of pointers to children (to allow for binary search)
};


/***********************************************
 * The following class describes a directed graph
 * It is mainly used to precalculate PathIDs for all potential paths
 * in the 2-hop vicinity. It tries to be memory efficient as 2-hop vicinities
 * may become quite large in large graphs (10000s of nodes!)
 **/
class nodeVicinityGraph {
public:
	static constexpr unsigned short vicinityDiscoveryRadius = 2; // this is fixed, otherwise it gets too large
	// cannot use constructor to initialize root, because ownNodeKey is only set in initialize()
	void setRoot(const OverlayKey& ownNodeKey) { rootNodeKey= &ownNodeKey; graph[*rootNodeKey]= { rootNodeKey, 0, {}, 0, OverlayKey::UNSPECIFIED_KEY, {} }; }
	/// adds child node to parent node
	std::tuple<bool, bool, nodeVicinityNode*> addNode(const OverlayKey& parentNodeKey, const OverlayKey& childNodeKey, unsigned short distance,  const OverlayKey& node_xorsum);
	void removeNode(const OverlayKey& nodeKey);
	bool removeLink(const OverlayKey& parentNodeKey, const OverlayKey& childNodeKey);
	size_t size() const { return graph.size(); }
	const nodeVicinityNode* getNode(const OverlayKey& NodeKey) const;
	const nodeVicinityNode* getRootNode() const { return getNode(*rootNodeKey); };
	bool updateNodeStateSeqNo(const OverlayKey& NodeKey, stateseqno_t newseqno);
	void updateNodeXORsum(const OverlayKey& NodeKey, const OverlayKey& newxorsum);

	std::pair<bool, std::vector<OverlayKey>> findPath(const OverlayKey& NodeKey) const;
	auto cbegin() const { return graph.cbegin(); }
	auto cend() const { return graph.cend(); }
	std::string str() const;
	OverlayKey xorsumtwohopULNs() const { return xorsum; };
private:
	std::unordered_map<OverlayKey,nodeVicinityNode> graph;
	const OverlayKey *rootNodeKey = nullptr;
	OverlayKey xorsum = OverlayKey::ZERO; // will be updated on every change
};


/// Updates state sequence number for node with given NodeKey if necessary
/// @returns true if newseqno is larger than the one that was stored
inline
bool
nodeVicinityGraph::updateNodeStateSeqNo(const OverlayKey& NodeKey, stateseqno_t newseqno)
{
	if (auto it= graph.find(NodeKey); it != graph.end()) {
		if (newseqno > it->second.stateseqno) {
			it->second.stateseqno = newseqno;
			return true;
		} else { // not larger or equals
			return false;
		}
	}
	return false;
}


/// Updates xorsum of node with given NodeKey (newxorsum must not be unsepcified)
inline
void
nodeVicinityGraph::updateNodeXORsum(const OverlayKey& NodeKey, const OverlayKey& newxorsum)
{
	// will not update if new xorsum is unspecified
	if (newxorsum.isUnspecified())
		return;

	if (auto it= graph.find(NodeKey); it != graph.end()) {
			it->second.twoHopULNXorSum = newxorsum;
	}
}


inline
const nodeVicinityNode*
nodeVicinityGraph::getNode(const OverlayKey& NodeKey) const
{
	if (auto it= graph.find(NodeKey); it != graph.end()) {
		return &it->second;
	}
	else { // entry not found
		return nullptr;
	}
}

} //namespace

#endif // __ROUTINGSIM_R2Kademlia_H_
