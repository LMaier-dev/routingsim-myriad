#ifndef __ROUTINGSIM_NodeVicinityGraph_H_
#define __ROUTINGSIM_NodeVicinityGraph_H_
#include <unordered_map>

#include "OverlayKey.h"
#include "stateseqno.h"

namespace routingsim {

class nodeVicinityNode {
public:
	const OverlayKey* nodeKey = nullptr; // NodeID for this node
	unsigned short distance = 0; // (shortest) distance from root node in hops to (this) node
	stateseqno_t stateseqno = stateseqno_t::invalidSeqNo; // last seen state seq no when communicated with this node
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
	// cannot use constructor to initialize root, because ownNodeKey is only set in initialize()
	void setRoot(const OverlayKey& ownNodeKey) { rootNodeKey= &ownNodeKey; graph[*rootNodeKey]= { rootNodeKey, 0, {} }; }
	/// adds child node to parent node
	std::tuple<bool, bool, nodeVicinityNode*> addNode(const OverlayKey& parentNodeKey, const OverlayKey& childNodeKey, unsigned short distance);
	void removeNode(const OverlayKey& nodeKey);
	bool removeLink(const OverlayKey& parentNodeKey, const OverlayKey& childNodeKey);
	size_t size() const { return graph.size(); }
	const nodeVicinityNode* getNode(const OverlayKey& NodeKey) const;
	bool updateNodeStateSeqNo(const OverlayKey& NodeKey, stateseqno_t newseqno);
	auto cbegin() const { return graph.cbegin(); }
	auto cend() const { return graph.cend(); }
	static constexpr unsigned short vicinityDiscoveryRadius = 2; // this is fixed, otherwise it gets too large
private:
	std::unordered_map<OverlayKey,nodeVicinityNode> graph;
	const OverlayKey* rootNodeKey = nullptr;
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
