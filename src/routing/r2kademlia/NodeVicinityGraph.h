#ifndef __ROUTINGSIM_NodeVicinityGraph_H_
#define __ROUTINGSIM_NodeVicinityGraph_H_
#include <unordered_map>
#include <list>

#include "OverlayKey.h"
#include "stateseqno.h"

namespace routingsim {

class nodeVicinityNode {
public:
	const OverlayKey* nodeKey = nullptr;
	unsigned short distance = 0; // distance in hops from node
	stateseqno_t stateseqno;
	std::list<const nodeVicinityNode*> children; // list of pointer to children
};


class nodeVicinityGraph {
public:
	// cannot use constructor to initialize root, because ownNodeKey is only set in initialize()
	void setRoot(const OverlayKey& ownNodeKey) { rootNodeKey= &ownNodeKey; graph[*rootNodeKey]= { rootNodeKey, 0, {} }; }
	/// adds child node to parent node
	bool addNode(const OverlayKey& parentNodeKey, const OverlayKey& childNodeKey, stateseqno_t stateseqno, unsigned short distance);
	void removeNode(const OverlayKey& nodeKey);
	size_t size() const { return graph.size(); }
	const nodeVicinityNode* getNode(const OverlayKey& NodeKey) const;
	auto cbegin() const { return graph.cbegin(); }
	auto cend() const { return graph.cend(); }
	static constexpr unsigned short vicinityDiscoveryRadius = 2; // this is fixed, otherwise it gets too large
private:
	std::unordered_map<OverlayKey,nodeVicinityNode> graph;
	const OverlayKey* rootNodeKey = nullptr;
};


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

