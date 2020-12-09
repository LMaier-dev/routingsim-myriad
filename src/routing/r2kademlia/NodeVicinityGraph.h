#ifndef __ROUTINGSIM_NodeVicinityGraph_H_
#define __ROUTINGSIM_NodeVicinityGraph_H_
#include <unordered_map>
#include <list>

#include "OverlayKey.h"

namespace routingsim {

class nodeVicinityNode {
public:
	const OverlayKey* nodeKey = nullptr;
	unsigned short distance = 0; // distance in hops from node
	std::list<const nodeVicinityNode*> children; // list of pointer to children
};


class nodeVicinityGraph {
public:
	// constructor initializes this node as root
	nodeVicinityGraph(const OverlayKey& ownNodeKey) { graph[ownNodeKey]= { &ownNodeKey, 0, {} }; }
	bool addNode(const OverlayKey& parentNodeKey, const OverlayKey& childNodeKey, unsigned short distance);
	void removeNode(const OverlayKey& nodeKey);
	size_t size() const { return graph.size(); }
	const nodeVicinityNode* getNode(const OverlayKey& NodeKey) const;
private:
	std::unordered_map<OverlayKey,nodeVicinityNode> graph;
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

