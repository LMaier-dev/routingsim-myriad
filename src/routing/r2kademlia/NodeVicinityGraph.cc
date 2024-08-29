#include "NodeVicinityGraph.h"
#include <algorithm>

namespace routingsim {

// adds a new childnode to the parent or at least the link if the childnode is already known
// also updates distance if childnode already known
// @returns (node added, link added, node*)
std::tuple<bool, bool, nodeVicinityNode*>
nodeVicinityGraph::addNode(const OverlayKey& parentNodeKey, const OverlayKey& childNodeKey, unsigned short distance) {
	ASSERT(distance > 0U);

	nodeVicinityNode* foundnode(nullptr);
	// nodes further away than the vicinityDiscoveryRadius are ignored
	if (distance > vicinityDiscoveryRadius)
		return { false, false, foundnode };

	//EV_TRACE << "vicinity.addNode(): " << childNodeKey.shortStr()
	//	 << " parent: " << parentNodeKey.shortStr() << " distance:" << distance << endl;

	bool node_added(false);
	bool link_added(false);

	// never add own key as child
	if (childNodeKey == *rootNodeKey) {
		foundnode= &graph.at(*rootNodeKey);
		return { node_added, link_added, foundnode };
	}

	if (auto parentnode_it= graph.find(parentNodeKey); parentnode_it != graph.end()) {

		nodeVicinityNode* childnodeptr(nullptr);
		// check for existence of childnodekey
		if (auto child_it= graph.find(childNodeKey); child_it == graph.end()) {
			// must create new node for child
			auto result= graph.insert(std::make_pair(childNodeKey, nodeVicinityNode()));
			graph[childNodeKey].nodeKey= &(result.first)->first;
			graph[childNodeKey].distance= distance;
			// EV_TRACE << "vicinity.addNode(): added new node " <<  childNodeKey.shortStr() << endl;
			childnodeptr= &graph[childNodeKey];
			node_added= true;
		}
		else { // entry for child exists already
			childnodeptr= &(child_it->second);
			// in case this node is closer than before, update the distance
			unsigned int old_distance= child_it->second.distance;
			// EV_TRACE << "vicinity.addNode(): node exists already, distance " <<  old_distance << endl;
			if (distance < old_distance) {
				child_it->second.distance= distance;
				link_added= true;
			}
		}

		foundnode= childnodeptr;
		auto& childrenlist(parentnode_it->second.children);
		// check for existing child node in children list of parent (if exists: link already known)
		auto child_it= std::lower_bound(childrenlist.cbegin(),childrenlist.cend(), childnodeptr);
		// if not found, add (parent, child) as new link
		if (child_it == childrenlist.cend() or *child_it != childnodeptr) {
			childrenlist.insert(child_it, childnodeptr);
			link_added= true;
		}
		//ASSERT(std::is_sorted(childrenlist.cbegin(),childrenlist.cend()));

	}
	else { // parent node not found
		EV_ERROR << "cannot add child " << childNodeKey.shortStr() << ", because parent " << parentNodeKey << " not found" << endl;
	}

	//EV_TRACE << "vicinity.addNode(): " << childNodeKey.shortStr() << (modified ? " modified" : " not modified") << endl;

	return { node_added, link_added, foundnode };
}


void
nodeVicinityGraph::removeNode(const OverlayKey& nodeKey)
{
	//EV_TRACE << "vicinity.removeNode(): " << nodeKey.shortStr() << endl;
	// never remove own root node
	if (nodeKey == *rootNodeKey) {
		EV_WARN << "vicinity.removeNode(): trying to remove root node" << endl;
		return;
	}

	// lookup node
	if (auto it= graph.find(nodeKey); it != graph.end()) {
		const nodeVicinityNode* node(&it->second);
		// now find all nodes at node.distance-1, because they may point to this node as being its child
		for (auto graphit= graph.begin(); graphit != graph.end(); graphit++) {
		  // remove any potential referrals to this node
		  if (not graphit->second.children.empty()) {
			if (auto pc_it= std::lower_bound(graphit->second.children.cbegin(), graphit->second.children.cend(), node);
			    pc_it != graphit->second.children.cend() and *pc_it == node) {
				graphit->second.children.erase(pc_it);
			}
		  }
		}
		graph.erase(it);
	}
	else {
		EV_WARN << "vicinity.removeNode(): " << nodeKey.shortStr() << " node not found" << endl;
	}
		
}


} //namespace
