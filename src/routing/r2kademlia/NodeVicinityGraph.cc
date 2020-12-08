#include "NodeVicinityGraph.h"

namespace routingsim {

// adds a new childnode to the parent
void
nodeVicinityGraph::addNode(const OverlayKey& parentNodeKey, const OverlayKey& childNodeKey, unsigned short distance) {
	ASSERT(distance > 0U);

	if (auto it= graph.find(parentNodeKey); it != graph.end()) {
		// in case this node is closer than before, update the distance
		if (distance < it->second.distance)
			it->second.distance= distance;

		const nodeVicinityNode* childnodeptr(nullptr);
		// check for existence of childnodekey
		if (auto child_it= graph.find(childNodeKey); child_it == graph.end()) {
			// must create new node for children
			graph[childNodeKey]= { &child_it->first, distance, {} };
			childnodeptr= &graph[childNodeKey];
		}
		else { // entry for child exists already
			childnodeptr= &child_it->second;
		}

		// add new node as child to parent
		auto child_it= it->second.children.begin();
		// check for existing child node
		for (; child_it != it->second.children.end(); child_it++) {
			if (*child_it == childnodeptr)
				break;
		}
		// if not found, add it
		if (child_it == it->second.children.end())
			it->second.children.push_back(childnodeptr); 
	}
	else { // parent node not found
		EV_ERROR << "cannot add child " << childNodeKey.shortStr() << ", because parent " << parentNodeKey << " not found" << endl;
	}
}

void
nodeVicinityGraph::removeNode(const OverlayKey& nodeKey)
{
	// lookup node
	if (auto it= graph.find(nodeKey); it != graph.end()) {
		const nodeVicinityNode* node(&it->second);
		// now find all nodes at node.distance-1, because they may point to this node as being its child
		for (auto graphit= graph.begin(); graphit != graph.end(); graphit++) {
			if (graphit->second.distance+1 == node->distance) {
				// remove any potential referrals to this node
				graphit->second.children.remove(node);
			}
		}
		graph.erase(it);
	}
	else {
		EV_WARN << "removeNode(): " << nodeKey.shortStr() << " node not found" << endl;
	}
		
}


} //namespace
