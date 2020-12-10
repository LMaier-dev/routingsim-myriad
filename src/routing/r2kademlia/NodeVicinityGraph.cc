#include "NodeVicinityGraph.h"

namespace routingsim {

// adds a new childnode to the parent or at least the link if the childnode is already known
// @returns whether something has been changed (link or node added)
bool
nodeVicinityGraph::addNode(const OverlayKey& parentNodeKey, const OverlayKey& childNodeKey, unsigned short distance) {
	ASSERT(distance > 0U);
	bool modified(false);

	if (auto parentnode_it= graph.find(parentNodeKey); parentnode_it != graph.end()) {

		const nodeVicinityNode* childnodeptr(nullptr);
		// check for existence of childnodekey
		if (auto child_it= graph.find(childNodeKey); child_it == graph.end()) {
			// must create new node for child
			auto result= graph.insert(std::make_pair(childNodeKey, nodeVicinityNode()));
			graph[childNodeKey].nodeKey= &(result.first)->first;
			graph[childNodeKey].distance= distance;
			EV_TRACE << "vicinity.addNode(): " << childNodeKey.shortStr() << " distance:" << distance << endl;
			childnodeptr= &graph[childNodeKey];
			modified= true;
		}
		else { // entry for child exists already
			childnodeptr= &(child_it->second);
			// in case this node is closer than before, update the distance
			unsigned int old_distance= child_it->second.distance;
			if (distance < old_distance) {
				child_it->second.distance= distance;
				modified= true;
				// now remove it from any children list in further distance
				// find all nodes at node.distance-1, because they may point to this node being its child
				for (auto graphit= graph.begin(); graphit != graph.end(); graphit++) {
					if (graphit->second.distance+1U == old_distance) {
						// remove any potential referrals to this node
						graphit->second.children.remove(&child_it->second);
					}
				}
			}

		}

		// add new node as child to parent in case it is not leading back
		if (distance <= parentnode_it->second.distance)
			return modified;

		auto child_it= parentnode_it->second.children.begin();
		// check for existing child node in list
		for (; child_it != parentnode_it->second.children.end(); child_it++) {
			if (*child_it == childnodeptr)
				break;
		}
		// if not found, add it
		if (child_it == parentnode_it->second.children.end()) {
			parentnode_it->second.children.push_back(childnodeptr);
			modified= true;
		}
	}
	else { // parent node not found
		EV_ERROR << "cannot add child " << childNodeKey.shortStr() << ", because parent " << parentNodeKey << " not found" << endl;
	}

	return modified;
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
