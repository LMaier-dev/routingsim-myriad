#include "NodeVicinityGraph.h"
#include <algorithm>
#include <sstream>

namespace routingsim {

std::string nodeVicinityGraph::str() const {
	std::ostringstream out;
	auto rootnode = getRootNode();
    if (!rootnode)
			return "NodeVicinity NULL";
	out << "NodeVicinity " << rootnode->nodeKey->shortStr() << " children: ";
	for (auto childit = rootnode->children.cbegin(); childit != rootnode->children.cend(); childit++) {
		if (auto childptr= *childit; childptr and childptr->nodeKey) {
			out << "* " << childptr->nodeKey->shortStr()
				<< " * (" <<  childptr->distance << "), s:" << childptr->stateseqno << " [";
			for (auto nchildit = childptr->children.cbegin(); nchildit != childptr->children.cend(); nchildit++) {
				if (auto nchildptr= *nchildit; nchildptr and nchildptr->nodeKey) {
					out << nchildptr->nodeKey->shortStr()
						<< " (" <<  nchildptr->distance << "), ";
				}
			} // end for
			out << "] ";
		}
	} // end for
	out << endl;
	return out.str();
}


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

	// EV_TRACE << "vicinity.addNode(): " << childNodeKey.shortStr()
	// 		 << " parent: " << parentNodeKey.shortStr() << " distance:" << distance << endl;

	bool node_added(false);
	bool link_added(false);

	// never add own key as child
	if (childNodeKey == *rootNodeKey) {
		foundnode= &graph.at(*rootNodeKey);
		return { node_added, link_added, foundnode };
	}

	// find parent node and try to add new child
	if (auto parentnode_it= graph.find(parentNodeKey); parentnode_it != graph.end()) {
		auto parentnode_ptr= &parentnode_it->second;
		nodeVicinityNode* childnodeptr(nullptr);
		// check for existence of childnodekey
		if (auto child_it= graph.find(childNodeKey); child_it == graph.end()) {
			// must create new node for child
			auto result= graph.insert(std::make_pair(childNodeKey, nodeVicinityNode()));
			graph[childNodeKey].nodeKey= &(result.first)->first; // add pointer to overlay key at index
			graph[childNodeKey].distance= distance;
			//EV_TRACE << "vicinity.addNode(): added new node " <<  childNodeKey.shortStr() << endl;
			childnodeptr= &graph[childNodeKey];
			node_added= true;
		}
		else { // entry for child exists already
			childnodeptr= &(child_it->second);
			// in case this node is closer than before, update the distance
			unsigned int old_distance= child_it->second.distance;
			//EV_TRACE << "vicinity.addNode(): node exists already, distance " <<  old_distance << endl;
			// smallest distance determines level of node
			if (distance < old_distance) {
				child_it->second.distance= distance;
				link_added= true;
			}
		}

		foundnode= childnodeptr;
		auto& childrenlist(parentnode_ptr->children);
		// check for existing child node in children list of parent (if exists: link already known)
		// perform binary search and then insertion for efficient lookup in large vicinities
		auto child_it= std::lower_bound(childrenlist.cbegin(),childrenlist.cend(), childnodeptr, [](auto p1, auto p2) { return *(p1->nodeKey) < *(p2->nodeKey); });
		// if not found, add (parent, child) as new link
		if (child_it == childrenlist.cend() or *child_it != childnodeptr) {
			childrenlist.insert(child_it, childnodeptr);
			link_added= true;
			foundnode->parents++;
			//EV_TRACE << "vicinity.addNode(): added (" << parentNodeKey.shortStr() << "," << childNodeKey.shortStr() << ")" << endl;
		}
		//ASSERT(std::is_sorted(childrenlist.cbegin(),childrenlist.cend(), [](auto p1, auto p2) { return *(p1->nodeKey) < *(p2->nodeKey); }));

		//  do not add the reverse link because the node at the other end will report the link
	}
	else { // parent node not found
		EV_ERROR << "cannot add child " << childNodeKey.shortStr() << ", because parent " << parentNodeKey << " not found" << endl;
	}

	//EV_TRACE << "vicinity.addNode(): " << childNodeKey.shortStr() << (node_added ? " added node," : " node existed,") << (link_added ? " added link" : " no link added") << endl;

	return { node_added, link_added, foundnode };
}


void
nodeVicinityGraph::removeNode(const OverlayKey& nodeKey)
{
	EV_TRACE << "vicinity.removeNode(): " << nodeKey.shortStr() << endl;
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
			if (auto pc_it= std::lower_bound(graphit->second.children.cbegin(), graphit->second.children.cend(), node, [](auto p1, auto p2) { return *(p1->nodeKey) < *(p2->nodeKey); });
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


/// removes a link between parentNodeKey and childNodeKey
/// it may be the case that the childNode is an orphan after the removal
/// then it will be erased automatically
/// @return true in case link was removed, false if not found
bool
nodeVicinityGraph::removeLink(const OverlayKey &parentNodeKey, const OverlayKey &childNodeKey) {
	// lookup parent node
	//EV_TRACE << "nodeVicinityGraph::removeLink(" << parentNodeKey.shortStr() << "," << childNodeKey.shortStr() << ") called" << endl;
	if (auto graphit= graph.find(parentNodeKey); graphit != graph.end()) {
		nodeVicinityNode* parentnode(&graphit->second);
		graphit = graph.find(childNodeKey);
		if (graphit != graph.end()) {
			nodeVicinityNode* childnode(&graphit->second);
			if (not parentnode->children.empty()) {
				if (auto child_it = std::lower_bound(parentnode->children.cbegin(), parentnode->children.cend(), childnode,
													 [](auto p1, auto p2) {
														 return *(p1->nodeKey) < *(p2->nodeKey);
													 });
					child_it != parentnode->children.cend() and
					*child_it == childnode) {
					// remove link
					EV_TRACE << "nodeVicinity: removing link <"
							 << parentNodeKey.shortStr()
							 << "," << childNodeKey.shortStr() << ">" << endl;
					parentnode->children.erase(child_it);
					if (childnode->parents > 0) {
						childnode->parents--;
						if (childnode->parents == 0) {
							// last parent removed we can delete this node now
							EV_TRACE << "nodeVicinity: removing node " << childnode->nodeKey->shortStr() << " has no parents anymore" << endl;
							graph.erase(graphit);
							return true;
						}
					} else {
						EV_ERROR << "nodeVicinity: childnode->parents == " << childnode->parents << endl;
						ASSERT(childnode->parents > 0);
					}
					// update distance if this was a direct link
					if (parentnode->nodeKey == rootNodeKey) {
						ASSERT(childnode->distance == 1);
						childnode->distance= 2;
					}
					//EV_TRACE << "nodeVicinityGraph::removeLink(" << parentNodeKey.shortStr() << "," << childNodeKey.shortStr() << ") done. " << str() << endl;
					return true;
				}
			}
		}
	}
	return false;
}


/// @return true and a path (may be empty for ULN) to given node in case one exists in the
/// nodeVicinityGraph, otherwise it returns false and also an empty path
std::pair<bool, std::vector<OverlayKey>>
nodeVicinityGraph::findPath(const OverlayKey &NodeKey) const
{
	//EV_TRACE << "nodeVicinityGraph::findPath(" << NodeKey.shortStr() << ") " << str() << endl;
	auto graphit = graph.find(NodeKey);
	if (graphit != graph.cend()) {
		const nodeVicinityNode* destnode(&graphit->second);
		const nodeVicinityNode* rootnode= getRootNode();

		// need to traverse all children
		if (destnode) {

			if (destnode->distance == 1) { // direct link may still exist if 2nd hop broke
				// use a fast binary search first
				if (auto children_it = std::lower_bound(rootnode->children.cbegin(), rootnode->children.cend(), destnode,
														[](auto p1, auto p2) { return *(p1->nodeKey) < *(p2->nodeKey);});
					children_it != rootnode->children.cend() and
					*children_it == destnode) {
					return {true, {}}; // found destnode
				}
			}

			for (auto childnode : rootnode->children) {

				if (childnode) {
					// compare to childnode first, should be covered by binary search above,
					// however, it can be the case that the distance information is inaccurate
					if (childnode->nodeKey == destnode->nodeKey) {
						return {true, {}}; // found destnode
					}
					// perform binary seach in children of this child
					// if found it will return the first match (according to numeric sort of overlay keys)
					if (auto child_it = std::lower_bound(childnode->children.cbegin(), childnode->children.cend(), destnode, [](auto p1, auto p2) { return *(p1->nodeKey) < *(p2->nodeKey); });
					    child_it != childnode->children.cend() and *child_it == destnode) {
						return {true, {*childnode->nodeKey}};
					}
				}

			} // end for every direct child
		}
		else {
			EV_WARN << "nodeVicinityGraph::findPath() Node " << NodeKey.shortStr() << " not found in vicinity" << endl;
		}
	}
	else {
		EV_WARN << "nodeVicinityGraph::findPath() Node " << NodeKey.shortStr() << " not found in vicinity" << endl;
	}

	return {false, {}};
}


} //namespace
