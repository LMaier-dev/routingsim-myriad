#include "R2Kademlia.h"

/*********************************************************
 * This file contains PathID related code from the 
 * R2Kademlia main class
 *********************************************************/

namespace routingsim {


// returns false if path already exists, returns true if path was installed
bool R2Kademlia::checkForPathInstallation(KadPacket& packet)
{
	if ( usePathIDs == false )
		return false;
	// will install path in data forward direction
	// ASSERT(packet.hasRemainingHops()); // this will be true because checkForPathInstallation() is called only then
	KadContact tmpcontact(packet.getTarget(),
			      packet.getSrcPath(packet.getProgress()+1, packet.getRouteArraySize()-1));

	if (auto incomingPID= fwdTablePathID.addEntry(&tmpcontact, true, tmpcontact.getPathLength() <= discoveryRadius);
	    incomingPID != PathID::UndefinedPathID) {
		// no need to install NId to PID mapping as this is no local contact
		return true;
	}
	else
		return false;
}


// this creates potential additional entries for PathIDs
// note that the provided contact is of temporary nature and need not be a part of the routing table
void R2Kademlia::vicinityPathIDCompletion(const KadContact& neighborhoodContact)
{
	const nodeVicinityNode* newnode= vicinity.getNode(neighborhoodContact.getKey());
	EV_TRACE << "vicinityPathIDCompletion(" << neighborhoodContact.getKey().shortStr() << ")" << endl;

	// check for further PID entries that need to be created leading to or starting from this neighborhood contact
	if (newnode != nullptr and newnode->distance>0 and newnode->distance <= discoveryRadius) {

		//EV_TRACE << "vicinityPathIDCompletion - collecting parent nodes" << endl;

		// now traverse back toward the root node to cover all paths that lead to this node
		// we need a temporary graph structure for this, therefore we use unique_ptr
		// since we also need the root node at level 0, we need discoveryRadius+1 levels in total
		std::list<std::unique_ptr<nodeVicinityNode>> nodeSet[discoveryRadius+1];
		// add the root node
		std::unique_ptr<nodeVicinityNode> rootnode= std::make_unique<nodeVicinityNode>();
		rootnode->nodeKey= vicinity.getNode(myOverlayKey)->nodeKey;
		rootnode->distance= 0;
		nodeSet[0].push_back(std::move(rootnode));

		// add the current node
		std::unique_ptr<nodeVicinityNode> tmpnewnode= std::make_unique<nodeVicinityNode>();
		tmpnewnode->nodeKey= newnode->nodeKey;
		tmpnewnode->distance= newnode->distance;
		nodeSet[newnode->distance].push_back(std::move(tmpnewnode));

		for (unsigned int level= newnode->distance; level > 0; level--) {

			for (auto current_node_it= nodeSet[level].cbegin();
			     current_node_it != nodeSet[level].cend();
			     current_node_it++) {

				// iterate through all potential parent nodes for current_node
				for (auto all_nodes_it= vicinity.cbegin(); all_nodes_it != vicinity.cend(); all_nodes_it++) {

					// check for parents at depth = level-1
					if (all_nodes_it->second.distance+1U == level) {
						// if the current_node is among the children of this node, the latter is a parent
						auto child_it= std::find_if(all_nodes_it->second.children.cbegin(),
									    all_nodes_it->second.children.cend(),
									    [&](auto v){ return v->nodeKey == (*current_node_it)->nodeKey; });

						if (child_it != all_nodes_it->second.children.cend()) {
							
							// found the current's node parent in all nodes (all_nodes_it points to parent)
							const auto& existing_parent= std::find_if(nodeSet[level-1].cbegin(),
												  nodeSet[level-1].cend(),
												  [&](const auto& v){ return v->nodeKey == all_nodes_it->second.nodeKey; });

							if (existing_parent == nodeSet[level-1].cend()) {
								// no parent exists already in nodeSet, add it
								std::unique_ptr<nodeVicinityNode> newparentnode= std::make_unique<nodeVicinityNode>();
								newparentnode->nodeKey= &all_nodes_it->first; // nodeKey should point to the vicinity graph instances
								newparentnode->distance= level-1;
								newparentnode->children= { current_node_it->get() }; // children point to these temporary nodes
								//EV_TRACE << "vicinityPathIDCompletion - create new parent " << newparentnode->nodeKey->shortStr() << " in node set @level " << level -1 << endl;
								// if not already present, add node to parent set
								nodeSet[level-1].push_back(std::move(newparentnode));
							}
							else { // just add this child to the parent node unless already there
								//EV_TRACE << "vicinityPathIDCompletion - found parent " << (*existing_parent)->nodeKey->shortStr() << " in node set @level " << level -1 << endl;
								if (std::find_if((*existing_parent)->children.cbegin(),
										 (*existing_parent)->children.cend(),
										 [&](auto v){ return v->nodeKey == current_node_it->get()->nodeKey; }) == (*existing_parent)->children.cend())
								    (*existing_parent)->children.push_back( current_node_it->get() );
							}
						}
					} // endif node has the correct depth
				} // end for iterating over all potential parent nodes one level above
			} // end for iterating over all nodes at the current level
		} // end for iterating over all levels

		// for (unsigned i= 0; i <= discoveryRadius; i++) {
		// 	EV_TRACE << "vicinityPathIDCompletion - collected nodes @level " << i;
		// 	for (auto cit= nodeSet[i].cbegin(); cit!= nodeSet[i].cend(); cit++)
		// 		EV_TRACE << ' ' << (*cit)->nodeKey->shortStr();
		//         EV_TRACE << endl;
		// }

		//EV_TRACE << "vicinityPathIDCompletion - collecting child nodes" << endl;

		// now collect all children starting from here
		for (unsigned int level= newnode->distance; level < discoveryRadius; level++) {
			
			for (auto node_it= nodeSet[level].cbegin(); node_it != nodeSet[level].cend(); node_it++) {
				// create a new node temporary for each child of the current node unless it exists already
				auto currentTmpNode_p(node_it->get());

				const nodeVicinityNode* currentVicinityNode= vicinity.getNode(*(currentTmpNode_p->nodeKey));

				// now iterate over all children of this node (must do this in vicinity graph)
				for (auto child_node : currentVicinityNode->children) {
					
					if (child_node->distance == level+1) { // check whether child is at the correct level
						// in case child is on lesser level, we ignore it now, because shortest path is preferred

						// check if child exists in next level already, otherwise create this node
						auto existing_child= std::find_if(nodeSet[level+1].cbegin(),
										  nodeSet[level+1].cend(),
										  [&](const auto& v){ return v->nodeKey == child_node->nodeKey; });
						if (existing_child == nodeSet[level+1].cend()) {
							// must create new child
							std::unique_ptr<nodeVicinityNode> newchildnode= std::make_unique<nodeVicinityNode>();
							newchildnode->nodeKey= child_node->nodeKey; // nodeKey should point to the vicinity graph instances
							newchildnode->distance= level;
							// add child to current node
							currentTmpNode_p->children.push_back(newchildnode.get());

							nodeSet[level+1].push_back(std::move(newchildnode));
						}
						else {
							// add existing child to current node
							currentTmpNode_p->children.push_back(existing_child->get());
						}
					} // endif child at correct level

				} // end for all children
			} // end for
		} // end for every following level

		// for (unsigned i= 0; i <= discoveryRadius; i++) {
		// 	EV_TRACE << "vicinityPathIDCompletion - collected nodes @level " << i;
		// 	for (auto cit= nodeSet[i].cbegin(); cit!= nodeSet[i].cend(); cit++)
		// 		EV_TRACE << ' ' << (*cit)->nodeKey->shortStr();
		//         EV_TRACE << endl;
		// }

		// now traverse all paths starting from the root node
		std::list<const OverlayKey*> currentpath;
		unsigned int currentLevel= 0;
		// iterator over children: note that childiter_state[i] iterates over nodes at level i+1!
		decltype(nodeVicinityNode::children)::const_iterator childiter_state[discoveryRadius];
		childiter_state[currentLevel]= nodeSet[0].front()->children.cbegin();
		while (childiter_state[0] != nodeSet[0].front()->children.cend())
		{
			//EV_TRACE << "vicinityPathIDCompletion - currentLevel: " << currentLevel << endl;

			if (currentLevel > 0 and childiter_state[currentLevel] == (*childiter_state[currentLevel-1])->children.cend()) {
				currentpath.pop_back();
				// end of children at currentLevel, go up one level and advance to next child
				currentLevel--;
				// advance to next child
				childiter_state[currentLevel]++;
				//EV_TRACE << "vicinityPathIDCompletion - going up to: " << currentLevel << endl;

				continue;
			}

			// currentNodep is at currentLevel+1 since it is a child of the node at currentLevel
			auto currentNodep= *childiter_state[currentLevel];
			//EV_TRACE << "vicinityPathIDCompletion - currentNodep: " << currentNodep->nodeKey->shortStr() << endl;
			// if at or beyond "new" node add pathID  for path
			if (currentLevel+1 >= newnode->distance) {
				kad_path basicpath;
				for (auto v : currentpath)
					basicpath.push_back(*v);
				KadContact tmpContact(*(currentNodep->nodeKey), basicpath);
				//EV_TRACE << "vicinityPathIDCompletion - trying to add " << tmpContact.getPath() << " " << tmpContact.getKey().shortStr() << endl;
				fwdTablePathID.addEntry(&tmpContact, false, true);
			}
			// enter next level of current node by using its children
			if (currentLevel < discoveryRadius and not currentNodep->children.empty()) {
				// start this new level from the beginning
				// enter next level (== discoverRadius at most)
				currentLevel++;
				// initialize children iterator for next level
				childiter_state[currentLevel]= currentNodep->children.cbegin();		
				// append current child to path
				currentpath.push_back(currentNodep->nodeKey);
			}
			else { // arrived at lowest level or no next level possible

				// advance to next child
				childiter_state[currentLevel]++;

			} // end else at lowest level

		} // end while
	}
}

} // end namespace
