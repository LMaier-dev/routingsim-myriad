//
// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU Lesser General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
// 
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU Lesser General Public License for more details.
// 
// You should have received a copy of the GNU Lesser General Public License
// along with this program.  If not, see http://www.gnu.org/licenses/.
// 

#include "NetworkTopology.h"

#include <cchannel.h>
#include <boost/tokenizer.hpp>
#include <boost/lexical_cast.hpp>
#include <boost/foreach.hpp>
#include <boost/unordered_set.hpp>
#include <map>
#include <fstream>
#include <cmodule.h>

namespace routingsim {

using namespace boost;
using namespace std;

// ============================================================================
// Topology Helpers
// ============================================================================

/**
 * TopologyLink class describes a link in the graph and the actual status in the
 * topology
 */
class TopologyLink {
private:
	string name; // link name
	float weight; // link weight
	bool connected; // true, if a gate exists
	bool active; // true, if forwarding is active in Node (de-)multiplexer
public:
	inline TopologyLink() :
	name(""), weight(1.0), connected(false), active(false) {
	}
	inline ~TopologyLink() {

	}
	inline bool isConnected() const {
		return connected;
	}
	inline void setConnected(bool state) {
		connected = state;
	}
	inline bool isActive() const {
		return active;
	}
	inline void setActive(bool state) {
		active = state;
	}
	inline float getWeight() const {
		return weight;
	}
};

/**
 * Set of links.
 */
class NetworkTopology::TopologyNode {
public:
	typedef unordered_map<nodeid_t, TopologyLink*> linkset_t;
	linkset_t linkset;
	typedef unordered_map<string,string> properties_t;
	properties_t properties;

public:
	inline TopologyNode() :
	linkset() {
	}

	inline ~TopologyNode() {
		// free all link properties
		for (linkset_t::iterator i = linkset.begin(); i != linkset.end(); i++)
			delete (i->second);

		// cleanup link-set
		linkset.clear();
	}

	/// Returns the number of links
	inline size_t count() const {
		return linkset.size();
	}

	/// add a new link to the set
	inline TopologyLink* addLink(nodeid_t id) {
		linkset_t::iterator i = linkset.find(id);
		if (i == linkset.end())
			i = linkset.insert(make_pair(id, new TopologyLink())).first;
		return i->second;
	}

	/// returns a link from the set
	inline TopologyLink* getLink(nodeid_t node) {
		if (linkset.find(node) != linkset.end())
			return linkset.find(node)->second;
		else
			return NULL;
	}
};

/**
 * DijkstraStatus provides the helper data structure to calculate
 * shortest paths in NetworkTopology::getDistance
 */
class DijkstraStatus {
private:
	bool visited; //has the node been visited?
	float weight; //cost as sum of weights
	NetworkTopology::TopologyNode *ls;

public:
	inline DijkstraStatus() :
	visited(false), weight(FLT_MAX) {
	}
	inline DijkstraStatus(NetworkTopology::TopologyNode *ls) :
				visited(false), weight(FLT_MAX), ls(ls) {
	}
	inline ~DijkstraStatus() {
	}
	inline void markVisited() {
		visited = true;
	}
	inline bool hasBeenVisited() const {
		return visited;
	}
	inline void setWeight(cost_t weight) {
		this->weight = weight;
	}
	inline cost_t getWeight() const {
		return weight;
	}
	inline NetworkTopology::TopologyNode* getLinkSet() {
		return ls;
	}
};

// ============================================================================
// Network Topology Class
// ============================================================================

/// constructor
NetworkTopology::NetworkTopology() :
			spath_available(false), numberNodes(0), numberLinks(0) {
}

/// destructor
NetworkTopology::~NetworkTopology() {
	unordered_map<nodeid_t, TopologyNode *>::iterator it;
	unordered_map<nodeid_t, cModule*>::iterator modMapIt;

	// stop modules and remove them
	// NOTE: happens without explicit call too ...
	for (modMapIt = moduleMap.begin(); modMapIt != moduleMap.end(); modMapIt++) {
		modMapIt->second->callFinish();
		modMapIt->second->deleteModule();
	}

	// remove graph / topology data structure
	for (it = adjacencyList.begin(); it != adjacencyList.end(); it++) {
		delete (it->second);
	}
	adjacencyList.clear();
}

// read a file from topology generator
// format:
// 1st line: number of nodes\tnumber of links
// following lines: src node\tdest node
// TODO: Input validation and sanity checks
void NetworkTopology::readTopology(string path) {
	ifstream topologyFile(path.c_str());
	string line;
	int numNodes = 0, numLinks = 0;
	nodeid_t fromNode, toNode;
	int lineNumber = 0;

	if (!topologyFile) {
		if (getEnvir()->isLoggingEnabled())
			EV << "Could not open" << path << endl;
	}

	while (std::getline(topologyFile, line)) {
		int tokenNumber = 0;
		lineNumber++;
		// escape char: "\", seperator "\t" and " ", quote chars " and '
		escaped_list_separator<char> sep("\\","\t ","\"'");
		typedef boost::tokenizer<escaped_list_separator<char> > tokenizer_t;
		tokenizer_t tok(line,sep);
		bool nodep=false;
		for (tokenizer_t::iterator it = tok.begin(); it != tok.end(); it++) {
			tokenNumber++;

			// read header line
			if (lineNumber == 1 && tokenNumber == 1) {
				numNodes = boost::lexical_cast<int>(*it);
				continue;
			}
			if (lineNumber == 1 && tokenNumber == 2) {
				numLinks = boost::lexical_cast<int>(*it);
				break;
			}

			// handle regular line
			if (tokenNumber == 1) {
				if (*it=="node") nodep = true;
				else fromNode = boost::lexical_cast<long long>(*it);
			}

			if (tokenNumber == 2 && !nodep) {
				toNode = boost::lexical_cast<long long>(*it);
				if (fromNode != toNode) {
					addLink(fromNode, toNode);
					addLink(toNode, fromNode);
					EV_DETAIL << "Added link from " << fromNode << " to " << toNode << "and back" << endl;
				} else {
					EV_WARN << "Attention: omitting reflexive link ("<< fromNode<<","<<toNode<<" as implicitly contained!" << endl;
				}
			}

			if (tokenNumber >= 2 && nodep) {
				string s = *it;
				if (tokenNumber==2) {
					toNode = boost::lexical_cast<long long>(*it);
					continue;
				}
				string::size_type i = s.find(':');
				if (i == string::npos) continue;
				string key = s.substr(0,i);
				string value = s.substr(i+1);
				adjacencyList[toNode]->properties[key]=value;
				EV_DETAIL << "Node " << toNode << ": key='" << key << "' value='" << value << "'" << endl;
			}

		}
	}
	numberNodes = numNodes;
	numberLinks = numLinks;
}

bool NetworkTopology::readShortestPaths(string shortestPathFile) {
	ifstream f(shortestPathFile.c_str(), ifstream::in | ifstream::binary);
	if (!f) {
		EV << "Could not open shortest-paths " << shortestPathFile << endl;
		return false;
	}
	numberNodes = adjacencyList.size();

	// check version
	uint32_t ver;
	f.read((char*) &ver, 4);
	if ((ver & 0x00FFFFFF) == 0x00FFFFFF) {
		ver >>= 24;
		uint32_t numNodes;
		f.read((char*) &numNodes, 4);
		EV << "Shortest path file version " << ver << endl;
		EV << "Number of nodes: " << numNodes << endl;
		if (numNodes!=numberNodes)
			throw cRuntimeError("number of nodes does not match: topology (%d), shortest path file (%d)", numberNodes, numNodes);

	} else {
		EV << "Shortest path file version 0" << endl;
		f.seekg(0, ios::beg);
	}

	// load shortest-paths
	spath_costs = new uint8_t[numberNodes * numberNodes];
	spath_available = true;

	// read row by row
	uint8_t* ptr = spath_costs;
	for (size_t i = 0; i < numberNodes; i++) {
		// read id
		uint32_t id;
		f.read((char*) &id, 4);

		// set mapping
		spath_map.insert(make_pair(id, i));

		// read row
		f.read((char*) ptr, numberNodes);
		ptr += numberNodes;
	}

	// check on file end
	f.sync();
	ifstream::pos_type curr = f.tellg();
	f.seekg(0, ios::end);
	ifstream::pos_type end = f.tellg();
	if (curr != end) {
		EV << "Shortest path file size mismatch. ";
		EV << curr << " vs. " << end << endl;
		f.close();
		return false;
	}

	f.close();
	return true;
}

/// returns a random node
nodeid_t NetworkTopology::getRandomNode(size_t rndvalue) const {
	adjacencylist_t::const_iterator i = adjacencyList.begin();
	std::advance(i, (rndvalue % adjacencyList.size()));
	return i->first;
}

/// Returns the node degree, i.e., number of links to neighbors
size_t NetworkTopology::getNodeDegree(nodeid_t node) const {
	adjacencylist_t::const_iterator i = adjacencyList.find(node);
	if (i == adjacencyList.end())
		return 0;
	return i->second->linkset.size();
}

vector<nodeid_t>* NetworkTopology::getNodeIDs() const {
	vector<nodeid_t>* ids = new vector<nodeid_t>();
	foreach(const adjacencylist_t::value_type& value, adjacencyList)
	ids->push_back(value.first);
	return ids;
}

unordered_set<nodeid_t>* NetworkTopology::getGiantComponent(
		unordered_set<nodeid_t>& disabled) const {

	// components store
	typedef unordered_map<int, vector<nodeid_t>*> components_t;
	components_t components;
	unordered_set<nodeid_t>* d = &disabled;

	// prepare unvisited nodes
	unordered_set<nodeid_t> unvisited;
	foreach(const adjacencylist_t::value_type& value, adjacencyList)
	if (d == NULL || d->count(value.first) == 0)
		unvisited.insert(value.first);

	// find components
	int compno = 0;
	while (unvisited.size() != 0) {
		compno++;
		nodeid_t start = *unvisited.begin();
		vector<nodeid_t>& p = *new vector<nodeid_t> ();

		// breath-first search
		p.push_back(start);
		unvisited.erase(start);

		for (size_t i = 0; i < p.size(); i++) {
			// add neighbors of node
			const TopologyNode::linkset_t& ls =
					adjacencyList.find(p[i])->second->linkset;
			foreach(const TopologyNode::linkset_t::value_type& value, ls )
			if (unvisited.count(value.first) == 1 && (d == NULL
					|| d->count(value.first) == 0)) {
				p.push_back(value.first);
				unvisited.erase(value.first);
			}
		}

		// store component
		components.insert(make_pair(compno, &p));
	}

	// find giant component
	vector<nodeid_t>* max_component = NULL;
	foreach( components_t::value_type& comp, components )
	if (max_component == NULL || comp.second->size()> max_component->size())
		max_component = comp.second;

	// erase other components
	foreach( components_t::value_type& comp, components )
	if (comp.second != max_component)
		delete comp.second;

	// convert to set
	unordered_set<nodeid_t>* set = new unordered_set<nodeid_t> (
			max_component->begin(), max_component->end());

	// free remaining
	delete max_component;
	components.clear();

	return set;
}

/// add a connected pair of nodes with default link properties
void NetworkTopology::addLink(nodeid_t from, nodeid_t to) {
	unordered_map<nodeid_t, TopologyNode *>::iterator it;

	// if not present yet, insert from into adjacencyList
	if ((it = adjacencyList.find(from)) == adjacencyList.end()) {
		TopologyNode *add = new TopologyNode();
		add->addLink(to);
		adjacencyList[from] = add;
	} else {
		// if not yet present add new link with default link properties
		it->second->addLink(to);
	}
}

/// mark node1 and node2 bi-directionally connected
void NetworkTopology::setConnected(nodeid_t node1, nodeid_t node2, bool status) {
	TopologyNode *ls1, *ls2;
	TopologyLink *lp1, *lp2;

	ls1 = getLinkset(node1);
	if (ls1) {
		lp1 = ls1->getLink(node2);
		lp1->setConnected(status);
	}

	ls2 = getLinkset(node2);
	if (ls2) {
		lp2 = ls2->getLink(node1);
		lp2->setConnected(status);
	}
}

// does a bidirectional gate exist between the two nodes?
bool NetworkTopology::isConnected(nodeid_t node1, nodeid_t node2) {
	TopologyNode *ls1 = NULL, *ls2 = NULL;
	TopologyLink *lp1 = NULL, *lp2 = NULL;

	ls1 = getLinkset(node1);
	lp1 = ls1->getLink(node2);

	ls2 = getLinkset(node2);
	lp2 = ls2->getLink(node1);

	if (ls1 == NULL || lp1 == NULL || ls2 == NULL || lp2 == NULL)
		return false;
	else {
		return (lp1->isConnected() && lp2->isConnected());
	}
}

/// Returns true, if the network is connected
/// TODO: implement...
bool NetworkTopology::isConnected() {
	return false;
}

NetworkTopology::TopologyNode* NetworkTopology::getLinkset(nodeid_t node) {
	unordered_map<nodeid_t, TopologyNode*>::iterator it;
	if ((it = adjacencyList.find(node)) != adjacencyList.end())
		return it->second;
	else
		return NULL;
}

void NetworkTopology::extractFromNetwork(cModule *networkManager) {
	cModule* mod = getSimulation()->getContextModule();
	while(mod->getParentModule()!=NULL) mod = mod->getParentModule();

	// check that the top level module is actually a network
	if ( mod->getModuleType()->isNetwork() ) {
		for (cModule::SubmoduleIterator i(mod); !i.end(); i++)
		{
			cModule *submodp = *i;
			// we are only interested in Nodes
			if (opp_strcmp(submodp->getClassName(), "routingsim::Node")==0) {
				int fromID = dynamic_cast<Node *>(submodp)->getNodeId();
				moduleMap[fromID] = submodp;
				cGate *toNeighbors = submodp->gate("toNeighbors$o",0);
				EV << "Node " << fromID << endl;
				for (int i=0; i<toNeighbors->size(); i++) {
					cGate *outGate = submodp->gate("toNeighbors$o",i);
					Node *to = dynamic_cast<Node *>(outGate->getNextGate()->getOwnerModule());
					EV << "\tconnected to node " << to->getNodeId() << endl;
					addLink(fromID, to->getNodeId());
				}


				// TODO: insert into networktopology data structures
				// iterate over toNeighbors gate vector
				// TODO: check that everything is there for querying
				// TODO: Count number of links

				numberNodes++;
			}
		}
	} else {
		throw cRuntimeError("Error parsing network structure!");
	}
}

// build the network topology with given modules
void NetworkTopology::createNetwork(cModule *parent) {
	unordered_map<nodeid_t, TopologyNode *>::iterator adjListIt; //adjacencyList;
	unordered_map<nodeid_t, cModule*>::iterator modMapIt; // moduleMap;

	// create nodes
	cModuleType *moduleType = cModuleType::get("routingsim.foundation.Node");
	for (adjListIt = adjacencyList.begin(); adjListIt != adjacencyList.end(); adjListIt++) {
		char name[256];
		snprintf(name, 255, "Node%llu", adjListIt->first);
		cModule *mod = moduleType->create(name, parent);
		mod->par("id") = (long int)adjListIt->first;
		moduleMap[adjListIt->first] = mod;
	}
	EV << "Created node modules." << endl;

	// setup and connect gates
	for (adjListIt = adjacencyList.begin(); adjListIt != adjacencyList.end(); adjListIt++) {
		unordered_map<nodeid_t, TopologyLink *> linkset;
		unordered_map<nodeid_t, TopologyLink *>::iterator link;
		nodeid_t fromId, toId;
		cModule *fromModule, *toModule;
		cGate *fromIn, *fromOut, *toIn, *toOut;

		fromId = adjListIt->first;

		fromModule = moduleMap.find(fromId)->second;
		linkset = adjListIt->second->linkset;

		// for each connected node
		for (link = linkset.begin(); link != linkset.end(); link++) {
			toId = link->first;
			toModule = moduleMap.find(toId)->second;
			if (!isConnected(fromId, toId)) {
				//connect both directions of Gate "toNeighbors"
				fromModule->getOrCreateFirstUnconnectedGatePair("toNeighbors",
						false, true, fromIn, fromOut);
				toModule->getOrCreateFirstUnconnectedGatePair("toNeighbors",
						false, true, toIn, toOut);

				fromOut->connectTo(toIn);
				toOut->connectTo(fromIn);

				setConnected(fromId, toId, true);
			}
		}
	}
	EV << "Created links." << endl;

	// start simulation (randomize?)
	for (modMapIt = moduleMap.begin(); modMapIt != moduleMap.end(); modMapIt++) {

		// finalize parameters
		modMapIt->second->finalizeParameters();

		// set display strings
		foreach(TopologyNode::properties_t::value_type& value,
				adjacencyList[modMapIt->first]->properties) {
			if (value.first=="!ds")
				modMapIt->second->getDisplayString().updateWith(value.second.c_str());
		}

		// build insides
		modMapIt->second->buildInside();

		// set parameters of inside modules
		foreach(TopologyNode::properties_t::value_type& value,
				adjacencyList[modMapIt->first]->properties) {
			if (value.first.find('!')!=string::npos) continue;
			for (cModule::SubmoduleIterator i(modMapIt->second); !i.end(); i++) {
				cModule *smod = *i;
				if (smod->findPar(value.first.c_str())!=-1) {
					cPar& p = smod->par(value.first.c_str());
					if (p.getType() == cPar::STRING)
						p.setStringValue(value.second);
					else
						p.parse(value.second.c_str());
				} else {
					EV <<"Module " << smod->getId() << " does not have a parameter " << value.second << endl;
				}
			}
		}
	}
	EV << "Modules ready to start." << endl;

	for (modMapIt = moduleMap.begin(); modMapIt != moduleMap.end(); modMapIt++) {
		modMapIt->second->scheduleStart(simTime());
	}
	EV << "Modules started." << endl;
}

/// map a Node to our topology-describing nodeid_t
nodeid_t NetworkTopology::moduleToNodeID(cModule *cmod) {
	unordered_map<nodeid_t, cModule*>::iterator modMapIt; // moduleMap;
	for (modMapIt = moduleMap.begin(); modMapIt != moduleMap.end(); modMapIt++) {
		if (modMapIt->second == cmod) {
			return modMapIt->first;
		}
	}
	// not found
	return -1;
}

/// map a topology-describing nodeid_t to a Node (cModule *)
cModule *NetworkTopology::NodeIDToModule(nodeid_t node) {
	unordered_map<nodeid_t, cModule*>::iterator modMapIt; // moduleMap;
	for (modMapIt = moduleMap.begin(); modMapIt != moduleMap.end(); modMapIt++) {
		if (modMapIt->first == node) {
			return modMapIt->second;
		}
	}
	// not found
	return NULL;
}

/// disable links in Nodes and topology description
void NetworkTopology::disableLink(nodeid_t f, nodeid_t t) {
	cModule *from, *to;
	from = NodeIDToModule(f);
	to = NodeIDToModule(t);

	if (from == NULL || to == NULL) {
		EV << "Could not determine cModule (Node) for " << f << " or " << t
				<< endl;
		return;
	}

	if (check_and_cast<Node *> (from)->disableLink(to, "toNeighbors$o"))
		adjacencyList[f]->getLink(t)->setActive(false);
	if (check_and_cast<Node *> (to)->disableLink(from, "toNeighbors$o"))
		adjacencyList[t]->getLink(f)->setActive(false);

}

/// enable links in Nodes and topology description
void NetworkTopology::enableLink(nodeid_t f, nodeid_t t) {
	cModule *from, *to;
	from = NodeIDToModule(f);
	to = NodeIDToModule(t);

	if (from == NULL || to == NULL) {
		EV << "Could not determine cModule (Node) for " << f << " or " << t
				<< endl;
		return;
	}

	if (check_and_cast<Node *> (from)->enableLink(to, "toNeighbors$o"))
		adjacencyList[f]->getLink(t)->setActive(true);
	if (check_and_cast<Node *> (to)->enableLink(from, "toNeighbors$o"))
		adjacencyList[t]->getLink(f)->setActive(true);
}

void NetworkTopology::disableNode(nodeid_t n) {
	unordered_map<nodeid_t, TopologyLink*>::iterator it;
	TopologyNode *ls = getLinkset(n);

	// Iterate over all links of the node and disable them
	for (it = ls->linkset.begin(); it != ls->linkset.end(); it++)
		disableLink(n, it->first);
}

void NetworkTopology::enableNode(nodeid_t n) {
	unordered_map<nodeid_t, TopologyLink*>::iterator it;
	TopologyNode *ls = getLinkset(n);

	// Iterate over all links of the node and disable them
	for (it = ls->linkset.begin(); it != ls->linkset.end(); it++)
		enableLink(n, it->first);
}

int NetworkTopology::getNetworkSize() {

	return numberNodes;
}

/// returns the distance between two cModules (class Node)
/// can be queried by via NetworkManager::getDistance()
cost_t NetworkTopology::getDistance(cModule *from, cModule *to) {
	return getDistance(moduleToNodeID(from), moduleToNodeID(to));
}

/// return the shortest path distance between two nodes
/// if a shortest path file is available, read it from there
/// and otherwise calculate it on the fly
cost_t NetworkTopology::getDistance(nodeid_t from, nodeid_t to) {
	unordered_map<nodeid_t, TopologyNode *>::iterator it;
	unordered_map<nodeid_t, DijkstraStatus *> status;
	unordered_map<nodeid_t, DijkstraStatus *>::iterator sit; //status iterator
	unordered_map<nodeid_t, TopologyLink *>::iterator lsit; //link set iterator

	// if we have a precalculated map, return result right now
	if (spath_available) {
		size_t f = spath_map[from];
		size_t t = spath_map[to];
		return spath_costs[f + t * numberNodes];
	}

	// initialize status map
	for (it = adjacencyList.begin(); it != adjacencyList.end(); it++) {
		status[it->first] = new DijkstraStatus(it->second);
	}

	// set start node
	status[from]->setWeight(0.0);
	nodeid_t current = from;
	TopologyNode *currentLS = status[current]->getLinkSet();

	while (true) {
		float minDistance = FLT_MAX;
		//cost_t minDistance=~0;

		// traverse linkset and update costs (if better)
		for (lsit = currentLS->linkset.begin(); lsit
		!= currentLS->linkset.end(); lsit++) {
			cost_t cost = status[current]->getWeight()+ lsit->second->getWeight();
			if (status[lsit->first]->getWeight() > cost) {
				//EV << " updating cost from " << status[lsit->first]->getWeight() << " to "<< cost << endl;
				status[lsit->first]->setWeight(cost);
			}
		}
		// mark that all neighbors of current have been visited
		status[current]->markVisited();
		//EV << "marked visited: " << current << endl;

		// find minimum distance node from the set of not yet visited nodes
		for (sit = status.begin(); sit != status.end(); sit++) {
			DijkstraStatus *cds = sit->second;
			if (!cds->hasBeenVisited() && cds->getWeight() < minDistance) {
				minDistance = sit->second->getWeight();
				current = sit->first;
				currentLS = status[current]->getLinkSet();
				//EV << "setting minDistance to " << minDistance << "for node " << current << endl;
			}
		}

		// if all nodes have been visited, return cost
		bool allVisited = true;
		for (sit = status.begin(); sit != status.end(); sit++) {
			if (sit->second->hasBeenVisited() != true) {
				allVisited = false;
				break;
			}
		}
		if (allVisited) {
			cost_t result = status[to]->getWeight();
			for (sit = status.begin(); sit != status.end(); sit++) {
				delete sit->second;
			}
			return result;
		}
	}

	EV << "should never reach here" << endl;
	for (sit = status.begin(); sit != status.end(); sit++) {
		delete sit->second;
	}
	return INVALID_DISTANCE;
}

} //namespace routingsim
