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
// Network Topology Class
// ============================================================================

NetworkTopology::NetworkTopology() :
																			BGLtopology(0), topologyAltered(false), spath(0), spathMatrixSize(0), spathAvailable(false) { }

/// destructor
NetworkTopology::~NetworkTopology() {
	delete BGLtopology;

	if (spathAvailable) {
		if (spath != NULL) {
			for  (int i=0; i<spathMatrixSize; i++) {
				delete[] spath[i];
			}
			delete[] spath;
		}
	}
}

void NetworkTopology::readTopology(string path, string topologytype, bool useOnlyLargestConnectedComponent, bool makeLinksBidirectional, bool allowNodeIdRemapping, bool compressNodeIDs) {
	// store edges of topology
	vector< pair<long long, long long> > edgesRead;
	// store node properties
	boost::unordered_map < nodeid_t, properties_t > nodeproperties;
	// sanity check for number of nodes
	set<nodeid_t> nodeset;

	EV_INFO << "Reading Topology from " << path << "(file format: " << topologytype << ")" << endl;

	ifstream topologyFile(path.c_str());
	if (!topologyFile) {
		string msg("Could not open topology file ");
		msg+=path;
		throw cRuntimeError(msg.c_str());
	}

	if (!topologytype.compare("SELF_DEFINED")) {
		EV_INFO << "using bidirectional parameter read from topology file " << path << "!" << endl;
		readSelfDefinedTopology(topologyFile, nodeset, edgesRead, nodeproperties);
	}
	else if (!topologytype.compare("DIMES")) {
		readDimesTopology(topologyFile, nodeset, edgesRead, nodeproperties, makeLinksBidirectional);
	} else {
		throw cRuntimeError("Unknown topology type!");
	}

	long long maxVectorEntries = edgesRead.max_size();
	long long maxElement=*max_element(nodeset.begin(), nodeset.end());
	// if there are ids that would lead to an exception when inserting into the
	// BGLtopology (nodes are stored in a vector), remap them.
	// edgesRead is taken as a general vector -  better way to access limit?
	if (allowNodeIdRemapping &&
			maxElement > maxVectorEntries) {
		do {
			long long newID;
			bool foundID=false;

			// find lowest available node id
			for (long long i=0; i<maxVectorEntries; i++) {
				if (nodeset.find(i)==nodeset.end()) {
					newID = i;
					foundID = true;
					break;
				}
			}

			// no more node ids left
			if (foundID==false) {
				throw cRuntimeError("BGL cannot handle that many node ids with the current data structures");
			}

			// replace any occurrence of maxElement by newID
			vector< pair<long long, long long> >::iterator edgeIt;
			for(edgeIt = edgesRead.begin(); edgeIt!= edgesRead.end(); edgeIt++) {
				if (edgeIt->first == maxElement)
					edgeIt->first = newID;
				if (edgeIt->second == maxElement)
					edgeIt->second = newID;
			}
			if (nodeproperties.find(maxElement) != nodeproperties.end()){
				throw cRuntimeError("Fixme: Reassign node properties!");
			}
			// add new id and remove offending element for next iteration
			nodeset.insert(newID);
			nodeset.erase( nodeset.find(maxElement) );

			EV_INFO << "Node ID " << maxElement << " has been reassigned to ID " << newID << endl;

		} while ((maxElement=*max_element(nodeset.begin(), nodeset.end())) > maxVectorEntries);
	}

	// compress the graph's id space; this may currently be required as we use the node ids
	// as direct indices into our graph structures and therefore high node ids lead to large
	// data structures and exorbitant memory consumption during shortest path calculation
	if (compressNodeIDs) {
		long long maxVectorEntries = edgesRead.max_size();
		bool continueLoop=true;

		do {
			long long newID;
			bool foundID=false;
			long long maxElement=*max_element(nodeset.begin(), nodeset.end());

			// find lowest available node id
			for (long long i=0; i<maxVectorEntries; i++) {
				if (nodeset.find(i)==nodeset.end()) {
					newID = i;
					foundID=true;
					break;
				}
			}

			// the id space is continuous and we are done
			if (foundID==false || newID==maxElement+1) {
				continueLoop=false;
			} else {
				// replace any occurrence of maxElement by newID
				vector< pair<long long, long long> >::iterator edgeIt;
				for(edgeIt = edgesRead.begin(); edgeIt!= edgesRead.end(); edgeIt++) {
					if (edgeIt->first == maxElement)
						edgeIt->first = newID;
					if (edgeIt->second == maxElement)
						edgeIt->second = newID;
				}
				if (nodeproperties.find(maxElement) != nodeproperties.end()){
					throw cRuntimeError("Fixme: Reassign node properties!");
				}
				// add new id and remove offending element for next iteration
				nodeset.insert(newID);
				nodeset.erase( nodeset.find(maxElement) );
				EV << "Node ID " << maxElement << " has been reassigned to ID " << newID << endl;
			}

		} while (continueLoop);
	}

	// build bgl graph
	BGLtopology = new topologyGraph(nodeset.size());
	typedef vector< pair<long long, long long> >::iterator vpairit;

	try {
		// add edges with default properties
		for ( vpairit v = edgesRead.begin(); v != edgesRead.end(); v++ ) {
			BGLEdgeDesc e; bool b;
			tie(e,b) = add_edge(v->first, v->second, *BGLtopology);
			if (b) {
				(*BGLtopology)[e].active = true;
				(*BGLtopology)[e].weight = 1.0;
			}
		}
	} catch (length_error &lerr) {
		throw cRuntimeError("When inserting nodes into our graph structure, a length_error occured - probably due to a too large node id.\n"\
				"To allow remapping, enable the network manager parameter \"allowNodeIdRemapping\"");
	}

	// set node properties
	for (BGLVertexIt it=vertices(*BGLtopology).first; it!=vertices(*BGLtopology).second; it++) {
		(*BGLtopology)[*it].maxRTentries = std::numeric_limits<int>::max();

		// if the node actually exists
		if (nodeset.count( *it )) {
			(*BGLtopology)[*it].disabled = false;
			(*BGLtopology)[*it].active = true;
			//cout << "active node " << *it << endl;
		}
		// if the node does not exist, set disabled (no Node module is created for disabled nodes)
		else {
			(*BGLtopology)[*it].disabled = true;
			(*BGLtopology)[*it].active = false;
		}

		if ( nodeproperties.find(*it) != nodeproperties.end() ) {
			(*BGLtopology)[*it].properties = nodeproperties[*it];
		}
	}

	// compute largest connected component if useOnlyLargestConnectedComponent (NetworkManager.ned) is true
	if ( useOnlyLargestConnectedComponent ) {
		FilteredGraph fg(*BGLtopology, activeEdges<topologyGraph>(*BGLtopology), activeNodes<topologyGraph>(*BGLtopology));
		std::vector<int> component(num_vertices(fg));
		std::vector<int>::iterator vit;
		int numComponents;
		int numDisabledNodes=0; // count nodes that are disabled for not being part of LCC
		int numDisabledEdges=0;

		if ( is_directed(fg) ) {
			numComponents = strong_components(fg, &component[0]);
		} else {
			numComponents = connected_components(fg, &component[0]);
		}

		if (numComponents > 1) {
			cout << "Determining Largest Connected Component of overall " << numComponents << " components" << endl;
			vector<long long> partitionSizes(numComponents, 0);

			// determine partition sizes
			for (vit=component.begin(); vit!=component.end(); vit++) {
				partitionSizes[*vit]++;
			}

			// determine size of max partition and its index
			long long maxPartitionSize = *(max_element(partitionSizes.begin(), partitionSizes.end()));
			int maxPartitionId;

			for (int i=0; i<numComponents; i++) {
				if ( partitionSizes[i] == maxPartitionSize ) {
					maxPartitionId = i;
				}
			}

			// disable nodes and edges of other partitions
			for (ActiveBGLVertexIt it=vertices(fg).first; it!=vertices(fg).second; it++) {
				if (component[*it] != maxPartitionId) {
					EV << "Deactivating node ["<< *it << endl;
					// iterate over attached edges (in+out) and deactivate them
					ActiveBGLEdgeIt ei, ee;
					for (tie(ei,ee)=edges(fg); ei!=ee; ei++) {
						if ( source(*ei, fg)== *it || target(*ei, fg)==*it ) {
							EV << "... deactivating edge ["<< source(*ei,fg) << "," << target(*ei,fg) << "]" << endl;
							fg[*ei].active = false;
							numDisabledEdges++;
						}
					}

					// deactivate after edge removal as otherwise fg does not contain the edges any more
					(fg)[*it].disabled = true;
					(fg)[*it].active = false;
					numDisabledNodes++;
				}
			}
		}
		cout << "Disabled " << numDisabledNodes << " nodes to only work on largest connected/strong component!" << endl;
		cout << "Disabled " << numDisabledEdges << " edges to only work on largest connected/strong component!" << endl;
	}


	cout << "Unique Nodes read from file: " << nodeset.size() << endl;
	cout << "Edges (not necessarily unique) read from file including possibly added bidirectional edges: " << edgesRead.size() << endl;
	cout << "Active Nodes: " << getActiveNumberNodes() << endl;
	cout << "Active Edges: " << getActiveNumberLinks() << endl;

	if (! isConnected()) {
		cout << "Working on partitioned graph!" << endl;
	}
}


// Read in a topology from a text file.
// Currently there are two formats supported:
// (1) The first file format (header version 0) consists of a single header line
//     stating the number of nodes and the number of links (int \t int)
//     Consecutive lines describe edges in the same format (int \t int)
//     where the first column is the source and the second column is the
//     destination of the edge
// (2) The second file format (header version 1) is a bit advanced and allows a more explicit
//     description of the topology. An exemplary header looks as follows:
//     ******************************************************************
//     ###     Nodes:  8210    Edges:  19732   Bidirectional:  True
//	   # GNM (10240, 10240), Largest connected component with 8210 nodes and 9866 edges, SEED=123
// 	   # Nodes: 8210
//     # Edges: 9866
//     # (Global) Clustering Coefficient: 4.87210718636e-05
//     # Local Density (value, node, radius): [315, 5924, 12]
//     # Diameter: 29
//     # Degree Min / Mean / Median / Max: 1 / 2.40341047503 / 2.0 / 10
//     2       6428
//     3       2146
//     ******************************************************************
//     The first line, starting with ### states the basic topology parameters
//     Lines starting with a single # may contain additional information and comments
//     The remainder of the file contains the edges as in header version 0:
//     (int \t int) describing the "from" and "to" of an edge
void NetworkTopology::readSelfDefinedTopology(ifstream &topologyFile, set<nodeid_t> &nodeset, vector< pair<long long, long long> > &edges, boost::unordered_map < nodeid_t, properties_t > &nodeproperties) {
	string line;
	unsigned int numNodes = 0, numLinks = 0; // as given by header
	nodeid_t fromNode, toNode;
	int lineNumber = 0;
	bool undirectedGraph;

	while (std::getline(topologyFile, line)) {
		if(line.length() == 0 || line.front() == '#') continue;
		int tokenNumber = 0;
		lineNumber++;
		// escape char: "\", seperator "\t" and " ", quote chars " and '
		escaped_list_separator<char> sep("\\","\t ","\"'");
		typedef boost::tokenizer<escaped_list_separator<char> > tokenizer_t;
		tokenizer_t tok(line,sep);
		bool nodep=false;

		int headerVersion=1;

		for (tokenizer_t::iterator it = tok.begin(); it != tok.end(); it++) {
			tokenNumber++;

			// determine header type
			if (lineNumber==1 && tokenNumber==1) {
				if (boost::lexical_cast<string>(*it).compare("###") == 0) {
					headerVersion=1;
					EV << "Detected Topology File Header Version " << headerVersion << endl;
					// go to next token
					continue;
				} else {
					headerVersion=0;
					// do continue processing below as the current token is the number of nodes
					EV << "Detected Topology File Header Version " << headerVersion << endl;
				}
			}

			// new header
			if (headerVersion == 1 && lineNumber == 1 && ((tokenNumber == 2) || (tokenNumber==4))) {
				continue;
			}
			if (headerVersion == 1 && lineNumber == 1 && tokenNumber == 3) {
				numNodes = boost::lexical_cast<int>(*it);
				continue;
			}
			if (headerVersion == 1 && lineNumber == 1 && tokenNumber == 5) {
				numLinks = boost::lexical_cast<int>(*it);
				continue;
			}
			if (headerVersion == 1 && lineNumber == 1 && tokenNumber == 7) {
				if ( string(*it) == string("True") ||
						string(*it) == string("true") ||
						string(*it) == string("1") ) {
					undirectedGraph = true;
					EV << "Network topology is an undirected graph!" << endl;
				} else {
					undirectedGraph = false;
					EV << "Network topology is a directed graph!" << endl;
				}
				continue;
			}


			// old header
			if (headerVersion == 0 && lineNumber == 1 && tokenNumber == 1) {
				numNodes = boost::lexical_cast<int>(*it);
				continue;
			}
			if (headerVersion == 0 && lineNumber == 1 && tokenNumber == 2) {
				numLinks = boost::lexical_cast<int>(*it);
				undirectedGraph = true;
				EV << "Old header version, assuming network topology is an undirected graph!" << endl;
				continue;
			}

			// skip comment lines in new header format
			if (headerVersion == 1 && lineNumber > 1 && tokenNumber == 1) {
				if (boost::lexical_cast<string>(*it).compare(0,1,"#") == 0) {
					break;
				}
			}

			// handle regular line
			if (tokenNumber == 1) {
				if (*it=="node") nodep = true;
				else {
					fromNode = boost::lexical_cast<long long>(*it);
					nodeset.insert(fromNode);
				}
			}

			if (tokenNumber == 2 && !nodep) {
				toNode = boost::lexical_cast<long long>(*it);
				if (fromNode != toNode) {
					edges.push_back( pair<long long, long long>(toNode, fromNode) );
					if ( undirectedGraph == true ) {
						// if the read topology is bidirectional/undirected, insert the reverse edge
						// this is safe in case of multi-edges (which we do not want) as we use setS for the edgeList
						edges.push_back( pair<long long, long long>(fromNode, toNode) );
					}
					nodeset.insert(toNode);
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
				// store for later insertion into bgl topology
				if ( nodeproperties.find(toNode) != nodeproperties.end() ) {
					nodeproperties[toNode].insert( pair<string, string>(key, value));
				} else {
					properties_t pt;
					pt[key] = value;
					nodeproperties.insert( pair<nodeid_t, properties_t>(toNode, pt));
				}
			}
		}
	}

#ifdef DEBUG
	// output node properties
	for (boost::unordered_map < nodeid_t, properties_t >::iterator i = nodeproperties.begin(); i!=nodeproperties.end(); i++) {
		EV << "Node: " <<i->first <<endl;
		for (properties_t::iterator pt=i->second.begin(); pt!=i->second.end(); pt++) {
			EV << "\tproperty [" << pt->first << "] value [" << pt->second << "]" << endl;
		}
	}
#endif


}

void NetworkTopology::readDimesTopology(ifstream &topologyFile, set<nodeid_t> &nodeset, vector< pair<long long, long long> > &edges, boost::unordered_map < nodeid_t, properties_t > &nodeproperties, bool makeBidirectional) {
	string line;
	// standart csv file...
	escaped_list_separator<char> sep("\\", "," ,"\"'");
	typedef boost::tokenizer<escaped_list_separator<char> > tokenizer_t;

	while (std::getline(topologyFile, line)) {
		unsigned int tokenNumber = 0;
		long long from = 0;
		long long to = 0;

		tokenizer_t tok(line,sep);
		for (tokenizer_t::iterator it = tok.begin(); it != tok.end(); it++) {
			if (tokenNumber == 0) {
				from = boost::lexical_cast<long long>(*it);
				nodeset.insert(from);
			}
			if (tokenNumber == 1) {
				to = boost::lexical_cast<long long>(*it);
				edges.push_back( make_pair(from, to) );
				if (makeBidirectional)
					edges.push_back( make_pair(to, from) );

				nodeset.insert(to);
			}
			if (tokenNumber >=2)
				continue;

			tokenNumber++;
		}
	}
}

bool NetworkTopology::readShortestPaths(string shortestPathFile) {
	EV_ERROR << "readShortestPaths currently not implemented (shortest paths are calculated)" << endl;

	// TODO:
	/* read shortest paths into bgl distance matrix
	 * set 	bool topologyAltered to false (if we read in a spath matrix, we assume it matches the current network
	 * set spathAvailable to true to indicate that further lookups are supposed to happen directly to the stored spaths
	 */

	return true;
}

nodeid_t NetworkTopology::getRandomNode(cRNG *rng) const {
	ActiveBGLVertexIt vi, vi_end;
	FilteredGraph fg(*BGLtopology, activeEdges<topologyGraph>(*BGLtopology), activeNodes<topologyGraph>(*BGLtopology));
	tie(vi, vi_end) = vertices(fg);
	unsigned int n = getActiveNumberNodes();
	std::advance(vi, rng->intRand(n));
	return *vi;
}

vector<pair<nodeid_t, nodeid_t> > NetworkTopology::getRandomNodePairs( cRNG *rng, size_t k ) const {
	vector<pair<nodeid_t, nodeid_t> > res;
	vector<pair<nodeid_t, nodeid_t> >::const_iterator resIt;

	FilteredGraph fg(*BGLtopology, activeEdges<topologyGraph>(*BGLtopology), activeNodes<topologyGraph>(*BGLtopology));
	unsigned int n = getActiveNumberNodes();
	ActiveBGLVertexIt vi, ve;

	if (k > n*(n-1)) {
		cout << "Maximum   pairs: " << n*(n-1) << endl;
		cout << "Requested pairs: " << k << endl;
		throw cRuntimeError("Not enough node pairs in topology to fulfill request");
	}

	for (unsigned int i=0; i<k; ) {
		nodeid_t src=undefined_node, dst=undefined_node;
		bool insertPair = true;

		tie(vi, ve) = vertices(fg);
		std::advance(vi, rng->intRand(n));
		src = *vi;
		tie(vi, ve) = vertices(fg);
		std::advance(vi, rng->intRand(n));
		dst = *vi;

		// ignore self-messages
		if (src == dst) continue;

		// check uniqueness
		for ( resIt = res.begin(); resIt != res.end(); resIt++ ) {
			if (resIt->first == src && resIt->second == dst) {
				insertPair = false;
				break;
			}
		}

		// insert unique pairs into result vector
		if ( insertPair ) {
			res.push_back( make_pair(src, dst) );
			++i;
		}

	}

	return res;
}


pair<nodeid_t, nodeid_t> NetworkTopology::getRandomEdge(size_t activeLinkNumber) const {
	ActiveBGLEdgeIt ei, ee;
	FilteredGraph fg(*BGLtopology, activeEdges<topologyGraph>(*BGLtopology), activeNodes<topologyGraph>(*BGLtopology));
	tie(ei, ee) = edges(fg);
	//unsigned int n = getActiveNumberLinks();
	std::advance(ei, activeLinkNumber);
	return pair<nodeid_t, nodeid_t>(source(*ei, fg), target(*ei, fg));
}

size_t NetworkTopology::getNodeDegree(nodeid_t node) const {
	ActiveBGLVertexIt vi, vi_end;
	FilteredGraph fg(*BGLtopology, activeEdges<topologyGraph>(*BGLtopology), activeNodes<topologyGraph>(*BGLtopology));

	for ( tie(vi, vi_end) = vertices(fg); vi!=vi_end; vi++ ) {
		if (*vi == node && fg[*vi].active==true)
			return out_degree(*vi, fg);
	}
	// if the node does not exist, return degree 0
	return 0;
}

size_t NetworkTopology::getNodeDegrees( map<nodeid_t, unsigned long> &retDegreeMap) const {
	ActiveBGLVertexIt vi, ve;
	FilteredGraph fg(*BGLtopology, activeEdges<topologyGraph>(*BGLtopology), activeNodes<topologyGraph>(*BGLtopology));
	//	FilteredGraph::out_edge_iterator f, l;

	for ( tie(vi,ve)=vertices(fg); vi!=ve; vi++ ) {
		if ( fg[*vi].active ) {
			//			for (boost::tie(f, l) = out_edges(*vi, fg); f != l; ++f) {
			//				EV << *f << " - "<< *l << endl;
			//			}
			retDegreeMap[*vi] = out_degree(*vi, fg);
			//EV << "Degree of node " << *vi << ": " << retDegreeMap[*vi] << endl;
		}
	}
	return retDegreeMap.size();
}

void NetworkTopology::getNodeIDs(vector<nodeid_t> &v) const {
	ActiveBGLVertexIt vi, ve;
	FilteredGraph fg(*BGLtopology, activeEdges<topologyGraph>(*BGLtopology), activeNodes<topologyGraph>(*BGLtopology));

	for ( tie(vi, ve)=vertices(fg); vi!=ve; vi++) {
		v.push_back(*vi);
	}
	return;
}

void NetworkTopology::getNeighborNodeIDs(nodeid_t fromID, vector<nodeid_t> &neighborIDs) const {
	ActiveBGLAdjacencyIt ai, ae;
	FilteredGraph fg(*BGLtopology, activeEdges<topologyGraph>(*BGLtopology), activeNodes<topologyGraph>(*BGLtopology));

	tie(ai, ae) = adjacent_vertices( fromID, fg);
	for ( ; ai!=ae; ai++ ) {
		neighborIDs.push_back(*ai);
	}
}

vector<nodeid_t>  NetworkTopology::getHighestDegreeNodes( unsigned int count ) const {
	map<nodeid_t, unsigned long> nodeDegrees;
	map<nodeid_t, unsigned long> selectedNodes;
	map<nodeid_t, unsigned long>::const_iterator ndit;
	map<nodeid_t, unsigned long>::const_iterator snit;
	vector<nodeid_t> returnIDs;

	getNodeDegrees( nodeDegrees );

	unsigned long minDeg=std::numeric_limits<unsigned long>::max();
	nodeid_t minNodeID;

	for ( ndit=nodeDegrees.begin(); ndit!=nodeDegrees.end(); ndit++) {

		// insert first count elements directly
		if (selectedNodes.size() < count) {
			//selectedNodes.insert( make_pair<nodeid_t, unsigned long>(ndit->first, ndit->second) );
			selectedNodes.insert( *ndit );
			if ( ndit->second < minDeg ) {
				minDeg = ndit->second;
				minNodeID = ndit->first;
			}
			continue;
		}

		// skip if node's degree is lower than or equal to current minimum degree
		if (ndit->second <= minDeg) continue;

		// exchange lowest degree node
		selectedNodes.erase(minNodeID);
		selectedNodes.insert(*ndit);

		// update minDeg and minNodeID
		minDeg=std::numeric_limits<unsigned long>::max();
		for ( snit=selectedNodes.begin(); snit!=selectedNodes.end(); snit++ ) {
			if ( snit->second < minDeg ) {
				minNodeID = snit->first;
				minDeg = snit->second;
			}
		}
	}

	// prepare return vector
	for ( snit=selectedNodes.begin(); snit!=selectedNodes.end(); snit++ ) {
		returnIDs.push_back(snit->first);
	}

	return returnIDs;
}


/// add a connected pair of nodes with default link properties
// TODO: extend to add new nodes / links on the fly?
void NetworkTopology::addLink(nodeid_t from, nodeid_t to) {
	EV_WARN << "UNTESTED" << endl;
	add_edge(from, to, *BGLtopology);
	enableLink(from, to);
	topologyAltered = true; // requires shortest path recalculation
}

bool NetworkTopology::isConnected(nodeid_t node1, nodeid_t node2) {
	BGLEdgeDesc e;
	bool edgeExists;
	tie(e, edgeExists) = edge(node1, node2, *BGLtopology);
	if (edgeExists && (*BGLtopology)[e].active) {
		return true;
	} else {
		return false;
	}
}

bool NetworkTopology::isConnected() {
	return ( getNumPartitions() ==  1 );
}

unsigned int NetworkTopology::getNumPartitions() {
	FilteredGraph fg(*BGLtopology, activeEdges<topologyGraph>(*BGLtopology), activeNodes<topologyGraph>(*BGLtopology));
	std::vector<int> component(num_vertices(fg));

	if ( is_directed(fg) ) {
		return ( strong_components(fg, &component[0]) );
	} else if (is_undirected(fg) ) {
		return ( connected_components(fg, &component[0]) );
	}

	throw cRuntimeError("Something went wrong, should never reach here");
	return false;
}


void NetworkTopology::extractFromNetwork(cModule *networkManager) {
	vector< pair<long long, long long> > edges;
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
				EV << "Node " << fromID << " (mod-id: " << submodp->getId() << ")" << endl;
				for (int i=0; i<toNeighbors->size(); i++) {
					cGate *outGate = submodp->gate("toNeighbors$o",i);
					Node *to = Node::getOtherNode(outGate);
					EV << "\tconnected to node " << to->getNodeId() << endl;
					edges.push_back( pair<long long, long long>(fromID, to->getNodeId()));
				}
			}
			BGLtopology = new topologyGraph();
			typedef vector< pair<long long, long long> >::iterator vpairit;

			// add edges with default properties
			for ( vpairit v = edges.begin(); v != edges.end(); v++ ) {
				BGLEdgeDesc e; bool b;
				tie(e,b) = add_edge(v->first, v->second, *BGLtopology);
				if (b) {
					(*BGLtopology)[e].active = true;
					(*BGLtopology)[e].weight = 1.0;
				}
			}

			// set node properties
			for (BGLVertexIt it=vertices(*BGLtopology).first; it!=vertices(*BGLtopology).second; it++) {
				(*BGLtopology)[*it].active = true;
				(*BGLtopology)[*it].maxRTentries = std::numeric_limits<int>::max();
			}
		}
	} else {
		throw cRuntimeError("Error parsing network structure!");
	}
}

void NetworkTopology::createNetwork(cModule *parent) {

	// create nodes from bgl representation
	BGLVertexIt vi, ve;
	cModuleType *moduleType = cModuleType::get("routingsim.foundation.NodeWrapper");

	for ( tie(vi,ve) = vertices(*BGLtopology); vi!=ve; vi++) {

		// jump over disabled nodes
		if ( (*BGLtopology)[*vi].disabled ) {
			//EV << "IGNORING NODE " << *vi << endl;
			continue;
		}
		char name[256];
		snprintf(name, 255, "Node%llu", (long long)*vi);
		cModule *mod = moduleType->create(name, parent);
		mod->par("nodeId") = *vi;
		moduleMap[*vi] = mod;
	}
	EV << "Created node modules." << endl;

	// setup and connect gates
	// vi = vertex iterator, ve = vertex end iterator
	for ( tie(vi, ve) = vertices(*BGLtopology); vi!=ve; vi++ ) {
		nodeid_t fromId = *vi;
		cModule *fromModule, *toModule;
		cGate *fromOut, *toIn;

		// jump over disabled nodes
		if ( (*BGLtopology)[*vi].disabled ) {
			//EV << "IGNORING NODE in gate setup" << *vi << endl;
			continue;
		}

		fromModule = moduleMap[fromId];
		// for each connected node
		graph_traits<topologyGraph>::adjacency_iterator ai, ae;
		for ( tie(ai, ae) = adjacent_vertices(fromId, *BGLtopology); ai!=ae; ai++ ) {
			nodeid_t toId = *ai;

			// jump over disabled neighbor nodes
			if ( (*BGLtopology)[*ai].disabled ) {
				//EV << "IGNORING NODE in gate setup (adjacent nodes)" << *ai << endl;
				continue;
			}
			toModule = moduleMap[toId];

			fromOut = fromModule->getOrCreateFirstUnconnectedGate("toNeighbors", 'o', false, true);
			toIn = toModule->getOrCreateFirstUnconnectedGate("toNeighbors", 'i', false, true);

			fromOut->connectTo(toIn);
		}
	}
	EV << "Created links." << endl;

	// start simulation
	boost::unordered_map<nodeid_t, cModule *>::iterator modMapIt;
	for (modMapIt = moduleMap.begin(); modMapIt != moduleMap.end(); modMapIt++) {
		cModule *module = modMapIt->second;

		// finalize parameters
		module->finalizeParameters();

		properties_t nodeProp = (*BGLtopology)[modMapIt->first].properties;
		// set display strings
		foreach(properties_t::value_type& value, nodeProp) {
			if (value.first=="!ds")
				module->getDisplayString().updateWith(value.second.c_str());
		}

		module->buildInside();

		// set parameters of inside modules
		foreach(properties_t::value_type& value, nodeProp) {
			if (value.first.find('!')!=string::npos) continue;
			for (cModule::SubmoduleIterator i(module); !i.end(); i++) {
				cModule *submod = *i;
				if (submod->findPar(value.first.c_str())!=-1) {
					cPar& p = submod->par(value.first.c_str());
					if (p.getType() == cPar::STRING)
						p.setStringValue(value.second);
					else
						p.parse(value.second.c_str());
				} else {
					EV_WARN <<"Module " << submod->getId() << " of type " << submod->getClassName() << " does not have a parameter " << value.second << endl;
				}
			}
		}
	}
	EV_INFO << "Modules ready to start." << endl;

	for (modMapIt = moduleMap.begin(); modMapIt != moduleMap.end(); modMapIt++) {
		modMapIt->second->scheduleStart(simTime());
	}
	EV_INFO << "Modules started." << endl;
}

void NetworkTopology::disableLink(nodeid_t f, nodeid_t t) {
	cModule *from, *to;

	// topology description
	BGLEdgeDesc e;
	bool edgeValid;

	tie( e, edgeValid) = edge(f, t, *BGLtopology);
	if (!edgeValid) {
		throw cRuntimeError("trying to disable non-existing link!");
	}
	// if link is already disabled, return instantly
	if ( (*BGLtopology)[e].active == false ) {
		return;
	}

	(*BGLtopology)[e].active = false;
	topologyAltered = true; // requires shortest path recalculation

	// omnet network
	from = moduleMap[f];
	to = moduleMap[t];

	if (from == NULL || to == NULL) {
		EV_WARN << "Could not determine cModule (Node) for " << f << " or " << t
				<< endl;
		return;
	}
	Node::getNode(from)->disableLink(Node::getNode(to), "toNeighbors$o");
}

void NetworkTopology::enableLink(nodeid_t f, nodeid_t t) {
	cModule *from, *to;

	// topology description
	BGLEdgeDesc e;
	bool edgeValid;

	tie( e, edgeValid) = edge(f, t, *BGLtopology);
	if (!edgeValid) {
		throw cRuntimeError("trying to enable non-existing link!");
	}

	// if one of the adjacent nodes is down, don't enable the link
	if ( !(*BGLtopology)[source(e, *BGLtopology)].active || !(*BGLtopology)[target(e, *BGLtopology)].active ) {
		return;
	}

	// if link is already enabled, return instantly
	if ( (*BGLtopology)[e].active == true ) {
		return;
	}

	(*BGLtopology)[e].active = true;
	topologyAltered = true; // requires shortest path recalculation

	from = moduleMap[f];
	to = moduleMap[t];

	if (from == NULL || to == NULL) {
		EV_WARN << "Could not determine cModule (Node) for " << f << " or " << t
				<< endl;
		return;
	}
	Node::getNode(from)->enableLink(Node::getNode(to), "toNeighbors$o");
}

void NetworkTopology::disableNode(nodeid_t n) {
	BGLOutEdgeIt oe_cur, oe_end;
	BGLInEdgeIt ie_cur, ie_end;

	// if node already down, return instantly
	if ( !(*BGLtopology)[n].active ) {
		return;
	}

	// iterate over out edges
	tie( oe_cur, oe_end) = out_edges(n, *BGLtopology);
	for (; oe_cur!=oe_end; ++oe_cur) {
		if ( (*BGLtopology)[*oe_cur].active ) {
			disableLink(n, target(*oe_cur, *BGLtopology));
		}
	}

	// iterate over in edges
	tie( ie_cur, ie_end) = in_edges(n, *BGLtopology);
	for (; ie_cur!=ie_end; ++ie_cur) {
		if ( (*BGLtopology)[*ie_cur].active ) {
			disableLink(source(*ie_cur, *BGLtopology), n);
		}
	}

	(*BGLtopology)[n].active = false;

	// set display string status icon i2
	if (cSimulation::getActiveEnvir()->isGUI()) {
		cDisplayString &ds = moduleMap[n]->getDisplayString();
		ds.setTagArg("i2", 0, "status/stop");
	}
}

void NetworkTopology::enableNode(nodeid_t n) {
	BGLOutEdgeIt oe_cur, oe_end;
	BGLInEdgeIt ie_cur, ie_end;

	// if node already up, return instantly
	if ( (*BGLtopology)[n].active ) {
		return;
	}

	(*BGLtopology)[n].active = true;

	// iterate over out edges
	tie( oe_cur, oe_end) = out_edges(n, *BGLtopology);
	for (; oe_cur!=oe_end; ++oe_cur) {
		if ( (*BGLtopology)[ target(*oe_cur, *BGLtopology) ].active ) {
			enableLink(n, target(*oe_cur, *BGLtopology));
		}
	}

	// iterate over in edges
	tie( ie_cur, ie_end) = in_edges(n, *BGLtopology);
	for (; ie_cur!=ie_end; ++ie_cur) {
		if ( (*BGLtopology)[ source(*ie_cur, *BGLtopology) ].active ) {
			enableLink(source(*ie_cur, *BGLtopology), n);
		}
	}

	// set display string status icon i2
	if (cSimulation::getActiveEnvir()->isGUI()) {
		cDisplayString &ds = moduleMap[n]->getDisplayString();
		ds.setTagArg("i2", 0, "");
	}
}

unsigned int NetworkTopology::getNetworkSize() {
	return getActiveNumberNodes();
}

unsigned int NetworkTopology::getActiveNumberNodes() const {
	int cnt = 0;
	BGLVertexIt vi, ve;

	for ( tie(vi, ve)=vertices(*BGLtopology); vi!=ve; vi++) {
		if ( (*BGLtopology)[*vi].active ) {
			++cnt;
		}
	}
	return cnt;
}

unsigned int NetworkTopology::getActiveNumberLinks() const {
	int cnt = 0;
	BGLEdgeIt ei, ee;

	for ( tie(ei, ee)=edges(*BGLtopology); ei!=ee; ei++) {
		if ( (*BGLtopology)[*ei].active ) {
			++cnt;
		}
	}
	return cnt;
}

float NetworkTopology::getDistance(nodeid_t from, nodeid_t to) {
	time_t start, end;

	if ( spathAvailable && !topologyAltered ) {
		// return result from calculated spaths
		return spath[from][to];
	} else if (shortestPathCalculationMode == "onDemand") {
		// subtract 1 as path contains sender
		return getActiveTopologyPath(from, to).size()-1;
	} else if (shortestPathCalculationMode == "fullDistanceMatrix"){
		// (re-)calculate shortest paths on ACTIVE graph and store
		// note that num_vertices() and num_edges() on a filtered graph do return the unfiltered sizes!

		int totalNodes = num_vertices(*BGLtopology);

		// make sure we have enough storage
		if ( spathMatrixSize < totalNodes ) {
			if (spath != NULL) {
				for  (int i=0; i<spathMatrixSize; i++) {
					delete[] spath[i];
				}
				delete[] spath;
				throw;
			}

			spath = new float*[totalNodes];

			// set to NULL for cleanup in case of exception
			for (int i=0; i<totalNodes; i++) {
				spath[i] = NULL;
			}

			// try/catch to cleanup in case of exception
			try {
				for (int i=0; i<totalNodes; i++) {
					spath[i] = new float[totalNodes];
				}
				spathMatrixSize = totalNodes;
			} catch(...) {
				for  (int i=0; i<totalNodes; i++) {
					delete[] spath[i];
				}
				delete[] spath;
				throw;
			}
		}

		FilteredGraph fg(*BGLtopology, activeEdges<topologyGraph>(*BGLtopology), activeNodes<topologyGraph>(*BGLtopology));
		EV_DETAIL << "Recalculating Shortest Paths at Simtime " << simTime() << endl;
		time(&start);
		// use floyd warshall for dense graphs only
		if ( (totalNodes*num_edges(*BGLtopology)*log(totalNodes)) < pow(totalNodes, 3) ) {
			johnson_all_pairs_shortest_paths(fg, spath, weight_map(get(&BGLEdge::weight, fg)));
		} else {
			floyd_warshall_all_pairs_shortest_paths(fg, spath, weight_map(get(&BGLEdge::weight, fg)));
		}
		time(&end);
		EV_DETAIL << "... took " << difftime(end, start) << " seconds." << endl;
		spathAvailable = true;
		topologyAltered = false;

		return spath[from][to];
	}

	throw cRuntimeError("Unknown shortest path calculation mode: must never reach here");
	return 0.0;
}

class dijkstra_finish : std::exception {};

class dvis : public default_dijkstra_visitor {
public:
	dvis(nodeid_t d) : destination(d) {}
	template <typename Vertex, typename Graph>
	void finish_vertex(Vertex u, Graph &g) {
		if (u == destination) throw dijkstra_finish();
	}
private:
	nodeid_t destination;
};


vector<nodeid_t> NetworkTopology::getActiveTopologyPath(nodeid_t from, nodeid_t to) const {
	FilteredGraph fg(*BGLtopology, activeEdges<topologyGraph>(*BGLtopology), activeNodes<topologyGraph>(*BGLtopology));
	unsigned int n = num_vertices(fg);

	//XXX: performance?!
	vector<nodeid_t> p(n);
	vector<nodeid_t> path;

	if (from == to) { path.push_back(to); return path; } // leads to correct length calculation: path.size()-1

	dvis dv(to);
	try {
		dijkstra_shortest_paths(fg, vertex(from, *BGLtopology), weight_map(get(&BGLEdge::weight, fg)).predecessor_map(&p[0]).visitor(dv));
	} catch (const dijkstra_finish&) {EV << "Aborted sp calculation from " << from << " to " << to << " as sp was found on active topology" << endl; };

	// if p[to] == to, then to is not reachable from from
	if (p[to] == to) { return path; }

	path.push_back(to);
	nodeid_t pred = to;
	bool done = false;

	while ( !done ) {
		pred = p[pred];

		// add next predecessor
		path.push_back(pred);

		// done construcing path?
		if (pred == from)
			done = true;
	}
	reverse(path.begin(), path.end());

	return path;
}

vector<nodeid_t> NetworkTopology::getFullTopologyPath(nodeid_t from, nodeid_t to) const {
	// we could use bfs for unweighted graphs
	unsigned int n = num_vertices(*BGLtopology);
	vector<nodeid_t> p(n);
	vector<nodeid_t> path;

	if (from == to) { return path; }

	dvis dv(to);
	try {
		dijkstra_shortest_paths(*BGLtopology, vertex(from, *BGLtopology), weight_map(get(&BGLEdge::weight, *BGLtopology)).predecessor_map(&p[0]).visitor(dv));
	} catch (const dijkstra_finish&) {EV << "Aborted sp calculation from " << from << " to " << to << " as sp was found on *full* topology" << endl; };

	path.push_back(to);
	nodeid_t pred = to;
	bool done = false;

	while ( !done ) {
		pred = p[pred];

		// add next predecessor
		path.push_back(pred);

		// done construcing path?
		if (pred == from)
			done = true;
	}
	reverse(path.begin(), path.end());

	return path;
}

bool NetworkTopology::fullTopologyPathFailed(nodeid_t from, nodeid_t to) const {
	vector<nodeid_t> path = getFullTopologyPath(from, to);

	EV_DEBUG << "Checking whether path from " << from << " to " << to << " has failed" << endl;

	graph_traits<topologyGraph>::vertex_iterator vi, ve;

	// check for node failures
	tie(vi, ve) = vertices(*BGLtopology);
	for (; vi!= ve; vi++) {
		/// skip disabled nodes (disabled == non-existent)
		if ( (*BGLtopology)[*vi].disabled == true ) {
			continue;
		}
		/// if a failed node is on the path, return true
		if ( (*BGLtopology)[*vi].active == false ) {
			if ( find(path.begin(), path.end(), *vi) != path.end() ) {
				return true;
			}
		}
	}

	// check for link failures
	vector<nodeid_t>::const_iterator pit = path.begin();
	nodeid_t prev=-1;
	for (; pit!=path.end(); pit++) {
		if (*pit==from) {
			prev=from;
			continue;
		}
		//only if the edge exists
		if ( edge(prev, *pit, *BGLtopology).second ) {
			EV_TRACE << "\t(" << prev << "\t" << *pit << ") active : " <<  (*BGLtopology)[ edge(prev, *pit, *BGLtopology).first ].active  << endl;
			if ( (*BGLtopology)[ edge(prev, *pit, *BGLtopology).first ].active == false ) {
				// failed edge on path
				return true;
			} else {
				// prepare for next iteration
				prev = *pit;
			}
		}
	}
	return false;
}

void NetworkTopology::setShortestPathCalculationMode(string mode) {
	shortestPathCalculationMode = mode;
}

void NetworkTopology::resetNode(nodeid_t n) {
	cModule *node = Node::getNode(moduleMap[n]);
	int size = node->gate("toProtocols$o", 0)->size();
	for (int i = 0; i < size; i++) {
		cGate *tnm = node->gate("toProtocols$o", i);
		check_and_cast<Protocol *> (tnm->getPathEndGate()->getOwner())->reset();
	}
}


} //namespace routingsim
