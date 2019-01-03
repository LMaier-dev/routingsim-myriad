#ifndef VIRTUALRINGQUEUE_HPP_
#define VIRTUALRINGQUEUE_HPP_

#include<vector>
#include<boost/unordered_map.hpp>

namespace routingsim {

using namespace std;
using namespace boost;

class update_queue {
private:
	/// backlog type & declaration
	typedef unordered_map<nodeid_t, vector<vroute_update_t> > backlog_t;
	backlog_t backlog;

public:
	/// queue messages
	void send( nodeid_t neighbor, const vroute_update_t& route ) {
		if (backlog.count(neighbor)==0)
			backlog.insert( make_pair(neighbor, vector<vroute_update_t>() ));
		vector<vroute_update_t>& routes = backlog[neighbor];

		/// add new entry to backlog
		routes.push_back(route);
	}

	/// cancel messages
	void cancel( nodeid_t neighbor ) {
		backlog.erase(neighbor);
	}

	/// returns a vector of the queued updates
	VRoutePkt* get_updates( nodeid_t to ) {
		if (backlog.count(to)==0) return NULL;
		vector<vroute_update_t>& routes = backlog[to];
		VRoutePkt* pkt = new VRoutePkt();
		pkt->setNumUpdates(routes.size());
		pkt->setUpdatesArraySize(routes.size());
		for (size_t i=0; i<routes.size(); i++)
			pkt->setUpdates(i, routes[i]);
		size_t length=0;
		for (size_t i=0; i<pkt->getUpdatesArraySize(); i++)
			length += pkt->getUpdates(i).byteLength();
		pkt->setByteLength(length);
		routes.clear();
		return pkt;
	}
};

}

#endif /* VIRTUALRINGQUEUE_HPP_ */
