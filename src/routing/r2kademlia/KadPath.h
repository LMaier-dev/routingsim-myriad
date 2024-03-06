#ifndef __ROUTINGSIM_KadPath_H_
#define __ROUTINGSIM_KadPath_H_

#include "OverlayKey.h"
#include <sstream>

namespace routingsim {

using kad_path = std::vector<OverlayKey>;
using link_t = std::pair<OverlayKey,OverlayKey> ;
using age_t = uint32_t;


class KadPath {
public:
	enum class pathState_t : uint8_t {
		undefined     = 0, // initial state, no path set
		exists        = 1, // path exists, but state unknown
		valid         = 2, // path should be valid
		rediscovering = 3, // path not usable, but rediscovering
		checking      = 4, // path probably usable, but need to test path
		invalid       = 5, // path invalid (contains broken link(s))
	};

	// constructors
	KadPath() : path(), lastseen(SIMTIME_ZERO), lastPathRefresh(SIMTIME_ZERO), pathHashSum(), state(pathState_t::undefined) { };
	KadPath(kad_path&& initpath) : path(std::move(initpath)), lastseen(SIMTIME_ZERO), lastPathRefresh(SIMTIME_ZERO), pathHashSum(), state(pathState_t::exists) { initpath.clear(); };
	KadPath(const kad_path& path) : path(path), lastseen(SIMTIME_ZERO), lastPathRefresh(SIMTIME_ZERO), pathHashSum(), state(pathState_t::exists) { };
	KadPath(const kad_path& path, const OverlayKey& pathHashSum) : path(path), lastseen(SIMTIME_ZERO), lastPathRefresh(SIMTIME_ZERO), pathHashSum(pathHashSum), state(pathState_t::exists) { };
	KadPath(const kad_path& path, const OverlayKey& pathHashSum, const simtime_t &lastseen) : path(path), lastseen(lastseen), lastPathRefresh(SIMTIME_ZERO), pathHashSum(pathHashSum), state(pathState_t::exists) { };
	KadPath(const kad_path& path, const OverlayKey& pathHashSum, const age_t &age) : path(path), lastseen(simTime() - SimTime(age, SIMTIME_MS)), lastPathRefresh(SIMTIME_ZERO), pathHashSum(pathHashSum), state(pathState_t::exists) { };

	// copy constructor
	KadPath(const KadPath& other) : path(other.path), lastseen(other.lastseen), lastPathRefresh(other.lastPathRefresh), pathHashSum(other.pathHashSum), state(other.state) { };

	// move constructor
	KadPath(KadPath&& other) : path(std::move(other.path)), lastseen(std::move(other.lastseen)), lastPathRefresh(std::move(other.lastPathRefresh)), pathHashSum(std::move(other.pathHashSum)), state(other.state) { other.path.clear(); other.lastseen= SIMTIME_ZERO; other.lastPathRefresh= SIMTIME_ZERO; other.state= pathState_t::undefined; };

	// copy assignment
	inline KadPath&  operator=(const KadPath& other);
	// move assignment
	inline KadPath&  operator=(KadPath&& other);

	// set back to initial state
	void clear() { 	path.clear(); lastseen= SIMTIME_ZERO; lastPathRefresh= SIMTIME_ZERO; pathHashSum={}; state= pathState_t::undefined; };
	const kad_path& getRawPath() const { return path; }
	void setPath(const kad_path& newpath, const OverlayKey& newPathHashSum, pathState_t newstate= pathState_t::exists) {
		path= newpath; pathHashSum= newPathHashSum; state= newstate; };
	void setPath(kad_path&& newpath, const OverlayKey& newPathHashSum, pathState_t newstate= pathState_t::exists) {
		path= std::move(newpath); pathHashSum= newPathHashSum; state= newstate; };

	size_t length() const { return path.size(); }
	size_t size() const { return length(); }
	// dlength is a length measure for diversity calculation,
	// where links AND nodes are counted.
	size_t dlength() const {
		// number of nodes + number of links, which is 2 longer
		return path.size() + (path.size() + 1);
	}
	bool isPhysicalNeighborPath() const { return (path.empty() == true); }

	// operator[]
	const OverlayKey& operator[](size_t pos) const { return path[pos]; }

	const OverlayKey& nextHop() const { return path.front(); }
	const OverlayKey& lastHop() const { return path.back(); }

	// iterators (begin, end) c/r
	kad_path::iterator begin() { return path.begin(); }
	kad_path::iterator end() { return path.end(); }
	kad_path::const_iterator begin() const noexcept { return path.begin(); }
	kad_path::const_iterator end() const noexcept { return path.end(); }
	kad_path::const_iterator cbegin() const noexcept { return path.cbegin(); }
	kad_path::const_iterator cend() const noexcept { return path.cend(); }
	kad_path::reverse_iterator rbegin() { return path.rbegin(); }
	kad_path::reverse_iterator rend() { return path.rend(); }
	kad_path::const_reverse_iterator crbegin() const noexcept { return path.crbegin(); }
	kad_path::const_reverse_iterator crend() const noexcept { return path.crend(); }

	//currently disabled as this would require to recompute the pathhashsum, too
	//void append(const OverlayKey& key) { path.push_back(key); }
	//void prepend(const OverlayKey& key) { path.insert(path.begin(), key); }
	//void removeSection(kad_path::iterator start, kad_path::iterator end) {
	//	path.erase(start, end);
	//}

	unsigned long getAge() const { return (simTime() - lastseen).inUnit(SIMTIME_MS); }
	simtime_t getLastSeen() const { return lastseen; }
	void updateLastSeen() { lastseen= simTime(); }
	void updateLastSeenTo(const simtime_t &new_time_stamp) { lastseen= new_time_stamp; }
	simtime_t getLastPathRefresh() const { return lastPathRefresh; }
	void updateLastPathRefresh() { lastPathRefresh = simTime(); }
	const OverlayKey& getPathHashSum() const { return pathHashSum; };

	bool isUndefined() const { return state == pathState_t::undefined; }
	bool isDefined() const { return state != pathState_t::undefined; }
	bool exists() const { return state == pathState_t::exists; }
	bool isValid() const { return state == pathState_t::valid; }
	bool isRediscovering() const { return state == pathState_t::rediscovering; }
	bool isChecking() const { return state == pathState_t::checking; }
	bool isInvalid() const { return state == pathState_t::invalid; }
	void setState(pathState_t newstate) { state= newstate; }

	bool containsUndirectedLink(const OverlayKey& source, const OverlayKey& target, const link_t link) const;

	uint commonLinks(const KadPath& otherPath) const;
	uint commonLinksAndNodes(const KadPath& otherPath) const;

	friend std::ostream& operator<<(std::ostream& os, const KadPath& c);

	// comparison operators
	bool operator==(const KadPath& other) const {
		if (other.length() != length()) {
			return false;
		}
		if (other.length() == 0 and length() == 0) {
			return true;
		}

		// this compares the paths from back to front, as paths tend
		// to be different at the end (starting with the destination)
		auto it_second= other.crbegin();
		for (auto it_first= crbegin(); it_first != crend(); it_first++) {
			if ( *it_first != *it_second )
				return false;
			it_second++;
		}
		return true;
	}

	bool operator!=(const KadPath& other) const { return !(*this == other); }

private:
    kad_path path;
    simtime_t lastseen;
    simtime_t lastPathRefresh; /// time of last path refresh (setupRsp / probeRsp)
    OverlayKey pathHashSum; /// the hash sum of the path
    pathState_t state;
};


inline
KadPath&  KadPath::operator=(const KadPath& other) {
		path= other.path;
		lastseen= other.lastseen;
		lastPathRefresh= other.lastPathRefresh;
		pathHashSum= other.pathHashSum;
		state= other.state;
		return *this;
}


inline
KadPath&  KadPath::operator=(KadPath&& other) {
		path= other.path;
		lastseen= other.lastseen;
		lastPathRefresh= other.lastPathRefresh;
		pathHashSum= other.pathHashSum;
		state= other.state;
		other.path.clear();
		other.lastseen= SIMTIME_ZERO;
		other.lastPathRefresh= SIMTIME_ZERO;
		other.state= pathState_t::undefined;
		return *this;
}


/// searches for undirected link in constructed path: <source + path + target>, returns true if found, false if not found
/// some paths exclude source and target nodes, e.g., those stored for KadContacts
bool containsUndirectedLink(const OverlayKey& source, const OverlayKey& target, const kad_path& path, const link_t link);

bool containsUndirectedLink(const kad_path& path, const OverlayKey& firstNodeKey, const OverlayKey& secondNodeKey);

inline
bool are_equal_r(const kad_path& first_path, const kad_path& second_path) {
	if (first_path.size() != second_path.size())
		return false;
	// precondition: paths have equal length (therefore no need to check for end of it_second)
	auto it_second= second_path.crbegin();
	for (auto it_first= first_path.crbegin(); it_first != first_path.crend(); it_first++) {
		if ( *it_first != *it_second )
			return false;
		it_second++;
	}
	return true;
};

// calculates an XXH3 hash of path and destkey, seed is usually the node's own key
OverlayKey HashSum(const OverlayKey& seed, const kad_path& path, const OverlayKey& destkey);

std::ostream& operator<<(std::ostream& os, const kad_path& c);

std::ostream& operator<<(std::ostream& os, const link_t& l);
}

#endif // __ROUTINGSIM_KadPath_H_
