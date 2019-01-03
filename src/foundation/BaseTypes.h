#ifndef __ROUTINGSIM_BASETYPES_H
#define __ROUTINGSIM_BASETYPES_H

#include<vector>
#include<iostream>
#include<omnetpp.h>
#include<boost/foreach.hpp>

namespace routingsim {

#ifdef __CDT_PARSER__
    #define foreach(a, b) a;
#else
    #define foreach(a, b) BOOST_FOREACH(a, b)
#endif

using namespace std;

/// sequence numbers
typedef unsigned int seqnum_t;

/// identifier type
typedef unsigned long long id_t;
const id_t undefined_id = 0;

/// node identifier definitions
typedef id_t nodeid_t;
const nodeid_t undefined_node = ~0;
const nodeid_t neighbor_cast = ~0;

/// virtual identifier definitions
typedef uint32_t vid_t;
const vid_t undefined_vid = 0;

/// edge identifier definitions
typedef id_t edgeid_t;
const edgeid_t undefined_edge = 0;
const edgeid_t default_edge = 1;

/// distance definition
typedef int distance_t;
const distance_t default_distance = 1;
const distance_t infinite_distance = ~0;

/// a simple vector_set class
template<class T>
class vector_set: public std::vector<T> {
public:
	typedef std::vector<T> super;

public:
	vector_set() :
		super() {
	}

	vector_set(const super& set) :
		super(set) {
	}

	/// insert a value to the set
	typename super::iterator insert(const T& value) {
		for (typename super::iterator i = super::begin(); i != super::end(); i++)
			if (*i == value) return i;
		push_back(value);
		return super::end() - 1;
	}

	/// insert a another set
	void insert(const vector_set<T>& set) {
		for (typename super::const_iterator i = set.begin(); i != set.end(); i++)
			insert(*i);
	}

	/// erase a value from the set
	void erase(const T& value) {
		for (typename super::iterator i = super::begin(); i != super::end(); i++)
			if (*i == value) {
				i = super::erase(i);
				break;
			}
	}

	/// erase a another set
	void erase(const vector_set<T>& set) {
		for (typename super::const_iterator i = set.begin(); i != set.end(); i++)
			erase(*i);
	}

	/// returns the number of elements
	size_t count(const T& value) const {
		size_t c = 0;
		for (typename super::const_iterator i = super::begin(); i
				!= super::end(); i++)
			if (*i == value)
				c++;
		return c;
	}
};


template<class T>
std::ostream& operator<<(std::ostream& os, const vector_set<T>& vs) {
	for (typename vector_set<T>::const_iterator i = vs.begin(); i != vs.end(); i++) {
		if (i != vs.begin())
			os << ",";
		os << *i;
	}
	return os;
}

/// hop definition
class hop_t {
public:
	hop_t() :
		from(undefined_node), to(undefined_node), edge(undefined_edge),
				distance(default_distance) {
	}
	hop_t(const hop_t& hop) :
		from(hop.from), to(hop.to), edge(hop.edge), distance(hop.distance) {
	}
	hop_t(nodeid_t from, nodeid_t to, distance_t distance = default_distance) :
		from(from), to(to), edge(default_edge), distance(distance) {
	}

	/// reverse hop
	hop_t reverse() const {
		hop_t rev(*this);
		nodeid_t temp = rev.to;
		rev.to = rev.from;
		rev.from = temp;
		return rev;
	}

	/// compare hop, distance is not compared!!
	bool operator==(const hop_t& hop) const {
		return hop.from == from && hop.to == to && hop.edge == edge;
	}

	/// compare hop
	bool operator!=(const hop_t& hop) const {
		return !(operator==)(hop);
	}

	nodeid_t from, to;
	edgeid_t edge;
	distance_t distance;
};

std::ostream& operator<<(std::ostream& os, const hop_t& h);

/// path definition
class path_t: public std::vector<hop_t> {
public:
	seqnum_t seq; //< The sequence number of the path

public:
	/// default constructor
	path_t() :
		std::vector<hop_t>(), seq(0) {
	}

	/// copy constructor
	path_t(const path_t& path) :
		std::vector<hop_t>(path), seq(path.seq) {
	}

	/// returns true, if this path contains the node
	bool contains(nodeid_t& node) const {
		for (const_iterator i = begin(); i != end(); i++)
			if (i->to == node || i->from == node)
				return true;
		return false;
	}

	/// returns true, if the hop is included in the path
	bool contains(hop_t& hop) const {
		for (const_iterator i = begin(); i != end(); i++)
			if (*i == hop)
				return true;
		return false;
	}

	/// returns the distance to the node in the path
	distance_t distance(nodeid_t& node) const {
		distance_t dist = 0;
		for (const_iterator i = begin(); i != end(); i++) {
			dist += i->distance;
			if (i->to == node)
				return dist;
		}
		return infinite_distance;
	}

	/// the sum of all weights
	distance_t distance() const {
		distance_t dist = 0;
		for (const_iterator i = begin(); i != end(); i++)
			dist += i->distance;
		return dist;
	}

	/// compare a path
	bool operator==(const path_t& path) const {
		if (path.size() != size())
			return false;
		for (const_iterator i1 = begin(), i2 = path.begin(); i1 != end(); i1++, i2++)
			if (*i1 != *i2)
				return false;
		return true;
	}

	bool operator!=(const path_t& path) const {
		return !(operator==(path));
	}
};

std::ostream& operator<<(std::ostream& os, const path_t& h);

/// a neighbor record of a node
class neighbor_t {
public:
	nodeid_t id;
	edgeid_t eid;
	simtime_t activity;
	cGate * out;
};

}

#endif /* __ROUTINGSIM_BASETYPES_H */
