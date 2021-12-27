#ifndef __ROUTINGSIM_BASETYPES_H
#define __ROUTINGSIM_BASETYPES_H

#include<vector>
#include<iostream>
#include<omnetpp.h>

namespace routingsim {

#ifdef __CDT_PARSER__
    #define foreach(a, b) a;
#else
    #define foreach(a, b) BOOST_FOREACH(a, b)
#endif


/// identifier type
typedef unsigned long id_t;

/// node identifier definitions
typedef id_t nodeid_t;
const nodeid_t undefined_node = ~0;
const nodeid_t neighbor_cast = ~0;

/// distance definition
typedef float distance_t;
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
		super::push_back(value);
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
		from(undefined_node), to(undefined_node),
				distance(default_distance) {
	}
	hop_t(const hop_t& hop) :
		from(hop.from), to(hop.to), distance(hop.distance) {
	}
	hop_t(nodeid_t from, nodeid_t to, distance_t distance = default_distance) :
		from(from), to(to), distance(distance) {
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
		return hop.from == from && hop.to == to;
	}

	/// compare hop
	bool operator!=(const hop_t& hop) const {
		return !(operator==)(hop);
	}

	nodeid_t from, to;
	distance_t distance;
};

std::ostream& operator<<(std::ostream& os, const hop_t& h);

/// a neighbor record of a node
class neighbor_t {
public:
	nodeid_t id;
	simtime_t activity;
	simtime_t lastsent;

	cGate * out;
};

}

#endif /* __ROUTINGSIM_BASETYPES_H */
