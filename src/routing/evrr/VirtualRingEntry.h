#ifndef VIRTUALRINGENTRY_HPP_
#define VIRTUALRINGENTRY_HPP_

#include<foundation.h>
#include<boost/unordered_set.hpp>

#include"VRouteUpdate.h"

namespace routingsim {

/// routing table entry
class entry_t : public vroute_update_t {
private:
	typedef vroute_update_t super;

public:
	/// routing table additions to the update
	nodeid_t next_hop;			//< the node the update came from

	/// references
	typedef boost::unordered_set<nodeid_t> nodeid_set;
	nodeid_set used_by; 		//< neighbors that adapted this entry
	uint8_t    unused;			//< number of rounds entry was not used

	/// flags
	bool neighbor; 				//< physical neighbor?
	bool changed; 				//< entry changed?

	/// route delegation
	bool delegated;				//< route delegated to me?
	bool delegated_new;			//< route newly delegated

	/// constructs a new entry
	entry_t() :	super(), next_hop(undefined_node),
		used_by(), unused(0),
		neighbor(false), changed(true),
		delegated(false), delegated_new(false) {
		update_type = INTABLE;
	}

	/// copy constructor
	entry_t(const entry_t& e) :
		super(e), next_hop(e.next_hop),
		used_by(e.used_by), unused(e.unused),
		neighbor(e.neighbor), changed(e.changed),
		delegated(e.delegated), delegated_new(e.delegated_new) {
	}

	/// assignent not allowed!
	inline entry_t& operator= ( const entry_t& e ) {
		assert(false);
		return *this;
	}

	/// returns true, if this is a new entry
	inline bool added() const {
		return next_hop == undefined_node && seq == 0;
	}

	/// returns true, if this is my own entry (distance is zero)
	inline bool own() const {
		return dist == 0 && seq != 0;
	}

	/// add route usage record
	inline void record_use( nodeid_t nid ) {
		used_by.insert(nid);
		unused = 0;
	}

	/// returns true if this entry is better than the given one
	bool better_than( entry_t* a ) {
		return a == NULL || dist < a->dist || (dist == a->dist && owner < a->owner);
	}

};

}

#endif /* VIRTUALRINGENTRY_HPP_ */
