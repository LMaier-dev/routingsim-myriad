#ifndef VROUTEUPDATE_HPP_
#define VROUTEUPDATE_HPP_

#include "foundation.h"

namespace routingsim {

// euclidean distance between two point on the ring
inline vid_t distance(vid_t lhs, vid_t rhs) {
	return lhs > rhs ? lhs - rhs : rhs - lhs;
}

/// definition of a route
class vroute_update_t {
public:
	/// route update type
	enum update_type_t {
		/// fundamental DSDV-inspired primitives
	    UPDATE=1,				///< new route
	    TEARDOWN=2, 			///< teardown route
	    DELEGATE=3,				///< delegates a route
	    NOTIFY=4,				///< notification about topology change
	    USED=8, 				///< notification that route is used
	    NOTUSED=9,				///< notification that route is not used

	    /// discovery primitives
	    DISCOVERY_PRED=16,		///< discovery message to predecessor
	    DISCOVERY_SUCC=17,		///< discovery message to successor

	    /// additional primitives
	    ANNOUNCEMENT=32,		///< announcement

	    /// special values
	    INTABLE=128,			///< table entry
	    INVALID_UPDATE=0		///< invalid update
	};

	/// route type
	enum route_type_t {
		RING=1,					///< normal ring route
		ANYCAST=2,				///< any-cast route
		MULTICAST=3,			///< multi-cast route ** NOT IMPLEMENTED YET **
		FLOOD=128,				///< flooded route    ** NOT IMPLEMENTED YET **

	    /// special values
		INVALID_ROUTE=0			///< invalid route
	};

	/// mandatory fields
	update_type_t 	update_type; 	///< type of update
	route_type_t  	route_type;		///< type of route
	vid_t 			id;				///< identifier of announcement
	uint32_t 		seq;			///< sequence number
	distance_t 		dist;			///< distance

	/// optional fields
	nodeid_t		owner;			///< node id of identifier
	vid_t 			dest;			///< delegation destination
	vid_t 			pred, succ;		///< best-known predecessor/successor
	bool			notify;			///< notification flag

	// adapt new successor/predecessor
	void consume(vid_t n) {
		if (is_new_pred(n)) pred=n;
		if (is_new_succ(n)) succ=n;
	}

	// check whether entry can be adapted as new successor/predecessor
	bool can_consume( vid_t n ) {
		return is_new_pred(n)||is_new_succ(n);
	}

	// check if the id is a new predecessor for the entry
	bool is_new_pred(vid_t n) {
		return (pred==undefined_vid && n<id) ||
			   (pred < id && n<id && distance(pred,n)<distance(pred,id));
	}

	// check if the id is a new successor for the entry
	bool is_new_succ(vid_t n) {
		return (succ==undefined_vid && n>id) ||
			   (succ > id && n>id && distance(succ,n)<distance(succ,id));
	}

	/// initializes a route update with default values
	vroute_update_t() : update_type(INVALID_UPDATE), route_type(INVALID_ROUTE),
		id(undefined_vid), owner(undefined_node),
		seq(0), dist(0), dest(undefined_vid), pred(undefined_vid), succ(undefined_vid) {
	}

	/// returns an adapted route update
	inline vroute_update_t route(update_type_t type, distance_t add = 0) const {
	    vroute_update_t r = *this;
	    r.update_type = type;
	    r.dist += add;
	    return r;
	}

	inline vroute_update_t operator() (update_type_t type, distance_t add = 0) const {
		return route(type,add);
	}

	/// returns the length in bytes
	inline size_t byteLength() const {
		size_t s= 4/*types,flags*/ + 16 /*id*/ + 4 /*seq,dist*/;
		if (owner!=undefined_node) s+=16;
		if (dest!=undefined_vid) s+=16;
		if (succ!=undefined_vid) s+=16*2;
		return s;
	}
};

/// displays a route
inline std::ostream& operator<<(std::ostream& os, const vroute_update_t& u ) {
    os << " id=" << u.id << " dist=" << u.dist << " seq=" << u.seq;
    return os;
}

}

#endif /* VROUTEUPDATE_HPP_ */
