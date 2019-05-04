
#ifndef VIRTUALRINGROUTES_HPP_
#define VIRTUALRINGROUTES_HPP_

#include "foundation.h"
#include "VirtualRingQueue.h"
#include "VirtualRingEntry.h"

#include <boost/unordered_map.hpp>

namespace routingsim {

/**
 * Implementation of a destination-sequenced routing table with route
 * delegation, route usage records, and automatic route teardowns.
 *
 * @author Sebastian Mies <mies@routingsim.org>
 */
class route_maintenance {
public://------------------------------------------------------- Management --
	route_maintenance( update_queue& queue ) : queue(queue) {
		reset();
	}

	~route_maintenance() {

	}

	/// the dsdv-table listener
	class listener {
	protected:
		friend class route_maintenance;

		virtual ~listener() {};
		virtual void entry_notify( entry_t* entry ) {}
		virtual void entry_update( entry_t* entry, bool new_entry ) {}
		virtual void entry_remove( entry_t* entry ) {}
 		virtual bool entry_needed( entry_t* entry ) {return false;}
	};

	/// register a dsdv listener
	void register_listener( listener* l ) {
		listeners.push_back(l);
	}

	/// returns the update queue
	update_queue& get_queue() {
		return queue;
	}

	/// returns true, if seq# has been changed
	bool seqno_changed() const {
		return seq_changed;
	}

	/// returns true, if table has been altered
	bool table_altered() const {
		return tab_changed;
	}

	void set_nodeid( nodeid_t nid ) {
		nodeid = nid;
	}

	nodeid_t get_nodeid() const {
		return nodeid;
	}

public://--------------------------------------- DSDV-Routing-Table-Entries ---

	/// returns the routing-table size
	size_t size( entry_t::route_type_t type = entry_t::INVALID_ROUTE ) const {
		if (type==entry_t::INVALID_ROUTE) return table.size();
		size_t s=0;
		foreach(const table_t::value_type& v, table)
			if (v.second->route_type == type ) s++;
		return s;
	}

	/// returns the table entry
	const entry_t& operator [] ( size_t index ) const {
		table_t::const_iterator it = table.begin();
		std::advance(it,index);
		return *it->second;
	}

	/// returns the routing table entry or null
	entry_t* get( vid_t id, nodeid_t owner = undefined_node, bool ensure = false ) {
		if (table.count(id)==0) {
			if (!ensure) return NULL;
		} else {
			foreach(table_t::value_type& v, table.equal_range(id) )
				if (v.second->owner == owner) return v.second;
			if (!ensure) return NULL;
		}
		entry_t* ne = new entry_t();
		ne->id = id;
		ne->owner = owner;
		table.insert(make_pair(id,ne));
		return ne;
	}

	/// returns all entries to an id
	vector<entry_t*> get_all( vid_t id ) {
		vector<entry_t*> all;
		if (table.count(id)==0) return all;
		foreach(table_t::value_type& v, table.equal_range(id) )
			all.push_back(v.second);
		return all;
	}

	/// announce a route with the given identifier and type
	entry_t* add_id( vid_t id, entry_t::route_type_t type, nodeid_t owner = undefined_node ) {

		// set entry
		entry_t& e = *get(id, owner, true);
		e.id = id;
		e.seq = 1;
		e.route_type = type;
		e.dist = 0;

		// default values for own entry
		e.neighbor = false;
		e.owner = owner;
		e.next_hop = undefined_node;
		e.changed = true;
		e.delegated = true;
		e.delegated_new = true;

		// inform about updated entry
		foreach(listener* l, listeners) l->entry_update( &e, true );

		return &e;
	}


	/// remove a route
	void remove_id( vid_t id, nodeid_t owner = undefined_node ) {
		entry_t* e = get(id,owner);
		if (e==NULL || !e->own()) return;

		// remove entry
		remove(e);
	}

public://------------------------ Delegates, Notifications, Seq#-Management ---

	/// delegates a route
	void delegate( vid_t vid, vid_t dest ) {
		send_delegate(vid,dest);
	}

	/// notify a node about a topology change
	void notify( vid_t vid, bool do_increase_seq = false, nodeid_t owner = undefined_node ) {
		send_notify( get(vid, owner), do_increase_seq );
	}

	/// increase all sequence numbers
	void increase_seq( entry_t::route_type_t type = entry_t::INVALID_ROUTE ) {
		foreach( table_t::value_type& value, table) {
			entry_t* entry = value.second;
			if (!entry->own()) continue;
			if (type != entry_t::INVALID_ROUTE && type != entry->route_type)
				continue;
			entry->seq++;
			entry->changed = true;
		}
		seq_changed = true;
		tab_changed = true;
	}

public://------------------------------------- Physical Neighbor Management ---

	/// removes all entries pointing to a neighbor
	void remove_neighbor(nodeid_t nid) {

		// remove from neighbor set
		neighbors.erase(nid);

		// remove all entries with next_hop pointing to neighbor
		vector<entry_t*> to_remove;
		foreach( table_t::value_type& value, table)
			if (value.second->next_hop == nid) to_remove.push_back(value.second);
		foreach( entry_t* e, to_remove ) remove(e);

		// notify virtual neighbors about topology change
		foreach( table_t::value_type& value, table) {
			entry_t* e = value.second;

			// my own entry -> ignore
			if (e->own()) continue;

			// if the entry was not used by the neighbor-> ignore
			if ( e->used_by.count(nid) == 0 ) continue;
			e->used_by.erase(nid);

			// notify node
			notify(e->id, true, e->owner);
		}


		/// cleanup backlog
		queue.cancel(nid);

		/// increase all sequence#
		increase_seq();

		/// notify
		foreach(listener* l, listeners) l->entry_notify(NULL);
	}

	/// inform about new new neighbor? -> increase seq#
	void add_neighbor(nodeid_t nid) {
		// add to neighbor set
		neighbors.insert(nid);

		// increase all seq. no.
		increase_seq();

		/// notify
		foreach(listener* l, listeners) l->entry_notify(NULL);
	}

	/// returns the neighbor set
	boost::unordered_set<nodeid_t> get_neighbors() const {
		return neighbors;
	}

public://---------------------------------- Message Processing, State Reset ---

	/// reset changed flags, and increase "unused" counters
	void reset() {

		// reset change flags
		foreach( table_t::value_type& value, table) {
			entry_t& e = *value.second;
			if (!e.changed && e.used_by.size()==0) e.unused++;
			e.changed = false;
		}

		// reset global change flags
		tab_changed = false;
		seq_changed = false;
	}

	/// process a update
	bool process( nodeid_t from, const vroute_update_t& route ) {
		// process update
		switch(route.update_type) {

		// new route
		case vroute_update_t::UPDATE:
			process_route_update(from, route);
			return true;

		// tear-down route
		case vroute_update_t::TEARDOWN:
			process_route_teardown(from, route);
			return true;

		// route delegation
		case vroute_update_t::DELEGATE:
			process_route_delegate(from, route);
			return true;

		// process notify
		case vroute_update_t::NOTIFY:
			process_notify(from, route);
			return true;

		// add record of routing-table entry usage
		case vroute_update_t::USED: {
			entry_t* e = get(route.id, route.owner);
			if (e == NULL) queue.send( from, route(vroute_update_t::TEARDOWN) );
			else e->record_use(from);
			return true;
		}

		// remove record of routing-table entry usage
		case vroute_update_t::NOTUSED: {
			entry_t* e = get(route.id, route.owner);
			if ( e == NULL ) return true;
			e->used_by.erase(from);
			return true;
		}

		default:
			break;
		}

		return false;
	}

public://--------------------------------------------------------- Triggers ---

	/// trigger a consistency check of the routing table
	void trigger_consistency_check() {
		// check if routes in the routing table exists
		foreach( table_t::value_type& value, table) {
			entry_t* e = value.second;
			if (e->own()) continue;
			queue.send(e->next_hop, e->route(vroute_update_t::USED));
		}
	}

	/// trigger cleanup of the routing table
	void maintenance( unsigned int ttl ) {

		// erase unused entries
		vector<entry_t*> to_remove;
		foreach( table_t::value_type& value, table) {
			entry_t& e = *value.second;

			// check if entry can be removed
			bool remove_entry = // do not remove ...
				!e.own() &&				// my own ids
				!e.neighbor && 			// neighbor entries
				!e.changed && 			// entries changed
				e.unused >= ttl &&		// recently sent/used
				e.used_by.size()==0;	// entries used by others
			if (!remove_entry) continue;

			/// check if a listener needs this entry
			foreach( listener* l, listeners )
				remove_entry &= !l->entry_needed( &e );
			if (!remove_entry) continue;

			/// remove unused entries
			to_remove.push_back(&e);
		}
		foreach( entry_t* e, to_remove ) remove(e,false);

		// cleanup blacklist
		cleanup_blacklist(ttl);
	}

//=============================================================================
//=============================================================================
// PRIVATE MEMBERS
//=============================================================================
//=============================================================================
private:
	update_queue& queue;

	// routing-table
	typedef boost::unordered_multimap<vid_t, entry_t*> table_t;
	typedef std::pair<table_t::iterator, table_t::iterator> equal_pair_t;
	table_t table;

	// listeners
	typedef vector<listener*> listeners_t;
	listeners_t listeners;

	// physical neighbors
	typedef boost::unordered_set<nodeid_t> neighbors_t;
	neighbors_t neighbors;

	// change flags
	bool tab_changed;
	bool seq_changed;

	// node id
	nodeid_t nodeid;

private://------------------------------------------------------- Blacklist ---

	/// blacklist types & list itself
	class blist_entry_t {
	public:
		entry_t* entry;	///< blacklisted routing table entry
		uint8_t rounds;	///< number of rounds the entry is on the list
	};
	typedef vector<blist_entry_t> blacklist_t;
	blacklist_t blacklist;

	/// add to blacklist
	void add_blacklist( entry_t* rt ) {

		// check if already added, if so-> refresh entry
		foreach(blist_entry_t& e, blacklist)
			if (e.entry->id == rt->id && rt->owner == e.entry->owner && rt->seq >= e.entry->seq  ) {
				if (rt->seq == e.entry->seq) {
					delete rt;
					return;
				}
				delete e.entry;
				e.entry = rt;
				e.rounds = 0;
				return;
			}

		// not in list-> add new entry
		blist_entry_t en;
		en.entry = rt;
		en.rounds = 0;
		blacklist.push_back( en );
	}

	/// returns true if id and seq.no. is on blacklist
	bool is_blacklisted( vid_t id, nodeid_t owner, uint32_t seq ) {
		foreach(const blist_entry_t& e, blacklist)
			if (e.entry->id == id && e.entry->owner == owner && e.entry->seq >= seq ) return true;
		return false;
	}

	/// cleanup blacklist
	void cleanup_blacklist( int ttl ) {
		for (blacklist_t::iterator i = blacklist.begin(); i!=blacklist.end(); i++) {
			i->rounds++;
			if (i->rounds >= ttl) {
				delete i->entry;
				i = blacklist.erase(i) - 1;
			}
		}
	}

private://------------------------------------------------- List operations ---

	/// remove a routing table entry by iterator
	void remove( entry_t* e, bool add_to_blacklist = true ) {

		// inform listeners
		foreach( listener* l, listeners ) l->entry_remove(e);

		// backlog tear-down messages to neighbors that use this entry
		foreach( const nodeid_t neighbor, e->used_by )
			send_teardown(neighbor, e);

		// inform next hop about removal
		queue.send( e->next_hop, e->route(vroute_update_t::NOTUSED) );

		// remove entry from table and return iterator to next entry
		equal_pair_t p = table.equal_range(e->id);
		for (table_t::iterator i = p.first; i != p.second; i++)
			if (i->second->owner == e->owner) {
				table.erase(i);
				break;
			}
		tab_changed = true;

		// delete entry when not transfered to blacklist
		if (add_to_blacklist) add_blacklist( e ); else delete e;
	}

private://---------------------------------------------------------- Update ---

	/// process route update
	void process_route_update( nodeid_t from, const vroute_update_t& route ) {
		ASSERT(route.id != undefined_vid);

		// do not add routes on blacklist
		if ( is_blacklisted(route.id, route.owner, route.seq) ) {
			queue.send( from, route(vroute_update_t::NOTUSED) );
			return;
		}

		// get entry
		entry_t& e = *get(route.id, route.owner, true);

		// sanity check: do not change my own identifiers
		if (e.own()) return;

		// remember if entry is new
		bool new_entry = e.added();

		// check whether the entry has a new sequence number from the same owner
		bool new_seq_entry = (route.seq > e.seq);

		// check whether the entry is closer with the same sequence number
		bool closer_entry = (route.dist + 1) < e.dist && route.seq == e.seq;

		// check whether the entry needs to be adapted
		bool adapt_entry = new_entry || closer_entry || new_seq_entry;

		// adapt entry
		if (adapt_entry) {
			// tab_changed next hop on ex. entry? -> notify old next_hop in backlog
			if (e.next_hop != from)
				queue.send( e.next_hop, e.route(vroute_update_t::NOTUSED) );

			// adapt update
			e.update_type 	= vroute_update_t::INTABLE;
			e.route_type 	= route.route_type;
			e.id 			= route.id;
			e.owner			= route.owner;
			e.seq 			= route.seq;
			e.dist 			= route.dist + 1;

			// optimization of succ/pred
			e.succ			= route.succ;
			e.pred			= route.pred;

			// set routing information
			e.neighbor 		= (route.dist == 0);
			e.next_hop 		= from;
			e.changed 		= true;
			e.delegated 	|= e.neighbor;
			e.delegated_new |= e.neighbor;
			tab_changed 	 = true;

			// backlog entry was added!
			queue.send( e.next_hop, e.route(vroute_update_t::USED) );

			// inform about updated entry
			foreach(listener* l, listeners) l->entry_update( &e, new_entry );
		}

		// when entry is not adapted backlog remove notification
		if (e.next_hop != from)
			queue.send(from, route(vroute_update_t::NOTUSED) );
	}

private://-------------------------------------------------------- Teardown ---

	/// teardown a route
	void send_teardown( nodeid_t neighbor, entry_t* e ) {
		vroute_update_t td;
		td.update_type = vroute_update_t::TEARDOWN;
		td.id = e->id;
		td.owner = e->owner;
		td.seq = e->seq;
		queue.send( neighbor, td );
	}

	/// process route tear-down
	void process_route_teardown( nodeid_t from, const vroute_update_t& route ) {

		// do not tear down my own identifiers, or unknown entries
		entry_t* entry = get(route.id, route.owner);

		// do not remove my own or routes from origin of tear-down
		if (entry == NULL || entry->own()
				|| entry->next_hop != from || entry->seq > route.seq) return;

		// remove entry
		remove(entry);
	}

private://---------------------------------------------------- Notification ---

	/// notify neighbor about routing change
	void send_notify( const entry_t* e, bool do_seq_increase ) {
		// do not notify myself
		if (e->own()) return;

		// send notification to virtual neighbors affected by the failure
		vroute_update_t notify_update;
		notify_update.update_type = vroute_update_t::NOTIFY;
		notify_update.dest = e->id;
		notify_update.owner = e->owner;
		notify_update.dist = 0;
		notify_update.notify = do_seq_increase;
		queue.send(e->next_hop, notify_update);
	}

	/// process notify message
	void process_notify( nodeid_t from, const vroute_update_t& route ) {

		// find route
		entry_t* dst = get(route.dest, route.owner);
		if ( dst == NULL ) return;

		// reached me-> increase sequence number
		if ( dst->own() ) {
			if (route.notify) {
				dst->seq++;
				dst->changed = true;
				seq_changed = true;
				tab_changed = true;
			}
			foreach(listener* l, listeners) l->entry_notify( dst );
			return;
		}

		// forward to destination... do not run in cycles
		if ( dst->next_hop != from && route.dist < 128) {
			vroute_update_t notify_update = route;
			notify_update.notify = route.notify;
			notify_update.dist++;
			queue.send( dst->next_hop, notify_update );
		}
	}

private://------------------------------------------------------ Delegation ---

	/// delegates an extising route to a destination
	void send_delegate( vid_t from, vid_t to, bool notify = true ) {

		// get both route entries
		entry_t *src = get(from);
		entry_t *dst = get(to);
		if (src == NULL || dst == NULL || dst->own() ) return;
		assert(src->owner == undefined_node && dst->owner == undefined_node);

		// create delegate update
		vroute_update_t u = src->route(vroute_update_t::DELEGATE);
		u.dest = dst->id;
		u.notify = notify;

		// add usage record
		if (src->next_hop != dst->next_hop)	src->record_use(dst->next_hop);

		// queue update message
		queue.send(dst->next_hop, u);
	}

	/// process route delegation
	void process_route_delegate( nodeid_t from, const vroute_update_t& route ) {
		// add route
		process_route_update(from, route(vroute_update_t::UPDATE));

		// get both route entries, if not found-> quit
		entry_t *src = get(route.id);
		entry_t *dst = get(route.dest);
		if (src == NULL || dst == NULL) return;

		// reached destination-> set delegated flag
		if (dst->own()) {
			src->delegated = true;
			src->delegated_new |= true;
			return;
		}

		// split horizon
		if (dst->next_hop == from) return;

		// add reference
		if (src->next_hop != dst->next_hop)	src->record_use(dst->next_hop);

		// route message
		vroute_update_t r = src->route(vroute_update_t::DELEGATE);
		r.dest 	 = route.dest;
		r.notify = route.notify;
		queue.send(dst->next_hop, r);
	}
};

}

#endif /* VIRTUALRINGDSDV_HPP_ */
