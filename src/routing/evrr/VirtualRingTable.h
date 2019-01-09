#ifndef VIRTUALRINGTABLE_HPP_
#define VIRTUALRINGTABLE_HPP_

#include<vector>

#include "VirtualRingEntry.h"
#include "VirtualRingQueue.h"
#include "VirtualRingRoutes.h"
#include "crng.h"

namespace routingsim {

using namespace std;
using namespace boost;

/**
 * Extended Virtual Ring Routing (eVRR)<br />
 * Uses DSDV-inspired routing-table construction.
 *
 * @author Sebastian Mies <mies@routingsim.org>
 */
class ring_table : public route_maintenance::listener {
public://------------------------------------------------- Con-/Destruction ---

	/// constructs the virtual ring routing-table
	ring_table(route_maintenance& dsdv, cRNG* rng_) :
		queue(dsdv.get_queue()), dsdv(dsdv) {
		stable = 0;
		stable_readvertize = 0;
		vnb_changed = false;
		use_linearization = false;
		discovery_alpha = 2;
		delay_threshold = 5;
		dsdv.register_listener(this);
		rng = rng_;
	}

	/// destructs the virtual ring routing-table
	~ring_table() {}

	/// listener: informs about changes in the virtual neighborhood
	class listener {
		friend class ring_table;
	protected:
		virtual ~listener() {}
		virtual void vring_changed() {}
	};

	/// registers a listener
	void register_listener( listener* listener ) {
		listeners.insert(listener);
	}

	/// returns the dsdv table
	route_maintenance& get_dsdv() {
		return dsdv;
	}

	/// sets whether linearization should be used
	void set_use_linearization( bool flag ) {
		this->use_linearization = flag;
	}

	/// sets the number of parallel discoveries
	void set_discovery_alpha( unsigned int alpha ) {
		this->discovery_alpha = alpha;
	}

	/// sets the number of parallel discoveries
	void set_delay_threshold( unsigned int threshold ) {
		this->delay_threshold = threshold;
	}


	/// returns true, if virtual neighbors changed
	bool vneighbors_changed() const {
		return vnb_changed;
	}

	/// sets its own id
	void set_id( vid_t id ) {
		dsdv.add_id( id, entry_t::RING );
	}

	/// returns a random own id
	vid_t random_id() const {
		foreach(const entry_t* e, table) if (e->own()) return e->id;
		return undefined_vid;
	}

public://----------------------------------------------------- Table Access ---

	// returns the route neighbor
	const entry_t& operator[] ( int index ) const {
		assert( index >= 0 && (size_t)index < size() );
		return *table[index];
	}

	/// returns the size of the routing table
	inline size_t size() const {
		return table.size();
	}

	/// returns the virtual neighbors including my own identifiers
	unordered_set<vid_t>* get_vneighbors() const {
		if (table.size() == 0)
			return NULL;

		// find my own entries and add their virtual neighbors
		unordered_set<vid_t>* ids = new unordered_set<vid_t>();
		for (size_t i = 0; i < table.size(); i++) {
			if (!table[i]->own()) continue;
			ids->insert(table[i]->id);
			if (i>=1) ids->insert(table[i-1]->id);
			if (i<(size()-1)) ids->insert(table[i+1]->id);
		}
		return ids;
	}

	/// returns the next hop
	entry_t* next_hop( vid_t vid ) {
		vid_t dist = ~(vid_t) 0;
		entry_t* next_hop_entry = NULL;
		for (size_t i = 0; i < size(); i++) {
			entry_t& e1 = *table[i];
			vid_t ndist = distance(e1.id, vid);
			if (ndist < dist) {
				next_hop_entry = &e1;
				dist = ndist;
			}
		}
		return next_hop_entry;
	}

public://----------------------------------------------------- Table Update ---

	bool process( const nodeid_t from, const vroute_update_t& route ) {
		// process update
		switch(route.update_type) {

		// discovery
		case vroute_update_t::DISCOVERY_SUCC:
		case vroute_update_t::DISCOVERY_PRED:
			process_discovery(from, route);	return true;

		default:
			break;
		}
		return false;
	}

	/// stabilize ring by updating it
	void maintenance() {
		// do not update when nothing is there :)
		if (size() == 0) return;

		// send my own id to neighbors
		foreach( const nodeid_t& to, dsdv.get_neighbors() ) {

			// inform neighbors about my own ids
			for (size_t i = 0; i < table.size(); i++) {
				entry_t& e1 = *get(i);
				if (!e1.own() || !e1.changed) continue;

				// inform neighbors about my own ids
				e1.record_use(to);
				queue.send(to, e1.route(vroute_update_t::UPDATE) );
			}
		}

		// send regular advertisements when not using linearization
		if (!use_linearization)	discover(); else linearize();
	}

	/// reset flags
	void reset() {
		vnb_changed = false;
	}

//=============================================================================
//=============================================================================
// PRIVATE MEMBERS
//=============================================================================
//=============================================================================
private:
	update_queue& queue;
	route_maintenance& dsdv;
	typedef vector<entry_t*> table_t;
	typedef table_t::iterator iterator;
	typedef table_t::const_iterator const_iterator;
	table_t table;
	bool vnb_changed;
	bool use_linearization;
	int discovery_alpha;

	// listeners
	unordered_set<listener*> listeners;

	// stability detection
	int stable;
	int stable_readvertize;
	int delay_threshold;

	// this is needed for random values in maintenance/discover
	cRNG* rng;

protected://------------------------------------------------- DSDV Listener ---

	/// reset stable count on notification
	virtual void entry_notify( entry_t* entry ) {
		// only process ring updates
		if (entry != NULL && entry->route_type != entry_t::RING) return;

		stable = 0;
	}

	/// called when a entry is updated
	virtual void entry_update( entry_t* entry, bool new_entry ) {

		// only process ring updates
		if (entry->route_type != entry_t::RING) return;

		// insert if new
		if (!new_entry) return;

		// find entry whose id is greater or equal
		size_t i = 0; while (i < size() && get(i)->id < entry->id) i++;
		if ( i == size() || get(i)->id != entry->id )
			table.insert( table.begin()+i, entry );

		// check if this a new virtual neighbor of mine
		vid_t myown_id = ownid_of_vneighbor(entry->id);
		if (myown_id != undefined_vid) {

			// delegate my route to the new virtual neighbor
			dsdv.delegate(myown_id, entry->id);

			/// inform virtual neighborhood about changes
			if (!use_linearization)	discovery_mark_change( i );
			else linearization_mark_change( i );

			// inform listeners
			foreach(listener* l, listeners) l->vring_changed();

			// set changed virtual neighbors and reset stable count
			vnb_changed = true;
		}
	}

	virtual void entry_remove( entry_t* e ) {

		// only process ring updatesdelay_threshold
		if (e->route_type != entry_t::RING) return;

		// removed virtual neighbor? yes-> increase seq#, set flags
		if (is_vneighbor(e->id)) {

			/// inform virtual neighborhood about changes
			if (use_linearization) linearization_mark_change(index_of(e->id));
			else discovery_mark_change(index_of(e->id));

			// inform listeners
			foreach(listener* l, listeners) l->vring_changed();

			// increase sequence number and set virtual neighbors changed
			dsdv.increase_seq( entry_t::RING );
			vnb_changed = true;
		}

		// remove entry
		for (table_t::iterator i=table.begin(); i!=table.end(); i++) {
			if ((*i) == e) {
				assert(!e->own());
				table.erase(i);
				break;
			}
		}
	}

	virtual bool entry_needed( entry_t* entry ) {
		// only process ring updates
		if (entry->route_type != entry_t::RING) return false;

		return is_vneighbor(entry->id);
	}

private:

	/// returns a table entry
	inline entry_t* entry_of( vid_t vid ) {
		foreach(entry_t* e, table) if (e->id == vid) return e;
		return NULL;
	}

	// returns a table entry
	inline const entry_t* entry_of( vid_t vid ) const {
		foreach(const entry_t* e, table) if (e->id == vid) return e;
		return NULL;
	}

	/// returns the index of the table entry with the given id or -1
	inline int index_of( vid_t vid ) const {
		for (size_t i=0; i<size(); i++) if (table[i]->id == vid) return i;
		return -1;
	}

	// returns the route neighbor
	inline entry_t* get( int index, int offset = 0) {
		if ((index+offset)<0 || (index+offset)>=(int)size()) return NULL;
		else return table[(size_t)(index+offset)];
	}

	/// left or right vneighbor?
	enum direction_t { dir_left=1, dir_right=2, dir_own = 0 };
	direction_t left_right( int idx ) const {
		if (table[idx]->own()) return dir_own;
		int left=-1, right=-1;
		for (int j=1; j<(int)table.size(); j++) {
			if (left==-1 && (idx+j) < (int)size() && table[idx+j]->own() ) left = idx+j;
			if (right==-1 &&(idx-j)>=0 && table[idx-j]->own()) right = idx-j;
		}
		if (left!=-1 && right!=-1)
			return distance(table[left]->id,table[idx]->id)<
				   distance(table[right]->id,table[idx]->id) ? dir_left:dir_right;
		return left != -1 ? dir_left : dir_right;
	}

	/// returns the own id to a given virtual neighbor index
	int ownid_of_vneighbor_by_index( int idx ) const {
		// return -1 if index is invalid
		if (idx == -1) return -1;

		if (idx>0 && table[idx-1]->own()) return idx-1;
		if (idx<((int)table.size()-1) && table[idx+1]->own()) return idx+1;
		return -1;
	}

	/// returns the own id of a virtual neighbor
	vid_t ownid_of_vneighbor( vid_t id ) const {
		int own_index = ownid_of_vneighbor_by_index(index_of(id));
		return (own_index == -1) ? undefined_vid : table[own_index]->id;
	}

	/// checks whether an id a virtual neighbor
	bool is_vneighbor( vid_t id ) {
		return ownid_of_vneighbor(id) != undefined_vid;
	}


private://--------------------------------------------------- Linearization ---

	/// linearization of virtual neighbors
	void linearize() {

		// linearize and ringify routes delegated to me
		for (size_t i = 0; i < table.size(); i++) {
			entry_t& ie = *table[i];

			// get entries announced from the node
			if (!ie.delegated || ie.own()) continue;

			// process linearization neighbors
			for (size_t j = 1; j<table.size(); j++) {
				size_t k = i + j;
				if (k >= table.size() || table[k]->own()) break;
				entry_t& ke = *table[k];

				if (!ke.delegated) continue;

				bool is_helping =
					ie.can_consume(ke.id) || ke.can_consume(ie.id);
				bool has_changes = ie.changed || ke.changed;

				if (is_helping && has_changes) {

					direction_t idir = left_right(i);
					direction_t kdir = left_right(k);

					// check if neighbors should be linearized
					bool delegate = false;
					if (ie.delegated_new && idir == dir_left && kdir == dir_left) {
						ie.delegated_new = false;
						delegate = true;
					} else
					if (ke.delegated_new && idir==dir_right && kdir==dir_right) {
						ke.delegated_new = false;
						delegate = true;
					}

					// linearize pair of neighbor
					if (delegate) {
						ie.consume(ke.id);
						ke.consume(ie.id);
						dsdv.delegate(ke.id,ie.id);
						dsdv.delegate(ie.id,ke.id);
					}
				}
				break;
			}
		}
	}

	/// linearization: mark virtual neighbors
	void linearization_mark_change( int index ) {
		assert( index >= 0 && index < (int)size() );

		// get direction
		int dir = left_right( index );

		// left
		if ( dir == dir_left && index != 0 ) {
			table[index-1]->delegated_new = true;
			table[index-1]->changed = true;
		}

		// right
		if ( dir == dir_right && (index+1) < (int)size() ) {
			table[index+1]->delegated_new = true;
			table[index+1]->changed = true;
		}
	}

private://------------------------------------------------------- Discovery ---

	// send discovery messages
	void discover() {
		stable++;
		if (stable > delay_threshold) stable_readvertize++;
		if (stable_readvertize < (stable-delay_threshold)) return;
		stable_readvertize = 0;

		// send discovery messages of my own id's
		for (size_t i=0; i<table.size(); i++) {

			// only process changed own entries
			entry_t& e = *table[i];
			if (!e.own()) continue;

			// issue parallel discovery messages
			unordered_set<nodeid_t> neighbors;
			neighbors = dsdv.get_neighbors();
			for (int s=0; s<discovery_alpha; s++) {
				if (neighbors.size()==0) break;
				unordered_set<nodeid_t>::iterator i=neighbors.begin();
				if (neighbors.size()>2) std::advance(i,intuniform(rng, 0, neighbors.size()-1));
				if (i == neighbors.end()) continue;
				queue.send( *i, e.route(vroute_update_t::DISCOVERY_PRED) );
				queue.send( *i, e.route(vroute_update_t::DISCOVERY_SUCC) );
				neighbors.erase( *i );
			}
		}
	}

	// process discovery of closest node
	void process_discovery( nodeid_t from, const vroute_update_t& route ) {

		// add route using the dsdv component
		dsdv.process( from, route(vroute_update_t::UPDATE) );

		// get source and destination routes
		int src_idx = index_of(route.id);

		// check if entries are available
		if (src_idx == -1) return;

		// set source, destination
		entry_t* src = table[src_idx];
		entry_t* dst = NULL;

		// get distance to best successor
		if ( route.update_type == vroute_update_t::DISCOVERY_SUCC ) {
			entry_t* succ = get( src_idx, +1 );
			if ( succ != NULL ) dst = succ;
		}

		// get distance to best predecessor
		if ( route.update_type == vroute_update_t::DISCOVERY_PRED ) {
			entry_t* pred = get( src_idx, -1 );
			if ( pred != NULL ) dst = pred;
		}

		if ( dst==NULL ||  dst->next_hop == from ) return;

		// closest node? -> register delegated route
		if ( dst->own() ) {
			if (dst->own()) {
				src->delegated = true;
				dsdv.delegate(dst->id, src->id);
				discovery_mark_change(src_idx);
			}
		}

		// route message
		vroute_update_t nroute = src->route( route.update_type );
		nroute.dest = dst->id;
		src->record_use( dst->next_hop );
		queue.send( dst->next_hop, nroute );
	}

	/// notify virtual neighbors
	void discovery_mark_change( int index ) {
		assert( index >= 0 && index < (int)size() );

		// do not mark changes when entry is unmodified
		if ( !table[index]->changed ) return;

		// reset stable counter
		stable = 0;

		// inform neighbors
		if (index>0) dsdv.notify(get(index,-1)->id);
		if (index<((int)size()-1)) dsdv.notify(get(index,1)->id);
	}
};

} /* namespace routingsim */

#endif /* VIRTUALRINGTABLE_HPP_ */
