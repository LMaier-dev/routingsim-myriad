#ifndef VIRTUALRINGANYCAST_H_
#define VIRTUALRINGANYCAST_H_

#include<vector>

#include "VirtualRingEntry.h"
#include "VirtualRingQueue.h"
#include "VirtualRingRoutes.h"

namespace routingsim {

using namespace std;
using namespace boost;

class anycast_table : public route_maintenance::listener, public ring_table::listener {
private:
	ring_table& ring;
	route_maintenance& dsdv;
	update_queue& queue;
	boost::unordered_set<entry_t*> table;
	boost::unordered_set<entry_t*> maintained;
	boost::unordered_set<entry_t*> announced;
	int stable;
	int stable_readvertize;
	int delay_threshold;

public:
	anycast_table( ring_table& ring ) :
		ring(ring), dsdv(ring.get_dsdv()), queue(dsdv.get_queue()){
		dsdv.register_listener(this);
		ring.register_listener(this);
		stable = 0;
		stable_readvertize = 0;
		delay_threshold = 5;
	}

	virtual ~anycast_table() {
	}

	/// sets the number of parallel discoveries
	void set_delay_threshold( unsigned int threshold ) {
		this->delay_threshold = threshold;
	}

	boost::unordered_set<entry_t*>& get_entries() {
		return table;
	}

protected:// ------------------------------------------------ dsdv listener ---

	virtual void entry_notify( entry_t* entry ) {
		if (entry == NULL || entry->route_type == entry_t::ANYCAST) {
			stable = 0;
		}
	}

	virtual void entry_update( entry_t* entry, bool new_entry ) {
		if (entry->route_type != entry_t::ANYCAST || !new_entry) return;
		table.insert(entry);
		stable = 0;
	}

	virtual void entry_remove( entry_t* entry ) {
		if (entry->route_type != entry_t::ANYCAST) return;
		stable=0;
		table.erase(entry);
		maintained.erase(entry);
		announced.erase(entry);

		// inform others
		foreach( entry_t* e, table )
			if (e->id == entry->id)
				dsdv.notify(e->id, true, e->owner);
	}

	virtual bool entry_needed( entry_t* entry ) {
		if (entry->route_type != entry_t::ANYCAST) return false;
		bool used = false;
		foreach(entry_t* e, table)
			if (e->id == entry->id) used |= (e->used_by.size()!=0);
		const entry_t* nhe = ring.next_hop( entry->id );
		return (nhe!= NULL && nhe->own()) || used;
	}

	virtual void vring_changed() {
		foreach( entry_t* e, maintained ) {
			const entry_t* nhe = ring.next_hop( e->id );
			if (!nhe->own()) dsdv.notify(e->id, false, e->owner);
		}
		stable = 0;
	}

public:

	// add anycast identifier
	void add_id( vid_t vid ) {
		dsdv.add_id(vid, entry_t::ANYCAST, dsdv.get_nodeid()+2);
	}

	/// remove anycast identifier
	void remove_id( vid_t vid ) {
		dsdv.remove_id(vid);
	}

	/// returns the next anycast hop
	entry_t* next_hop( vid_t vid ) {
		entry_t* best = NULL;
		foreach( entry_t* e, table )
			if ( e->id == vid && e->better_than(best) ) best = e;
		return best;
	}

	/// process an update
	bool process( nodeid_t from, const vroute_update_t& route ) {
		switch(route.update_type) {
		case vroute_update_t::ANNOUNCEMENT:
			process_announcement(from,route);
			return true;
		default:
			break;
		}
		return false;
	}

	/// trigger update
	void maintenance() {
		send_announcements();
	}

private:

	void send_announcements() {
		stable++;
		if (stable > delay_threshold) stable_readvertize++;
		if (stable_readvertize < (stable-delay_threshold)) return;
		stable_readvertize = 0;

		foreach(entry_t* e, table) {
			if (!e->own()) continue;

			// inform neighbor about the vid
			if (e->changed) {
				foreach (nodeid_t nid, dsdv.get_neighbors()) {
					e->record_use(nid);
					queue.send(nid, e->route(vroute_update_t::UPDATE));
				}
			}

			// get next hop and announce route
			entry_t* nhe = ring.next_hop(e->id);
			if (nhe==NULL || nhe->own())
				continue;

			e->record_use(nhe->next_hop);
			queue.send(nhe->next_hop, e->route(vroute_update_t::ANNOUNCEMENT));

		}
	}

	void process_announcement( nodeid_t from, const vroute_update_t& route ) {

	    // register route
		dsdv.process( from, route(vroute_update_t::UPDATE) );

		// add announced entry
		entry_t* e = dsdv.get(route.id,route.owner);
		if (e==NULL) return;
		announced.insert( e );

		// blacklisted, or not adapted? -> stop announcement
		foreach( entry_t* te, table )
			if ( te->id == route.id && te->better_than(e) && announced.count(te)!=0 ) return;

		// get next hop and announce route
		const entry_t* nhe = ring.next_hop(route.id);

		// split horizon
		if (nhe->next_hop == from) return;

		// closest node on the ring? yes-> add to maintained anycast routes
		if (nhe != NULL && !nhe->own()) {
			e->record_use(nhe->next_hop);
			queue.send(nhe->next_hop, e->route(vroute_update_t::ANNOUNCEMENT));
		} else
			maintained.insert(e);
	}
};

} /* namespace routingsim */

#endif /* VIRTUALRINGANYCAST_H_ */
