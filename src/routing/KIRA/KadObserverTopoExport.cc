//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// as published by the Free Software Foundation; either version 2
// of the License, or (at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with this program; if not, write to the Free Software
// Foundation, Inc., 59 Temple Place - Suite 330, Boston, MA  02111-1307, USA.

/**
 * @file KadObserverTopoExport.cc
 * @author Paul Seehofer
 */

#ifdef R2K_RT_EXPORT
#include "KadObserver.h"
#include "KadObserverTopoExport.h"
#include "nlohmann/json.hpp"
#include <fstream>
#include <map>

namespace routingsim {


using json = nlohmann::json;

void KadObserver::exportRoutingTables(std::ofstream file) const {
    for (const auto& [id,ignored] : nodeid_to_key) {
        json j;
        j["nodeid"] = id;

        auto& rt = getR2Kademlia(id).getRoutingTable();
        size_t bucketCount(rt.getBucketCount());
        std::vector<kad_bucket> buckets; 
        for (int i = 0; i < bucketCount; i++) {
            kad_bucket bucket;
            for(auto& contact: rt.getValidContactsFromBucket(i)) {
                nodeid_t contactId(getNodeId(contact->getKey()));
                node_path path;
                path.push_back(id);
                for(auto& hop: contact->getPath()) {
                    path.push_back(getNodeId(hop));
                }
                path.push_back(contactId);
                bucket[contactId] = path;
            }
            buckets.push_back(bucket);
        }
        j["buckets"] = buckets;
        j["bucketSize"] = rt.getSplitSize();
        j["accelerationFactor"] = rt.getAccelerationFactor();
        file << j << endl;
    }
    file.close();
}

void KadObserver::exportOverlayKeyNodeIdMapping(std::ofstream file) const {
    std::map<nodeid_t, std::string> mapping {}; 
    for (const auto& [id,key] : nodeid_to_key) {
        mapping[id] = key.str();
    }
    json j_map(mapping);
    file << j_map << endl;
    file.close();
}
}
#endif
