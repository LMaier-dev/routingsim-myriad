//
// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU Lesser General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
// 
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU Lesser General Public License for more details.
// 
// You should have received a copy of the GNU Lesser General Public License
// along with this program.  If not, see http://www.gnu.org/licenses/.
// 

#ifndef __ROUTINGSIM_DISCOOBSERVER_H_
#define __ROUTINGSIM_DISCOOBSERVER_H_

#include "Disco.h"
#include <omnetpp.h>
#include "NetworkManager.h"

namespace routingsim {

using namespace std;

class Disco;

struct discoid_t;

/**
 * TODO - Generated class
 */
class DiscoObserver : public cSimpleModule
{
private:
	vector<int> entries;
	NetworkManager *networkMgr;
    int NumofLandmarkAnnouncements;

	std::set<int> sampleNodes(int n);
	Disco* getDisco(int id) const;

  protected:
    virtual void initialize() override;
    virtual void finish() override;
    virtual void handleMessage(cMessage *msg) override;

    void recordStatistics();
    void recordLinkUsage() const;

  public:
    DiscoObserver();
    ~DiscoObserver();

    /// add a node to the observer's entries
    void registerNode(Disco *DiscoModule);

    /// remove a node to the observer's entries
    void unregisterNode(Disco *DiscoModule);


    /// return a random node by its discoid_t from the available nodes
    discoid_t &getRandomNodeAddress() const;

    /// return the address of a node to a given node id
    discoid_t &getNodeAddress(nodeid_t id ) const;

    /// return the cost / distance between two nodes identified by their
    /// discoid_t
    float getDistance(discoid_t source, discoid_t destination) const;

    /// return network size
    int getNetworkSize() const;

    /// return number of landmarks
    int getNumLandmarks() const;

    int getNumofLandmarkAnnouncements() const;
    void setNumofLandmarkAnnouncements();
    void setNumofLandmarkAnnouncements(int n);

};

DiscoObserver *getDiscoObserver();


} //namespace

#endif
