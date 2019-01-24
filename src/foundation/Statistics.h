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

#ifndef __ROUTINGSIM_STATISTICS_H_
#define __ROUTINGSIM_STATISTICS_H_

#include <omnetpp.h>
#include <vector>
#include <boost/unordered_map.hpp>
#include <boost/unordered_set.hpp>
#include <chistogram.h>

#include "BaseTypes.h"

namespace routingsim {

using namespace boost;

class StatisticSample;

/**
 * Samples statistics from several modules.
 *
 * @author Sebastian Mies <mies@kit.edu>
 * @author Christoph Werle <werle@kit.edu>
 */
class Statistics: public cSimpleModule {
protected:
	virtual void initialize(int stage);
	virtual void finish();
	virtual void handleMessage(cMessage *msg);
    virtual int numInitStages() const;

private:
	//const char* name;
	class Value;
	typedef boost::unordered_map<const char*, Value*> Values;
	Values values;
	simtime_t samplingInterval;
	cMessage* samplingTimer;
	std::vector<StatisticSample*> samples;
	bool recordShortestPathDeliveryRatio;

	// message delivery ratio accounting
	boost::unordered_set<cMessage*> msgs;
	cOutVector* deliveryRatio;
	cOutVector* messagesDroppedVector;
	cOutVector* messagesDeliveredVector;
	cHistogram* hopCount;
	cHistogram* realHopCount;
	cHistogram* addStretch;
	cHistogram* mulStretch;
	unsigned int messagesDropped;
	unsigned int messagesDelivered;
	cOutVector* shortestPathDeliveryRatio;
	cOutVector* shortestPathMessagesDroppedVector;
	cOutVector* shortestPathMessagesDeliveredVector;
	unsigned int shortestPathMessagesDropped;
	unsigned int shortestPathMessagesDelivered;

	void sample();

public:
	Statistics();
	virtual ~Statistics();

	void addSample( StatisticSample* sample );
	void value(const char* name, double value = 1, bool average = true);
	void valueTrigger(const char* name, double value);

	// delivery ratio of messages

	/// register a message
	void registerMessage( cMessage* msg );

	/// notify that the message has been dropped;
	/// as dstNode pass the final destination of the packet
	void dropMessage( cMessage* msg, nodeid_t dstNode  );

	/// notify that the message has been delivered
	void deliveredMessage( cMessage* msg, int hopCount );
};

extern const char* STATS_MAIN;
extern const char* STATS_TRAFFIC;

/// Returns the statistics singleton
Statistics& getStatistics( const char* name = NULL );

/// Adds a statistic sample to the statistics module
void addStatisticSample( StatisticSample* sample, const char* name = NULL );

/// gets a preconfigured cHistogram in integer mode
cHistogram* getIntHistogram(const char* name, int rangeLower = 0, int rangeUpper = 128, int numBins = 128);


} //namespace

#endif
