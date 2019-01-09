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

#include "Statistics.h"
#include "StatisticSample.h"
#include "NetworkManager.h"

#include<boost/unordered_map.hpp>
#include<boost/foreach.hpp>
#include<boost/lexical_cast.hpp>
#include<algorithm>
#include<string>
#include<omnetpp.h>

namespace routingsim {

using namespace std;
using namespace boost;

Define_Module(Statistics);

static Statistics* stats = NULL;
static boost::unordered_map<const char*, Statistics*> statsMap;

// convinience method, used in Statistics::Value()
cHistogram* getDensityHistogram(const char* name1, const char* name2, size_t size) {
	cHistogram* hist = new cHistogram((string(name1) + string(name2)).c_str());
	hist->setNumBinsHint(512);
	hist->setNumPrecollectedValues(size*50);
	hist->setRangeExtensionFactor(2.0);
	return hist;
}

// convinience method, used in Statistics::initialize and MessageStatistics::MessageStatistics()
cHistogram* getIntHistogram(const char* name, int rangeLower, int rangeUpper, int numBins) {
	cHistogram* hist = new cHistogram(name);
	hist->setRange(rangeLower, rangeUpper);
	hist->setNumBinsHint(numBins);
	hist->setMode(cHistogram::MODE_INTEGERS);
	return hist;
}

// this class describes a sampling value
class Statistics::Value {
public:
	int n;
	double avg;
	double min;
	double max;
	double lastvalue;
	double lastavg;
	double lastmin;
	double lastmax;
	simtime_t lastLog;
	bool first;
	cOutVector* avgVec;
	cOutVector* avgVecDelta;
	cOutVector* minVec;
	cOutVector* minVecDelta;
	cOutVector* maxVec;
	cOutVector* maxVecDelta;
	cHistogram* histogram;
	cHistogram* histogramAbs;
	cHistogram* histogramDelta;
	int f;

	double eqTrigger;
	string eqTriggerName;
	int eqTriggerCount;
	int eqTriggerNum;
public:
	Value(const char* name, size_t size ) :
		n(0), avg(0), min(INFINITY), max(-INFINITY), lastavg(0), lastmin(0),
		lastmax(0) {
		f=0;
		avgVec = new cOutVector((string(name) + string("Avg")).c_str());
		minVec = new cOutVector((string(name) + string("Min")).c_str());
		maxVec = new cOutVector((string(name) + string("Max")).c_str());
		avgVecDelta = new cOutVector(
				(string(name) + string("AvgDelta")).c_str());
		minVecDelta = new cOutVector(
				(string(name) + string("MinDelta")).c_str());
		maxVecDelta = new cOutVector(
				(string(name) + string("MaxDelta")).c_str());
		histogram = getDensityHistogram(name, "AvgDensity", size);
		histogramAbs = getDensityHistogram(name, "Density", size);
		histogramDelta = getDensityHistogram(name, "AvgDeltaDensity", size);
		eqTriggerName = (string(name) + string("Trigger"));
		eqTrigger = INFINITY;
		lastLog = simTime();
		first = true;
	}

	~Value() {
		delete avgVec;
		delete minVec;
		delete maxVec;
		delete avgVecDelta;
		delete minVecDelta;
		delete maxVecDelta;
		delete histogram;
		delete histogramAbs;
		delete histogramDelta;
	}

	void finish(cSimpleModule* module) {
		histogram->record();
		histogramAbs->record();
		histogramDelta->record();
		if (eqTrigger != INFINITY) {
			module->recordScalar(eqTriggerName.c_str(),
					((double) eqTriggerCount / (double) eqTriggerNum));
		}
	}

	void add(double value, bool average = true) {
		if (!first) histogramAbs->collect(value);
		avg += value;
		min = value < min ? value : min;
		max = value > max ? value : max;
		if (average)
			n++;
		else
			n = 1;
	}

	void equalTrigger(double trigger) {
		eqTrigger = trigger;
	}

	void record(cSimpleModule* module) {
		double value = avg / n;
		if (eqTrigger != INFINITY) {
			if (eqTrigger == value)
				eqTriggerCount++;
			eqTriggerNum++;
		}
		histogram->collect(value);
		avgVec->record(value);
		minVec->record(min);
		maxVec->record(max);
		if (!first) {
			double dx = (simTime() - lastLog).dbl();
			avgVecDelta->record((value - lastavg) / dx);
			histogramDelta->collect((value - lastavg) / dx);
			minVecDelta->record((min - lastmin) / dx);
			maxVecDelta->record((max - lastmax) / dx);
		}
		lastavg = value;
		lastmin = min;
		lastmax = max;
		lastLog = simTime();
		first = false;
		avg = 0;
		max = -INFINITY;
		min = INFINITY;
		n = 0;
	}
};

int Statistics::numInitStages() const {
	return 4;
}

Statistics::Statistics() {
	samplingTimer = NULL;
	deliveryRatio = NULL;
	messagesDeliveredVector = NULL;
	messagesDroppedVector = NULL;
	shortestPathDeliveryRatio = NULL;
	shortestPathMessagesDeliveredVector = NULL;
	shortestPathMessagesDroppedVector = NULL;
	hopCount = NULL;
	realHopCount = NULL;
	addStretch = NULL;
	mulStretch = NULL;
}

void Statistics::initialize(int stage) {
	if (stage==0) {
		// start sampling
		samplingInterval = par("samplingInterval");
		recordShortestPathDeliveryRatio = par("recordShortestPathDeliveryRatio");
		samplingTimer = new cMessage("samplingTimer");
		messagesDelivered = 0;
		messagesDropped = 0;
		shortestPathMessagesDelivered = 0;
		shortestPathMessagesDropped = 0;
		deliveryRatio = new cOutVector("deliveryRatio");
		messagesDeliveredVector = new cOutVector("messagesDelivered");
		messagesDroppedVector = new cOutVector("messagesDropped");
		if (recordShortestPathDeliveryRatio) {
			shortestPathDeliveryRatio = new cOutVector("shortestPathDeliveryRatio");
			shortestPathMessagesDeliveredVector = new cOutVector("shortestPathMessagesDelivered");
			shortestPathMessagesDroppedVector = new cOutVector("shortestPathMessagesDropped");
		}
		hopCount = getIntHistogram("hopCountRouting");
		realHopCount = getIntHistogram("hopCountReal");
		addStretch = getIntHistogram("stretchAdditive");
		mulStretch = new cHistogram("stretchMultiplicative");
		mulStretch->setRange(1.0,64.0);
		mulStretch->setNumBinsHint(1024);
		scheduleAt(simTime() + samplingInterval, samplingTimer);
	}
	if (stage==3)
		sample();
}

void Statistics::finish() {
	foreach( Values::value_type& value, values ) {
		value.second->finish(this);
		delete value.second;
	}
	values.clear();
	recordScalar("simTime", simTime());
	cancelAndDelete(samplingTimer);
	hopCount->record();
	realHopCount->record();
	addStretch->record();
	mulStretch->record();
	msgs.clear();
}

Statistics::~Statistics() {
	boost::unordered_map<const char*, Statistics*>::iterator it;
	delete deliveryRatio;
	delete messagesDeliveredVector;
	delete messagesDroppedVector;
	delete shortestPathDeliveryRatio;
	delete shortestPathMessagesDeliveredVector;
	delete shortestPathMessagesDroppedVector;
	delete hopCount;
	delete realHopCount;
	delete addStretch;
	delete mulStretch;

	//delete all statistics and set to NULL
	for (it=statsMap.begin(); it!= statsMap.end(); it++) {
		if (it->second == this)
			statsMap.erase(it);
	}
	if (stats == this)
		stats = NULL;
}

const char* deliveryRatioStr = "messageDeliveryRatio";
const char* messagesDeliveredStr = "messagesDelivered";
const char* messagesDroppedStr = "messagesDropped";

void Statistics::sample() {
	if ((messagesDelivered+messagesDropped)!=0) {
		deliveryRatio->record((double)messagesDelivered/((double)messagesDelivered+(double)messagesDropped));
		messagesDeliveredVector->record(messagesDelivered);
		messagesDroppedVector->record(messagesDropped);
		if (recordShortestPathDeliveryRatio) {
			shortestPathDeliveryRatio->record((double)shortestPathMessagesDelivered/((double)shortestPathMessagesDelivered+(double)shortestPathMessagesDropped));
			shortestPathMessagesDeliveredVector->record(shortestPathMessagesDelivered);
			shortestPathMessagesDroppedVector->record(shortestPathMessagesDropped);
		}
	}
	messagesDelivered = 0;
	messagesDropped = 0;
	shortestPathMessagesDelivered = 0;
	shortestPathMessagesDropped = 0;

	foreach( StatisticSample* sample, samples )	sample->sampleStatistics(*this);
	foreach( Values::value_type& value, values ) value.second->record(this);
}

void Statistics::handleMessage(cMessage *msg) {
	sample();
	scheduleAt(simTime() + samplingInterval, samplingTimer);
}

void Statistics::value(const char* name, double value, bool average) {
	if (values.count(name) == 0)
		values.insert(make_pair(name, new Value(name, samples.size())));
	values[name]->add(value, average);
}

void Statistics::valueTrigger(const char* name, double value) {
	values[name]->equalTrigger(value);
}

void Statistics::addSample(StatisticSample* sample ) {
	samples.push_back(sample);
}

void Statistics::registerMessage( cMessage* msg ) {
	msgs.insert(msg);
	cModule* srcNode = getSimulation()->getContextModule()->getParentModule();
	msg->setContextPointer(srcNode);
}

void Statistics::dropMessage( cMessage* msg, nodeid_t dstNode ) {
	if (msgs.count(msg)==0) return;

	nodeid_t srcNode = dynamic_cast<Node *>(static_cast<cModule*>(msg->getContextPointer()))->getNodeId();

	if (recordShortestPathDeliveryRatio) {
		if (getNetworkManager()!=NULL) {
			if (getNetworkManager()->getTopology().FullTopologyPathFailed(srcNode, dstNode)) {
				shortestPathMessagesDropped++;
			} else {
				shortestPathMessagesDelivered++;
			}
		}
	}

	messagesDropped++;
	msgs.erase(msg);
}

const char* stretchStr="stretch";

void Statistics::deliveredMessage( cMessage* msg, int hop_count ) {
	if (msgs.count(msg)==0) return;
	hopCount->collect(hop_count);
	nodeid_t srcNode = dynamic_cast<Node *>(static_cast<cModule*>(msg->getContextPointer()))->getNodeId();
	nodeid_t dstNode = dynamic_cast<Node *>(getSimulation()->getContextModule()->getParentModule())->getNodeId();
	if (getNetworkManager()!=NULL) {
		int realDistance = (int)getNetworkManager()->getDistance(srcNode, dstNode);
		if (realDistance >= 0) {
			realHopCount->collect(realDistance);
			addStretch->collect(hop_count-realDistance);
			if (realDistance>0.5) {
				double stretch = (double)hop_count / (double)realDistance;
				mulStretch->collect(stretch);
			}
		}

		if (recordShortestPathDeliveryRatio) {
			if (getNetworkManager()->getTopology().FullTopologyPathFailed(srcNode, dstNode)) {
				shortestPathMessagesDropped++;
			} else {
				shortestPathMessagesDelivered++;
			}
		}
	}

	messagesDelivered++;
	msgs.erase(msg);
}




const char* STATS_MAIN = "mainStatistics";
const char* STATS_TRAFFIC = "trafficStatistics";

inline Statistics& createStats( const char* name = STATS_MAIN ) {
	cModuleType* type =
			cModuleType::get("routingsim.foundation.Statistics");
	cModule* mod = getSimulation()->getContextModule();
	while (mod->getParentModule() != NULL)
		mod = mod->getParentModule();
	Statistics& s = *dynamic_cast<Statistics*> (type->create(name, mod));
	return s;
}

Statistics& getStatistics( const char* name ) {
	if (name != NULL) {
		if (statsMap.count(name)==0) statsMap.insert(make_pair(name, &createStats(name)));
		return *statsMap[name];
	}
	if (name == NULL && stats == NULL) stats = &createStats();
	return *stats;
}

void addStatisticSample(StatisticSample* sample, const char* name ) {
	getStatistics(name).addSample(sample);
}

} //namespace
