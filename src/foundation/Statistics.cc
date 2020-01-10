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

#include<boost/lexical_cast.hpp>
#include<algorithm>
#include<string>
#include<omnetpp.h>

namespace routingsim {
Define_Module(Statistics);

static Statistics* stats = nullptr;
static std::unordered_map<const char*, Statistics*> statsMap;

// convenience method, used in Statistics::Value()
cHistogram* getDensityHistogram(const char* name1, const char* name2, size_t size) {
	cHistogram* hist = new cHistogram((string(name1) + string(name2)).c_str());
	hist->setNumBinsHint(512);
	hist->setNumPrecollectedValues(size*50);
	hist->setRangeExtensionFactor(2.0);
	return hist;
}

// convenience method, used in Statistics::initialize and MessageStatistics::MessageStatistics()
cHistogram* getIntHistogram(const char* name, int rangeLower, int rangeUpper, int numBins) {
	cHistogram* hist = new cHistogram(name);
	hist->setRange(rangeLower, rangeUpper);
	hist->setNumBinsHint(numBins);
	hist->setMode(cHistogram::MODE_INTEGERS);
	return hist;
}




// this class describes a mean sampling value
class Statistics::Value {
public:
	static constexpr unsigned short qvaluenumber = 10;
	static const double qvalues[qvaluenumber];

	int n;
	double avg; // note: this contains the sum of all samples until record is called
	double min; // min average over sample set
	double max; // max average over sample set
	double lastvalue;
	double lastavg;
	double lastmin;
	double lastmax;
	simtime_t lastLog;
	bool first;
	cOutVector* avgVec; // Average over sample set
	cOutVector* avgVecDiffQuot; // Difference Quotient
	cOutVector* minVec;
	cOutVector* minVecDiffQuot; // Difference Quotient
	cOutVector* maxVec;
	cOutVector* maxVecDiffQuot; // Difference Quotient
	cHistogram* histogram; // histogram of average values
	cHistogram* histogramAbs; // histogram of actual values from sample set
	cHistogram* histogramDiffQuot; // histogram of difference quotient from average values
	cOutVector* quantileVec[qvaluenumber];
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
		avgVecDiffQuot = new cOutVector(
				(string(name) + string("AvgDiffQuot")).c_str());
		minVecDiffQuot = new cOutVector(
				(string(name) + string("MinDiffQuot")).c_str());
		maxVecDiffQuot = new cOutVector(
				(string(name) + string("MaxDiffQuot")).c_str());
		histogram = getDensityHistogram(name, "AvgDensity", size);
		histogramAbs = getDensityHistogram(name, "Density", size);
		histogramDiffQuot = getDensityHistogram(name, "AvgDiffQuotDensity", size);
		// initialize quantile vectors
		for (unsigned int i= 0; i < qvaluenumber; i++)
		{
			quantileVec[i]= new cOutVector((string(name)+ string("Q") + std::to_string((int) (qvalues[i]*100.0))).c_str());
		}
		eqTriggerName = (string(name) + string("Trigger"));
		eqTrigger = INFINITY;
		lastLog = simTime();
		first = true;
	}

	~Value() {
		delete avgVec;
		delete minVec;
		delete maxVec;
		delete avgVecDiffQuot;
		delete minVecDiffQuot;
		delete maxVecDiffQuot;
		delete histogram;
		delete histogramAbs;
		delete histogramDiffQuot;
		for (unsigned int i= 0; i < qvaluenumber; i++)
		{
			delete quantileVec[i];
			quantileVec[i]= nullptr;
		}
	}

	void finish(cSimpleModule* module) {
		histogram->record();
		histogramAbs->record();
		histogramDiffQuot->record();

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

	void calcQuantiles() {
		// this function calculates a number of quantiles given in qvalues[] from Absolute Values of each
		// sampling node (not the average value) as stored in histogram
		// this is more efficient than running getCDF() in a loop, because we are doing it in one sweep
		int numBins = histogramAbs->getNumBins();
		if (numBins <= 0) // cannot work with empty histogram, sorry
			return;

		unsigned int qindex= 0;
		double lowerEdge = histogramAbs->getBinEdge(0);
		double upperEdge = histogramAbs->getBinEdge(0);
		double sum= 0;
		double sumweights= histogramAbs->getSumWeights();
		double delta = 0;

		// TODO: use underflowbin first and overflowbin as last
		int i= 0; 
		while (qindex < qvaluenumber and i < numBins)
		{
			lowerEdge = histogramAbs->getBinEdge(i);
			upperEdge = histogramAbs->getBinEdge(i + 1);
			delta = histogramAbs->getBinValue(i) / sumweights;

			if ( (sum + delta) < qvalues[qindex] ) 
			{ // sum + delta does not exceed quantile value yet
				// proceed with next bin
				i++;
				sum+= delta;
			}
			else // (sum + delta >= qvalues[qindex])
			{
				// current bin (+sum) hits or exceeds desired quantile
				// calculate x based on interpolation of partial bin
				quantileVec[qindex]->record( ((qvalues[qindex] - sum) / delta * (upperEdge-lowerEdge)) + lowerEdge );

				// advance quantile index
				qindex++;
			}
		} // end while

		// since the histogram should sum up to 1, all quantile values should have been covered
	}

	void record(cSimpleModule* module) {
		ASSERT(n != 0);
		double value = avg / n; // value is now the average value
		//EV_STATICCONTEXT
		//EV_TRACE << "Value::record(" << avgVec->getName() << "), n=" << n << " avg=" << avg << " value: " << value << endl;
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
			// DiffQuot is the difference quotient (= average rate of change)
			avgVecDiffQuot->record((value - lastavg) / dx);
			//EV_TRACE << "Value::record(" << avgVecDiffQuot->getName() << "), lastavg:" << lastavg << " diff:" << (value - lastavg) << " delta:" << (value - lastavg)/dx << " dx=" << dx << endl;

			histogramDiffQuot->collect((value - lastavg) / dx);
			minVecDiffQuot->record((min - lastmin) / dx);
			maxVecDiffQuot->record((max - lastmax) / dx);
			calcQuantiles();
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


constexpr const double Statistics::Value::qvalues[]= { 0.50, 0.60, 0.70, 0.75, 0.80, 0.85, 0.90, 0.95, 0.98, 0.99 };


int Statistics::numInitStages() const {
	return 4;
}

Statistics::Statistics() {
	samplingTimer = nullptr;
	deliveryRatio = nullptr;
	messagesDeliveredVector = nullptr;
	messagesDroppedVector = nullptr;
	shortestPathDeliveryRatio = nullptr;
	shortestPathMessagesDeliveredVector = nullptr;
	shortestPathMessagesDroppedVector = nullptr;
	hopCount = nullptr;
	realHopCount = nullptr;
	addStretch = nullptr;
	mulStretch = nullptr;
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
		if (simTime() >= getSimulation()->getWarmupPeriod())
			scheduleAt(simTime() + samplingInterval, samplingTimer);
		else
			scheduleAt(getSimulation()->getWarmupPeriod(), samplingTimer);
	}
	if (stage == 3)
		if (simTime() >= getSimulation()->getWarmupPeriod())
			sample();
}

void Statistics::finish() {
	for (auto& value : values) {
		value.second->finish(this);
	}
	recordScalar("simTime", simTime());
	cancelEvent(samplingTimer);
	hopCount->record();
	realHopCount->record();
	addStretch->record();
	mulStretch->record();
}

Statistics::~Statistics() {
	cancelAndDelete(samplingTimer);

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

	for (auto& value : values)
		delete value.second;
	values.clear();
	msgs.clear();

	// delete all statistics and set to nullptr
	for (auto it = statsMap.begin(); it != statsMap.end(); it++) {
		if (it->second == this)
			statsMap.erase(it);
	}
	// std::cerr << "statsMap: size=" << statsMap.size() << endl;

	if (stats == this)
		stats = nullptr;
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

	for (StatisticSample* sample : samples)	sample->sampleStatistics(*this);
	for (Values::value_type& value : values) value.second->record(this);
}

void Statistics::handleMessage(cMessage *msg) {
	sample();
	scheduleAt(simTime() + samplingInterval, samplingTimer);
}

void Statistics::value(const char* name, double value, bool average) {
	ASSERT(not std::isnan(value));
	ASSERT(not std::isnan(average));
	if (values.count(name) == 0)
		values.insert(std::make_pair(name, new Value(name, samples.size())));
	values[name]->add(value, average);
}

void Statistics::valueTrigger(const char* name, double value) {
	values[name]->equalTrigger(value);
}

void Statistics::addSample(StatisticSample* sample ) {
	samples.push_back(sample);
}

void Statistics::registerMessage( cMessage* msg ) {
	if (simTime() < getSimulation()->getWarmupPeriod()) return;
	msgs.insert(msg);
	// save current context module for getting the source node later
	msg->setContextPointer(getSimulation()->getContextModule());
}

void Statistics::dropMessage( cMessage* msg, nodeid_t dstNode ) {
	if (msgs.count(msg)==0) return;

	if (!msg->getContextPointer())
	{
		EV_ERROR << "Message not registered: " << msg->getName() << " Id:" << msg->getId() << endl;
		return;
	}
	// the context pointer is set in registerMessage
	nodeid_t srcNode = Node::getNode(static_cast<cModule*>(msg->getContextPointer()))->getNodeId();

	if (recordShortestPathDeliveryRatio and getNetworkManager() != nullptr) {
		if (getNetworkManager()->getTopology().fullTopologyPathFailed(srcNode, dstNode)) {
			shortestPathMessagesDropped++;
		} else {
			shortestPathMessagesDelivered++;
		}
	}
	messagesDropped++;
	msgs.erase(msg);
}

const char* stretchStr="stretch";

void Statistics::deliveredMessage( cMessage* msg, int hop_count ) {
	if (msgs.count(msg) == 0) return;
	hopCount->collect(hop_count);
	// the context pointer is set in registerMessage
	if (!msg->getContextPointer())
	{
		EV_ERROR << "Message not registered: " << msg->getName() << " Id:" << msg->getId() << endl;
		return;
	}
	nodeid_t srcNode = Node::getNode(static_cast<cModule*>(msg->getContextPointer()))->getNodeId();
	nodeid_t dstNode = Node::getNode(getSimulation()->getContextModule())->getNodeId();
	if (getNetworkManager()!=nullptr) {
		int realDistance = (int)getNetworkManager()->getDistance(srcNode, dstNode);
		if (realDistance >= 0) {
			realHopCount->collect(realDistance);
			addStretch->collect(hop_count-realDistance);
			if (realDistance>0.5) {
				double stretch = (double)hop_count / (double)realDistance;
				ASSERT(not std::isnan(stretch));
				mulStretch->collect(stretch);
			}
		}

		if (recordShortestPathDeliveryRatio) {
			if (getNetworkManager()->getTopology().fullTopologyPathFailed(srcNode, dstNode)) {
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

inline Statistics *createStats( const char* name = STATS_MAIN ) {
	cModuleType* type =
			cModuleType::get("routingsim.foundation.Statistics");
	cModule* mod = getSimulation()->getModuleByPath("<root>");
	return dynamic_cast<Statistics*> (type->create(name, mod));
}

Statistics& getStatistics( const char* name ) {
	if (name != nullptr) {
		if (statsMap.count(name)==0)
			statsMap.insert(std::make_pair(name, createStats(name)));
		return *statsMap[name];
	}
	if (name == nullptr && stats == nullptr) {
		stats = dynamic_cast<Statistics*>(
			getSimulation()->getModuleByPath(STATS_MAIN));
		if (stats == nullptr)
			stats = createStats();
	}
	return *stats;
}

void addStatisticSample(StatisticSample* sample, const char* name ) {
	getStatistics(name).addSample(sample);
}

} //namespace
