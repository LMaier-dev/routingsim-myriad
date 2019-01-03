#ifndef MESSAGESTATISTICS_H_
#define MESSAGESTATISTICS_H_

#include<omnetpp.h>
#include<string>
#include<boost/unordered_set.hpp>

namespace routingsim {

using namespace boost;
using namespace std;

/**
 * A helper class for measuring message delivery. I.e., delivery ratio,
 * distances and stretch.
 *
 * @author Sebastian Mies <mies@kit.edu>
 */
class MessageStatistics {
private:
	unordered_set<long> msgs;
	cOutVector* deliveryRatio;
	cLongHistogram* distanceRouting;
	cLongHistogram* distanceReality;
	cLongHistogram* stretchAdditive;
	cDoubleHistogram* stretchMultiplicative;
	unsigned int messagesDelivered;
	unsigned int messagesDropped;

public:
	// init recording
	MessageStatistics(const char* prefix) {
		// create names
		string p = string(prefix);
		string deliveryRatioName =
				(prefix != NULL) ?
						(prefix + string("DeliveryRatio")) :
						"deliveryRatio";
		string distanceRoutingName =
				(prefix != NULL) ?
						(prefix + string("DistanceRouting")) :
						"distanceRouting";
		string distanceRealityName =
				(prefix != NULL) ?
						(prefix + string("DistanceReality")) :
						"distanceReality";
		string stretchAdditiveName =
				(prefix != NULL) ?
						(prefix + string("StretchAdditive")) :
						"stretchAdditive";
		string stretchMultiplicativeName =
				(prefix != NULL) ?
						(prefix + string("StretchMultiplicative")) :
						"stretchMultiplicative";

		// create statistics
		deliveryRatio = new cOutVector(deliveryRatioName.c_str());

		distanceRouting = new cLongHistogram(distanceRoutingName.c_str());
		distanceRouting->setRange(0, 128);
		distanceRouting->setNumCells(128);

		distanceReality = new cLongHistogram(distanceRealityName.c_str());
		distanceReality->setRange(0, 128);
		distanceReality->setNumCells(128);

		stretchAdditive = new cLongHistogram(stretchAdditiveName.c_str());
		stretchAdditive->setRange(0, 128);
		stretchAdditive->setNumCells(128);

		stretchMultiplicative = new cDoubleHistogram(
				stretchMultiplicativeName.c_str());
		stretchMultiplicative->setRange(1.0, 64.0);
		stretchMultiplicative->setNumCells(1024);

		// messages delivered / dropped
		messagesDelivered = 0;
		messagesDropped = 0;
	}

	// finish recording
	void finish() {
		distanceRouting->record();
		distanceReality->record();
		stretchAdditive->record();
		stretchMultiplicative->record();
		delete deliveryRatio;
		delete distanceRouting;
		delete distanceReality;
		delete stretchAdditive;
		delete stretchMultiplicative;
	}

	// sample delivery ratio
	void sample() {
		if ((messagesDelivered + messagesDropped) != 0)
			deliveryRatio->record(
					(double) messagesDelivered
							/ ((double) messagesDelivered
									+ (double) messagesDropped));
		else
			deliveryRatio->record(0);
		messagesDelivered = 0;
		messagesDropped = 0;
	}

	// register a message
	void messageRegister(cMessage* msg) {
		msgs.insert(msg->getId());
	}

	// message delivered
	void messageDelivered(cMessage* msg,
			distance_t distRouting, distance_t distReality) {

		// ignore unregistered messages
		if (msgs.count(msg->getId()) == 0) return;

		// record hop count
		distanceRouting->collect(distRouting);
		distanceReality->collect(distReality);

		// increase delivered count and remove message from set
		messagesDelivered++;
		msgs.erase(msg->getId());

		// account stretch
		stretchAdditive->collect(distRouting - distReality);
		if (distReality < 0.5) return;
		double stretch = (double) distRouting / (double) distReality;
		stretchMultiplicative->collect(stretch);
	}

	// message dropped
	void messageDrop(cMessage* msg) {
		if (msgs.count(msg->getId()) == 0) return;
		messagesDropped++;
		msgs.erase(msg->getId());
	}

	bool messageRegistered(cMessage* msg) const {
		return msgs.count(msg->getId()) != 0;
	}
};

}

#endif /* MESSAGESTATISTICS_H_ */
