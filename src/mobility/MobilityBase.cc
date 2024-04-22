
#include "MobilityBase.h"

namespace routingsim {

Register_Abstract_Class(MobilityBase);

    simsignal_t MobilityBase::mobilityStateChangedSignal = cComponent::registerSignal("mobilityStateChanged");

    void MobilityBase::initialize() {
        Environment::initialize();
        updateInterval = par("updateInterval");
        initializePosition();
        initializeHeading();
        scheduleAt(simTime() + updateInterval, nullptr);
    }

    void MobilityBase::initializePosition() {
        lastPosition = getRandomPosition();
    }

    void MobilityBase::initializeHeading() {
        heading.setRadius(getRNG(0)->doubleRand() * 360);
    }

    void MobilityBase::handleMessage(omnetpp::cMessage *msg) {
        cSimpleModule::handleMessage(msg);
        if (msg->isSelfMessage()) {
            handleSelfMessage(msg);
        } else {
            throw new cRuntimeError("Can only handle self messages.");
        }
    }

    void MobilityBase::handleSelfMessage(cMessage *pMessage) {
        updatePosition();
        scheduleAt(simTime() + updateInterval, nullptr);
    }

    void MobilityBase::updatePosition() {
        move();
        if (isOutside(lastPosition)) {
            handleIfOutside();
        }
    }

    void MobilityBase::handleIfOutside() {
        reflectIfOutside();
    }

    void MobilityBase::reflectIfOutside() {
        int sign;

        if (lastPosition.x < borderMin.x || borderMax.x < lastPosition.x) {
            sign = reflect(borderMin.x, borderMax.x, lastPosition.x, velocity.x);
            heading.setRadius(90 + (heading.getRadius() - 90) * sign);
        }
        if (lastPosition.y < borderMin.y || borderMax.y < lastPosition.y) {
            sign = reflect(borderMin.y, borderMax.y, lastPosition.y, velocity.y);
            heading.setRadius(heading.getRadius() * sign);
        }
    }

    double MobilityBase::getUpdateInterval() const {
        return updateInterval;
    }

    int MobilityBase::reflect(double min, double max, double& coordinate, double& speed)
    {
        double size = max - min;
        double value = coordinate - min;
        int sign = 1 - fmod(floor(value / size), 2) * 2;
        ASSERT(sign == 1 || sign == -1);
        coordinate = fmod(sign * value, size) + min;
        speed = sign * speed;
        return sign;
    }

}
