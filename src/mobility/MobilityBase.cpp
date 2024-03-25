//
// Created by s_schaut on 22.03.24.
//

#include <omnetpp.h>
#include <cmath>
#include "MobilityBase.h"
#include "Environment.h"

namespace routingsim {

Register_Abstract_Class(MobilityBase);

    void MobilityBase::initialize() {
        cComponent::initialize();
        cModule *env = getSimulation()->getModuleByPath("env");
        environment = check_and_cast<Environment *>(env);
        updateInterval = par("updateInterval");
        initializePosition();
        initializeHeading();
        initializeVelocity();
        scheduleAt(simTime() + updateInterval, nullptr);
    }

    void MobilityBase::initializePosition() {
        position = environment->getRandomPosition();
    }

    void MobilityBase::initializeHeading() {
        heading.setRadius(getRNG(0)->doubleRand() * 360);
    }

    void MobilityBase::initializeVelocity() {
        velocity = Coord2D(0, 0);
    }

    void MobilityBase::handleMessage(omnetpp::cMessage *msg) {
        cSimpleModule::handleMessage(msg);
        if (msg->isSelfMessage()) {
            handleSelfMessage(msg);
            scheduleAt(simTime() + updateInterval, nullptr);
        }
    }

    void MobilityBase::handleSelfMessage(cMessage *pMessage) {
        updatePosition();
    }

    void MobilityBase::updatePosition() {
        position = calculateNewPosition();
        if (environment->isOutside(position)) {
            handleIfOutside();
        }
    }

    void MobilityBase::handleIfOutside() {
        reflectIfOutside();
    }

    void MobilityBase::reflectIfOutside() {
        int sign;
        const Coord2D borderMin = environment->getBorderMin();
        const Coord2D borderMax = environment->getBorderMax();

        if (position.x < borderMin.x || borderMax.x < position.x) {
            sign = reflect(borderMin.x, borderMax.x, position.x, velocity.x);
            heading.setRadius(90 + (heading.getRadius() - 90) * sign);
        }
        if (position.y < borderMin.y || borderMax.y < position.y) {
            sign = reflect(borderMin.y, borderMax.y, position.y, velocity.y);
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
