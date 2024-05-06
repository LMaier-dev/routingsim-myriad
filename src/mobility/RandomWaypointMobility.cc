
#include "RandomWaypointMobility.h"

namespace routingsim {

Define_Module(RandomWaypointMobility);

void RandomWaypointMobility::initialize() {
  MobilityBase::initialize();
  waitTime = par("waitTime").doubleValue();
  maxSpeed = par("maxSpeed").doubleValue();
  minSpeed = par("minSpeed").doubleValue();
  setNextTarget();
}

void RandomWaypointMobility::move() {
  simtime_t now = simTime();
  if (now >= nextChange) {
    lastPosition = targetPosition;
    EV_INFO << "reached current target position = " << lastPosition.toString()
            << endl;
    setNextTarget();
    EV_INFO << "new target position = " << targetPosition.toString()
            << ", next change = " << nextChange << endl;
  } else if (now > lastUpdate) {
    ASSERT(nextChange == -1 || now < nextChange);
    lastPosition += velocity * (now - lastUpdate).dbl();
    lastUpdate = now;
  }
}

void RandomWaypointMobility::setNextTarget() {
  setTargetPosition();
  setTargetSpeed();
  setTargetVelocity();
  setTargetNextPeriod();
  waitNextPeriod = !waitNextPeriod;
}

void RandomWaypointMobility::setTargetPosition() {
  if (waitNextPeriod) {
    targetPosition = lastPosition;
  } else {
    targetPosition = Coord2D(uniform(borderMin.x, borderMax.x),
                             uniform(borderMin.y, borderMax.y));
  }
}

void RandomWaypointMobility::setTargetVelocity() {
  if (waitNextPeriod) {
    velocity = Coord2D::ZERO;
    timeToNextPeriod = waitTime;
  } else {
    Coord2D direction = targetPosition - lastPosition;
    double distance = direction.distance_squared();
    EV_DEBUG << "distance " << distance << endl;
    velocity = (direction/distance) * speed;
    timeToNextPeriod = distance / speed;
  }
}

void RandomWaypointMobility::setTargetSpeed() {
  speed = uniform(minSpeed, maxSpeed);
}

void RandomWaypointMobility::setTargetNextPeriod() {
  nextChange = simTime() + timeToNextPeriod;
}

}; // namespace routingsim
