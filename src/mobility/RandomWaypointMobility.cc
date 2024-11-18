
#include "RandomWaypointMobility.h"
#include "BaseTypes.h"

namespace routingsim {

Define_Module(RandomWaypointMobility);

void RandomWaypointMobility::initialize() {
  MobilityBase::initialize();
  defaultWaitTime = par("waitTime").doubleValue();
  maxSpeed = par("maxSpeed").doubleValue();
  minSpeed = par("minSpeed").doubleValue();
  setNextTarget();
}

void RandomWaypointMobility::move() {
  simtime_t now = simTime();
  if (now >= nextChange) {
    EV_TRACE << "Should reached current target position = " << targetPosition.toString()
             << " with current position " << lastPosition.toString()
             << " at t=" << simTime()
             << endl;
    lastPosition = targetPosition;
    setNextTarget();
    EV_TRACE << "new target position = " << targetPosition.toString()
            << ", next change t=" << nextChange << endl;
  } else if (now > lastUpdate) {
    ASSERT(nextChange == -1 || now < nextChange);
    lastPosition += velocity * (now - lastUpdate).dbl();

    Coord2D direction = targetPosition - lastPosition;
    if (direction.get_normalized() != heading.toVector()) {
      // Missed target by overshooting
      lastPosition = targetPosition;
      // Calculate the time we still need to wait at the target
      waitTime = defaultWaitTime - (direction.distance() / speed);
    } else {
      waitTime = defaultWaitTime;
    }
    lastUpdate = now;
  }
}

void RandomWaypointMobility::setNextTarget() {
  setTargetPosition();
  setTargetSpeed();
  setTargetVelocity();
  setHeading();
  setTargetNextPeriod();
  waitNextPeriod = !waitNextPeriod;
}

void RandomWaypointMobility::setTargetPosition() {
  if (waitNextPeriod) {
    targetPosition = lastPosition;
  } else {
    targetPosition = Coord2D(uniform(borderMin.x, borderMax.x, 1),
                             uniform(borderMin.y, borderMax.y, 1));
  }
}

void RandomWaypointMobility::setTargetVelocity() {
  if (waitNextPeriod) {
    velocity = Coord2D::ZERO;
    timeToNextPeriod = waitTime;
  } else {
    Coord2D direction = targetPosition - lastPosition;
    double distance = direction.distance();
    velocity = (direction/distance) * speed;
    timeToNextPeriod = distance / speed;
  }
}

void RandomWaypointMobility::setHeading() {
  heading.setFromVector(velocity);
}

void RandomWaypointMobility::setTargetSpeed() {
  speed = uniform(minSpeed, maxSpeed, 1);
}

void RandomWaypointMobility::setTargetNextPeriod() {
  nextChange = simTime() + timeToNextPeriod;
}

double RandomWaypointMobility::getLeaveDuration(Coord2D topLeft, Coord2D bottomRight) {
  if ( topLeft.x < targetPosition.x
    && topLeft.y > targetPosition.y
    && targetPosition.x < bottomRight.x
    && targetPosition.y > bottomRight.y) {
      // Current target is in given rectangle
      return timeToNextPeriod;
    }
  
  if (Coord2D::ZERO == velocity) {
    // We have to redetermine the leave duration after the wait time is over
    return timeToNextPeriod;
  }

  // Calcuate the time when we leave the given rectangle
  // Determine relevant rectangle side
  if (velocity.x == 0) {
    // Top/Bottom intersection
    if (velocity.y > 0) {
      // Top intersection
      return (topLeft.y - lastPosition.y) / velocity.y;
    } else {
      // Bottom intersection
      return (lastPosition.y - bottomRight.y) / velocity.y;
    }
  }

  if (velocity.y == 0) {
    // Left/Right intersection
    if (velocity.x > 0) {
      // Right intersection
      return (bottomRight.x - lastPosition.x) / velocity.x;
    } else {
      // Left intersection
      return (lastPosition.x - bottomRight.x) / velocity.x;
    }
  }

  if (velocity.y > 0) {
    // Top/Left/Right intersection
    double top = (topLeft.y - lastPosition.y) / velocity.y;
    if (velocity.x > 0) {
      // Top/Right intersection
      double right = (bottomRight.x - lastPosition.x) / velocity.x;
      return std::min(top, right);
    } else {
      // Top/Left intersection
      double left = (lastPosition.x - topLeft.x) / velocity.x;
      return std::min(top, left);
    }
  } else {
  // if (velocity.y < 0) {
    // Bottom/Left/Right intersection
    double bottom = (lastPosition.y - bottomRight.y) / velocity.y;
    if (velocity.x > 0) {
      // Bottom/Right intersection
      double right = (bottomRight.x - lastPosition.x) / velocity.x;
      return std::min(bottom, right);
    } else {
      // Bottom/Left intersection
      double left = (lastPosition.x - topLeft.x) / velocity.x;
      return std::min(bottom, left);
    }
  }
}


}; // namespace routingsim
