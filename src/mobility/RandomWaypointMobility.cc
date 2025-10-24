
#include "RandomWaypointMobility.h"
#include "BaseTypes.h"
#include "cenvir.h"

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
    // either has reached/overshoot target position or waiting time is over

    if (!inWaitingState) {
      // we probably moved beyond the target position, so stop at target
      Coord2D direction = targetPosition - lastPosition;
      EV_TRACE << "Should reached current target position = "
               << targetPosition.toString() << " with current position "
               << lastPosition.toString() << " at t=" << now
               << " distance:" << direction.distance() << endl;

      // vectors would opposite directions if overshot target
      if (direction.get_normalized() != heading.toVector()) {
        lastPosition = targetPosition;
        // Calculate the time we still need to wait at the target
        waitTime = std::max(defaultWaitTime - (direction.distance() / speed), 0.0);
      } else {
        waitTime = defaultWaitTime;
      }
    }

    // state transition from waiting to move or other way round
    inWaitingState = !inWaitingState;
    setNextTarget();
    EV_TRACE << "new target position = " << targetPosition.toString()
             << ", next change t=" << nextChange << " inWaitingState=" << (inWaitingState ? 'T' : 'F')  << endl;
    lastUpdate = now;
    return;
  }

  if (now > lastUpdate) {
    ASSERT(nextChange == -1 || now < nextChange);
    EV_TRACE << "Updating position from " << lastPosition.toString();
    lastPosition += velocity * (now - lastUpdate).dbl();
    EV_TRACE << " to " << lastPosition.toString() << " next change: "
             << nextChange << endl;

    EV_TRACE << " targetpos=" << targetPosition.toString() << " inWaitingState=" << (inWaitingState ? 'T' : 'F') <<  endl;
    lastUpdate = now;
  }
}


void RandomWaypointMobility::setNextTarget() {
  setTargetPosition();
  setTargetSpeed();
  setTargetVelocity();
  setHeading();
  setTargetNextPeriod();
}

void RandomWaypointMobility::setTargetPosition() {
  if (not inWaitingState) {
    targetPosition = Coord2D(uniform(borderMin.x, borderMax.x, 1),
                             uniform(borderMin.y, borderMax.y, 1));
  }
}

void RandomWaypointMobility::setTargetVelocity() {
  if (inWaitingState) {
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

double RandomWaypointMobility::getLeaveDuration(Coord2D pos, Coord2D topLeft, Coord2D bottomRight) {
  if (Coord2D::ZERO == velocity) {
    // We have to redetermine the leave duration after the wait time is over
    return timeToNextPeriod;
  }

  // Calculate the time the destination is reached
  double targetDuration = lastPosition.distance(targetPosition) / speed;

  // Calcuate the time when we leave the given rectangle
  // Determine relevant rectangle side
  if (velocity.x == 0) {
    // Top/Bottom intersection
    if (velocity.y > 0) {
      // Top intersection
      return std::min(targetDuration, (topLeft.y - pos.y) / velocity.y);
    } else {
      // Bottom intersection
      return std::min(targetDuration, (pos.y - bottomRight.y) / velocity.y);
    }
  }

  if (velocity.y == 0) {
    // Left/Right intersection
    if (velocity.x > 0) {
      // Right intersection
      return std::min(targetDuration, (bottomRight.x - pos.x) / velocity.x);
    } else {
      // Left intersection
      return std::min(targetDuration, (pos.x - bottomRight.x) / velocity.x);
    }
  }

  if (velocity.y > 0) {
    // Top/Left/Right intersection
    double top = (topLeft.y - pos.y) / velocity.y;
    if (velocity.x > 0) {
      // Top/Right intersection
      double right = (bottomRight.x - pos.x) / velocity.x;
      return std::min(targetDuration, std::min(top, right));
    } else {
      // Top/Left intersection
      double left = (pos.x - topLeft.x) / velocity.x;
      return std::min(targetDuration, std::min(top, left));
    }
  } else {
  // if (velocity.y < 0) {
    // Bottom/Left/Right intersection
    double bottom = (pos.y - bottomRight.y) / velocity.y;
    if (velocity.x > 0) {
      // Bottom/Right intersection
      double right = (bottomRight.x - pos.x) / velocity.x;
      return std::min(targetDuration, std::min(bottom, right));
    } else {
      // Bottom/Left intersection
      double left = (pos.x - topLeft.x) / velocity.x;
      return std::min(targetDuration, std::min(bottom, left));
    }
  }
}


}; // namespace routingsim
