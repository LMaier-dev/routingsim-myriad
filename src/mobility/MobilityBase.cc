
#include "MobilityBase.h"
#include "checkandcast.h"
#include "coutvector.h"
#include "mobility/MobilityNode.h"
#include <format>

namespace routingsim {

Register_Abstract_Class(MobilityBase);

void MobilityBase::initialize() {
  Environment::initialize();
  updateInterval = par("updateInterval");
  initializePosition();
  initializeHeading();

  nodeid_t nodeId = check_and_cast<MobilityNode*>(getParentModule()->getSubmodule("node"))->getNodeId();
  EV_DEBUG << "MobilityBase of Node: " << nodeId << endl;

  auto prefix= std::format("MobilityNode_{}",nodeId);
  posX = new cOutVector((prefix+"_posX").c_str());
  take(posX);
  posY = new cOutVector((prefix+"_posY").c_str());
  take(posY);
}

void MobilityBase::preDelete(cComponent *root) {
  delete posX;
  delete posY;
}

void MobilityBase::initializePosition() { lastPosition = getRandomPosition(); }

void MobilityBase::initializeHeading() {
  heading.setRadius(getRNG(MOBILITY_RNG)->doubleRand() * 360.0);
}

void MobilityBase::updatePosition() {
  Coord2D prevPosition(lastPosition.x, lastPosition.y);
  move();
  heading.setFromVector(velocity);
  if (isOutside(lastPosition)) {
    handleIfOutside();
  }

  ASSERT(borderMin.x <= lastPosition.x && lastPosition.x <= borderMax.x);
  ASSERT(borderMin.y <= lastPosition.y && lastPosition.y <= borderMax.y);

  EV_DEBUG << "Prev node pos: (" << prevPosition.x << "," << prevPosition.y << ")"
    << " New pos: (" << lastPosition.x << "," << lastPosition.y << ") heading: " << heading.getRadius() << " speed: " << velocity.distance() << "[m/s]"<< endl;

  if (prevPosition != lastPosition) {
    // Only record changes in position
    posX->record(lastPosition.x);
    posY->record(lastPosition.y);
  }
}

void MobilityBase::handleIfOutside() { reflectIfOutside(); }

void MobilityBase::reflectIfOutside() {
  int sign;

  if (lastPosition.x < borderMin.x || borderMax.x < lastPosition.x) {
    sign = reflect(borderMin.x, borderMax.x, lastPosition.x, velocity.x);
    heading.setRadius(90 + (heading.getRadius() - 90) * sign);
    velocity = heading.toVector() * velocity.distance();
  }
  if (lastPosition.y < borderMin.y || borderMax.y < lastPosition.y) {
    sign = reflect(borderMin.y, borderMax.y, lastPosition.y, velocity.y);
    heading.setRadius(heading.getRadius() * sign);
    velocity = heading.toVector() * velocity.distance();
  }
}

double MobilityBase::getUpdateInterval() const { return updateInterval; }

int MobilityBase::reflect(double min, double max, double &coordinate,
                          double &speed) {
  double size = max - min;
  double value = coordinate - min;
  double correction = -fmod(value, size);

  coordinate += 2*correction; // subtract the amount twice (first to reach the border, second to reflect the overlap)
  speed = -speed;
  ASSERT(min <= coordinate && coordinate <= max);
  return -1;
}

} // namespace routingsim
