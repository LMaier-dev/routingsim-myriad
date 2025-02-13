
#include "MobilityBase.h"
#include "checkandcast.h"
#include "coutvector.h"
#include "mobility/MobilityNode.h"

namespace routingsim {

Register_Abstract_Class(MobilityBase);

void MobilityBase::initialize() {
  Environment::initialize();
  updateInterval = par("updateInterval");
  initializePosition();
  initializeHeading();

  nodeid_t nodeId = check_and_cast<MobilityNode*>(getParentModule()->getSubmodule("node"))->getNodeId();
  EV_DEBUG << "MobilityBase of Node: " << nodeId << endl;
  posX = new cOutVector("posX");
  take(posX);
  posY = new cOutVector("posY");
  take(posY);
}

void MobilityBase::preDelete(cComponent *root) {
  delete posX;
  delete posY;
}

void MobilityBase::initializePosition() { lastPosition = getRandomPosition(); }

void MobilityBase::initializeHeading() {
  heading.setRadius(getRNG(1)->doubleRand() * 360);
}

void MobilityBase::updatePosition() {
  move();
  heading.setFromVector(velocity);
  if (isOutside(lastPosition)) {
    handleIfOutside();
  }

  posX->record(lastPosition.x);
  posY->record(lastPosition.y);

}

void MobilityBase::handleIfOutside() { reflectIfOutside(); }

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
