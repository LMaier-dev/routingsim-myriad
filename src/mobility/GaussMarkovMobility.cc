

#include "GaussMarkovMobility.h"
#include "clog.h"

namespace routingsim {

Define_Module(GaussMarkovMobility);

void GaussMarkovMobility::initialize() {
    MobilityBase::initialize();
    double beta = par("beta").doubleValue();

    alpha = exp(-beta * updateInterval);
    ASSERT(0 <= alpha && alpha <= 1);

    speedMean = par("speedMean");
    speedVariance = par("speedVariance");
    headingMean = par("headingMean");
    headingVariance = par("headingVariance");

    targetPosition = lastPosition;

    EV_DEBUG << "initialize gauss-markov with memory level alpha: " << alpha << endl;
}

void GaussMarkovMobility::move() {
    updateDirection();
    // Update position
    lastPosition = targetPosition;
    targetPosition = lastPosition + velocity;
    // TODO change mean heading when node is inside border margin
}

void GaussMarkovMobility::handleIfOutside() {
    EV_DEBUG << "Node outside: (" << lastPosition.x << "," << lastPosition.y << ")" << endl;
}

void GaussMarkovMobility::updateDirection() {
    // Speed
    double speed = alpha * velocity.distance() + (1 - alpha) * speedMean + speedVariance * sqrt(1 - alpha*alpha) * normal(0, 1, MOBILITY_RNG);
    // Heading
    heading = Rad2D(alpha * heading.getRadius() + (1 - alpha) * headingMean + headingVariance * sqrt(1 - alpha*alpha) * normal(0, 1, MOBILITY_RNG));
    // Velocity
    velocity = heading.toVector() * speed;
}

};
